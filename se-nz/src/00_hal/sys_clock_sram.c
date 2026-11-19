#include "../../include/hc32f460_regs.h"
#include "../../include/boryoza_math_mem.h"

#if !defined(__STDC_HOSTED__) || (__STDC_HOSTED__ == 0)
BORYOZA_RAMFUNC void *memcpy(void *dst, const void *src, size_t len) {
    uint8_t *out = (uint8_t *)dst;
    const uint8_t *inp = (const uint8_t *)src;
    if ((len >= 16U) && ((((uintptr_t)out | (uintptr_t)inp) & 3U) == 0U)) {
        uint32_t *out32 = (uint32_t *)(void *)out;
        const uint32_t *inp32 = (const uint32_t *)(const void *)inp;
        size_t words = len >> 2U;
        size_t idx = 0U;
        while ((idx + 4U) <= words) {
            out32[idx]      = inp32[idx];
            out32[idx + 1U] = inp32[idx + 1U];
            out32[idx + 2U] = inp32[idx + 2U];
            out32[idx + 3U] = inp32[idx + 3U];
            idx += 4U;
        }
        while (idx < words) {
            out32[idx] = inp32[idx];
            idx += 1U;
        }
        size_t copied = words << 2U;
        out += copied;
        inp += copied;
        len -= copied;
    }
    for (size_t idx = 0U; idx < len; ++idx) {
        out[idx] = inp[idx];
    }
    return dst;
}

BORYOZA_RAMFUNC void *memset(void *dst, int byte_val, size_t len) {
    uint8_t *out = (uint8_t *)dst;
    uint8_t fill = (uint8_t)((uint32_t)byte_val & 0xFFUL);
    while ((len > 0U) && (((uintptr_t)out & 3U) != 0U)) {
        *out = fill;
        out += 1;
        len -= 1U;
    }
    if (len >= 16U) {
        uint32_t fill32 = (uint32_t)fill * 0x01010101UL;
        uint32_t *out32 = (uint32_t *)(void *)out;
        size_t words = len >> 2U;
        size_t idx = 0U;
        while ((idx + 4U) <= words) {
            out32[idx]      = fill32;
            out32[idx + 1U] = fill32;
            out32[idx + 2U] = fill32;
            out32[idx + 3U] = fill32;
            idx += 4U;
        }
        while (idx < words) {
            out32[idx] = fill32;
            idx += 1U;
        }
        size_t set_bytes = words << 2U;
        out += set_bytes;
        len -= set_bytes;
    }
    for (size_t idx = 0U; idx < len; ++idx) {
        out[idx] = fill;
    }
    return dst;
}

BORYOZA_RAMFUNC size_t strlen(const char *str) {
    size_t len = 0U;
    while (str[len] != '\0') {
        len += 1U;
    }
    return len;
}
#endif

static const uint32_t MRC_UHS_SETTLE_LOOPS  = 1200UL; /* >=100us @ 8MHz MRC */
static const uint32_t MRC_STOP_SETTLE_LOOPS = 64UL;
static const uint32_t HCLK_SW_SETTLE_LOOPS  = 1920UL;
static const uint32_t OSC_POLL_LIMIT        = 200000UL;
static const uint8_t  CKSWR_SRC_MASK        = 0x07U;
static const uint8_t  XTALSTB_DEFAULT_1MS   = 0x05U;  /* Hardware reset default ~1ms */
static const uint32_t SRAM23_END_ADDR       = 0x20027000UL;
static const uint8_t  MRC_ENABLE_VALUE     = 0x80U;

volatile uint32_t g_boot_reset_flags;
volatile uint32_t g_boot_debug_state;
volatile uint32_t g_boot_sram_status;
volatile uint32_t g_boot_sram_check;
volatile uint32_t g_boot_nmi_enable;
volatile uint32_t g_nmi_snapshot[NMI_SNAPSHOT_WORDS];
volatile uint32_t g_clock_init_stage;
volatile uint32_t g_clock_osc_status;

static void delay_mrc_cycles(uint32_t loops) {
    for (uint32_t idx = 0U; idx < loops; ++idx) {
        cpu_nop();
    }
}

enum {
    BOOT_SNAP_RSTF0_IDX  = 0,
    BOOT_SNAP_DBGC_IDX   = 1,
    BOOT_SNAP_CKSR_IDX   = 2,
    BOOT_SNAP_CKCR_IDX   = 3,
    BOOT_SNAP_NMI_IDX    = 4,
    BOOT_SNAP_STAGE_IDX  = 5,
    BOOT_SNAP_OSC_IDX    = 6,
    BOOT_SNAP_OK_IDX     = 7,
    CLK_STAGE_DONE       = 4U,
};

static const uint32_t CLK_STAGE_MRC_STOP_U = 1UL;
static const uint32_t CLK_STAGE_XTAL_U     = 2UL;
static const uint32_t CLK_STAGE_MPLL_U     = 3UL;

/* NOLINTBEGIN(bugprone-easily-swappable-parameters) */
static bool wait_oscillator_raw(uint8_t mask, bool stable, uint32_t stage,
                                uint32_t *out_stage, uint32_t *out_status) {
/* NOLINTEND(bugprone-easily-swappable-parameters) */
    *out_stage = stage;
    for (uint32_t idx = 0U; idx < OSC_POLL_LIMIT; ++idx) {
        uint8_t status = mmio_r8(CMU_OSCSTBSR_ADDR);
        *out_status = status;
        uint8_t expected = stable ? mask : 0U;
        if ((status & mask) == expected) {
            return true;
        }
    }
    mmio_w16(PWC_FPRC_ADDR, PWC_FPRC_LOCK);
    return false;
}

static bool configure_clocks_192mhz_core(uint32_t boot_snap[BOOT_SNAP_WORDS]) {
    static const uint32_t NMI_ENABLE_ADDR = 0x40051004UL;
    boot_snap[BOOT_SNAP_RSTF0_IDX] = mmio_r16(RMU_RSTF0_ADDR);
    boot_snap[BOOT_SNAP_DBGC_IDX] = mmio_r32(DBGC_MCUSTPCTL_ADDR);
    boot_snap[BOOT_SNAP_CKSR_IDX] = mmio_r32(SRAMC_CKSR_ADDR);
    boot_snap[BOOT_SNAP_CKCR_IDX] = mmio_r32(SRAMC_CKCR_ADDR);
    boot_snap[BOOT_SNAP_NMI_IDX] = mmio_r32(NMI_ENABLE_ADDR);
    boot_snap[BOOT_SNAP_STAGE_IDX] = 0UL;
    boot_snap[BOOT_SNAP_OSC_IDX] = 0UL;
    boot_snap[BOOT_SNAP_OK_IDX] = 1UL;

    /* Step 1: Unlock SRAMC protection -> set 1-wait-state WTCR, CKCR=0, clear CKSR */
    mmio_w32(SRAMC_WTPR_ADDR, SRAMC_WTPR_UNLOCK);
    mmio_w32(SRAMC_CKPR_ADDR, SRAMC_WTPR_UNLOCK);
    mmio_w32(SRAMC_WTCR_ADDR, SRAMC_WTCR_192MHZ);
    (void)mmio_r32(SRAMC_WTCR_ADDR);
    mmio_w32(SRAMC_CKCR_ADDR, 0x00000000UL);
    mmio_w32(SRAMC_CKSR_ADDR, SRAMC_CKSR_ALL_CLR);
    mmio_w32(SRAMC_CKPR_ADDR, SRAMC_WTPR_LOCK);
    mmio_w32(SRAMC_WTPR_ADDR, SRAMC_WTPR_LOCK);

    /* Configure GPIO_PCCR.RDWT = 3 read wait cycles BEFORE switching HCLK to 192 MHz */
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_UNLOCK);
    uint32_t pccr = mmio_r16(GPIO_PCCR_ADDR);
    uint32_t next_pccr = (pccr & ~((uint32_t)GPIO_PCCR_RDWT_MASK)) | (uint32_t)GPIO_PCCR_RDWT_3CYC;
    mmio_w16(GPIO_PCCR_ADDR, (uint16_t)next_pccr);
    mmio_w16(GPIO_PWPR_ADDR, GPIO_PWPR_LOCK);

    if (mmio_r16(PWC_RAMOPM_ADDR) != PWC_RAMOPM_UHS_EXPECTED) {
        mmio_w16(PWC_FPRC_ADDR, PWC_FPRC_UNLOCK_PWC);
        mmio_w16(PWC_RAMOPM_ADDR, PWC_RAMOPM_UHS_EXPECTED);
        mmio_w16(PWC_FPRC_ADDR, PWC_FPRC_LOCK);
    }

    /* Step 2: Unlock EFM_FAPRT -> wake EFM_FSTP=0 -> set 5-wait-state EFM_FRMC -> relock */
    if ((mmio_r32(EFM_FAPRT_ADDR) & 1UL) == 0UL) {
        mmio_w32(EFM_FAPRT_ADDR, EFM_KEY1);
        mmio_w32(EFM_FAPRT_ADDR, EFM_KEY2);
    }
    mmio_w32(EFM_FSTP_ADDR, 0x00000000UL);
    mmio_w32(EFM_FRMC_ADDR, EFM_FRMC_192MHZ_NOCACHE | EFM_FRMC_CRST_BIT);
    (void)mmio_r32(EFM_FRMC_ADDR);
    mmio_w32(EFM_FRMC_ADDR, EFM_FRMC_192MHZ_CACHE);
    (void)mmio_r32(EFM_FRMC_ADDR);
    mmio_w32(EFM_FAPRT_ADDR, EFM_LOCK_VAL);
    mmio_w32(EFM_FAPRT_ADDR, EFM_LOCK_VAL);

    /* Step 3: Gate all peripheral clocks (including bootloader AES) before SCFGR/CKSWR switch */
    mmio_w32(PWC_FCG0PC_ADDR, PWC_FCG0PC_UNLOCK);
    mmio_w32(PWC_FCG0_ADDR, PWC_FCG0_PERIPH_GATED);
    mmio_w32(PWC_FCG1_ADDR, PWC_FCG_ALL_GATED);
    mmio_w32(PWC_FCG2_ADDR, PWC_FCG_ALL_GATED);
    mmio_w32(PWC_FCG3_ADDR, PWC_FCG_ALL_GATED);
    mmio_w32(PWC_FCG0PC_ADDR, PWC_FCG0PC_LOCK);
    delay_mrc_cycles(MRC_STOP_SETTLE_LOOPS);

    /* Step 4: Enter Ultra-High-Speed (UHS) power mode required for >168 MHz */
    mmio_w16(PWC_FPRC_ADDR, PWC_FPRC_UNLOCK_PWC);
    mmio_w8(PWC_PWRC2_ADDR, PWC_PWRC2_UHS);
    mmio_w8(PWC_MDSWCR_ADDR, PWC_MDSWCR_UHS_TRIG);
    delay_mrc_cycles(MRC_UHS_SETTLE_LOOPS);
    mmio_w16(PWC_FPRC_ADDR, PWC_FPRC_LOCK);

    /* Step 5: Configure bus dividers (HCLK=192, PCLK1/4=96, PCLK2/3=48 MHz) & start 16MHz XTAL */
    mmio_w16(PWC_FPRC_ADDR, PWC_FPRC_UNLOCK_CMU);
    mmio_w8(CMU_MRCCR_ADDR, MRC_ENABLE_VALUE);
    delay_mrc_cycles(MRC_STOP_SETTLE_LOOPS);
    if ((mmio_r8(CMU_CKSWR_ADDR) & CKSWR_SRC_MASK) != CMU_CKSWR_MRC) {
        mmio_w8(CMU_CKSWR_ADDR, CMU_CKSWR_MRC);
        delay_mrc_cycles(MRC_STOP_SETTLE_LOOPS);
    }
    if ((mmio_r8(CMU_PLLCR_ADDR) & 1U) == 0U) {
        mmio_w8(CMU_PLLCR_ADDR, 1U);
        if (!wait_oscillator_raw(CMU_OSCSTBSR_MPLLSTBF, false, CLK_STAGE_MRC_STOP_U,
                                 &boot_snap[BOOT_SNAP_STAGE_IDX], &boot_snap[BOOT_SNAP_OSC_IDX])) {
            return false;
        }
    }

    mmio_w32(CMU_SCFGR_ADDR, CMU_SCFGR_192_96_48);
    delay_mrc_cycles(MRC_STOP_SETTLE_LOOPS);
    /* Stop XTAL before writing XTALSTBCR/XTALCFGR */
    if ((mmio_r8(CMU_XTALCR_ADDR) & 1U) == 0U) {
        mmio_w8(CMU_XTALCR_ADDR, 1U);
        delay_mrc_cycles(MRC_STOP_SETTLE_LOOPS);
    }
    mmio_w8(CMU_XTALSTBCR_ADDR, XTALSTB_DEFAULT_1MS);
    mmio_w8(CMU_XTALCFGR_ADDR, CMU_XTALCFGR_16MHZ_OSC);
    mmio_w8(CMU_XTALCR_ADDR, 0x00U);
    if (!wait_oscillator_raw(CMU_OSCSTBSR_XTALSTBF, true, CLK_STAGE_XTAL_U,
                             &boot_snap[BOOT_SNAP_STAGE_IDX], &boot_snap[BOOT_SNAP_OSC_IDX])) {
        return false;
    }

    /* Step 6: Configure MPLL (16MHz / 2 * 48 / 2 = 192.000 MHz), wait lock, switch CKSWR=5 */
    mmio_w32(CMU_PLLCFGR_ADDR, CMU_PLLCFGR_16M_TO_192M);
    mmio_w8(CMU_PLLCR_ADDR, 0x00U);
    if (!wait_oscillator_raw(CMU_OSCSTBSR_MPLLSTBF, true, CLK_STAGE_MPLL_U,
                             &boot_snap[BOOT_SNAP_STAGE_IDX], &boot_snap[BOOT_SNAP_OSC_IDX])) {
        return false;
    }
    delay_mrc_cycles(MRC_STOP_SETTLE_LOOPS);
    mmio_w8(CMU_CKSWR_ADDR, CMU_CKSWR_MPLL);
    mmio_w16(PWC_FPRC_ADDR, PWC_FPRC_LOCK);
    delay_mrc_cycles(HCLK_SW_SETTLE_LOOPS);

    /* Step 7: Open peripheral clock gates: FCG0, FCG1, FCG2 */
    mmio_w32(PWC_FCG0PC_ADDR, PWC_FCG0PC_UNLOCK);
    uint32_t fcg0 = PWC_FCG0_PERIPH_GATED;
    fcg0 &= ~(PWC_FCG0_SRAMH_BIT | PWC_FCG0_SRAM12_BIT | PWC_FCG0_SRAM3_BIT |
              PWC_FCG0_SRAMRET_BIT | PWC_FCG0_DMA1_BIT | PWC_FCG0_DMA2_BIT | PWC_FCG0_AOS_BIT);
    mmio_w32(PWC_FCG0_ADDR, fcg0);

    uint32_t fcg1 = PWC_FCG_ALL_GATED;
    fcg1 &= ~(PWC_FCG1_SPI1_BIT | PWC_FCG1_SPI3_BIT | PWC_FCG1_USART1_BIT);
    mmio_w32(PWC_FCG1_ADDR, fcg1);

    uint32_t fcg2 = PWC_FCG_ALL_GATED;
    fcg2 &= ~(PWC_FCG2_TMRA1_BIT | PWC_FCG2_TMRA4_BIT);
    mmio_w32(PWC_FCG2_ADDR, fcg2);
    mmio_w32(PWC_FCG0PC_ADDR, PWC_FCG0PC_LOCK);
    return true;
}

void sys_clock_early_192mhz(uint32_t boot_snap[BOOT_SNAP_WORDS]) {
    if (!configure_clocks_192mhz_core(boot_snap)) {
        boot_snap[BOOT_SNAP_OK_IDX] = 0UL;
    }
}

bool sys_clock_finish_boot_state(const uint32_t boot_snap[BOOT_SNAP_WORDS]) {
    g_boot_reset_flags = boot_snap[BOOT_SNAP_RSTF0_IDX];
    g_boot_debug_state = boot_snap[BOOT_SNAP_DBGC_IDX];
    g_boot_sram_status = boot_snap[BOOT_SNAP_CKSR_IDX];
    g_boot_sram_check = boot_snap[BOOT_SNAP_CKCR_IDX];
    g_boot_nmi_enable = boot_snap[BOOT_SNAP_NMI_IDX];
    g_clock_init_stage = boot_snap[BOOT_SNAP_STAGE_IDX];
    g_clock_osc_status = boot_snap[BOOT_SNAP_OSC_IDX];
    if (boot_snap[BOOT_SNAP_OK_IDX] == 0UL) {
        return false;
    }

    /* Initialize retained-memory parity via 32-bit word writes on cold power-on resets. */
    if (((g_boot_reset_flags & RMU_RSTF0_PORF_BIT) != 0UL) ||
        (((g_boot_debug_state & DBGC_COLD_SEEN_BIT) == 0UL) &&
         ((g_boot_reset_flags & RMU_RSTF0_PDRF_BIT) == 0UL))) {
        for (uint32_t addr = HC32_RETRAM_BASE; addr < HC32_RETRAM_END; addr += REG_WORD_STRIDE) {
            mmio_w32(addr, 0x00000000UL);
        }
        mmio_w32(DBGC_MCUSTPCTL_ADDR, g_boot_debug_state | DBGC_COLD_SEEN_BIT);
    }
    mmio_w16(PWC_FPRC_ADDR, PWC_FPRC_UNLOCK_PWC);
    mmio_w16(RMU_RSTF0_ADDR, RMU_RSTF0_CLRF_BIT);
    delay_mrc_cycles(MRC_STOP_SETTLE_LOOPS);
    mmio_w16(PWC_FPRC_ADDR, PWC_FPRC_LOCK);
    mmio_w32(SRAMC_CKPR_ADDR, SRAMC_WTPR_UNLOCK);
    mmio_w32(SRAMC_CKSR_ADDR, SRAMC_CKSR_ALL_CLR);
    mmio_w32(SRAMC_CKPR_ADDR, SRAMC_WTPR_LOCK);

    /* Pre-bind all 4 verified INTC event slots deterministically at cold boot */
    mmio_w32(INTC_SEL40_ADDR, INT_SRC_DMA2_TC0);
    mmio_w32(INTC_SEL41_ADDR, INT_SRC_DMA2_TC3);
    mmio_w32(INTC_SEL80_ADDR, INT_SRC_TMRA4_OVF);
    mmio_w32(INTC_SEL82_ADDR, INT_SRC_TMRA1_OVF);
    cpu_dsb_isb();
    g_clock_init_stage = (uint32_t)CLK_STAGE_DONE;
    return true;
}

bool sys_clock_sram_init_192mhz(void) {
    if (g_clock_init_stage == (uint32_t)CLK_STAGE_DONE) {
        return true;
    }
    uint32_t boot_snap[BOOT_SNAP_WORDS] = {0};
    if (!configure_clocks_192mhz_core(boot_snap)) {
        g_boot_reset_flags = boot_snap[BOOT_SNAP_RSTF0_IDX];
        g_boot_debug_state = boot_snap[BOOT_SNAP_DBGC_IDX];
        g_boot_sram_status = boot_snap[BOOT_SNAP_CKSR_IDX];
        g_boot_sram_check = boot_snap[BOOT_SNAP_CKCR_IDX];
        g_boot_nmi_enable = boot_snap[BOOT_SNAP_NMI_IDX];
        g_clock_init_stage = boot_snap[BOOT_SNAP_STAGE_IDX];
        g_clock_osc_status = boot_snap[BOOT_SNAP_OSC_IDX];
        return false;
    }
    for (uint32_t addr = HC32_SRAM2_BASE; addr < SRAM23_END_ADDR; addr += REG_WORD_STRIDE) {
        mmio_w32(addr, 0x00000000UL);
    }
    return sys_clock_finish_boot_state(boot_snap);
}
