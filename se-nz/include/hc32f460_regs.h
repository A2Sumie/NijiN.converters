#ifndef BORYOZA_HC32F460_REGS_H
#define BORYOZA_HC32F460_REGS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) && defined(__ELF__) && defined(__ARM_ARCH) && (__ARM_ARCH == 7)
#define BORYOZA_RAMFUNC __attribute__((section(".ramfunc"), noinline, long_call))
#define BORYOZA_DMA_BUF __attribute__((section(".dma_sramh"), aligned(4)))
#define BORYOZA_SRAM2_BUF __attribute__((section(".optics_sram2"), aligned(4)))
#define BORYOZA_SRAM3_BUF __attribute__((section(".diag_sram3"), aligned(4)))
#define BORYOZA_RETRAM_BUF __attribute__((section(".retram_noload"), aligned(4)))
#else
#define BORYOZA_RAMFUNC
#define BORYOZA_DMA_BUF __attribute__((aligned(4)))
#define BORYOZA_SRAM2_BUF __attribute__((aligned(4)))
#define BORYOZA_SRAM3_BUF __attribute__((aligned(4)))
#define BORYOZA_RETRAM_BUF __attribute__((aligned(4)))
#endif

#define BORYOZA_FORCE_INLINE static inline __attribute__((always_inline))

BORYOZA_FORCE_INLINE void mmio_w8(uint32_t addr, uint8_t val) {
    *(volatile uint8_t *)(uintptr_t)addr = val; // NOLINT(performance-no-int-to-ptr)
}

BORYOZA_FORCE_INLINE uint8_t mmio_r8(uint32_t addr) {
    return *(volatile const uint8_t *)(uintptr_t)addr; // NOLINT(performance-no-int-to-ptr)
}

BORYOZA_FORCE_INLINE void mmio_w16(uint32_t addr, uint16_t val) {
    *(volatile uint16_t *)(uintptr_t)addr = val; // NOLINT(performance-no-int-to-ptr)
}

BORYOZA_FORCE_INLINE uint16_t mmio_r16(uint32_t addr) {
    return *(volatile const uint16_t *)(uintptr_t)addr; // NOLINT(performance-no-int-to-ptr)
}

BORYOZA_FORCE_INLINE void mmio_w32(uint32_t addr, uint32_t val) {
    *(volatile uint32_t *)(uintptr_t)addr = val; // NOLINT(performance-no-int-to-ptr)
}

BORYOZA_FORCE_INLINE uint32_t mmio_r32(uint32_t addr) {
    return *(volatile const uint32_t *)(uintptr_t)addr; // NOLINT(performance-no-int-to-ptr)
}

BORYOZA_FORCE_INLINE void cpu_dsb_isb(void) {
#if defined(__ARM_ARCH) && (__ARM_ARCH == 7)
    __asm__ volatile("dsb sy\n\tisb sy" ::: "memory");
#endif
}

BORYOZA_FORCE_INLINE void cpu_dmb(void) {
#if defined(__ARM_ARCH) && (__ARM_ARCH == 7)
    __asm__ volatile("dmb sy" ::: "memory");
#endif
}

BORYOZA_FORCE_INLINE void cpu_nop(void) {
#if defined(__ARM_ARCH) && (__ARM_ARCH == 7)
    __asm__ volatile("nop");
#endif
}

BORYOZA_FORCE_INLINE uint32_t cpu_irq_save(void) {
#if defined(__ARM_ARCH) && (__ARM_ARCH == 7)
    uint32_t primask = 0U;
    __asm__ volatile("mrs %0, primask\n\tcpsid i" : "=r"(primask) :: "memory");
    return primask;
#else
    return 0U;
#endif
}

BORYOZA_FORCE_INLINE void cpu_irq_restore(uint32_t primask) {
#if defined(__ARM_ARCH) && (__ARM_ARCH == 7)
    __asm__ volatile("msr primask, %0" :: "r"(primask) : "memory");
#else
    (void)primask;
#endif
}

BORYOZA_FORCE_INLINE void cpu_irq_enable(void) {
#if defined(__ARM_ARCH) && (__ARM_ARCH == 7)
    __asm__ volatile("cpsie i" ::: "memory");
#endif
}

/* =========================================================================
 * Physical Flash & 5-Bank SRAM Memory Map
 * ========================================================================= */
static const uint32_t HC32_FLASH_APP_BASE          = 0x00010000UL;
static const uint32_t HC32_FLASH_APP_END           = 0x00060000UL;
static const uint32_t HC32_FLASH_JOURNAL_PONG_BASE = 0x00078000UL;
static const uint32_t HC32_FLASH_BLACKBOX_BASE     = 0x0007A000UL;
static const uint32_t HC32_FLASH_BLACKBOX_BYTES    = 0x00002000UL;
static const uint32_t HC32_FLASH_JOURNAL_A_BASE    = 0x0007C000UL;
static const uint32_t HC32_FLASH_JOURNAL_B_BASE    = 0x0007E000UL;
static const uint32_t HC32_FLASH_SECTOR_BYTES      = 0x00002000UL;

static const uint32_t HC32_SRAMH_BASE           = 0x1FFF8000UL;
static const uint32_t HC32_SRAMH_VTOR_BASE      = 0x1FFF8000UL;
static const uint32_t HC32_SRAMH_MSP_INIT       = 0x1FFFFF00UL;
static const uint32_t HC32_SRAM1_BASE           = 0x20000000UL;
static const uint32_t HC32_SRAM2_BASE           = 0x20010000UL;
static const uint32_t HC32_SRAM3_BASE           = 0x20020000UL;
static const uint32_t HC32_RETRAM_BASE          = 0x200F0000UL;
static const uint32_t HC32_RETRAM_END           = 0x200F1000UL;

/* =========================================================================
 * System Frequencies
 * ========================================================================= */
static const uint32_t HC32_XTAL_HZ              = 16000000UL;
static const uint32_t HC32_HCLK_HZ              = 192000000UL;
static const uint32_t HC32_PCLK1_HZ             = 96000000UL;
static const uint32_t HC32_PCLK2_HZ             = 48000000UL;
static const uint32_t HC32_TMRA_TICK_HZ         = 3000000UL;
static const uint32_t HC32_TICKS_PER_US         = 3UL;
static const uint32_t HC32_TICKS_PER_MS         = 3000UL;

/* =========================================================================
 * Processor Core Peripherals (SCB / NVIC / FPU)
 * ========================================================================= */
static const uint32_t SCB_ICSR_ADDR             = 0xE000ED04UL;
static const uint32_t SCB_VTOR_ADDR             = 0xE000ED08UL;
static const uint32_t SCB_AIRCR_ADDR            = 0xE000ED0CUL;
static const uint32_t SCB_CFSR_ADDR             = 0xE000ED28UL;
static const uint32_t SCB_HFSR_ADDR             = 0xE000ED2CUL;
static const uint32_t SCB_MMFAR_ADDR            = 0xE000ED34UL;
static const uint32_t SCB_BFAR_ADDR             = 0xE000ED38UL;
static const uint32_t SCB_CPACR_ADDR            = 0xE000ED88UL;
static const uint32_t FPU_FPCCR_ADDR            = 0xE000EF34UL;

static const uint32_t NVIC_ISER_BASE_ADDR       = 0xE000E100UL;
static const uint32_t NVIC_ICER_BASE_ADDR       = 0xE000E180UL;
static const uint32_t NVIC_ICPR_BASE_ADDR       = 0xE000E280UL;
static const uint32_t NVIC_IPR_BASE_ADDR        = 0xE000E400UL;
static const uint32_t REG_WORD_STRIDE           = 4UL;

BORYOZA_FORCE_INLINE uint32_t nvic_iser_addr(uint32_t reg_idx) {
    return NVIC_ISER_BASE_ADDR + (reg_idx * REG_WORD_STRIDE);
}
BORYOZA_FORCE_INLINE uint32_t nvic_icer_addr(uint32_t reg_idx) {
    return NVIC_ICER_BASE_ADDR + (reg_idx * REG_WORD_STRIDE);
}
BORYOZA_FORCE_INLINE uint32_t nvic_icpr_addr(uint32_t reg_idx) {
    return NVIC_ICPR_BASE_ADDR + (reg_idx * REG_WORD_STRIDE);
}
BORYOZA_FORCE_INLINE uint32_t nvic_ipr_addr(uint32_t irq_num) {
    return NVIC_IPR_BASE_ADDR + irq_num;
}

/* =========================================================================
 * SRAMC Wait-State & Parity Controller
 * ========================================================================= */
static const uint32_t SRAMC_WTCR_ADDR           = 0x40050800UL;
static const uint32_t SRAMC_WTPR_ADDR           = 0x40050804UL;
static const uint32_t SRAMC_CKCR_ADDR           = 0x40050808UL;
static const uint32_t SRAMC_CKPR_ADDR           = 0x4005080CUL;
static const uint32_t SRAMC_CKSR_ADDR           = 0x40050810UL;

static const uint32_t SRAMC_WTPR_UNLOCK         = 0x00000077UL;
static const uint32_t SRAMC_WTPR_LOCK           = 0x00000076UL;
static const uint32_t SRAMC_WTCR_192MHZ         = 0x11111111UL;
static const uint32_t SRAMC_CKSR_ALL_CLR        = 0x0000001FUL;

/* =========================================================================
 * EFM Embedded Flash Controller
 * ========================================================================= */
static const uint32_t EFM_FAPRT_ADDR            = 0x40010400UL;
static const uint32_t EFM_FSTP_ADDR             = 0x40010404UL;
static const uint32_t EFM_FRMC_ADDR             = 0x40010408UL;
static const uint32_t EFM_FWMC_ADDR             = 0x4001040CUL;
static const uint32_t EFM_FSR_ADDR              = 0x40010410UL;
static const uint32_t EFM_FSCLR_ADDR            = 0x40010414UL;
static const uint32_t EFM_FPMTSW_ADDR           = 0x40010420UL;
static const uint32_t EFM_FPMTEW_ADDR           = 0x40010424UL;
static const uint32_t EFM_UQID0_ADDR            = 0x40010450UL;

static const uint32_t EFM_KEY1                  = 0x00000123UL;
static const uint32_t EFM_KEY2                  = 0x00003210UL;
static const uint32_t EFM_LOCK_VAL              = 0x00003210UL;
static const uint32_t EFM_FRMC_192MHZ_NOCACHE   = 0x00000050UL;
static const uint32_t EFM_FRMC_192MHZ_CACHE     = 0x00010050UL;
static const uint32_t EFM_FRMC_CACHE_BIT        = 0x00010000UL;
static const uint32_t EFM_FRMC_CRST_BIT         = 0x01000000UL;
static const uint32_t EFM_FRMC_192MHZ_CACHE_RST = 0x01010050UL;
static const uint32_t EFM_FRMC_SLPMD_BIT        = 0x00000001UL;
static const uint32_t EFM_FWMC_PEMODE_BIT       = 0x00000001UL;
static const uint32_t EFM_FWMC_SINGLE_PROG      = 0x00000011UL;
static const uint32_t EFM_FWMC_SECTOR_ERASE     = 0x00000041UL;
static const uint32_t EFM_FSR_OPTEND_BIT        = 0x00000010UL;
static const uint32_t EFM_FSR_RDY_BIT           = 0x00000100UL;
static const uint32_t EFM_FSR_ERR_MASK          = 0x0000002FUL;
static const uint32_t EFM_FSCLR_ALL             = 0x0000003FUL;

/* =========================================================================
 * PWC & CMU Clock / Power Controller
 * ========================================================================= */
static const uint32_t PWC_FCG0_ADDR             = 0x40048000UL;
static const uint32_t PWC_FCG1_ADDR             = 0x40048004UL;
static const uint32_t PWC_FCG2_ADDR             = 0x40048008UL;
static const uint32_t PWC_FCG3_ADDR             = 0x4004800CUL;
static const uint32_t PWC_FCG0PC_ADDR           = 0x40048010UL;

static const uint32_t PWC_FCG0PC_UNLOCK         = 0xA5A50001UL;
static const uint32_t PWC_FCG0PC_LOCK           = 0xA5A50000UL;

static const uint32_t PWC_FCG0_SRAMH_BIT        = (1UL << 0U);
static const uint32_t PWC_FCG0_SRAM12_BIT       = (1UL << 4U);
static const uint32_t PWC_FCG0_SRAM3_BIT        = (1UL << 8U);
static const uint32_t PWC_FCG0_SRAMRET_BIT      = (1UL << 10U);
static const uint32_t PWC_FCG0_DMA1_BIT         = (1UL << 14U);
static const uint32_t PWC_FCG0_DMA2_BIT         = (1UL << 15U);
static const uint32_t PWC_FCG0_AOS_BIT          = (1UL << 17U);
static const uint32_t PWC_FCG0_PERIPH_GATED     = 0xFFFFFAEEUL;
static const uint32_t PWC_FCG_ALL_GATED         = 0xFFFFFFFFUL;

static const uint32_t PWC_FCG1_SPI1_BIT         = (1UL << 16U);
static const uint32_t PWC_FCG1_SPI3_BIT         = (1UL << 18U);
static const uint32_t PWC_FCG1_USART1_BIT       = (1UL << 24U);

static const uint32_t PWC_FCG2_TMRA1_BIT        = (1UL << 2U);
static const uint32_t PWC_FCG2_TMRA4_BIT        = (1UL << 5U);

static const uint32_t PWC_RAMPC0_ADDR           = 0x40054014UL;
static const uint32_t PWC_RAMOPM_ADDR           = 0x40054018UL;
static const uint32_t CMU_SCFGR_ADDR            = 0x40054020UL;
static const uint32_t CMU_CKSWR_ADDR            = 0x40054026UL;
static const uint32_t CMU_PLLCR_ADDR            = 0x4005402AUL;
static const uint32_t CMU_XTALCR_ADDR           = 0x40054032UL;
static const uint32_t CMU_MRCCR_ADDR            = 0x40054038UL;
static const uint32_t CMU_OSCSTBSR_ADDR         = 0x4005403CUL;
static const uint32_t CMU_XTALSTBCR_ADDR        = 0x400540A2UL;
static const uint32_t RMU_RSTF0_ADDR            = 0x400540C0UL;
static const uint32_t DBGC_MCUSTPCTL_ADDR       = 0xE0042020UL;
static const uint32_t DBGC_COLD_SEEN_BIT        = 0x00010000UL;
static const uint32_t CMU_PLLCFGR_ADDR          = 0x40054100UL;
static const uint32_t PWC_FPRC_ADDR             = 0x400543FEUL;
static const uint32_t PWC_PWRC2_ADDR            = 0x40054402UL;
static const uint32_t PWC_MDSWCR_ADDR           = 0x4005440FUL;
static const uint32_t CMU_XTALCFGR_ADDR         = 0x40054410UL;

static const uint16_t PWC_FPRC_LOCK             = 0xA500U;
static const uint16_t PWC_FPRC_UNLOCK_CMU       = 0xA501U;
static const uint16_t PWC_FPRC_UNLOCK_PWC       = 0xA502U;
static const uint8_t  PWC_PWRC2_UHS             = 0xCFU;
static const uint8_t  PWC_MDSWCR_UHS_TRIG       = 0x10U;
static const uint16_t PWC_RAMOPM_UHS_EXPECTED   = 0x8043U;
static const uint16_t RMU_RSTF0_PORF_BIT        = 0x0001U;
static const uint16_t RMU_RSTF0_PDRF_BIT        = 0x0080U;
static const uint16_t RMU_RSTF0_CLRF_BIT        = 0x8000U;

static const uint32_t CMU_SCFGR_192_96_48       = 0x00112210UL;
static const uint8_t  CMU_XTALCFGR_16MHZ_OSC    = 0x90U;
static const uint8_t  CMU_OSCSTBSR_XTALSTBF     = 0x08U;
static const uint8_t  CMU_OSCSTBSR_MPLLSTBF     = 0x20U;
static const uint32_t CMU_PLLCFGR_16M_TO_192M   = 0x11102F01UL;
static const uint8_t  CMU_CKSWR_MRC             = 0x01U;
static const uint8_t  CMU_CKSWR_MPLL            = 0x05U;

/* =========================================================================
 * GPIO Port Controller — Port index: A=0, B=1, C=2, H=5
 * ========================================================================= */
static const uint32_t GPIO_PORT_A               = 0UL;
static const uint32_t GPIO_PORT_B               = 1UL;
static const uint32_t GPIO_PORT_C               = 2UL;
static const uint32_t GPIO_PORT_H               = 5UL;

static const uint32_t GPIO_PIDR_BASE_ADDR       = 0x40053800UL;
static const uint32_t GPIO_PODR_BASE_ADDR       = 0x40053804UL;
static const uint32_t GPIO_POER_BASE_ADDR       = 0x40053806UL;
static const uint32_t GPIO_POSR_BASE_ADDR       = 0x40053808UL;
static const uint32_t GPIO_PORR_BASE_ADDR       = 0x4005380AUL;
static const uint32_t GPIO_PCR_BASE_ADDR        = 0x40053C00UL;
static const uint32_t GPIO_PFSR_BASE_ADDR       = 0x40053C02UL;
static const uint32_t GPIO_DATA_PORT_STRIDE     = 0x10UL;
static const uint32_t GPIO_PCR_PORT_STRIDE      = 0x40UL;

BORYOZA_FORCE_INLINE uint32_t gpio_pidr_addr(uint32_t port) {
    return GPIO_PIDR_BASE_ADDR + (port * GPIO_DATA_PORT_STRIDE);
}
BORYOZA_FORCE_INLINE uint32_t gpio_podr_addr(uint32_t port) {
    return GPIO_PODR_BASE_ADDR + (port * GPIO_DATA_PORT_STRIDE);
}
BORYOZA_FORCE_INLINE uint32_t gpio_poer_addr(uint32_t port) {
    return GPIO_POER_BASE_ADDR + (port * GPIO_DATA_PORT_STRIDE);
}
BORYOZA_FORCE_INLINE uint32_t gpio_posr_addr(uint32_t port) {
    return GPIO_POSR_BASE_ADDR + (port * GPIO_DATA_PORT_STRIDE);
}
BORYOZA_FORCE_INLINE uint32_t gpio_porr_addr(uint32_t port) {
    return GPIO_PORR_BASE_ADDR + (port * GPIO_DATA_PORT_STRIDE);
}
BORYOZA_FORCE_INLINE uint32_t gpio_pcr_addr(uint32_t port, uint32_t pin) {
    return GPIO_PCR_BASE_ADDR + (port * GPIO_PCR_PORT_STRIDE) + (pin * REG_WORD_STRIDE);
}
BORYOZA_FORCE_INLINE uint32_t gpio_pfsr_addr(uint32_t port, uint32_t pin) {
    return GPIO_PFSR_BASE_ADDR + (port * GPIO_PCR_PORT_STRIDE) + (pin * REG_WORD_STRIDE);
}

static const uint32_t GPIO_PCCR_ADDR            = 0x40053BF8UL;
static const uint16_t GPIO_PCCR_RDWT_MASK       = 0xC000U;
static const uint16_t GPIO_PCCR_RDWT_3CYC       = 0xC000U;
static const uint32_t GPIO_PWPR_ADDR            = 0x40053BFCUL;
static const uint16_t GPIO_PWPR_UNLOCK          = 0xA501U;
static const uint16_t GPIO_PWPR_LOCK            = 0xA500U;

static const uint32_t GPIO_PCR_POUT_BIT         = 0x0001UL;
static const uint32_t GPIO_PCR_POUTE_BIT        = 0x0002UL;
static const uint32_t GPIO_PCR_DRV_LOW          = 0x0000UL;
static const uint32_t GPIO_PCR_PUU_BIT          = 0x0040UL;
static const uint32_t GPIO_PCR_PIN_BIT          = 0x0100UL;
static const uint32_t GPIO_PCR_LTE_BIT          = 0x4000UL;
static const uint32_t GPIO_PCR_DDIS_BIT         = 0x8000UL;

static const uint16_t GPIO_FSEL_GPIO            = 0U;
static const uint16_t GPIO_FSEL_TMRA4_PWM8      = 4U;
static const uint16_t GPIO_FSEL_USART1_TX       = 32U;
static const uint16_t GPIO_FSEL_USART1_RX       = 33U;
static const uint16_t GPIO_FSEL_SPI_MOSI        = 40U;
static const uint16_t GPIO_FSEL_SPI_MISO        = 41U;
static const uint16_t GPIO_FSEL_SPI_SCK         = 43U;

/* =========================================================================
 * AOS Trigger Router & INTC Interrupt Selector
 * ========================================================================= */
static const uint32_t AOS_DMA1_TRGSEL0_ADDR     = 0x40010814UL;
static const uint32_t AOS_DMA1_TRGSEL1_ADDR     = 0x40010818UL;
static const uint32_t AOS_DMA2_TRGSEL0_ADDR     = 0x40010824UL;
static const uint32_t AOS_DMA2_TRGSEL2_ADDR     = 0x4001082CUL;
static const uint32_t AOS_DMA2_TRGSEL3_ADDR     = 0x40010830UL;

static const uint32_t EVT_SRC_USART1_RI         = 0x0117UL; /* 279 */
static const uint32_t EVT_SRC_USART1_TI         = 0x0118UL; /* 280 */
static const uint32_t EVT_SRC_SPI1_SPTI         = 0x012CUL; /* 300 */
static const uint32_t EVT_SRC_SPI3_SPRI         = 0x0135UL; /* 309 */
static const uint32_t EVT_SRC_SPI3_SPTI         = 0x0136UL; /* 310 */

static const uint32_t INTC_SEL40_ADDR           = 0x400510FCUL;
static const uint32_t INTC_SEL41_ADDR           = 0x40051100UL;
static const uint32_t INTC_SEL80_ADDR           = 0x4005119CUL;
static const uint32_t INTC_SEL81_ADDR           = 0x400511A0UL;
static const uint32_t INTC_SEL82_ADDR           = 0x400511A4UL;

static const uint32_t IRQ_NUM_SPI1_DMA2_TC0     = 40UL;
static const uint32_t IRQ_NUM_USART1_DMA2_TC3   = 41UL;
static const uint32_t IRQ_NUM_TMRA4_OVF         = 80UL;
static const uint32_t IRQ_NUM_TMRA1_OVF         = 82UL;

static const uint32_t INT_SRC_DMA2_TC0          = 36UL;  /* Event 36 -> IRQ40 */
static const uint32_t INT_SRC_DMA2_TC3          = 39UL;  /* Event 39 -> IRQ41 */
static const uint32_t INT_SRC_TMRA1_OVF         = 256UL; /* Event 256 -> IRQ82 */
static const uint32_t INT_SRC_TMRA4_OVF         = 265UL; /* Event 265 -> IRQ80 */

/* =========================================================================
 * DMA1 & DMA2 Controllers
 * ========================================================================= */
static const uint32_t DMA1_BASE_ADDR            = 0x40053000UL;
static const uint32_t DMA2_BASE_ADDR            = 0x40053400UL;
static const uint32_t DMA_OFF_INTSTAT1          = 0x08UL;
static const uint32_t DMA_OFF_INTMASK1          = 0x10UL;
static const uint32_t DMA_OFF_INTCLR1           = 0x18UL;
static const uint32_t DMA_OFF_CHEN              = 0x1CUL;
static const uint32_t DMA_OFF_CH0               = 0x40UL;
static const uint32_t DMA_CH_STRIDE             = 0x40UL;
static const uint32_t DMA_CH_OFF_DAR            = 0x04UL;
static const uint32_t DMA_CH_OFF_DTCTL          = 0x08UL;
static const uint32_t DMA_CH_OFF_CHCTL          = 0x1CUL;
static const uint32_t DMA_CH_OFF_MONSAR         = 0x20UL;
static const uint32_t DMA_CH_OFF_MONDAR         = 0x24UL;
static const uint32_t DMA_CH_OFF_MONDTCTL       = 0x28UL;
static const uint32_t DMA_BTC_SHIFT_BASE        = 16U;
static const uint32_t DMA_CFG_VERIFY_LIMIT      = 64UL;

BORYOZA_FORCE_INLINE uint32_t dma_en_addr(uint32_t base) {
    return base;
}
BORYOZA_FORCE_INLINE uint32_t dma_intstat1_addr(uint32_t base) {
    return base + DMA_OFF_INTSTAT1;
}
BORYOZA_FORCE_INLINE uint32_t dma_intmask1_addr(uint32_t base) {
    return base + DMA_OFF_INTMASK1;
}
BORYOZA_FORCE_INLINE uint32_t dma_intclr1_addr(uint32_t base) {
    return base + DMA_OFF_INTCLR1;
}
BORYOZA_FORCE_INLINE uint32_t dma_chen_addr(uint32_t base) {
    return base + DMA_OFF_CHEN;
}
BORYOZA_FORCE_INLINE uint32_t dma_ch_base(uint32_t base, uint32_t chan) {
    return base + DMA_OFF_CH0 + (chan * DMA_CH_STRIDE);
}
BORYOZA_FORCE_INLINE uint32_t dma_sar_addr(uint32_t base, uint32_t chan) {
    return dma_ch_base(base, chan);
}
BORYOZA_FORCE_INLINE uint32_t dma_dar_addr(uint32_t base, uint32_t chan) {
    return dma_ch_base(base, chan) + DMA_CH_OFF_DAR;
}
BORYOZA_FORCE_INLINE uint32_t dma_dtctl_addr(uint32_t base, uint32_t chan) {
    return dma_ch_base(base, chan) + DMA_CH_OFF_DTCTL;
}
BORYOZA_FORCE_INLINE uint32_t dma_chctl_addr(uint32_t base, uint32_t chan) {
    return dma_ch_base(base, chan) + DMA_CH_OFF_CHCTL;
}
BORYOZA_FORCE_INLINE uint32_t dma_monsar_addr(uint32_t base, uint32_t chan) {
    return dma_ch_base(base, chan) + DMA_CH_OFF_MONSAR;
}
BORYOZA_FORCE_INLINE uint32_t dma_mondar_addr(uint32_t base, uint32_t chan) {
    return dma_ch_base(base, chan) + DMA_CH_OFF_MONDAR;
}
BORYOZA_FORCE_INLINE uint32_t dma_mondtctl_addr(uint32_t base, uint32_t chan) {
    return dma_ch_base(base, chan) + DMA_CH_OFF_MONDTCTL;
}
/* Read back MONSAR/MONDAR/MONDTCTL until hardware channel shadow latches */
BORYOZA_FORCE_INLINE void dma_config_chan_verified(uint32_t base, uint32_t chan,
                                                   uint32_t sar, uint32_t dar,
                                                   uint32_t dtctl, uint32_t chctl) {
    mmio_w32(dma_chctl_addr(base, chan), chctl);
    for (uint32_t attempt = 0U; attempt < DMA_CFG_VERIFY_LIMIT; ++attempt) {
        mmio_w32(dma_sar_addr(base, chan), sar);
        mmio_w32(dma_dar_addr(base, chan), dar);
        mmio_w32(dma_dtctl_addr(base, chan), dtctl);
        if ((mmio_r32(dma_monsar_addr(base, chan)) == sar) &&
            (mmio_r32(dma_mondar_addr(base, chan)) == dar) &&
            (mmio_r32(dma_mondtctl_addr(base, chan)) == dtctl)) {
            break;
        }
    }
}
BORYOZA_FORCE_INLINE uint32_t dma_tc_bit(uint32_t chan) {
    return 1UL << chan;
}
BORYOZA_FORCE_INLINE uint32_t dma_tc_btc_mask(uint32_t chan) {
    return (1UL << chan) | (1UL << (DMA_BTC_SHIFT_BASE + chan));
}

static const uint32_t DMA_CHCTL_SPI3_RX_INC     = 0x00000104UL;
static const uint32_t DMA_CHCTL_SPI3_RX_SINK    = 0x00000100UL;
static const uint32_t DMA_CHCTL_SPI3_TX_INC     = 0x00000101UL;
static const uint32_t DMA_CHCTL_SPI1_TX_IRQ     = 0x00001001UL;
static const uint32_t DMA_CHCTL_USART1_RX_INC   = 0x00000004UL;
static const uint32_t DMA_CHCTL_USART1_TX_IRQ   = 0x00001001UL;

/* =========================================================================
 * SPI1 & SPI3 Peripherals
 * ========================================================================= */
static const uint32_t SPI1_BASE_ADDR            = 0x4001C000UL;
static const uint32_t SPI3_BASE_ADDR            = 0x40020000UL;
static const uint32_t SPI_OFF_CR1               = 0x04UL;
static const uint32_t SPI_OFF_CFG1              = 0x0CUL;
static const uint32_t SPI_OFF_SR                = 0x14UL;
static const uint32_t SPI_OFF_CFG2              = 0x18UL;

BORYOZA_FORCE_INLINE uint32_t spi_dr_addr(uint32_t base) {
    return base;
}
BORYOZA_FORCE_INLINE uint32_t spi_cr1_addr(uint32_t base) {
    return base + SPI_OFF_CR1;
}
BORYOZA_FORCE_INLINE uint32_t spi_cfg1_addr(uint32_t base) {
    return base + SPI_OFF_CFG1;
}
BORYOZA_FORCE_INLINE uint32_t spi_sr_addr(uint32_t base) {
    return base + SPI_OFF_SR;
}
BORYOZA_FORCE_INLINE uint32_t spi_cfg2_addr(uint32_t base) {
    return base + SPI_OFF_CFG2;
}

static const uint32_t SPI_SR_IDLNF_BIT          = 0x00000002UL;
static const uint32_t SPI_SR_TDEF_BIT           = 0x00000020UL;
static const uint32_t SPI_SR_RDFF_BIT           = 0x00000080UL;
static const uint32_t SPI_SR_ERR_MASK           = 0x0000001DUL;

static const uint32_t SPI3_CR1_STOPPED          = 0x00000001UL;
static const uint32_t SPI3_CR1_RUNNING          = 0x00000041UL;
static const uint32_t SPI3_CFG1_VAL             = 0x00000010UL;
static const uint32_t SPI3_CFG2_16B_LSBF_MODE3  = 0x00001C07UL;

static const uint32_t SPI1_CR1_STOPPED          = 0x0000000BUL;
static const uint32_t SPI1_CR1_RUNNING          = 0x0000004BUL;
static const uint32_t SPI1_CFG1_VAL             = 0x00000010UL;
static const uint32_t SPI1_CFG2_3MHZ_8B_MODE3   = 0x0000F413UL;

/* =========================================================================
 * USART1 — Split TDR (+4) / RDR (+6)
 * ========================================================================= */
static const uint32_t USART1_BASE_ADDR          = 0x4001D000UL;
static const uint32_t USART1_SR_ADDR            = 0x4001D000UL;
static const uint32_t USART1_TDR_ADDR           = 0x4001D004UL;
static const uint32_t USART1_RDR_ADDR           = 0x4001D006UL;
static const uint32_t USART1_BRR_ADDR           = 0x4001D008UL;
static const uint32_t USART1_CR1_ADDR           = 0x4001D00CUL;
static const uint32_t USART1_CR2_ADDR           = 0x4001D010UL;
static const uint32_t USART1_CR3_ADDR           = 0x4001D014UL;
static const uint32_t USART1_PR_ADDR            = 0x4001D018UL;

static const uint32_t USART_SR_TC_BIT           = (1UL << 6U);
static const uint32_t USART_SR_TXE_BIT          = (1UL << 7U);
static const uint32_t USART_CR1_8N1_OVER8_IDLE  = 0x80008000UL;
static const uint32_t USART_CR1_8N1_OVER8_RUN   = 0x8000800CUL;
static const uint32_t USART_CR1_ERR_CLR_MASK    = 0x000B0000UL;
static const uint32_t USART_BRR_750KBPS         = 0x00000FFFUL;
static const uint32_t USART_BRR_1500KBPS        = 0x000007FFUL;

/* =========================================================================
 * TMRA1 & TMRA4 Timers
 * ========================================================================= */
static const uint32_t TMRA1_BASE_ADDR           = 0x40015000UL;
static const uint32_t TMRA4_BASE_ADDR           = 0x40015C00UL;
static const uint32_t TMRA_OFF_PERAR            = 0x04UL;
static const uint32_t TMRA_OFF_CMPAR8           = 0x5CUL;
static const uint32_t TMRA_OFF_BCSTR            = 0x80UL;
static const uint32_t TMRA_OFF_BCSTRL           = 0x80UL;
static const uint32_t TMRA_OFF_BCSTRH           = 0x81UL;
static const uint32_t TMRA_OFF_CCONR8           = 0x11CUL;
static const uint32_t TMRA_OFF_PCONR8           = 0x15CUL;

BORYOZA_FORCE_INLINE uint32_t tmra_cnter_addr(uint32_t base) {
    return base;
}
BORYOZA_FORCE_INLINE uint32_t tmra_perar_addr(uint32_t base) {
    return base + TMRA_OFF_PERAR;
}
BORYOZA_FORCE_INLINE uint32_t tmra_cmpar8_addr(uint32_t base) {
    return base + TMRA_OFF_CMPAR8;
}
BORYOZA_FORCE_INLINE uint32_t tmra_bcstr_addr(uint32_t base) {
    return base + TMRA_OFF_BCSTR;
}
BORYOZA_FORCE_INLINE uint32_t tmra_bcstrl_addr(uint32_t base) {
    return base + TMRA_OFF_BCSTRL;
}
BORYOZA_FORCE_INLINE uint32_t tmra_bcstrh_addr(uint32_t base) {
    return base + TMRA_OFF_BCSTRH;
}
BORYOZA_FORCE_INLINE uint32_t tmra_cconr8_addr(uint32_t base) {
    return base + TMRA_OFF_CCONR8;
}
BORYOZA_FORCE_INLINE uint32_t tmra_pconr8_addr(uint32_t base) {
    return base + TMRA_OFF_PCONR8;
}

static const uint8_t  TMRA_BCSTRL_DIV1_STOP     = 0x02U; /* START=0, CKDIV=0, DIR=UP before CNTER write */
static const uint8_t  TMRA_BCSTRL_DIV32_STOP    = 0x52U;
static const uint8_t  TMRA_BCSTRL_DIV32_RUN     = 0x53U;
static const uint8_t  TMRA_BCSTRH_ITENOVF       = 0x10U;
static const uint8_t  TMRA_BCSTRH_OVFF          = 0x40U;
static const uint8_t  TMRA_BCSTRH_UDFF_KEEP     = 0x80U;

static const uint16_t TMRA4_PERAR_60HZ          = 49999U;
static const uint16_t TMRA4_PERAR_120HZ         = 24999U;
static const uint16_t TMRA4_FIRST_VD_PRELOAD    = 300U;
static const uint16_t TMRA4_CMPAR8_LOW_PULSE    = 190U;
static const uint16_t TMRA4_PCONR8_PWM_CFG      = 0x1015U;

/* =========================================================================
 * HAL / Transport Public Prototypes (Integer-only, zero FPU)
 * ========================================================================= */
enum { BOOT_SNAP_WORDS = 8 };
void sys_clock_early_192mhz(uint32_t boot_snap[BOOT_SNAP_WORDS]);
bool sys_clock_finish_boot_state(const uint32_t boot_snap[BOOT_SNAP_WORDS]);
bool sys_clock_sram_init_192mhz(void);
extern volatile uint32_t g_boot_reset_flags;
extern volatile uint32_t g_boot_debug_state;
extern volatile uint32_t g_boot_sram_status;
extern volatile uint32_t g_boot_sram_check;
extern volatile uint32_t g_boot_nmi_enable;
enum { NMI_SNAPSHOT_WORDS = 5 };
extern volatile uint32_t g_nmi_snapshot[NMI_SNAPSHOT_WORDS];
extern volatile uint32_t g_clock_init_stage;
extern volatile uint32_t g_clock_osc_status;
void gpio_pads_cold_init(void);
void gpio_pads_bind_z_bus(void);
void gpio_pads_release_z_bus(void);
void gpio_pads_quiesce_z_powerdown(void);
void gpio_pads_bind_pc13_vsync(void);
void gpio_pads_release_pc13_vsync(void);
void gpio_pa9_set_busy(void);
void gpio_pa9_set_ready(void);
uint8_t gpio_pa9_is_busy(void);
uint8_t gpio_pa0_rxd_high(void);
uint8_t gpio_pa3_lens_mounted(void);
uint8_t gpio_pb14_sck_high(void);
uint8_t gpio_ph2_high(void);
void gpio_pa2_set_high(void);
void gpio_pa2_set_low(void);
void gpio_pa1_enable_uart_tx(void);
void gpio_pa1_drive_low_gpio(void);
void gpio_pa1_latch_high_for_baud_switch(uint8_t latch_en);

void tmra1_clock_init(void);
uint64_t tmra1_now_ticks(void);
uint32_t tmra1_now_us32(void);
void tmra1_delay_ticks(uint32_t delta_ticks);
void tmra1_delay_us(uint32_t delay_us);

int efm_program_word_ramfunc(uint32_t flash_addr, uint32_t word_val);
int efm_erase_sector_ramfunc(uint32_t sector_addr);

/* Transport & Diagnostics Prototypes (Integer-only, zero FPU) */
struct adapter_ctx_s;
#include "boryoza_types.h"

void z_spi3_slave_init(void);
bool z_spi3_slave_try_open_first_window(uint64_t now_ticks);
void z_spi3_slave_reset_to_wait_sck(void);
void z_spi3_slave_quiesce_powerdown(void);
void z_spi3_slave_quiesce_mid_phase2(void);
void z_spi3_slave_stop_and_busy(void);
uint8_t z_spi3_slave_poll(adapter_ctx_t *ctx, const optics_workspace_t *optics, uint64_t now_ticks);
uint32_t z_spi3_slave_rx_Window_count(void);
bool z_spi3_slave_provisional_identity_seen(void);
void z_spi3_slave_clear_provisional_seen(void);
bool z_spi3_slave_is_idle_hdr(void);
bool z_spi3_slave_phase2_busy(void);
bool z_spi3_slave_phase2_powerdown_seen(void);
bool z_spi3_slave_cmd51_ready_delivered(void);
void z_spi3_slave_clear_cmd51_ready(void);
uint32_t z_spi3_slave_idle_ticks(uint64_t now_ticks);
uint8_t z_spi3_slave_poll_cmd20_rate_target(const adapter_ctx_t *ctx, uint64_t now_ticks);

void z_spi1_hotline_init(void);
void z_spi1_hotline_stop(void);
void z_spi1_hotline_reset_seq(void);
void z_spi1_hotline_service(adapter_ctx_t *ctx, uint64_t now_ticks);
uint32_t z_spi1_hotline_sent_count(void);
void dma2_set_channel_enabled(uint32_t chan, bool enable);

void e_usart1_dma_init_750k(void);
void e_usart1_dma_switch_baud(uint32_t baud_bps);
uint32_t e_usart1_dma_active_baud(void);
bool e_usart1_dma_link_ready(void);
void e_usart1_dma_stop(void);
bool e_usart1_dma_tx_busy(void);
void e_usart1_dma_set_sync_hold(bool hold);
bool e_usart1_dma_send_frame(const adapter_ctx_t *ctx, uint64_t now_ticks,
                            const uint8_t *wire, uint16_t wire_len);
void e_usart1_dma_service(adapter_ctx_t *ctx, optics_workspace_t *optics, uint64_t now_ticks);
uint32_t e_usart1_dma_rx_error_count(void);

void e_tmra4_vsync_init_60hz(void);
void e_tmra4_vsync_set_rate(uint8_t rate_hz);
void e_tmra4_vsync_stop(void);
uint32_t e_tmra4_vsync_count(void);
uint8_t e_tmra4_vsync_mod240(void);
uint16_t e_tmra4_vsync_phase_ticks(void);
uint64_t e_tmra4_last_vd_ticks(void);
void e_tmra4_vsync_snapshot(uint32_t *out_count, uint8_t *out_mod240,
                            uint16_t *out_phase_ticks, uint64_t *out_last_edge_ticks);
uint8_t e_tmra4_vsync_current_hz(void);

void blackbox_init(void);
int blackbox_record_event(const bb_event_input_t *evt);
void blackbox_flush_to_flash(void);
void blackbox_record_v2_park_summary(adapter_ctx_t *ctx);
void boryoza_hardfault_c_handler(const uint32_t *stack_frame);

void journal_init_and_load(adapter_ctx_t *ctx);
int journal_commit_idle(const adapter_ctx_t *ctx);

bool retram_warm_cache_is_valid(void);
const retram_warm_hdr_t *retram_warm_cache_get(void);
void retram_warm_cache_update_bb_slot(uint16_t next_slot);
void retram_warm_cache_update_journal(uint32_t active_sector, uint16_t next_slot, uint64_t generation,
                                      uint16_t e_lens_id, int32_t e_inf, int32_t e_near,
                                      uint8_t linear_mf_code);
void retram_warm_cache_store_ds13(uint16_t e_lens_id, const uint8_t *buf, uint16_t len);
bool retram_warm_cache_load_ds13(uint16_t e_lens_id, uint8_t *out_buf, uint16_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* BORYOZA_HC32F460_REGS_H */
