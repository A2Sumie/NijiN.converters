#include "../../include/hc32f460_regs.h"

static const uint8_t  NVIC_PRIO_TMRA1_OVF = (uint8_t)(3U << 4U);
static const uint32_t IRQ82_REG_IDX       = 82UL / 32UL;
static const uint32_t IRQ82_BIT_MASK      = 1UL << (82UL % 32UL);
static const uint16_t TMRA_FULL_PERIOD    = 0xFFFFU;
static const uint16_t TMRA_HALF_PERIOD    = 0x8000U;
static const uint32_t EPOCH_SHIFT_BITS    = 16U;
static const uint64_t WORD_LO_MASK        = 0xFFFFULL;
static const uint64_t US_PER_EPOCH_FACTOR = 21845ULL;
static const uint64_t US_ROUND_HALF       = 32768ULL;

static volatile uint32_t g_tmra1_epoch = 0U;

void tmra1_clock_init(void) {
    g_tmra1_epoch = 0U;

    /* Split 8-bit BCSTRL/BCSTRH: clear START=0 and CKDIV=0 BEFORE writing CNTER,
     * then configure PCLK1/32 = 3.000 MHz */
    mmio_w8(tmra_bcstrl_addr(TMRA1_BASE_ADDR), TMRA_BCSTRL_DIV1_STOP);
    mmio_w16(tmra_cnter_addr(TMRA1_BASE_ADDR), 0x0000U);
    mmio_w16(tmra_perar_addr(TMRA1_BASE_ADDR), TMRA_FULL_PERIOD);
    mmio_w8(tmra_bcstrl_addr(TMRA1_BASE_ADDR), TMRA_BCSTRL_DIV32_STOP);
    mmio_w8(tmra_bcstrh_addr(TMRA1_BASE_ADDR), TMRA_BCSTRH_ITENOVF);

    /* Bind IRQ82 -> INT_SRC_TMRA_1_OVF (event 256) */
    mmio_w32(INTC_SEL82_ADDR, INT_SRC_TMRA1_OVF);
    mmio_w8(nvic_ipr_addr(IRQ_NUM_TMRA1_OVF), NVIC_PRIO_TMRA1_OVF);
    mmio_w32(nvic_icpr_addr(IRQ82_REG_IDX), IRQ82_BIT_MASK);
    mmio_w32(nvic_iser_addr(IRQ82_REG_IDX), IRQ82_BIT_MASK);

    mmio_w8(tmra_bcstrl_addr(TMRA1_BASE_ADDR), TMRA_BCSTRL_DIV32_RUN);
}

BORYOZA_RAMFUNC void TMRA1_OVF_IRQHandler(void) {
    uint8_t bcstrh = mmio_r8(tmra_bcstrh_addr(TMRA1_BASE_ADDR));
    if ((bcstrh & TMRA_BCSTRH_OVFF) != 0U) {
        /* W0C on OVFF (bit6) while keeping UDFF (bit7) high and ITENOVF (bit4) enabled */
        mmio_w8(tmra_bcstrh_addr(TMRA1_BASE_ADDR),
                (uint8_t)(((uint32_t)bcstrh | (uint32_t)TMRA_BCSTRH_UDFF_KEEP) &
                          ~(uint32_t)TMRA_BCSTRH_OVFF));
        g_tmra1_epoch = g_tmra1_epoch + 1U;
    }
    cpu_dsb_isb();
}

BORYOZA_RAMFUNC uint64_t tmra1_now_ticks(void) {
    for (;;) {
        uint32_t epoch_hi1 = g_tmra1_epoch;
        uint16_t cnt_lo = mmio_r16(tmra_cnter_addr(TMRA1_BASE_ADDR));
        uint8_t bcstrh = mmio_r8(tmra_bcstrh_addr(TMRA1_BASE_ADDR));
        uint32_t epoch_hi2 = g_tmra1_epoch;
        if (epoch_hi1 == epoch_hi2) {
            if (((bcstrh & TMRA_BCSTRH_OVFF) != 0U) && (cnt_lo < TMRA_HALF_PERIOD)) {
                epoch_hi1 += 1U;
            }
            return (((uint64_t)epoch_hi1) << EPOCH_SHIFT_BITS) | (uint64_t)cnt_lo;
        }
    }
}

BORYOZA_RAMFUNC uint32_t tmra1_now_us32(void) {
    uint64_t ticks = tmra1_now_ticks();
    uint64_t hi_part = (ticks >> EPOCH_SHIFT_BITS) * US_PER_EPOCH_FACTOR;
    uint64_t lo_part = (((ticks & WORD_LO_MASK) * US_PER_EPOCH_FACTOR) + US_ROUND_HALF) >> EPOCH_SHIFT_BITS;
    return (uint32_t)(hi_part + lo_part);
}

BORYOZA_RAMFUNC void tmra1_delay_ticks(uint32_t delta_ticks) {
    uint64_t start = tmra1_now_ticks();
    while ((tmra1_now_ticks() - start) < (uint64_t)delta_ticks) {
        cpu_nop();
    }
}

void tmra1_delay_us(uint32_t delay_us) {
    tmra1_delay_ticks(delay_us * HC32_TICKS_PER_US);
}
