#include "../../include/hc32f460_regs.h"

static const uint8_t  NVIC_PRIO_TMRA4_OVF = (uint8_t)(2U << 4U);
static const uint32_t IRQ80_REG_IDX       = 80UL / 32UL;
static const uint32_t IRQ80_BIT_MASK      = 1UL << (80UL % 32UL);
static const uint8_t  RATE_120HZ_VAL      = 120U;
static const uint8_t  VD_MODULO_240       = 240U;

static volatile uint32_t s_vd_total      = 0U;
static volatile uint8_t  s_vd_mod240     = 0U;
static volatile uint64_t s_last_vd_ticks = 0ULL;

void e_tmra4_vsync_init_60hz(void) {
    s_vd_total = 0U;
    s_vd_mod240 = 0U;
    s_last_vd_ticks = tmra1_now_ticks();

    gpio_pads_bind_pc13_vsync();

    /* Clear START=0 and CKDIV=0 BEFORE writing CNTER */
    mmio_w8(tmra_bcstrl_addr(TMRA4_BASE_ADDR), TMRA_BCSTRL_DIV1_STOP);
    mmio_w16(tmra_perar_addr(TMRA4_BASE_ADDR), TMRA4_PERAR_60HZ);
    mmio_w16(tmra_cmpar8_addr(TMRA4_BASE_ADDR), TMRA4_CMPAR8_LOW_PULSE);
    mmio_w16(tmra_cconr8_addr(TMRA4_BASE_ADDR), 0x0000U);
    mmio_w16(tmra_pconr8_addr(TMRA4_BASE_ADDR), TMRA4_PCONR8_PWM_CFG);
    /* Preload counter for 100 us initial VSync pulse */
    mmio_w16(tmra_cnter_addr(TMRA4_BASE_ADDR),
             (uint16_t)(TMRA4_PERAR_60HZ - TMRA4_FIRST_VD_PRELOAD));
    mmio_w8(tmra_bcstrl_addr(TMRA4_BASE_ADDR), TMRA_BCSTRL_DIV32_STOP);
    mmio_w8(tmra_bcstrh_addr(TMRA4_BASE_ADDR), TMRA_BCSTRH_ITENOVF);

    /* Bind IRQ80 -> INT_SRC_TMRA_4_OVF (event 265), priority 2 */
    mmio_w32(INTC_SEL80_ADDR, INT_SRC_TMRA4_OVF);
    mmio_w8(nvic_ipr_addr(IRQ_NUM_TMRA4_OVF), NVIC_PRIO_TMRA4_OVF);
    mmio_w32(nvic_icpr_addr(IRQ80_REG_IDX), IRQ80_BIT_MASK);
    mmio_w32(nvic_iser_addr(IRQ80_REG_IDX), IRQ80_BIT_MASK);

    mmio_w8(tmra_bcstrl_addr(TMRA4_BASE_ADDR), TMRA_BCSTRL_DIV32_RUN);
}

void e_tmra4_vsync_set_rate(uint8_t rate_hz) {
    uint16_t target_perar = (rate_hz >= RATE_120HZ_VAL) ? TMRA4_PERAR_120HZ : TMRA4_PERAR_60HZ;
    uint32_t primask = cpu_irq_save();
    mmio_w16(tmra_perar_addr(TMRA4_BASE_ADDR), target_perar);
    if (mmio_r16(tmra_cnter_addr(TMRA4_BASE_ADDR)) >= target_perar) {
        mmio_w8(tmra_bcstrl_addr(TMRA4_BASE_ADDR), TMRA_BCSTRL_DIV1_STOP);
        mmio_w16(tmra_cnter_addr(TMRA4_BASE_ADDR), 0x0000U);
        mmio_w8(tmra_bcstrl_addr(TMRA4_BASE_ADDR), TMRA_BCSTRL_DIV32_RUN);
    }
    cpu_irq_restore(primask);
}

void e_tmra4_vsync_stop(void) {
    mmio_w8(tmra_bcstrl_addr(TMRA4_BASE_ADDR), TMRA_BCSTRL_DIV1_STOP);
    mmio_w16(tmra_cnter_addr(TMRA4_BASE_ADDR), 0x0000U);
    gpio_pads_release_pc13_vsync();
}

BORYOZA_RAMFUNC void TMRA4_OVF_IRQHandler(void) {
    uint8_t bcstrh = mmio_r8(tmra_bcstrh_addr(TMRA4_BASE_ADDR));
    if ((bcstrh & TMRA_BCSTRH_OVFF) != 0U) {
        mmio_w8(tmra_bcstrh_addr(TMRA4_BASE_ADDR),
                (uint8_t)(((uint32_t)bcstrh | (uint32_t)TMRA_BCSTRH_UDFF_KEEP) &
                          ~(uint32_t)TMRA_BCSTRH_OVFF));
        s_vd_total = s_vd_total + 1U;
        uint8_t next_mod = (uint8_t)(s_vd_mod240 + 1U);
        s_vd_mod240 = (next_mod >= VD_MODULO_240) ? 0U : next_mod;
        s_last_vd_ticks = tmra1_now_ticks();
    }
    cpu_dsb_isb();
}

uint32_t e_tmra4_vsync_count(void) {
    return s_vd_total;
}

uint8_t e_tmra4_vsync_mod240(void) {
    return s_vd_mod240;
}

uint16_t e_tmra4_vsync_phase_ticks(void) {
    return mmio_r16(tmra_cnter_addr(TMRA4_BASE_ADDR));
}

uint64_t e_tmra4_last_vd_ticks(void) {
    uint32_t primask = cpu_irq_save();
    uint64_t ticks = s_last_vd_ticks;
    cpu_irq_restore(primask);
    return ticks;
}
