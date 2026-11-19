#include "../../include/hc32f460_regs.h"

static const uint32_t DMA2_CH0_SPI1_TX     = 0UL;
static const uint32_t DMA2_CH2_UART_RX_ID  = 2UL;
static const uint32_t DMA2_CH3_UART_TX_ID  = 3UL;
static const uint32_t DMA2_CH0_CLR_MASK    = 0x00010001UL;
static const uint32_t DMA_DTCTL_CNT_SHIFT  = 16U;
static const uint32_t DMA_CNT_LO_MASK      = 0xFFFFUL;
static const uint32_t DMA_RMW_NEAR_END_CNT = 8UL;
static const uint32_t DMA_RMW_SPIN_LIMIT   = 0x5000UL; /* > 200 us guard spin bound */
static const uint32_t DMA_DTCTL_42B        = (42UL << 16U) | 1UL;
static const uint8_t  NVIC_PRIO_SPI1_TC0   = (uint8_t)(1U << 4U);
static const uint32_t IRQ40_REG_IDX        = 40UL / 32UL;
static const uint32_t IRQ40_BIT_MASK       = 1UL << (40UL % 32UL);
static const uint32_t HOTLINE_PERIOD_TICKS = 3000UL; /* Exact 1.000 ms @ 3.000 MHz */
static const uint32_t HOTLINE_TIMEOUT_MULT = 5UL;

BORYOZA_DMA_BUF static uint8_t s_spi1_hot_buf[2][Z_HOTLINE_STRIDE];

static volatile uint8_t s_hotline_dma_tc  = 0U;
static uint8_t          s_hotline_tx_busy = 0U;
static uint8_t          s_hotline_ping    = 0U;
static uint8_t          s_hotline_seq     = 0U;
static uint8_t          s_prev_enabled    = 0U;
static uint32_t         s_last_hot_ticks  = 0U;
static uint32_t         s_due_wait_ticks  = 0U;
static uint32_t         s_hot_sent_total  = 0U;

static void dma2_wait_neighbor_near_end(uint32_t target_chan, uint32_t neighbor_chan) {
    if (target_chan == neighbor_chan) {
        return;
    }
    uint32_t nbit = dma_tc_bit(neighbor_chan);
    if ((mmio_r32(dma_chen_addr(DMA2_BASE_ADDR)) & nbit) == 0UL) {
        return;
    }
    uint32_t init_cnt = (mmio_r32(dma_dtctl_addr(DMA2_BASE_ADDR, neighbor_chan)) >> DMA_DTCTL_CNT_SHIFT) &
                        DMA_CNT_LO_MASK;
    uint32_t mon_cnt = (mmio_r32(dma_mondtctl_addr(DMA2_BASE_ADDR, neighbor_chan)) >> DMA_DTCTL_CNT_SHIFT) &
                       DMA_CNT_LO_MASK;
    if ((mon_cnt <= DMA_RMW_NEAR_END_CNT) && (mon_cnt < init_cnt)) {
        for (uint32_t spin = 0U; spin < DMA_RMW_SPIN_LIMIT; ++spin) {
            if ((mmio_r32(dma_chen_addr(DMA2_BASE_ADDR)) & nbit) == 0UL) {
                break;
            }
            cpu_nop();
        }
    }
}

/* Atomic RMW guard across active channels on multi-channel DMA2 */
BORYOZA_RAMFUNC void dma2_set_channel_enabled(uint32_t chan, bool enable) {
    uint32_t primask = cpu_irq_save();
    dma2_wait_neighbor_near_end(chan, DMA2_CH0_SPI1_TX);
    dma2_wait_neighbor_near_end(chan, DMA2_CH2_UART_RX_ID);
    dma2_wait_neighbor_near_end(chan, DMA2_CH3_UART_TX_ID);
    uint32_t chen = mmio_r32(dma_chen_addr(DMA2_BASE_ADDR));
    uint32_t cbit = dma_tc_bit(chan);
    if (enable) {
        chen |= cbit;
    } else {
        chen &= ~cbit;
    }
    mmio_w32(dma_chen_addr(DMA2_BASE_ADDR), chen);
    cpu_irq_restore(primask);
}

void z_spi1_hotline_init(void) {
    s_hotline_dma_tc = 0U;
    s_hotline_tx_busy = 0U;
    s_hotline_ping = 0U;
    s_hotline_seq = 0U;
    s_prev_enabled = 0U;
    s_last_hot_ticks = 0U;
    s_due_wait_ticks = 0U;
    s_hot_sent_total = 0U;

    mmio_w32(spi_cr1_addr(SPI1_BASE_ADDR), SPI1_CR1_STOPPED);
    mmio_w32(spi_cfg1_addr(SPI1_BASE_ADDR), SPI1_CFG1_VAL);
    mmio_w32(spi_cfg2_addr(SPI1_BASE_ADDR), SPI1_CFG2_3MHZ_8B_MODE3);

    mmio_w32(dma_en_addr(DMA2_BASE_ADDR), 1UL);
    dma2_set_channel_enabled(DMA2_CH0_SPI1_TX, false);
    mmio_w32(dma_intclr1_addr(DMA2_BASE_ADDR), DMA2_CH0_CLR_MASK);

    uint32_t imask = mmio_r32(dma_intmask1_addr(DMA2_BASE_ADDR));
    imask &= ~dma_tc_bit(DMA2_CH0_SPI1_TX);
    mmio_w32(dma_intmask1_addr(DMA2_BASE_ADDR), imask);

    mmio_w32(AOS_DMA2_TRGSEL0_ADDR, EVT_SRC_SPI1_SPTI);

    /* Bind IRQ40 -> INT_SRC_DMA2_TC0 (event 36), highest priority 1 */
    mmio_w32(INTC_SEL40_ADDR, INT_SRC_DMA2_TC0);
    mmio_w8(nvic_ipr_addr(IRQ_NUM_SPI1_DMA2_TC0), NVIC_PRIO_SPI1_TC0);
    mmio_w32(nvic_icpr_addr(IRQ40_REG_IDX), IRQ40_BIT_MASK);
    mmio_w32(nvic_iser_addr(IRQ40_REG_IDX), IRQ40_BIT_MASK);
}

void z_spi1_hotline_stop(void) {
    dma2_set_channel_enabled(DMA2_CH0_SPI1_TX, false);
    mmio_w32(dma_intclr1_addr(DMA2_BASE_ADDR), DMA2_CH0_CLR_MASK);
    mmio_w32(spi_cr1_addr(SPI1_BASE_ADDR), SPI1_CR1_STOPPED);
    s_hotline_dma_tc = 0U;
    s_hotline_tx_busy = 0U;
    s_prev_enabled = 0U;
}

void z_spi1_hotline_reset_seq(void) {
    s_hotline_ping = 0U;
    s_hotline_seq = 0U;
}

BORYOZA_RAMFUNC void SPI1_DMA2_TC0_IRQHandler(void) {
    mmio_w32(dma_intclr1_addr(DMA2_BASE_ADDR), DMA2_CH0_CLR_MASK);
    s_hotline_dma_tc = 1U;
    cpu_dsb_isb();
}

void z_spi1_hotline_service(adapter_ctx_t *ctx, uint64_t now_ticks) {
    uint32_t now32 = (uint32_t)now_ticks;
    ctx->vr.hotline_seq = s_hotline_seq;

    if (s_hotline_tx_busy != 0U) {
        bool dma_done = false;
        if ((s_hotline_dma_tc != 0U) ||
            ((mmio_r32(dma_chen_addr(DMA2_BASE_ADDR)) & dma_tc_bit(DMA2_CH0_SPI1_TX)) == 0UL)) {
            dma_done = true;
        }
        if (dma_done &&
            ((mmio_r32(spi_sr_addr(SPI1_BASE_ADDR)) & SPI_SR_IDLNF_BIT) == 0UL)) {
            s_hotline_dma_tc = 0U;
            s_hotline_tx_busy = 0U;
            mmio_w32(spi_cr1_addr(SPI1_BASE_ADDR), SPI1_CR1_STOPPED);
            s_hot_sent_total += 1U;
        } else if ((now32 - s_last_hot_ticks) > (HOTLINE_TIMEOUT_MULT * HOTLINE_PERIOD_TICKS)) {
            z_spi1_hotline_stop();
        } else {
            return;
        }
    }

    if (ctx->vr.hotline_enabled == 0U) {
        if (s_prev_enabled != 0U) {
            z_spi1_hotline_stop();
        }
        return;
    }
    if (s_prev_enabled == 0U) {
        s_prev_enabled = 1U;
        s_last_hot_ticks = now32;
        s_due_wait_ticks = HOTLINE_PERIOD_TICKS;
    }
    if ((now32 - s_last_hot_ticks) < s_due_wait_ticks) {
        return;
    }

    /* Next deadline anchored from frame launch; slot 0 starts at seq=0
     * so 1000-slot ring wrap always lands on odd sequence parity */
    s_last_hot_ticks = now32;
    s_due_wait_ticks = HOTLINE_PERIOD_TICKS;

    uint8_t bank = s_hotline_ping;
    s_hotline_ping = (uint8_t)(bank ^ 1U);
    uint8_t cur_seq = s_hotline_seq;

    af_servo_build_hotline_frame(ctx, cur_seq, s_spi1_hot_buf[bank], now_ticks);

    s_hotline_seq = (uint8_t)(cur_seq + 1U);
    ctx->vr.hotline_seq = s_hotline_seq;

    s_hotline_dma_tc = 0U;
    s_hotline_tx_busy = 1U;

    mmio_w32(dma_intclr1_addr(DMA2_BASE_ADDR), DMA2_CH0_CLR_MASK);
    dma_config_chan_verified(DMA2_BASE_ADDR, DMA2_CH0_SPI1_TX,
                             (uint32_t)(uintptr_t)s_spi1_hot_buf[bank],
                             spi_dr_addr(SPI1_BASE_ADDR),
                             DMA_DTCTL_42B, DMA_CHCTL_SPI1_TX_IRQ);
    cpu_dmb();

    dma2_set_channel_enabled(DMA2_CH0_SPI1_TX, true);
    mmio_w32(spi_cr1_addr(SPI1_BASE_ADDR), SPI1_CR1_RUNNING);
}

uint32_t z_spi1_hotline_sent_count(void) {
    return s_hot_sent_total;
}
