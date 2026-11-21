#include "../../include/hc32f460_regs.h"

typedef enum {
    UART_TX_IDLE       = 0,
    UART_TX_CS_SETUP   = 1,
    UART_TX_DMA_ACTIVE = 2,
    UART_TX_TAIL_HOLD  = 3,
    UART_TX_COOLDOWN   = 4,
} uart_tx_phase_t;

typedef enum {
    E_HW_STOPPED          = 0,
    E_HW_WAKE_RETRY_LOW   = 1,
    E_HW_POWERUP_HOLD     = 2,
    E_HW_WAKE_WAIT_PH2_HI = 3,
    E_HW_WAKE_HOLD_PA2_HI = 4,
    E_HW_WAKE_WAIT_PH2_LO = 5,
    E_HW_READY            = 6,
    E_HW_BAUD_PRE_GAP     = 7,
    E_HW_BAUD_WAIT_PH2_HI = 8,
    E_HW_BAUD_PRE_SWITCH  = 9,
    E_HW_BAUD_POST_HOLD   = 10,
    E_HW_BAUD_WAIT_PH2_LO = 11,
} e_link_hw_state_t;

static const uint32_t DMA2_CH2_UART_RX      = 2UL;
static const uint32_t DMA2_CH3_UART_TX      = 3UL;
static const uint32_t DMA2_CH2_CLR_MASK     = 0x00040004UL;
static const uint32_t DMA2_CH3_CLR_MASK     = 0x00080008UL;
static const uint32_t DMA_DTCTL_CNT_SHIFT   = 16U;
static const uint32_t DMA_DTCTL_RX_2048     = (2048UL << 16U) | 1UL;
static const uint8_t  NVIC_PRIO_UART_TC3    = (uint8_t)(2U << 4U);
static const uint32_t IRQ41_REG_IDX         = 41UL / 32UL;
static const uint32_t IRQ41_BIT_MASK        = 1UL << (41UL % 32UL);

static const uint32_t CS_SETUP_TICKS        = 60UL;    /* 20 us @ 3MHz */
static const uint32_t CS_TAIL_HOLD_TICKS    = 270UL;   /* 90 us @ 3MHz */
static const uint32_t CS_COOLDOWN_TICKS     = 360UL;   /* 120 us post-ACK gap */
static const uint32_t NO_ACK_GAP_TICKS      = 3600UL;  /* 1.20 ms wait if lens sends no PH2 ACK */
static const uint32_t TX_TIMEOUT_TICKS      = 30000UL; /* 10 ms safety */

/* Physical E-mount PA2 <-> PH2 dataless wakeup & 1.5M baud-switch timings */
static const uint32_t WAKE_RETRY_LOW_TKS    = 900UL;   /* 300 us PA2=0 hold between wakeup retries */
static const uint32_t WAKE_PH2_FALLBACK_TKS = 45000UL; /* 15.00 ms wait for PH2 rise before fallback */
static const uint32_t WAKE_PA2_HOLD_TICKS   = 15450UL; /* 5.15 ms PA2 wakeup pulse hold */
static const uint32_t WAKE_RXD_MAX_TICKS    = 18000UL; /* 6.00 ms RXD idle-high guard */
static const uint32_t WAKE_POST_IDLE_TICKS  = 3000UL;  /* 1.00 ms post-PH2-fall settle */
static const uint32_t BAUD_PRE_GAP_TICKS    = 300UL;   /* 100 us pre-PA2 gap */
static const uint32_t BAUD_PH2_FALLBACK_TKS = 4500UL;  /* 1.50 ms PH2 ack fallback */
static const uint32_t BAUD_PRE_SW_TICKS     = 6000UL;  /* 2.00 ms pre-BRR switch hold */
static const uint32_t BAUD_POST_HOLD_TICKS  = 9000UL;  /* 3.00 ms post-BRR switch hold */
static const uint32_t BAUD_POST_IDLE_TKS    = 60000UL; /* 20.00 ms post-1.5M switch settle */
static const uint32_t PH2_LOW_GUARD_TICKS   = 15000UL; /* 5.00 ms PH2 stuck-high safety guard */

static const uint8_t  EMOUNT_SOF_BYTE       = 0xF0U;
static const uint8_t  EMOUNT_EOF_BYTE       = 0x55U;
static const size_t   EMOUNT_MIN_WIRE_LEN   = 8U;
static const size_t   E_WIRE_COMMAND_OFFSET = 5U;
static const uint32_t BAUD_750K_VAL         = 750000UL;
static const uint32_t BAUD_1500K_VAL        = 1500000UL;
static const uint32_t SHIFT_8               = 8U;
static const uint32_t SHIFT_16              = 16U;
static const uint32_t SHIFT_24              = 24U;
static const uint32_t BYTE_MASK             = 0xFFUL;
static const uint32_t MASK_16BIT            = 0xFFFFUL;

BORYOZA_DMA_BUF static uint8_t s_usart1_tx_buf[E_UART_TX_BUF_BYTES];
BORYOZA_DMA_BUF static uint8_t s_usart1_rx_ring[E_UART_RX_SLOTS][E_UART_RX_SLOT_BYTES];

static e_link_hw_state_t s_hw_state       = E_HW_STOPPED;
static uint32_t          s_hw_ticks       = 0U;
static uint32_t          s_ph2_lo_ticks   = 0U;
static uint8_t           s_ph2_lo_armed   = 0U;
static uart_tx_phase_t   s_tx_phase       = UART_TX_IDLE;
static uint16_t          s_tx_pending_len = 0U;
static uint32_t          s_tx_phase_ticks = 0U;
static uint8_t           s_saw_ph2_reply  = 0U;
static volatile uint8_t  s_tx_dma_tc      = 0U;
static volatile uint32_t s_tx_tc_ticks    = 0U;
static uint32_t          s_tx_owner_epoch = 0U;
static uint32_t          s_tx_owner_serial = 0U;

static uint8_t           s_rx_active_slot = 0U;
static uint8_t           s_prev_ph2       = 0U;
static uint8_t           s_sync_hold      = 0U;
static uint32_t          s_rx_err_count   = 0U;
static uint32_t          s_active_baud    = BAUD_750K_VAL;

static void usart1_arm_rx_slot(uint8_t slot_idx) {
    dma2_set_channel_enabled(DMA2_CH2_UART_RX, false);
    mmio_w32(dma_intclr1_addr(DMA2_BASE_ADDR), DMA2_CH2_CLR_MASK);

    /* Drain RDR before clearing UART framing/parity/overrun status flags */
    (void)mmio_r16(USART1_RDR_ADDR);
    uint32_t cr1 = mmio_r32(USART1_CR1_ADDR);
    mmio_w32(USART1_CR1_ADDR, cr1 | USART_CR1_ERR_CLR_MASK);

    /* SAR must point to USART1->RDR (separate halfword from TDR) */
    dma_config_chan_verified(DMA2_BASE_ADDR, DMA2_CH2_UART_RX,
                             USART1_RDR_ADDR,
                             (uint32_t)(uintptr_t)s_usart1_rx_ring[slot_idx],
                             DMA_DTCTL_RX_2048, DMA_CHCTL_USART1_RX_INC);
    cpu_dmb();

    dma2_set_channel_enabled(DMA2_CH2_UART_RX, true);
}

static void usart1_enable_750k_pads_and_rx(void) {
    gpio_pa1_enable_uart_tx();
    mmio_w32(USART1_CR1_ADDR, USART_CR1_8N1_OVER8_IDLE);
    mmio_w32(USART1_PR_ADDR, 0x00000000UL);
    mmio_w32(USART1_CR2_ADDR, 0x00000000UL);
    mmio_w32(USART1_CR3_ADDR, 0x00000000UL);
    mmio_w32(USART1_BRR_ADDR, USART_BRR_750KBPS);
    mmio_w32(USART1_CR1_ADDR, USART_CR1_8N1_OVER8_RUN | USART_CR1_ERR_CLR_MASK);
    s_active_baud = BAUD_750K_VAL;
    usart1_arm_rx_slot(s_rx_active_slot);
}

void e_usart1_dma_init_750k(void) {
    s_hw_state = E_HW_POWERUP_HOLD;
    s_hw_ticks = (uint32_t)tmra1_now_ticks();
    s_ph2_lo_ticks = 0U;
    s_ph2_lo_armed = 0U;
    s_tx_phase = UART_TX_IDLE;
    s_tx_pending_len = 0U;
    s_saw_ph2_reply = 0U;
    s_tx_dma_tc = 0U;
    s_tx_owner_epoch = 0U;
    s_rx_active_slot = 0U;
    s_prev_ph2 = 0U;
    s_active_baud = BAUD_750K_VAL;

    /* Hold PA2=0 and PA1=0 during lens rail settle before PA2<->PH2 wakeup */
    gpio_pa2_set_low();
    gpio_pa1_drive_low_gpio();
    mmio_w32(USART1_CR1_ADDR, USART_CR1_8N1_OVER8_IDLE);

    mmio_w32(dma_en_addr(DMA2_BASE_ADDR), 1UL);
    uint32_t imask = mmio_r32(dma_intmask1_addr(DMA2_BASE_ADDR));
    imask |= dma_tc_bit(DMA2_CH2_UART_RX);  /* RX polled via PH2 falling edge */
    imask &= ~dma_tc_bit(DMA2_CH3_UART_TX); /* TX uses IRQ41 */
    mmio_w32(dma_intmask1_addr(DMA2_BASE_ADDR), imask);

    mmio_w32(AOS_DMA2_TRGSEL2_ADDR, EVT_SRC_USART1_RI);
    mmio_w32(AOS_DMA2_TRGSEL3_ADDR, EVT_SRC_USART1_TI);

    /* Bind IRQ41 -> INT_SRC_DMA2_TC3 (event 39), priority 2 */
    mmio_w32(INTC_SEL41_ADDR, INT_SRC_DMA2_TC3);
    mmio_w8(nvic_ipr_addr(IRQ_NUM_USART1_DMA2_TC3), NVIC_PRIO_UART_TC3);
    mmio_w32(nvic_icpr_addr(IRQ41_REG_IDX), IRQ41_BIT_MASK);
    mmio_w32(nvic_iser_addr(IRQ41_REG_IDX), IRQ41_BIT_MASK);
}

void e_usart1_dma_switch_baud(uint32_t baud_bps) {
    if (baud_bps == s_active_baud) {
        return;
    }
    uint32_t brr_val = (baud_bps >= BAUD_1500K_VAL) ? USART_BRR_1500KBPS : USART_BRR_750KBPS;

    /* Latch PA1 TXD high via PCR.LTE=1 during baud switch so zero glitch hits lens RX */
    gpio_pa1_latch_high_for_baud_switch(1U);
    mmio_w32(USART1_CR1_ADDR, USART_CR1_8N1_OVER8_IDLE);
    mmio_w32(USART1_BRR_ADDR, brr_val);
    mmio_w32(USART1_CR1_ADDR, USART_CR1_8N1_OVER8_RUN | USART_CR1_ERR_CLR_MASK);
    gpio_pa1_latch_high_for_baud_switch(0U);

    s_active_baud = baud_bps;
    if (gpio_ph2_high() == 0U) {
        usart1_arm_rx_slot(s_rx_active_slot);
    }
}

uint32_t e_usart1_dma_active_baud(void) {
    return s_active_baud;
}

bool e_usart1_dma_link_ready(void) {
    return s_hw_state == E_HW_READY;
}

void e_usart1_dma_stop(void) {
    dma2_set_channel_enabled(DMA2_CH2_UART_RX, false);
    dma2_set_channel_enabled(DMA2_CH3_UART_TX, false);
    mmio_w32(dma_intclr1_addr(DMA2_BASE_ADDR), DMA2_CH2_CLR_MASK | DMA2_CH3_CLR_MASK);
    mmio_w32(USART1_CR1_ADDR, USART_CR1_8N1_OVER8_IDLE);
    gpio_pa2_set_low();
    gpio_pa1_drive_low_gpio();
    s_hw_state = E_HW_STOPPED;
    s_tx_phase = UART_TX_IDLE;
    s_tx_pending_len = 0U;
    s_tx_owner_epoch = 0U;
}

BORYOZA_RAMFUNC void USART1_DMA2_TC3_IRQHandler(void) {
    mmio_w32(dma_intclr1_addr(DMA2_BASE_ADDR), DMA2_CH3_CLR_MASK);
    s_tx_tc_ticks = (uint32_t)tmra1_now_ticks();
    s_tx_dma_tc = 1U;
    cpu_dsb_isb();
}

bool e_usart1_dma_tx_busy(void) {
    if (s_tx_phase != UART_TX_IDLE) {
        return true;
    }
    return s_hw_state != E_HW_READY;
}

void e_usart1_dma_set_sync_hold(bool hold) {
    s_sync_hold = hold ? 1U : 0U;
}

enum {
    LOCAL_CMD_WIRE_CAP = 72,
};

static const uint16_t ASYNC_SLOT_MIN_60HZ_TKS   = 33600U; /* 11.20 ms post-VSync async start */
static const uint16_t ASYNC_SLOT_MAX_60HZ_TKS   = 45600U; /* 15.20 ms post-VSync async end */
static const uint16_t ASYNC_SLOT_MIN_120HZ_TKS  = 15600U; /* 5.20 ms post-VSync async start */
static const uint16_t ASYNC_SLOT_MAX_120HZ_TKS  = 23700U; /* 7.90 ms post-VSync async end */
static const uint16_t HS_CMD0A_MAX_PHASE_TKS    = 45499U; /* 1.50 ms pre-VSync guard for CMD 0A */
static const uint8_t  LEGACY_SEQ_CMD_MIN        = 0xF0U;
static const uint8_t  RATE_120HZ_THRESH         = 120U;

bool e_usart1_dma_send_frame(const adapter_ctx_t *ctx, uint64_t now_ticks,
                            const uint8_t *wire, uint16_t wire_len) {
    if ((s_hw_state != E_HW_READY) || (s_tx_phase != UART_TX_IDLE) ||
        (wire_len == 0U) || (wire_len > (uint16_t)E_UART_TX_BUF_BYTES)) {
        return false;
    }
    if (gpio_ph2_high() != 0U) {
        return false;
    }
    for (uint16_t idx = 0U; idx < wire_len; ++idx) {
        s_usart1_tx_buf[idx] = wire[idx];
    }
    s_tx_pending_len = wire_len;
    s_tx_owner_epoch = ctx->session_epoch;
    s_tx_owner_serial = (wire_len >= EMOUNT_MIN_WIRE_LEN && wire[E_WIRE_COMMAND_OFFSET] == 0x03U)
                            ? ctx->photo_owner_serial : 0U;
    s_tx_dma_tc = 0U;
    s_saw_ph2_reply = 0U;
    gpio_pa2_set_high();
    s_tx_phase_ticks = (uint32_t)now_ticks;
    s_tx_phase = UART_TX_CS_SETUP;
    return true;
}

static void note_e_proto_err(adapter_ctx_t *ctx, uint32_t err_tag,
                             size_t wire_len, uint8_t cmd_byte) {
    s_rx_err_count += 1U;
    if (ctx->bb_e_err_logged == 0U) {
        ctx->bb_e_err_logged = 1U;
        uint32_t diag_a0 = ((uint32_t)wire_len & MASK_16BIT) |
                           ((uint32_t)cmd_byte << SHIFT_16) |
                           ((err_tag & BYTE_MASK) << SHIFT_24);
        uint32_t diag_a2 = ((uint32_t)ctx->hs_step) |
                           ((uint32_t)ctx->session_state << SHIFT_8);
        lens_session_push_diag(ctx, BB_EVT_E_PROTO_ERR, diag_a0, s_rx_err_count, diag_a2, s_active_baud);
    }
}

static void usart1_parse_settled_slot(adapter_ctx_t *ctx, optics_workspace_t *optics,
                                      uint64_t now_ticks, const uint8_t *slot_buf, size_t rx_len) {
    size_t offset = 0U;
    while (offset < rx_len) {
        while ((offset < rx_len) && (slot_buf[offset] != EMOUNT_SOF_BYTE)) {
            offset += 1U;
        }
        if ((rx_len - offset) < EMOUNT_MIN_WIRE_LEN) {
            break;
        }

        const uint8_t *frame = &slot_buf[offset];
        size_t rem_bytes = rx_len - offset;
        size_t wire_len = (size_t)((uint32_t)frame[1] | ((uint32_t)frame[2] << SHIFT_8));
        uint8_t cmd_byte = (rem_bytes > E_WIRE_COMMAND_OFFSET) ? frame[E_WIRE_COMMAND_OFFSET] : 0U;

        if ((wire_len < EMOUNT_MIN_WIRE_LEN) || (wire_len > rem_bytes)) {
            note_e_proto_err(ctx, 1UL, wire_len, cmd_byte);
            break;
        }
        if (frame[wire_len - 1U] != EMOUNT_EOF_BYTE) {
            note_e_proto_err(ctx, 2UL, wire_len, cmd_byte);
            offset += 1U;
            continue;
        }

        uint16_t expected_sum = (uint16_t)((uint32_t)frame[wire_len - 3U] |
                                           ((uint32_t)frame[wire_len - 2U] << SHIFT_8));
        uint16_t actual_sum = boryoza_sum16_emount(frame, wire_len);
        if (expected_sum != actual_sum) {
            note_e_proto_err(ctx, 3UL, wire_len, cmd_byte);
            offset += wire_len;
            continue;
        }

        if (!lens_session_on_emount_frame(ctx, optics, now_ticks, frame, wire_len)) {
            note_e_proto_err(ctx, 4UL, wire_len, cmd_byte);
        }
        offset += wire_len;
    }
}

static void usart1_kickstart_tx_dma(uint32_t now32) {
    uint16_t wire_len = s_tx_pending_len;
    mmio_w32(dma_intclr1_addr(DMA2_BASE_ADDR), DMA2_CH3_CLR_MASK);
    s_tx_dma_tc = 0U;

    if (wire_len == 1U) {
        mmio_w16(USART1_TDR_ADDR, (uint16_t)s_usart1_tx_buf[0]);
        s_tx_tc_ticks = now32;
        s_tx_dma_tc = 1U;
        s_tx_phase = UART_TX_DMA_ACTIVE;
        return;
    }

    /* PIO KICKSTART RULE:
     * Arm DMA2 CH3 with SAR = &tx_buf[1], DAR = USART1_TDR, CNT = N - 1,
     * enable channel, then CPU-write tx_buf[0] (SOF byte) to TDR! */
    uint32_t dtctl_val = (((uint32_t)wire_len - 1UL) << DMA_DTCTL_CNT_SHIFT) | 1UL;
    dma_config_chan_verified(DMA2_BASE_ADDR, DMA2_CH3_UART_TX,
                             (uint32_t)(uintptr_t)&s_usart1_tx_buf[1],
                             USART1_TDR_ADDR,
                             dtctl_val, DMA_CHCTL_USART1_TX_IRQ);
    cpu_dmb();

    dma2_set_channel_enabled(DMA2_CH3_UART_TX, true);
    mmio_w16(USART1_TDR_ADDR, (uint16_t)s_usart1_tx_buf[0]);

    s_tx_phase_ticks = now32;
    s_tx_phase = UART_TX_DMA_ACTIVE;
}

static uint8_t usart1_settle_rx_on_ph2_fall(adapter_ctx_t *ctx, optics_workspace_t *optics,
                                            uint64_t now_ticks) {
    uint8_t cur_ph2 = gpio_ph2_high();
    if (cur_ph2 != 0U) {
        s_saw_ph2_reply = 1U;
    }
    if ((s_prev_ph2 != 0U) && (cur_ph2 == 0U)) {
        s_saw_ph2_reply = 1U;
        uint8_t settled_slot = s_rx_active_slot;
        dma2_set_channel_enabled(DMA2_CH2_UART_RX, false);

        uint32_t mondtctl = mmio_r32(dma_mondtctl_addr(DMA2_BASE_ADDR, DMA2_CH2_UART_RX));
        uint32_t rem_cnt = (mondtctl >> DMA_DTCTL_CNT_SHIFT) & MASK_16BIT;
        size_t rx_len = (rem_cnt <= (uint32_t)E_UART_RX_SLOT_BYTES)
                            ? ((size_t)E_UART_RX_SLOT_BYTES - (size_t)rem_cnt)
                            : 0U;

        s_rx_active_slot = (uint8_t)((s_rx_active_slot + 1U) & (uint8_t)(E_UART_RX_SLOTS - 1U));
        usart1_arm_rx_slot(s_rx_active_slot);

        if (rx_len >= EMOUNT_MIN_WIRE_LEN) {
            usart1_parse_settled_slot(ctx, optics, now_ticks, s_usart1_rx_ring[settled_slot], rx_len);
        }
    }
    s_prev_ph2 = cur_ph2;
    return cur_ph2;
}

static void usart1_complete_ph2_low_wait(adapter_ctx_t *ctx, uint64_t now_ticks,
                                         const uint32_t *settle_ticks_ptr) {
    uint32_t now32 = (uint32_t)now_ticks;
    if ((gpio_ph2_high() != 0U) && ((now32 - s_hw_ticks) < PH2_LOW_GUARD_TICKS)) {
        return;
    }
    if (s_ph2_lo_armed == 0U) {
        s_ph2_lo_armed = 1U;
        s_ph2_lo_ticks = now32;
        return;
    }
    if ((now32 - s_ph2_lo_ticks) >= *settle_ticks_ptr) {
        usart1_arm_rx_slot(s_rx_active_slot);
        s_prev_ph2 = gpio_ph2_high();
        ctx->wake_ready = 1U;
        ctx->state_enter_ticks = now_ticks;
        s_hw_state = E_HW_READY;
    }
}

static void usart1_step_boot_wakeup(adapter_ctx_t *ctx, uint64_t now_ticks) {
    uint32_t now32 = (uint32_t)now_ticks;
    if (s_hw_state == E_HW_WAKE_RETRY_LOW) {
        if ((now32 - s_hw_ticks) >= WAKE_RETRY_LOW_TKS) {
            s_hw_ticks = now32;
            s_hw_state = E_HW_POWERUP_HOLD;
        }
        return;
    }
    if (s_hw_state == E_HW_POWERUP_HOLD) {
        if (ctx->session_state == E_SESSION_HANDSHAKE) {
            gpio_pa2_set_high();
            s_hw_ticks = tmra1_now_ticks();
            s_hw_state = E_HW_WAKE_WAIT_PH2_HI;
        }
        return;
    }
    if (s_hw_state == E_HW_WAKE_WAIT_PH2_HI) {
        if ((gpio_ph2_high() != 0U) || ((now32 - s_hw_ticks) >= WAKE_PH2_FALLBACK_TKS)) {
            usart1_enable_750k_pads_and_rx();
            s_hw_state = E_HW_WAKE_HOLD_PA2_HI;
        }
        return;
    }
    if (s_hw_state == E_HW_WAKE_HOLD_PA2_HI) {
        bool rxd_ok = false;
        if ((gpio_pa0_rxd_high() != 0U) || ((now32 - s_hw_ticks) >= WAKE_RXD_MAX_TICKS)) {
            rxd_ok = true;
        }
        if (((now32 - s_hw_ticks) >= WAKE_PA2_HOLD_TICKS) && rxd_ok) {
            gpio_pa2_set_low();
            s_hw_ticks = now32;
            s_ph2_lo_armed = 0U;
            s_hw_state = E_HW_WAKE_WAIT_PH2_LO;
        }
        return;
    }
    if (s_hw_state == E_HW_WAKE_WAIT_PH2_LO) {
        usart1_complete_ph2_low_wait(ctx, now_ticks, &WAKE_POST_IDLE_TICKS);
    }
}

static void usart1_step_baud_handshake(adapter_ctx_t *ctx, uint64_t now_ticks) {
    uint32_t now32 = (uint32_t)now_ticks;
    if (s_hw_state == E_HW_BAUD_PRE_GAP) {
        if ((now32 - s_hw_ticks) >= BAUD_PRE_GAP_TICKS) {
            gpio_pa2_set_high();
            s_hw_ticks = tmra1_now_ticks();
            s_hw_state = E_HW_BAUD_WAIT_PH2_HI;
        }
        return;
    }
    if (s_hw_state == E_HW_BAUD_WAIT_PH2_HI) {
        if ((gpio_ph2_high() != 0U) || ((now32 - s_hw_ticks) >= BAUD_PH2_FALLBACK_TKS)) {
            s_hw_ticks = now32;
            s_hw_state = E_HW_BAUD_PRE_SWITCH;
        }
        return;
    }
    if (s_hw_state == E_HW_BAUD_PRE_SWITCH) {
        if ((now32 - s_hw_ticks) >= BAUD_PRE_SW_TICKS) {
            e_usart1_dma_switch_baud(ctx->uart_baud);
            s_hw_ticks = now32;
            s_hw_state = E_HW_BAUD_POST_HOLD;
        }
        return;
    }
    if (s_hw_state == E_HW_BAUD_POST_HOLD) {
        if ((now32 - s_hw_ticks) >= BAUD_POST_HOLD_TICKS) {
            gpio_pa2_set_low();
            s_hw_ticks = now32;
            s_ph2_lo_armed = 0U;
            s_hw_state = E_HW_BAUD_WAIT_PH2_LO;
        }
        return;
    }
    if (s_hw_state == E_HW_BAUD_WAIT_PH2_LO) {
        usart1_complete_ph2_low_wait(ctx, now_ticks, &BAUD_POST_IDLE_TKS);
    }
}

static void usart1_service_tx_idle(uint8_t cur_ph2, adapter_ctx_t *ctx, uint64_t now_ticks) {
    if (cur_ph2 != 0U) {
        return;
    }
    /* Execute 5.0ms PA2 <-> PH2 dataless handshake before post-0C 1.5M frame */
    if ((ctx->uart_baud != 0U) && (ctx->uart_baud != s_active_baud)) {
        s_hw_ticks = (uint32_t)now_ticks;
        s_hw_state = E_HW_BAUD_PRE_GAP;
        return;
    }
    if ((ctx->session_state == E_SESSION_ACTIVE) ||
        (ctx->session_state == E_SESSION_PARKING)) {
        if ((ctx->session_state == E_SESSION_ACTIVE) && (s_sync_hold != 0U)) {
            return;
        }
        uint16_t vd_phase = e_tmra4_vsync_phase_ticks();
        bool fast_120 = (ctx->active_vsync_hz >= RATE_120HZ_THRESH);
        uint16_t slot_min = fast_120 ? ASYNC_SLOT_MIN_120HZ_TKS : ASYNC_SLOT_MIN_60HZ_TKS;
        uint16_t slot_max = fast_120 ? ASYNC_SLOT_MAX_120HZ_TKS : ASYNC_SLOT_MAX_60HZ_TKS;
        if ((vd_phase < slot_min) || (vd_phase > slot_max)) {
            return;
        }
    } else if ((ctx->session_state == E_SESSION_HANDSHAKE) &&
               (ctx->hs_step == HANDSHAKE_STEP_CMD0A) &&
               (e_tmra4_vsync_phase_ticks() > HS_CMD0A_MAX_PHASE_TKS)) {
        return;
    }
    e_tx_cmd_t next_cmd;
    uint8_t previous_head = ctx->tx_q_head;
    uint8_t previous_seq = ctx->e_tx_seq;
    if (lens_session_pop_tx(ctx, &next_cmd)) {
        uint8_t raw_wire[LOCAL_CMD_WIRE_CAP];
        uint8_t seq_val = 0U;
        if (next_cmd.cmd >= LEGACY_SEQ_CMD_MIN) {
            ctx->e_tx_seq = (uint8_t)(ctx->e_tx_seq + 1U);
            seq_val = ctx->e_tx_seq;
        }
        size_t wlen = boryoza_wrap_emount_frame(
            raw_wire, next_cmd.frame_type, &seq_val,
            next_cmd.cmd, next_cmd.payload, next_cmd.payload_len);
        if (!e_usart1_dma_send_frame(ctx, now_ticks, raw_wire, (uint16_t)wlen)) {
            ctx->tx_q_head = previous_head;
            ctx->e_tx_seq = previous_seq;
        } else {
            s_tx_owner_serial = next_cmd.serial;
        }
    }
}

void e_usart1_dma_service(adapter_ctx_t *ctx, optics_workspace_t *optics, uint64_t now_ticks) {
    uint32_t now32 = (uint32_t)now_ticks;
    if (s_hw_state == E_HW_STOPPED) {
        return;
    }
    if (ctx->wake_retry_req != 0U) {
        ctx->wake_retry_req = 0U;
        ctx->wake_ready = 0U;
        dma2_set_channel_enabled(DMA2_CH2_UART_RX, false);
        dma2_set_channel_enabled(DMA2_CH3_UART_TX, false);
        mmio_w32(USART1_CR1_ADDR, USART_CR1_8N1_OVER8_IDLE);
        gpio_pa2_set_low();
        gpio_pa1_drive_low_gpio();
        s_tx_phase = UART_TX_IDLE;
        s_tx_pending_len = 0U;
        s_hw_ticks = now32;
        s_hw_state = E_HW_WAKE_RETRY_LOW;
        return;
    }
    if (s_hw_state < E_HW_READY) {
        usart1_step_boot_wakeup(ctx, now_ticks);
        return;
    }
    if (s_hw_state > E_HW_READY) {
        usart1_step_baud_handshake(ctx, now_ticks);
        return;
    }

    uint8_t cur_ph2 = usart1_settle_rx_on_ph2_fall(ctx, optics, now_ticks);

    switch (s_tx_phase) {
        case UART_TX_IDLE:
            usart1_service_tx_idle(cur_ph2, ctx, now_ticks);
            break;
        case UART_TX_CS_SETUP:
            if ((now32 - s_tx_phase_ticks) >= CS_SETUP_TICKS) {
                usart1_kickstart_tx_dma(now32);
            }
            break;
        case UART_TX_DMA_ACTIVE:
            if ((s_tx_dma_tc != 0U) && ((mmio_r32(USART1_SR_ADDR) & USART_SR_TC_BIT) != 0UL)) {
                s_tx_phase_ticks = now32;
                s_tx_phase = UART_TX_TAIL_HOLD;
            } else if ((now32 - s_tx_phase_ticks) > TX_TIMEOUT_TICKS) {
                dma2_set_channel_enabled(DMA2_CH3_UART_TX, false);
                mmio_w32(dma_intclr1_addr(DMA2_BASE_ADDR), DMA2_CH3_CLR_MASK);
                s_tx_dma_tc = 0U;
                gpio_pa2_set_low();
                vr_controller_on_emount_tx_result(ctx, s_tx_owner_epoch,
                                                  s_usart1_tx_buf, s_tx_pending_len,
                                                  s_tx_owner_serial, false, now_ticks);
                s_saw_ph2_reply = 0U;
                s_tx_phase_ticks = now32;
                s_tx_phase = UART_TX_COOLDOWN;
            }
            break;
        case UART_TX_TAIL_HOLD: {
            (void)s_tx_tc_ticks;
            if ((now32 - s_tx_phase_ticks) >= CS_TAIL_HOLD_TICKS) {
                gpio_pa2_set_low();
                vr_controller_on_emount_tx_result(ctx, s_tx_owner_epoch,
                                                  s_usart1_tx_buf, s_tx_pending_len,
                                                  s_tx_owner_serial, true, now_ticks);
                s_tx_phase_ticks = now32;
                s_tx_phase = UART_TX_COOLDOWN;
            }
            break;
        }
        case UART_TX_COOLDOWN: {
            uint32_t elapsed = now32 - s_tx_phase_ticks;
            bool is_unacked_id04 = (bool)((s_tx_pending_len >= (uint16_t)EMOUNT_MIN_WIRE_LEN) &&
                                          (s_usart1_tx_buf[E_WIRE_COMMAND_OFFSET] == 0x04U));
            uint32_t req_gap = ((s_saw_ph2_reply != 0U) || is_unacked_id04)
                                   ? CS_COOLDOWN_TICKS
                                   : NO_ACK_GAP_TICKS;
            if ((cur_ph2 == 0U) && (elapsed >= req_gap)) {
                s_tx_phase = UART_TX_IDLE;
            }
            break;
        }
    }
}

uint32_t e_usart1_dma_rx_error_count(void) {
    return s_rx_err_count;
}
