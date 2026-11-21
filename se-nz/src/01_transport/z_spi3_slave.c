#include "../../include/hc32f460_regs.h"

typedef enum {
    SPI3_WIN_STOPPED  = 0,
    SPI3_WIN_CMD_HDR  = 1,
    SPI3_WIN_DATA_PAY = 2,
} spi3_win_phase_t;

static const uint32_t DMA1_CH0_RX              = 0UL;
static const uint32_t DMA1_CH1_TX              = 1UL;
static const uint32_t DMA1_BOTH_TC_MASK        = 0x00000003UL;
static const uint32_t DMA1_BOTH_CLR_MASK       = 0x00030003UL;
static const uint32_t DMA1_BOTH_CHEN_MASK      = 0x00000003UL;
static const uint32_t DMA_DTCTL_CNT_SHIFT      = 16U;
static const uint32_t DMA_DTCTL_BLK_ONE        = 1UL;
static const uint32_t DMA_MASK_ALL_IRQS        = 0xFFFFFFFFUL;
static const uint32_t HDR_HALFWORDS            = 4UL;
static const uint32_t MAX_WIN_HALFWORDS        = 4096UL;
static const uint32_t SMALL_WIN_MAX_BYTES      = 512UL;
static const uint32_t PA9_MIN_BUSY_TICKS       = 90UL;
static const uint32_t PA9_FIRST_WIN_HOLD_TICKS = 1500UL;
static const uint8_t  Z_SYNC_BYTE              = 0x55U;
static const uint8_t  Z_CMD_HOTLINE_CTL        = 0x16U;
static const uint8_t  Z_CMD_LENS_IDENTITY      = 0x10U;
static const uint8_t  Z_CMD_STATUS_20          = 0x20U;
static const uint8_t  Z_CMD_OPTICS_24          = 0x24U;
static const uint8_t  Z_CMD_PARK_50            = 0x50U;
static const uint8_t  Z_CMD_PARK_STAT_51       = 0x51U;
static const uint8_t  Z_CMD51_READY_VAL        = 0x01U;
static const uint8_t  Z_CMD_APERTURE_A0        = 0xA0U;
static const uint8_t  Z_A0_MAX_SUB             = 0x07U;
static const uint8_t  Z_CMD_APERTURE_A1        = 0xA1U;
static const uint8_t  Z_CMD_APERTURE_A3        = 0xA3U;
static const uint8_t  Z_CMD_AF_C0              = 0xC0U;
static const uint8_t  Z_CMD_AF_C4              = 0xC4U;
static const uint8_t  Z_CMD_VR_D2              = 0xD2U;
static const uint8_t  Z_CMD_VR_D8              = 0xD8U;
static const uint8_t  Z_CMD_AF_E0              = 0xE0U;
static const uint8_t  Z_CMD_AF_EF              = 0xEFU;
static const uint32_t SCK_STABLE_HIGH_TICKS    = 15000UL;
static const uint32_t PHASE2_STUCK_MAX_TICKS   = 3000000UL;
static const uint64_t RATE_OBS_RETRY_MIN_TKS   = 12000ULL;
static const uint64_t RATE_OBS_FAST_MIN_TKS    = 18000ULL;
static const uint64_t RATE_OBS_FAST_MAX_TKS    = 34500ULL;
static const uint64_t RATE_OBS_SLOW_MIN_TKS    = 40500ULL;
static const uint64_t RATE_OBS_GAP_MAX_TKS     = 240000ULL;
static const uint8_t  RATE_OBS_FAST_STREAK     = 4U;
static const uint8_t  RATE_OBS_SLOW_STREAK     = 3U;
enum {
    RATE_OBS_HZ_60  = 60U,
    RATE_OBS_HZ_120 = 120U,
};

typedef struct {
    uint32_t session_epoch;
    uint64_t done_ticks;
} spi3_rate_sample_t;
static const size_t   HDR_OFF_CMD              = 3U;
static const size_t   HDR_OFF_SUB              = 4U;
static const size_t   HDR_OFF_LEN_LO           = 5U;
static const size_t   HDR_OFF_LEN_HI           = 6U;
static const uint8_t  POLL_EVT_HDR_OK          = 0x01U;
static const uint8_t  POLL_EVT_DATA_OK         = 0x02U;
static const uint8_t  POLL_EVT_HDR_ERR         = 0x04U;
static const uint32_t BYTE_MASK                = 0xFFUL;
static const uint32_t SHIFT_8                  = 8U;

BORYOZA_DMA_BUF static uint8_t  s_spi3_rx_buf[Z_SPI3_SMALL_BUF_BYTES];
BORYOZA_DMA_BUF static uint8_t  s_spi3_tx_buf[Z_SPI3_SMALL_BUF_BYTES];
BORYOZA_DMA_BUF static uint32_t s_spi3_rx_trash_sink;

static spi3_win_phase_t s_spi3_phase      = SPI3_WIN_STOPPED;
static uint8_t          s_cur_cmd         = 0U;
static uint8_t          s_cur_sub         = 0U;
static uint16_t         s_cur_data_bytes  = 0U;
static uint32_t         s_spi3_win_count  = 0U;
static uint8_t          s_first_win_bb    = 0U;
static uint8_t          s_sck_high_seen   = 0U;
static uint32_t         s_sck_high_t32    = 0U;
static uint32_t         s_phase2_t32      = 0U;
static uint64_t         s_last_done_ticks = 0ULL;
static uint8_t          s_cmd51_tx_stat   = 0U;
static uint8_t          s_cmd51_ready_ok  = 0U;
static uint8_t          s_phase2_pdown    = 0U;
static uint8_t          s_identity_ready_at_reply = 0U;
static uint8_t          s_optics_ready_at_reply   = 0U;
static bool             s_provisional_identity_seen = false;
static uint32_t         s_hdr_epoch               = 0UL;
static uint32_t         s_hdr_snap_sig            = 0UL;
static uint32_t         s_hdr_opt_gen             = 0UL;
static uint32_t         s_z20_obs_epoch           = 0UL;
static uint64_t         s_last_z20_full_ticks     = 0ULL;
static uint8_t          s_z20_fast_streak         = 0U;
static uint8_t          s_z20_slow_streak         = 0U;
static uint8_t          s_z20_target_hz           = (uint8_t)RATE_OBS_HZ_60;
static uint8_t          s_z20_rate_decided        = 0U;

static void spi3_reset_cmd20_rate_observer(uint32_t session_epoch) {
    s_z20_obs_epoch = session_epoch;
    s_last_z20_full_ticks = 0ULL;
    s_z20_fast_streak = 0U;
    s_z20_slow_streak = 0U;
    s_z20_target_hz = (uint8_t)RATE_OBS_HZ_60;
    s_z20_rate_decided = 0U;
}

static void spi3_observe_cmd20_rate(const spi3_rate_sample_t *sample) {
    if (s_z20_obs_epoch != sample->session_epoch) {
        spi3_reset_cmd20_rate_observer(sample->session_epoch);
    }
    if (s_last_z20_full_ticks == 0ULL) {
        s_last_z20_full_ticks = (sample->done_ticks != 0ULL) ? sample->done_ticks : 1ULL;
        return;
    }
    if (sample->done_ticks <= s_last_z20_full_ticks) {
        return;
    }
    uint64_t delta_ticks = sample->done_ticks - s_last_z20_full_ticks;
    if (delta_ticks < RATE_OBS_RETRY_MIN_TKS) {
        return;
    }
    s_last_z20_full_ticks = sample->done_ticks;
    if ((delta_ticks >= RATE_OBS_FAST_MIN_TKS) && (delta_ticks <= RATE_OBS_FAST_MAX_TKS)) {
        s_z20_slow_streak = 0U;
        if (s_z20_fast_streak < (uint8_t)BYTE_MASK) {
            s_z20_fast_streak = (uint8_t)(s_z20_fast_streak + 1U);
        }
        if (s_z20_fast_streak >= RATE_OBS_FAST_STREAK) {
            s_z20_target_hz = (uint8_t)RATE_OBS_HZ_120;
            s_z20_rate_decided = 1U;
        }
    } else if ((delta_ticks >= RATE_OBS_SLOW_MIN_TKS) && (delta_ticks <= RATE_OBS_GAP_MAX_TKS)) {
        s_z20_fast_streak = 0U;
        if (s_z20_slow_streak < (uint8_t)BYTE_MASK) {
            s_z20_slow_streak = (uint8_t)(s_z20_slow_streak + 1U);
        }
        if (s_z20_slow_streak >= RATE_OBS_SLOW_STREAK) {
            s_z20_target_hz = (uint8_t)RATE_OBS_HZ_60;
            s_z20_rate_decided = 1U;
        }
    } else if (delta_ticks > RATE_OBS_GAP_MAX_TKS) {
        s_z20_fast_streak = 0U;
        s_z20_slow_streak = 0U;
        s_z20_target_hz = (uint8_t)RATE_OBS_HZ_60;
        s_z20_rate_decided = 1U;
    } else {
        s_z20_fast_streak = 0U;
        s_z20_slow_streak = 0U;
    }
}

BORYOZA_RAMFUNC static void spi3_arm_window_ramfunc(const uint8_t *tx_src, bool use_rx_sink,
                                                    uint32_t halfword_cnt) {
    uint32_t rx_dst_addr = use_rx_sink ? (uint32_t)(uintptr_t)&s_spi3_rx_trash_sink
                                       : (uint32_t)(uintptr_t)s_spi3_rx_buf;
    uint32_t rx_chctl = use_rx_sink ? DMA_CHCTL_SPI3_RX_SINK : DMA_CHCTL_SPI3_RX_INC;
    mmio_w32(spi_cr1_addr(SPI3_BASE_ADDR), SPI3_CR1_STOPPED);
    uint32_t spi_sr = mmio_r32(spi_sr_addr(SPI3_BASE_ADDR));
    if ((spi_sr & SPI_SR_RDFF_BIT) != 0UL) {
        (void)mmio_r32(spi_dr_addr(SPI3_BASE_ADDR));
    }
    if ((spi_sr & SPI_SR_ERR_MASK) != 0UL) {
        static const uint32_t SPI_SR_W0C_CLEAR_ALL_ERR = 0x000000A2UL;
        mmio_w32(spi_sr_addr(SPI3_BASE_ADDR), SPI_SR_W0C_CLEAR_ALL_ERR);
    }

    mmio_w32(dma_chen_addr(DMA1_BASE_ADDR), 0x00000000UL);
    mmio_w32(dma_intclr1_addr(DMA1_BASE_ADDR), DMA1_BOTH_CLR_MASK);

    uint32_t dtctl_val = (halfword_cnt << DMA_DTCTL_CNT_SHIFT) | DMA_DTCTL_BLK_ONE;

    /* DMA1 CH0/CH1: SPI3 RX/TX 16-bit halfword with MON* shadow readback verification */
    dma_config_chan_verified(DMA1_BASE_ADDR, DMA1_CH0_RX,
                             spi_dr_addr(SPI3_BASE_ADDR), rx_dst_addr,
                             dtctl_val, rx_chctl);
    dma_config_chan_verified(DMA1_BASE_ADDR, DMA1_CH1_TX,
                             (uint32_t)(uintptr_t)tx_src, spi_dr_addr(SPI3_BASE_ADDR),
                             dtctl_val, DMA_CHCTL_SPI3_TX_INC);

    cpu_dmb();
    mmio_w32(dma_chen_addr(DMA1_BASE_ADDR), DMA1_BOTH_CHEN_MASK);
    mmio_w32(spi_cr1_addr(SPI3_BASE_ADDR), SPI3_CR1_RUNNING);
}

BORYOZA_RAMFUNC static void spi3_arm_cmd_hdr_ramfunc(uint64_t busy_since_ticks,
                                                     const uint32_t *min_hold_ptr) {
    for (uint32_t idx = 0U; idx < (uint32_t)Z_CMD_HDR_BYTES; ++idx) {
        s_spi3_tx_buf[idx] = 0x00U;
        s_spi3_rx_buf[idx] = 0x00U;
    }
    spi3_arm_window_ramfunc(s_spi3_tx_buf, false, HDR_HALFWORDS);
    s_spi3_phase = SPI3_WIN_CMD_HDR;
    s_phase2_pdown = 0U;

    uint32_t min_hold_ticks = *min_hold_ptr;
    while ((tmra1_now_ticks() - busy_since_ticks) < (uint64_t)min_hold_ticks) {
        cpu_nop();
    }
    gpio_pa9_set_ready();
}

void z_spi3_slave_init(void) {
    /* Keep PA9 LOW at cold init until first SPI3 command window is armed */
    s_spi3_phase = SPI3_WIN_STOPPED;
    s_spi3_win_count = 0U;
    s_sck_high_seen = 0U;
    s_sck_high_t32 = 0U;
    s_phase2_t32 = 0U;
    s_last_done_ticks = 0ULL;
    s_cmd51_tx_stat = 0U;
    s_cmd51_ready_ok = 0U;
    s_phase2_pdown = 0U;
    s_identity_ready_at_reply = 0U;
    s_optics_ready_at_reply = 0U;
    s_provisional_identity_seen = false;
    s_hdr_epoch = 0UL;
    s_hdr_snap_sig = 0UL;
    s_hdr_opt_gen = 0UL;
    spi3_reset_cmd20_rate_observer(0UL);

    mmio_w32(spi_cr1_addr(SPI3_BASE_ADDR), SPI3_CR1_STOPPED);
    mmio_w32(spi_cfg1_addr(SPI3_BASE_ADDR), SPI3_CFG1_VAL);
    mmio_w32(spi_cfg2_addr(SPI3_BASE_ADDR), SPI3_CFG2_16B_LSBF_MODE3);

    mmio_w32(dma_en_addr(DMA1_BASE_ADDR), 1UL);
    mmio_w32(dma_chen_addr(DMA1_BASE_ADDR), 0UL);
    mmio_w32(dma_intmask1_addr(DMA1_BASE_ADDR), DMA_MASK_ALL_IRQS);
    mmio_w32(dma_intclr1_addr(DMA1_BASE_ADDR), DMA1_BOTH_CLR_MASK);

    mmio_w32(AOS_DMA1_TRGSEL0_ADDR, EVT_SRC_SPI3_SPRI);
    mmio_w32(AOS_DMA1_TRGSEL1_ADDR, EVT_SRC_SPI3_SPTI);
}

static void spi3_stop_dma_and_reset_flags(bool reset_sck_seen) {
    mmio_w32(spi_cr1_addr(SPI3_BASE_ADDR), SPI3_CR1_STOPPED);
    mmio_w32(dma_chen_addr(DMA1_BASE_ADDR), 0UL);
    mmio_w32(dma_intclr1_addr(DMA1_BASE_ADDR), DMA1_BOTH_CLR_MASK);
    s_spi3_phase = SPI3_WIN_STOPPED;
    s_cmd51_tx_stat = 0U;
    s_phase2_pdown = 0U;
    if (reset_sck_seen) {
        s_sck_high_seen = 0U;
        s_sck_high_t32 = 0U;
        s_phase2_t32 = 0U;
        s_identity_ready_at_reply = 0U;
        s_optics_ready_at_reply = 0U;
        s_provisional_identity_seen = false;
        spi3_reset_cmd20_rate_observer(0UL);
    }
}

void z_spi3_slave_reset_to_wait_sck(void) {
    gpio_pa9_set_busy();
    spi3_stop_dma_and_reset_flags(true);
    gpio_pads_release_z_bus();
}

void z_spi3_slave_quiesce_powerdown(void) {
    spi3_stop_dma_and_reset_flags(true);
    gpio_pads_quiesce_z_powerdown();
}

void z_spi3_slave_quiesce_mid_phase2(void) {
    mmio_w32(spi_cr1_addr(SPI3_BASE_ADDR), SPI3_CR1_STOPPED);
    gpio_pads_quiesce_z_powerdown();
    s_phase2_pdown = 1U;
}

bool z_spi3_slave_try_open_first_window(uint64_t now_ticks) {
    uint32_t now32 = (uint32_t)now_ticks;
    /* Do not bind/enable SPI3 while PB14 SCK is low */
    if (gpio_pb14_sck_high() == 0U) {
        s_sck_high_seen = 0U;
        return false;
    }
    if (s_sck_high_seen == 0U) {
        s_sck_high_seen = 1U;
        s_sck_high_t32 = now32;
        return false;
    }
    if ((now32 - s_sck_high_t32) < SCK_STABLE_HIGH_TICKS) {
        return false;
    }
    gpio_pads_bind_z_bus();
    gpio_pa9_set_busy();
    uint64_t pulse_start = tmra1_now_ticks();
    spi3_arm_cmd_hdr_ramfunc(pulse_start, &PA9_FIRST_WIN_HOLD_TICKS);
    return true;
}

void z_spi3_slave_stop_and_busy(void) {
    gpio_pa9_set_busy();
    spi3_stop_dma_and_reset_flags(false);
}

static bool spi3_phase2_mosi_valid(uint8_t cmd, const uint8_t *mosi, size_t mosi_len) {
    (void)cmd;
    (void)mosi;
    /* Accept all completed Phase-2 windows including asymmetric payload lengths */
    return mosi_len >= 2U;
}

static void spi3_consume_phase2_mosi(adapter_ctx_t *ctx, const optics_workspace_t *optics,
                                     uint64_t now_ticks, const uint8_t *mosi, size_t mosi_len) {
    af_servo_note_wall_ticks(ctx, now_ticks);
    uint8_t cmd = s_cur_cmd;
    uint8_t sub = s_cur_sub;
    if (!spi3_phase2_mosi_valid(cmd, mosi, mosi_len)) {
        return;
    }

    if ((cmd == Z_CMD_STATUS_20) && (sub == 0x00U)) {
        uint8_t prev_hb = ctx->z_ring_heartbeat;
        uint8_t prev_b18 = ctx->vr.e_id03_b18;
        uint8_t prev_df = ctx->vr.z_df_state;
        aperture_controls_on_z_mosi20(ctx, mosi, mosi_len, &now_ticks);
        if ((ctx->z_ring_heartbeat != prev_hb) ||
            (ctx->vr.e_id03_b18 != prev_b18) ||
            (ctx->vr.z_df_state != prev_df)) {
            z_snapshot_publish(ctx, optics);
        }
    } else if (((cmd == Z_CMD_APERTURE_A0) && (sub <= Z_A0_MAX_SUB)) ||
               ((cmd == Z_CMD_APERTURE_A1) && (sub == 0x00U)) ||
               ((cmd == Z_CMD_APERTURE_A3) && (sub <= 0x01U))) {
        aperture_controls_on_z_a1(ctx, mosi, mosi_len);
        z_snapshot_publish(ctx, optics);
    } else if ((cmd == Z_CMD_HOTLINE_CTL) || (cmd == Z_CMD_PARK_50) ||
               ((cmd >= Z_CMD_VR_D2) && (cmd <= Z_CMD_VR_D8))) {
        if (cmd == Z_CMD_PARK_50) {
            s_cmd51_ready_ok = 0U;
            z_spi1_hotline_stop();
            z_spi1_hotline_reset_seq();
            ctx->vr.hotline_seq = 0U;
        }
        vr_controller_on_z_command(ctx, cmd, mosi, mosi_len, &now_ticks);
        z_snapshot_publish(ctx, optics);
    } else if (((cmd >= Z_CMD_AF_C0) && (cmd <= Z_CMD_AF_C4)) ||
               ((cmd >= Z_CMD_AF_E0) && (cmd <= Z_CMD_AF_EF))) {
        int32_t prev_lz = ctx->af.mech_lz;
        int32_t prev_hz = ctx->af.mech_hz;
        int32_t prev_cz = ctx->af.cur_cz;
        af_servo_on_z_command(ctx, cmd, mosi, mosi_len);
        if ((ctx->af.mech_lz != prev_lz) || (ctx->af.mech_hz != prev_hz) ||
            (ctx->af.cur_cz != prev_cz)) {
            z_snapshot_publish(ctx, optics);
        }
    }
}

static void spi3_note_phase2_done(uint64_t busy_ticks) {
    s_last_done_ticks = busy_ticks;
    if ((s_cur_cmd == Z_CMD_PARK_STAT_51) && (s_cmd51_tx_stat == Z_CMD51_READY_VAL)) {
        s_cmd51_ready_ok = 1U;
    }
}

static void spi3_note_phase2_tc_delivery(adapter_ctx_t *ctx) {
    enum {
        Z_CMD_BODY_CAP_03   = 0x03U,
        Z_CMD_ID11_VAL      = 0x11U,
        Z_CMD_ID12_VAL      = 0x12U,
        Z_CMD_BODY_CAP_23   = 0x23U,
        Z_CMD_OPTICS_24_VAL = 0x24U,
        Z_CMD_BODY_CAP_28   = 0x28U,
        Z_CMD_BODY_CAP_31   = 0x31U,
        Z_CMD_BODY_CAP_39   = 0x39U,
        Z_CMD_AF_E8_VAL     = 0xE8U,
        Z_CMD24_SUB_A0      = 0x00U,
        Z_CMD24_SUB_A2      = 0x02U,
        Z_CMD24_SUB_B0      = 0x10U,
        Z_CMD24_SUB_B2      = 0x12U,
    };
    typedef struct {
        uint8_t  cmd;
        uint32_t bit;
    } z_body_cap_map_t;
    static const z_body_cap_map_t k_body_cap_map[] = {
        { Z_CMD_BODY_CAP_03,   (uint32_t)BODY_CAP_CMD03_BIT },
        { Z_CMD_BODY_CAP_23,   (uint32_t)BODY_CAP_CMD23_BIT },
        { Z_CMD_OPTICS_24_VAL, (uint32_t)BODY_CAP_CMD24_BIT },
        { Z_CMD_BODY_CAP_28,   (uint32_t)BODY_CAP_CMD28_BIT },
        { Z_CMD_BODY_CAP_31,   (uint32_t)BODY_CAP_CMD31_BIT },
        { Z_CMD_BODY_CAP_39,   (uint32_t)BODY_CAP_CMD39_BIT },
        { Z_CMD_AF_E8_VAL,     (uint32_t)BODY_CAP_CMDE8_BIT },
    };
    for (size_t idx = 0U; idx < (sizeof(k_body_cap_map) / sizeof(k_body_cap_map[0])); ++idx) {
        if (s_cur_cmd == k_body_cap_map[idx].cmd) {
            ctx->v224.body_cap_flags |= k_body_cap_map[idx].bit;
            break;
        }
    }

    if ((((s_cur_cmd == Z_CMD_LENS_IDENTITY) && (s_cur_sub == 0U) &&
          (s_cur_data_bytes >= Z_CMD10_BYTES)) ||
         ((s_cur_cmd == Z_CMD_ID11_VAL) && (s_cur_sub == 0U) &&
          (s_cur_data_bytes >= Z_CMD11_BYTES)) ||
         ((s_cur_cmd == Z_CMD_ID12_VAL) && (s_cur_sub == 0U) &&
          (s_cur_data_bytes >= Z_CMD12_BYTES))) &&
        (s_hdr_snap_sig != 0UL)) {
        ctx->v224.delivered_snap_sig = s_hdr_snap_sig;
    }

    bool is_cmd24_a = (bool)(((s_cur_sub == Z_CMD24_SUB_A0) || (s_cur_sub == Z_CMD24_SUB_A2)) &&
                             (s_cur_data_bytes == (uint16_t)Z_CMD24_A_BYTES));
    bool is_cmd24_b = (bool)(((s_cur_sub == Z_CMD24_SUB_B0) || (s_cur_sub == Z_CMD24_SUB_B2)) &&
                             (s_cur_data_bytes == (uint16_t)Z_CMD24_B_BYTES));
    if ((s_cur_cmd == Z_CMD_OPTICS_24) && (is_cmd24_a || is_cmd24_b) && (s_hdr_opt_gen != 0UL)) {
        ctx->v224.phase2_tc_epoch = s_hdr_epoch;
        ctx->v224.phase2_tc_opt_gen = s_hdr_opt_gen;
        if ((ctx->v224.delivered_opt_epoch != s_hdr_epoch) ||
            (ctx->v224.delivered_opt_gen == 0UL) ||
            (s_hdr_opt_gen < ctx->v224.delivered_opt_gen)) {
            ctx->v224.delivered_opt_epoch = s_hdr_epoch;
            ctx->v224.delivered_opt_gen = s_hdr_opt_gen;
        }
    }
}

static void spi3_record_provisional_if_unready(void) {
    bool unready_id10 = (bool)((s_cur_cmd == Z_CMD_LENS_IDENTITY) && (s_cur_sub == 0U) &&
                               (s_cur_data_bytes >= Z_CMD10_BYTES) &&
                               ((s_identity_ready_at_reply == 0U) ||
                                (s_optics_ready_at_reply == 0U)));
    bool unready_opt24 = (bool)((s_cur_cmd == Z_CMD_OPTICS_24) &&
                                (s_optics_ready_at_reply == 0U));
    if (unready_id10 || unready_opt24) {
        s_provisional_identity_seen = true;
    }
}

/* NOLINTBEGIN(readability-magic-numbers) */
static void note_z_hdr_err(adapter_ctx_t *ctx, uint32_t err_tag) {
    if (ctx->diag_z_hdr_err_count < 0xFFFFU) {
        ctx->diag_z_hdr_err_count = (uint16_t)(ctx->diag_z_hdr_err_count + 1U);
    }
    if (ctx->bb_z_err_logged == 0U) {
        ctx->bb_z_err_logged = 1U;
        uint32_t hdr0 = (uint32_t)s_spi3_rx_buf[0] |
                        ((uint32_t)s_spi3_rx_buf[1] << 8U) |
                        ((uint32_t)s_spi3_rx_buf[2] << 16U) |
                        ((uint32_t)s_spi3_rx_buf[3] << 24U);
        uint32_t hdr1 = (uint32_t)s_spi3_rx_buf[4] |
                        ((uint32_t)s_spi3_rx_buf[5] << 8U) |
                        ((uint32_t)s_spi3_rx_buf[6] << 16U) |
                        ((uint32_t)s_spi3_rx_buf[7] << 24U);
        uint32_t info = (err_tag & 0xFFUL) | ((uint32_t)s_cur_cmd << 8U) |
                        ((uint32_t)s_spi3_phase << 16U);
        lens_session_push_diag(ctx, BB_EVT_Z_HDR_CSUM_ERR, hdr0, hdr1, info, s_spi3_win_count);
    }
}

uint8_t z_spi3_slave_poll(adapter_ctx_t *ctx, const optics_workspace_t *optics,
                         uint64_t now_ticks) {
    if (s_spi3_phase == SPI3_WIN_STOPPED) {
        return 0U;
    }

    uint32_t stat = mmio_r32(dma_intstat1_addr(DMA1_BASE_ADDR));
    if ((stat & DMA1_BOTH_TC_MASK) != DMA1_BOTH_TC_MASK) {
        if ((s_spi3_phase == SPI3_WIN_DATA_PAY) &&
            (((uint32_t)now_ticks - s_phase2_t32) > PHASE2_STUCK_MAX_TICKS) &&
            (gpio_pb14_sck_high() != 0U)) {
            gpio_pa9_set_busy();
            note_z_hdr_err(ctx, 1UL);
            uint64_t abort_ticks = tmra1_now_ticks();
            spi3_arm_cmd_hdr_ramfunc(abort_ticks, &PA9_MIN_BUSY_TICKS);
            return POLL_EVT_HDR_ERR;
        }
        return 0U;
    }

    gpio_pa9_set_busy();
    uint64_t busy_ticks = tmra1_now_ticks();
    mmio_w32(spi_cr1_addr(SPI3_BASE_ADDR), SPI3_CR1_STOPPED);
    mmio_w32(dma_chen_addr(DMA1_BASE_ADDR), 0UL);
    mmio_w32(dma_intclr1_addr(DMA1_BASE_ADDR), DMA1_BOTH_CLR_MASK);

    if (s_spi3_phase == SPI3_WIN_CMD_HDR) {
        uint32_t hdr_sum = 0U;
        for (uint32_t idx = 0U; idx < (uint32_t)Z_CMD_HDR_BYTES; ++idx) {
            hdr_sum += s_spi3_rx_buf[idx];
        }
        if ((s_spi3_rx_buf[0] != Z_SYNC_BYTE) || ((hdr_sum & BYTE_MASK) != 0U)) {
            note_z_hdr_err(ctx, 2UL);
            spi3_arm_cmd_hdr_ramfunc(busy_ticks, &PA9_MIN_BUSY_TICKS);
            return POLL_EVT_HDR_ERR;
        }

        uint8_t cmd = s_spi3_rx_buf[HDR_OFF_CMD];
        uint8_t sub = s_spi3_rx_buf[HDR_OFF_SUB];
        uint16_t req_words = (uint16_t)((uint32_t)s_spi3_rx_buf[HDR_OFF_LEN_LO] |
                                        ((uint32_t)s_spi3_rx_buf[HDR_OFF_LEN_HI] << SHIFT_8));
        uint32_t halfwords = (uint32_t)req_words + 1UL;
        if ((halfwords == 0UL) || (halfwords > MAX_WIN_HALFWORDS)) {
            note_z_hdr_err(ctx, 3UL);
            spi3_arm_cmd_hdr_ramfunc(busy_ticks, &PA9_MIN_BUSY_TICKS);
            return POLL_EVT_HDR_ERR;
        }
        uint32_t data_bytes = halfwords * 2UL;

        s_cur_cmd = cmd;
        s_cur_sub = sub;
        s_cur_data_bytes = (uint16_t)data_bytes;
        s_identity_ready_at_reply = ctx->identity_ready;
        s_optics_ready_at_reply = ctx->optics_ready;
        s_hdr_epoch = ctx->session_epoch;
        s_hdr_snap_sig = z_snapshot_compute_sig(ctx);
        s_hdr_opt_gen = ctx->v224.current_opt_gen;

        z_cmd_req_t req = {
            .cmd = cmd,
            .sub = sub,
            .req_words = req_words,
            .mosi_buf = NULL,
            .mosi_len = 0U,
            .now_ticks = now_ticks,
        };
        const uint8_t *direct_ptr = NULL;
        (void)z_dispatch_command_ram_safe(ctx, optics, &req, s_spi3_tx_buf, &direct_ptr);

        const uint8_t *tx_src = (direct_ptr != NULL) ? direct_ptr : s_spi3_tx_buf;
        s_cmd51_tx_stat = (data_bytes >= 4UL) ? tx_src[1] : 0U;
        /* MANDATORY N>512 stack/SRAMH overflow guard: route MOSI to DINC=0 trash sink */
        bool use_sink = (data_bytes > SMALL_WIN_MAX_BYTES);

        spi3_arm_window_ramfunc(tx_src, use_sink, halfwords);
        s_spi3_phase = SPI3_WIN_DATA_PAY;
        s_phase2_t32 = (uint32_t)busy_ticks;

        while ((tmra1_now_ticks() - busy_ticks) < (uint64_t)PA9_MIN_BUSY_TICKS) {
            cpu_nop();
        }
        gpio_pa9_set_ready();
        return POLL_EVT_HDR_OK;
    }

    /* Phase 2 completed: parse any host MOSI payload, log 1st window WHILE PA9==1 Busy, then re-open */
    if ((uint32_t)s_cur_data_bytes <= SMALL_WIN_MAX_BYTES) {
        spi3_consume_phase2_mosi(ctx, optics, now_ticks, s_spi3_rx_buf, s_cur_data_bytes);
    }
    if ((ctx->session_state == E_SESSION_ACTIVE) && (ctx->identity_ready != 0U) &&
        (s_cur_cmd == Z_CMD_STATUS_20) && (s_cur_sub == 0x00U) &&
        (s_cur_data_bytes == (uint16_t)Z_CMD20_00_BYTES)) {
        spi3_rate_sample_t sample = {
            .session_epoch = ctx->session_epoch,
            .done_ticks = busy_ticks,
        };
        spi3_observe_cmd20_rate(&sample);
    }
    spi3_note_phase2_done(busy_ticks);
    s_spi3_win_count += 1U;
    spi3_note_phase2_tc_delivery(ctx);
    spi3_record_provisional_if_unready();
    if (s_first_win_bb == 0U) {
        s_first_win_bb = 1U;
        bb_event_input_t evt = {
            .event_code = BB_EVT_Z_FIRST_WIN,
            .lens_id = ctx->e_lens_id,
            .arg0 = s_spi3_win_count,
            .arg1 = ctx->z_primary_lens_id,
            .arg2 = ctx->optics_ready,
            .arg3 = ctx->identity_ready,
        };
        (void)blackbox_record_event(&evt);
    }
    spi3_arm_cmd_hdr_ramfunc(busy_ticks, &PA9_MIN_BUSY_TICKS);
    return POLL_EVT_DATA_OK;
}
/* NOLINTEND(readability-magic-numbers) */

uint32_t z_spi3_slave_rx_Window_count(void) {
    return s_spi3_win_count;
}

bool z_spi3_slave_provisional_identity_seen(void) {
    return s_provisional_identity_seen;
}

void z_spi3_slave_clear_provisional_seen(void) {
    s_provisional_identity_seen = false;
}

bool z_spi3_slave_is_idle_hdr(void) {
    return (bool)((s_spi3_phase == SPI3_WIN_CMD_HDR) &&
                  ((mmio_r32(dma_intstat1_addr(DMA1_BASE_ADDR)) & DMA1_BOTH_TC_MASK) == 0UL));
}

bool z_spi3_slave_phase2_busy(void) {
    return (bool)(s_spi3_phase == SPI3_WIN_DATA_PAY);
}

bool z_spi3_slave_phase2_powerdown_seen(void) {
    return (bool)(s_phase2_pdown != 0U);
}

bool z_spi3_slave_cmd51_ready_delivered(void) {
    return (bool)(s_cmd51_ready_ok != 0U);
}

void z_spi3_slave_clear_cmd51_ready(void) {
    s_cmd51_ready_ok = 0U;
}

uint32_t z_spi3_slave_idle_ticks(uint64_t now_ticks) {
    static const uint64_t U32_IDLE_MAX = 0xFFFFFFFFULL;
    if (now_ticks <= s_last_done_ticks) {
        return 0U;
    }
    uint64_t elapsed = now_ticks - s_last_done_ticks;
    return (elapsed > U32_IDLE_MAX) ? UINT32_MAX : (uint32_t)elapsed;
}

uint8_t z_spi3_slave_poll_cmd20_rate_target(const adapter_ctx_t *ctx, uint64_t now_ticks) {
    uint32_t session_epoch = (ctx != NULL) ? ctx->session_epoch : 0U;
    if (s_z20_obs_epoch != session_epoch) {
        spi3_reset_cmd20_rate_observer(session_epoch);
        return 0U;
    }
    if ((s_last_z20_full_ticks != 0ULL) && (now_ticks > s_last_z20_full_ticks) &&
        ((now_ticks - s_last_z20_full_ticks) > RATE_OBS_GAP_MAX_TKS)) {
        s_z20_fast_streak = 0U;
        s_z20_slow_streak = 0U;
        if (s_z20_rate_decided != 0U) {
            s_z20_target_hz = (uint8_t)RATE_OBS_HZ_60;
        }
    }
    return (s_z20_rate_decided != 0U) ? s_z20_target_hz : 0U;
}
