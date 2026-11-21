#include "../include/hc32f460_regs.h"
#include "../include/boryoza_math_mem.h"

enum {
    LOCAL_ID03_WIRE_CAP = 72,
};

static const uint32_t COLD_PA3_SAMPLE_COUNT = 5UL;
static const uint32_t COLD_PA3_SAMPLE_US    = 10UL;      /* 50 us glitch filter -> <1.5ms to SPI3 arm */
static const uint64_t HOTPLUG_POLL_TICKS    = 15000ULL;    /* 5.000 ms @ 3.000 MHz */
static const uint64_t DETACH_REOPEN_TICKS    = 1350000ULL;  /* 450.0 ms @ 3.000 MHz */
static const uint64_t Z8_STATE03_GUARD_TKS   = 390000ULL;   /* 130.0 ms @ 3.000 MHz (<165 ms from POR) */
static const uint64_t LIVE_OPTICS_WAIT_TKS   = 252000ULL;   /* 84.0 ms max multi-frame curve wait */
static const uint64_t WAKE_FALLBACK_TKS      = 450000ULL;
static const uint64_t PASSIVE_FALLBACK_TKS   = 1080000ULL;
static const uint64_t WAKE_ZERO_HOLD_TKS     = 720000ULL;
static const uint64_t ACTIVE_HS_MAX_TICKS   = 19500000ULL;  /* 6500 ms active homing handshake budget */
static const uint16_t ID03_PHASE_60HZ_TKS   = 22500U;      /* 7.50 ms post-VSync phase for ID03 @ 60Hz */
static const uint16_t ID03_MAX_60HZ_TKS     = 40500U;      /* 13.50 ms latest launch for ID03 @ 60Hz */
static const uint16_t ID04_PHASE_60HZ_TKS   = 27000U;      /* 9.00 ms post-VSync phase for ID04 @ 60Hz */
static const uint16_t ID04_MAX_60HZ_TKS     = 46500U;      /* 15.50 ms latest launch for ID04 @ 60Hz */
static const uint64_t ID04_GAP_60HZ_TKS     = 3000ULL;     /* 1.00 ms min gap after ID03 @ 60Hz */
static const uint16_t ID03_PHASE_120HZ_TKS  = 9000U;       /* 3.00 ms post-VSync phase for ID03 @ 120Hz */
static const uint16_t ID03_MAX_120HZ_TKS    = 19200U;      /* 6.40 ms latest launch for ID03 @ 120Hz */
static const uint16_t ID04_PHASE_120HZ_TKS  = 12000U;      /* 4.00 ms post-VSync phase for ID04 @ 120Hz */
static const uint16_t ID04_MAX_120HZ_TKS    = 22800U;      /* 7.60 ms latest launch for ID04 @ 120Hz */
static const uint64_t ID04_GAP_120HZ_TKS    = 2400ULL;     /* 0.80 ms min gap after ID03 @ 120Hz */
static const uint32_t BAUD_750K_INIT        = 750000UL;
static const uint32_t BAUD_1500K_FAST       = 1500000UL;
static const uint8_t  VSYNC_60HZ_NOMINAL    = 60U;
static const uint8_t  VSYNC_120HZ_FAST      = 120U;
static const uint64_t E_PAIR_LOSS_80MS_TKS  = 240000ULL;
static const uint8_t  E_PAIR_ERR_STREAK_MAX = 3U;
static const uint16_t VD_COMMIT_PHASE_MAX_TKS = 3000U;
static const uint32_t PARK_POST_51_IDLE_TKS = 6000UL;
static const uint32_t WAKE_GUARD_QUIET_TKS  = 6000UL;
static const uint64_t PARK_SCK_LOW_TKS      = 4500ULL;
static const uint64_t SCK_POWERDOWN_LOW_TKS = 15000ULL;
static const uint64_t PARK_COMMIT_MAX_TKS   = 180000ULL;
static const uint64_t SCK_AUTO_PARK_LOW_TKS = 180000ULL;
static const uint64_t WAKE_COOLDOWN_TKS     = 1500000ULL;
static const uint8_t  WAKE_REQ_PASSIVE_POLL   = 1U;
static const uint8_t  WAKE_REQ_ACTIVE_CMD     = 2U;
static const uint8_t  MAX_WAKE_RECOVERY_HOLDS = 3U;
static const uint8_t  Z_CMD_PARK_AUTO_50    = 0x50U;
static const uint32_t PACK_SHIFT_8          = 8U;
static const uint32_t PACK_SHIFT_16         = 16U;
static const uint32_t PACK_SHIFT_17         = 17U;
static const uint32_t PACK_SHIFT_24         = 24U;
static const uint32_t MASK_U16_LO           = 0xFFFFUL;
static const uint32_t READY_TICKS32_CAP     = 0x0FFFFFFFUL;
static const uint32_t TENTHS_PER_MS         = 10UL;
static const uint32_t U16_MAX_BOUND         = 65535UL;

typedef struct {
    uint32_t applied_baud;
    uint8_t  applied_vsync_hz;
    bool     z_want_open;
    bool     z_window_opened;
    bool     z_ever_opened;
    bool     cold_z_opened_once;
    bool     pending_optics_reopen;
    bool     startup_repulse_done;
    bool     reopen_active;
    bool     e_link_logged;
    bool     park_committed;
    bool     post51_guard_armed;
    bool     wake_timeout_logged;
    uint8_t  wake_recovery_attempts;
    bool     sck_power_cycled_low;
    bool     wake_after_sck_cycle;
    bool     detach_hold;
    bool     vd_pending;
    bool     id04_pending;
    uint32_t last_vd_seen;
    uint64_t id03_sent_ticks;
    uint64_t last_hotplug_ticks;
    uint64_t detach_ticks;
    uint64_t mount_boot_ticks;
    uint64_t park_ready_ticks;
    uint64_t sck_low_since_ticks;
    uint64_t wake_cooldown_until_ticks;
    uint64_t wake_hold_zero_until_ticks;
    uint32_t tracked_epoch;
} loop_tracker_t;

static adapter_ctx_t                        g_adapter_ctx;
BORYOZA_SRAM2_BUF static optics_workspace_t g_optics_ws;

static void record_bb4(uint16_t code, uint16_t lens_id,
                       uint32_t arg0, uint32_t arg1, uint32_t arg2, uint32_t arg3) {
    bb_event_input_t evt = {
        .event_code = code,
        .lens_id = lens_id,
        .arg0 = arg0,
        .arg1 = arg1,
        .arg2 = arg2,
        .arg3 = arg3,
    };
    (void)blackbox_record_event(&evt);
}

static void record_bb(uint16_t code, uint16_t lens_id,
                      const uint32_t *arg0_opt, uint32_t arg1, uint32_t arg2) {
    record_bb4(code, lens_id, (arg0_opt != NULL) ? *arg0_opt : 0U, arg1, arg2, 0U);
}

static void drain_core_diag_to_blackbox(void) {
    bb_event_input_t evt;
    while (lens_session_pop_diag(&g_adapter_ctx, &evt)) {
        (void)blackbox_record_event(&evt);
    }
}

static int16_t clamp_s32_to_i16(int32_t val) {
    enum {
        I16_ABS_MAX = 32767,
    };
    if (val > I16_ABS_MAX) {
        return (int16_t)I16_ABS_MAX;
    }
    if (val < -I16_ABS_MAX) {
        return (int16_t)(-I16_ABS_MAX);
    }
    return (int16_t)val;
}

static void record_session_summary_bb(const loop_tracker_t *trk) {
    uint32_t sum_a0 = ((uint32_t)g_adapter_ctx.diag_af_cmd_count) |
                      ((uint32_t)g_adapter_ctx.diag_af_stall_count << PACK_SHIFT_16);
    uint32_t sum_a1 = (e_usart1_dma_rx_error_count() & MASK_U16_LO) |
                      ((uint32_t)g_adapter_ctx.diag_z_hdr_err_count << PACK_SHIFT_16);
    uint32_t sum_a2 = ((uint32_t)g_adapter_ctx.f_cur_mm) |
                      ((uint32_t)g_adapter_ctx.optics_disconnected << PACK_SHIFT_16) |
                      ((uint32_t)g_adapter_ctx.dist_report_invalid << PACK_SHIFT_24);
    uint32_t sum_a3 = ((uint32_t)g_adapter_ctx.id05.dist_code_x) |
                      ((uint32_t)trk->applied_vsync_hz << PACK_SHIFT_16) |
                      ((uint32_t)g_adapter_ctx.home_completed_ok << PACK_SHIFT_24);
    record_bb4(BB_EVT_SESSION_SUMMARY, g_adapter_ctx.e_lens_id,
               sum_a0, sum_a1, sum_a2, sum_a3);

    uint32_t perf_a0 = ((uint32_t)g_adapter_ctx.ap.z_dist_0x117) |
                       ((uint32_t)(uint16_t)g_adapter_ctx.dist_cam_bias_117 << PACK_SHIFT_16);
    uint32_t perf_a1 = ((uint32_t)g_adapter_ctx.ap.z_fnorm_0x33) |
                       ((uint32_t)g_adapter_ctx.ap.z_dnorm_0x31 << PACK_SHIFT_16);
    uint32_t perf_a2 = ((uint32_t)g_adapter_ctx.ap.z_eff_focal_q4) |
                       ((uint32_t)g_adapter_ctx.ap.z_mag_a3_q14 << PACK_SHIFT_16);
    uint32_t perf_a3 = ((uint32_t)g_adapter_ctx.first_ready_ms_x10) |
                       ((uint32_t)(g_adapter_ctx.retram_warm_hit & 1U) << PACK_SHIFT_16) |
                       ((uint32_t)(g_adapter_ctx.ds13_cache_hit & 1U) << PACK_SHIFT_17);
    record_bb4(BB_EVT_PERF_OPTICS, g_adapter_ctx.e_lens_id,
               perf_a0, perf_a1, perf_a2, perf_a3);

    if (g_adapter_ctx.v224.v224_bb_v2_parked_done == 0U) {
        g_adapter_ctx.v224.v224_bb_v2_parked_done = 1U;
        v224_trace_counts_t trace_cnt = {0};
        v224_trace_get_counts(&trace_cnt);
        uint32_t af_a0 = ((uint32_t)g_adapter_ctx.v224.txn_owner_cmd) |
                         ((uint32_t)g_adapter_ctx.v224.latched_abort_submode << PACK_SHIFT_8) |
                         ((uint32_t)g_adapter_ctx.v224.landing_redrive_count << PACK_SHIFT_16) |
                         ((uint32_t)g_adapter_ctx.v224.cdaf_min_pulse_se << PACK_SHIFT_24);
        int32_t res_se = g_adapter_ctx.af.target_se - g_adapter_ctx.id06.focus_pos_se;
        uint32_t af_a1 = ((uint32_t)(uint16_t)clamp_s32_to_i16(res_se)) |
                         ((uint32_t)g_adapter_ctx.v224.af_stall_abort_total << PACK_SHIFT_16);
        record_bb4(BB_EVT_V2_AF_SUMMARY, g_adapter_ctx.e_lens_id,
                   af_a0, af_a1, g_adapter_ctx.v224.txn_seq, (uint32_t)trace_cnt.af_cnt);

        uint32_t pf_a0 = ((uint32_t)(uint16_t)(int16_t)g_adapter_ctx.parfocal_cl_trim_cz) |
                         ((uint32_t)g_adapter_ctx.v224.parfocal_diverge_count << PACK_SHIFT_16) |
                         ((uint32_t)g_adapter_ctx.v224.parfocal_fuse_latched << PACK_SHIFT_24);
        record_bb4(BB_EVT_V2_PF_SUMMARY, g_adapter_ctx.e_lens_id,
                   pf_a0, (uint32_t)g_adapter_ctx.v224.anchor_x_q8,
                   (uint32_t)trace_cnt.pf_cnt, (uint32_t)g_adapter_ctx.parfocal_anchor_dist_x);

        static const uint32_t BB_DIST_SUMMARY_TAG_HI = 0x53550000UL;
        uint32_t ds_a0 = ((uint32_t)g_adapter_ctx.v224.dist_state) |
                         ((uint32_t)g_adapter_ctx.v224.dist_good_streak << PACK_SHIFT_8) |
                         ((uint32_t)g_adapter_ctx.dist_report_invalid << PACK_SHIFT_16) |
                         ((uint32_t)g_adapter_ctx.v224.z_advertise_fx << PACK_SHIFT_24);
        record_bb4(BB_EVT_V2_DIST_SUMMARY, g_adapter_ctx.e_lens_id,
                   ds_a0, (uint32_t)g_adapter_ctx.v224.dist_last_raw_x,
                   BB_DIST_SUMMARY_TAG_HI | (uint32_t)g_adapter_ctx.v224.dist_last_valid_raw_x,
                   (uint32_t)g_adapter_ctx.ap.z_dist_0x117);

        uint32_t frz_a0 = ((uint32_t)trace_cnt.freeze_cnt) |
                          ((uint32_t)trace_cnt.freeze_reason << PACK_SHIFT_16) |
                          ((uint32_t)(uint8_t)trace_cnt.freeze_pre_cnt << PACK_SHIFT_24);
        record_bb4(BB_EVT_V2_FREEZE_META, g_adapter_ctx.e_lens_id,
                   frz_a0, g_adapter_ctx.v224.body_cap_flags,
                   g_adapter_ctx.v224.delivered_opt_gen,
                   g_adapter_ctx.v224.current_opt_gen);

        record_bb4(BB_EVT_V2_BODY_ID, g_adapter_ctx.e_lens_id,
                   g_adapter_ctx.v224.body_cap_flags, g_adapter_ctx.v224.delivered_snap_sig,
                   MASK_U16_LO,
                   g_adapter_ctx.v224.phase2_tc_opt_gen);
    }
}

static bool sample_cold_pa3_mounted(void) {
    uint32_t mounted_votes = 0U;
    for (uint32_t idx = 0U; idx < COLD_PA3_SAMPLE_COUNT; ++idx) {
        if (gpio_pa3_lens_mounted() != 0U) {
            mounted_votes += 1U;
        }
        tmra1_delay_us(COLD_PA3_SAMPLE_US);
    }
    return mounted_votes >= (COLD_PA3_SAMPLE_COUNT / 2UL);
}

static void request_z_bus_open(loop_tracker_t *trk) {
    trk->z_want_open = true;
    trk->cold_z_opened_once = true;
}

static uint64_t update_sck_low_elapsed(loop_tracker_t *trk, uint64_t now_ticks) {
    if (gpio_pb14_sck_high() != 0U) {
        if (trk->sck_power_cycled_low) {
            trk->sck_power_cycled_low = false;
            trk->wake_after_sck_cycle = true;
            trk->wake_cooldown_until_ticks = 0ULL;
        }
        trk->sck_low_since_ticks = 0ULL;
        return 0ULL;
    }
    if (trk->sck_low_since_ticks == 0ULL) {
        trk->wake_after_sck_cycle = false;
        trk->sck_low_since_ticks = (now_ticks != 0ULL) ? now_ticks : 1ULL;
        return 0ULL;
    }
    uint64_t elapsed = (now_ticks >= trk->sck_low_since_ticks)
                           ? (now_ticks - trk->sck_low_since_ticks)
                           : 0ULL;
    if (elapsed >= SCK_POWERDOWN_LOW_TKS) {
        trk->sck_power_cycled_low = true;
    }
    return elapsed;
}

__attribute__((noinline)) static void quiesce_hotline_on_sck_loss(void) {
    g_adapter_ctx.vr.hotline_enabled = 0U;
    g_adapter_ctx.vr.hotline_seq = 0U;
    g_adapter_ctx.af_ext.hotline_seen = 0U;
    g_adapter_ctx.af_ext.active_step_se = 0;
    g_adapter_ctx.z_sck_powerdown_latched = 1U;
    lens_session_arm_home_retrigger(&g_adapter_ctx);
    z_spi1_hotline_stop();
    z_spi1_hotline_reset_seq();
}

static void service_open_z_window(loop_tracker_t *trk, uint64_t now_ticks,
                                  bool sck_powerdown) {
    if (z_spi3_slave_phase2_powerdown_seen() && (gpio_pb14_sck_high() != 0U)) {
        z_spi3_slave_reset_to_wait_sck();
        trk->z_window_opened = false;
        trk->z_want_open = true;
        return;
    }
    (void)z_spi3_slave_poll(&g_adapter_ctx, &g_optics_ws, now_ticks);
    if (!sck_powerdown) {
        return;
    }
    quiesce_hotline_on_sck_loss();
    if (z_spi3_slave_is_idle_hdr()) {
        z_spi3_slave_quiesce_powerdown();
        trk->z_window_opened = false;
        trk->z_want_open = true;
    } else if (!z_spi3_slave_phase2_powerdown_seen()) {
        z_spi3_slave_quiesce_mid_phase2();
    }
}

static bool is_empty_mount_attach_debouncing(void) {
    return (bool)((g_adapter_ctx.session_state == E_SESSION_EMPTY_MOUNT) &&
                  (gpio_pa3_lens_mounted() != 0U));
}

static void service_z_bus(loop_tracker_t *trk, uint64_t now_ticks) {
    uint64_t sck_low_elapsed = update_sck_low_elapsed(trk, now_ticks);
    if (trk->z_ever_opened && (sck_low_elapsed >= SCK_POWERDOWN_LOW_TKS) &&
        (g_adapter_ctx.z_sck_powerdown_latched == 0U)) {
        quiesce_hotline_on_sck_loss();
    }
    if (trk->z_want_open && (!trk->z_window_opened)) {
        if (is_empty_mount_attach_debouncing()) {
            if (gpio_pb14_sck_high() != 0U) {
                gpio_pa9_set_busy();
            }
        } else if ((sck_low_elapsed >= PARK_SCK_LOW_TKS) &&
                   (!trk->reopen_active) && (!trk->detach_hold)) {
            gpio_pa9_set_ready();
        } else if (z_spi3_slave_try_open_first_window(now_ticks)) {
            trk->z_window_opened = true;
            if (!trk->z_ever_opened) {
                uint32_t ticks32 = (now_ticks > (uint64_t)READY_TICKS32_CAP)
                                       ? READY_TICKS32_CAP
                                       : (uint32_t)now_ticks;
                uint32_t ready_ms_x10 = ticks32 / (HC32_TICKS_PER_MS / TENTHS_PER_MS);
                g_adapter_ctx.first_ready_ms_x10 = (ready_ms_x10 <= U16_MAX_BOUND)
                                                       ? (uint16_t)ready_ms_x10
                                                       : (uint16_t)U16_MAX_BOUND;
            }
            trk->z_ever_opened = true;
        }
    } else if (trk->z_window_opened) {
        service_open_z_window(trk, now_ticks, (bool)(sck_low_elapsed >= SCK_POWERDOWN_LOW_TKS));
    }
    z_spi1_hotline_service(&g_adapter_ctx, now_ticks);
}

static uint64_t current_hs_origin(const loop_tracker_t *trk) {
    if (trk->cold_z_opened_once && (g_adapter_ctx.hs_start_ticks > trk->mount_boot_ticks)) {
        return g_adapter_ctx.hs_start_ticks;
    }
    if (trk->mount_boot_ticks != 0ULL) {
        return trk->mount_boot_ticks;
    }
    if (g_adapter_ctx.hs_start_ticks != 0ULL) {
        return g_adapter_ctx.hs_start_ticks;
    }
    return g_adapter_ctx.state_enter_ticks;
}

static void log_hs_timeout_bb(uint32_t hs_elapsed) {
    uint32_t hs_a0 = ((uint32_t)g_adapter_ctx.hs_step) |
                     ((uint32_t)g_adapter_ctx.cmd01_retries << PACK_SHIFT_8) |
                     ((uint32_t)g_adapter_ctx.hs_cmd07_ok << PACK_SHIFT_16) |
                     ((uint32_t)g_adapter_ctx.home_completed_ok << PACK_SHIFT_24);
    record_bb4(BB_EVT_HS_TIMEOUT_FALLBACK, g_adapter_ctx.e_lens_id,
               hs_a0, hs_elapsed,
               e_usart1_dma_rx_error_count(), g_adapter_ctx.uart_baud);
}

static bool handle_wake_probe_timeout(loop_tracker_t *trk, uint64_t now_ticks) {
    bool active_boot = (bool)((g_adapter_ctx.wake_boot_prologue_seen != 0U) ||
                              (g_adapter_ctx.vr.hotline_enabled != 0U) ||
                              trk->wake_after_sck_cycle);
    if (!active_boot) {
        trk->wake_recovery_attempts = 0U;
        trk->wake_after_sck_cycle = false;
        e_usart1_dma_stop();
        e_tmra4_vsync_stop();
        g_adapter_ctx.wake_rehandshake = 0U;
        g_adapter_ctx.wake_from_park_req = 0U;
        g_adapter_ctx.wake_boot_prologue_seen = 0U;
        g_adapter_ctx.home_retrigger_armed = 0U;
        g_adapter_ctx.cmd15_seen_since_arm = 0U;
        g_adapter_ctx.session_state = E_SESSION_PARKED;
        g_adapter_ctx.hs_step = HANDSHAKE_STEP_READY;
        g_adapter_ctx.state_enter_ticks = now_ticks;
        trk->park_committed = true;
        trk->post51_guard_armed = true;
        trk->wake_cooldown_until_ticks = 0ULL;
        z_snapshot_publish(&g_adapter_ctx, &g_optics_ws);
        return true;
    }
    if (trk->wake_recovery_attempts >= MAX_WAKE_RECOVERY_HOLDS) {
        trk->wake_recovery_attempts = 0U;
        trk->wake_after_sck_cycle = false;
        g_adapter_ctx.wake_boot_prologue_seen = 0U;
        return false;
    }
    uint64_t hs_origin = current_hs_origin(trk);
    uint32_t hs_elapsed = (now_ticks >= hs_origin) ? (uint32_t)(now_ticks - hs_origin) : 0UL;
    trk->wake_recovery_attempts = (uint8_t)(trk->wake_recovery_attempts + 1U);
    g_adapter_ctx.vr.hotline_enabled = 0U;
    g_adapter_ctx.vr.hotline_seq = 0U;
    g_adapter_ctx.wake_from_park_req = 0U;
    e_usart1_dma_stop();
    e_tmra4_vsync_stop();
    z_spi1_hotline_stop();
    z_spi1_hotline_reset_seq();
    z_spi3_slave_stop_and_busy();
    z_spi3_slave_reset_to_wait_sck();
    gpio_pa9_set_busy();
    if (!trk->wake_timeout_logged) {
        trk->wake_timeout_logged = true;
        log_hs_timeout_bb(hs_elapsed);
        drain_core_diag_to_blackbox();
        blackbox_flush_to_flash();
    }
    g_adapter_ctx.cmd15_stage = 3U;
    g_adapter_ctx.cmd15_polls = 0U;
    g_adapter_ctx.cmd15_open_ticks = 0ULL;
    g_adapter_ctx.cmd01_retries = 0U;
    g_adapter_ctx.wake_ready = 0U;
    g_adapter_ctx.wake_retry_req = 0U;
    g_adapter_ctx.hs_step = HANDSHAKE_STEP_NONE;
    g_adapter_ctx.session_state = E_SESSION_POWERUP_WAIT;
    g_adapter_ctx.state_enter_ticks = now_ticks;
    g_adapter_ctx.hs_start_ticks = now_ticks;
    trk->mount_boot_ticks = now_ticks;
    trk->wake_hold_zero_until_ticks = now_ticks + WAKE_ZERO_HOLD_TKS;
    trk->z_want_open = false;
    trk->z_window_opened = false;
    trk->reopen_active = true;
    trk->detach_ticks = tmra1_now_ticks();
    z_snapshot_publish(&g_adapter_ctx, &g_optics_ws);
    return true;
}

static void handle_hs_timeout_action(loop_tracker_t *trk, uint64_t now_ticks) {
    uint64_t hs_origin = current_hs_origin(trk);
    uint32_t hs_elapsed = (now_ticks >= hs_origin) ? (uint32_t)(now_ticks - hs_origin) : 0UL;
    if ((g_adapter_ctx.hs_cmd07_ok == 0U) && (gpio_pa3_lens_mounted() == 0U)) {
        trk->wake_hold_zero_until_ticks = 0ULL;
        e_usart1_dma_stop();
        e_tmra4_vsync_stop();
        z_spi1_hotline_stop();
        z_spi3_slave_stop_and_busy();
        gpio_pa9_set_busy();
        return;
    }
    if (g_adapter_ctx.hs_cmd07_ok != 0U) {
        /* Active handshake identity known */
        log_hs_timeout_bb(hs_elapsed);
        lens_session_force_active(&g_adapter_ctx, &g_optics_ws);
        return;
    }
    if ((g_adapter_ctx.wake_rehandshake != 0U) &&
        (g_adapter_ctx.e_lens_id != 0U) &&
        handle_wake_probe_timeout(trk, now_ticks)) {
        return;
    }
    if (!trk->wake_timeout_logged) {
        log_hs_timeout_bb(hs_elapsed);
        trk->wake_timeout_logged = true;
    }
    trk->wake_after_sck_cycle = false;
    bool identity_changed = (g_adapter_ctx.e_lens_id != 0U);
    e_usart1_dma_stop();
    e_tmra4_vsync_stop();
    lens_session_enter_passive_mount(&g_adapter_ctx, &g_optics_ws);
    drain_core_diag_to_blackbox();
    blackbox_flush_to_flash();
    trk->tracked_epoch = g_adapter_ctx.session_epoch;
    if (identity_changed && (trk->z_window_opened || trk->reopen_active)) {
        z_spi1_hotline_stop();
        z_spi3_slave_stop_and_busy();
        z_spi3_slave_reset_to_wait_sck();
        gpio_pa9_set_busy();
        trk->z_want_open = false;
        trk->z_window_opened = false;
        trk->reopen_active = true;
        trk->detach_ticks = tmra1_now_ticks();
    } else if (!trk->reopen_active) {
        request_z_bus_open(trk);
    }
}

static void step_mount_handshake_guard(loop_tracker_t *trk, uint64_t now_ticks) {
    bool waiting_hs = false;
    if ((g_adapter_ctx.session_state == E_SESSION_POWERUP_WAIT) ||
        (g_adapter_ctx.session_state == E_SESSION_HANDSHAKE)) {
        waiting_hs = true;
    }
    if (!waiting_hs) {
        return;
    }

    uint64_t hs_origin = current_hs_origin(trk);
    uint64_t hs_elapsed = (now_ticks >= hs_origin) ? (now_ticks - hs_origin) : 0ULL;

    /* Cold-boot only: open Z-bus ready window within 150 ms so mount power stays alive while slow-POR
     * lenses complete E-mount homing under busy status reporting. */
    if ((!trk->cold_z_opened_once) && (!trk->z_want_open) &&
        ((hs_elapsed >= Z8_STATE03_GUARD_TKS) ||
         ((g_adapter_ctx.hs_tx08_done != 0U) &&
          (g_adapter_ctx.identity_ready != 0U) &&
          (g_adapter_ctx.optics_ready != 0U)))) {
        trk->tracked_epoch = g_adapter_ctx.session_epoch;
        request_z_bus_open(trk);
        return;
    }

    uint64_t probe_limit = (g_adapter_ctx.wake_rehandshake != 0U)
                               ? WAKE_FALLBACK_TKS
                               : PASSIVE_FALLBACK_TKS;
    uint64_t fallback_limit = lens_session_initial_probe_exhausted(&g_adapter_ctx, now_ticks)
                                  ? probe_limit
                                  : ACTIVE_HS_MAX_TICKS;
    if (hs_elapsed > fallback_limit) {
        handle_hs_timeout_action(trk, now_ticks);
    }
}

static void trigger_z_reattach_pulse(loop_tracker_t *trk) {
    trk->pending_optics_reopen = false;
    g_adapter_ctx.cmd15_stage = 3U;
    g_adapter_ctx.cmd15_polls = 0U;
    g_adapter_ctx.cmd15_open_ticks = 0ULL;
    g_adapter_ctx.v224.delivered_snap_sig = 0UL;
    g_adapter_ctx.v224.delivered_opt_gen = 0UL;
    z_spi3_slave_clear_provisional_seen();
    z_spi1_hotline_stop();
    z_spi3_slave_stop_and_busy();
    z_spi3_slave_reset_to_wait_sck();
    gpio_pa9_set_busy();
    trk->z_want_open = false;
    trk->z_window_opened = false;
    trk->reopen_active = true;
    trk->detach_ticks = tmra1_now_ticks();
}

static void promote_aberration_valid_optics_on_expiry(void) {
    if ((g_adapter_ctx.optics_ready == 0U) && (g_adapter_ctx.id05.aberration_valid != 0U)) {
        g_adapter_ctx.optics_ready = 1U;
        uint32_t next_gen = g_adapter_ctx.v224.current_opt_gen + 1UL;
        g_adapter_ctx.v224.current_opt_gen = (next_gen == 0UL) ? 1UL : next_gen;
    }
}

static bool maybe_service_generation_reattach(loop_tracker_t *trk) {
    bool stale_snap = (bool)((g_adapter_ctx.identity_ready != 0U) &&
                             (g_adapter_ctx.v224.delivered_snap_sig != 0UL) &&
                             (g_adapter_ctx.v224.delivered_snap_sig !=
                              z_snapshot_compute_sig(&g_adapter_ctx)));
    bool stale_opt = (bool)((g_adapter_ctx.optics_ready != 0U) &&
                            (g_adapter_ctx.v224.delivered_opt_epoch == g_adapter_ctx.session_epoch) &&
                            (g_adapter_ctx.v224.delivered_opt_gen != 0UL) &&
                            (g_adapter_ctx.v224.delivered_opt_gen < g_adapter_ctx.v224.current_opt_gen));
    if ((!stale_snap) && (!stale_opt)) {
        return false;
    }
    if (g_adapter_ctx.v224.phase2_reattach_tries >= 2U) {
        return false;
    }
    if ((!trk->z_window_opened) || trk->reopen_active ||
        z_spi3_slave_phase2_busy() || (g_adapter_ctx.af.af_drive_active != 0U)) {
        trk->pending_optics_reopen = true;
        return false;
    }
    g_adapter_ctx.v224.phase2_reattach_tries =
        (uint16_t)(g_adapter_ctx.v224.phase2_reattach_tries + 1U);
    if (g_adapter_ctx.optics_ready != 0U) {
        optics_disconnect_post_init(&g_adapter_ctx);
    }
    lens_session_push_diag(&g_adapter_ctx, BB_EVT_V2_PHASE2_OPT,
                           g_adapter_ctx.v224.delivered_snap_sig,
                           z_snapshot_compute_sig(&g_adapter_ctx),
                           g_adapter_ctx.v224.delivered_opt_gen,
                           g_adapter_ctx.v224.current_opt_gen);
    trigger_z_reattach_pulse(trk);
    return true;
}

static bool live_optics_ready_or_expired(uint64_t now_ticks) {
    if (g_adapter_ctx.optics_ready != 0U) {
        return true;
    }
    return (bool)((g_adapter_ctx.live_optics_wait_ticks != 0ULL) &&
                  (now_ticks >= g_adapter_ctx.live_optics_wait_ticks) &&
                  ((now_ticks - g_adapter_ctx.live_optics_wait_ticks) >= LIVE_OPTICS_WAIT_TKS));
}

static void on_active_link_up(loop_tracker_t *trk, uint64_t now_ticks) {
    trk->e_link_logged = true;
    trk->post51_guard_armed = false;
    trk->wake_timeout_logged = false;
    trk->wake_recovery_attempts = 0U;
    trk->wake_hold_zero_until_ticks = 0ULL;
    trk->wake_after_sck_cycle = false;
    g_adapter_ctx.wake_boot_prologue_seen = 0U;
    if (g_adapter_ctx.active_vsync_hz != trk->applied_vsync_hz) {
        e_tmra4_vsync_set_rate(g_adapter_ctx.active_vsync_hz);
        trk->applied_vsync_hz = g_adapter_ctx.active_vsync_hz;
        g_adapter_ctx.v224.rate_epoch += 1UL;
        if (trk->applied_vsync_hz >= VSYNC_120HZ_FAST) {
            uint32_t hz_arg = trk->applied_vsync_hz;
            record_bb(BB_EVT_E_120HZ_ACTIVE, g_adapter_ctx.e_lens_id,
                      &hz_arg, trk->applied_baud, 0U);
        }
    }
    g_adapter_ctx.v224.link_grace_ticks = now_ticks;
    g_adapter_ctx.v224.pending_vsync_hz = trk->applied_vsync_hz;
    uint32_t link_a0 = ((uint32_t)g_adapter_ctx.z_primary_lens_id) |
                       ((uint32_t)g_adapter_ctx.lens_family << PACK_SHIFT_16) |
                       ((uint32_t)g_adapter_ctx.af_class << PACK_SHIFT_24);
    uint32_t link_a1 = ((uint32_t)trk->applied_vsync_hz) |
                       ((uint32_t)g_adapter_ctx.af.scale_exp_n << PACK_SHIFT_8) |
                       ((uint32_t)g_adapter_ctx.e_lens_ver << PACK_SHIFT_16);
    uint32_t link_a2 = ((uint32_t)(uint16_t)g_adapter_ctx.af.e_inf_true) |
                       ((uint32_t)(uint16_t)g_adapter_ctx.af.e_near_true << PACK_SHIFT_16);
    uint32_t link_a3 = ((uint32_t)g_adapter_ctx.id06.defocus_k_se) |
                       ((uint32_t)g_adapter_ctx.id05.dist_code_x << PACK_SHIFT_16);
    record_bb4(BB_EVT_E_LINK_UP, g_adapter_ctx.e_lens_id,
               link_a0, link_a1, link_a2, link_a3);
    trk->tracked_epoch = g_adapter_ctx.session_epoch;

    /* Re-pulse interface only when Phase-2 TC delivered an outdated snapshot or optics gen */
    if (live_optics_ready_or_expired(now_ticks)) {
        trk->startup_repulse_done = true;
        promote_aberration_valid_optics_on_expiry();
        if (g_adapter_ctx.optics_ready != 0U) {
            optics_disconnect_post_init(&g_adapter_ctx);
        }
    }
    if (maybe_service_generation_reattach(trk)) {
        return;
    }
    if ((!trk->reopen_active) && (!trk->pending_optics_reopen)) {
        z_spi3_slave_clear_provisional_seen();
    }
    if (!trk->reopen_active) {
        request_z_bus_open(trk);
    }
}

static bool wait_initial_live_optics(const loop_tracker_t *trk, uint64_t now_ticks) {
    static const uint64_t GUARD_OPTICS_MARGIN_TKS = 9000ULL;
    if (g_adapter_ctx.optics_ready != 0U) {
        return false;
    }
    if (g_adapter_ctx.live_optics_wait_ticks == 0ULL) {
        g_adapter_ctx.live_optics_wait_ticks = (now_ticks != 0ULL) ? now_ticks : 1ULL;
    }
    uint64_t hs_origin = current_hs_origin(trk);
    uint64_t hs_elapsed = (now_ticks >= hs_origin) ? (now_ticks - hs_origin) : 0ULL;
    uint64_t opt_wait = (now_ticks >= g_adapter_ctx.live_optics_wait_ticks)
                            ? (now_ticks - g_adapter_ctx.live_optics_wait_ticks)
                            : 0ULL;
    bool guard_near = (bool)((!trk->cold_z_opened_once) &&
                             (hs_elapsed >= (Z8_STATE03_GUARD_TKS - GUARD_OPTICS_MARGIN_TKS)));
    return (bool)((!guard_near) && (opt_wait < LIVE_OPTICS_WAIT_TKS));
}

static void maybe_finish_startup_repulse(loop_tracker_t *trk, uint64_t now_ticks) {
    if ((!trk->startup_repulse_done) && live_optics_ready_or_expired(now_ticks)) {
        trk->startup_repulse_done = true;
        promote_aberration_valid_optics_on_expiry();
        if (g_adapter_ctx.optics_ready != 0U) {
            optics_disconnect_post_init(&g_adapter_ctx);
        }
    }
    if ((!maybe_service_generation_reattach(trk)) &&
        (!trk->pending_optics_reopen) && trk->startup_repulse_done) {
        z_spi3_slave_clear_provisional_seen();
    }
}

static uint8_t scale_frame_counter_hz(uint8_t cur_val, uint8_t old_hz, uint8_t new_hz) {
    static const uint32_t U8_BYTE_MAX = 0xFFU;
    if ((cur_val == 0U) || (old_hz == 0U) || (new_hz == 0U) || (old_hz == new_hz)) {
        return cur_val;
    }
    uint32_t scaled = (((uint32_t)cur_val * (uint32_t)new_hz) + ((uint32_t)old_hz - 1U)) /
                      (uint32_t)old_hz;
    return (scaled > U8_BYTE_MAX) ? (uint8_t)U8_BYTE_MAX : (uint8_t)scaled;
}

static void service_vsync_rate_governance(uint64_t now_ticks, loop_tracker_t *trk,
                                          uint16_t vd_phase_ticks) {
    if ((g_adapter_ctx.session_state != E_SESSION_ACTIVE) || (!trk->e_link_logged)) {
        return;
    }
    uint8_t cur_hz = e_tmra4_vsync_current_hz();
    uint64_t loss_anchor = (g_adapter_ctx.v224.link_grace_ticks > g_adapter_ctx.v224.last_valid_pair_ticks)
                               ? g_adapter_ctx.v224.link_grace_ticks
                               : g_adapter_ctx.v224.last_valid_pair_ticks;
    bool pair_lost = (bool)((g_adapter_ctx.v224.last_valid_pair_ticks == 0ULL) ||
                            ((now_ticks > loss_anchor) &&
                             ((now_ticks - loss_anchor) >= E_PAIR_LOSS_80MS_TKS)) ||
                            (g_adapter_ctx.v224.e_pair_err_streak >= E_PAIR_ERR_STREAK_MAX));
    if (pair_lost) {
        g_adapter_ctx.v224.e_feedback_unhealthy = 1U;
        g_adapter_ctx.v224.unhealthy_settle_streak = 0U;
        if ((cur_hz >= VSYNC_120HZ_FAST) || (g_adapter_ctx.supports_120hz != 0U)) {
            g_adapter_ctx.v224.fallback_60hz_latched = 1U;
        }
        if ((g_adapter_ctx.af.af_drive_active != 0U) ||
            (g_adapter_ctx.af.af_cmd_pending != 0U) ||
            (g_adapter_ctx.af_ext.parfocal_trim_pending != 0U)) {
            g_adapter_ctx.af.af_stop_req = 1U;
            g_adapter_ctx.af.af_drive_active = 0U;
            g_adapter_ctx.af.af_cmd_pending = 0U;
            g_adapter_ctx.af_ext.parfocal_trim_pending = 0U;
            g_adapter_ctx.parfocal_cmd_valid = 0U;
            g_adapter_ctx.v224.drive_ep_start_t256 = 0UL;
        }
    }

    bool pair_fresh = (bool)((g_adapter_ctx.v224.last_valid_pair_ticks != 0ULL) &&
                              (now_ticks >= g_adapter_ctx.v224.last_valid_pair_ticks) &&
                              ((now_ticks - g_adapter_ctx.v224.last_valid_pair_ticks) <
                               (E_PAIR_LOSS_80MS_TKS / 2ULL)));
    bool lens_allows_120 = (bool)((g_adapter_ctx.uart_baud >= BAUD_1500K_FAST) &&
                                  (g_adapter_ctx.supports_120hz != 0U) &&
                                  (g_adapter_ctx.v224.fallback_60hz_latched == 0U) &&
                                  (g_adapter_ctx.v224.e_feedback_unhealthy == 0U) &&
                                  ((cur_hz >= VSYNC_120HZ_FAST) || pair_fresh));
    uint8_t desired_hz = lens_allows_120 ? VSYNC_120HZ_FAST : VSYNC_60HZ_NOMINAL;
    g_adapter_ctx.v224.pending_vsync_hz = desired_hz;
    if (desired_hz == cur_hz) {
        return;
    }

    bool bus_safe = (bool)((vd_phase_ticks <= VD_COMMIT_PHASE_MAX_TKS) &&
                           (!e_usart1_dma_tx_busy()) &&
                           (gpio_ph2_high() == 0U) &&
                           (!z_spi3_slave_phase2_busy()));
    if (!bus_safe) {
        return;
    }
    bool emergency_fb = (bool)((!trk->id04_pending) &&
                               (g_adapter_ctx.v224.fallback_60hz_latched != 0U) &&
                               (desired_hz == VSYNC_60HZ_NOMINAL));
    bool idle_af = (bool)((!trk->id04_pending) &&
                          (g_adapter_ctx.af.af_drive_active == 0U) &&
                          (g_adapter_ctx.af.af_cmd_pending == 0U) &&
                          (g_adapter_ctx.af.af_stop_req == 0U) &&
                          (g_adapter_ctx.af.e0_brake_rem == 0U) &&
                          (g_adapter_ctx.id06.hw_moving == 0U) &&
                          (g_adapter_ctx.parfocal_hold_vsyncs == 0U) &&
                          (g_adapter_ctx.af_ext.parfocal_trim_pending == 0U));
    if ((!idle_af) && (!emergency_fb)) {
        return;
    }

    g_adapter_ctx.af.s1_hold_vsyncs =
        scale_frame_counter_hz(g_adapter_ctx.af.s1_hold_vsyncs, cur_hz, desired_hz);
    g_adapter_ctx.af.stall_vsyncs =
        scale_frame_counter_hz(g_adapter_ctx.af.stall_vsyncs, cur_hz, desired_hz);
    g_adapter_ctx.v224.id04_no_rx06_vsyncs =
        scale_frame_counter_hz(g_adapter_ctx.v224.id04_no_rx06_vsyncs, cur_hz, desired_hz);
    g_adapter_ctx.parfocal_cl_cooldown =
        scale_frame_counter_hz(g_adapter_ctx.parfocal_cl_cooldown, cur_hz, desired_hz);
    g_adapter_ctx.parfocal_e0_coast_rem =
        scale_frame_counter_hz(g_adapter_ctx.parfocal_e0_coast_rem, cur_hz, desired_hz);
    e_tmra4_vsync_set_rate(desired_hz);
    g_adapter_ctx.active_vsync_hz = desired_hz;
    trk->applied_vsync_hz = desired_hz;
    g_adapter_ctx.v224.rate_epoch += 1UL;
    g_adapter_ctx.v224.e_pair_err_streak = 0U;
    if (desired_hz >= VSYNC_120HZ_FAST) {
        uint32_t hz_arg = desired_hz;
        record_bb(BB_EVT_E_120HZ_ACTIVE, g_adapter_ctx.e_lens_id,
                  &hz_arg, trk->applied_baud, 0U);
    }
    z_snapshot_publish(&g_adapter_ctx, &g_optics_ws);
}

static bool step_wake_zero_hold(loop_tracker_t *trk, uint64_t now_ticks) {
    if (trk->wake_hold_zero_until_ticks == 0ULL) {
        return false;
    }
    if (now_ticks < trk->wake_hold_zero_until_ticks) {
        return true;
    }
    if (gpio_pa3_lens_mounted() == 0U) {
        trk->wake_hold_zero_until_ticks = now_ticks + WAKE_ZERO_HOLD_TKS;
        return true;
    }
    trk->wake_hold_zero_until_ticks = 0ULL;
    e_tmra4_vsync_init_60hz();
    e_usart1_dma_init_750k();
    g_adapter_ctx.vd_boot_start = e_tmra4_vsync_count();
    g_adapter_ctx.state_enter_ticks = now_ticks;
    g_adapter_ctx.hs_start_ticks = now_ticks;
    trk->mount_boot_ticks = now_ticks;
    return true;
}

static void service_e_handshake_and_link(loop_tracker_t *trk, uint64_t now_ticks) {
    if ((g_adapter_ctx.session_state == E_SESSION_DETACHED) ||
        (g_adapter_ctx.session_state == E_SESSION_EMPTY_MOUNT) ||
        (g_adapter_ctx.session_state == E_SESSION_PASSIVE_MOUNT)) {
        return;
    }
    if (step_wake_zero_hold(trk, now_ticks)) {
        return;
    }

    e_usart1_dma_service(&g_adapter_ctx, &g_optics_ws, now_ticks);

    uint32_t cur_hw_baud = e_usart1_dma_active_baud();
    if (cur_hw_baud != trk->applied_baud) {
        trk->applied_baud = cur_hw_baud;
        if ((trk->applied_baud >= BAUD_1500K_FAST) && (!trk->z_window_opened)) {
            record_bb(BB_EVT_E_BAUD_1M5, g_adapter_ctx.e_lens_id, &trk->applied_baud, 0U, 0U);
        }
    }

    if (g_adapter_ctx.session_state == E_SESSION_ACTIVE) {
        if (!trk->e_link_logged) {
            if (g_adapter_ctx.active_vsync_hz != trk->applied_vsync_hz) {
                e_tmra4_vsync_set_rate(g_adapter_ctx.active_vsync_hz);
                trk->applied_vsync_hz = g_adapter_ctx.active_vsync_hz;
                g_adapter_ctx.v224.rate_epoch += 1UL;
                if (trk->applied_vsync_hz >= VSYNC_120HZ_FAST) {
                    uint32_t hz_arg = trk->applied_vsync_hz;
                    record_bb(BB_EVT_E_120HZ_ACTIVE, g_adapter_ctx.e_lens_id,
                              &hz_arg, trk->applied_baud, 0U);
                }
            }
            if (wait_initial_live_optics(trk, now_ticks)) {
                return;
            }
            on_active_link_up(trk, now_ticks);
            return;
        }
        maybe_finish_startup_repulse(trk, now_ticks);
        return;
    }
    step_mount_handshake_guard(trk, now_ticks);
}

static bool park_bus_is_quiet(const loop_tracker_t *trk, uint64_t now_ticks) {
    uint64_t park_wait = (now_ticks >= trk->park_ready_ticks)
                             ? (now_ticks - trk->park_ready_ticks)
                             : 0ULL;
    uint64_t sck_low = ((trk->sck_low_since_ticks != 0ULL) &&
                        (now_ticks >= trk->sck_low_since_ticks))
                           ? (now_ticks - trk->sck_low_since_ticks)
                           : 0ULL;
    if (z_spi3_slave_cmd51_ready_delivered() &&
        (z_spi3_slave_idle_ticks(now_ticks) >= PARK_POST_51_IDLE_TKS)) {
        return true;
    }
    return (bool)((sck_low >= PARK_SCK_LOW_TKS) || (park_wait >= PARK_COMMIT_MAX_TKS));
}

static void arm_post51_wake_guard_if_quiet(loop_tracker_t *trk, uint64_t now_ticks) {
    if ((!trk->post51_guard_armed) && z_spi3_slave_cmd51_ready_delivered() &&
        (z_spi3_slave_idle_ticks(now_ticks) >= WAKE_GUARD_QUIET_TKS)) {
        trk->post51_guard_armed = true;
        trk->wake_cooldown_until_ticks = now_ticks + WAKE_COOLDOWN_TKS;
        if (g_adapter_ctx.wake_from_park_req == WAKE_REQ_PASSIVE_POLL) {
            g_adapter_ctx.wake_from_park_req = 0U;
        }
    }
}

static void try_commit_parked_session(loop_tracker_t *trk, uint64_t now_ticks) {
    if (trk->park_committed) {
        return;
    }
    if (trk->park_ready_ticks == 0ULL) {
        trk->park_ready_ticks = (now_ticks != 0ULL) ? now_ticks : 1ULL;
    }
    bool spi3_safe = (bool)((!trk->z_window_opened) || z_spi3_slave_is_idle_hdr() ||
                            z_spi3_slave_phase2_powerdown_seen());
    if (e_usart1_dma_tx_busy() || (!spi3_safe) || (!park_bus_is_quiet(trk, now_ticks))) {
        return;
    }
    arm_post51_wake_guard_if_quiet(trk, now_ticks);
    trk->park_committed = true;
    trk->wake_timeout_logged = false;
    trk->wake_recovery_attempts = 0U;
    trk->wake_hold_zero_until_ticks = 0ULL;
    g_adapter_ctx.wake_boot_prologue_seen = 0U;
    z_spi1_hotline_stop();
    e_usart1_dma_stop();
    e_tmra4_vsync_stop();
    if (gpio_pb14_sck_high() == 0U) {
        z_spi3_slave_quiesce_powerdown();
    } else {
        z_spi3_slave_stop_and_busy();
    }
    record_session_summary_bb(trk);
    drain_core_diag_to_blackbox();
    record_bb(BB_EVT_SHUTDOWN_PARKED, g_adapter_ctx.e_lens_id, NULL, 0U, 0U);
    blackbox_flush_to_flash();
    (void)journal_commit_idle(&g_adapter_ctx);
    if (gpio_pb14_sck_high() == 0U) {
        z_spi3_slave_quiesce_powerdown();
    } else {
        z_spi3_slave_reset_to_wait_sck();
    }
    request_z_bus_open(trk);
    trk->z_window_opened = false;
}

static void maybe_auto_park_on_sck_loss(const loop_tracker_t *trk, uint64_t now_ticks) {
    if ((!trk->z_ever_opened) || (g_adapter_ctx.session_state != E_SESSION_ACTIVE) ||
        (trk->sck_low_since_ticks == 0ULL) || (now_ticks < trk->sck_low_since_ticks)) {
        return;
    }
    if ((now_ticks - trk->sck_low_since_ticks) >= SCK_AUTO_PARK_LOW_TKS) {
        static const uint8_t AUTO_PARK_MOSI[4] = {0x55U, 0x00U, 0x00U, 0xABU};
        quiesce_hotline_on_sck_loss();
        z_spi3_slave_clear_cmd51_ready();
        vr_controller_on_z_command(&g_adapter_ctx, Z_CMD_PARK_AUTO_50,
                                   AUTO_PARK_MOSI, sizeof(AUTO_PARK_MOSI), &now_ticks);
    }
}

static void service_park_and_wake(loop_tracker_t *trk, uint64_t now_ticks) {
    uint8_t pa3_now = gpio_pa3_lens_mounted();
    maybe_auto_park_on_sck_loss(trk, now_ticks);
    if ((g_adapter_ctx.session_state == E_SESSION_PARKING) ||
        (g_adapter_ctx.park_after_handshake != 0U)) {
        vr_controller_step_parking(&g_adapter_ctx, now_ticks);
    }

    if (g_adapter_ctx.session_state == E_SESSION_PARKED) {
        arm_post51_wake_guard_if_quiet(trk, now_ticks);
    }

    bool force_wake = (bool)((g_adapter_ctx.vr.hotline_enabled != 0U) ||
                             (g_adapter_ctx.wake_from_park_req >= WAKE_REQ_ACTIVE_CMD));
    bool passive_wake_ok = (bool)((g_adapter_ctx.wake_from_park_req == WAKE_REQ_PASSIVE_POLL) &&
                                  (now_ticks >= trk->wake_cooldown_until_ticks));
    bool allow_wake = (bool)(force_wake || passive_wake_ok);
    if ((g_adapter_ctx.session_state == E_SESSION_PARKED) &&
        allow_wake &&
        (pa3_now != 0U)) {
        trk->park_committed = false;
        trk->post51_guard_armed = false;
        z_spi3_slave_clear_cmd51_ready();
        trk->park_ready_ticks = 0ULL;
        trk->e_link_logged = false;
        trk->pending_optics_reopen = false;
        trk->startup_repulse_done = false;
        trk->applied_baud = BAUD_750K_INIT;
        trk->applied_vsync_hz = VSYNC_60HZ_NOMINAL;
        g_adapter_ctx.z_sck_powerdown_latched = 0U;
        g_adapter_ctx.home_completed_ok = 0U;
        lens_session_arm_home_retrigger(&g_adapter_ctx);
        g_adapter_ctx.wake_from_park_req = 0U;
        g_adapter_ctx.wake_rehandshake = 1U;
        g_adapter_ctx.v224.phase2_reattach_tries = 0U;
        g_adapter_ctx.v224.v224_bb_v2_parked_done = 0U;
        g_adapter_ctx.hs_cmd07_ok = 0U;
        g_adapter_ctx.hs_tx08_done = 0U;
        g_adapter_ctx.tx_q_head = 0U;
        g_adapter_ctx.tx_q_tail = 0U;
        g_adapter_ctx.cmd01_retries = 0U;
        g_adapter_ctx.wake_ready = 0U;
        g_adapter_ctx.wake_retry_req = 0U;
        g_adapter_ctx.live_optics_wait_ticks = 0ULL;
        g_adapter_ctx.uart_baud = BAUD_750K_INIT;
        g_adapter_ctx.active_vsync_hz = VSYNC_60HZ_NOMINAL;
        g_adapter_ctx.hs_step = HANDSHAKE_STEP_NONE;
        g_adapter_ctx.session_state = E_SESSION_POWERUP_WAIT;
        e_tmra4_vsync_init_60hz();
        e_usart1_dma_init_750k();
        g_adapter_ctx.vd_boot_start = e_tmra4_vsync_count();
        g_adapter_ctx.state_enter_ticks = now_ticks;
        g_adapter_ctx.hs_start_ticks = now_ticks;
        trk->mount_boot_ticks = now_ticks;
        trk->tracked_epoch = g_adapter_ctx.session_epoch;
        return;
    }

    if ((g_adapter_ctx.session_state == E_SESSION_PARKED) &&
        (g_adapter_ctx.wake_from_park_req == WAKE_REQ_PASSIVE_POLL) &&
        (now_ticks < trk->wake_cooldown_until_ticks)) {
        g_adapter_ctx.wake_from_park_req = 0U;
        g_adapter_ctx.wake_boot_prologue_seen = 0U;
    }

    if (g_adapter_ctx.session_state != E_SESSION_PARKED) {
        trk->park_ready_ticks = 0ULL;
        trk->post51_guard_armed = false;
        z_spi3_slave_clear_cmd51_ready();
        return;
    }

    try_commit_parked_session(trk, now_ticks);
}

static void begin_hotplug_attach(loop_tracker_t *trk, uint64_t now_ticks) {
    if (trk->z_ever_opened) {
        g_adapter_ctx.z_sck_powerdown_latched = (gpio_pb14_sck_high() == 0U) ? 1U : 0U;
        lens_session_arm_home_retrigger(&g_adapter_ctx);
    } else {
        g_adapter_ctx.z_sck_powerdown_latched = 0U;
        g_adapter_ctx.home_retrigger_armed = 0U;
        g_adapter_ctx.cmd15_seen_since_arm = 0U;
    }
    z_spi1_hotline_stop();
    z_spi3_slave_stop_and_busy();
    z_spi3_slave_reset_to_wait_sck();
    gpio_pa9_set_busy(); /* Hold PA9=1 Busy while SPI3 is stopped */
    trk->z_want_open = false;
    trk->z_window_opened = false;
    trk->cold_z_opened_once = true;
    trk->reopen_active = true;
    trk->detach_ticks = tmra1_now_ticks();
    trk->e_link_logged = false;
    trk->pending_optics_reopen = false;
    trk->startup_repulse_done = false;
    g_adapter_ctx.v224.delivered_snap_sig = 0UL;
    g_adapter_ctx.v224.delivered_opt_gen = 0UL;
    g_adapter_ctx.v224.phase2_reattach_tries = 0U;
    trk->park_committed = false;
    trk->post51_guard_armed = false;
    z_spi3_slave_clear_cmd51_ready();
    trk->wake_timeout_logged = false;
    trk->wake_recovery_attempts = 0U;
    trk->wake_hold_zero_until_ticks = 0ULL;
    g_adapter_ctx.wake_boot_prologue_seen = 0U;
    trk->sck_power_cycled_low = false;
    trk->wake_after_sck_cycle = false;
    trk->detach_hold = false;
    trk->vd_pending = false;
    trk->id04_pending = false;
    trk->applied_baud = BAUD_750K_INIT;
    trk->applied_vsync_hz = VSYNC_60HZ_NOMINAL;
    e_tmra4_vsync_init_60hz();
    e_usart1_dma_init_750k();
    g_adapter_ctx.hs_cmd07_ok = 0U;
    g_adapter_ctx.hs_tx08_done = 0U;
    g_adapter_ctx.vd_boot_start = e_tmra4_vsync_count();
    g_adapter_ctx.hs_start_ticks = now_ticks;
    trk->mount_boot_ticks = now_ticks;
    trk->tracked_epoch = g_adapter_ctx.session_epoch;
}

static void begin_hotplug_detach(loop_tracker_t *trk) {
    z_spi1_hotline_stop();
    z_spi3_slave_stop_and_busy();
    gpio_pa9_set_busy();
    e_usart1_dma_stop();
    e_tmra4_vsync_stop();
    trk->z_want_open = false;
    trk->z_window_opened = false;
    trk->cold_z_opened_once = true;
    trk->reopen_active = false;
    trk->e_link_logged = false;
    trk->pending_optics_reopen = false;
    trk->startup_repulse_done = false;
    g_adapter_ctx.v224.delivered_snap_sig = 0UL;
    g_adapter_ctx.v224.delivered_opt_gen = 0UL;
    g_adapter_ctx.v224.phase2_reattach_tries = 0U;
    trk->park_committed = false;
    trk->post51_guard_armed = false;
    z_spi3_slave_clear_cmd51_ready();
    trk->wake_timeout_logged = false;
    trk->wake_recovery_attempts = 0U;
    trk->wake_hold_zero_until_ticks = 0ULL;
    g_adapter_ctx.wake_boot_prologue_seen = 0U;
    trk->sck_power_cycled_low = false;
    trk->wake_after_sck_cycle = false;
    trk->detach_hold = true;
    trk->detach_ticks = tmra1_now_ticks();
    trk->vd_pending = false;
    trk->id04_pending = false;
    trk->applied_baud = BAUD_750K_INIT;
    trk->applied_vsync_hz = VSYNC_60HZ_NOMINAL;
    record_session_summary_bb(trk);
    drain_core_diag_to_blackbox();
    record_bb(BB_EVT_HOTPLUG_DETACH, g_adapter_ctx.e_lens_id, &trk->tracked_epoch,
              g_adapter_ctx.diag_af_cmd_count, g_adapter_ctx.diag_af_stall_count);
    blackbox_flush_to_flash();
    lens_session_enter_detached(&g_adapter_ctx, &g_optics_ws);
    trk->tracked_epoch = g_adapter_ctx.session_epoch;
}

static void service_reopen_and_detach_hold(loop_tracker_t *trk, uint64_t now_ticks, bool hs_busy) {
    uint64_t cur_ticks = tmra1_now_ticks();
    bool hold_elapsed = (bool)((cur_ticks >= trk->detach_ticks) &&
                               ((cur_ticks - trk->detach_ticks) >= DETACH_REOPEN_TICKS));
    if (!hold_elapsed) {
        return;
    }
    bool waiting_live_opt = (bool)((g_adapter_ctx.session_state == E_SESSION_ACTIVE) &&
                                   (!trk->e_link_logged) &&
                                   wait_initial_live_optics(trk, now_ticks));
    bool can_reopen_z = (bool)(((!hs_busy) && (!waiting_live_opt)) ||
                               ((g_adapter_ctx.hs_cmd07_ok != 0U) &&
                                (g_adapter_ctx.identity_ready != 0U) &&
                                (g_adapter_ctx.optics_ready != 0U)));
    if (trk->reopen_active && can_reopen_z) {
        trk->reopen_active = false;
        z_spi3_slave_reset_to_wait_sck();
        request_z_bus_open(trk);
    }
    if (trk->detach_hold && (g_adapter_ctx.session_state == E_SESSION_DETACHED)) {
        trk->detach_hold = false;
        z_spi3_slave_reset_to_wait_sck();
        request_z_bus_open(trk);
        if (gpio_pb14_sck_high() == 0U) {
            gpio_pa9_set_ready();
        }
        lens_session_enter_empty_mount(&g_adapter_ctx, &g_optics_ws);
        trk->tracked_epoch = g_adapter_ctx.session_epoch;
    }
}

static void service_hotplug_and_parking(loop_tracker_t *trk, uint64_t now_ticks) {
    uint8_t pa3_now = gpio_pa3_lens_mounted();
    if (trk->park_committed &&
        (g_adapter_ctx.session_state != E_SESSION_PARKED) &&
        (g_adapter_ctx.session_state != E_SESSION_PARKING)) {
        trk->park_committed = false;
        if ((pa3_now == 0U) && (g_adapter_ctx.session_state == E_SESSION_DETACHED)) {
            lens_session_enter_empty_mount(&g_adapter_ctx, &g_optics_ws);
            trk->tracked_epoch = g_adapter_ctx.session_epoch;
        }
    }

    if ((now_ticks - trk->last_hotplug_ticks) >= HOTPLUG_POLL_TICKS) {
        trk->last_hotplug_ticks = now_ticks;
        uint32_t cur_vd = e_tmra4_vsync_count();
        e_session_state_t prev_state = g_adapter_ctx.session_state;
        lens_session_step(&g_adapter_ctx, &g_optics_ws, pa3_now, &cur_vd, now_ticks);

        bool was_unattached = (bool)((prev_state == E_SESSION_DETACHED) ||
                                     (prev_state == E_SESSION_EMPTY_MOUNT) ||
                                     (prev_state == E_SESSION_PASSIVE_MOUNT) ||
                                     (g_adapter_ctx.session_epoch != trk->tracked_epoch));
        if (was_unattached && (g_adapter_ctx.session_state == E_SESSION_POWERUP_WAIT)) {
            begin_hotplug_attach(trk, now_ticks);
        } else if (g_adapter_ctx.session_epoch != trk->tracked_epoch) {
            trk->tracked_epoch = g_adapter_ctx.session_epoch;
            if (g_adapter_ctx.session_state == E_SESSION_DETACHED) {
                begin_hotplug_detach(trk);
            }
        }
    }

    bool hs_busy = false;
    if ((g_adapter_ctx.session_state == E_SESSION_POWERUP_WAIT) ||
        (g_adapter_ctx.session_state == E_SESSION_HANDSHAKE)) {
        hs_busy = true;
    }
    service_reopen_and_detach_hold(trk, now_ticks, hs_busy);

    service_park_and_wake(trk, now_ticks);
}

static void service_vsync_id04(loop_tracker_t *trk, uint64_t now_ticks,
                               bool fast_120, uint16_t vd_phase) {
    if (!trk->id04_pending) {
        return;
    }
    uint16_t id04_max = fast_120 ? ID04_MAX_120HZ_TKS : ID04_MAX_60HZ_TKS;
    if ((g_adapter_ctx.session_state != E_SESSION_ACTIVE) || (vd_phase > id04_max)) {
        trk->id04_pending = false;
        return;
    }
    uint16_t id04_phase = fast_120 ? ID04_PHASE_120HZ_TKS : ID04_PHASE_60HZ_TKS;
    uint64_t id04_gap = fast_120 ? ID04_GAP_120HZ_TKS : ID04_GAP_60HZ_TKS;
    if ((vd_phase < id04_phase) ||
        ((now_ticks - trk->id03_sent_ticks) < id04_gap) ||
        e_usart1_dma_tx_busy() || (gpio_ph2_high() != 0U)) {
        return;
    }
    af_servo_note_wall_ticks(&g_adapter_ctx, now_ticks);
    af_servo_state_t previous_af = g_adapter_ctx.af;
    af_v218_ext_t previous_ext = g_adapter_ctx.af_ext;
    uint8_t previous_sync = g_adapter_ctx.init_sync_rem;
    float previous_pf_dist = g_adapter_ctx.parfocal_dist_m;
    int32_t previous_pf_se = g_adapter_ctx.parfocal_last_cmd_se;
    uint16_t previous_pf_f01 = g_adapter_ctx.parfocal_last_f01;
    uint8_t previous_pf_hold = g_adapter_ctx.parfocal_hold_vsyncs;
    uint8_t previous_pf_valid = g_adapter_ctx.parfocal_cmd_valid;
    int32_t previous_pf_cl_trim = g_adapter_ctx.parfocal_cl_trim_cz;
    uint16_t previous_pf_anchor_dx = g_adapter_ctx.parfocal_anchor_dist_x;
    uint8_t previous_pf_cl_cd = g_adapter_ctx.parfocal_cl_cooldown;
    uint8_t previous_pf_cl_steps = g_adapter_ctx.parfocal_cl_steps;
    uint16_t previous_pf_anchor_f01 = g_adapter_ctx.parfocal_anchor_f01;
    uint16_t previous_pf_trim_f01 = g_adapter_ctx.parfocal_trim_f01;
    uint16_t previous_pf_min_rem = g_adapter_ctx.parfocal_min_rem_se;
    uint8_t previous_pf_refresh = g_adapter_ctx.parfocal_anchor_refresh;
    uint8_t previous_pf_coast = g_adapter_ctx.parfocal_e0_coast_rem;
    uint8_t previous_id05_pair = g_adapter_ctx.id05_fresh_since_06;
    uint16_t previous_stall_cnt = g_adapter_ctx.diag_af_stall_count;
    uint8_t previous_bb_stall = g_adapter_ctx.bb_stall_logged;
    uint8_t previous_diag_head = g_adapter_ctx.core_diag_head;
    uint8_t previous_diag_tail = g_adapter_ctx.core_diag_tail;
    bb_event_input_t saved_dq[CORE_DIAG_Q_CAP];
    memcpy(saved_dq, g_adapter_ctx.core_diag_q, sizeof(saved_dq));
    uint8_t previous_vsync_hz = g_adapter_ctx.active_vsync_hz;
    uint8_t prev_v224_raw[sizeof(g_adapter_ctx.v224)];
    memcpy(prev_v224_raw, &g_adapter_ctx.v224, sizeof(prev_v224_raw));
    uint8_t id04_wire[LOCAL_ID03_WIRE_CAP];
    size_t id04_len = lens_session_build_id04_wire(
        &g_adapter_ctx, id04_wire, g_adapter_ctx.vd_seq_latched);
    if (e_usart1_dma_send_frame(&g_adapter_ctx, now_ticks, id04_wire, (uint16_t)id04_len)) {
        trk->id04_pending = false;
    } else {
        g_adapter_ctx.active_vsync_hz = previous_vsync_hz;
        g_adapter_ctx.af = previous_af;
        g_adapter_ctx.af_ext = previous_ext;
        g_adapter_ctx.init_sync_rem = previous_sync;
        g_adapter_ctx.parfocal_dist_m = previous_pf_dist;
        g_adapter_ctx.parfocal_last_cmd_se = previous_pf_se;
        g_adapter_ctx.parfocal_last_f01 = previous_pf_f01;
        g_adapter_ctx.parfocal_hold_vsyncs = previous_pf_hold;
        g_adapter_ctx.parfocal_cmd_valid = previous_pf_valid;
        g_adapter_ctx.parfocal_cl_trim_cz = previous_pf_cl_trim;
        g_adapter_ctx.parfocal_anchor_dist_x = previous_pf_anchor_dx;
        g_adapter_ctx.parfocal_cl_cooldown = previous_pf_cl_cd;
        g_adapter_ctx.parfocal_cl_steps = previous_pf_cl_steps;
        g_adapter_ctx.parfocal_anchor_f01 = previous_pf_anchor_f01;
        g_adapter_ctx.parfocal_trim_f01 = previous_pf_trim_f01;
        g_adapter_ctx.parfocal_min_rem_se = previous_pf_min_rem;
        g_adapter_ctx.parfocal_anchor_refresh = previous_pf_refresh;
        g_adapter_ctx.parfocal_e0_coast_rem = previous_pf_coast;
        g_adapter_ctx.id05_fresh_since_06 = previous_id05_pair;
        g_adapter_ctx.diag_af_stall_count = previous_stall_cnt;
        g_adapter_ctx.bb_stall_logged = previous_bb_stall;
        if (g_adapter_ctx.core_diag_head != previous_diag_head) {
            memcpy(g_adapter_ctx.core_diag_q, saved_dq, sizeof(saved_dq));
        }
        g_adapter_ctx.core_diag_head = previous_diag_head;
        g_adapter_ctx.core_diag_tail = previous_diag_tail;
        memcpy(&g_adapter_ctx.v224, prev_v224_raw, sizeof(g_adapter_ctx.v224));
        if ((previous_af.af_drive_active != 0U) || (previous_af.af_cmd_pending != 0U)) {
            enum {
                TICKS_PER_01MS = 300U,
            };
            int32_t d_pos = g_adapter_ctx.id06.focus_pos_se - g_adapter_ctx.af_drive_start_se;
            int32_t d_tgt = g_adapter_ctx.af.target_se - g_adapter_ctx.af_drive_start_se;
            v224_af_trace_entry_t rej_tr = {
                .ts_ms_x10    = boryoza_ticks64_div_u16_lo32(now_ticks, (uint32_t)TICKS_PER_01MS),
                .pos_se       = clamp_s32_to_i16(d_pos),
                .tgt_se       = clamp_s32_to_i16(d_tgt),
                .step_se      = 0,
                .defocus_mag  = g_adapter_ctx.id06.defocus_k_se,
                .owner_cmd    = g_adapter_ctx.v224.txn_owner_cmd,
                .submode      = g_adapter_ctx.af.af_submode,
                .flags        = 0x01U,
                .stall_frames = g_adapter_ctx.af.stall_vsyncs,
            };
            v224_trace_af_push(&rej_tr);
        }
    }
}

static void service_vsync_id03(loop_tracker_t *trk, uint64_t now_ticks) {
    bool wait_05_06_idle = false;
    if ((g_adapter_ctx.session_state == E_SESSION_HANDSHAKE) &&
        (g_adapter_ctx.hs_step == HANDSHAKE_STEP_WAIT_05_06) &&
        (g_adapter_ctx.tx_q_head == g_adapter_ctx.tx_q_tail)) {
        wait_05_06_idle = true;
    }
    bool can_poll_id03 = false;
    if ((g_adapter_ctx.session_state == E_SESSION_ACTIVE) || wait_05_06_idle ||
        ((g_adapter_ctx.session_state == E_SESSION_PARKING) &&
         (g_adapter_ctx.vr.park_state == PARK_WAIT_VR_STOP))) {
        can_poll_id03 = true;
    }

    bool fast_120 = (trk->applied_vsync_hz >= VSYNC_120HZ_FAST);
    uint16_t vd_phase = 0U;
    uint8_t vd_seq = 0U;
    e_tmra4_vsync_snapshot(NULL, &vd_seq, &vd_phase, NULL);
    uint16_t id03_phase = fast_120 ? ID03_PHASE_120HZ_TKS : ID03_PHASE_60HZ_TKS;
    uint16_t id03_max = fast_120 ? ID03_MAX_120HZ_TKS : ID03_MAX_60HZ_TKS;

    if (trk->vd_pending && (vd_phase > id03_max)) {
        trk->vd_pending = false;
    }

    if (trk->vd_pending && can_poll_id03 && (vd_phase >= id03_phase) &&
        (!e_usart1_dma_tx_busy()) && (gpio_ph2_high() == 0U)) {
        uint8_t id03_wire[LOCAL_ID03_WIRE_CAP];
        uint8_t previous_toggle = g_adapter_ctx.id03_frame_toggle;
        size_t id03_len = lens_session_build_id03_wire(
            &g_adapter_ctx, id03_wire, vd_seq);
        if (!e_usart1_dma_send_frame(&g_adapter_ctx, now_ticks, id03_wire, (uint16_t)id03_len)) {
            g_adapter_ctx.id03_frame_toggle = previous_toggle;
            return;
        }
        trk->vd_pending = false;
        g_adapter_ctx.vd_seq_latched = vd_seq;
        g_adapter_ctx.e_tx_seq = vd_seq;
        trk->id03_sent_ticks = now_ticks;
        if (g_adapter_ctx.session_state == E_SESSION_ACTIVE) {
            if (g_adapter_ctx.af_class == AF_CLASS_B_LEGACY_F1_F2) {
                af_servo_emit_vsync_drive(&g_adapter_ctx);
            } else {
                trk->id04_pending = true;
            }
        }
        return;
    }

    service_vsync_id04(trk, now_ticks, fast_120, vd_phase);
}

void boryoza_main(void) {
    gpio_pads_cold_init();
    if (!sys_clock_sram_init_192mhz()) {
        for (;;) {
            cpu_nop();
        }
    }
    tmra1_clock_init();
    z_spi3_slave_init();
    z_spi1_hotline_init();
    blackbox_init();
    lens_session_init(&g_adapter_ctx, &g_optics_ws);
    journal_init_and_load(&g_adapter_ctx);

    cpu_irq_enable();

    loop_tracker_t trk = {
        .applied_baud = BAUD_750K_INIT,
        .applied_vsync_hz = VSYNC_60HZ_NOMINAL,
        .z_want_open = false,
        .z_window_opened = false,
        .e_link_logged = false,
        .park_committed = false,
        .post51_guard_armed = false,
        .wake_timeout_logged = false,
        .wake_recovery_attempts = 0U,
        .sck_power_cycled_low = false,
        .wake_after_sck_cycle = false,
        .detach_hold = false,
        .vd_pending = false,
        .id04_pending = false,
        .last_vd_seen = 0U,
        .id03_sent_ticks = 0ULL,
        .last_hotplug_ticks = 0ULL,
        .detach_ticks = 0ULL,
        .mount_boot_ticks = 0ULL,
        .wake_cooldown_until_ticks = 0ULL,
        .wake_hold_zero_until_ticks = 0ULL,
        .tracked_epoch = g_adapter_ctx.session_epoch,
    };

    uint32_t hclk_val = HC32_HCLK_HZ;
    record_bb(BB_EVT_BOOT_OK, 0U, &hclk_val,
              mmio_r32(SRAMC_WTCR_ADDR), mmio_r32(EFM_FRMC_ADDR));
    uint32_t reset_flags = g_boot_reset_flags;
    record_bb(BB_EVT_RESET_SOURCE, 0U, &reset_flags, g_boot_debug_state, g_boot_sram_status);
    uint32_t sram_check = g_boot_sram_check;
    record_bb(BB_EVT_RESET_ROUTING, 0U, &sram_check, g_boot_nmi_enable, g_nmi_snapshot[2]);

    if (!sample_cold_pa3_mounted()) {
        lens_session_enter_empty_mount(&g_adapter_ctx, &g_optics_ws);
        g_adapter_ctx.retram_warm_hit = retram_warm_cache_is_valid() ? 1U : 0U;
        uint32_t f_mm = g_adapter_ctx.f_cur_mm;
        record_bb(BB_EVT_EMPTY_MOUNT_READY, 0U, &f_mm, g_adapter_ctx.ap.c_open_cur, 0U);
        request_z_bus_open(&trk);
    } else {
        g_adapter_ctx.retram_warm_hit = retram_warm_cache_is_valid() ? 1U : 0U;
        e_tmra4_vsync_init_60hz();
        e_usart1_dma_init_750k();
        uint64_t boot_ticks = tmra1_now_ticks();
        g_adapter_ctx.session_state = E_SESSION_POWERUP_WAIT;
        g_adapter_ctx.vd_boot_start = e_tmra4_vsync_count();
        g_adapter_ctx.state_enter_ticks = boot_ticks;
        g_adapter_ctx.hs_start_ticks = boot_ticks;
        trk.mount_boot_ticks = boot_ticks;
    }

    trk.last_vd_seen = e_tmra4_vsync_count();
    trk.last_hotplug_ticks = tmra1_now_ticks();
    trk.tracked_epoch = g_adapter_ctx.session_epoch;

    for (;;) {
        uint64_t now_ticks = tmra1_now_ticks();
        service_z_bus(&trk, now_ticks);
        if (vr_controller_step(&g_adapter_ctx, now_ticks)) {
            z_snapshot_publish(&g_adapter_ctx, &g_optics_ws);
        }

        uint32_t vd_now = 0U;
        uint16_t vd_phase_now = 0U;
        e_tmra4_vsync_snapshot(&vd_now, NULL, &vd_phase_now, NULL);
        if (vd_now != trk.last_vd_seen) {
            trk.last_vd_seen = vd_now;
            trk.vd_pending = true;
            trk.id04_pending = false;
        }
        bool sync_hold = false;
        if ((g_adapter_ctx.session_state == E_SESSION_ACTIVE) &&
            (trk.vd_pending || trk.id04_pending)) {
            sync_hold = true;
        }
        e_usart1_dma_set_sync_hold(sync_hold);
        service_e_handshake_and_link(&trk, now_ticks);
        service_vsync_rate_governance(now_ticks, &trk, vd_phase_now);
        service_vsync_id03(&trk, now_ticks);

        service_hotplug_and_parking(&trk, now_ticks);
        drain_core_diag_to_blackbox();
    }
}
