#include "../../include/boryoza_types.h"
#include "../../include/boryoza_math_mem.h"
#include "../../include/lens_db.h"

enum {
    MF_GATE_TABLE_LEN = 24,
    Z_A0_RESP_BYTES   = 24,
};

static const uint32_t Z_AV_F1_0           = 0x0180UL; /* 384 */
static const uint32_t Z_AV_PER_EV         = 0x00C0UL; /* 192 */
static const int32_t  Z_AV_MIN_CLAMP      = 0x00C0;
static const int32_t  Z_AV_LOW_F_CLAMP    = 0x0100;
static const int32_t  Z_AV_MAX_CLAMP      = 0x0B40;
static const uint16_t Z_AV_F2_0_FALLBACK  = 0x0300U;
static const uint16_t Z_AV_F22_FALLBACK   = 0x0840U;
static const int32_t  Z_DELTA_MAX_POS     = 0x7FFF;
static const int32_t  E_AV_F1_0_HI        = 0x10;
static const uint32_t E_AV_OFFSET_BASE    = 0x0E00UL;
static const uint32_t E_AV_ROUND_HALF     = 128UL;
static const uint16_t E_AV_VALID_FLOOR    = 0x0E00U;
static const int32_t  E_AV_MAX_CLAMP      = 0x2000;
static const size_t   ID05_GRP_RING       = 3U;
static const size_t   ID05_GRP_FOCAL      = 4U;
static const size_t   ID05_GRP_ABERR      = 5U;

static const uint16_t OPTICAL_INF_CODE_X     = 0x0700U;
static const uint16_t Z_DIST_INF_0X117       = 0xFFFFU;
static const int32_t  Z_DIST_FINITE_MIN      = 0x0008;
static const int32_t  Z_DIST_FINITE_MAX_SAFE = 0xBE00;
static const int32_t  E_TO_Z_DIST_BASE       = 2048;
static const int32_t  E_TO_Z_DIST_SLOPE      = 48;

static const uint16_t BUCKET_DIST_CLOSE      = 15100U;
static const uint16_t BUCKET_DIST_1M         = 20480U;
static const uint16_t BUCKET_DIST_10M        = 31000U;
static const int32_t  BUCKET_SPAN_1M         = 5380;
static const int32_t  BUCKET_SPAN_10M        = 10520;
static const int32_t  BUCKET_SPAN_TAIL       = 17640;
static const int32_t  BUCKET_HYPERFOCAL      = 48640;
static const int32_t  BUCKET_STEP_1M         = 128;
static const int32_t  BUCKET_STEP_10M        = 42;
static const int32_t  BUCKET_STEP_SPAN12     = 86;
static const int32_t  BUCKET_STEP_SPAN5      = 41;
static const uint16_t MF_BUCKETS_TOTAL       = 255U;

static const uint16_t NORM_COORD_MAX      = 4095U;
static const uint16_t ZOOM_NORM_WIDE      = 0x0100U;
static const uint16_t ZOOM_NORM_TELE      = 0x3F00U;
static const uint16_t MIN_FOCAL_0P1MM     = 50U;
static const uint16_t FOCAL_ROUND_HALF    = 5U;
static const uint16_t FOCAL_DIV_10             = 10U;
static const uint8_t  DIST_BIAS_UNCALIBRATED   = 1U;
static const uint8_t  DIST_BIAS_CALIBRATED     = 2U;
static const int32_t  DIST_HALF_BIN_CODES      = 24;
static const int32_t  DIST_TUBE_HALF_CODES     = 26;
static const int32_t  DIST_UNCAL_TUBE_CODES    = 48;
static const int32_t  DIST_REANCHOR_JUMP_CODES = 96;
static const uint8_t  MAG_MAX_FINITE_CODE      = 0xFEU;
static const uint16_t Q4_INT_MULT         = 16U;
static const int32_t  Q4_MIN_CLAMP        = 64;
static const int32_t  Q4_MAX_CLAMP        = 65535;
static const uint16_t MIN_FNUM_X100       = 50U;
static const size_t   MOSI20_TUNING_OFF   = 0x28U;
static const size_t   MOSI20_TUNING_MIN   = 42U;
static const uint64_t RING_HB_PERIOD_TKS  = 300000ULL; /* 100.0 ms @ 3.000 MHz */
static const uint32_t BYTE_MASK           = 0xFFUL;
static const uint32_t SHIFT_8             = 8U;
static const uint32_t SHIFT_16            = 16U;
static const int32_t  S16_MAX_BOUND       = 32767;
static const float    MAG_BETA_MIN        = 0.001F;
static const float    MAG_BETA_MAX        = 10.0F;
static const uint16_t DIST_SENTINEL_FF00  = 0xFF00U;
static const uint16_t DIST_SENTINEL_7F00  = 0x7F00U;
static const uint16_t DIST_SENTINEL_FFFF  = 0xFFFFU;
static const uint16_t DIST_SENTINEL_00FF  = 0x00FFU;
static const int32_t  SAFE_CZ_MAX_256K    = 262144;
static const int32_t  PERMILLE_SCALE_1024 = 1024;
static const int32_t  PM_CLOSE_FLOOR      = 64;
static const int32_t  PM_FROZEN_TRAVEL    = 220;
static const int32_t  SE_FROZEN_MIN_TRV   = 64;
static const int32_t  RECOVER_MAX_STEP_X  = 128;
static const uint8_t  DIST_SAMPLES_CAP    = 250U;
static const uint8_t  DIST_FROZEN_MIN_CNT = 4U;
static const uint8_t  DIST_RECOVER_VOTES  = 3U;

static const float LOG2_ANCHOR_1M_CODE    = 20480.0F;
static const float LOG2_OCTAVE_CODES      = 3072.0F;
static const float Q4_FOCAL_SCALE         = 16.0F;
static const float FNUM_CENTI_SCALE       = 0.01F;
static const float FNUM_DEFAULT_1P4       = 1.4F;
static const float EV_STOP_FACTOR         = 2.0F;
static const float HALF_ROUND_F           = 0.5F;
static const float TWO_F                  = 2.0F;
static const float CURVE_SQRT_WEIGHT      = 0.75F;
static const float CURVE_LIN_WEIGHT       = 0.25F;
static const float MIN_VALID_DIST_M       = 0.001F;
static const float DECIMILLIMETER_TO_MM   = 0.1F;
static const float MAG_CODE_EXP_BASE      = 5.0F;
static const float MAG_CODE_DIV           = 16.0F;
static const float DEFAULT_FOCAL_M        = 0.05F;

/* NOLINTBEGIN(readability-magic-numbers) */
static const uint8_t MF_GATE_TABLE[MF_GATE_TABLE_LEN][2] = {
    {0x00U, 0x2BU}, {0x8AU, 0x2BU}, {0x27U, 0x19U}, {0x18U, 0x11U},
    {0x11U, 0x0EU}, {0x0EU, 0x0BU}, {0x0BU, 0x0AU}, {0x0AU, 0x09U},
    {0x09U, 0x08U}, {0x08U, 0x08U}, {0x08U, 0x07U}, {0x07U, 0x07U},
    {0x07U, 0x06U}, {0x06U, 0x06U}, {0x06U, 0x05U}, {0x05U, 0x05U},
    {0x05U, 0x04U}, {0x04U, 0x04U}, {0x04U, 0x03U}, {0x03U, 0x03U},
    {0x03U, 0x02U}, {0x02U, 0x02U}, {0x02U, 0x01U}, {0x01U, 0x01U},
};
/* NOLINTEND(readability-magic-numbers) */

static int32_t clamp_s32(int32_t val, int32_t lo_val, int32_t hi_val) {
    if (val < lo_val) {
        return lo_val;
    }
    if (val > hi_val) {
        return hi_val;
    }
    return val;
}

uint16_t aperture_z_to_emount_p(uint16_t z_av_c) {
    uint32_t num = (4UL * (uint32_t)z_av_c) + 1UL;
    return (uint16_t)((num / 3UL) + E_AV_OFFSET_BASE);
}

uint16_t aperture_emount_p_to_z(uint16_t e_av_p) {
    int32_t p_hi = (int32_t)((uint32_t)e_av_p >> SHIFT_8);
    uint32_t p_lo = (uint32_t)e_av_p & BYTE_MASK;
    int32_t whole = (int32_t)Z_AV_F1_0 + ((p_hi - E_AV_F1_0_HI) * (int32_t)Z_AV_PER_EV);
    int32_t frac = (int32_t)(((p_lo * Z_AV_PER_EV) + E_AV_ROUND_HALF) >> SHIFT_8);
    return (uint16_t)clamp_s32(whole + frac, Z_AV_MIN_CLAMP, Z_AV_MAX_CLAMP);
}

uint16_t aperture_fnum_x100_to_z(uint16_t fnum_x100) {
    float fnum = (fnum_x100 > MIN_FNUM_X100)
                     ? ((float)fnum_x100 * FNUM_CENTI_SCALE)
                     : FNUM_DEFAULT_1P4;
    float ev_stops = EV_STOP_FACTOR * boryoza_log2f(fnum);
    int32_t code = (int32_t)((float)Z_AV_F1_0 + ((float)Z_AV_PER_EV * ev_stops) + HALF_ROUND_F);
    return (uint16_t)clamp_s32(code, Z_AV_LOW_F_CLAMP, Z_AV_MAX_CLAMP);
}

uint16_t exif_dist_x_to_z_0x117(uint16_t e_dist_x) {
    if (e_dist_x >= OPTICAL_INF_CODE_X) {
        return Z_DIST_INF_0X117;
    }
    int32_t raw = E_TO_Z_DIST_BASE + (E_TO_Z_DIST_SLOPE * (int32_t)e_dist_x);
    if (raw > Z_DIST_FINITE_MAX_SAFE) {
        return Z_DIST_INF_0X117;
    }
    return (uint16_t)clamp_s32(raw, Z_DIST_FINITE_MIN, Z_DIST_FINITE_MAX_SAFE);
}

static uint16_t compute_finite_step_bucket(uint16_t z_code_0x117) {
    if (z_code_0x117 >= Z_DIST_INF_0X117) {
        return 0U;
    }
    if (z_code_0x117 <= BUCKET_DIST_CLOSE) {
        return MF_BUCKETS_TOTAL;
    }
    if (z_code_0x117 <= BUCKET_DIST_1M) {
        int32_t span = (int32_t)BUCKET_DIST_1M - (int32_t)z_code_0x117;
        int32_t step = BUCKET_STEP_1M + ((span * BUCKET_STEP_1M) / BUCKET_SPAN_1M);
        return (uint16_t)clamp_s32(step, BUCKET_STEP_1M, (int32_t)MF_BUCKETS_TOTAL);
    }
    if (z_code_0x117 <= BUCKET_DIST_10M) {
        int32_t span = (int32_t)BUCKET_DIST_10M - (int32_t)z_code_0x117;
        int32_t step = BUCKET_STEP_10M + ((span * BUCKET_STEP_SPAN12) / BUCKET_SPAN_10M);
        return (uint16_t)clamp_s32(step, BUCKET_STEP_10M, BUCKET_STEP_1M);
    }
    int32_t tail = BUCKET_HYPERFOCAL - (int32_t)z_code_0x117;
    int32_t step = 1 + ((tail * BUCKET_STEP_SPAN5) / BUCKET_SPAN_TAIL);
    return (uint16_t)clamp_s32(step, 1, BUCKET_STEP_10M);
}

static bool source_distance_is_infinite(const adapter_ctx_t *ctx) {
    if (ctx->id05.dist_code_x >= OPTICAL_INF_CODE_X) {
        return true;
    }
    if ((ctx->id06.valid != 0U) && (ctx->id06.lim_near_se != ctx->id06.lim_far_se) &&
        (ctx->af.cur_cz <= 0) && (ctx->id06.at_far_end != 0U)) {
        return true;
    }
    return false;
}

static uint16_t focus_position_bucket(const adapter_ctx_t *ctx, uint16_t distance_code) {
    if ((ctx->id06.valid != 0U) && (ctx->af.mech_hz > 0)) {
        int32_t position = clamp_s32(ctx->af.cur_cz, 1, ctx->af.mech_hz);
        float norm_t = (float)position / (float)ctx->af.mech_hz;
        float sqrt_t = boryoza_exp2f(HALF_ROUND_F * boryoza_log2f(norm_t));
        float curve_u = (CURVE_SQRT_WEIGHT * sqrt_t) + (CURVE_LIN_WEIGHT * norm_t);
        int32_t bucket = (int32_t)((curve_u * (float)MF_BUCKETS_TOTAL) + HALF_ROUND_F);
        return (uint16_t)clamp_s32(bucket, 1, (int32_t)MF_BUCKETS_TOTAL);
    }
    return compute_finite_step_bucket(distance_code);
}

static int32_t compute_kinematic_dist_code(const adapter_ctx_t *ctx) {
    float d_mfd = af_servo_effective_mfd_meters(ctx);
    float d_kin = (d_mfd * (float)ctx->af.mech_hz) / (float)ctx->af.cur_cz;
    if (d_kin < MIN_VALID_DIST_M) {
        d_kin = MIN_VALID_DIST_M;
    }
    float raw_kin = LOG2_ANCHOR_1M_CODE + (LOG2_OCTAVE_CODES * boryoza_log2f(d_kin));
    return (raw_kin >= 0.0F) ? (int32_t)(raw_kin + HALF_ROUND_F) : (int32_t)(raw_kin - HALF_ROUND_F);
}

static int32_t compute_mag_dist_code(const adapter_ctx_t *ctx) {
    float beta = boryoza_exp2f(MAG_CODE_EXP_BASE - ((float)ctx->id05.mag_code / MAG_CODE_DIV));
    if (beta < MAG_BETA_MIN) {
        beta = MAG_BETA_MIN;
    } else if (beta > MAG_BETA_MAX) {
        beta = MAG_BETA_MAX;
    }
    float f_m = (ctx->f_cur_mm > 0U) ? ((float)ctx->f_cur_mm * MAG_BETA_MIN) : DEFAULT_FOCAL_M;
    float d_mag = f_m * (TWO_F + beta + (1.0F / beta));
    float raw_mag = LOG2_ANCHOR_1M_CODE + (LOG2_OCTAVE_CODES * boryoza_log2f(d_mag));
    return (raw_mag >= 0.0F) ? (int32_t)(raw_mag + HALF_ROUND_F) : (int32_t)(raw_mag - HALF_ROUND_F);
}

/* NOLINTBEGIN(bugprone-easily-swappable-parameters) */
static int32_t fuse_cam_and_kin_dist_code(adapter_ctx_t *ctx, uint16_t dist_x,
                                          int32_t cam_code, int32_t kin_code) {
/* NOLINTEND(bugprone-easily-swappable-parameters) */
    int32_t raw_diff = kin_code - cam_code;
    int32_t abs_raw_diff = (raw_diff >= 0) ? raw_diff : -raw_diff;
    int32_t pred_err = (kin_code - (int32_t)ctx->dist_cam_bias_117) - cam_code;
    int32_t abs_pred_err = (pred_err >= 0) ? pred_err : -pred_err;
    int32_t step_dx = (int32_t)dist_x - (int32_t)ctx->dist_anchor_x;
    int32_t abs_dx = (step_dx >= 0) ? step_dx : -step_dx;
    if ((ctx->dist_cam_bias_valid == 0U) || (ctx->dist_anchor_valid == 0U) ||
        (abs_dx > 1) || (abs_pred_err > DIST_REANCHOR_JUMP_CODES)) {
        bool aligned = (bool)(abs_raw_diff <= DIST_HALF_BIN_CODES);
        ctx->dist_cam_bias_valid = aligned ? DIST_BIAS_CALIBRATED : DIST_BIAS_UNCALIBRATED;
        int32_t init_bias = aligned ? 0 : raw_diff;
        ctx->dist_cam_bias_117 = (int16_t)clamp_s32(init_bias, -S16_MAX_BOUND, S16_MAX_BOUND);
    } else if (abs_dx == 1) {
        ctx->dist_cam_bias_valid = DIST_BIAS_CALIBRATED;
    }
    int32_t tube_half = (ctx->dist_cam_bias_valid >= DIST_BIAS_CALIBRATED)
                            ? DIST_TUBE_HALF_CODES
                            : DIST_UNCAL_TUBE_CODES;
    int32_t bias_lo = kin_code - (cam_code + tube_half);
    int32_t bias_hi = kin_code - (cam_code - tube_half);
    int32_t bias = clamp_s32((int32_t)ctx->dist_cam_bias_117, bias_lo, bias_hi);
    ctx->dist_cam_bias_117 = (int16_t)clamp_s32(bias, -S16_MAX_BOUND, S16_MAX_BOUND);
    ctx->dist_anchor_valid = 1U;
    ctx->dist_anchor_x = dist_x;
    ctx->dist_anchor_cz = ctx->af.cur_cz;
    ctx->dist_anchor_pending = 0U;
    return kin_code - bias;
}

static bool compute_finite_distance_code(adapter_ctx_t *ctx, int32_t *out_code) {
    uint16_t dist_x = ctx->id05.dist_code_x;
    if (dist_x >= OPTICAL_INF_CODE_X) {
        *out_code = (int32_t)Z_DIST_INF_0X117;
        ctx->dist_cam_bias_valid = 0U;
        ctx->dist_cam_bias_117 = 0;
        return false;
    }

    int32_t cam_code = E_TO_Z_DIST_BASE + (E_TO_Z_DIST_SLOPE * (int32_t)dist_x);
    bool has_kin = (bool)((ctx->id06.valid != 0U) && (ctx->af.cur_cz > 0) && (ctx->af.mech_hz > 0));
    int32_t kin_code = has_kin ? compute_kinematic_dist_code(ctx) : cam_code;

    int32_t fine_code = cam_code;
    if ((ctx->id05.grp_present[ID05_GRP_FOCAL] != 0U) && (dist_x > 0U)) {
        if (!has_kin) {
            ctx->dist_anchor_valid = 0U;
            ctx->dist_anchor_pending = 0U;
            ctx->dist_cam_bias_valid = 0U;
            ctx->dist_cam_bias_117 = 0;
        } else {
            fine_code = fuse_cam_and_kin_dist_code(ctx, dist_x, cam_code, kin_code);
        }
    } else if (has_kin) {
        fine_code = kin_code;
        ctx->dist_cam_bias_valid = 0U;
        ctx->dist_cam_bias_117 = 0;
    } else if ((ctx->id05.grp_present[ID05_GRP_FOCAL] != 0U) && (ctx->id05.mag_code != 0U) &&
               (ctx->id05.mag_code < MAG_MAX_FINITE_CODE)) {
        fine_code = compute_mag_dist_code(ctx);
    }
    *out_code = fine_code;
    return (bool)(fine_code <= Z_DIST_FINITE_MAX_SAFE);
}

static void sync_mf_and_exif_distance_gates(adapter_ctx_t *ctx) {
    aperture_exif_state_t *ap_st = &ctx->ap;
    if (ctx->dist_report_invalid != 0U) {
        bool kin_trust = (bool)((ctx->v224.dist_state == (uint8_t)DIST_STATE_DEGRADED_KIN) &&
                                (ctx->id06.valid != 0U) && (ctx->af.cur_cz > 0) &&
                                (ctx->af.mech_hz > 0) &&
                                (ctx->af.e_inf_true != ctx->af.e_near_true));
        if (kin_trust) {
            int32_t kin_code = compute_kinematic_dist_code(ctx);
            uint16_t code_117 = (uint16_t)clamp_s32(kin_code, Z_DIST_FINITE_MIN,
                                                    Z_DIST_FINITE_MAX_SAFE);
            ap_st->z_dist_0x117 = code_117;
            ap_st->z_steps_0xbb = focus_position_bucket(ctx, code_117);
            ap_st->z_alt_k_bucket = (ap_st->z_steps_0xbb != 0U) ? ap_st->z_steps_0xbb : 1U;
            uint32_t tbl_idx = 1UL + (((uint32_t)ap_st->z_alt_k_bucket *
                                       ((uint32_t)MF_GATE_TABLE_LEN - 1UL)) /
                                      (uint32_t)MF_BUCKETS_TOTAL);
            if (tbl_idx >= (uint32_t)MF_GATE_TABLE_LEN) {
                tbl_idx = (uint32_t)MF_GATE_TABLE_LEN - 1UL;
            }
            ap_st->z_mf_gate_2d = MF_GATE_TABLE[tbl_idx][0];
            ap_st->z_mf_gate_2e = MF_GATE_TABLE[tbl_idx][1];
        } else {
            ap_st->z_dist_0x117 = 0x0000U;
            ap_st->z_steps_0xbb = 0U;
            ap_st->z_alt_k_bucket = 0U;
            ap_st->z_mf_gate_2d = 0x00U;
            ap_st->z_mf_gate_2e = 0x00U;
        }
        ctx->dist_anchor_valid = 0U;
        ctx->dist_anchor_pending = 0U;
        ctx->dist_cam_bias_valid = 0U;
        ctx->dist_cam_bias_117 = 0;
        return;
    }
    bool at_infinity = source_distance_is_infinite(ctx);
    if (((ctx->id05.grp_present[ID05_GRP_FOCAL] == 0U) || (ctx->id05.dist_code_x == 0U)) &&
        (ctx->id06.valid != 0U)) {
        at_infinity = (ctx->af.cur_cz <= 0);
    }

    int32_t fine_code = 0;
    if (at_infinity || (!compute_finite_distance_code(ctx, &fine_code))) {
        ap_st->z_dist_0x117 = Z_DIST_INF_0X117;
        ap_st->z_steps_0xbb = 0U;
        ap_st->z_alt_k_bucket = 0U;
        if (at_infinity) {
            ctx->dist_anchor_valid = 0U;
            ctx->dist_cam_bias_valid = 0U;
            ctx->dist_cam_bias_117 = 0;
        }
    } else {
        uint16_t code_117 = (uint16_t)clamp_s32(fine_code,
                                                Z_DIST_FINITE_MIN,
                                                Z_DIST_FINITE_MAX_SAFE);
        ap_st->z_dist_0x117 = code_117;
        ap_st->z_steps_0xbb = focus_position_bucket(ctx, code_117);
        ap_st->z_alt_k_bucket = (ap_st->z_steps_0xbb != 0U) ? ap_st->z_steps_0xbb : 1U;
    }

    uint32_t tbl_idx = 0UL;
    if (ap_st->z_alt_k_bucket != 0U) {
        tbl_idx = 1UL + (((uint32_t)ap_st->z_alt_k_bucket *
                          ((uint32_t)MF_GATE_TABLE_LEN - 1UL)) /
                         (uint32_t)MF_BUCKETS_TOTAL);
        if (tbl_idx >= (uint32_t)MF_GATE_TABLE_LEN) {
            tbl_idx = (uint32_t)MF_GATE_TABLE_LEN - 1UL;
        }
    }
    ap_st->z_mf_gate_2d = MF_GATE_TABLE[tbl_idx][0];
    ap_st->z_mf_gate_2e = MF_GATE_TABLE[tbl_idx][1];
}

static const tamron_z_optics_profile_t *active_tamron_profile_for_ap(adapter_ctx_t *ctx) {
    const lens_db_entry_t *db_ent = lens_db_lookup_cached(ctx);
    return (db_ent != NULL) ? lens_db_get_tamron_optics(db_ent->tamron_z_idx) : NULL;
}

void aperture_controls_configure_for_lens(adapter_ctx_t *ctx) {
    aperture_exif_state_t *ap_st = &ctx->ap;
    if (ap_st->raw_p_open_08 >= E_AV_VALID_FLOOR) {
        ap_st->c_open_cur = aperture_emount_p_to_z(ap_st->raw_p_open_08);
        if (ap_st->raw_p_min_08 > ap_st->raw_p_open_08) {
            ap_st->c_min_cur = aperture_emount_p_to_z(ap_st->raw_p_min_08);
        }
    }
    if (ap_st->c_open_cur == 0U) {
        ap_st->c_open_cur = Z_AV_F2_0_FALLBACK;
    }
    if (ap_st->c_min_cur < ap_st->c_open_cur) {
        ap_st->c_min_cur = Z_AV_F22_FALLBACK;
    }
    ap_st->c_cur = ap_st->c_open_cur;
    ap_st->c_base_reported = ap_st->c_open_cur;
    ap_st->cur_stopdown_delta = 0U;
    ap_st->max_stopdown_delta = (uint16_t)(ap_st->c_min_cur - ap_st->c_open_cur);
    ap_st->body_a1_delta = 0;
    ap_st->p_target_emount = (ap_st->raw_p_open_08 >= E_AV_VALID_FLOOR)
                                 ? ap_st->raw_p_open_08
                                 : aperture_z_to_emount_p(ap_st->c_open_cur);
    ap_st->last_p_target = ap_st->p_target_emount;
    ap_st->id03_toggle = 0U;
    ap_st->z_dnorm_0x31 = 0U;
    const tamron_z_optics_profile_t *tam_prof = active_tamron_profile_for_ap(ctx);
    if ((tam_prof != NULL) && (tam_prof->u_axis_mode == OPT_U_AXIS_RATIO_FMAX) &&
        (ctx->f_max_mm > 0U) && (ctx->f_max_mm > ctx->f_min_mm)) {
        int32_t num = ((int32_t)ctx->f_cur_mm * (int32_t)NORM_COORD_MAX) + ((int32_t)ctx->f_max_mm / 2);
        ap_st->z_fnorm_0x33 = (uint16_t)clamp_s32(num / (int32_t)ctx->f_max_mm, 0, (int32_t)NORM_COORD_MAX);
    } else {
        ap_st->z_fnorm_0x33 = (ctx->f_max_mm > ctx->f_min_mm) ? 0U : NORM_COORD_MAX;
    }
    ap_st->z_eff_focal_q4 = (uint16_t)(ctx->f_cur_mm * Q4_INT_MULT);
    ap_st->z_mag_a3_q14 = 0U;
    ctx->v224.dist_last_id06_seq = 0UL;
    sync_mf_and_exif_distance_gates(ctx);
}

/* NOLINTNEXTLINE(bugprone-easily-swappable-parameters) */
static uint8_t detect_frozen_dist_encoder(adapter_ctx_t *ctx, uint16_t dist_x, int32_t cur_pm) {
    if ((ctx->dist_first_x != 0U) && (dist_x != ctx->dist_first_x) &&
        (dist_x != DIST_SENTINEL_FFFF)) {
        ctx->dist_verified_dynamic = 1U;
        ctx->v224.dist_state = (uint8_t)DIST_STATE_VALID;
        ctx->v224.dist_last_valid_raw_x = dist_x;
        return 0U;
    }
    if (dist_x == OPTICAL_INF_CODE_X) {
        return 0U;
    }
    if (ctx->dist_verified_dynamic != 0U) {
        return 0U;
    }
    if ((ctx->v224.id06_sample_seq == 0UL) ||
        (ctx->v224.id06_sample_seq == ctx->v224.dist_last_id06_seq)) {
        return 0U;
    }
    ctx->v224.dist_last_id06_seq = ctx->v224.id06_sample_seq;
    if ((ctx->dist_first_x == 0U) && (dist_x != 0U)) {
        ctx->dist_first_x = dist_x;
        ctx->v224.dist_stall_ref_se = ctx->id06.focus_pos_se;
    }
    if (ctx->dist_id05_samples == 0U) {
        ctx->dist_first_x = dist_x;
        ctx->dist_min_se = cur_pm;
        ctx->dist_max_se = cur_pm;
        ctx->dist_id05_samples = 1U;
        ctx->v224.dist_stall_ref_se = ctx->id06.focus_pos_se;
        ctx->v224.dist_last_raw_x = dist_x;
        return 0U;
    }
    if (dist_x != ctx->dist_first_x) {
        ctx->dist_verified_dynamic = 1U;
        ctx->v224.dist_state = (uint8_t)DIST_STATE_VALID;
        ctx->v224.dist_last_valid_raw_x = dist_x;
        return 0U;
    }
    bool independent_pos = (bool)((ctx->id06.focus_pos_se != ctx->id06.focus_prev_se) ||
                                  (ctx->v224.dist_last_raw_x != dist_x) ||
                                  (cur_pm != ctx->dist_min_se && cur_pm != ctx->dist_max_se));
    if (cur_pm < ctx->dist_min_se) {
        ctx->dist_min_se = cur_pm;
    }
    if (cur_pm > ctx->dist_max_se) {
        ctx->dist_max_se = cur_pm;
    }
    if (independent_pos && (ctx->dist_id05_samples < DIST_SAMPLES_CAP)) {
        ctx->dist_id05_samples = (uint8_t)(ctx->dist_id05_samples + 1U);
    }
    ctx->v224.dist_last_raw_x = dist_x;
    int32_t se_travel = ctx->id06.focus_pos_se - ctx->v224.dist_stall_ref_se;
    if (se_travel < 0) {
        se_travel = -se_travel;
    }
    bool cross_thresh = (bool)(((ctx->dist_max_se - ctx->dist_min_se) >= PM_FROZEN_TRAVEL) ||
                               (se_travel >= SE_FROZEN_MIN_TRV));
    if ((ctx->dist_id05_samples >= DIST_FROZEN_MIN_CNT) && cross_thresh &&
        (dist_x != DIST_SENTINEL_FFFF)) {
        return 3U;
    }
    if (cross_thresh) {
        ctx->v224.dist_state = (uint8_t)DIST_STATE_SUSPECT;
    }
    return 0U;
}

static void reset_frozen_window_on_zoom(adapter_ctx_t *ctx, int32_t cur_pm) {
    static const int32_t DIST_ZOOM_STEP_THRESH = 16;
    if (ctx->dist_verified_dynamic == 0U) {
        int32_t dzoom = (int32_t)ctx->id06.zoom_norm - (int32_t)ctx->dist_last_zoom;
        if (dzoom < 0) {
            dzoom = -dzoom;
        }
        bool zoom_moved = (bool)(((ctx->dist_last_f01 != 0U) &&
                                  (ctx->id05.f_nom_0p1mm != ctx->dist_last_f01)) ||
                                 ((ctx->dist_last_zoom != 0U) &&
                                  (dzoom >= DIST_ZOOM_STEP_THRESH)));
        if ((ctx->parfocal_hold_vsyncs > 0U) || zoom_moved) {
            ctx->dist_min_se = cur_pm;
            ctx->dist_max_se = cur_pm;
            ctx->dist_id05_samples = 1U;
            ctx->v224.dist_stall_ref_se = ctx->id06.focus_pos_se;
            ctx->dist_last_f01 = ctx->id05.f_nom_0p1mm;
            ctx->dist_last_zoom = ctx->id06.zoom_norm;
        } else {
            if (ctx->dist_last_f01 == 0U) {
                ctx->dist_last_f01 = ctx->id05.f_nom_0p1mm;
            }
            if (ctx->dist_last_zoom == 0U) {
                ctx->dist_last_zoom = ctx->id06.zoom_norm;
            }
        }
        return;
    }
    ctx->dist_last_f01 = ctx->id05.f_nom_0p1mm;
    ctx->dist_last_zoom = ctx->id06.zoom_norm;
}

static bool try_recover_degraded_distance(adapter_ctx_t *ctx, uint16_t dist_x) {
    bool is_sentinel = (bool)((dist_x == 0U) || (dist_x == DIST_SENTINEL_FF00) ||
                              (dist_x == DIST_SENTINEL_FFFF) ||
                              ((dist_x >= DIST_SENTINEL_7F00) && (dist_x < DIST_SENTINEL_FFFF)));
    if (is_sentinel) {
        if ((dist_x == DIST_SENTINEL_FF00) ||
            ((dist_x >= DIST_SENTINEL_7F00) && (dist_x < DIST_SENTINEL_FFFF))) {
            ctx->v224.dist_good_streak = 0U;
            ctx->v224.dist_recover_dir = 0;
        }
        ctx->v224.dist_last_raw_x = dist_x;
        return false;
    }
    if (dist_x >= OPTICAL_INF_CODE_X) {
        return false;
    }
    uint16_t prev_x = ctx->v224.dist_last_raw_x;
    ctx->v224.dist_last_raw_x = dist_x;
    if ((prev_x == 0U) || (prev_x >= OPTICAL_INF_CODE_X) || (dist_x == prev_x)) {
        return false;
    }
    int32_t d_x = (int32_t)dist_x - (int32_t)prev_x;
    int32_t abs_dx = (d_x >= 0) ? d_x : -d_x;
    if (abs_dx > RECOVER_MAX_STEP_X) {
        ctx->v224.dist_good_streak = 0U;
        ctx->v224.dist_recover_dir = 0;
        return false;
    }
    int8_t step_dir = (d_x > 0) ? 1 : -1;
    if ((ctx->v224.dist_good_streak == 0U) || (ctx->v224.dist_recover_dir != step_dir)) {
        ctx->v224.dist_recover_dir = step_dir;
        ctx->v224.dist_good_streak = 1U;
    } else if (ctx->v224.dist_good_streak < DIST_RECOVER_VOTES) {
        ctx->v224.dist_good_streak = (uint8_t)(ctx->v224.dist_good_streak + 1U);
    }
    if (ctx->v224.dist_good_streak >= DIST_RECOVER_VOTES) {
        static const uint32_t BB_DIST_RECOVER_TAG = 0x52454356UL;
        ctx->dist_report_invalid = 0U;
        ctx->dist_verified_dynamic = 1U;
        ctx->v224.dist_state = (uint8_t)DIST_STATE_VALID;
        ctx->v224.dist_good_streak = 0U;
        ctx->v224.dist_recover_dir = 0;
        ctx->v224.dist_last_valid_raw_x = dist_x;
        ctx->z_snap_cache.c10_done_mask = 0U;
        ctx->z_snap_cache.c12_done_mask = 0U;
        lens_session_push_diag(ctx, BB_EVT_V2_DIST_SUMMARY,
                               ((uint32_t)DIST_STATE_VALID) | ((uint32_t)dist_x << SHIFT_16),
                               (uint32_t)ctx->id06.focus_pos_se, BB_DIST_RECOVER_TAG,
                               (uint32_t)ctx->af.cur_cz);
        return true;
    }
    return false;
}

static uint8_t classify_distance_validity(adapter_ctx_t *ctx, uint16_t dist_x,
                                          bool id06_ready, int32_t cur_pm) {
    if ((dist_x == DIST_SENTINEL_FF00) ||
        ((dist_x >= DIST_SENTINEL_7F00) && (dist_x < DIST_SENTINEL_FFFF))) {
        return 1U;
    }
    if (dist_x == DIST_SENTINEL_FFFF) {
        return 0U;
    }
    if ((dist_x == DIST_SENTINEL_00FF) && (ctx->dist_verified_dynamic == 0U) && id06_ready &&
        (cur_pm < PM_CLOSE_FLOOR)) {
        return 2U;
    }
    if (id06_ready && (dist_x != 0U)) {
        return detect_frozen_dist_encoder(ctx, dist_x, cur_pm);
    }
    if ((ctx->dist_first_x != 0U) && (dist_x != 0U) && (dist_x != ctx->dist_first_x)) {
        ctx->dist_verified_dynamic = 1U;
        ctx->v224.dist_state = (uint8_t)DIST_STATE_VALID;
        ctx->v224.dist_last_valid_raw_x = dist_x;
    } else if ((ctx->dist_first_x == 0U) && (dist_x != 0U)) {
        ctx->dist_first_x = dist_x;
    }
    return 0U;
}

static void record_distance_invalid(adapter_ctx_t *ctx, uint8_t bad_reason,
                                    uint16_t dist_x, bool id06_ready) {
    ctx->dist_report_invalid = bad_reason;
    ctx->v224.dist_state = (id06_ready && (ctx->af.e_inf_true != ctx->af.e_near_true))
                               ? (uint8_t)DIST_STATE_DEGRADED_KIN
                               : (uint8_t)DIST_STATE_INVALID;
    ctx->v224.dist_good_streak = 0U;
    ctx->v224.dist_recover_dir = 0;
    ctx->z_snap_cache.c10_done_mask = 0U;
    ctx->z_snap_cache.c12_done_mask = 0U;
    uint32_t diag_a0 = ((uint32_t)bad_reason) |
                       ((uint32_t)ctx->id05.mag_code << SHIFT_8) |
                       ((uint32_t)dist_x << SHIFT_16);
    uint32_t diag_a2 = ((uint32_t)(uint16_t)ctx->af.e_inf_true) |
                       ((uint32_t)(uint16_t)ctx->af.e_near_true << SHIFT_16);
    lens_session_push_diag(ctx, BB_EVT_DIST_FALLBACK, diag_a0,
                           (uint32_t)ctx->id06.focus_pos_se, diag_a2,
                           (uint32_t)ctx->af.cur_cz);
}

void aperture_controls_validate_distance(adapter_ctx_t *ctx) {
    if ((ctx->id05.grp_present[ID05_GRP_FOCAL] == 0U) ||
        (ctx->session_state == E_SESSION_EMPTY_MOUNT) ||
        (ctx->session_state == E_SESSION_PASSIVE_MOUNT)) {
        return;
    }
    uint16_t dist_x = ctx->id05.dist_code_x;
    bool id06_ready = (bool)((ctx->id06.valid != 0U) && (ctx->af.mech_hz > 0) &&
                             (ctx->id06.lim_near_se != ctx->id06.lim_far_se));
    if (ctx->dist_report_invalid != 0U) {
        (void)try_recover_degraded_distance(ctx, dist_x);
        if (ctx->dist_report_invalid != 0U) {
            ctx->v224.dist_state = (id06_ready && (ctx->af.e_inf_true != ctx->af.e_near_true))
                                       ? (uint8_t)DIST_STATE_DEGRADED_KIN
                                       : (uint8_t)DIST_STATE_INVALID;
            return;
        }
    }
    int32_t cur_pm = 0;
    if (id06_ready) {
        int32_t safe_cz = clamp_s32(ctx->af.cur_cz, 0, SAFE_CZ_MAX_256K);
        cur_pm = clamp_s32((safe_cz * PERMILLE_SCALE_1024) / ctx->af.mech_hz, 0, PERMILLE_SCALE_1024);
        reset_frozen_window_on_zoom(ctx, cur_pm);
    }

    uint8_t bad_reason = classify_distance_validity(ctx, dist_x, id06_ready, cur_pm);
    ctx->v224.dist_last_raw_x = dist_x;
    if (bad_reason != 0U) {
        record_distance_invalid(ctx, bad_reason, dist_x, id06_ready);
    }
}

static void note_id03_target_change(aperture_exif_state_t *ap_st) {
    if (ap_st->p_target_emount != ap_st->last_p_target) {
        ap_st->last_p_target = ap_st->p_target_emount;
        ap_st->id03_toggle = (uint8_t)((uint32_t)ap_st->id03_toggle ^ 1UL);
    }
}

static void refresh_live_emount_aperture(adapter_ctx_t *ctx) {
    aperture_exif_state_t *ap_st = &ctx->ap;
    if (ap_st->raw_p_open_08 >= E_AV_VALID_FLOOR) {
        int32_t delta_p = (ctx->id05.grp_present[ID05_GRP_ABERR] != 0U) ? (int32_t)ctx->id05.delta_av : 0;
        int32_t live_open = clamp_s32((int32_t)ap_st->raw_p_open_08 + delta_p,
                                      (int32_t)E_AV_VALID_FLOOR, E_AV_MAX_CLAMP);
        ctx->id05.p_open = (uint16_t)live_open;
        if (ap_st->raw_p_min_08 > ctx->id05.p_open) {
            ctx->id05.p_min = ap_st->raw_p_min_08;
        }
    }
    if (ctx->id05.p_open >= E_AV_VALID_FLOOR) {
        ap_st->c_open_cur = aperture_emount_p_to_z(ctx->id05.p_open);
        if (ctx->id05.p_min > ctx->id05.p_open) {
            ap_st->c_min_cur = aperture_emount_p_to_z(ctx->id05.p_min);
        }
    }
    if ((ctx->id05.grp_present[0] != 0U) && (ctx->id05.p_cur >= E_AV_VALID_FLOOR)) {
        ap_st->c_cur = aperture_emount_p_to_z(ctx->id05.p_cur);
    }
}

static void update_focal_and_zoom_norm(adapter_ctx_t *ctx) {
    aperture_exif_state_t *ap_st = &ctx->ap;
    if ((ctx->id05.grp_present[ID05_GRP_FOCAL] != 0U) && (ctx->id05.f_nom_0p1mm >= MIN_FOCAL_0P1MM)) {
        ctx->f_cur_mm = (uint16_t)((ctx->id05.f_nom_0p1mm + FOCAL_ROUND_HALF) / FOCAL_DIV_10);
    } else if ((ctx->f_max_mm > ctx->f_min_mm) && (ctx->id06.zoom_norm >= ZOOM_NORM_WIDE)) {
        int32_t span_mm = (int32_t)ctx->f_max_mm - (int32_t)ctx->f_min_mm;
        int32_t num = ((int32_t)ctx->id06.zoom_norm - (int32_t)ZOOM_NORM_WIDE) * span_mm;
        int32_t den = (int32_t)ZOOM_NORM_TELE - (int32_t)ZOOM_NORM_WIDE;
        ctx->f_cur_mm = (uint16_t)clamp_s32((int32_t)ctx->f_min_mm + (num / den),
                                            (int32_t)ctx->f_min_mm, (int32_t)ctx->f_max_mm);
    }

    if (ctx->af.mech_hz > 0) {
        int32_t dnorm = (ctx->af.cur_cz * (int32_t)NORM_COORD_MAX) / ctx->af.mech_hz;
        ap_st->z_dnorm_0x31 = (uint16_t)clamp_s32(dnorm, 0, (int32_t)NORM_COORD_MAX);
    } else {
        ap_st->z_dnorm_0x31 = 0U;
    }

    const tamron_z_optics_profile_t *tam_prof = active_tamron_profile_for_ap(ctx);
    int32_t znorm = (int32_t)NORM_COORD_MAX;
    if (ctx->f_max_mm > ctx->f_min_mm) {
        bool have_f_nom = (bool)((ctx->id05.grp_present[ID05_GRP_FOCAL] != 0U) &&
                                 (ctx->id05.f_nom_0p1mm >= MIN_FOCAL_0P1MM));
        int32_t f_cur_0p1 = have_f_nom
                                ? (int32_t)ctx->id05.f_nom_0p1mm
                                : ((int32_t)ctx->f_cur_mm * (int32_t)FOCAL_DIV_10);
        if ((tam_prof != NULL) && (tam_prof->u_axis_mode == OPT_U_AXIS_RATIO_FMAX)) {
            int32_t f_max_0p1 = (int32_t)ctx->f_max_mm * (int32_t)FOCAL_DIV_10;
            znorm = ((f_cur_0p1 * (int32_t)NORM_COORD_MAX) + (f_max_0p1 / 2)) / f_max_0p1;
        } else if ((tam_prof != NULL) && have_f_nom) {
            int32_t f_min_0p1 = (int32_t)ctx->f_min_mm * (int32_t)FOCAL_DIV_10;
            int32_t span_0p1 = ((int32_t)ctx->f_max_mm - (int32_t)ctx->f_min_mm) * (int32_t)FOCAL_DIV_10;
            znorm = (((f_cur_0p1 - f_min_0p1) * (int32_t)NORM_COORD_MAX) + (span_0p1 / 2)) / span_0p1;
        } else if (ctx->id06.zoom_norm >= ZOOM_NORM_WIDE) {
            znorm = (((int32_t)ctx->id06.zoom_norm - (int32_t)ZOOM_NORM_WIDE) *
                     (int32_t)NORM_COORD_MAX) /
                    ((int32_t)ZOOM_NORM_TELE - (int32_t)ZOOM_NORM_WIDE);
        } else if (have_f_nom) {
            int32_t f_min_0p1 = (int32_t)ctx->f_min_mm * (int32_t)FOCAL_DIV_10;
            int32_t span_0p1 = ((int32_t)ctx->f_max_mm - (int32_t)ctx->f_min_mm) * (int32_t)FOCAL_DIV_10;
            znorm = (((f_cur_0p1 - f_min_0p1) * (int32_t)NORM_COORD_MAX) + (span_0p1 / 2)) / span_0p1;
        } else {
            znorm = 0;
        }
    }
    ap_st->z_fnorm_0x33 = (uint16_t)clamp_s32(znorm, 0, (int32_t)NORM_COORD_MAX);
}

void aperture_controls_update_from_id05(adapter_ctx_t *ctx, const optics_workspace_t *optics) {
    aperture_exif_state_t *ap_st = &ctx->ap;
    update_focal_and_zoom_norm(ctx);

    float f_opt_mm = (ctx->id05.f_opt_0p1mm >= MIN_FOCAL_0P1MM)
                         ? ((float)ctx->id05.f_opt_0p1mm * DECIMILLIMETER_TO_MM)
                         : (float)ctx->f_cur_mm;
    float f_breath_mm = optics_eval_sony_ds13_focal_mm(ctx, optics, f_opt_mm);
    int32_t q4_focal = (int32_t)((f_breath_mm * Q4_FOCAL_SCALE) + HALF_ROUND_F);
    ap_st->z_eff_focal_q4 = (uint16_t)clamp_s32(q4_focal, Q4_MIN_CLAMP, Q4_MAX_CLAMP);

    refresh_live_emount_aperture(ctx);

    bool ring_manual = false;
    if ((ctx->id05.grp_present[ID05_GRP_RING] != 0U) &&
        (ctx->id05.manual_ring_active != 0U) &&
        (ctx->id05.p_ring >= E_AV_VALID_FLOOR)) {
        ring_manual = true;
    }
    if (ring_manual) {
        uint16_t c_ring = aperture_emount_p_to_z(ctx->id05.p_ring);
        c_ring = (uint16_t)clamp_s32((int32_t)c_ring,
                                     (int32_t)ap_st->c_open_cur,
                                     (int32_t)ap_st->c_min_cur);
        ap_st->c_base_reported = c_ring;
        ap_st->c_cur = c_ring;
        ap_st->cur_stopdown_delta = (uint16_t)clamp_s32((int32_t)c_ring - (int32_t)ap_st->c_open_cur,
                                                        0, Z_DELTA_MAX_POS);
        ap_st->max_stopdown_delta = 0U;
        ap_st->p_target_emount = ctx->id05.p_ring;
    } else {
        ap_st->c_base_reported = ap_st->c_open_cur;
        int32_t target_c = clamp_s32((int32_t)ap_st->c_open_cur + (int32_t)ap_st->body_a1_delta,
                                     (int32_t)ap_st->c_open_cur,
                                     (int32_t)ap_st->c_min_cur);
        if ((ap_st->body_a1_delta == 0) && (ctx->id05.p_open >= E_AV_VALID_FLOOR)) {
            ap_st->p_target_emount = ctx->id05.p_open;
        } else {
            ap_st->p_target_emount = aperture_z_to_emount_p((uint16_t)target_c);
        }
        ap_st->c_cur = (uint16_t)target_c;
        int32_t cur_d = target_c - (int32_t)ap_st->c_open_cur;
        ap_st->cur_stopdown_delta = (uint16_t)clamp_s32(cur_d, 0, Z_DELTA_MAX_POS);
        int32_t max_d = (int32_t)ap_st->c_min_cur - (int32_t)ap_st->c_open_cur;
        ap_st->max_stopdown_delta = (uint16_t)clamp_s32(max_d, 0, Z_DELTA_MAX_POS);
    }
    note_id03_target_change(ap_st);

    ap_st->z_mag_a3_q14 = optics_compute_a3_q14(ctx, optics);
    sync_mf_and_exif_distance_gates(ctx);
}

void aperture_controls_on_z_a1(adapter_ctx_t *ctx, const uint8_t *mosi, size_t mosi_len) {
    if (mosi_len < 3U) {
        return;
    }
    /* 16-bit signed stopdown delta in 1/192 EV units at mosi[1..2] */
    uint16_t raw = (uint16_t)((uint32_t)mosi[1] | ((uint32_t)mosi[2] << SHIFT_8));
    ctx->ap.body_a1_delta = (int16_t)raw;

    if (ctx->id05.manual_ring_active == 0U) {
        int32_t target_c = clamp_s32((int32_t)ctx->ap.c_open_cur + (int32_t)ctx->ap.body_a1_delta,
                                     (int32_t)ctx->ap.c_open_cur,
                                     (int32_t)ctx->ap.c_min_cur);
        if ((ctx->ap.body_a1_delta == 0) && (ctx->id05.p_open >= E_AV_VALID_FLOOR)) {
            ctx->ap.p_target_emount = ctx->id05.p_open;
        } else {
            ctx->ap.p_target_emount = aperture_z_to_emount_p((uint16_t)target_c);
        }
        ctx->ap.c_cur = (uint16_t)target_c;
        ctx->ap.cur_stopdown_delta = (uint16_t)(target_c - (int32_t)ctx->ap.c_open_cur);
        note_id03_target_change(&ctx->ap);
    }
}

void aperture_controls_fill_z_a0(const adapter_ctx_t *ctx, uint8_t sub, uint8_t *out_miso24) {
    /* 20-byte iris actuator response profile + 16-bit current stopdown delta */
    static const uint8_t A0_IRIS_PROFILE_20B[20] = {
        0x04U, 0x04U, 0x19U, 0x12U, 0x17U, 0x1CU, 0x20U, 0x06U, 0x03U, 0x02U,
        0x03U, 0x03U, 0x03U, 0x06U, 0x06U, 0x06U, 0x06U, 0x06U, 0x01U, 0x01U,
    };
    static const size_t A0_OFF_CUR_DELTA_LO = 21U;
    static const size_t A0_OFF_CUR_DELTA_HI = 22U;
    (void)sub;
    memset(out_miso24, 0, Z_A0_RESP_BYTES);
    memcpy(&out_miso24[1], A0_IRIS_PROFILE_20B, sizeof(A0_IRIS_PROFILE_20B));
    out_miso24[A0_OFF_CUR_DELTA_LO] = (uint8_t)((uint32_t)ctx->ap.cur_stopdown_delta & BYTE_MASK);
    out_miso24[A0_OFF_CUR_DELTA_HI] = (uint8_t)(((uint32_t)ctx->ap.cur_stopdown_delta >> SHIFT_8) & BYTE_MASK);
    boryoza_seal_z_frame(out_miso24, Z_A0_RESP_BYTES);
}

void aperture_controls_on_z_mosi20(adapter_ctx_t *ctx, const uint8_t *mosi,
                                   size_t mosi_len, const uint64_t *now_ticks_ptr) {
    uint64_t now_ticks = (now_ticks_ptr != NULL) ? *now_ticks_ptr : 0ULL;
    vr_controller_on_z_mosi20(ctx, mosi, mosi_len);
    size_t copy_n = mosi_len;
    if (copy_n > 1U) {
        copy_n -= 1U;
    }
    if (copy_n > (size_t)Z_MOSI20_PREFIX_BYTES) {
        copy_n = (size_t)Z_MOSI20_PREFIX_BYTES;
    }
    memcpy(ctx->z_mosi20_retained, mosi, copy_n);
    if (mosi_len >= MOSI20_TUNING_MIN) {
        ctx->ap.tuning_ref_2000 = (uint16_t)((uint32_t)mosi[MOSI20_TUNING_OFF] |
                                             ((uint32_t)mosi[MOSI20_TUNING_OFF + 1U] << SHIFT_8));
    }
    if (((ctx->session_state == E_SESSION_EMPTY_MOUNT) ||
         (ctx->session_state == E_SESSION_PASSIVE_MOUNT)) &&
        (now_ticks > ctx->last_ring_hb_ticks) &&
        ((now_ticks - ctx->last_ring_hb_ticks) >= RING_HB_PERIOD_TKS)) {
        ctx->last_ring_hb_ticks = now_ticks;
        ctx->z_ring_heartbeat = (uint8_t)(ctx->z_ring_heartbeat + 1U);
    }
    vr_controller_refresh_state(ctx);
}
