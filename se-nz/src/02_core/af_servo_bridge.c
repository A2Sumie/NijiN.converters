#include "../../include/boryoza_types.h"
#include "../../include/boryoza_math_mem.h"
#include "../../include/lens_db.h"

static const uint16_t FOCAL_TIER_MID_MM       = 200U;
static const uint16_t FOCAL_TIER_TELE_MM      = 400U;
static const uint8_t  SCALE_EXP_SHORT         = 7U;
static const uint8_t  SCALE_EXP_MID           = 9U;
static const uint8_t  SCALE_EXP_TELE          = 10U;
static const int32_t  MECH_LZ_DEFAULT         = -512;
static const int32_t  MIN_HZ_FLOOR            = 256;
static const int32_t  MIN_SPAN_TICKS          = 16;
static const int32_t  DEFAULT_SPAN_TICKS      = 16384;
static const int32_t  MAX_U16_CLAMP           = 65535;
static const int32_t  MAX_3C_VEL_CLAMP        = 32000;
static const uint16_t SIGMA_150600_REV2       = 0xC56EU;
static const uint16_t SIGMA_150600_REV1       = 0xC55FU;
static const uint16_t SIGMA_105_MACRO_ID      = 0xC55AU;
static const uint16_t SIGMA_500_ID            = 0xC572U;
static const uint16_t SIGMA_35_II_ID          = 0xC583U;
static const uint16_t SIGMA_70_MACRO_ID       = 0xC551U;
static const uint16_t LEGACY_ADAPTER_ID       = 0x0007U;
static const uint16_t DEFAULT_ZOOM_WIDE       = 0x0100U;
static const uint8_t  Z_UNLIMITED_SPD_LO      = 0xFFU;
static const uint8_t  Z_UNLIMITED_SPD_HI      = 0x3FU;
static const uint32_t Z_UNUSED_SPD_HI_MASK    = 0x7EUL;
static const uint32_t Z_UNUSED_SPD_HI_TAG     = 0x3EUL;
static const uint32_t Z_SENTINEL_WALL_CZ      = 0x7FFFFFFFUL;
static const uint32_t Z_SPD_EVEN_MASK         = 0xFEUL;
static const int32_t  Z_SPD_EXP_MIN_S32       = -7;
static const int32_t  Z_SPD_EXP_MAX_S32       = 24;
static const int32_t  Z_SPD_EXP_BIAS_S32      = 9;
static const uint8_t  KZ_MAX_SHIFT_BITS       = 22U;
static const int32_t  KZ_MANT_OFFSET          = 256;
static const int32_t  KZ_MANT_MAX_BYTE        = 255;
static const uint8_t  SETTLE_HOLD_FRAMES      = 2U;
static const uint8_t  STALL_ABORT_HOT_FRAMES  = 48U;
static const uint8_t  MF_RING_HOLD_FRAMES     = 5U;
static const uint8_t  STALL_GUARD_VSYNCS_60HZ = 14U;
static const uint8_t  STALL_GUARD_VSYNCS_120HZ = 28U;
static const uint8_t  PRE_STOP_MAX_VSYNCS     = 3U;
static const uint8_t  E0_BRAKE_MAX_VSYNCS     = 2U;
static const uint8_t  MIN_CUSHION_DRIVE_VSYNCS = 2U;
static const uint8_t  S1_HOLD_VSYNCS_60HZ     = 9U;
static const uint8_t  S1_HOLD_VSYNCS_120HZ    = 18U;
static const uint8_t  FAST_VSYNC_THRESH_HZ    = 120U;
static const uint64_t AF_STALL_TIMEOUT_TICKS  = 900000ULL; /* 300 ms */
static const uint8_t  HOT6_POS_MOVING         = 0x1DU;
static const uint8_t  HOT6_NEG_MOVING         = 0x1BU;
static const uint8_t  HOT6_E0_SETTLED         = 0x06U;
static const uint8_t  HOT6_STALL_ABORT        = 0x26U;
static const uint8_t  HOT6_IDLE_SETTLED       = 0x06U;
static const uint8_t  HOT6_MF_RING_TURN       = 0x0EU;
static const uint8_t  HOT6_MF_STATUS_BIT      = 0x80U;
static const uint8_t  HOT6_DRIVE_RESULT_MASK  = 0x30U;
static const uint8_t  ID04_B9_IDLE_MF         = 0x81U;
static const uint8_t  ID04_B9_SERVO_ON        = 0x83U;
static const uint8_t  ID04_B16_SYNC_ZERO      = 0x00U;
static const uint8_t  ID04_B16_APSC_STOP      = 0x41U;
static const uint8_t  ID04_B16_FF_DRIVE       = 0x49U;
static const uint8_t  E_CAP_CMD_3C_SCAN       = 0x3CU;
static const size_t   MOSI20_RING1_MODE_OFF   = 0x2BU;
static const size_t   MOSI20_RING2_MODE_OFF   = 0x2CU;
static const uint8_t  MOSI20_FOCUS_MODE_MF    = 0x02U;
static const uint32_t MOSI20_MF_FLAGS_MASK    = 0x06UL;
static const uint32_t HOT7_NEAR_LIMIT_BIT     = 0x01UL;
static const uint32_t HOT7_FAR_LIMIT_BIT      = 0x02UL;
static const uint64_t EXTRAP_MAX_TICKS        = 36000ULL;
static const int32_t  STD_CAPTURE_TOL         = 1;
static const int32_t  MIN_1D_STEP_SE          = 16;
static const int32_t  SONY_XD_SWEEP_MIN_STEP_SE = 32;
static const int32_t  TRACK_PULSE_MIN_SE      = 10;
static const int32_t  FAM2_DECEL_BAND         = 3;
static const int32_t  MF_ENCODER_DELTA_MIN    = 4;
static const int32_t  LIMITER_SHRINK_THRESH   = 400;
static const uint8_t  DEFAULT_ACTIVE_FPS      = 60U;
static const uint32_t SEQ_3C_MASK             = 0x03UL;
static const uint32_t BYTE_MASK               = 0xFFUL;
static const uint32_t SHIFT_8                 = 8U;
static const uint32_t SHIFT_16                = 16U;
static const uint32_t SHIFT_24                = 24U;

static const float SCALE_A_SHORT              = 16384.0F;
static const float SCALE_A_MID                = 65536.0F;
static const float SCALE_A_TELE               = 131072.0F;
static const float DEFAULT_MFD_METERS         = 0.50F;
static const float MIN_MFD_FLOOR_M            = 0.08F;
static const float MM_TO_METERS               = 0.001F;
static const float FAR_INF_DIST_M             = 1000.0F;
static const float MIN_JACOBIAN_GUARD         = 0.001F;
static const float DEFAULT_VMAX_SE_TPS        = 60000.0F;
static const float SONY_XD_VMAX_SE_TPS        = 150000.0F;
static const float FAST_JUMP_CZ_VPPS          = 65536.0F;
static const float DUAL_FLOAT_BETA_THRESH     = 0.25F;
static const float DUAL_FLOAT_VEL_MARGIN      = 0.80F;
static const float FAM2_DECEL_GAIN            = 0.33333334F;
static const float KZ_BASE_MANTISSA           = 256.0F;
static const float KZ_Q16_DOUBLE_SCALE        = 131072.0F;
static const float KZ_MAX_MANTISSA            = 511.0F;
static const float HALF_SCALE_F               = 0.5F;
static const float DOUBLE_SCALE_F             = 2.0F;
static const float SPEED_MANTISSA_BASE        = 256.0F;
static const float Q14_SCALE_F                = 16384.0F;
static const float MACRO_105_SCALE_COMP       = 0.5F;
static const float PARFOCAL_NODAL_SCALE       = 0.5F;
static const float PARFOCAL_NET_MFD_MIN       = 0.02F;
static const float DECIMM_TO_MM_F             = 0.1F;
static const uint16_t MIN_FOCAL_DECIMM        = 40U;
static const uint16_t DECIMM_PER_MM           = 10U;
static const uint8_t  PARFOCAL_HOLD_VSYNCS    = 15U;
static const uint8_t  PARFOCAL_HOLD_VSYNCS_120 = 30U;
static const uint8_t  PARFOCAL_POST_SLEW_60   = 10U;
static const uint8_t  PARFOCAL_POST_SLEW_120  = 20U;
static const uint8_t  PARFOCAL_CL_COOLDOWN    = 2U;
static const uint8_t  PARFOCAL_CL_COOLDOWN_120 = 4U;
static const uint8_t  PARFOCAL_ANCHOR_REFR    = 3U;
static const uint8_t  PARFOCAL_E0_COAST_MAX   = 16U;
static const int32_t  PARFOCAL_CMD_SETTLE_MIN = 12;
static const int32_t  PARFOCAL_ZOOM_STEP_DIFF = 16;
static const int32_t  PARFOCAL_F01_DIFF_MIN   = 25;
static const uint16_t PARFOCAL_ZOOM_WIDE_U16  = 0x0100U;
static const uint16_t PARFOCAL_ZOOM_TELE_U16  = 0x3F00U;
static const uint16_t PARFOCAL_INF_CODE_X     = 0x0700U;
static const uint16_t PARFOCAL_MAX_VALID_X    = 0x7F00U;
static const uint16_t PARFOCAL_MIN_F01        = 50U;
static const uint16_t PARFOCAL_F01_HALF       = 5U;
static const uint16_t PARFOCAL_F01_DIV        = 10U;
static const uint8_t  MAX_PARFOCAL_CL_STEPS   = 6U;
static const int32_t  PARFOCAL_ANCHOR_CZ_TOL  = 64;
static const int32_t  PARFOCAL_CL_MAX_BINS    = 64;
static const int32_t  PARFOCAL_CL_ERR_DB      = 1;
static const int32_t  PARFOCAL_MIN_CL_SE_STEP = 2;
static const int32_t  PARFOCAL_CONT_STEP_DIV  = 32;
static const float    PARFOCAL_CL_GAIN        = 0.75F;
static const float    PARFOCAL_CONT_CL_DAMP   = 0.5F;
static const float    PARFOCAL_BINS_INV_64    = 0.015625F;
static const float    PARFOCAL_DIST_X_REF     = 384.0F;

static int32_t read_s32_le(const uint8_t *buf) {
    uint32_t raw = (uint32_t)buf[0] |
                   ((uint32_t)buf[1] << SHIFT_8) |
                   ((uint32_t)buf[2] << SHIFT_16) |
                   ((uint32_t)buf[3] << SHIFT_24);
    return (int32_t)raw;
}

static void write_s32_le(uint8_t *buf, int32_t val) {
    uint32_t uval = (uint32_t)val;
    buf[0] = (uint8_t)(uval & BYTE_MASK);
    buf[1] = (uint8_t)((uval >> SHIFT_8) & BYTE_MASK);
    buf[2] = (uint8_t)((uval >> SHIFT_16) & BYTE_MASK);
    buf[3] = (uint8_t)((uval >> SHIFT_24) & BYTE_MASK);
}

static int32_t clamp_s32(int32_t val, int32_t lo_val, int32_t hi_val) {
    if (val < lo_val) {
        return lo_val;
    }
    if (val > hi_val) {
        return hi_val;
    }
    return val;
}

static int32_t round_f32_to_s32(float val) {
    return (val >= 0.0F) ? (int32_t)(val + HALF_SCALE_F) : (int32_t)(val - HALF_SCALE_F);
}

uint16_t af_servo_encode_kz_pair(float kz_val) {
    static const uint16_t KZ_MIN_ENCODED = 0x0100U;
    float target_x2 = kz_val * KZ_Q16_DOUBLE_SCALE;
    if (!(target_x2 >= KZ_BASE_MANTISSA)) {
        return KZ_MIN_ENCODED;
    }
    uint32_t shift = 0UL;
    while ((target_x2 > KZ_MAX_MANTISSA) && (shift < (uint32_t)KZ_MAX_SHIFT_BITS)) {
        target_x2 *= HALF_SCALE_F;
        shift += 1UL;
    }
    while ((target_x2 < KZ_BASE_MANTISSA) && (shift > 0UL)) {
        target_x2 *= DOUBLE_SCALE_F;
        shift -= 1UL;
    }
    int32_t mant = round_f32_to_s32(target_x2) - KZ_MANT_OFFSET;
    mant = clamp_s32(mant, 0, KZ_MANT_MAX_BYTE);
    return (uint16_t)((uint32_t)mant | (shift << SHIFT_8));
}

static float calibrated_emount_k(const adapter_ctx_t *ctx, uint16_t raw_k) {
    static const uint16_t AV_ONE = 4096U;
    static const uint16_t AV_VALID_MIN = 3840U;
    static const uint32_t BODY_DEFOCUS_SCALE = 1792UL;
    static const uint32_t K_CAL_NUMERATOR = 9UL;
    static const uint32_t K_CAL_DENOMINATOR = 10UL;
    static const float AV_PER_FNUM_OCTAVE = 512.0F;
    static const float FNUM_Q8_EXPONENT = 8.0F;
    static const float K_SCALE_DIVISOR = 64.0F;
    static const float K_ZERO_FALLBACK = 5.015625F;
    static const float K_QUANTIZED_ZERO_FALLBACK = 0.15673828125F;
    static const uint32_t F2_FALLBACK_Q8 = 512UL;
    if (raw_k == 0U) {
        return K_ZERO_FALLBACK;
    }
    uint16_t aperture_av = ctx->id05.p_cur;
    if (aperture_av < AV_VALID_MIN) {
        aperture_av = ctx->ap.raw_p_open_08;
    }
    if ((aperture_av < AV_VALID_MIN) && (ctx->ap.c_open_cur != 0U)) {
        aperture_av = aperture_z_to_emount_p(ctx->ap.c_open_cur);
    }
    uint32_t fnum_q8 = F2_FALLBACK_Q8;
    if (aperture_av >= AV_VALID_MIN) {
        uint16_t bounded_av = (aperture_av < AV_ONE) ? AV_ONE : aperture_av;
        fnum_q8 = (uint32_t)boryoza_exp2f(
            ((float)(bounded_av - AV_ONE) / AV_PER_FNUM_OCTAVE) + FNUM_Q8_EXPONENT);
    }
    uint32_t divisor = (fnum_q8 * BODY_DEFOCUS_SCALE) >> SHIFT_8;
    uint32_t normalized = ((uint32_t)raw_k << SHIFT_16) / divisor;
    normalized = (normalized * K_CAL_NUMERATOR) / K_CAL_DENOMINATOR;
    if (normalized == 0UL) {
        return K_QUANTIZED_ZERO_FALLBACK;
    }
    return (float)normalized / K_SCALE_DIVISOR;
}

float af_servo_decode_z_speed_pps(uint8_t spd_lo, bool *out_unlimited,
                                  uint8_t spd_hi, int8_t *out_dir) {
    if ((spd_lo == Z_UNLIMITED_SPD_LO) && (spd_hi == Z_UNLIMITED_SPD_HI)) {
        if (out_unlimited != NULL) {
            *out_unlimited = true;
        }
        if (out_dir != NULL) {
            *out_dir = 1;
        }
        return 0.0F;
    }
    if (out_unlimited != NULL) {
        *out_unlimited = false;
    }
    int8_t dir = (((uint32_t)spd_hi & 1UL) != 0UL) ? -1 : 1;
    if (out_dir != NULL) {
        *out_dir = dir;
    }
    if ((spd_lo == 0U) && (spd_hi == 0U)) {
        return 0.0F;
    }
    int32_t even_s8 = (int32_t)(int8_t)(uint8_t)((uint32_t)spd_hi & Z_SPD_EVEN_MASK);
    int32_t exp7 = clamp_s32(even_s8 / 2, Z_SPD_EXP_MIN_S32, Z_SPD_EXP_MAX_S32);
    return (SPEED_MANTISSA_BASE + (float)spd_lo) *
           boryoza_exp2f((float)(exp7 - Z_SPD_EXP_BIAS_S32));
}

static bool apply_bound_optical_endpoints(adapter_ctx_t *ctx, bool initialize) {
    static const int32_t WIRE_500_FAR = 2480;
    static const int32_t WIRE_500_NEAR = 20103;
    static const int32_t WIRE_35_FAR = 2396;
    static const int32_t WIRE_35_NEAR = 9059;
    int32_t wire_far = 0;
    int32_t wire_near = 0;
    if (ctx->e_lens_id == SIGMA_500_ID) {
        wire_far = WIRE_500_FAR;
        wire_near = WIRE_500_NEAR;
    } else if (ctx->e_lens_id == SIGMA_35_II_ID) {
        wire_far = WIRE_35_FAR;
        wire_near = WIRE_35_NEAR;
    } else if (ctx->e_lens_id != SIGMA_70_MACRO_ID) {
        return false;
    }
    const lens_db_entry_t *entry = lens_db_lookup_cached(ctx);
    if (entry == NULL) {
        return false;
    }
    if (initialize || (ctx->e_lens_id == SIGMA_70_MACRO_ID)) {
        ctx->af.e_inf_true = entry->cam_e_inf;
        ctx->af.e_near_true = entry->cam_e_near;
    }
    if ((ctx->id06.valid != 0U) && (ctx->e_lens_id != SIGMA_70_MACRO_ID)) {
        int32_t far_delta = ctx->id06.lim_far_se - wire_far;
        int32_t near_delta = ctx->id06.lim_near_se - wire_near;
        if (far_delta == near_delta) {
            ctx->af.e_inf_true = entry->cam_e_inf + far_delta;
            ctx->af.e_near_true = entry->cam_e_near + near_delta;
        }
    }
    return true;
}

static const int32_t C56E_CAM_FRAC_NUM[E_CURVE_NODES_COUNT] = {0, 1, 2, 4, 8, 16, 24, 32};
static const int64_t C56E_KNOT_HALF   = 16LL;
static const int64_t C56E_KNOT_DENOM  = 32LL;
static const int32_t C56E_PAST_DEF    = 1200;
static const int32_t C56E_LZ_DEF_MAG  = 512;
static const size_t  C56E_LAST_SEG    = 6U;
static const int32_t C56E_JAC_STEP    = 64;
static const int32_t ENDPOINT_HYST_SE = 2;

static int32_t c56e_knot_cz(int32_t mech_hz, size_t idx) {
    return (int32_t)((((int64_t)mech_hz * (int64_t)C56E_CAM_FRAC_NUM[idx]) + C56E_KNOT_HALF) /
                     C56E_KNOT_DENOM);
}

static int32_t c56e_se_to_cz_unclamped(const sigma_150600_zoom_eval_t *zeval,
                                       const af_servo_state_t *servo, int32_t pos_se) {
    int32_t se0 = zeval->cam_se_fracs[0];
    if (pos_se < se0) {
        int32_t past_span = se0 - zeval->e_far_mech;
        if (past_span <= 0) {
            past_span = C56E_PAST_DEF;
        }
        return round_f32_to_s32(((float)(pos_se - se0) * (float)(-servo->mech_lz)) / (float)past_span);
    }
    size_t seg = C56E_LAST_SEG;
    for (size_t idx = 0U; idx < C56E_LAST_SEG; ++idx) {
        if (pos_se <= zeval->cam_se_fracs[idx + 1U]) {
            seg = idx;
            break;
        }
    }
    int32_t se_lo = zeval->cam_se_fracs[seg];
    int32_t se_hi = zeval->cam_se_fracs[seg + 1U];
    int32_t cz_lo = c56e_knot_cz(servo->mech_hz, seg);
    int32_t cz_hi = c56e_knot_cz(servo->mech_hz, seg + 1U);
    int32_t d_se = se_hi - se_lo;
    if (d_se == 0) {
        return cz_lo;
    }
    return cz_lo + round_f32_to_s32(((float)(pos_se - se_lo) * (float)(cz_hi - cz_lo)) / (float)d_se);
}

static int32_t c56e_cz_to_se_unclamped(const sigma_150600_zoom_eval_t *zeval,
                                       const af_servo_state_t *servo, int32_t clamped_cz) {
    int32_t se0 = zeval->cam_se_fracs[0];
    if (clamped_cz < 0) {
        int32_t past_span = se0 - zeval->e_far_mech;
        if (past_span <= 0) {
            past_span = C56E_PAST_DEF;
        }
        int32_t lz_mag = (servo->mech_lz < 0) ? -servo->mech_lz : C56E_LZ_DEF_MAG;
        return se0 + round_f32_to_s32(((float)clamped_cz * (float)past_span) / (float)lz_mag);
    }
    size_t seg = C56E_LAST_SEG;
    for (size_t idx = 0U; idx < C56E_LAST_SEG; ++idx) {
        if (clamped_cz <= c56e_knot_cz(servo->mech_hz, idx + 1U)) {
            seg = idx;
            break;
        }
    }
    int32_t cz_lo = c56e_knot_cz(servo->mech_hz, seg);
    int32_t cz_hi = c56e_knot_cz(servo->mech_hz, seg + 1U);
    int32_t se_lo = zeval->cam_se_fracs[seg];
    int32_t se_hi = zeval->cam_se_fracs[seg + 1U];
    int32_t d_cz = cz_hi - cz_lo;
    if (d_cz <= 0) {
        return se_lo;
    }
    return se_lo + round_f32_to_s32(((float)(clamped_cz - cz_lo) * (float)(se_hi - se_lo)) / (float)d_cz);
}

static void sync_observed_endpoints(adapter_ctx_t *ctx, bool fixed_catalog_cam) {
    af_servo_state_t *servo = &ctx->af;
    if (ctx->id06.lim_near_se == ctx->id06.lim_far_se) {
        return;
    }
    if (fixed_catalog_cam) {
        int32_t cur_s = servo->e_near_true - servo->e_inf_true;
        int32_t abs_cur = (cur_s >= 0) ? cur_s : -cur_s;
        int32_t live_s = ctx->id06.lim_near_se - ctx->id06.lim_far_se;
        int32_t abs_live = (live_s >= 0) ? live_s : -live_s;
        if (abs_live > abs_cur) {
            servo->e_inf_true = ctx->id06.lim_far_se;
            servo->e_near_true = ctx->id06.lim_near_se;
        }
        return;
    }
    int32_t dfar = ctx->id06.lim_far_se - servo->e_inf_true;
    int32_t dnear = ctx->id06.lim_near_se - servo->e_near_true;
    if ((dfar < -ENDPOINT_HYST_SE) || (dfar > ENDPOINT_HYST_SE) ||
        (dnear < -ENDPOINT_HYST_SE) || (dnear > ENDPOINT_HYST_SE)) {
        servo->e_inf_true = ctx->id06.lim_far_se;
        servo->e_near_true = ctx->id06.lim_near_se;
    }
}

static void update_barrel_limiter_flag(adapter_ctx_t *ctx, const sigma_150600_zoom_eval_t *zeval,
                                       bool fixed_catalog_cam, bool is_c56e, int32_t abs_span) {
    af_servo_state_t *servo = &ctx->af;
    servo->barrel_limiter_on = 0U;
    if ((ctx->id06.valid != 0U) && (fixed_catalog_cam || is_c56e)) {
        int32_t ref_span = is_c56e ? (zeval->e_near_true - zeval->e_far_mech) : abs_span;
        int32_t id06_span = ctx->id06.lim_near_se - ctx->id06.lim_far_se;
        int32_t abs_id06_span = (id06_span >= 0) ? id06_span : -id06_span;
        if ((abs_id06_span > 0) && ((ref_span - abs_id06_span) > LIMITER_SHRINK_THRESH)) {
            servo->barrel_limiter_on = 1U;
        }
    }
}

static bool lens_has_past_infinity(const adapter_ctx_t *ctx, bool fixed_catalog_cam,
                                   bool is_c56e) {
    if (is_c56e || ((ctx->id06.valid == 0U) && fixed_catalog_cam)) {
        return true;
    }
    if (ctx->id06.valid == 0U) {
        return false;
    }
    return (bool)((ctx->af.dir_sign * (ctx->id06.lim_far_se - ctx->af.e_inf_true)) < -ENDPOINT_HYST_SE);
}

static float compute_current_jacobian(const adapter_ctx_t *ctx,
                                      const sigma_150600_zoom_eval_t *zeval,
                                      const af_servo_state_t *servo,
                                      bool is_c56e,
                                      int32_t abs_span) {
    float jacobian = (float)servo->mech_hz / (float)abs_span;
    if (is_c56e) {
        int32_t cur_se = (ctx->id06.valid != 0U) ? ctx->id06.focus_pos_se : zeval->e_inf_true;
        int32_t se_lo = ((cur_se - C56E_JAC_STEP) > zeval->e_inf_true)
                            ? (cur_se - C56E_JAC_STEP)
                            : zeval->e_inf_true;
        int32_t se_hi = ((cur_se + C56E_JAC_STEP) > (se_lo + C56E_JAC_STEP))
                            ? (cur_se + C56E_JAC_STEP)
                            : (se_lo + C56E_JAC_STEP);
        int32_t cz_lo = c56e_se_to_cz_unclamped(zeval, servo, se_lo);
        int32_t cz_hi = c56e_se_to_cz_unclamped(zeval, servo, se_hi);
        jacobian = (float)(cz_hi - cz_lo) / (float)(se_hi - se_lo);
    }
    return (jacobian < MIN_JACOBIAN_GUARD) ? MIN_JACOBIAN_GUARD : jacobian;
}

static void refresh_cam_endpoints_and_jacobian(adapter_ctx_t *ctx) {
    af_servo_state_t *servo = &ctx->af;
    const lens_db_entry_t *entry = lens_db_lookup_cached(ctx);
    bool is_zoom_lens = (bool)((ctx->f_max_mm > ctx->f_min_mm) ||
                               ((entry != NULL) && (entry->f_max_mm > entry->f_min_mm)));
    bool fixed_catalog_cam = (bool)(!is_zoom_lens);
    bool is_c56e = (bool)((ctx->e_lens_id == SIGMA_150600_REV2) ||
                          (ctx->e_lens_id == SIGMA_150600_REV1));
    sigma_150600_zoom_eval_t zeval;

    if (is_c56e) {
        uint16_t znorm = (ctx->id06.zoom_norm != 0U) ? ctx->id06.zoom_norm : DEFAULT_ZOOM_WIDE;
        sigma_150600_eval_zoom(znorm, &zeval);
        servo->e_inf_true = zeval.e_inf_true;
        servo->e_near_true = zeval.e_near_true;
        servo->mfd_meters = zeval.mfd_meters;
        if (ctx->id05.f_nom_0p1mm == 0U) {
            ctx->f_cur_mm = (uint16_t)round_f32_to_s32(zeval.focal_mm);
        }
    } else if ((ctx->id06.valid != 0U) && !apply_bound_optical_endpoints(ctx, false)) {
        sync_observed_endpoints(ctx, fixed_catalog_cam);
    }

    int32_t raw_span = servo->e_near_true - servo->e_inf_true;
    servo->dir_sign = (raw_span >= 0) ? 1 : -1;
    int32_t abs_span = (raw_span >= 0) ? raw_span : -raw_span;
    if (abs_span < MIN_SPAN_TICKS) {
        abs_span = DEFAULT_SPAN_TICKS;
    }

    if (servo->mfd_meters < MIN_MFD_FLOOR_M) {
        servo->mfd_meters = DEFAULT_MFD_METERS;
    }
    servo->mech_lz = lens_has_past_infinity(ctx, fixed_catalog_cam, is_c56e) ? MECH_LZ_DEFAULT : 0;
    servo->mech_hz = round_f32_to_s32(servo->scale_const_a / servo->mfd_meters);
    if (servo->mech_hz < MIN_HZ_FLOOR) {
        servo->mech_hz = MIN_HZ_FLOOR;
    }

    servo->cur_jacobian = compute_current_jacobian(ctx, &zeval, servo, is_c56e, abs_span);
    update_barrel_limiter_flag(ctx, &zeval, fixed_catalog_cam, is_c56e, abs_span);
}

static uint16_t derive_zoom_norm_f01(const adapter_ctx_t *ctx) {
    static const int32_t ROUND_HALF_DIV = 2;
    int32_t wide_z = (int32_t)PARFOCAL_ZOOM_WIDE_U16;
    int32_t tele_z = (int32_t)PARFOCAL_ZOOM_TELE_U16;
    int32_t znorm = clamp_s32((int32_t)ctx->id06.zoom_norm, wide_z, tele_z);
    int32_t span_z = tele_z - wide_z;
    int32_t fmin01 = (int32_t)ctx->f_min_mm * (int32_t)DECIMM_PER_MM;
    int32_t fmax01 = (int32_t)ctx->f_max_mm * (int32_t)DECIMM_PER_MM;
    int32_t numer = (znorm - wide_z) * (fmax01 - fmin01);
    return (uint16_t)(fmin01 + ((numer + (span_z / ROUND_HALF_DIV)) / span_z));
}

static uint16_t current_focal_0p1mm_u16(const adapter_ctx_t *ctx) {
    int32_t dzoom = (int32_t)ctx->id06.zoom_norm - (int32_t)ctx->af_ext.parfocal_last_zoom;
    if (dzoom < 0) {
        dzoom = -dzoom;
    }
    if ((ctx->id05_fresh_since_06 != (uint8_t)ID05_PAIR_EARLY_05) &&
        (ctx->f_max_mm > ctx->f_min_mm) &&
        (ctx->id06.zoom_norm >= PARFOCAL_ZOOM_WIDE_U16) &&
        (dzoom >= PARFOCAL_ZOOM_STEP_DIFF)) {
        uint16_t zf01 = derive_zoom_norm_f01(ctx);
        int32_t fdiff = (int32_t)zf01 - (int32_t)ctx->id05.f_nom_0p1mm;
        int32_t abs_fdiff = (fdiff >= 0) ? fdiff : -fdiff;
        if ((ctx->id05.f_nom_0p1mm < MIN_FOCAL_DECIMM) ||
            (abs_fdiff >= PARFOCAL_F01_DIFF_MIN)) {
            return zf01;
        }
    }
    if (ctx->id05.f_nom_0p1mm >= MIN_FOCAL_DECIMM) {
        return ctx->id05.f_nom_0p1mm;
    }
    return (uint16_t)((uint32_t)ctx->f_cur_mm * (uint32_t)DECIMM_PER_MM);
}

static float current_focal_mm_f32(const adapter_ctx_t *ctx) {
    uint16_t f01 = current_focal_0p1mm_u16(ctx);
    if (f01 >= MIN_FOCAL_DECIMM) {
        return (float)f01 * DECIMM_TO_MM_F;
    }
    return (float)ctx->f_cur_mm;
}

static int32_t snap_endstop_cz(const adapter_ctx_t *ctx, int32_t raw_cz);
static void arm_target_drive(adapter_ctx_t *ctx, int32_t req_cz, bool unlim, float vpps);

static void select_af_focal_tier(adapter_ctx_t *ctx) {
    af_servo_state_t *servo = &ctx->af;
    if (ctx->f_max_mm >= FOCAL_TIER_TELE_MM) {
        servo->scale_exp_n = SCALE_EXP_TELE;
        servo->scale_const_a = SCALE_A_TELE;
    } else if (ctx->f_max_mm >= FOCAL_TIER_MID_MM) {
        servo->scale_exp_n = SCALE_EXP_MID;
        servo->scale_const_a = SCALE_A_MID;
    } else {
        servo->scale_exp_n = SCALE_EXP_SHORT;
        servo->scale_const_a = SCALE_A_SHORT;
    }
}

static void init_configured_endpoints(adapter_ctx_t *ctx) {
    af_servo_state_t *servo = &ctx->af;
    const lens_db_entry_t *entry = lens_db_lookup_cached(ctx);
    servo->mfd_meters = (entry != NULL) ? ((float)entry->mfd_mm * MM_TO_METERS) : DEFAULT_MFD_METERS;
    if (apply_bound_optical_endpoints(ctx, true)) {
        ctx->af_limits_seen = 1U;
    } else if ((ctx->id06.valid != 0U) && (ctx->id06.lim_near_se != ctx->id06.lim_far_se)) {
        servo->e_inf_true = ctx->id06.lim_far_se;
        servo->e_near_true = ctx->id06.lim_near_se;
        ctx->af_limits_seen = 1U;
    } else if ((entry != NULL) && (entry->cam_e_near != entry->cam_e_inf)) {
        servo->e_inf_true = entry->cam_e_inf;
        servo->e_near_true = entry->cam_e_near;
    } else {
        servo->e_inf_true = 0;
        servo->e_near_true = DEFAULT_SPAN_TICKS;
    }
    refresh_cam_endpoints_and_jacobian(ctx);
    if (ctx->id06.valid != 0U) {
        servo->cur_cz = snap_endstop_cz(ctx, af_servo_se_to_cz(ctx, ctx->id06.focus_pos_se));
        servo->cur_dist_m = ((servo->cur_cz > 0) && (servo->mech_hz > 0))
                                ? ((af_servo_effective_mfd_meters(ctx) * (float)servo->mech_hz) /
                                   (float)servo->cur_cz)
                                : FAR_INF_DIST_M;
    } else {
        servo->cur_cz = 0;
        servo->cur_dist_m = FAR_INF_DIST_M;
    }
}

void af_servo_configure_for_lens(adapter_ctx_t *ctx) {
    af_servo_state_t *servo = &ctx->af;
    uint8_t saved_linear_mf = servo->linear_mf_code;
    bool same_session_lens = (bool)((ctx->af_configured_lid == ctx->e_lens_id) &&
                                    (ctx->af_configured_epoch == ctx->session_epoch) &&
                                    (ctx->e_lens_id != 0U));
    bool in_hs_af_seen = (bool)((ctx->session_state == E_SESSION_HANDSHAKE) &&
                                (ctx->af_ext.last_z_af_cmd != 0U));
    bool preserve_af = (bool)(same_session_lens || in_hs_af_seen);
    uint8_t saved_last_cmd = preserve_af ? ctx->af_ext.last_z_af_cmd : 0U;
    uint8_t saved_cookie   = preserve_af ? servo->latest_cookie : 0U;
    uint8_t saved_drive    = preserve_af ? servo->af_drive_active : 0U;
    uint8_t saved_cont     = preserve_af ? servo->is_continuous_track : 0U;
    uint8_t saved_unlim    = preserve_af ? servo->unlimited_speed : 0U;
    int8_t  saved_dir      = preserve_af ? servo->motion_dir_cz : 0;
    int32_t saved_tgt_cz   = preserve_af ? servo->target_cz : 0;
    float   saved_vpps     = preserve_af ? servo->cmd_vel_cz_pps : 0.0F;

    memset(servo, 0, sizeof(*servo));
    memset(&ctx->af_ext, 0, sizeof(ctx->af_ext));
    if ((!same_session_lens) || (ctx->id06.valid == 0U)) {
        ctx->parfocal_dist_m = 0.0F;
        ctx->parfocal_last_cmd_se = 0;
        ctx->parfocal_last_f01 = 0U;
        ctx->parfocal_hold_vsyncs = 0U;
        ctx->parfocal_cmd_valid = 0U;
        ctx->parfocal_cl_trim_cz = 0;
        ctx->parfocal_anchor_dist_x = 0U;
        ctx->parfocal_cl_cooldown = 0U;
        ctx->parfocal_cl_steps = 0U;
        ctx->parfocal_anchor_f01 = 0U;
        ctx->parfocal_trim_f01 = 0U;
        ctx->parfocal_min_rem_se = (uint16_t)MAX_U16_CLAMP;
        ctx->parfocal_anchor_refresh = 0U;
        ctx->parfocal_e0_coast_rem = 0U;
    }
    servo->linear_mf_code = saved_linear_mf;
    ctx->af_ext.kz_c0_b1 = 1U;
    servo->arrival_confirmed = 1U;
    ctx->af_drive_start_se = 0;
    ctx->af_drive_moved = 0U;
    ctx->af_limits_seen = 0U;

    select_af_focal_tier(ctx);
    init_configured_endpoints(ctx);

    uint16_t kz_init = af_servo_encode_kz_pair(
        servo->cur_jacobian * calibrated_emount_k(ctx, 0U));
    servo->kz_b0 = (uint8_t)((uint32_t)kz_init & BYTE_MASK);
    servo->kz_b1 = (uint8_t)((uint32_t)kz_init >> SHIFT_8);

    if (ctx->e_lens_id != 0U) {
        ctx->af_configured_lid = ctx->e_lens_id;
        ctx->af_configured_epoch = ctx->session_epoch;
    }
    if (preserve_af) {
        ctx->af_ext.last_z_af_cmd = saved_last_cmd;
        servo->latest_cookie = saved_cookie;
        if (saved_drive != 0U) {
            servo->is_continuous_track = saved_cont;
            arm_target_drive(ctx, saved_tgt_cz, (bool)(saved_unlim != 0U), saved_vpps);
            if ((servo->af_drive_active != 0U) && (saved_dir != 0)) {
                servo->motion_dir_cz = saved_dir;
            }
        }
    }
}

float af_servo_effective_mfd_meters(const adapter_ctx_t *ctx) {
    if ((ctx->e_lens_id == SIGMA_150600_REV2) || (ctx->e_lens_id == SIGMA_150600_REV1) ||
        (ctx->f_max_mm <= ctx->f_min_mm)) {
        return ctx->af.mfd_meters;
    }
    const lens_db_entry_t *entry = lens_db_lookup(ctx->e_lens_id);
    if (entry == NULL) {
        return ctx->af.mfd_meters;
    }
    uint16_t mfd_w = entry->mfd_mm;
    uint16_t mfd_t = (entry->mfd_tele_mm != 0U) ? entry->mfd_tele_mm : mfd_w;
    if ((mfd_w == 0U) || (mfd_w == mfd_t)) {
        return ctx->af.mfd_meters;
    }
    float f_cur = current_focal_mm_f32(ctx);
    float f_min = (float)ctx->f_min_mm;
    float f_max = (float)ctx->f_max_mm;
    float zoom_t = (f_cur - f_min) / (f_max - f_min);
    if (zoom_t < 0.0F) {
        zoom_t = 0.0F;
    } else if (zoom_t > 1.0F) {
        zoom_t = 1.0F;
    }
    float eff_m = ((float)mfd_w + (zoom_t * (float)((int32_t)mfd_t - (int32_t)mfd_w))) * MM_TO_METERS;
    return (eff_m >= MIN_MFD_FLOOR_M) ? eff_m : ctx->af.mfd_meters;
}

int32_t af_servo_se_to_cz(const adapter_ctx_t *ctx, int32_t pos_se) {
    const af_servo_state_t *servo = &ctx->af;
    if ((ctx->e_lens_id == SIGMA_150600_REV2) || (ctx->e_lens_id == SIGMA_150600_REV1)) {
        uint16_t znorm = (ctx->id06.zoom_norm != 0U) ? ctx->id06.zoom_norm : DEFAULT_ZOOM_WIDE;
        sigma_150600_zoom_eval_t zeval;
        sigma_150600_eval_zoom(znorm, &zeval);
        int32_t cz_nl = c56e_se_to_cz_unclamped(&zeval, servo, pos_se);
        return clamp_s32(cz_nl, servo->mech_lz, servo->mech_hz);
    }
    int32_t net_delta = servo->dir_sign * (pos_se - servo->e_inf_true);
    int32_t cz_val = round_f32_to_s32((float)net_delta * servo->cur_jacobian);
    return clamp_s32(cz_val, servo->mech_lz, servo->mech_hz);
}

int32_t af_servo_cz_to_se(const adapter_ctx_t *ctx, int32_t target_cz) {
    const af_servo_state_t *servo = &ctx->af;
    int32_t clamped_cz = clamp_s32(target_cz, servo->mech_lz, servo->mech_hz);
    int32_t raw_se = 0;
    if ((ctx->e_lens_id == SIGMA_150600_REV2) || (ctx->e_lens_id == SIGMA_150600_REV1)) {
        uint16_t znorm = (ctx->id06.zoom_norm != 0U) ? ctx->id06.zoom_norm : DEFAULT_ZOOM_WIDE;
        sigma_150600_zoom_eval_t zeval;
        sigma_150600_eval_zoom(znorm, &zeval);
        raw_se = c56e_cz_to_se_unclamped(&zeval, servo, clamped_cz);
    } else {
        int32_t delta_se = round_f32_to_s32((float)clamped_cz / servo->cur_jacobian);
        raw_se = servo->e_inf_true + (servo->dir_sign * delta_se);
    }

    if ((ctx->id06.valid != 0U) && (ctx->id06.lim_near_se != ctx->id06.lim_far_se)) {
        int32_t lo_se = (ctx->id06.lim_far_se < ctx->id06.lim_near_se)
                            ? ctx->id06.lim_far_se
                            : ctx->id06.lim_near_se;
        int32_t hi_se = (ctx->id06.lim_far_se < ctx->id06.lim_near_se)
                            ? ctx->id06.lim_near_se
                            : ctx->id06.lim_far_se;
        raw_se = clamp_s32(raw_se, lo_se, hi_se);
    }
    return raw_se;
}

static uint8_t active_stall_guard_vsyncs(const adapter_ctx_t *ctx) {
    return (ctx->active_vsync_hz >= FAST_VSYNC_THRESH_HZ)
               ? STALL_GUARD_VSYNCS_120HZ
               : STALL_GUARD_VSYNCS_60HZ;
}

static uint8_t active_s1_hold_vsyncs(const adapter_ctx_t *ctx) {
    return (ctx->active_vsync_hz >= FAST_VSYNC_THRESH_HZ)
               ? S1_HOLD_VSYNCS_120HZ
               : S1_HOLD_VSYNCS_60HZ;
}

static bool is_body_in_manual_focus(const adapter_ctx_t *ctx) {
    if ((((uint32_t)ctx->z_mosi20_retained[2] & MOSI20_MF_FLAGS_MASK) != 0UL) ||
        (ctx->z_mosi20_retained[MOSI20_RING1_MODE_OFF] == MOSI20_FOCUS_MODE_MF) ||
        (ctx->z_mosi20_retained[MOSI20_RING2_MODE_OFF] == MOSI20_FOCUS_MODE_MF)) {
        return true;
    }
    return ctx->id05.is_mf_switch != 0U;
}

typedef struct {
    int32_t span_div;
    int32_t min_tol;
    int32_t max_tol;
} span_tol_cfg_t;

__attribute__((noinline)) static int32_t span_tolerance_se(const adapter_ctx_t *ctx,
                                                           const span_tol_cfg_t *cfg) {
    int32_t raw_span = ctx->af.e_near_true - ctx->af.e_inf_true;
    int32_t abs_span = (raw_span >= 0) ? raw_span : -raw_span;
    if (abs_span < MIN_SPAN_TICKS) {
        return STD_CAPTURE_TOL;
    }
    return clamp_s32(abs_span / cfg->span_div, cfg->min_tol, cfg->max_tol);
}

static int32_t capture_tolerance_se(const adapter_ctx_t *ctx) {
    static const span_tol_cfg_t CAP_CFG = {3072, 2, 6};
    return span_tolerance_se(ctx, &CAP_CFG);
}

static int32_t deadband_tolerance_se(const adapter_ctx_t *ctx) {
    static const span_tol_cfg_t DB_CFG = {1024, 4, 14};
    return span_tolerance_se(ctx, &DB_CFG);
}

static bool can_lens_use_3c_scan(const adapter_ctx_t *ctx) {
    if ((ctx->has_ext_5a != 0U) || (ctx->lens_family != 3U)) {
        return false;
    }
    return lens_session_cap_has(ctx, E_CAP_CMD_3C_SCAN);
}

static void sync_parfocal_anchor(adapter_ctx_t *ctx, int32_t anchor_cz);

static int32_t parfocal_cmd_settle_tol_se(const adapter_ctx_t *ctx) {
    int32_t tol = deadband_tolerance_se(ctx);
    return (tol > PARFOCAL_CMD_SETTLE_MIN) ? tol : PARFOCAL_CMD_SETTLE_MIN;
}

static bool is_parfocal_motor_tracking_settled(const adapter_ctx_t *ctx) {
    if (ctx->id06.hw_moving == 0U) {
        return true;
    }
    if (ctx->parfocal_cmd_valid == 0U) {
        return false;
    }
    int32_t err_se = ctx->parfocal_last_cmd_se - ctx->id06.focus_pos_se;
    int32_t abs_err = (err_se >= 0) ? err_se : -err_se;
    return (bool)(abs_err <= parfocal_cmd_settle_tol_se(ctx));
}

static bool is_parfocal_own_motion(const adapter_ctx_t *ctx, bool zoom_changed) {
    if ((ctx->parfocal_hold_vsyncs > 0U) &&
        (zoom_changed || (ctx->af_ext.parfocal_trim_pending != 0U))) {
        return true;
    }
    if (ctx->parfocal_cmd_valid == 0U) {
        return false;
    }
    int32_t rem_cur = ctx->parfocal_last_cmd_se - ctx->id06.focus_pos_se;
    int32_t abs_cur = (rem_cur >= 0) ? rem_cur : -rem_cur;
    int32_t rem_prev = ctx->parfocal_last_cmd_se - ctx->id06.focus_prev_se;
    int32_t abs_prev = (rem_prev >= 0) ? rem_prev : -rem_prev;
    if (ctx->id06.hw_moving != 0U) {
        return (bool)(abs_cur <= abs_prev);
    }
    int32_t tol_se = parfocal_cmd_settle_tol_se(ctx);
    int32_t d_se = ctx->id06.focus_pos_se - ctx->id06.focus_prev_se;
    bool moved_toward_cmd = (bool)(((d_se > 0) && (rem_prev > 0)) ||
                                   ((d_se < 0) && (rem_prev < 0)));
    if (moved_toward_cmd && (abs_cur <= (2 * tol_se))) {
        return true;
    }
    return (bool)((abs_cur < abs_prev) && (abs_cur <= tol_se));
}

static int32_t detect_manual_focus_ring_dir(const adapter_ctx_t *ctx, bool zoom_changed) {
    static const size_t ID06_AF_FLAGS_OFFSET = 6U;
    static const uint32_t ID06_AF_ACTIVE_MASK = 0x10UL;
    static const int32_t MF_RING_CLAMP_MAX = 127;
    const af_servo_state_t *servo = &ctx->af;
    if (ctx->id05.mf_ring_dir != 0) {
        int32_t step = (ctx->id05.mf_ring_step > 2U) ? (int32_t)ctx->id05.mf_ring_step : 2;
        return clamp_s32((ctx->id05.mf_ring_dir > 0) ? step : -step,
                         -MF_RING_CLAMP_MAX, MF_RING_CLAMP_MAX);
    }
    if ((((uint32_t)ctx->id06.raw_bytes[ID06_AF_FLAGS_OFFSET] & ID06_AF_ACTIVE_MASK) != 0UL) ||
        (servo->af_drive_active != 0U) || (ctx->init_sync_rem != 0U) ||
        (ctx->parfocal_e0_coast_rem != 0U)) {
        return 0;
    }
    if ((!is_body_in_manual_focus(ctx)) &&
        ((servo->s1_hold_vsyncs != 0U) || (servo->settle_frames_rem != 0U))) {
        return 0;
    }
    if ((servo->settle_frames_rem != 0U) && (ctx->id06.mf_ring_byte == 0U)) {
        return 0;
    }
    if (is_parfocal_own_motion(ctx, zoom_changed)) {
        return 0;
    }
    if (ctx->id06.mf_ring_byte == 2U) {
        return 2;
    }
    if (ctx->id06.mf_ring_byte == 1U) {
        return -2;
    }
    if (ctx->id06.t_prev_ticks == 0ULL) {
        return 0;
    }
    int32_t d_se = ctx->id06.focus_pos_se - ctx->id06.focus_prev_se;
    int32_t abs_d = (d_se >= 0) ? d_se : -d_se;
    if (abs_d < MF_ENCODER_DELTA_MIN) {
        return 0;
    }
    int32_t d_cz = servo->dir_sign * d_se;
    return (d_cz >= 0) ? 2 : -2;
}

static void update_manual_focus_ring_state(adapter_ctx_t *ctx, bool zoom_changed) {
    af_servo_state_t *servo = &ctx->af;
    int32_t mf_dir = detect_manual_focus_ring_dir(ctx, zoom_changed);
    if (mf_dir != 0) {
        servo->s1_hold_vsyncs = 0U;
        servo->stalled_abort = 0U;
        servo->mf_ring_delta_bf = (int8_t)mf_dir;
        servo->mf_hold_frames_rem = MF_RING_HOLD_FRAMES;
        ctx->parfocal_hold_vsyncs = 0U;
        ctx->parfocal_cmd_valid = 0U;
        ctx->af_ext.parfocal_trim_pending = 0U;
        ctx->z_ring_heartbeat = (uint8_t)(ctx->z_ring_heartbeat + 1U);
    } else if (servo->mf_hold_frames_rem > 0U) {
        servo->mf_hold_frames_rem = (uint8_t)(servo->mf_hold_frames_rem - 1U);
        if (servo->mf_hold_frames_rem == 0U) {
            servo->mf_ring_delta_bf = 0;
            sync_parfocal_anchor(ctx, servo->cur_cz);
        }
    } else {
        servo->mf_ring_delta_bf = 0;
    }
}

static int32_t endstop_cushion_band_se(const adapter_ctx_t *ctx) {
    static const int32_t MIN_CUSHION_BAND_SE = 24;
    static const int32_t CUSHION_SPAN_DIV    = 48;
    int32_t span = ctx->id06.lim_near_se - ctx->id06.lim_far_se;
    int32_t abs_span = (span >= 0) ? span : -span;
    int32_t scaled = abs_span / CUSHION_SPAN_DIV;
    return (scaled > MIN_CUSHION_BAND_SE) ? scaled : MIN_CUSHION_BAND_SE;
}

static void clear_expired_cushion_latches(adapter_ctx_t *ctx) {
    if (ctx->id06.valid == 0U) {
        return;
    }
    int32_t band = endstop_cushion_band_se(ctx);
    int32_t d_near = ctx->id06.focus_pos_se - ctx->id06.lim_near_se;
    int32_t d_far = ctx->id06.focus_pos_se - ctx->id06.lim_far_se;
    if (((d_near >= 0) ? d_near : -d_near) > band) {
        ctx->af.cushioned_near = 0U;
    }
    if (((d_far >= 0) ? d_far : -d_far) > band) {
        ctx->af.cushioned_far = 0U;
    }
}

static bool check_endstop_cushion_hit(const adapter_ctx_t *ctx, bool near_side) {
    static const uint8_t CUSHION_SETTLE_VSYNCS    = 2U;
    static const uint8_t CUSHION_RECONFIRM_VSYNCS = 2U;
    const af_servo_state_t *servo = &ctx->af;
    if ((ctx->id06.valid == 0U) || (ctx->id06.lim_near_se == ctx->id06.lim_far_se) ||
        (ctx->id06.hw_moving != 0U)) {
        return false;
    }
    uint8_t cushion_flag = near_side ? servo->cushioned_near : servo->cushioned_far;
    bool already_cushioned = (bool)(cushion_flag != 0U);
    if (already_cushioned) {
        if (servo->stable_idle_count < CUSHION_RECONFIRM_VSYNCS) {
            return false;
        }
    } else if ((servo->stable_idle_count < CUSHION_SETTLE_VSYNCS) ||
               ((ctx->af_drive_moved == 0U) && (servo->drive_vsyncs < MIN_CUSHION_DRIVE_VSYNCS))) {
        return false;
    }
    if (near_side && (servo->target_cz < servo->mech_hz)) {
        return false;
    }
    if ((!near_side) && (servo->target_cz > servo->mech_lz)) {
        return false;
    }
    int32_t lim_se = near_side ? ctx->id06.lim_near_se : ctx->id06.lim_far_se;
    int32_t dist = ctx->id06.focus_pos_se - lim_se;
    int32_t abs_dist = (dist >= 0) ? dist : -dist;
    return (bool)(abs_dist <= endstop_cushion_band_se(ctx));
}

static void latch_endstop_on_settle(adapter_ctx_t *ctx) {
    af_servo_state_t *servo = &ctx->af;
    if ((ctx->id06.valid == 0U) || (ctx->id06.lim_near_se == ctx->id06.lim_far_se) ||
        (servo->barrel_limiter_on != 0U)) {
        return;
    }
    int32_t band = endstop_cushion_band_se(ctx);
    if (servo->target_cz >= servo->mech_hz) {
        int32_t d_near = ctx->id06.focus_pos_se - ctx->id06.lim_near_se;
        if (((d_near >= 0) ? d_near : -d_near) <= band) {
            ctx->id06.at_near_end = 1U;
            servo->cushioned_near = 1U;
        }
    }
    if (servo->target_cz <= servo->mech_lz) {
        int32_t d_far = ctx->id06.focus_pos_se - ctx->id06.lim_far_se;
        if (((d_far >= 0) ? d_far : -d_far) <= band) {
            ctx->id06.at_far_end = 1U;
            servo->cushioned_far = 1U;
        }
    }
}

static int32_t closed_loop_moved_tol_se(const adapter_ctx_t *ctx) {
    static const span_tol_cfg_t MOVED_CFG = {192, 24, 64};
    return span_tolerance_se(ctx, &MOVED_CFG);
}

static int32_t closed_loop_micro_tol_se(const adapter_ctx_t *ctx) {
    static const span_tol_cfg_t MICRO_CFG = {256, 16, 48};
    return span_tolerance_se(ctx, &MICRO_CFG);
}

static bool evaluate_step_settle_ready(const adapter_ctx_t *ctx, int32_t abs_rem,
                                       bool hit_near_limit, bool hit_far_limit) {
    const af_servo_state_t *servo = &ctx->af;
    if (ctx->id06.hw_moving != 0U) {
        return false;
    }
    if ((servo->stable_idle_count >= 1U) && (abs_rem <= capture_tolerance_se(ctx))) {
        return true;
    }
    if (hit_near_limit || hit_far_limit) {
        return true;
    }
    if (servo->af_cmd_pending != 0U) {
        return false;
    }
    if ((ctx->af_drive_moved != 0U) && (servo->stable_idle_count >= 2U)) {
        return (bool)((abs_rem <= deadband_tolerance_se(ctx)) ||
                      (abs_rem <= closed_loop_moved_tol_se(ctx)));
    }
    if ((ctx->af_drive_moved == 0U) && (servo->stable_idle_count >= 3U)) {
        return (bool)((abs_rem <= deadband_tolerance_se(ctx)) ||
                      (abs_rem <= closed_loop_micro_tol_se(ctx)));
    }
    return false;
}

static void step_active_af_telemetry(adapter_ctx_t *ctx, uint64_t now_ticks) {
    static const uint8_t DRIVE_VSYNCS_MAX = 255U;
    af_servo_state_t *servo = &ctx->af;
    servo->mf_hold_frames_rem = 0U;
    servo->mf_ring_delta_bf = 0;
    if (servo->drive_vsyncs < DRIVE_VSYNCS_MAX) {
        servo->drive_vsyncs = (uint8_t)(servo->drive_vsyncs + 1U);
    }
    if (ctx->id06.focus_pos_se != ctx->id06.focus_prev_se) {
        servo->stall_vsyncs = 0U;
        servo->s1_hold_vsyncs = active_s1_hold_vsyncs(ctx);
        if (now_ticks != 0ULL) {
            servo->af_cmd_ticks = now_ticks;
        }
        servo->stable_idle_count = 0U;
    } else {
        servo->stable_idle_count = (uint8_t)(servo->stable_idle_count + 1U);
    }
    if (ctx->id06.focus_pos_se != ctx->af_drive_start_se) {
        ctx->af_drive_moved = 1U;
    }
    int32_t rem_err = servo->target_se - ctx->id06.focus_pos_se;
    int32_t abs_rem = (rem_err >= 0) ? rem_err : -rem_err;
    int32_t dir_rem_cz = servo->dir_sign * rem_err;
    bool hit_near_limit = (bool)((dir_rem_cz > 0) &&
                                 ((ctx->id06.at_near_end != 0U) ||
                                  check_endstop_cushion_hit(ctx, true)));
    bool hit_far_limit = (bool)((dir_rem_cz < 0) &&
                                ((ctx->id06.at_far_end != 0U) ||
                                 check_endstop_cushion_hit(ctx, false)));
    if (hit_near_limit) {
        ctx->id06.at_near_end = 1U;
        if (servo->barrel_limiter_on == 0U) {
            servo->cushioned_near = 1U;
        }
    }
    if (hit_far_limit) {
        ctx->id06.at_far_end = 1U;
        if (servo->barrel_limiter_on == 0U) {
            servo->cushioned_far = 1U;
        }
    }
    if (evaluate_step_settle_ready(ctx, abs_rem, hit_near_limit, hit_far_limit)) {
        latch_endstop_on_settle(ctx);
        servo->arrival_confirmed = 1U;
        servo->stalled_abort = 0U;
        servo->af_drive_active = 0U;
        servo->af_cmd_pending = 0U;
        servo->af_submode = 0U;
        servo->prev_drive_dir_se = 0;
        servo->step_accum_se = 0;
        servo->track_primed = 0U;
        servo->e0_brake_rem = 0U;
        servo->settle_frames_rem = SETTLE_HOLD_FRAMES;
        sync_parfocal_anchor(ctx, servo->cur_cz);
    }
}

static int32_t snap_endstop_cz(const adapter_ctx_t *ctx, int32_t raw_cz) {
    if ((ctx->id06.valid != 0U) && (ctx->id06.hw_moving == 0U) &&
        (ctx->af.barrel_limiter_on == 0U)) {
        if (ctx->af.cushioned_near != 0U) {
            return ctx->af.mech_hz;
        }
        if (ctx->af.cushioned_far != 0U) {
            return ctx->af.mech_lz;
        }
    }
    return raw_cz;
}

static float abs_f32_diff(float val_a, float val_b) {
    float diff = val_a - val_b;
    return (diff >= 0.0F) ? diff : -diff;
}

static void observe_k_se_inf_ref(adapter_ctx_t *ctx, float k_se_raw) {
    static const float BETA_INF_THRESH   = 0.03F;
    static const float LEVER_SQ_MIN      = 1.25F;
    static const float KE_MATCH_LO       = 0.90F;
    static const float KE_MATCH_HI       = 1.10F;
    static const float INF_REFRESH_FLOOR = 0.85F;

    af_v218_ext_t *ext = &ctx->af_ext;
    float beta = (float)ctx->ap.z_mag_a3_q14 / Q14_SCALE_F;
    float one_plus_beta = 1.0F + beta;
    float k_se_scaled_inf = k_se_raw * one_plus_beta * one_plus_beta;

    if (beta < BETA_INF_THRESH) {
        bool mag_group_missing = (bool)((ctx->id05.active_mask & (1UL << 4U)) == 0UL);
        bool mag_group_ready = (bool)(mag_group_missing || (ctx->id05.grp_present[4] != 0U));
        if (!mag_group_ready) {
            return;
        }
        if (ext->ke_inf_valid == 0U) {
            ext->ke_inf_ref = k_se_scaled_inf;
            ext->ke_inf_valid = 1U;
        } else if ((k_se_scaled_inf > ext->ke_inf_ref) ||
                   ((ctx->id06.hw_moving == 0U) &&
                    (k_se_scaled_inf >= (ext->ke_inf_ref * INF_REFRESH_FLOOR)))) {
            ext->ke_inf_ref = k_se_scaled_inf;
        }
        return;
    }
    if (ext->ke_inf_valid != 0U) {
        return;
    }
    if (ext->ke_first_valid == 0U) {
        ext->ke_first_obs = k_se_raw;
        ext->beta_first_obs = beta;
        ext->ke_first_valid = 1U;
        return;
    }
    float base_one = 1.0F + ext->beta_first_obs;
    float base_sq = base_one * base_one;
    float cur_sq = one_plus_beta * one_plus_beta;
    float lev_sq = (cur_sq >= base_sq) ? (cur_sq / base_sq) : (base_sq / cur_sq);
    if ((lev_sq >= LEVER_SQ_MIN) && (ext->ke_first_obs > 1.0F)) {
        float ref_first = ext->ke_first_obs * base_sq;
        float ratio_scaled = k_se_scaled_inf / ref_first;
        float ratio_raw = k_se_raw / ext->ke_first_obs;
        float err_scaled = abs_f32_diff(ratio_scaled, 1.0F);
        float err_raw = abs_f32_diff(ratio_raw, 1.0F);
        if ((ratio_scaled >= KE_MATCH_LO) && (ratio_scaled <= KE_MATCH_HI) &&
            (err_scaled < (err_raw * HALF_SCALE_F))) {
            ext->ke_inf_ref = k_se_scaled_inf;
            ext->ke_inf_valid = 1U;
        }
    }
}

static float compute_single_beta_scale(adapter_ctx_t *ctx) {
    af_v218_ext_t *ext = &ctx->af_ext;
    if (ext->prev_f_cur_mm != ctx->f_cur_mm) {
        ext->prev_f_cur_mm = ctx->f_cur_mm;
        ext->ke_inf_valid = 0U;
        ext->ke_first_valid = 0U;
    }
    float beta = (float)ctx->ap.z_mag_a3_q14 / Q14_SCALE_F;
    float one_plus_beta = 1.0F + beta;
    float beta_sq = one_plus_beta * one_plus_beta;
    if (ctx->id06.defocus_k_se == 0U) {
        return 1.0F / beta_sq;
    }
    float k_se_raw = (float)ctx->id06.defocus_k_se;
    observe_k_se_inf_ref(ctx, k_se_raw);
    if (ext->ke_inf_valid != 0U) {
        float k_cap = ext->ke_inf_ref / beta_sq;
        return (k_se_raw > k_cap) ? (k_cap / k_se_raw) : 1.0F;
    }
    return 1.0F / beta_sq;
}

static float aperture_indep_f2_base_k(const adapter_ctx_t *ctx, float k_inf_raw_units) {
    static const float F2_CANONICAL_SCALE = 9.0F / 35.0F;
    static const float K_ZERO_FALLBACK    = 5.015625F;
    if (k_inf_raw_units <= 0.0F) {
        return K_ZERO_FALLBACK;
    }
    float k_base = k_inf_raw_units * F2_CANONICAL_SCALE;
    if (ctx->e_lens_id == SIGMA_105_MACRO_ID) {
        k_base *= MACRO_105_SCALE_COMP;
    }
    return k_base;
}

static void update_pole_safe_c0(adapter_ctx_t *ctx, float ke_inf_f2_base) {
    static const uint16_t MIN_VALID_FOCAL_MM = 8U;
    static const int32_t  C0_MAG_MAX         = 5;
    static const float    C0_FOCAL_NUM       = 160.0F;
    static const float    C0_SAFE_NUM        = 150.0F;
    static const float    FALLBACK_FOCAL_MM  = 50.0F;
    static const float    INTEGRAL_MIN_MULT  = 1.20F;
    static const float    BETA_QUADRATIC_DIV = 3.0F;
    static const uint8_t C0_ENCODE_TBL[6][2] = {
        {0x00U, 0x01U},
        {0x80U, 0xF9U},
        {0x80U, 0xFAU},
        {0xC0U, 0xFAU},
        {0x80U, 0xFBU},
        {0xA0U, 0xFBU},
    };
    float beta = (float)ctx->ap.z_mag_a3_q14 / Q14_SCALE_F;
    int32_t raw_span = ctx->af.e_near_true - ctx->af.e_inf_true;
    int32_t abs_span = (raw_span >= 0) ? raw_span : -raw_span;
    if (abs_span < MIN_SPAN_TICKS) {
        abs_span = DEFAULT_SPAN_TICKS;
    }
    float integral_mult = 1.0F + beta + ((beta * beta) / BETA_QUADRATIC_DIV);
    if (integral_mult < INTEGRAL_MIN_MULT) {
        integral_mult = INTEGRAL_MIN_MULT;
    }
    float d_mm_max = (ke_inf_f2_base > 1.0F)
                         ? (((float)abs_span / ke_inf_f2_base) * integral_mult)
                         : FAR_INF_DIST_M;
    static const uint16_t Q4_FOCAL_DIV_U   = 16U;
    static const float    Q4_FOCAL_DIV_F   = 16.0F;
    float f_mm = FALLBACK_FOCAL_MM;
    if (ctx->ap.z_eff_focal_q4 >= (uint16_t)(MIN_VALID_FOCAL_MM * Q4_FOCAL_DIV_U)) {
        f_mm = (float)ctx->ap.z_eff_focal_q4 / Q4_FOCAL_DIV_F;
    } else if (ctx->f_cur_mm >= MIN_VALID_FOCAL_MM) {
        f_mm = (float)ctx->f_cur_mm;
    }
    int32_t c0_from_f = round_f32_to_s32(C0_FOCAL_NUM / (f_mm * (1.0F + beta)));
    int32_t c0_safe = (d_mm_max > 1.0F) ? (int32_t)(C0_SAFE_NUM / d_mm_max) : 0;
    int32_t c0_mag = clamp_s32((c0_from_f < c0_safe) ? c0_from_f : c0_safe, 0, C0_MAG_MAX);
    if ((ctx->dist_report_invalid != 0U) &&
        ((ctx->id06.valid == 0U) || (ctx->id06.lim_near_se == ctx->id06.lim_far_se))) {
        c0_mag = 0;
    }
    ctx->af_ext.kz_c0_b0 = C0_ENCODE_TBL[c0_mag][0];
    ctx->af_ext.kz_c0_b1 = C0_ENCODE_TBL[c0_mag][1];
}

void af_servo_refresh_kz(adapter_ctx_t *ctx) {
    af_servo_state_t *servo = &ctx->af;
    float ke_raw = calibrated_emount_k(ctx, ctx->id06.defocus_k_se);
    if (ctx->e_lens_id == SIGMA_105_MACRO_ID) {
        ke_raw *= MACRO_105_SCALE_COMP;
    }
    float beta = (float)ctx->ap.z_mag_a3_q14 / Q14_SCALE_F;
    float ke_damped = ke_raw * compute_single_beta_scale(ctx);
    float one_plus_beta = 1.0F + beta;
    float k_inf_raw_units = 0.0F;
    if ((ctx->af_ext.ke_inf_valid != 0U) && (ctx->id06.defocus_k_se > 0U)) {
        k_inf_raw_units = ctx->af_ext.ke_inf_ref;
    } else if (ctx->id06.defocus_k_se > 0U) {
        k_inf_raw_units = (float)ctx->id06.defocus_k_se * one_plus_beta * one_plus_beta;
    }
    update_pole_safe_c0(ctx, aperture_indep_f2_base_k(ctx, k_inf_raw_units));
    float kz_eff = servo->cur_jacobian * ke_damped;
    uint16_t kz_packed = af_servo_encode_kz_pair(kz_eff);
    servo->kz_b0 = (uint8_t)((uint32_t)kz_packed & BYTE_MASK);
    servo->kz_b1 = (uint8_t)((uint32_t)kz_packed >> SHIFT_8);
}

static bool is_parfocal_dist_code_valid(uint16_t dist_x) {
    return (bool)((dist_x > 0U) &&
                  ((dist_x < PARFOCAL_MAX_VALID_X) || (dist_x == (uint16_t)MAX_U16_CLAMP)));
}

static bool is_sigma_cam_zoom(const adapter_ctx_t *ctx) {
    return (bool)((ctx->e_lens_id == SIGMA_150600_REV2) ||
                  (ctx->e_lens_id == SIGMA_150600_REV1));
}

static void sync_live_id05_focal_and_dist_flag(adapter_ctx_t *ctx) {
    if (ctx->id05.grp_present[4] == 0U) {
        return;
    }
    if (ctx->id05.f_nom_0p1mm >= PARFOCAL_MIN_F01) {
        ctx->f_cur_mm = (uint16_t)((ctx->id05.f_nom_0p1mm + PARFOCAL_F01_HALF) /
                                   PARFOCAL_F01_DIV);
    }
    if (ctx->dist_report_invalid != 0U) {
        return;
    }
    uint16_t dist_x = ctx->id05.dist_code_x;
    if (is_parfocal_dist_code_valid(dist_x)) {
        if (ctx->dist_first_x == 0U) {
            ctx->dist_first_x = dist_x;
        } else if (dist_x != ctx->dist_first_x) {
            ctx->dist_verified_dynamic = 1U;
        }
    }
}

static float current_parfocal_net_mfd(const adapter_ctx_t *ctx, float *out_nodal_d0_m) {
    float focal_m = current_focal_mm_f32(ctx) * MM_TO_METERS;
    float nodal_d0 = PARFOCAL_NODAL_SCALE * focal_m;
    float d_mfd = af_servo_effective_mfd_meters(ctx);
    float net_mfd = d_mfd - nodal_d0;
    if (net_mfd < PARFOCAL_NET_MFD_MIN) {
        net_mfd = PARFOCAL_NET_MFD_MIN;
    }
    if (out_nodal_d0_m != NULL) {
        *out_nodal_d0_m = nodal_d0;
    }
    return net_mfd;
}

static void sync_parfocal_anchor(adapter_ctx_t *ctx, int32_t anchor_cz) {
    static const uint8_t PARFOCAL_REFR_120_MULT = 2U;
    af_servo_state_t *servo = &ctx->af;
    af_v218_ext_t *ext = &ctx->af_ext;
    ext->parfocal_cz_anchor = anchor_cz;
    if ((anchor_cz > 0) && (servo->mech_hz > 0)) {
        float stroke_r0 = (float)anchor_cz / (float)servo->mech_hz;
        if (stroke_r0 > 1.0F) {
            stroke_r0 = 1.0F;
        }
        float nodal_d0 = 0.0F;
        float net_mfd = current_parfocal_net_mfd(ctx, &nodal_d0);
        ctx->parfocal_dist_m = nodal_d0 + (net_mfd / stroke_r0);
    } else {
        ctx->parfocal_dist_m = FAR_INF_DIST_M;
    }
    uint16_t cur_f01 = current_focal_0p1mm_u16(ctx);
    ctx->parfocal_last_f01 = cur_f01;
    ctx->parfocal_anchor_f01 = cur_f01;
    ctx->parfocal_trim_f01 = 0U;
    ext->parfocal_last_f_mm = ctx->f_cur_mm;
    ext->parfocal_last_zoom = ctx->id06.zoom_norm;
    ext->parfocal_anchor_valid = (ctx->id06.valid != 0U) ? 1U : 0U;
    ext->parfocal_wait_vsyncs = 0U;
    ext->parfocal_trim_pending = 0U;
    ctx->parfocal_cmd_valid = 0U;
    ctx->parfocal_cl_trim_cz = 0;
    ctx->parfocal_cl_cooldown = 0U;
    ctx->parfocal_cl_steps = 0U;
    ctx->parfocal_min_rem_se = (uint16_t)MAX_U16_CLAMP;
    ctx->parfocal_anchor_refresh = 0U;
    if (ctx->id06.valid != 0U) {
        ctx->parfocal_anchor_refresh = (ctx->active_vsync_hz >= FAST_VSYNC_THRESH_HZ)
                                           ? (uint8_t)(PARFOCAL_ANCHOR_REFR * PARFOCAL_REFR_120_MULT)
                                           : PARFOCAL_ANCHOR_REFR;
    }

    int32_t anchor_diff = anchor_cz - servo->cur_cz;
    int32_t abs_anchor_diff = (anchor_diff >= 0) ? anchor_diff : -anchor_diff;
    uint16_t dist_x = ctx->id05.dist_code_x;
    bool latch_dx = (bool)((ctx->id06.valid != 0U) &&
                           (ctx->id06.hw_moving == 0U) &&
                           (ctx->id05.grp_present[4] != 0U) &&
                           (ctx->dist_report_invalid == 0U) &&
                           (ctx->id05_fresh_since_06 == (uint8_t)ID05_PAIR_EARLY_05) &&
                           is_parfocal_dist_code_valid(dist_x) &&
                           (abs_anchor_diff <= PARFOCAL_ANCHOR_CZ_TOL));
    ctx->parfocal_anchor_dist_x = latch_dx ? dist_x : 0U;
}

static void maybe_backfill_parfocal_anchor_dist_x(adapter_ctx_t *ctx) {
    if ((ctx->id06.valid == 0U) || (ctx->id06.hw_moving != 0U) ||
        (ctx->id05.grp_present[4] == 0U) || (ctx->dist_report_invalid != 0U) ||
        (ctx->id05_fresh_since_06 != (uint8_t)ID05_PAIR_EARLY_05)) {
        return;
    }
    if ((ctx->parfocal_cmd_valid != 0U) || (ctx->parfocal_hold_vsyncs != 0U) ||
        (ctx->af_ext.parfocal_trim_pending != 0U) || (ctx->af.af_drive_active != 0U) ||
        (ctx->af.e0_brake_rem != 0U) || (ctx->parfocal_e0_coast_rem != 0U)) {
        return;
    }
    if ((ctx->parfocal_anchor_dist_x != 0U) && (ctx->parfocal_anchor_refresh == 0U)) {
        return;
    }
    uint16_t dist_x = ctx->id05.dist_code_x;
    if (!is_parfocal_dist_code_valid(dist_x)) {
        return;
    }
    int32_t diff = ctx->af_ext.parfocal_cz_anchor - ctx->af.cur_cz;
    int32_t abs_diff = (diff >= 0) ? diff : -diff;
    if (abs_diff <= PARFOCAL_ANCHOR_CZ_TOL) {
        ctx->parfocal_anchor_dist_x = dist_x;
    }
}

static int32_t compute_parfocal_ff_cz(const adapter_ctx_t *ctx) {
    const af_servo_state_t *servo = &ctx->af;
    const af_v218_ext_t *ext = &ctx->af_ext;
    if (is_sigma_cam_zoom(ctx)) {
        return ext->parfocal_cz_anchor;
    }
    if ((ctx->parfocal_anchor_dist_x >= PARFOCAL_INF_CODE_X) &&
        (ctx->dist_verified_dynamic != 0U) && (ctx->dist_report_invalid == 0U)) {
        return (ext->parfocal_cz_anchor < 0) ? ext->parfocal_cz_anchor : 0;
    }
    if ((ext->parfocal_cz_anchor <= 0) || (servo->mech_hz <= 0)) {
        return ext->parfocal_cz_anchor;
    }
    float nodal_d0 = 0.0F;
    float net_mfd = current_parfocal_net_mfd(ctx, &nodal_d0);
    float denom = ctx->parfocal_dist_m - nodal_d0;
    float r_cur = (denom > net_mfd) ? (net_mfd / denom) : 1.0F;
    if (r_cur < 0.0F) {
        r_cur = 0.0F;
    }
    return round_f32_to_s32(r_cur * (float)servo->mech_hz);
}

static int32_t compute_parfocal_target_se(const adapter_ctx_t *ctx) {
    const af_servo_state_t *servo = &ctx->af;
    if (is_sigma_cam_zoom(ctx)) {
        return af_servo_cz_to_se(ctx, ctx->af_ext.parfocal_cz_anchor);
    }
    int32_t net_cz = compute_parfocal_ff_cz(ctx) + ctx->parfocal_cl_trim_cz;
    if (ctx->parfocal_cl_trim_cz != 0) {
        net_cz = clamp_s32(net_cz, 0, servo->mech_hz);
    }
    return af_servo_cz_to_se(ctx, net_cz);
}

static bool can_run_parfocal_dist_x_cl(const adapter_ctx_t *ctx) {
    return (bool)((!is_sigma_cam_zoom(ctx)) &&
                  (ctx->id06.valid != 0U) &&
                  (ctx->id05.grp_present[4] != 0U) &&
                  (ctx->dist_report_invalid == 0U) &&
                  (ctx->dist_verified_dynamic != 0U) &&
                  (ctx->id05_fresh_since_06 == (uint8_t)ID05_PAIR_EARLY_05) &&
                  (ctx->parfocal_anchor_dist_x > 0U) &&
                  (ctx->parfocal_anchor_dist_x < PARFOCAL_INF_CODE_X) &&
                  is_parfocal_dist_code_valid(ctx->id05.dist_code_x));
}

static int32_t parfocal_anchor_recover_cz(const adapter_ctx_t *ctx) {
    const af_servo_state_t *servo = &ctx->af;
    float dist_exp = ((float)ctx->parfocal_anchor_dist_x - PARFOCAL_DIST_X_REF) *
                     PARFOCAL_BINS_INV_64;
    float anchor_dist_m = boryoza_exp2f(dist_exp);
    float nodal_d0 = 0.0F;
    float net_mfd = current_parfocal_net_mfd(ctx, &nodal_d0);
    float denom = anchor_dist_m - nodal_d0;
    float est_cz = (denom > net_mfd)
                       ? ((net_mfd * (float)servo->mech_hz) / denom)
                       : (float)servo->mech_hz;
    return clamp_s32(round_f32_to_s32(est_cz), 1, servo->mech_hz);
}

static int32_t compute_parfocal_cl_delta_cz(const adapter_ctx_t *ctx, int32_t err_x,
                                            bool raw_inf, bool cont_damp) {
    const af_servo_state_t *servo = &ctx->af;
    int32_t abs_ex = (err_x >= 0) ? err_x : -err_x;
    if (abs_ex <= PARFOCAL_CL_ERR_DB) {
        return 0;
    }
    int32_t ref_cz = servo->cur_cz;
    if ((ref_cz <= 0) || raw_inf) {
        int32_t rec_cz = parfocal_anchor_recover_cz(ctx);
        if (rec_cz > ref_cz) {
            ref_cz = rec_cz;
        }
    }
    if (ref_cz <= 0) {
        return 0;
    }
    float cl_scale = cont_damp ? PARFOCAL_CONT_CL_DAMP : 1.0F;
    float ratio = boryoza_exp2f(PARFOCAL_CL_GAIN * (float)err_x * PARFOCAL_BINS_INV_64) - 1.0F;
    int32_t delta_cz = round_f32_to_s32((float)ref_cz * ratio * cl_scale);
    int32_t abs_cz = (delta_cz >= 0) ? delta_cz : -delta_cz;
    int32_t step_se = round_f32_to_s32((float)abs_cz / servo->cur_jacobian);
    if (step_se < PARFOCAL_MIN_CL_SE_STEP) {
        return 0;
    }
    return delta_cz;
}

static int32_t compute_feasible_parfocal_cl_step(const adapter_ctx_t *ctx, int32_t ff_cz,
                                                 bool cont_damp) {
    if ((!can_run_parfocal_dist_x_cl(ctx)) ||
        (ctx->parfocal_cl_steps >= MAX_PARFOCAL_CL_STEPS)) {
        return 0;
    }
    if (ctx->parfocal_cmd_valid != 0U) {
        int32_t cmd_err = ctx->parfocal_last_cmd_se - ctx->id06.focus_pos_se;
        int32_t abs_cmd_err = (cmd_err >= 0) ? cmd_err : -cmd_err;
        if (abs_cmd_err > (2 * parfocal_cmd_settle_tol_se(ctx))) {
            return 0;
        }
    }
    uint16_t raw_x = ctx->id05.dist_code_x;
    bool raw_inf = (bool)(raw_x >= PARFOCAL_INF_CODE_X);
    int32_t cur_x = raw_inf ? (int32_t)PARFOCAL_INF_CODE_X : (int32_t)raw_x;
    int32_t err_x = clamp_s32(cur_x - (int32_t)ctx->parfocal_anchor_dist_x,
                              -PARFOCAL_CL_MAX_BINS, PARFOCAL_CL_MAX_BINS);
    int32_t delta_cz = compute_parfocal_cl_delta_cz(ctx, err_x, raw_inf, cont_damp);
    if (delta_cz == 0) {
        return 0;
    }
    int32_t ff_bounded = clamp_s32(ff_cz, 0, ctx->af.mech_hz);
    int32_t cur_net_cz = ff_bounded + ctx->parfocal_cl_trim_cz;
    if ((delta_cz > 0) && ((ctx->id06.at_near_end != 0U) || (cur_net_cz >= ctx->af.mech_hz))) {
        return 0;
    }
    if ((delta_cz < 0) && ((ctx->id06.at_far_end != 0U) || (cur_net_cz <= 0))) {
        return 0;
    }
    return delta_cz;
}

static void rescale_parfocal_trim_on_zoom(adapter_ctx_t *ctx, int32_t ff_bounded) {
    if ((ctx->parfocal_cl_trim_cz == 0) || (ctx->parfocal_anchor_f01 == 0U)) {
        ctx->parfocal_cl_trim_cz = 0;
        return;
    }
    uint16_t cur_f01 = current_focal_0p1mm_u16(ctx);
    if (ctx->parfocal_trim_f01 == 0U) {
        ctx->parfocal_cl_trim_cz = clamp_s32(ctx->parfocal_cl_trim_cz, -ff_bounded,
                                             ctx->af.mech_hz - ff_bounded);
        return;
    }
    int32_t prev_span = (int32_t)ctx->parfocal_trim_f01 - (int32_t)ctx->parfocal_anchor_f01;
    int32_t cur_span = (int32_t)cur_f01 - (int32_t)ctx->parfocal_anchor_f01;
    int32_t abs_prev = (prev_span >= 0) ? prev_span : -prev_span;
    int32_t abs_cur = (cur_span >= 0) ? cur_span : -cur_span;
    bool same_side = (bool)(((prev_span > 0) && (cur_span > 0)) ||
                            ((prev_span < 0) && (cur_span < 0)));
    if (abs_prev < PARFOCAL_F01_DIFF_MIN) {
        if (abs_cur >= PARFOCAL_F01_DIFF_MIN) {
            ctx->parfocal_cl_trim_cz = 0;
            ctx->parfocal_trim_f01 = cur_f01;
        }
    } else if (!same_side) {
        ctx->parfocal_cl_trim_cz = 0;
        ctx->parfocal_trim_f01 = cur_f01;
    } else if (abs_cur < abs_prev) {
        float scale = (float)abs_cur / (float)abs_prev;
        ctx->parfocal_cl_trim_cz = round_f32_to_s32((float)ctx->parfocal_cl_trim_cz * scale);
        ctx->parfocal_trim_f01 = cur_f01;
    }
    ctx->parfocal_cl_trim_cz = clamp_s32(ctx->parfocal_cl_trim_cz, -ff_bounded,
                                         ctx->af.mech_hz - ff_bounded);
}

static uint8_t active_parfocal_cl_cooldown(const adapter_ctx_t *ctx) {
    return (ctx->active_vsync_hz >= FAST_VSYNC_THRESH_HZ)
               ? PARFOCAL_CL_COOLDOWN_120
               : PARFOCAL_CL_COOLDOWN;
}

static bool update_parfocal_dist_x_feedback(adapter_ctx_t *ctx, bool zoom_changed,
                                            int32_t ff_cz, bool allow_cont,
                                            bool tick_timers) {
    const af_servo_state_t *servo = &ctx->af;
    bool settled_for_cl = (ctx->id06.hw_moving == 0U);
    if (zoom_changed && allow_cont) {
        settled_for_cl = is_parfocal_motor_tracking_settled(ctx);
    }
    if (!settled_for_cl) {
        ctx->parfocal_cl_cooldown = active_parfocal_cl_cooldown(ctx);
        return false;
    }
    if ((ctx->af_ext.parfocal_trim_pending != 0U) && tick_timers) {
        return false;
    }
    if (zoom_changed && (!allow_cont)) {
        ctx->parfocal_cl_cooldown = active_parfocal_cl_cooldown(ctx);
        return false;
    }
    if (ctx->parfocal_cl_cooldown > 0U) {
        if (tick_timers) {
            ctx->parfocal_cl_cooldown = (uint8_t)(ctx->parfocal_cl_cooldown - 1U);
        }
        return false;
    }
    if (!can_run_parfocal_dist_x_cl(ctx)) {
        return false;
    }
    int32_t delta_cz = compute_feasible_parfocal_cl_step(ctx, ff_cz, zoom_changed);
    if (delta_cz == 0) {
        return false;
    }
    int32_t ff_bounded = clamp_s32(ff_cz, 0, servo->mech_hz);
    ctx->parfocal_cl_trim_cz = clamp_s32(ctx->parfocal_cl_trim_cz + delta_cz,
                                         -ff_bounded, servo->mech_hz - ff_bounded);
    ctx->parfocal_trim_f01 = current_focal_0p1mm_u16(ctx);
    ctx->parfocal_cl_steps = (uint8_t)(ctx->parfocal_cl_steps + 1U);
    ctx->parfocal_cl_cooldown = active_parfocal_cl_cooldown(ctx);
    return true;
}

static bool detect_idle_zoom_changed(const adapter_ctx_t *ctx, int32_t prev_e_inf,
                                     int32_t prev_e_near) {
    const af_servo_state_t *servo = &ctx->af;
    const af_v218_ext_t *ext = &ctx->af_ext;
    int32_t dzoom = (int32_t)ctx->id06.zoom_norm - (int32_t)ext->parfocal_last_zoom;
    if (dzoom < 0) {
        dzoom = -dzoom;
    }
    uint16_t cur_f01 = current_focal_0p1mm_u16(ctx);
    return (bool)((cur_f01 != ctx->parfocal_last_f01) ||
                  (dzoom >= PARFOCAL_ZOOM_STEP_DIFF) ||
                  (servo->e_inf_true != prev_e_inf) ||
                  (servo->e_near_true != prev_e_near));
}

static size_t encode_1d_abs_subrecord(int32_t target_se, uint8_t *out_sub) {
    static const uint8_t SUBREC_1D_OP  = 0x1DU;
    static const size_t  SUBREC_1D_LEN = 5U;
    uint32_t utgt = (uint32_t)(uint16_t)(int16_t)target_se;
    out_sub[0] = SUBREC_1D_OP;
    out_sub[1] = (uint8_t)(utgt & BYTE_MASK);
    out_sub[2] = (uint8_t)((utgt >> SHIFT_8) & BYTE_MASK);
    out_sub[3] = 0x00U;
    out_sub[4] = 0x00U;
    return SUBREC_1D_LEN;
}

static void reproject_active_af_on_zoom(adapter_ctx_t *ctx, int32_t prev_e_inf,
                                        int32_t prev_e_near, float prev_jac) {
    static const float JACOBIAN_DELTA_EPS = 1e-4F;
    af_servo_state_t *servo = &ctx->af;
    if ((ctx->f_max_mm > ctx->f_min_mm) &&
        ((servo->e_inf_true != prev_e_inf) || (servo->e_near_true != prev_e_near) ||
         (abs_f32_diff(servo->cur_jacobian, prev_jac) > JACOBIAN_DELTA_EPS) ||
         (current_focal_0p1mm_u16(ctx) != ctx->parfocal_last_f01))) {
        servo->target_se = af_servo_cz_to_se(ctx, servo->target_cz);
        if ((servo->unlimited_speed == 0U) && (servo->cmd_vel_cz_pps > 0.0F)) {
            servo->cmd_vel_se_pps = servo->cmd_vel_cz_pps / servo->cur_jacobian;
        }
    }
    sync_parfocal_anchor(ctx, servo->target_cz);
}

static uint8_t active_parfocal_hold_vsyncs(const adapter_ctx_t *ctx) {
    return (ctx->active_vsync_hz >= FAST_VSYNC_THRESH_HZ)
               ? PARFOCAL_HOLD_VSYNCS_120
               : PARFOCAL_HOLD_VSYNCS;
}

static uint8_t active_parfocal_post_slew_hold(const adapter_ctx_t *ctx) {
    return (ctx->active_vsync_hz >= FAST_VSYNC_THRESH_HZ)
               ? PARFOCAL_POST_SLEW_120
               : PARFOCAL_POST_SLEW_60;
}

static bool parfocal_slew_hit_new_min(adapter_ctx_t *ctx) {
    if ((ctx->id06.hw_moving == 0U) || (ctx->parfocal_cmd_valid == 0U)) {
        return false;
    }
    int32_t rem_cur = ctx->parfocal_last_cmd_se - ctx->id06.focus_pos_se;
    int32_t abs_cur = (rem_cur >= 0) ? rem_cur : -rem_cur;
    if (abs_cur <= parfocal_cmd_settle_tol_se(ctx)) {
        return false;
    }
    uint16_t cur_u16 = (uint16_t)clamp_s32(abs_cur, 0, MAX_U16_CLAMP);
    if (cur_u16 < ctx->parfocal_min_rem_se) {
        ctx->parfocal_min_rem_se = cur_u16;
        return true;
    }
    return false;
}

static bool is_parfocal_idle_settled(const adapter_ctx_t *ctx) {
    if ((ctx->id06.hw_moving != 0U) || (ctx->af_ext.parfocal_wait_vsyncs != 0U)) {
        return false;
    }
    int32_t ff_cz = compute_parfocal_ff_cz(ctx);
    bool wait_pair_05 = (bool)((ctx->id05_fresh_since_06 == (uint8_t)ID05_PAIR_WAIT_05) &&
                               (!is_sigma_cam_zoom(ctx)) &&
                               (ctx->dist_report_invalid == 0U));
    bool cl_cd_wait = (bool)(can_run_parfocal_dist_x_cl(ctx) &&
                             (ctx->parfocal_cl_cooldown > 0U));
    bool cl_wait = (bool)(cl_cd_wait ||
                          wait_pair_05 ||
                          (compute_feasible_parfocal_cl_step(ctx, ff_cz, false) != 0));
    if (cl_wait) {
        return false;
    }
    if (ctx->parfocal_cmd_valid == 0U) {
        return true;
    }
    int32_t cmd_rem = ctx->parfocal_last_cmd_se - ctx->id06.focus_pos_se;
    int32_t abs_cmd_rem = (cmd_rem >= 0) ? cmd_rem : -cmd_rem;
    return (bool)(abs_cmd_rem <= parfocal_cmd_settle_tol_se(ctx));
}

static void step_parfocal_hold_vsyncs(adapter_ctx_t *ctx, bool cl_step_queued) {
    af_v218_ext_t *ext = &ctx->af_ext;
    if ((ctx->parfocal_hold_vsyncs == 0U) || (ext->parfocal_trim_pending != 0U) ||
        cl_step_queued) {
        return;
    }
    if (parfocal_slew_hit_new_min(ctx)) {
        uint8_t post_hold = active_parfocal_post_slew_hold(ctx);
        if (ctx->parfocal_hold_vsyncs < post_hold) {
            ctx->parfocal_hold_vsyncs = post_hold;
        }
        return;
    }
    if (is_parfocal_idle_settled(ctx)) {
        ctx->parfocal_hold_vsyncs = 0U;
    } else {
        ctx->parfocal_hold_vsyncs = (uint8_t)(ctx->parfocal_hold_vsyncs - 1U);
    }
}

static void maybe_queue_parfocal_trim(adapter_ctx_t *ctx, int32_t tol_se) {
    af_servo_state_t *servo = &ctx->af;
    af_v218_ext_t *ext = &ctx->af_ext;
    int32_t want_se = compute_parfocal_target_se(ctx);
    int32_t err_se = want_se - ctx->id06.focus_pos_se;
    int32_t abs_err = (err_se >= 0) ? err_se : -err_se;
    int32_t cmd_diff = want_se - ctx->parfocal_last_cmd_se;
    int32_t abs_cmd_diff = (cmd_diff >= 0) ? cmd_diff : -cmd_diff;
    if ((abs_err > tol_se) &&
        ((ctx->parfocal_cmd_valid == 0U) || (abs_cmd_diff >= tol_se))) {
        ext->parfocal_trim_se = want_se;
        ext->parfocal_trim_pending = 1U;
        if ((servo->af_drive_active == 0U) && (servo->e0_brake_rem == 0U)) {
            servo->af_stop_req = 0U;
        }
    } else if ((abs_err <= tol_se) && (ctx->id06.hw_moving == 0U) &&
               (ctx->id06.focus_pos_se == ctx->id06.focus_prev_se)) {
        ctx->parfocal_cmd_valid = 0U;
    }
}

static void clear_settled_parfocal_cmd(adapter_ctx_t *ctx) {
    if ((ctx->parfocal_cmd_valid == 0U) || (ctx->id06.hw_moving != 0U) ||
        (ctx->af_ext.parfocal_trim_pending != 0U)) {
        return;
    }
    int32_t err_se = ctx->parfocal_last_cmd_se - ctx->id06.focus_pos_se;
    int32_t abs_err = (err_se >= 0) ? err_se : -err_se;
    int32_t settle_band = 2 * parfocal_cmd_settle_tol_se(ctx);
    if (abs_err <= settle_band) {
        if (ctx->id06.focus_pos_se == ctx->id06.focus_prev_se) {
            ctx->parfocal_cmd_valid = 0U;
        }
    } else if (ctx->parfocal_hold_vsyncs == 0U) {
        ctx->parfocal_cmd_valid = 0U;
    }
}

static void handle_idle_zoom_parfocal(adapter_ctx_t *ctx, int32_t tol_se, bool tick_timers) {
    static const uint8_t PARFOCAL_SETTLE_VSYNCS = 2U;
    af_servo_state_t *servo = &ctx->af;
    af_v218_ext_t *ext = &ctx->af_ext;
    ctx->parfocal_anchor_refresh = 0U;
    uint16_t cur_f01 = current_focal_0p1mm_u16(ctx);
    int32_t ff_cz = compute_parfocal_ff_cz(ctx);
    int32_t ff_bounded = clamp_s32(ff_cz, 0, servo->mech_hz);
    rescale_parfocal_trim_on_zoom(ctx, ff_bounded);
    int32_t est_cz = ff_bounded + ctx->parfocal_cl_trim_cz;
    int32_t step_cz = est_cz - servo->cur_cz;
    int32_t abs_step_cz = (step_cz >= 0) ? step_cz : -step_cz;
    bool no_pending = (bool)((ext->parfocal_trim_pending == 0U) || (!tick_timers));
    bool allow_cont = (bool)(is_parfocal_motor_tracking_settled(ctx) && no_pending &&
                             (servo->mech_hz > 0) &&
                             (abs_step_cz <= (servo->mech_hz / PARFOCAL_CONT_STEP_DIV)));
    ctx->parfocal_cl_steps = 0U;
    ext->parfocal_wait_vsyncs = PARFOCAL_SETTLE_VSYNCS;
    ctx->parfocal_hold_vsyncs = active_parfocal_hold_vsyncs(ctx);
    (void)update_parfocal_dist_x_feedback(ctx, true, ff_cz, allow_cont, tick_timers);
    maybe_queue_parfocal_trim(ctx, tol_se);
    ctx->parfocal_last_f01 = cur_f01;
    ext->parfocal_last_f_mm = ctx->f_cur_mm;
    ext->parfocal_last_zoom = ctx->id06.zoom_norm;
}

static void step_idle_parfocal_core(adapter_ctx_t *ctx, int32_t prev_e_inf,
                                    int32_t prev_e_near, bool tick_timers) {
    af_servo_state_t *servo = &ctx->af;
    af_v218_ext_t *ext = &ctx->af_ext;

    if ((ctx->id06.valid == 0U) || (ctx->f_max_mm <= ctx->f_min_mm) ||
        (ctx->af_class == AF_CLASS_B_LEGACY_F1_F2) ||
        (servo->mf_hold_frames_rem > 0U) || (ctx->parfocal_e0_coast_rem > 0U)) {
        ctx->parfocal_hold_vsyncs = 0U;
        if (ctx->parfocal_e0_coast_rem == 0U) {
            sync_parfocal_anchor(ctx, servo->cur_cz);
        }
        return;
    }
    if (ext->parfocal_anchor_valid == 0U) {
        ctx->parfocal_hold_vsyncs = 0U;
        sync_parfocal_anchor(ctx, servo->cur_cz);
        return;
    }

    int32_t tol_se = capture_tolerance_se(ctx);
    if (detect_idle_zoom_changed(ctx, prev_e_inf, prev_e_near)) {
        handle_idle_zoom_parfocal(ctx, tol_se, tick_timers);
        return;
    }

    maybe_backfill_parfocal_anchor_dist_x(ctx);
    if (tick_timers && (ctx->parfocal_anchor_refresh > 0U) && (ctx->id06.hw_moving == 0U)) {
        ctx->parfocal_anchor_refresh = (uint8_t)(ctx->parfocal_anchor_refresh - 1U);
    }

    bool cl_updated = false;
    if (ctx->parfocal_hold_vsyncs > 0U) {
        int32_t ff_cz = compute_parfocal_ff_cz(ctx);
        cl_updated = update_parfocal_dist_x_feedback(ctx, false, ff_cz, false, tick_timers);
        if (cl_updated) {
            maybe_queue_parfocal_trim(ctx, tol_se);
        }
    }
    if (tick_timers) {
        if ((ext->parfocal_wait_vsyncs > 0U) && (ctx->id06.hw_moving == 0U)) {
            ext->parfocal_wait_vsyncs = (uint8_t)(ext->parfocal_wait_vsyncs - 1U);
            if (ext->parfocal_wait_vsyncs == 0U) {
                maybe_queue_parfocal_trim(ctx, tol_se);
            }
        }
        step_parfocal_hold_vsyncs(ctx, cl_updated);
    } else if ((ctx->parfocal_hold_vsyncs > 0U) && (ext->parfocal_trim_pending == 0U) &&
               (!cl_updated) && is_parfocal_idle_settled(ctx)) {
        ctx->parfocal_hold_vsyncs = 0U;
    }
    clear_settled_parfocal_cmd(ctx);
}

void af_servo_on_standalone_id05(adapter_ctx_t *ctx, bool paired_after_06) {
    if (ctx == NULL) {
        return;
    }
    af_servo_state_t *servo = &ctx->af;
    sync_live_id05_focal_and_dist_flag(ctx);
    if ((ctx->id05.mf_ring_dir != 0) && (servo->af_drive_active == 0U) &&
        (ctx->init_sync_rem == 0U) && (ctx->parfocal_e0_coast_rem == 0U)) {
        update_manual_focus_ring_state(ctx, false);
        return;
    }
    if (!paired_after_06) {
        return;
    }
    int32_t prev_e_inf = servo->e_inf_true;
    int32_t prev_e_near = servo->e_near_true;
    float prev_jac = servo->cur_jacobian;
    refresh_cam_endpoints_and_jacobian(ctx);
    servo->cur_cz = snap_endstop_cz(ctx, af_servo_se_to_cz(ctx, ctx->id06.focus_pos_se));
    servo->cur_dist_m = ((servo->cur_cz > 0) && (servo->mech_hz > 0))
                            ? ((af_servo_effective_mfd_meters(ctx) * (float)servo->mech_hz) /
                               (float)servo->cur_cz)
                            : FAR_INF_DIST_M;
    if (servo->af_drive_active != 0U) {
        reproject_active_af_on_zoom(ctx, prev_e_inf, prev_e_near, prev_jac);
        return;
    }
    step_idle_parfocal_core(ctx, prev_e_inf, prev_e_near, false);
}

static void update_e0_brake_and_coast_state(adapter_ctx_t *ctx) {
    af_servo_state_t *servo = &ctx->af;
    bool brake_wait = (bool)((servo->e0_brake_rem > 0U) || (ctx->parfocal_e0_coast_rem > 0U));
    if (ctx->id06.hw_moving == 0U) {
        if (brake_wait) {
            sync_parfocal_anchor(ctx, servo->cur_cz);
        }
        servo->e0_brake_rem = 0U;
        ctx->parfocal_e0_coast_rem = 0U;
        ctx->af_ext.last_hot_cz = servo->cur_cz;
        return;
    }
    if ((servo->af_stop_req == 0U) && (servo->e0_brake_rem > 0U)) {
        servo->e0_brake_rem = (uint8_t)(servo->e0_brake_rem - 1U);
    }
    if (ctx->parfocal_e0_coast_rem > 0U) {
        ctx->parfocal_e0_coast_rem = (uint8_t)(ctx->parfocal_e0_coast_rem - 1U);
        if (ctx->parfocal_e0_coast_rem == 0U) {
            sync_parfocal_anchor(ctx, servo->cur_cz);
        }
    }
}

void af_servo_update_telemetry(adapter_ctx_t *ctx, uint64_t now_ticks) {
    af_servo_state_t *servo = &ctx->af;

    if ((ctx->af_limits_seen == 0U) &&
        (ctx->id06.valid != 0U) && (ctx->id06.lim_near_se != ctx->id06.lim_far_se)) {
        servo->e_inf_true = ctx->id06.lim_far_se;
        servo->e_near_true = ctx->id06.lim_near_se;
        ctx->af_limits_seen = 1U;
    }

    sync_live_id05_focal_and_dist_flag(ctx);
    int32_t prev_e_inf = servo->e_inf_true;
    int32_t prev_e_near = servo->e_near_true;
    float prev_jac = servo->cur_jacobian;
    refresh_cam_endpoints_and_jacobian(ctx);
    servo->cur_cz = snap_endstop_cz(ctx, af_servo_se_to_cz(ctx, ctx->id06.focus_pos_se));
    servo->cur_dist_m = ((servo->cur_cz > 0) && (servo->mech_hz > 0))
                            ? ((af_servo_effective_mfd_meters(ctx) * (float)servo->mech_hz) / (float)servo->cur_cz)
                            : FAR_INF_DIST_M;

    clear_expired_cushion_latches(ctx);

    if (servo->af_drive_active != 0U) {
        reproject_active_af_on_zoom(ctx, prev_e_inf, prev_e_near, prev_jac);
        step_active_af_telemetry(ctx, now_ticks);
        servo->cur_cz = snap_endstop_cz(ctx, servo->cur_cz);
        if (servo->af_drive_active != 0U) {
            return;
        }
        sync_parfocal_anchor(ctx, servo->cur_cz);
    }
    ctx->af_ext.active_step_se = 0;
    update_e0_brake_and_coast_state(ctx);
    bool idle_zoom = detect_idle_zoom_changed(ctx, prev_e_inf, prev_e_near);
    if ((ctx->af_ext.parfocal_anchor_valid != 0U) && (ctx->f_max_mm > ctx->f_min_mm) &&
        (ctx->af_class != AF_CLASS_B_LEGACY_F1_F2) &&
        (servo->mf_hold_frames_rem == 0U) && (ctx->parfocal_e0_coast_rem == 0U) &&
        idle_zoom) {
        ctx->parfocal_hold_vsyncs = active_parfocal_hold_vsyncs(ctx);
    }
    update_manual_focus_ring_state(ctx, idle_zoom);
    step_idle_parfocal_core(ctx, prev_e_inf, prev_e_near, true);
}

static bool is_fast_sony_xd(const adapter_ctx_t *ctx) {
    return (bool)((ctx->brand == LENS_BRAND_SONY) &&
                  ((ctx->supports_120hz != 0U) || (ctx->lens_family == 2U)));
}

static float lens_max_velocity_tps(const adapter_ctx_t *ctx) {
    float vmax = is_fast_sony_xd(ctx) ? SONY_XD_VMAX_SE_TPS : DEFAULT_VMAX_SE_TPS;
    if ((ctx->e_lens_id == SIGMA_150600_REV2) || (ctx->e_lens_id == SIGMA_150600_REV1)) {
        vmax = (float)sigma_150600_thermal_vmax_tps(ctx->id06.temp_celsius);
    }
    if (ctx->is_dual_floating != 0U) {
        float beta = (float)ctx->ap.z_mag_a3_q14 / Q14_SCALE_F;
        if (beta > DUAL_FLOAT_BETA_THRESH) {
            vmax *= DUAL_FLOAT_VEL_MARGIN;
        }
    }
    return vmax;
}

/* NOLINTBEGIN(readability-magic-numbers) — composite focus sub-record opcodes */
static int32_t compute_frame_step_se(const adapter_ctx_t *ctx, uint8_t frame_rate) {
    const af_servo_state_t *servo = &ctx->af;
    float vmax_tps = lens_max_velocity_tps(ctx);
    float req_tps = (servo->unlimited_speed != 0U) ? vmax_tps : servo->cmd_vel_se_pps;
    if (req_tps > vmax_tps) {
        req_tps = vmax_tps;
    }
    uint8_t fps_u8 = (frame_rate != 0U) ? frame_rate : DEFAULT_ACTIVE_FPS;
    int32_t step_per_vd = round_f32_to_s32(req_tps / (float)fps_u8);
    return (step_per_vd >= 1) ? step_per_vd : 1;
}

static int32_t compute_vsync_step_se(const adapter_ctx_t *ctx) {
    return compute_frame_step_se(ctx, ctx->active_vsync_hz);
}

static bool can_single_shot_direct_abs(const adapter_ctx_t *ctx, bool fam2_ext) {
    const af_servo_state_t *servo = &ctx->af;
    if ((ctx->e_lens_id == LEGACY_ADAPTER_ID) || (servo->is_continuous_track != 0U)) {
        return false;
    }
    if (servo->unlimited_speed != 0U) {
        return true;
    }
    return (bool)(fam2_ext && ((servo->cmd_vel_cz_pps >= FAST_JUMP_CZ_VPPS) ||
                               (servo->cmd_vel_se_pps >= lens_max_velocity_tps(ctx))));
}

typedef enum {
    STEP_ACT_REL = 0,
    STEP_ACT_ABS = 1,
    STEP_ACT_HOLD = 2,
} step_act_t;

typedef struct {
    step_act_t act;
    int32_t    step;
} step_decision_t;

static int32_t compute_1d_max_span_step(const adapter_ctx_t *ctx) {
    const af_servo_state_t *servo = &ctx->af;
    int32_t raw_span = servo->e_near_true - servo->e_inf_true;
    int32_t abs_span = (raw_span >= 0) ? raw_span : -raw_span;
    if (abs_span < MIN_SPAN_TICKS) {
        abs_span = DEFAULT_SPAN_TICKS;
    }
    int32_t max_step = abs_span / 8;
    if ((ctx->brand == LENS_BRAND_SIGMA) && (servo->is_continuous_track != 0U)) {
        max_step = abs_span / 12;
    } else if (is_fast_sony_xd(ctx)) {
        max_step = abs_span / 4;
    }
    return (max_step < MIN_1D_STEP_SE) ? MIN_1D_STEP_SE : max_step;
}

static void prime_or_advance_accum(af_servo_state_t *servo, int32_t signed_inc) {
    if (((servo->step_accum_se > 0) && (signed_inc < 0)) ||
        ((servo->step_accum_se < 0) && (signed_inc > 0))) {
        servo->step_accum_se = 0;
    }
    if (servo->track_primed == 0U) {
        servo->track_primed = 1U;
        servo->step_accum_se = (signed_inc >= 0) ? TRACK_PULSE_MIN_SE : -TRACK_PULSE_MIN_SE;
    } else {
        servo->step_accum_se += signed_inc;
    }
}

static bool eval_fam2_approach(const adapter_ctx_t *ctx, int32_t err_se, bool fam2_ext,
                               int32_t step_vd, int32_t *out_tapered) {
    const af_servo_state_t *servo = &ctx->af;
    int32_t min_step = (servo->is_continuous_track != 0U) ? TRACK_PULSE_MIN_SE : MIN_1D_STEP_SE;
    int32_t pred_err = servo->target_se - ctx->af_predicted_se;
    int32_t abs_pred = (pred_err >= 0) ? pred_err : -pred_err;
    bool sign_crossed = (bool)(((err_se > 0) && (pred_err < 0)) ||
                               ((err_se < 0) && (pred_err > 0)));
    *out_tapered = step_vd;
    if (fam2_ext && (ctx->id06.hw_moving != 0U) &&
        ((abs_pred <= step_vd) || sign_crossed)) {
        return true;
    }
    if (fam2_ext && (servo->target_cz > servo->mech_lz) &&
        (servo->target_cz < servo->mech_hz) && (abs_pred < (FAM2_DECEL_BAND * step_vd))) {
        int32_t env_step = round_f32_to_s32(
            boryoza_sqrtf(FAM2_DECEL_GAIN * (float)step_vd * (float)abs_pred));
        *out_tapered = clamp_s32(env_step, min_step, step_vd);
    }
    return false;
}

static step_decision_t decide_continuous_1d(adapter_ctx_t *ctx, int32_t err_se, bool fam2_ext) {
    af_servo_state_t *servo = &ctx->af;
    int32_t abs_err = (err_se >= 0) ? err_se : -err_se;
    int32_t raw_step = compute_vsync_step_se(ctx);
    int32_t max_span_step = compute_1d_max_span_step(ctx);
    if (raw_step >= TRACK_PULSE_MIN_SE) {
        servo->track_primed = 1U;
        servo->step_accum_se = 0;
        int32_t step_vd = clamp_s32(raw_step, TRACK_PULSE_MIN_SE, max_span_step);
        int32_t tapered_step = step_vd;
        bool pred_land = eval_fam2_approach(ctx, err_se, fam2_ext, step_vd, &tapered_step);
        if ((abs_err <= step_vd) || pred_land) {
            return (step_decision_t){STEP_ACT_ABS, 0};
        }
        return (step_decision_t){STEP_ACT_REL, (err_se >= 0) ? tapered_step : -tapered_step};
    }
    int32_t signed_inc = (err_se >= 0) ? raw_step : -raw_step;
    prime_or_advance_accum(servo, signed_inc);
    int32_t abs_acc = (servo->step_accum_se >= 0) ? servo->step_accum_se : -servo->step_accum_se;
    if (abs_acc < TRACK_PULSE_MIN_SE) {
        if ((abs_err < TRACK_PULSE_MIN_SE) && (abs_acc >= abs_err)) {
            servo->step_accum_se = 0;
            return (step_decision_t){STEP_ACT_ABS, 0};
        }
        return (step_decision_t){STEP_ACT_HOLD, 0};
    }
    int32_t pulse_mag = clamp_s32(abs_acc, TRACK_PULSE_MIN_SE, max_span_step);
    int32_t signed_acc = servo->step_accum_se;
    servo->step_accum_se = 0;
    if (abs_err <= pulse_mag) {
        return (step_decision_t){STEP_ACT_ABS, 0};
    }
    return (step_decision_t){STEP_ACT_REL, (signed_acc >= 0) ? pulse_mag : -pulse_mag};
}

static step_decision_t decide_singleshot_1d(const adapter_ctx_t *ctx, int32_t err_se,
                                            bool fam2_ext) {
    const af_servo_state_t *servo = &ctx->af;
    int32_t abs_err = (err_se >= 0) ? err_se : -err_se;
    bool wall_sweep = (bool)((servo->target_cz <= servo->mech_lz) ||
                             (servo->target_cz >= servo->mech_hz));
    int32_t min_step = (is_fast_sony_xd(ctx) && wall_sweep)
                           ? SONY_XD_SWEEP_MIN_STEP_SE
                           : MIN_1D_STEP_SE;
    int32_t step_per_vd = clamp_s32(compute_vsync_step_se(ctx), min_step,
                                    compute_1d_max_span_step(ctx));
    int32_t tapered_step = step_per_vd;
    bool pred_landing = eval_fam2_approach(ctx, err_se, fam2_ext, step_per_vd, &tapered_step);
    if (can_single_shot_direct_abs(ctx, fam2_ext) || (abs_err <= step_per_vd) ||
        (abs_err <= capture_tolerance_se(ctx)) || pred_landing) {
        return (step_decision_t){STEP_ACT_ABS, 0};
    }
    return (step_decision_t){STEP_ACT_REL, (err_se >= 0) ? tapered_step : -tapered_step};
}

static size_t format_1d_subrecord(adapter_ctx_t *ctx, int32_t err_se, uint8_t *out_sub) {
    af_servo_state_t *servo = &ctx->af;
    int32_t abs_err = (err_se >= 0) ? err_se : -err_se;
    bool fam2_ext = (bool)((ctx->lens_family == 2U) && (ctx->brand != LENS_BRAND_SIGMA));
    step_decision_t dec = {STEP_ACT_ABS, 0};

    if (servo->unlimited_speed != 0U) {
        servo->step_accum_se = 0;
    } else if (servo->is_continuous_track != 0U) {
        if (can_single_shot_direct_abs(ctx, fam2_ext) ||
            (abs_err <= capture_tolerance_se(ctx))) {
            servo->step_accum_se = 0;
        } else {
            dec = decide_continuous_1d(ctx, err_se, fam2_ext);
            if (dec.act == STEP_ACT_HOLD) {
                ctx->af_ext.active_step_se = 0;
                return 0U;
            }
        }
    } else {
        servo->step_accum_se = 0;
        dec = decide_singleshot_1d(ctx, err_se, fam2_ext);
    }

    if (dec.act == STEP_ACT_ABS) {
        ctx->af_ext.active_step_se = 0;
        servo->prev_drive_dir_se = (err_se >= 0) ? 1 : -1;
        size_t len = encode_1d_abs_subrecord(servo->target_se, out_sub);
        servo->af_cmd_pending = 0U;
        servo->af_submode = 0U;
        servo->stable_idle_count = 0U;
        return len;
    }

    out_sub[0] = 0x1DU;

    ctx->af_ext.active_step_se = (dec.step >= 0) ? dec.step : -dec.step;
    servo->prev_drive_dir_se = (dec.step >= 0) ? 1 : -1;
    uint32_t ustep = (uint32_t)(uint16_t)(int16_t)dec.step;
    out_sub[1] = (uint8_t)(ustep & BYTE_MASK);
    out_sub[2] = (uint8_t)((ustep >> SHIFT_8) & BYTE_MASK);
    out_sub[3] = (fam2_ext && (dec.step < 0)) ? 0xFFU : 0x00U;
    out_sub[4] = fam2_ext ? 0x3CU : 0x2CU;
    return 5U;
}

static size_t format_3c_subrecord(adapter_ctx_t *ctx, int32_t err_se, uint8_t *out_sub) {
    af_servo_state_t *servo = &ctx->af;
    int32_t abs_err = (err_se >= 0) ? err_se : -err_se;
    int32_t nominal_step = compute_vsync_step_se(ctx);
    if (abs_err <= nominal_step) {
        ctx->af_ext.active_step_se = 0;
        servo->af_submode = 1U;
        out_sub[0] = 0x1CU;
        return 1U;
    }

    ctx->af_ext.active_step_se = nominal_step;
    servo->seq_3c = (uint8_t)(((uint32_t)servo->seq_3c + 1UL) & SEQ_3C_MASK);
    uint8_t ctrl = (uint8_t)(0x01UL | ((uint32_t)servo->seq_3c << 2U));
    int32_t signed_step = (err_se >= 0) ? nominal_step : -nominal_step;
    servo->prev_drive_dir_se = (signed_step >= 0) ? 1 : -1;
    int32_t k_div = (ctx->id06.defocus_k_se != 0U) ? (int32_t)ctx->id06.defocus_k_se : 16;
    int32_t scaled = clamp_s32((signed_step * 16) / k_div, -MAX_3C_VEL_CLAMP, MAX_3C_VEL_CLAMP);
    uint32_t uvel = (uint32_t)(uint16_t)(int16_t)scaled;
    if ((ctx->brand == LENS_BRAND_SIGMA) || (ctx->e_lens_id == 0xC149U)) {
        int32_t minimum = (16 + k_div - 1) / k_div;
        if ((scaled < minimum) && (scaled > -minimum)) {
            uvel = (uint32_t)(uint16_t)(int16_t)((err_se >= 0) ? minimum : -minimum);
        }
    } else {
        uvel &= 0xFFF0UL;
        if (uvel == 0UL) {
            uvel = (err_se >= 0) ? 0x0010UL : 0xFFF0UL;
        }
    }

    out_sub[0] = 0x3CU;
    out_sub[1] = ctrl;
    out_sub[2] = (uint8_t)(uvel & BYTE_MASK);
    out_sub[3] = (uint8_t)((uvel >> SHIFT_8) & BYTE_MASK);
    out_sub[4] = 0x00U;
    out_sub[5] = 0x00U;
    out_sub[6] = 0x00U;
    out_sub[7] = 0x00U;
    return 8U;
}

static uint8_t emit_stall_abort_stop(adapter_ctx_t *ctx, bool body_in_mf,
                                     uint8_t *out_sub, size_t *out_sub_len,
                                     uint8_t *out_b16) {
    af_servo_state_t *servo = &ctx->af;
    servo->af_cmd_pending = 0U;
    servo->af_drive_active = 0U;
    servo->af_submode = 0U;
    servo->prev_drive_dir_se = 0;
    servo->step_accum_se = 0;
    servo->track_primed = 0U;
    servo->stalled_abort = (servo->arrival_confirmed == 0U) ? STALL_ABORT_HOT_FRAMES : 0U;
    servo->settle_frames_rem = SETTLE_HOLD_FRAMES;
    if (servo->arrival_confirmed == 0U) {
        if (ctx->diag_af_stall_count < 0xFFFFU) {
            ctx->diag_af_stall_count = (uint16_t)(ctx->diag_af_stall_count + 1U);
        }
        if (ctx->bb_stall_logged == 0U) {
            ctx->bb_stall_logged = 1U;
            uint32_t info0 = ((uint32_t)ctx->af_ext.last_z_af_cmd) |
                             ((uint32_t)servo->af_submode << 8U) |
                             ((uint32_t)servo->drive_vsyncs << 16U);
            lens_session_push_diag(ctx, BB_EVT_AF_STALL_ABORT, info0,
                                   (uint32_t)servo->target_se,
                                   (uint32_t)ctx->id06.focus_pos_se,
                                   (uint32_t)servo->cur_cz);
        }
    }
    ctx->parfocal_hold_vsyncs = 0U;
    ctx->parfocal_e0_coast_rem = 0U;
    sync_parfocal_anchor(ctx, servo->cur_cz);
    out_sub[0] = 0x1CU;
    *out_sub_len = 1U;
    *out_b16 = ID04_B16_APSC_STOP;
    return body_in_mf ? ID04_B9_IDLE_MF : ID04_B9_SERVO_ON;
}

static bool is_idle_servo_engaged(const adapter_ctx_t *ctx, bool body_in_mf) {
    const af_servo_state_t *servo = &ctx->af;
    if (ctx->parfocal_hold_vsyncs > 0U) {
        return true;
    }
    if (servo->mf_hold_frames_rem != 0U) {
        return false;
    }
    if ((servo->af_drive_active != 0U) ||
        ((servo->e0_brake_rem > 0U) && (ctx->id06.hw_moving != 0U))) {
        return true;
    }
    return (bool)((!body_in_mf) &&
                  ((servo->s1_hold_vsyncs > 0U) || (servo->settle_frames_rem > 0U)));
}

static uint8_t format_idle_id04_tail(adapter_ctx_t *ctx, bool body_in_mf,
                                     uint8_t drive_b16, uint8_t *out_sub, size_t *out_sub_len,
                                     uint8_t *out_b16) {
    af_servo_state_t *servo = &ctx->af;
    uint8_t stall_limit = active_stall_guard_vsyncs(ctx);
    if (servo->af_drive_active != 0U) {
        servo->stall_vsyncs = (uint8_t)(servo->stall_vsyncs + 1U);
        if (servo->stall_vsyncs > stall_limit) {
            return emit_stall_abort_stop(ctx, body_in_mf, out_sub, out_sub_len, out_b16);
        }
    }
    if (servo->mf_hold_frames_rem != 0U) {
        servo->s1_hold_vsyncs = 0U;
    } else if (servo->s1_hold_vsyncs > 0U) {
        servo->s1_hold_vsyncs = (uint8_t)(servo->s1_hold_vsyncs - 1U);
    }
    if ((ctx->af_ext.hotline_seen == 0U) && (servo->settle_frames_rem > 0U)) {
        servo->settle_frames_rem = (uint8_t)(servo->settle_frames_rem - 1U);
    }

    if ((ctx->af_ext.parfocal_trim_pending != 0U) &&
        (servo->mf_hold_frames_rem == 0U)) {
        bool prev_tracking_ok = is_parfocal_motor_tracking_settled(ctx);
        int32_t new_step_se = ctx->af_ext.parfocal_trim_se - ctx->id06.focus_pos_se;
        int32_t abs_new_se = (new_step_se >= 0) ? new_step_se : -new_step_se;
        int32_t abs_new_cz = round_f32_to_s32((float)abs_new_se * servo->cur_jacobian);
        bool small_cont_step = (bool)((servo->mech_hz > 0) &&
                                      (abs_new_cz <= (servo->mech_hz / PARFOCAL_CONT_STEP_DIV)));
        bool same_cmd = (bool)((ctx->parfocal_cmd_valid != 0U) &&
                               (ctx->af_ext.parfocal_trim_se == ctx->parfocal_last_cmd_se));
        ctx->af_ext.parfocal_trim_pending = 0U;
        ctx->af_ext.active_step_se = 0;
        ctx->parfocal_last_cmd_se = ctx->af_ext.parfocal_trim_se;
        ctx->parfocal_cmd_valid = 1U;
        ctx->parfocal_min_rem_se = (uint16_t)MAX_U16_CLAMP;
        ctx->parfocal_hold_vsyncs = active_parfocal_hold_vsyncs(ctx);
        if ((!same_cmd) && ((!prev_tracking_ok) || (!small_cont_step))) {
            ctx->parfocal_cl_cooldown = active_parfocal_cl_cooldown(ctx);
        }
        *out_sub_len = encode_1d_abs_subrecord(ctx->af_ext.parfocal_trim_se, out_sub);
        *out_b16 = drive_b16;
        return ID04_B9_SERVO_ON;
    }

    bool servo_engaged = is_idle_servo_engaged(ctx, body_in_mf);
    *out_b16 = drive_b16;
    *out_sub_len = 0U;
    return servo_engaged ? ID04_B9_SERVO_ON : ID04_B9_IDLE_MF;
}

static void issue_controlled_brake(adapter_ctx_t *ctx);

static uint8_t format_pending_af_id04(adapter_ctx_t *ctx, bool body_in_mf,
                                      uint8_t drive_b16, uint8_t *out_sub,
                                      size_t *out_sub_len, uint8_t *out_b16) {
    af_servo_state_t *servo = &ctx->af;
    uint8_t stall_limit = active_stall_guard_vsyncs(ctx);
    servo->s1_hold_vsyncs = active_s1_hold_vsyncs(ctx);
    if (servo->af_submode == 2U) {
        servo->stall_vsyncs = (uint8_t)(servo->stall_vsyncs + 1U);
        if (servo->stall_vsyncs > stall_limit) {
            return emit_stall_abort_stop(ctx, body_in_mf, out_sub, out_sub_len, out_b16);
        }
        if ((ctx->id06.hw_moving == 0U) || (servo->stall_vsyncs >= PRE_STOP_MAX_VSYNCS)) {
            bool resume_3c = false;
            if (can_lens_use_3c_scan(ctx) && (servo->is_continuous_track != 0U) &&
                (servo->unlimited_speed == 0U)) {
                resume_3c = true;
            }
            servo->af_submode = resume_3c ? 3U : 1U;
        } else {
            out_sub[0] = 0x1CU;
            *out_sub_len = 1U;
            *out_b16 = ID04_B16_APSC_STOP;
            return ID04_B9_SERVO_ON;
        }
    }

    int32_t err_se = servo->target_se - ctx->id06.focus_pos_se;
    size_t sub_len = 0U;
    if (servo->af_submode == 3U) {
        sub_len = format_3c_subrecord(ctx, err_se, out_sub);
        *out_b16 = (out_sub[0] == 0x1CU) ? ID04_B16_APSC_STOP : drive_b16;
    } else {
        sub_len = format_1d_subrecord(ctx, err_se, out_sub);
        *out_b16 = drive_b16;
    }
    *out_sub_len = sub_len;
    if ((sub_len != 0U) || (ctx->id06.hw_moving != 0U)) {
        servo->stall_vsyncs = (uint8_t)(servo->stall_vsyncs + 1U);
        if (servo->stall_vsyncs > stall_limit) {
            return emit_stall_abort_stop(ctx, body_in_mf, out_sub, out_sub_len, out_b16);
        }
    }
    return ID04_B9_SERVO_ON;
}

uint8_t af_servo_build_id04_subrecord(adapter_ctx_t *ctx, uint8_t *out_sub,
                                      size_t *out_sub_len, uint8_t *out_b16) {
    af_servo_state_t *servo = &ctx->af;
    uint8_t drive_b16 = (ctx->is_apsc != 0U) ? ID04_B16_APSC_STOP : ID04_B16_FF_DRIVE;
    bool body_in_mf = is_body_in_manual_focus(ctx);

    if (ctx->init_sync_rem > 0U) {
        ctx->init_sync_rem = (uint8_t)(ctx->init_sync_rem - 1U);
        if (ctx->init_sync_rem == 0U) {
            servo->s1_hold_vsyncs = 0U;
        }
        out_sub[0] = 0x1DU;
        out_sub[1] = 0x00U;
        out_sub[2] = 0x00U;
        out_sub[3] = 0x00U;
        out_sub[4] = 0x2CU;
        *out_sub_len = 5U;
        *out_b16 = ID04_B16_SYNC_ZERO;
        return ID04_B9_SERVO_ON;
    }

    if (servo->af_stop_req != 0U) {
        servo->af_stop_req = 0U;
        servo->af_cmd_pending = 0U;
        servo->af_drive_active = 0U;
        servo->af_submode = 0U;
        servo->prev_drive_dir_se = 0;
        servo->step_accum_se = 0;
        servo->track_primed = 0U;
        bool need_active_stop = (bool)((ctx->id06.hw_moving != 0U) ||
                                       (servo->settle_frames_rem != 0U));
        if (ctx->id06.hw_moving == 0U) {
            servo->e0_brake_rem = 0U;
            servo->s1_hold_vsyncs = 0U;
            servo->settle_frames_rem = 0U;
        } else {
            servo->s1_hold_vsyncs = body_in_mf ? 0U : active_s1_hold_vsyncs(ctx);
        }
        out_sub[0] = 0x1CU;
        *out_sub_len = 1U;
        *out_b16 = ID04_B16_APSC_STOP;
        return (body_in_mf && (!need_active_stop)) ? ID04_B9_IDLE_MF : ID04_B9_SERVO_ON;
    }

    if (servo->af_cmd_pending != 0U) {
        return format_pending_af_id04(ctx, body_in_mf, drive_b16, out_sub, out_sub_len, out_b16);
    }

    return format_idle_id04_tail(ctx, body_in_mf, drive_b16, out_sub, out_sub_len, out_b16);
}

void af_servo_emit_vsync_drive(adapter_ctx_t *ctx) {
    af_servo_state_t *servo = &ctx->af;
    if ((servo->af_drive_active == 0U) || (ctx->session_state != E_SESSION_ACTIVE)) {
        return;
    }
    if (ctx->af_class == AF_CLASS_B_LEGACY_F1_F2) {
        int32_t step_per_vd = compute_vsync_step_se(ctx);
        uint32_t tgt_u16 = (uint32_t)clamp_s32(servo->target_se, 0, MAX_U16_CLAMP);
        uint32_t stp_u16 = (uint32_t)clamp_s32(step_per_vd, 1, MAX_U16_CLAMP);
        uint8_t pay[4] = {
            (uint8_t)(tgt_u16 & BYTE_MASK),
            (uint8_t)((tgt_u16 >> SHIFT_8) & BYTE_MASK),
            (uint8_t)(stp_u16 & BYTE_MASK),
            (uint8_t)((stp_u16 >> SHIFT_8) & BYTE_MASK),
        };
        (void)lens_session_enqueue_tx(ctx, 0xF1U, pay, (uint16_t)sizeof(pay));
    }
}

static void reset_arm_drive_counters(adapter_ctx_t *ctx, bool same_active_target,
                                     bool drive_was_active, bool dir_reversed) {
    af_servo_state_t *servo = &ctx->af;
    bool mode_changed = (bool)(servo->is_continuous_track != servo->prev_continuous);
    servo->prev_continuous = servo->is_continuous_track;
    if (dir_reversed || (!drive_was_active) || mode_changed) {
        servo->step_accum_se = 0;
        servo->drive_vsyncs = 0U;
        servo->track_primed = 0U;
    }
    if (servo->motion_dir_cz < 0) {
        servo->cushioned_near = 0U;
    } else if (servo->motion_dir_cz > 0) {
        servo->cushioned_far = 0U;
    }
    bool reset_stall = (bool)((!drive_was_active) || dir_reversed || (ctx->af_drive_moved != 0U));
    if ((!same_active_target) && reset_stall) {
        ctx->af_drive_start_se = ctx->id06.focus_pos_se;
        ctx->af_drive_moved = 0U;
        servo->stable_idle_count = 0U;
        servo->stall_vsyncs = 0U;
        servo->af_cmd_ticks = 0ULL;
    } else if (!same_active_target) {
        servo->stable_idle_count = 0U;
    }
}

static uint8_t select_arm_submode(const adapter_ctx_t *ctx, int32_t abs_err,
                                  bool dir_reversed, bool unlim) {
    const af_servo_state_t *servo = &ctx->af;
    bool fam2_ext = (bool)((ctx->lens_family == 2U) && (ctx->brand != LENS_BRAND_SIGMA));
    bool burst_direct_rev = (bool)(fam2_ext &&
                                   ((ctx->af_ext.last_z_af_cmd == 0xE4U) ||
                                    (ctx->af_ext.last_z_af_cmd == 0xEEU)));
    int32_t scan_thresh = compute_vsync_step_se(ctx);
    if ((ctx->id06.hw_moving != 0U) && dir_reversed &&
        (!(fam2_ext && can_single_shot_direct_abs(ctx, fam2_ext))) &&
        (!burst_direct_rev)) {
        return 2U;
    }
    if (can_lens_use_3c_scan(ctx) && (servo->is_continuous_track != 0U) &&
        (!unlim) && (abs_err > scan_thresh)) {
        return 3U;
    }
    return 1U;
}

static void arm_target_drive(adapter_ctx_t *ctx, int32_t req_cz, bool unlim, float vpps) {
    af_servo_state_t *servo = &ctx->af;
    servo->arrival_confirmed = 0U;
    servo->stalled_abort = 0U;
    servo->e0_brake_rem = 0U;
    bool drive_was_active = ((uint32_t)servo->af_drive_active |
                             (uint32_t)servo->af_cmd_pending | (uint32_t)servo->af_stop_req) != 0UL;
    int32_t prev_target_se = servo->target_se;
    servo->target_cz = clamp_s32(req_cz, servo->mech_lz, servo->mech_hz);
    servo->target_se = af_servo_cz_to_se(ctx, servo->target_cz);
    bool same_active_target = (bool)((servo->af_drive_active != 0U) &&
                                     (servo->target_se == prev_target_se));
    servo->unlimited_speed = unlim ? 1U : 0U;
    servo->cmd_vel_cz_pps = vpps;
    servo->cmd_vel_se_pps = vpps / servo->cur_jacobian;
    int32_t err_se = servo->target_se - ctx->id06.focus_pos_se;
    int32_t abs_err = (err_se >= 0) ? err_se : -err_se;
    int8_t new_dir_se = (err_se >= 0) ? 1 : -1;
    bool dir_reversed = (bool)((servo->prev_drive_dir_se != 0) &&
                               (new_dir_se != servo->prev_drive_dir_se));
    servo->motion_dir_cz = ((servo->dir_sign * err_se) >= 0) ? 1 : -1;
    reset_arm_drive_counters(ctx, same_active_target, drive_was_active, dir_reversed);
    servo->af_stop_req = 0U;
    ctx->parfocal_hold_vsyncs = 0U;
    ctx->parfocal_e0_coast_rem = 0U;
    ctx->af_ext.parfocal_trim_pending = 0U;
    ctx->parfocal_cmd_valid = 0U;
    servo->mf_hold_frames_rem = 0U;
    servo->mf_ring_delta_bf = 0;
    ctx->id06.mf_ring_byte = 0U;
    servo->s1_hold_vsyncs = active_s1_hold_vsyncs(ctx);

    if (abs_err <= capture_tolerance_se(ctx)) {
        latch_endstop_on_settle(ctx);
        servo->cur_cz = snap_endstop_cz(ctx, servo->cur_cz);
        servo->arrival_confirmed = (ctx->id06.valid != 0U) ? 1U : 0U;
        if (drive_was_active || (ctx->id06.hw_moving != 0U)) {
            issue_controlled_brake(ctx);
            return;
        }
        servo->af_drive_active = 0U;
        servo->af_cmd_pending = 0U;
        servo->af_submode = 0U;
        servo->prev_drive_dir_se = 0;
        servo->step_accum_se = 0;
        servo->settle_frames_rem = SETTLE_HOLD_FRAMES;
        sync_parfocal_anchor(ctx, servo->cur_cz);
        return;
    }

    servo->af_drive_active = 1U;
    servo->af_cmd_pending = 1U;
    servo->af_submode = select_arm_submode(ctx, abs_err, dir_reversed, unlim);
    af_servo_emit_vsync_drive(ctx);
}

static int32_t relative_focus_target(const adapter_ctx_t *ctx, int32_t delta_cz) {
    const af_servo_state_t *servo = &ctx->af;
    int32_t anchor_cz = (ctx->af_ext.hotline_seen != 0U) ? ctx->af_ext.last_hot_cz : servo->cur_cz;
    int64_t target = (int64_t)anchor_cz + (int64_t)delta_cz;
    if (target < (int64_t)servo->mech_lz) {
        return servo->mech_lz;
    }
    if (target > (int64_t)servo->mech_hz) {
        return servo->mech_hz;
    }
    return (int32_t)target;
}

static int32_t resolve_sentinel_target(const af_servo_state_t *servo, int32_t raw_cz, bool neg_dir) {
    if ((uint32_t)raw_cz == Z_SENTINEL_WALL_CZ) {
        return neg_dir ? servo->mech_lz : servo->mech_hz;
    }
    return raw_cz;
}

static void issue_controlled_brake(adapter_ctx_t *ctx) {
    af_servo_state_t *servo = &ctx->af;
    bool af_was_driving = (bool)(((uint32_t)servo->af_drive_active |
                                  (uint32_t)servo->af_cmd_pending) != 0UL);
    bool pf_busy = (bool)((ctx->parfocal_hold_vsyncs > 0U) ||
                          (ctx->af_ext.parfocal_trim_pending != 0U));
    if ((!af_was_driving) && pf_busy) {
        servo->settle_frames_rem = SETTLE_HOLD_FRAMES;
        servo->motion_dir_cz = 0;
        return;
    }
    if ((!af_was_driving) && (ctx->parfocal_e0_coast_rem > 0U) &&
        (ctx->id06.hw_moving != 0U)) {
        servo->settle_frames_rem = SETTLE_HOLD_FRAMES;
        return;
    }
    bool was_moving = (bool)(af_was_driving || (ctx->id06.hw_moving != 0U));
    bool coasting = (bool)(af_was_driving && (ctx->id06.hw_moving != 0U));
    ctx->af_ext.active_step_se = 0;
    servo->af_drive_active = 0U;
    servo->af_cmd_pending = 0U;
    servo->af_submode = 0U;
    servo->prev_drive_dir_se = 0;
    servo->step_accum_se = 0;
    servo->track_primed = 0U;
    servo->af_stop_req = 1U;
    servo->e0_brake_rem = coasting ? E0_BRAKE_MAX_VSYNCS : 0U;
    ctx->parfocal_e0_coast_rem = coasting ? PARFOCAL_E0_COAST_MAX : 0U;
    servo->s1_hold_vsyncs = coasting ? active_s1_hold_vsyncs(ctx) : 0U;
    servo->settle_frames_rem = was_moving ? SETTLE_HOLD_FRAMES : 0U;
    if (servo->e0_brake_rem == 0U) {
        servo->motion_dir_cz = 0;
    }
    if (coasting) {
        ctx->parfocal_hold_vsyncs = 0U;
        ctx->parfocal_cmd_valid = 0U;
        ctx->af_ext.parfocal_trim_pending = 0U;
        ctx->af_ext.parfocal_anchor_valid = 0U;
    } else if (af_was_driving || (ctx->af_ext.parfocal_anchor_valid == 0U)) {
        ctx->parfocal_hold_vsyncs = 0U;
        sync_parfocal_anchor(ctx, servo->cur_cz);
    }
}

static void note_af_z_cmd(adapter_ctx_t *ctx, uint8_t cmd) {
    ctx->af_ext.last_z_af_cmd = cmd;
    if (ctx->diag_af_cmd_count < 0xFFFFU) {
        ctx->diag_af_cmd_count = (uint16_t)(ctx->diag_af_cmd_count + 1U);
    }
}

static void handle_z_cmd_c0(adapter_ctx_t *ctx, const uint8_t *mosi) {
    af_servo_state_t *servo = &ctx->af;
    if (((uint32_t)mosi[11] & 0x06UL) != 0UL) {
        servo->stalled_abort = 0U;
        issue_controlled_brake(ctx);
        return;
    }
    servo->latest_cookie = mosi[12];
    servo->is_continuous_track = 0U;
    bool unlim = true;
    int8_t sdir = 0;
    float vpps = af_servo_decode_z_speed_pps(mosi[5], &unlim, mosi[6], &sdir);
    int32_t raw_1 = read_s32_le(&mosi[1]);
    int32_t raw_13 = read_s32_le(&mosi[13]);
    int32_t req_cz = resolve_sentinel_target(servo, raw_13, (bool)(sdir < 0));
    if (raw_1 != 0) {
        int32_t anchor_cz = (ctx->af_ext.hotline_seen != 0U) ? ctx->af_ext.last_hot_cz : servo->cur_cz;
        req_cz = relative_focus_target(ctx, raw_1);
        if (((uint32_t)raw_13 != Z_SENTINEL_WALL_CZ) && (raw_13 != 0)) {
            bool clamp_pos = (bool)((raw_1 > 0) && (raw_13 >= anchor_cz) && (req_cz > raw_13));
            bool clamp_neg = (bool)((raw_1 < 0) && (raw_13 <= anchor_cz) && (req_cz < raw_13));
            if (clamp_pos || clamp_neg) {
                req_cz = raw_13;
            }
        }
    }
    arm_target_drive(ctx, req_cz, unlim, vpps);
    if ((servo->af_drive_active != 0U) && (!unlim) && (sdir != 0) && (raw_1 == 0)) {
        servo->motion_dir_cz = sdir;
    }
}

static bool handle_z_af_c_cmds(adapter_ctx_t *ctx, uint8_t cmd,
                               const uint8_t *mosi, size_t mosi_len) {
    af_servo_state_t *servo = &ctx->af;
    if (cmd == 0xC2U) {
        if (mosi_len >= 10U) {
            note_af_z_cmd(ctx, cmd);
            servo->linear_mf_code = mosi[9];
            if ((ctx->session_state == E_SESSION_ACTIVE) && lens_session_cap_has(ctx, 0x22U)) {
                uint8_t pay22[2] = {0x01U, servo->linear_mf_code};
                (void)lens_session_enqueue_tx(ctx, 0x22U, pay22, (uint16_t)sizeof(pay22));
            }
        }
        return true;
    }
    if ((cmd == 0xC3U) || (cmd == 0xC4U)) {
        return true;
    }
    if ((cmd == 0xC0U) && (mosi_len >= 18U)) {
        note_af_z_cmd(ctx, cmd);
        handle_z_cmd_c0(ctx, mosi);
        return true;
    }
    if ((cmd == 0xC1U) && (mosi_len >= 24U)) {
        note_af_z_cmd(ctx, cmd);
        servo->latest_cookie = mosi[21];
        servo->is_continuous_track = 0U;
        bool unlim = true;
        float vpps = 0.0F;
        if ((mosi[1] != 0U) || (mosi[2] != 0U)) {
            int8_t sdir = 0;
            vpps = af_servo_decode_z_speed_pps(mosi[1], &unlim, mosi[2], &sdir);
        }
        arm_target_drive(ctx, read_s32_le(&mosi[3]), unlim, vpps);
        return true;
    }
    return false;
}

static int32_t resolve_native_e4_wall(const af_servo_state_t *servo,
                                      int32_t wall_cz, const int8_t *sdir_ptr) {
    int8_t sdir = *sdir_ptr;
    if ((uint32_t)wall_cz == Z_SENTINEL_WALL_CZ) {
        return (sdir < 0) ? servo->mech_lz : servo->mech_hz;
    }
    if (sdir > 0) {
        if (wall_cz == 0) {
            return servo->mech_hz;
        }
        return (servo->cur_cz >= wall_cz) ? servo->cur_cz : wall_cz;
    }
    if (sdir < 0) {
        if (wall_cz == 0) {
            return servo->mech_lz;
        }
        return (servo->cur_cz <= wall_cz) ? servo->cur_cz : wall_cz;
    }
    return wall_cz;
}

static float decode_e4_speed(const uint8_t *mosi, int32_t delta_cz,
                             bool *out_unlim, int8_t *out_sdir) {
    bool unused_spd1 = (bool)(((mosi[7] == 0U) && (mosi[8] == 0U)) ||
                              ((mosi[7] == Z_UNLIMITED_SPD_LO) &&
                               (((uint32_t)mosi[8] & Z_UNUSED_SPD_HI_MASK) == Z_UNUSED_SPD_HI_TAG)));
    bool use_spd1 = (bool)((mosi[9] == 0U) && (!unused_spd1));
    uint8_t spd_lo = use_spd1 ? mosi[7] : mosi[5];
    uint8_t spd_hi = use_spd1 ? mosi[8] : mosi[6];
    float vpps = af_servo_decode_z_speed_pps(spd_lo, out_unlim, spd_hi, out_sdir);
    if ((delta_cz == 0) && (!use_spd1) && (!unused_spd1) &&
        ((mosi[5] != 0U) || (mosi[6] != 0U))) {
        bool unlim1 = false;
        int8_t sdir1 = 0;
        float vpps1 = af_servo_decode_z_speed_pps(mosi[7], &unlim1, mosi[8], &sdir1);
        if ((!unlim1) && (vpps1 > 0.0F) && ((*out_unlim) || (vpps1 < vpps))) {
            *out_unlim = false;
            vpps = vpps1;
        }
        if (*out_sdir == 0) {
            *out_sdir = sdir1;
        }
    }
    return vpps;
}

static void handle_z_cmd_e4(adapter_ctx_t *ctx, const uint8_t *mosi) {
    af_servo_state_t *servo = &ctx->af;
    if (((uint32_t)mosi[9] & 0x06UL) != 0UL) {
        servo->stalled_abort = 0U;
        issue_controlled_brake(ctx);
        return;
    }
    servo->latest_cookie = mosi[10];
    servo->is_continuous_track = 1U;
    int32_t delta_cz = read_s32_le(&mosi[1]);
    bool unlim = false;
    int8_t sdir = 0;
    float vpps = decode_e4_speed(mosi, delta_cz, &unlim, &sdir);
    int32_t target2_cz = read_s32_le(&mosi[11]);
    int32_t req_cz = resolve_sentinel_target(servo, target2_cz, (bool)(sdir < 0));
    if (delta_cz != 0) {
        req_cz = relative_focus_target(ctx, delta_cz);
        if (((uint32_t)target2_cz != Z_SENTINEL_WALL_CZ) && (target2_cz != 0)) {
            int32_t wall_abs = (mosi[9] != 0U)
                                   ? resolve_native_e4_wall(servo, target2_cz, &sdir)
                                   : relative_focus_target(ctx, target2_cz);
            bool clamp_pos = (bool)((delta_cz > 0) && (req_cz > wall_abs));
            bool clamp_neg = (bool)((delta_cz < 0) && (req_cz < wall_abs));
            if (clamp_pos || clamp_neg) {
                req_cz = wall_abs;
            }
        }
    } else if (mosi[9] != 0U) {
        req_cz = resolve_native_e4_wall(servo, target2_cz, &sdir);
    }
    arm_target_drive(ctx, req_cz, unlim, vpps);
    if ((servo->af_drive_active != 0U) && (sdir != 0) && (delta_cz == 0)) {
        servo->motion_dir_cz = sdir;
    }
}

static void handle_z_cmd_ee(adapter_ctx_t *ctx, const uint8_t *mosi) {
    af_servo_state_t *servo = &ctx->af;
    if (((uint32_t)mosi[25] & 0x06UL) != 0UL) {
        servo->stalled_abort = 0U;
        servo->latest_cookie = mosi[26];
        issue_controlled_brake(ctx);
        return;
    }
    servo->latest_cookie = mosi[26];
    servo->is_continuous_track = 1U;
    bool unlim = false;
    int8_t sdir = 1;
    float vpps = af_servo_decode_z_speed_pps(mosi[5], &unlim, mosi[6], &sdir);
    int32_t raw_t1 = read_s32_le(&mosi[1]);
    int32_t req_cz = resolve_sentinel_target(servo, raw_t1, (bool)(sdir < 0));
    int32_t wall_raw = read_s32_le(&mosi[20]);
    if (((uint32_t)raw_t1 != Z_SENTINEL_WALL_CZ) &&
        ((uint32_t)wall_raw != Z_SENTINEL_WALL_CZ) && (wall_raw != 0)) {
        int32_t step_cz = req_cz - servo->cur_cz;
        int32_t wall_cz = clamp_s32(wall_raw, servo->mech_lz, servo->mech_hz);
        bool clamp_pos = (bool)((step_cz > 0) && (wall_cz >= servo->cur_cz) && (req_cz > wall_cz));
        bool clamp_neg = (bool)((step_cz < 0) && (wall_cz <= servo->cur_cz) && (req_cz < wall_cz));
        if (clamp_pos || clamp_neg) {
            req_cz = wall_cz;
        }
    }
    arm_target_drive(ctx, req_cz, unlim, vpps);
    if ((servo->af_drive_active != 0U) && (sdir != 0)) {
        servo->motion_dir_cz = sdir;
    }
}

static void handle_z_af_e_cmds(adapter_ctx_t *ctx, uint8_t cmd,
                               const uint8_t *mosi, size_t mosi_len) {
    af_servo_state_t *servo = &ctx->af;
    if ((cmd == 0xEBU) || (cmd == 0xEDU) || (cmd == 0xEFU)) {
        return;
    }
    if ((cmd == 0xE4U) && (mosi_len >= 16U)) {
        note_af_z_cmd(ctx, cmd);
        handle_z_cmd_e4(ctx, mosi);
        return;
    }
    if (((cmd == 0xE2U) || (cmd == 0xE6U)) && (mosi_len >= 10U)) {
        note_af_z_cmd(ctx, cmd);
        if (((uint32_t)mosi[7] & 0x06UL) == 0x06UL) {
            servo->stalled_abort = 0U;
            issue_controlled_brake(ctx);
            return;
        }
        servo->latest_cookie = mosi[8];
        bool unlim = false;
        int8_t sdir = 1;
        float vpps = af_servo_decode_z_speed_pps(mosi[5], &unlim, mosi[6], &sdir);
        bool fast_direct = (bool)(unlim || (is_fast_sony_xd(ctx) && (vpps >= FAST_JUMP_CZ_VPPS)));
        servo->is_continuous_track = fast_direct ? 0U : 1U;
        int32_t raw_arg = read_s32_le(&mosi[1]);
        int32_t req_cz = (cmd == 0xE2U)
                             ? relative_focus_target(ctx, raw_arg)
                             : resolve_sentinel_target(servo, raw_arg, (bool)(sdir < 0));
        arm_target_drive(ctx, req_cz, unlim, vpps);
        return;
    }
    if ((cmd == 0xE8U) && (mosi_len >= 10U)) {
        note_af_z_cmd(ctx, cmd);
        servo->latest_cookie = mosi[4];
        servo->is_continuous_track = 1U;
        bool unlim = false;
        int8_t sdir = 1;
        float vpps = af_servo_decode_z_speed_pps(mosi[1], &unlim, mosi[2], &sdir);
        int32_t req_cz = resolve_sentinel_target(servo, read_s32_le(&mosi[5]), (bool)(sdir < 0));
        arm_target_drive(ctx, req_cz, unlim, vpps);
        if (servo->af_drive_active != 0U) {
            servo->motion_dir_cz = sdir;
        }
        return;
    }
    if ((cmd == 0xEEU) && (mosi_len >= 28U)) {
        note_af_z_cmd(ctx, cmd);
        handle_z_cmd_ee(ctx, mosi);
        return;
    }
    if ((cmd == 0xECU) && (mosi_len >= 12U)) {
        note_af_z_cmd(ctx, cmd);
        servo->latest_cookie = mosi[10];
        servo->is_continuous_track = 0U;
        arm_target_drive(ctx, read_s32_le(&mosi[1]), true, 0.0F);
        return;
    }
    servo->latest_cookie = mosi[1];
}

void af_servo_on_z_command(adapter_ctx_t *ctx, uint8_t cmd,
                           const uint8_t *mosi, size_t mosi_len) {
    af_servo_state_t *servo = &ctx->af;
    if (mosi_len < 4U) {
        return;
    }
    if ((cmd == 0xE0U) || (cmd == 0xE1U)) {
        note_af_z_cmd(ctx, cmd);
        servo->arrival_confirmed = (ctx->id06.valid != 0U) ? 1U : 0U;
        servo->stalled_abort = 0U;
        servo->latest_cookie = mosi[1];
        issue_controlled_brake(ctx);
        return;
    }
    if (handle_z_af_c_cmds(ctx, cmd, mosi, mosi_len)) {
        if ((ctx->session_state == E_SESSION_EMPTY_MOUNT) ||
            (ctx->session_state == E_SESSION_PASSIVE_MOUNT)) {
            servo->af_drive_active = 0U;
            servo->af_cmd_pending = 0U;
            servo->af_submode = 0U;
            servo->prev_drive_dir_se = 0;
            servo->settle_frames_rem = SETTLE_HOLD_FRAMES;
            servo->motion_dir_cz = 0;
        }
        return;
    }
    handle_z_af_e_cmds(ctx, cmd, mosi, mosi_len);
    if ((ctx->session_state == E_SESSION_EMPTY_MOUNT) ||
        (ctx->session_state == E_SESSION_PASSIVE_MOUNT)) {
        servo->af_drive_active = 0U;
        servo->af_cmd_pending = 0U;
        servo->af_submode = 0U;
        servo->prev_drive_dir_se = 0;
        servo->settle_frames_rem = SETTLE_HOLD_FRAMES;
        servo->motion_dir_cz = 0;
    }
}
/* NOLINTEND(readability-magic-numbers) */

static uint8_t encode_hotline_mf_ring_byte(const adapter_ctx_t *ctx) {
    const af_servo_state_t *servo = &ctx->af;
    if (servo->mf_hold_frames_rem == 0U) {
        return 0U;
    }
    if (ctx->id06.mf_ring_byte != 0U) {
        return ctx->id06.mf_ring_byte;
    }
    if (servo->mf_ring_delta_bf > 0) {
        return 2U;
    }
    if (servo->mf_ring_delta_bf < 0) {
        return 1U;
    }
    return 0U;
}

static int32_t interpolate_hotline_cz(const adapter_ctx_t *ctx, uint64_t now_ticks) {
    static const uint32_t MIN_SAMPLE_DT_U32 = 18000UL;
    static const uint32_t MAX_SAMPLE_DT_U32 = 300000UL;
    const af_servo_state_t *servo = &ctx->af;
    if ((ctx->id06.valid == 0U) || (ctx->id06.t_sample_ticks <= ctx->id06.t_prev_ticks) ||
        (ctx->id06.t_prev_ticks == 0ULL) || (ctx->id06.hw_moving == 0U)) {
        return snap_endstop_cz(ctx, servo->cur_cz);
    }
    uint64_t dt_sample64 = ctx->id06.t_sample_ticks - ctx->id06.t_prev_ticks;
    if ((dt_sample64 < (uint64_t)MIN_SAMPLE_DT_U32) ||
        (dt_sample64 > (uint64_t)MAX_SAMPLE_DT_U32)) {
        return servo->cur_cz;
    }
    uint32_t dt_sample_u32 = (uint32_t)dt_sample64;
    uint64_t dt_now64 = (now_ticks >= ctx->id06.t_sample_ticks)
                            ? (now_ticks - ctx->id06.t_sample_ticks)
                            : 0ULL;
    uint32_t max_extrap_u32 = (uint32_t)EXTRAP_MAX_TICKS;
    uint32_t dt_now_u32 = (dt_now64 > (uint64_t)max_extrap_u32) ? max_extrap_u32 : (uint32_t)dt_now64;
    if (dt_now_u32 > dt_sample_u32) {
        dt_now_u32 = dt_sample_u32;
    }
    if ((servo->af_stop_req != 0U) || (servo->e0_brake_rem > 0U)) {
        dt_now_u32 >>= 1U;
    }
    float v_se = (float)(ctx->id06.focus_pos_se - ctx->id06.focus_prev_se) / (float)dt_sample_u32;
    int32_t d_pred_se = round_f32_to_s32(v_se * (float)dt_now_u32);
    if (ctx->af_ext.active_step_se > 0) {
        d_pred_se = clamp_s32(d_pred_se, -ctx->af_ext.active_step_se, ctx->af_ext.active_step_se);
    }
    int32_t pred_se = ctx->id06.focus_pos_se + d_pred_se;
    if (servo->af_drive_active != 0U) {
        bool crossed_pos = (bool)((ctx->id06.focus_pos_se <= servo->target_se) &&
                                  (pred_se > servo->target_se));
        bool crossed_neg = (bool)((ctx->id06.focus_pos_se >= servo->target_se) &&
                                  (pred_se < servo->target_se));
        if (crossed_pos || crossed_neg) {
            pred_se = servo->target_se;
        }
    }
    return af_servo_se_to_cz(ctx, pred_se);
}

static void check_hotline_stall_timeout(adapter_ctx_t *ctx, uint64_t now_ticks) {
    af_servo_state_t *servo = &ctx->af;
    if (servo->af_drive_active == 0U) {
        return;
    }
    if (servo->af_cmd_ticks == 0ULL) {
        servo->af_cmd_ticks = now_ticks;
        return;
    }
    if ((now_ticks > servo->af_cmd_ticks) &&
        ((now_ticks - servo->af_cmd_ticks) > AF_STALL_TIMEOUT_TICKS) &&
        (ctx->id06.hw_moving == 0U)) {
        servo->af_stop_req = 1U;
        servo->af_drive_active = 0U;
        servo->af_cmd_pending = 0U;
        servo->af_submode = 0U;
        servo->prev_drive_dir_se = 0;
        servo->step_accum_se = 0;
        servo->stalled_abort = (servo->arrival_confirmed == 0U) ? STALL_ABORT_HOT_FRAMES : 0U;
        servo->settle_frames_rem = SETTLE_HOLD_FRAMES;
    }
}

static uint8_t advance_hotline_hot6(af_servo_state_t *servo, bool still_braking) {
    uint8_t hot6 = HOT6_IDLE_SETTLED;
    if ((servo->af_drive_active != 0U) || still_braking) {
        hot6 = (servo->motion_dir_cz >= 0) ? HOT6_POS_MOVING : HOT6_NEG_MOVING;
    } else if (servo->mf_hold_frames_rem > 0U) {
        hot6 = (uint8_t)((uint32_t)HOT6_MF_RING_TURN & ~(uint32_t)HOT6_DRIVE_RESULT_MASK);
    } else if (servo->stalled_abort > 0U) {
        hot6 = HOT6_STALL_ABORT;
        servo->stalled_abort = (uint8_t)(servo->stalled_abort - 1U);
        if (servo->settle_frames_rem > 0U) {
            servo->settle_frames_rem = (uint8_t)(servo->settle_frames_rem - 1U);
        }
        if (servo->stalled_abort == 0U) {
            servo->motion_dir_cz = 0;
        }
    } else if (servo->settle_frames_rem > 0U) {
        hot6 = HOT6_E0_SETTLED;
        servo->settle_frames_rem = (uint8_t)(servo->settle_frames_rem - 1U);
        if (servo->settle_frames_rem == 0U) {
            servo->motion_dir_cz = 0;
        }
    }
    if ((servo->af_drive_active == 0U) && (!still_braking) && (hot6 != HOT6_STALL_ABORT)) {
        hot6 = (uint8_t)((uint32_t)hot6 & ~(uint32_t)HOT6_DRIVE_RESULT_MASK);
    }
    return hot6;
}

void af_servo_build_hotline_frame(adapter_ctx_t *ctx, uint8_t seq,
                                  uint8_t *out_hot42, uint64_t now_ticks) {
    static const size_t HOT_OFF_SEQ     = 1U;
    static const size_t HOT_OFF_CZ      = 2U;
    static const size_t HOT_OFF_STATE   = 6U;
    static const size_t HOT_OFF_LIMITS  = 7U;
    static const size_t HOT_OFF_COOKIE  = 8U;
    static const size_t HOT_OFF_MF_RING = 21U;

    af_servo_state_t *servo = &ctx->af;
    ctx->vr.hotline_seq = seq;
    memset(out_hot42, 0, Z_HOTLINE_BYTES);
    out_hot42[HOT_OFF_SEQ] = seq;

    int32_t cz_interp = interpolate_hotline_cz(ctx, now_ticks);
    ctx->af_ext.last_hot_cz = cz_interp;
    ctx->af_ext.hotline_seen = 1U;
    write_s32_le(&out_hot42[HOT_OFF_CZ], cz_interp);

    uint8_t mf_bit = (((uint32_t)ctx->z_mosi20_retained[2] & MOSI20_MF_FLAGS_MASK) != 0UL)
                         ? HOT6_MF_STATUS_BIT
                         : 0x00U;
    check_hotline_stall_timeout(ctx, now_ticks);
    if ((ctx->id06.hw_moving == 0U) && (servo->af_stop_req == 0U)) {
        servo->e0_brake_rem = 0U;
    }

    bool still_braking = (bool)((servo->af_stop_req != 0U) ||
                                ((servo->e0_brake_rem > 0U) && (ctx->id06.hw_moving != 0U)));
    uint8_t hot6 = advance_hotline_hot6(servo, still_braking);
    out_hot42[HOT_OFF_STATE] = (uint8_t)(mf_bit | hot6);
    out_hot42[HOT_OFF_MF_RING] = encode_hotline_mf_ring_byte(ctx);

    uint32_t hot7 = 0x00UL;
    if ((cz_interp >= servo->mech_hz) || (ctx->id06.at_near_end != 0U) ||
        (servo->cushioned_near != 0U)) {
        hot7 |= HOT7_NEAR_LIMIT_BIT;
    }
    if ((cz_interp <= servo->mech_lz) || (ctx->id06.at_far_end != 0U) ||
        (servo->cushioned_far != 0U)) {
        hot7 |= HOT7_FAR_LIMIT_BIT;
    }
    out_hot42[HOT_OFF_LIMITS] = (uint8_t)hot7;
    out_hot42[HOT_OFF_COOKIE] = servo->latest_cookie;

    boryoza_seal_z_frame(out_hot42, Z_HOTLINE_BYTES);
}
