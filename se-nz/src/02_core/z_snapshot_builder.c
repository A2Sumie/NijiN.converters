#include "../../include/boryoza_types.h"
#include "../../include/boryoza_math_mem.h"

static const uint16_t EXIT_PUPIL_WIDE_DEF    = 0x01C0U;
static const uint16_t EXIT_PUPIL_TELE_DEF    = 0x011BU;
static const uint16_t PROVISIONAL_Z_LENS_ID  = 0x00A0U;
static const uint16_t MF_BUCKET_MAX          = 0x00FFU;
static const uint16_t MF_RANGE_STD_HI        = 0x0024U;
static const uint16_t MF_LIMITER_LO          = 0x0038U;
static const uint16_t MF_LIMITER_HI          = 0x00C6U;
static const uint16_t MF_LIMITER_SPAN        = 0x00AAU;
static const uint16_t KZ_NONLIN_DISABLED     = 0x0100U;
static const uint16_t ABERRATION_NEUTRAL_Z   = 0x0000U;
static const uint16_t VIG_DIST_DISABLED      = 0x7FFFU;
static const uint16_t VIG_AV_DISABLED        = 0x0C01U;
static const uint16_t VIG_P2_DIST_FAST_TELE  = 0x00A0U;
static const uint16_t VIG_P2_AV_FAST_TELE    = 0x02ECU;
static const uint16_t VIG_P3_DIST_EMOUNT     = 0x025AU;
static const uint16_t VIG_P3_AV_EMOUNT       = 0x02DCU;
static const uint16_t VIG_WIN_H1_FF          = 0x00A0U;
static const uint16_t VIG_WIN_H2_FF          = 0x01B3U;
static const uint16_t VIG_WIN_H3_FF          = 0xFFBFU;
static const uint16_t VIG_WIN_H4_FF          = 0xFEEBU;
static const uint32_t SONY_BRAND_ID          = 1UL;
static const uint16_t FAST_TELE_MIN_FOCAL_MM = 85U;
static const uint16_t FAST_TELE_MAX_OPEN_AV  = 0x0380U;
static const uint32_t MAX_VPPS_STILLS_DEF     = 0x00010000UL;
static const uint32_t MAX_VPPS_STILLS_SONY_XD = 0x00040000UL;
static const uint32_t MAX_VPPS_MOVIE_DEF      = 0x00004000UL;
static const uint16_t CDAF_WOBBLE_SPAN_DEF   = 0x0320U;
static const uint8_t  CDAF_TURNAROUND_DEF    = 0x0AU;
static const uint16_t SYNCHRO_INVALID_8000   = 0x8000U;
static const size_t   Z_MAX_SMALL_BYTES    = 512U;
static const uint32_t BYTE_MASK            = 0xFFUL;
static const uint32_t SHIFT_4              = 4U;
static const uint32_t SHIFT_8              = 8U;
static const uint32_t SHIFT_16             = 16U;
static const uint32_t SHIFT_24             = 24U;

static void put_u16_le(uint8_t *dst, uint16_t val) {
    uint32_t uval = val;
    dst[0] = (uint8_t)(uval & BYTE_MASK);
    dst[1] = (uint8_t)((uval >> SHIFT_8) & BYTE_MASK);
}

static void put_u32_le(uint8_t *dst, uint32_t val) {
    dst[0] = (uint8_t)(val & BYTE_MASK);
    dst[1] = (uint8_t)((val >> SHIFT_8) & BYTE_MASK);
    dst[2] = (uint8_t)((val >> SHIFT_16) & BYTE_MASK);
    dst[3] = (uint8_t)((val >> SHIFT_24) & BYTE_MASK);
}

static void put_s32_le(uint8_t *dst, int32_t val) {
    uint32_t uval = (uint32_t)val;
    dst[0] = (uint8_t)(uval & BYTE_MASK);
    dst[1] = (uint8_t)((uval >> SHIFT_8) & BYTE_MASK);
    dst[2] = (uint8_t)((uval >> SHIFT_16) & BYTE_MASK);
    dst[3] = (uint8_t)((uval >> SHIFT_24) & BYTE_MASK);
}

/* NOLINTBEGIN(readability-magic-numbers) — Z-mount snapshot wire-byte offsets */
static void build_cmd10_28b(const adapter_ctx_t *ctx, uint8_t *out28) {
    memset(out28, 0, Z_CMD10_BYTES);
    uint16_t wire_z_lid = ctx->z_primary_lens_id;
    if ((ctx->identity_ready == 0U) &&
        (ctx->session_state != E_SESSION_EMPTY_MOUNT) &&
        (ctx->session_state != E_SESSION_PASSIVE_MOUNT)) {
        wire_z_lid = PROVISIONAL_Z_LENS_ID;
    }
    put_u16_le(&out28[0x01], wire_z_lid);

    /* Populate protocol version and focus distance metadata flag */
    out28[0x03] = 0x00U;
    out28[0x04] = 0x00U;
    out28[0x05] = (uint8_t)BORYOZA_LF_BCD_MINOR;
    out28[0x06] = (uint8_t)BORYOZA_LF_BCD_MAJOR;

    out28[0x07] = (ctx->dist_report_invalid != 0U) ? 0x00U : 0x01U;
    out28[0x08] = 0x00U;

    uint32_t fmt_flags = 0x00UL;
    if (ctx->is_apsc != 0U) {
        fmt_flags |= 0x10UL;
    }
    if (ctx->is_macro != 0U) {
        fmt_flags |= 0x04UL;
    }
    out28[0x09] = (uint8_t)fmt_flags;
    out28[0x0A] = 0x00U;
    bool is_manual_only = false;
    if ((ctx->session_state == E_SESSION_EMPTY_MOUNT) ||
        (ctx->session_state == E_SESSION_PASSIVE_MOUNT)) {
        is_manual_only = true;
    }
    out28[0x0B] = is_manual_only ? 0x00U : (uint8_t)((uint32_t)ctx->af.scale_exp_n << SHIFT_4);

    put_u16_le(&out28[0x0E], ctx->e_lens_id);
    put_u16_le(&out28[0x10], is_manual_only ? 0x0000U : ctx->e_lens_ver);

    /* Capability class bytes selecting symmetric A1/00 aperture framing */
    out28[0x13] = 0x25U;
    out28[0x14] = 0x2AU;
    out28[0x15] = 0x1AU;
    out28[0x16] = 0x1AU;

    boryoza_seal_z_frame(out28, Z_CMD10_BYTES);
}

static void build_cmd11_74b(const adapter_ctx_t *ctx, uint8_t *out74) {
    memset(out74, 0, Z_CMD11_BYTES);
    memcpy(&out74[0x01], ctx->lens_name_ascii, LENS_NAME_ASCII_CAP);
    out74[0x40] = '\0';
    memcpy(&out74[0x41], ctx->vendor_tag_5b, VENDOR_TAG_BYTES);
    boryoza_seal_z_frame(out74, Z_CMD11_BYTES);
}

static void build_cmd12_66b(const adapter_ctx_t *ctx, uint8_t *out66) {
    memset(out66, 0, Z_CMD12_BYTES);
    put_u16_le(&out66[0x01], ctx->ap.c_open_wide);
    put_u16_le(&out66[0x03], ctx->ap.c_open_tele);
    put_u16_le(&out66[0x07], ctx->f_cur_mm);
    put_u16_le(&out66[0x0B], ctx->f_cur_mm);
    put_u16_le(&out66[0x0D], EXIT_PUPIL_WIDE_DEF);
    put_u16_le(&out66[0x0F], EXIT_PUPIL_TELE_DEF);

    out66[0x19] = 0x03U;
    out66[0x1A] = 0x01U;
    out66[0x1B] = 0x9CU;
    out66[0x1C] = 0x04U;
    out66[0x1D] = 0x01U;
    out66[0x20] = 0x04U;

    out66[0x27] = 0x03U;
    out66[0x28] = 0xFEU;

    put_u16_le(&out66[0x29], 0x0000U);
    put_u16_le(&out66[0x2B], ctx->f_min_mm);
    put_u16_le(&out66[0x2D], 0x0000U);
    put_u16_le(&out66[0x2F], ctx->f_max_mm);
    put_u16_le(&out66[0x31], (ctx->dist_report_invalid != 0U) ? 0x0000U : MF_BUCKET_MAX);
    put_u16_le(&out66[0x33], 0x0000U);

    out66[0x39] = ctx->vr.z_vr_cap0;
    out66[0x3A] = ctx->vr.z_vr_cap1;

    boryoza_seal_z_frame(out66, Z_CMD12_BYTES);
}

static void build_cmd20_01_24b(const adapter_ctx_t *ctx, uint8_t *out24) {
    memset(out24, 0, Z_CMD20_01_BYTES);
    put_u16_le(&out24[0x01], ctx->ap.c_open_cur);
    put_u16_le(&out24[0x03], ctx->ap.c_cur);
    put_u16_le(&out24[0x07], ctx->f_cur_mm);
    put_u16_le(&out24[0x09], (ctx->dist_report_invalid != 0U) ? 0x0000U : MF_BUCKET_MAX);
    put_u16_le(&out24[0x0B], (ctx->dist_report_invalid != 0U) ? 0x0000U : ctx->ap.z_alt_k_bucket);
    put_u16_le(&out24[0x0D], 0xFFFEU);
    put_u16_le(&out24[0x0F], 0xFFFEU);
    put_u16_le(&out24[0x11], 0xFFFEU);
    boryoza_seal_z_frame(out24, Z_CMD20_01_BYTES);
}

static void write_cmd20_pupil_block(const adapter_ctx_t *ctx, uint8_t *out258) {
    uint16_t ep_code = (ctx->id05.exit_pupil_raw != 0U) ? ctx->id05.exit_pupil_raw : EXIT_PUPIL_WIDE_DEF;
    uint16_t ep_av = (ctx->ap.c_open_cur != 0U) ? ctx->ap.c_open_cur : ctx->ap.c_cur;
    bool fast_sony_tele = (bool)((ctx->brand == (lens_brand_t)SONY_BRAND_ID) &&
                                 (ctx->f_cur_mm >= FAST_TELE_MIN_FOCAL_MM) &&
                                 (ctx->ap.c_open_cur <= FAST_TELE_MAX_OPEN_AV));
    put_u16_le(&out258[0x11], ep_code);
    put_u16_le(&out258[0x13], ep_av);
    put_u16_le(&out258[0x15], fast_sony_tele ? VIG_P2_DIST_FAST_TELE : VIG_DIST_DISABLED);
    put_u16_le(&out258[0x17], fast_sony_tele ? VIG_P2_AV_FAST_TELE : VIG_AV_DISABLED);
    put_u16_le(&out258[0x19], fast_sony_tele ? VIG_P3_DIST_EMOUNT : VIG_DIST_DISABLED);
    put_u16_le(&out258[0x1B], fast_sony_tele ? VIG_P3_AV_EMOUNT : VIG_AV_DISABLED);
    put_u16_le(&out258[0x1D], VIG_WIN_H1_FF);
    put_u16_le(&out258[0x1F], VIG_WIN_H2_FF);
    put_u16_le(&out258[0x21], fast_sony_tele ? 0x0000U : VIG_WIN_H3_FF);
    put_u16_le(&out258[0x23], fast_sony_tele ? 0x0000U : VIG_WIN_H4_FF);
}

static void write_cmd20_kz_block(const adapter_ctx_t *ctx, uint8_t *out258) {
    bool has_c0 = (bool)((ctx->af_ext.kz_c0_b0 != 0U) || (ctx->af_ext.kz_c0_b1 != 0U));
    uint8_t c0_b0 = has_c0 ? ctx->af_ext.kz_c0_b0 : 0x00U;
    uint8_t c0_b1 = has_c0 ? ctx->af_ext.kz_c0_b1 : 0x01U;
    out258[0x63] = c0_b0;
    out258[0x64] = c0_b1;
    put_u16_le(&out258[0x65], KZ_NONLIN_DISABLED);
    out258[0x67] = ctx->af.kz_b0;
    out258[0x68] = ctx->af.kz_b1;
    out258[0x69] = c0_b0;
    out258[0x6A] = c0_b1;
    put_u16_le(&out258[0x6B], KZ_NONLIN_DISABLED);

    out258[0x6D] = ctx->af.kz_b0;
    out258[0x6E] = ctx->af.kz_b1;
    out258[0x6F] = ctx->af.kz_b0;
    out258[0x70] = ctx->af.kz_b1;
    out258[0x71] = ctx->af.kz_b0;
    out258[0x72] = ctx->af.kz_b1;
}

static void write_cmd20_mf_and_limits(const adapter_ctx_t *ctx, uint8_t *out258,
                                      uint16_t f_frac_q16) {
    if (ctx->af.barrel_limiter_on != 0U) {
        put_u16_le(&out258[0x5D], MF_LIMITER_LO);
        put_u16_le(&out258[0x5F], MF_LIMITER_HI);
    } else {
        put_u16_le(&out258[0x5D], 0x0000U);
        put_u16_le(&out258[0x5F], MF_RANGE_STD_HI);
    }

    uint32_t flag_ac = 0x00UL;
    bool manual_mount = (bool)((ctx->session_state == E_SESSION_EMPTY_MOUNT) ||
                               (ctx->session_state == E_SESSION_PASSIVE_MOUNT));
    bool live_mf_sw = (bool)((ctx->id05.grp_present[7] != 0U) &&
                             (ctx->id05.is_mf_switch != 0U));
    if (manual_mount || live_mf_sw) {
        flag_ac |= 0x80UL;
    }
    if (ctx->af.barrel_limiter_on != 0U) {
        flag_ac |= 0x04UL;
    }
    out258[0xAC] = (uint8_t)flag_ac;
    out258[0xAD] = (ctx->id05.l_fn_pressed != 0U) ? 0x01U : 0x00U;
    out258[0xAE] = 0x00U;

    put_u16_le(&out258[0xB5], f_frac_q16);
    put_u16_le(&out258[0xB7], ctx->f_cur_mm);
    uint16_t f9_val = MF_BUCKET_MAX;
    if (ctx->dist_report_invalid != 0U) {
        f9_val = 0x0000U;
    } else if (ctx->af.barrel_limiter_on != 0U) {
        f9_val = MF_LIMITER_SPAN;
    }
    put_u16_le(&out258[0xB9], f9_val);
    put_u16_le(&out258[0xBB], ctx->ap.z_steps_0xbb);
}

static void build_cmd20_00_258b(const adapter_ctx_t *ctx, uint8_t *out258) {
    memset(out258, 0, Z_CMD20_00_BYTES);

    uint32_t f01 = (ctx->id05.f_nom_0p1mm >= 40U)
                       ? (uint32_t)ctx->id05.f_nom_0p1mm
                       : ((uint32_t)ctx->f_cur_mm * 10UL);
    uint16_t f_frac_q16 = (uint16_t)(((f01 % 10UL) * 65536UL) / 10UL);

    put_u16_le(&out258[0x01], ctx->ap.c_base_reported);
    put_u16_le(&out258[0x09], ctx->ap.c_cur);
    put_u16_le(&out258[0x0B], f_frac_q16);
    put_u16_le(&out258[0x0D], ctx->f_cur_mm);
    put_u16_le(&out258[0x0F], ctx->ap.c_cur);
    write_cmd20_pupil_block(ctx, out258);

    out258[0x29] = 0xFCU;
    out258[0x2A] = 0xFDU;
    out258[0x2B] = 0xF2U;
    out258[0x2C] = 0xF3U;

    out258[0x2D] = ctx->ap.z_mf_gate_2d;
    out258[0x2E] = ctx->ap.z_mf_gate_2e;
    put_u16_le(&out258[0x2F], ctx->ap.z_dist_0x117);
    put_u16_le(&out258[0x31], ctx->ap.z_dnorm_0x31);
    put_u16_le(&out258[0x33], ctx->ap.z_fnorm_0x33);

    for (uint32_t idx = 0U; idx < 6U; ++idx) {
        put_u16_le(&out258[0x39U + (idx * 2U)], ABERRATION_NEUTRAL_Z);
    }

    out258[0x57] = 0x01U;
    put_u16_le(&out258[0x59], ctx->ap.c_cur);
    put_u16_le(&out258[0x5B], ctx->ap.c_cur);

    write_cmd20_kz_block(ctx, out258);

    int32_t mech_span = (ctx->af.mech_hz > ctx->af.mech_lz)
                            ? (ctx->af.mech_hz - ctx->af.mech_lz)
                            : 16384L;
    put_s32_le(&out258[0x75], ctx->af.mech_lz);
    put_s32_le(&out258[0x79], mech_span);
    put_s32_le(&out258[0x7D], ctx->af.mech_hz);

    out258[0x83] = 0x03U;
    out258[0x84] = 0xFEU;
    bool fast_sony_xd = (bool)((ctx->brand == (lens_brand_t)SONY_BRAND_ID) &&
                               ((ctx->supports_120hz != 0U) || (ctx->lens_family == 2U)));
    put_u32_le(&out258[0x85], fast_sony_xd ? MAX_VPPS_STILLS_SONY_XD : MAX_VPPS_STILLS_DEF);
    put_u32_le(&out258[0x89], MAX_VPPS_MOVIE_DEF);
    out258[0x8D] = (ctx->af.barrel_limiter_on != 0U) ? 0x01U : 0x00U;

    out258[0xA1] = 0x00U;
    put_u16_le(&out258[0xA2], ctx->ap.c_open_cur);
    put_u16_le(&out258[0xA4], ctx->ap.c_cur);
    /* Set active lens readiness bit */
    out258[0xA9] = 0x40U;
    out258[0xAA] = 0x03U;

    write_cmd20_mf_and_limits(ctx, out258, f_frac_q16);

    out258[0xBD] = ctx->z_ring_heartbeat;
    out258[0xBE] = 0x00U;
    out258[0xBF] = (uint8_t)ctx->af.mf_ring_delta_bf;

    put_u16_le(&out258[0xC5], ctx->ap.cur_stopdown_delta);
    put_u16_le(&out258[0xC7], ctx->ap.max_stopdown_delta);
    put_u16_le(&out258[0xC9], 0x0000U);

    out258[0xCE] = 0x0CU;
    put_u16_le(&out258[0xCF], CDAF_WOBBLE_SPAN_DEF);
    put_u16_le(&out258[0xD1], CDAF_WOBBLE_SPAN_DEF);
    out258[0xD3] = 0x04U;
    out258[0xD4] = 0x04U;
    out258[0xD5] = CDAF_TURNAROUND_DEF;
    out258[0xD8] = 0x03U;

    if (ctx->vr.lens_has_ois != 0U) {
        out258[0xDD] = 0xFEU;
        out258[0xDE] = 0x8BU;
    } else {
        out258[0xDD] = 0x0EU;
        out258[0xDE] = 0x00U;
    }
    out258[0xDF] = (ctx->vr.stop_latched != 0U) ? 0x00U : ctx->vr.z_df_state;

    put_u16_le(&out258[0xE1], ctx->ap.z_eff_focal_q4);
    put_u16_le(&out258[0xE3], ctx->ap.z_mag_a3_q14);
    out258[0xE5] = 0x07U;
    out258[0xE6] = 0x05U;

    if (ctx->vr.lens_has_ois != 0U) {
        put_u16_le(&out258[0xE9], ctx->vr.cap_lens_lv);
        put_u16_le(&out258[0xEB], ctx->vr.cap_lens_exp);
        put_u16_le(&out258[0xED], ctx->vr.cap_body_lv);
        put_u16_le(&out258[0xEF], ctx->vr.cap_body_exp);
    } else {
        uint32_t stroke = (ctx->vr.body_mode == VR_BODY_CMD_OFF_A3)
                              ? 0UL
                              : ((uint32_t)ctx->f_cur_mm * 37UL);
        if (stroke > 4000UL) {
            stroke = 4000UL;
        }
        uint16_t ibis_u16 = (uint16_t)stroke;
        put_u16_le(&out258[0xE9], ibis_u16);
        put_u16_le(&out258[0xEB], ibis_u16);
        put_u16_le(&out258[0xED], ibis_u16);
        put_u16_le(&out258[0xEF], ibis_u16);
    }
    out258[0xF1] = 0x64U;

    uint8_t df_base = (uint8_t)((uint32_t)ctx->vr.z_df_state & 0x7FUL);
    out258[0xF2] = ((df_base == 0x31U) || (df_base == 0x34U)) ? 0x11U : 0x00U;

    /* Populate trailing synchro invalid sentinels */
    for (uint32_t idx = 0U; idx < 4U; ++idx) {
        put_u16_le(&out258[0xF9U + (idx * 2U)], SYNCHRO_INVALID_8000);
    }

    boryoza_seal_z_frame(out258, Z_CMD20_00_BYTES);
}

static uint32_t hash_c11_identity(const adapter_ctx_t *ctx) {
    static const uint32_t FNV_OFFSET = 2166136261UL;
    static const uint32_t FNV_PRIME  = 16777619UL;
    uint32_t hval = FNV_OFFSET;
    uint32_t tag_lo = 0UL;
    memcpy(&tag_lo, ctx->vendor_tag_5b, 4U);
    hval = (hval ^ tag_lo) * FNV_PRIME;
    hval = (hval ^ (uint32_t)ctx->vendor_tag_5b[4]) * FNV_PRIME;
    for (size_t idx = 0U; idx < sizeof(ctx->lens_name_ascii); idx += 4U) {
        uint32_t word = 0UL;
        memcpy(&word, &ctx->lens_name_ascii[idx], 4U);
        hval = (hval ^ word) * FNV_PRIME;
    }
    return hval;
}

static void publish_static_snapshots_cached(adapter_ctx_t *ctx, z_fast_snapshots_t *dst,
                                            uint8_t bank_bit) {
    z_static_snap_cache_t *cache = &ctx->z_snap_cache;
    uint8_t manual_only = ((ctx->session_state == E_SESSION_EMPTY_MOUNT) ||
                           (ctx->session_state == E_SESSION_PASSIVE_MOUNT)) ? 1U : 0U;
    uint8_t fmt_flags = (uint8_t)(((ctx->is_apsc != 0U) ? 0x10U : 0U) |
                                  ((ctx->is_macro != 0U) ? 0x04U : 0U));
    uint8_t scale_n = (manual_only != 0U) ? 0U : ctx->af.scale_exp_n;
    uint16_t eff_ver = (manual_only != 0U) ? 0U : ctx->e_lens_ver;
    uint8_t dist_invalid = (ctx->dist_report_invalid != 0U) ? 1U : 0U;
    uint32_t c11_hash = hash_c11_identity(ctx);

    bool c10_match = (bool)((cache->c10_z_lid == ctx->z_primary_lens_id) &&
                            (cache->c10_e_lid == ctx->e_lens_id) &&
                            (cache->c10_e_ver == eff_ver) &&
                            (cache->c10_flags == fmt_flags) &&
                            (cache->c10_scale_n == scale_n) &&
                            (cache->c10_state_class == manual_only) &&
                            (cache->c10_identity_ready == ctx->identity_ready) &&
                            (cache->dist_invalid == dist_invalid));
    bool c11_match = (bool)(c10_match && (cache->c11_identity_hash == c11_hash));
    if (!c10_match) {
        cache->c10_z_lid = ctx->z_primary_lens_id;
        cache->c10_e_lid = ctx->e_lens_id;
        cache->c10_e_ver = eff_ver;
        cache->c10_flags = fmt_flags;
        cache->c10_scale_n = scale_n;
        cache->c10_state_class = manual_only;
        cache->c10_identity_ready = ctx->identity_ready;
        cache->c10_done_mask = 0U;
    }
    if (!c11_match) {
        cache->c11_identity_hash = c11_hash;
        cache->c11_done_mask = 0U;
    }
    if ((cache->c10_done_mask & bank_bit) == 0U) {
        build_cmd10_28b(ctx, dst->cmd10);
        cache->c10_done_mask = (uint8_t)(cache->c10_done_mask | bank_bit);
    }
    if ((cache->c11_done_mask & bank_bit) == 0U) {
        build_cmd11_74b(ctx, dst->cmd11);
        cache->c11_done_mask = (uint8_t)(cache->c11_done_mask | bank_bit);
    }

    bool c12_match = (bool)((cache->c12_open_w == ctx->ap.c_open_wide) &&
                            (cache->c12_open_t == ctx->ap.c_open_tele) &&
                            (cache->c12_f_cur == ctx->f_cur_mm) &&
                            (cache->c12_f_min == ctx->f_min_mm) &&
                            (cache->c12_f_max == ctx->f_max_mm) &&
                            (cache->c12_vr0 == ctx->vr.z_vr_cap0) &&
                            (cache->c12_vr1 == ctx->vr.z_vr_cap1) &&
                            (cache->dist_invalid == dist_invalid));
    if (!c12_match) {
        cache->c12_open_w = ctx->ap.c_open_wide;
        cache->c12_open_t = ctx->ap.c_open_tele;
        cache->c12_f_cur = ctx->f_cur_mm;
        cache->c12_f_min = ctx->f_min_mm;
        cache->c12_f_max = ctx->f_max_mm;
        cache->c12_vr0 = ctx->vr.z_vr_cap0;
        cache->c12_vr1 = ctx->vr.z_vr_cap1;
        cache->c12_done_mask = 0U;
    }
    cache->dist_invalid = dist_invalid;
    if ((cache->c12_done_mask & bank_bit) == 0U) {
        build_cmd12_66b(ctx, dst->cmd12);
        cache->c12_done_mask = (uint8_t)(cache->c12_done_mask | bank_bit);
    }
}

void z_snapshot_publish(adapter_ctx_t *ctx, const optics_workspace_t *optics) {
    (void)optics;
    uint8_t next_bank = (uint8_t)((uint32_t)ctx->active_z_bank ^ 1UL);
    z_fast_snapshots_t *dst = &ctx->z_banks[next_bank];
    uint8_t bank_bit = (uint8_t)(1U << (next_bank & 1U));

    publish_static_snapshots_cached(ctx, dst, bank_bit);
    build_cmd20_00_258b(ctx, dst->cmd20_00);
    build_cmd20_01_24b(ctx, dst->cmd20_01);

    ctx->active_z_bank = next_bank;
}

static void apply_sync_mosi_if_present(adapter_ctx_t *ctx, const optics_workspace_t *optics,
                                       const z_cmd_req_t *req) {
    if ((req->mosi_buf == NULL) || (req->mosi_len < 2U)) {
        return;
    }
    if ((req->cmd == 0x20U) && (req->sub == 0x00U)) {
        uint8_t prev_hb = ctx->z_ring_heartbeat;
        uint8_t prev_b18 = ctx->vr.e_id03_b18;
        uint8_t prev_df = ctx->vr.z_df_state;
        aperture_controls_on_z_mosi20(ctx, req->mosi_buf, req->mosi_len, &req->now_ticks);
        if ((ctx->z_ring_heartbeat != prev_hb) ||
            (ctx->vr.e_id03_b18 != prev_b18) ||
            (ctx->vr.z_df_state != prev_df)) {
            z_snapshot_publish(ctx, optics);
        }
    } else if (((req->cmd == 0xA0U) && (req->sub <= 0x07U)) ||
               ((req->cmd == 0xA1U) && (req->sub == 0x00U)) ||
               ((req->cmd == 0xA3U) && (req->sub <= 0x01U))) {
        aperture_controls_on_z_a1(ctx, req->mosi_buf, req->mosi_len);
        z_snapshot_publish(ctx, optics);
    } else if ((req->cmd == 0x16U) || (req->cmd == 0x50U) ||
               ((req->cmd >= 0xD2U) && (req->cmd <= 0xD8U))) {
        vr_controller_on_z_command(ctx, req->cmd, req->mosi_buf, req->mosi_len, &req->now_ticks);
        z_snapshot_publish(ctx, optics);
    } else if (((req->cmd >= 0xC0U) && (req->cmd <= 0xC4U)) ||
               ((req->cmd >= 0xE0U) && (req->cmd <= 0xEFU))) {
        af_servo_on_z_command(ctx, req->cmd, req->mosi_buf, req->mosi_len);
    }
}

static void copy_and_seal_prefix(uint8_t *dst, size_t win_bytes,
                                 const uint8_t *src, size_t src_bytes) {
    size_t copy_n = (win_bytes <= src_bytes) ? win_bytes : src_bytes;
    memcpy(dst, src, copy_n);
    boryoza_seal_z_frame(dst, win_bytes);
}

static bool try_dispatch_large_cmd24(adapter_ctx_t *ctx, const optics_workspace_t *optics,
                                     const z_cmd_req_t *req, size_t win_bytes,
                                     const uint8_t **out_direct_ptr) {
    if (req->cmd != 0x24U) {
        return false;
    }
    uint8_t obank = optics->active_optics_bank;
    if (((req->sub == 0x00U) || (req->sub == 0x02U)) &&
        (out_direct_ptr != NULL) && (win_bytes == (size_t)Z_CMD24_A_BYTES)) {
        *out_direct_ptr = optics->cmd24_sub00[obank];
        ctx->cmd24_read_mask = (uint8_t)((uint32_t)ctx->cmd24_read_mask | 1UL);
        if ((((uint32_t)ctx->cmd24_read_mask & 3UL) == 3UL) && (ctx->optics_ready != 0U)) {
            optics_disconnect_post_init(ctx);
        }
        return true;
    }
    if (((req->sub == 0x10U) || (req->sub == 0x12U)) &&
        (out_direct_ptr != NULL) && (win_bytes == (size_t)Z_CMD24_B_BYTES)) {
        *out_direct_ptr = optics->cmd24_sub10[obank];
        ctx->cmd24_read_mask = (uint8_t)((uint32_t)ctx->cmd24_read_mask | 2UL);
        if ((((uint32_t)ctx->cmd24_read_mask & 3UL) == 3UL) && (ctx->optics_ready != 0U)) {
            optics_disconnect_post_init(ctx);
        }
        return true;
    }
    return false;
}

static uint8_t step_cmd15_ready_stage(adapter_ctx_t *ctx, uint64_t now_ticks) {
    static const uint64_t STAGE2_TICKS = 72000ULL;  /* 24 ms */
    static const uint64_t STAGE0_TICKS = 168000ULL; /* 56 ms */

    if ((ctx->session_state == E_SESSION_POWERUP_WAIT) ||
        (ctx->session_state == E_SESSION_HANDSHAKE) ||
        (ctx->wake_from_park_req != 0U) ||
        (ctx->wake_rehandshake != 0U) ||
        ((ctx->session_state == E_SESSION_PARKED) &&
         ((ctx->vr.hotline_enabled != 0U) || (ctx->wake_boot_prologue_seen != 0U)))) {
        ctx->cmd15_stage = 3U;
        ctx->cmd15_polls = 0U;
        ctx->cmd15_open_ticks = 0ULL;
        return 3U;
    }

    if (ctx->cmd15_open_ticks == 0ULL) {
        ctx->cmd15_open_ticks = (now_ticks != 0ULL) ? now_ticks : 1ULL;
    }
    uint64_t elapsed = (now_ticks >= ctx->cmd15_open_ticks)
                           ? (now_ticks - ctx->cmd15_open_ticks)
                           : 0ULL;

    uint8_t cur_poll = ctx->cmd15_polls;
    if (cur_poll < 10U) {
        ctx->cmd15_polls = (uint8_t)(cur_poll + 1U);
    }

    if ((ctx->cmd15_stage == 0U) || (elapsed >= STAGE0_TICKS) || (cur_poll >= 2U)) {
        ctx->cmd15_stage = 0U;
        if ((ctx->session_state == E_SESSION_ACTIVE) && (ctx->optics_ready != 0U)) {
            optics_disconnect_post_init(ctx);
        }
    } else if ((elapsed >= STAGE2_TICKS) || (cur_poll >= 1U)) {
        ctx->cmd15_stage = 2U;
    } else {
        ctx->cmd15_stage = 3U;
    }
    return ctx->cmd15_stage;
}

static size_t z8_cmd70_asymmetric_rx_len(const z_cmd_req_t *req, size_t win_bytes) {
    uint8_t sub = req->sub;
    if ((sub == 0x45U) || (sub == 0xDAU) || (sub == 0xDDU) || (sub == 0xDEU) || (sub == 0xFDU)) {
        return (win_bytes >= 8U) ? 8U : win_bytes;
    }
    if ((sub == 0x46U) || ((sub >= 0xB0U) && (sub <= 0xB6U)) ||
        ((sub >= 0xD1U) && (sub <= 0xD9U)) || (sub == 0xE0U) ||
        (sub == 0xE7U) || (sub == 0xE8U) || (sub == 0xEAU) ||
        (sub == 0xECU) || (sub == 0xEFU) || (sub == 0xFCU) || (sub == 0xFEU)) {
        return (win_bytes >= 6U) ? 6U : win_bytes;
    }
    return win_bytes;
}

static void fill_aux_small_window(adapter_ctx_t *ctx, const z_cmd_req_t *req,
                                  uint8_t *small_tx_buf, size_t win_bytes) {
    if ((req->cmd == 0xA0U) && (req->sub <= 0x07U) && (win_bytes == 24U)) {
        aperture_controls_fill_z_a0(ctx, req->sub, small_tx_buf);
    } else if ((req->cmd == 0xC3U) && (req->sub == 0x00U) && (win_bytes == 20U)) {
        for (uint32_t idx = 0U; idx < 6U; ++idx) {
            put_u16_le(&small_tx_buf[0x01U + (idx * 2U)], ABERRATION_NEUTRAL_Z);
        }
        put_u16_le(&small_tx_buf[0x0DU], KZ_NONLIN_DISABLED);
        put_u16_le(&small_tx_buf[0x0FU], KZ_NONLIN_DISABLED);
        small_tx_buf[0x11U] = ctx->af.kz_b0;
        small_tx_buf[0x12U] = ctx->af.kz_b1;
        boryoza_seal_z_frame(small_tx_buf, win_bytes);
    } else if ((req->cmd == 0x51U) && (win_bytes >= 4U)) {
        small_tx_buf[1] = vr_controller_poll_cmd51_status(ctx, req->now_ticks);
        boryoza_seal_z_frame(small_tx_buf, win_bytes);
    } else if ((req->cmd == 0xF3U) && (req->sub == 0x00U) && (win_bytes >= 8U)) {
        small_tx_buf[2] = 0x01U;
        small_tx_buf[3] = 0x83U;
        boryoza_seal_z_frame(small_tx_buf, 8U);
    } else if ((req->cmd == 0x74U) && (req->sub == 0x00U) && (win_bytes >= 8U)) {
        small_tx_buf[1] = 0x01U;
        small_tx_buf[2] = 0x23U;
        small_tx_buf[4] = 0x03U;
        small_tx_buf[5] = 0x01U;
        boryoza_seal_z_frame(small_tx_buf, 8U);
    } else if ((req->cmd == 0x7EU) && ((req->sub == 0x02U) || (req->sub == 0x03U)) && (win_bytes >= 6U)) {
        boryoza_seal_z_frame(small_tx_buf, 6U);
    } else if ((req->cmd == 0x7EU) && (req->sub == 0x06U) && (win_bytes >= 8U)) {
        boryoza_seal_z_frame(small_tx_buf, 8U);
    } else if (req->cmd == 0x70U) {
        boryoza_seal_z_frame(small_tx_buf, z8_cmd70_asymmetric_rx_len(req, win_bytes));
    } else {
        boryoza_seal_z_frame(small_tx_buf, win_bytes);
    }
}

static bool is_z_boot_recog_cmd(uint8_t cmd) {
    return (bool)((cmd == 0x1FU) || (cmd == 0xF3U) || (cmd == 0x10U));
}

static void maybe_arm_wake_on_recog(adapter_ctx_t *ctx, uint8_t cmd) {
    if (!is_z_boot_recog_cmd(cmd)) {
        return;
    }
    bool parked_only = (bool)(ctx->session_state == E_SESSION_PARKED);
    bool active_cold_recog = (bool)((ctx->session_state == E_SESSION_ACTIVE) &&
                                    (ctx->z_sck_powerdown_latched != 0U) &&
                                    (cmd == 0x1FU));
    if ((!parked_only) && (!active_cold_recog)) {
        return;
    }
    if ((cmd == 0x1FU) || (cmd == 0xF3U)) {
        ctx->wake_boot_prologue_seen = 1U;
    }
    ctx->z_sck_powerdown_latched = 0U;
    lens_session_arm_home_retrigger(ctx);
    ctx->session_state = E_SESSION_PARKED;
    ctx->vr.park_state = PARK_IDLE;
    ctx->vr.hotline_enabled = 0U;
    ctx->vr.hotline_seq = 0U;
    ctx->cmd15_stage = 3U;
    ctx->cmd15_polls = 0U;
    ctx->cmd15_open_ticks = 0ULL;
    if (ctx->wake_from_park_req == 0U) {
        ctx->wake_from_park_req = 1U;
    }
}

static void fill_small_snapshot_window(adapter_ctx_t *ctx, const z_cmd_req_t *req,
                                       uint8_t *small_tx_buf, size_t win_bytes) {
    const z_fast_snapshots_t *snap = &ctx->z_banks[ctx->active_z_bank];
    memset(small_tx_buf, 0, win_bytes);

    maybe_arm_wake_on_recog(ctx, req->cmd);

    if ((req->cmd == 0x10U) && (req->sub == 0x00U) && (win_bytes >= 2U)) {
        copy_and_seal_prefix(small_tx_buf, win_bytes, snap->cmd10, Z_CMD10_BYTES);
    } else if ((req->cmd == 0x11U) && (req->sub == 0x00U) && (win_bytes >= 2U)) {
        copy_and_seal_prefix(small_tx_buf, win_bytes, snap->cmd11, Z_CMD11_BYTES);
    } else if ((req->cmd == 0x12U) && (req->sub == 0x00U) && (win_bytes >= 2U)) {
        copy_and_seal_prefix(small_tx_buf, win_bytes, snap->cmd12, Z_CMD12_BYTES);
    } else if ((req->cmd == 0x15U) && (req->sub == 0x00U) && (win_bytes >= 6U)) {
        if ((ctx->wake_rehandshake != 0U) || (ctx->wake_from_park_req != 0U)) {
            ctx->wake_boot_prologue_seen = 1U;
        }
        if ((ctx->home_retrigger_armed != 0U) || (ctx->z_sck_powerdown_latched != 0U)) {
            (void)lens_session_retrigger_homing(ctx, req->now_ticks);
        }
        small_tx_buf[1] = step_cmd15_ready_stage(ctx, req->now_ticks);
        small_tx_buf[3] = 0x02U;
        boryoza_seal_z_frame(small_tx_buf, 6U);
    } else if ((req->cmd == 0x16U) && (win_bytes >= 4U)) {
        small_tx_buf[1] = (uint8_t)((uint32_t)ctx->vr.hotline_seq - 10UL);
        boryoza_seal_z_frame(small_tx_buf, win_bytes);
    } else if ((req->cmd == 0x20U) && (req->sub == 0x00U) && (win_bytes >= 2U)) {
        copy_and_seal_prefix(small_tx_buf, win_bytes, snap->cmd20_00, Z_CMD20_00_BYTES);
    } else if ((req->cmd == 0x20U) && (req->sub == 0x01U) && (win_bytes >= 2U)) {
        copy_and_seal_prefix(small_tx_buf, win_bytes, snap->cmd20_01, Z_CMD20_01_BYTES);
    } else {
        fill_aux_small_window(ctx, req, small_tx_buf, win_bytes);
    }
}

size_t z_dispatch_command_ram_safe(adapter_ctx_t *ctx, const optics_workspace_t *optics,
                                   const z_cmd_req_t *req, uint8_t *small_tx_buf,
                                   const uint8_t **out_direct_ptr) {
    if (vr_controller_step(ctx, req->now_ticks)) {
        z_snapshot_publish(ctx, optics);
    }
    size_t win_bytes = ((size_t)req->req_words + 1U) * 2U;
    if (out_direct_ptr != NULL) {
        *out_direct_ptr = NULL;
    }

    if (try_dispatch_large_cmd24(ctx, optics, req, win_bytes, out_direct_ptr)) {
        return win_bytes;
    }
    if (win_bytes > Z_MAX_SMALL_BYTES) {
        size_t cap = (win_bytes <= (size_t)Z_LARGE_ZERO_BUF_BYTES)
                         ? (size_t)Z_LARGE_ZERO_BUF_BYTES
                         : (size_t)E_LF_WORKBUF_BYTES;
        if (win_bytes > cap) {
            win_bytes = cap;
        }
        if (out_direct_ptr != NULL) {
            /* Use 36KiB SRAM2 lf_work_buf when >1028B so DMA1 stays within SRAM2 */
            uint8_t *zbuf = (win_bytes <= (size_t)Z_LARGE_ZERO_BUF_BYTES)
                                ? ((optics_workspace_t *)(uintptr_t)optics)->z_large_zero_buf // NOLINT
                                : ((optics_workspace_t *)(uintptr_t)optics)->lf_work_buf;     // NOLINT
            memset(zbuf, 0, win_bytes);
            size_t seal_len = ((req->cmd == 0x70U) && (req->sub == 0x03U)) ? 6U : win_bytes;
            boryoza_seal_z_frame(zbuf, seal_len);
            *out_direct_ptr = zbuf;
        }
        return win_bytes;
    }

    fill_small_snapshot_window(ctx, req, small_tx_buf, win_bytes);
    apply_sync_mosi_if_present(ctx, optics, req);
    return win_bytes;
}
/* NOLINTEND(readability-magic-numbers) */
