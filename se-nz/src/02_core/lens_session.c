#include "../../include/boryoza_types.h"
#include "../../include/boryoza_math_mem.h"
#include "../../include/lens_db.h"
#include "../../include/lens_db_generated.h"
#if !defined(__STDC_HOSTED__) || (__STDC_HOSTED__ == 0)
void retram_warm_cache_store_ds13(uint16_t e_lens_id, const uint8_t *buf, uint16_t len);
bool retram_warm_cache_load_ds13(uint16_t e_lens_id, uint8_t *out_buf, uint16_t *out_len);
#endif

enum {
    ID03_PARAM_BYTES     = 20,
    ID03_COMPOSITE_BYTES = 23,
    ID04_HDR_PARAM_BYTES = 13,
    ID04_MAX_PARAM_BYTES = 34,
};

static const uint32_t CRC32_INIT_MASK        = 0xFFFFFFFFUL;
static const size_t   EMOUNT_ENVELOPE_OVHD   = 9U;
static const uint8_t  EMOUNT_FRAME_TYPE_SYNC = 0x01U;
static const uint8_t  EMOUNT_FRAME_TYPE_B2L  = 0x02U;
static const uint8_t  EMOUNT_SOF             = 0xF0U;
static const uint8_t  EMOUNT_EOF             = 0x55U;
static const uint8_t  Z_SYNC_HDR             = 0x55U;
static const uint8_t  CAP_CMD_MAX            = 0x63U;
static const uint8_t  ATTACH_DEBOUNCE_THRESH = 10U;
static const uint8_t  DETACH_DEBOUNCE_THRESH = 100U;
static const uint8_t  HOME_RETRIGGER_BUDGET  = 3U;
static const uint32_t BOOT_VD_WAIT_CYCLES    = 2UL;
static const uint8_t  MAX_CMD01_RETRIES      = 18U;
static const uint8_t  WAKE_CMD01_RETRIES     = 5U;
static const size_t   TX07_IDENTITY_MIN_BYTES = 11U;
static const size_t   TX3D_CAPABILITY_BYTES = 63U;
static const size_t   TX3D_CAP14_OFFSET = 33U;
static const uint8_t  TX3D_CAP14_MASK = 64U;
static const uint8_t  TX07_CLASS_MASK = 3U;
static const uint8_t  E_CMD_IRIS_HOME_10     = 0x10U;
static const uint8_t  CMD10_HOME_STD         = 0x1FU;
static const uint8_t  CMD10_HOME_FAM2        = 0x3FU;
static const uint8_t  CMD10_HOME_IRIS_ONLY   = 0x08U;
static const uint8_t  CMD10_HOME_FOCUS_ONLY  = 0x04U;
static const uint16_t E_AV_MIN_OPEN_P        = 0x0E00U;
static const uint8_t  E_CMD_APERTURE_STATUS  = 0x3EU;
static const size_t   APERTURE_STATUS_BYTES = 11U;
static const uint64_t CMD01_RETRY_TICKS      = 36000ULL;
static const uint64_t HOME_REPOLL_TICKS      = 1650000ULL;
static const uint64_t WAIT_05_06_MAX_TICKS   = 900000ULL;
static const uint64_t RX_SILENCE_DETACH_TKS  = 900000ULL;
static const uint64_t HS_STEP_TIMEOUT_TICKS  = 240000ULL;   /* 80 ms per command step */
static const uint64_t HS_HOME_TIMEOUT_TICKS  = 13500000ULL; /* 4500 ms for 2-3 stroke telephoto homing */
static const uint32_t BAUD_750K              = 750000UL;
static const uint32_t BAUD_1500K             = 1500000UL;
static const uint8_t  VSYNC_60HZ             = 60U;
static const uint8_t  VSYNC_120HZ            = 120U;
static const uint16_t DEFAULT_EMPTY_FOCAL_MM = 50U;
static const uint16_t DEFAULT_EMPTY_AV_OPEN  = 0x0240U;
static const uint16_t DEFAULT_EMPTY_AV_MIN   = 0x0780U;
static const uint16_t DEFAULT_F22_AV_MIN     = 0x0840U;
static const uint16_t DEFAULT_Z_GENERIC_ID     = 0x00B0U;
static const uint16_t DEFAULT_Z_PROVISIONAL_ID = 0x00A0U;
static const uint16_t DEFAULT_Z_ACTIVE_BASE_ID = 0x0C00U;
static const uint16_t DEFAULT_EMPTY_VER        = 0x0200U;
static const uint16_t SONY_LENS_ID_MIN       = 0x8000U;
static const uint16_t SONY_LENS_ID_MAX       = 0x8FFFU;
static const uint16_t SONY_LAEA5_ID          = 0x0007U;
static const uint16_t SONY_50150_ID          = 0x807BU;
static const uint16_t SIGMA_150600_ID_REV2   = 0xC56EU;
static const uint16_t SIGMA_150600_ID_REV1   = 0xC55FU;
static const uint16_t ZEISS_BATIS40_ID       = 0xC044U;
static const uint32_t DS13_BIT_MASK          = (1UL << 13U);
static const uint32_t LF_DS0_BIT_MASK        = (1UL << 0U);
static const uint32_t ID05_STD_15GRP_MASK    = 0x00007FFFUL;
static const uint32_t ID05_FULL_18GRP_MASK   = 0x0003FFFFUL;
static const uint8_t  ID06_STD_6GRP_MASK     = 0x3FU;
/* TX08 reply payload offsets (payload = &wire[6] = &TX08[1]) */
static const size_t   TX08_APSC_OFFSET       = 0x08U;
static const size_t   TX08_OIS_OFFSET        = 0x0BU;
static const size_t   TX08_120HZ_OFFSET      = 0x0CU;
static const size_t   TX08_FMIN_BCD_OFFSET   = 0x10U;
static const size_t   TX08_FMAX_BCD_OFFSET   = 0x12U;
static const size_t   TX08_FISHEYE_OFFSET    = 0x18U;
static const uint32_t TX08_120HZ_BIT         = 0x80UL;
static const uint32_t TX08_FISHEYE_BIT       = 0x40UL;
static const int32_t  ID06_LIMIT_MARGIN_SE   = 3;
static const uint32_t ID03_IRIS_WINDOW_DEF   = 0x00754BUL;
static const uint8_t  ID03_WIRE13_STD_GEAR   = 0x18U;
static const uint8_t  ID03_WIRE13_EXT_GEAR   = 0x1CU;
static const uint8_t  CMD2F_SUB0             = 0x2FU;
static const uint8_t  CMD2F_TAM_SUB1         = 0x09U;
static const uint8_t  CMD2F_TAM_SUB2         = 0x0AU;
static const uint8_t  CMD2F_STD_SUB1         = 0x15U;
static const uint8_t  CMD2F_STD_SUB2_BASE    = 0x16U;
static const uint32_t CLEAR_BIT0_MASK        = 0xFEUL;
static const uint16_t DIST_INF_SENTINEL      = 0xFFFFU;
static const uint8_t  MAG_INVALID_SENTINEL   = 0xFFU;
static const int8_t   DEFAULT_ROOM_TEMP_C    = 25;
static const uint32_t MASK_16BIT             = 0xFFFFUL;
static const uint32_t BYTE_MASK              = 0xFFUL;
static const uint32_t SHIFT_8                = 8U;
static const uint32_t SHIFT_16               = 16U;
static const uint32_t SHIFT_24               = 24U;

static const float SPAN_GUARD_F = 0.5F;
static const float DECIMM_TO_MM = 0.1F;
static const float MM_TO_M_F    = 0.001F;
static const float HALF_F       = 0.5F;

/* NOLINTBEGIN(readability-magic-numbers) */
static const uint8_t GRP05_LEN[E_ID05_GROUP_COUNT] = {
    8U, 1U, 8U, 3U, 10U, 28U, 2U, 6U, 1U, 2U, 8U, 6U, 6U, 6U, 1U, 6U, 3U, 3U,
};
/* NOLINTEND(readability-magic-numbers) */

static uint16_t read_u16_le(const uint8_t *buf) {
    return (uint16_t)((uint32_t)buf[0] | ((uint32_t)buf[1] << SHIFT_8));
}

static uint32_t read_u32_le(const uint8_t *buf) {
    return (uint32_t)buf[0] |
           ((uint32_t)buf[1] << SHIFT_8) |
           ((uint32_t)buf[2] << SHIFT_16) |
           ((uint32_t)buf[3] << SHIFT_24);
}

/* NOLINTBEGIN(readability-magic-numbers) — SWAR checksums and table LUT */
uint8_t boryoza_neg_sum8(const uint8_t *buf, size_t len_without_csum) {
    uint32_t acc = 0U;
    size_t idx = 0U;
    while ((idx < len_without_csum) && (((uintptr_t)&buf[idx] & 3U) != 0U)) {
        acc += (uint32_t)buf[idx];
        ++idx;
    }
    while ((idx + 4U) <= len_without_csum) {
        size_t rem = len_without_csum - idx;
        size_t chunk_bytes = (rem >= 256U) ? 256U : (rem & ~((size_t)3U));
        size_t chunk_end = idx + chunk_bytes;
        uint32_t pair_sum = 0U;
        while (idx < chunk_end) {
            uint32_t word_val = *(const uint32_t *)(const void *)&buf[idx];
            pair_sum += (word_val & 0x00FF00FFUL) + ((word_val >> 8U) & 0x00FF00FFUL);
            idx += 4U;
        }
        acc += (pair_sum & 0xFFFFUL) + (pair_sum >> 16U);
    }
    while (idx < len_without_csum) {
        acc += (uint32_t)buf[idx];
        ++idx;
    }
    return (uint8_t)((0UL - acc) & BYTE_MASK);
}

void boryoza_seal_z_frame(uint8_t *buf, size_t total_len) {
    if (total_len < 2U) {
        return;
    }
    buf[0] = Z_SYNC_HDR;
    buf[total_len - 1U] = boryoza_neg_sum8(buf, total_len - 1U);
}

uint16_t boryoza_sum16_emount(const uint8_t *wire, size_t wire_len) {
    if (wire_len < 4U) {
        return 0U;
    }
    uint32_t acc = 0U;
    size_t end = wire_len - 3U;
    size_t idx = 1U;
    while ((idx < end) && (((uintptr_t)&wire[idx] & 3U) != 0U)) {
        acc += (uint32_t)wire[idx];
        ++idx;
    }
    while ((idx + 4U) <= end) {
        size_t rem = end - idx;
        size_t chunk_bytes = (rem >= 256U) ? 256U : (rem & ~((size_t)3U));
        size_t chunk_end = idx + chunk_bytes;
        uint32_t pair_sum = 0U;
        while (idx < chunk_end) {
            uint32_t word_val = *(const uint32_t *)(const void *)&wire[idx];
            pair_sum += (word_val & 0x00FF00FFUL) + ((word_val >> 8U) & 0x00FF00FFUL);
            idx += 4U;
        }
        acc += (pair_sum & 0xFFFFUL) + (pair_sum >> 16U);
    }
    while (idx < end) {
        acc += (uint32_t)wire[idx];
        ++idx;
    }
    return (uint16_t)(acc & MASK_16BIT);
}

size_t boryoza_wrap_emount_frame(uint8_t *out_wire, uint8_t frame_type, const uint8_t *seq_ptr,
                                 uint8_t cmd, const uint8_t *payload, size_t payload_len) {
    size_t wire_len = payload_len + EMOUNT_ENVELOPE_OVHD;
    out_wire[0] = EMOUNT_SOF;
    out_wire[1] = (uint8_t)(wire_len & BYTE_MASK);
    out_wire[2] = (uint8_t)((wire_len >> SHIFT_8) & BYTE_MASK);
    out_wire[3] = frame_type;
    out_wire[4] = (seq_ptr != NULL) ? *seq_ptr : 0U;
    static const size_t WIRE_OFF_CMD = 5U;
    static const size_t WIRE_OFF_PAY = 6U;
    out_wire[WIRE_OFF_CMD] = cmd;
    if ((payload != NULL) && (payload_len > 0U)) {
        memcpy(&out_wire[WIRE_OFF_PAY], payload, payload_len);
    }
    uint32_t csum = boryoza_sum16_emount(out_wire, wire_len);
    out_wire[wire_len - 3U] = (uint8_t)(csum & BYTE_MASK);
    out_wire[wire_len - 2U] = (uint8_t)((csum >> SHIFT_8) & BYTE_MASK);
    out_wire[wire_len - 1U] = EMOUNT_EOF;
    return wire_len;
}

static uint32_t g_crc32_nibble_lut[16] = {
    0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
    0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
    0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
    0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL,
};

uint32_t boryoza_crc32_ieee(const void *data, size_t len) {
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = CRC32_INIT_MASK;
    for (size_t idx = 0U; idx < len; ++idx) {
        uint32_t byte_val = (uint32_t)bytes[idx];
        crc = (crc >> 4U) ^ g_crc32_nibble_lut[(crc ^ byte_val) & 0x0FUL];
        crc = (crc >> 4U) ^ g_crc32_nibble_lut[(crc ^ (byte_val >> 4U)) & 0x0FUL];
    }
    return ~crc;
}
/* NOLINTEND(readability-magic-numbers) */

const lens_db_entry_t *lens_db_lookup(uint16_t e_lens_id) {
    static const uint32_t LENS_BUCKET_SHIFT = 12U;
    size_t bucket = (size_t)((uint32_t)e_lens_id >> LENS_BUCKET_SHIFT);
    size_t low = (size_t)g_lens_bucket_bounds[bucket];
    size_t high = (size_t)g_lens_bucket_bounds[bucket + 1U];
    while (low < high) {
        size_t mid = low + ((high - low) / 2U);
        uint16_t mid_id = g_lens_catalog[mid].e_lens_id;
        if (mid_id == e_lens_id) {
            return &g_lens_catalog[mid];
        }
        if (mid_id < e_lens_id) {
            low = mid + 1U;
        } else {
            high = mid;
        }
    }
    return NULL;
}

const lens_db_entry_t *lens_db_lookup_cached(adapter_ctx_t *ctx) {
    if (ctx == NULL) {
        return NULL;
    }
    uint16_t lid = ctx->e_lens_id;
    if (lid == 0U) {
        return NULL;
    }
    if (ctx->cached_db_lid == lid) {
        if ((ctx->cached_db_idx >= 0) &&
            (ctx->cached_db_idx < (int16_t)LENS_DB_CATALOG_COUNT) &&
            (g_lens_catalog[ctx->cached_db_idx].e_lens_id == lid)) {
            return &g_lens_catalog[ctx->cached_db_idx];
        }
        return NULL;
    }
    const lens_db_entry_t *entry = lens_db_lookup(lid);
    ctx->cached_db_lid = lid;
    ctx->cached_db_idx = (entry != NULL) ? (int16_t)(entry - g_lens_catalog) : -1;
    return entry;
}

const char *lens_db_entry_name(const lens_db_entry_t *entry) {
    if ((entry == NULL) || ((size_t)entry->name_off >= sizeof(g_lens_names_blob))) {
        return "E-Mount AF Lens";
    }
    return (const char *)&g_lens_names_blob[entry->name_off];
}

const char *lens_db_vendor_tag5(uint8_t brand) {
    switch (brand) {
        case LENS_BRAND_SONY:
            return "SONY\0";
        case LENS_BRAND_SIGMA:
            return "SIGMA";
        case LENS_BRAND_TAMRON:
            return "TAMRO";
        case LENS_BRAND_SAMYANG:
            return "SAMYA";
        case LENS_BRAND_ZEISS:
            return "ZEISS";
        default:
            return "NIKON";
    }
}

const uint8_t *sigma_150600_get_optical_a_packed(size_t *out_len) {
    if (out_len != NULL) {
        *out_len = sizeof(g_optical_primary_a_packed);
    }
    return g_optical_primary_a_packed;
}

/* NOLINTBEGIN(readability-magic-numbers) — delta-nibble spectral curve unpacker */
bool lens_db_unpack_spectral81(int16_t spectral_idx, uint8_t out81[SPECTRAL_81_POINTS]) {
    if ((spectral_idx < 0) || (spectral_idx >= (int16_t)LENS_DB_SPECTRAL_COUNT) ||
        (out81 == NULL)) {
        return false;
    }
    uint16_t start = g_spectral_packed_offs[spectral_idx];
    uint16_t end = g_spectral_packed_offs[spectral_idx + 1];
    if ((start >= end) || ((size_t)end > sizeof(g_spectral_packed_blob))) {
        return false;
    }
    out81[0] = g_spectral_packed_blob[start];
    const uint8_t *nib_ptr = &g_spectral_packed_blob[(size_t)start + 1U];
    size_t nib_total = ((size_t)end - (size_t)start - 1U) * 2U;
    size_t nib_pos = 0U;
    for (size_t idx = 1U; idx < (size_t)SPECTRAL_81_POINTS; ++idx) {
        if (nib_pos >= nib_total) {
            return false;
        }
        uint32_t raw_byte = (uint32_t)nib_ptr[nib_pos >> 1U];
        uint32_t shift_bits = (uint32_t)(nib_pos & 1U) * 4UL;
        uint32_t nib = (raw_byte >> shift_bits) & 0x0FUL;
        nib_pos += 1U;
        if (nib == 0x08UL) {
            if ((nib_pos + 1U) >= nib_total) {
                return false;
            }
            uint32_t lo_shift = (uint32_t)(nib_pos & 1U) * 4UL;
            uint32_t nib_lo = ((uint32_t)nib_ptr[nib_pos >> 1U] >> lo_shift) & 0x0FUL;
            nib_pos += 1U;
            uint32_t hi_shift = (uint32_t)(nib_pos & 1U) * 4UL;
            uint32_t nib_hi = ((uint32_t)nib_ptr[nib_pos >> 1U] >> hi_shift) & 0x0FUL;
            nib_pos += 1U;
            out81[idx] = (uint8_t)(nib_lo | (nib_hi << 4UL));
        } else {
            int32_t delta = ((nib & 0x08UL) != 0UL) ? ((int32_t)nib - 16) : (int32_t)nib;
            int32_t cur_val = (int32_t)out81[idx - 1U] + delta;
            if (cur_val < 0) {
                cur_val = 0;
            } else if (cur_val > 255) {
                cur_val = 255;
            }
            out81[idx] = (uint8_t)cur_val;
        }
    }
    return true;
}
/* NOLINTEND(readability-magic-numbers) */

/* NOLINTNEXTLINE(bugprone-easily-swappable-parameters) */
void lens_db_get_opt_huff(const uint8_t **counts12, const uint8_t **syms, size_t *sym_count) {
    if (counts12 != NULL) {
        *counts12 = g_opt_huff_counts;
    }
    if (syms != NULL) {
        *syms = g_opt_huff_syms;
    }
    if (sym_count != NULL) {
        *sym_count = sizeof(g_opt_huff_syms);
    }
}

const tamron_z_optics_profile_t *lens_db_get_tamron_optics(int16_t tamron_z_idx) {
    if ((tamron_z_idx < 0) || (tamron_z_idx >= (int16_t)LENS_DB_TAMRON_Z_COUNT)) {
        return NULL;
    }
    return &g_tamron_z_optics_db[tamron_z_idx];
}

void sigma_150600_eval_zoom(uint16_t zoom_norm, sigma_150600_zoom_eval_t *out_eval) {
    if (out_eval == NULL) {
        return;
    }
    const sigma_150600_zoom_knot_t *knots = g_sigma_150600_zoom_knots;
    size_t count = (size_t)SIGMA_150600_ZOOM_KNOT_COUNT;
    size_t seg = 0U;
    if (zoom_norm >= knots[count - 1U].zoom_norm) {
        seg = count - 2U;
    } else {
        for (size_t idx = 0U; idx < (count - 1U); ++idx) {
            if ((zoom_norm >= knots[idx].zoom_norm) && (zoom_norm <= knots[idx + 1U].zoom_norm)) {
                seg = idx;
                break;
            }
        }
    }

    const sigma_150600_zoom_knot_t *k_lo = &knots[seg];
    const sigma_150600_zoom_knot_t *k_hi = &knots[seg + 1U];
    float span = (float)((int32_t)k_hi->zoom_norm - (int32_t)k_lo->zoom_norm);
    float frac = 0.0F;
    if (span > SPAN_GUARD_F) {
        frac = ((float)((int32_t)zoom_norm - (int32_t)k_lo->zoom_norm)) / span;
        if (frac < 0.0F) {
            frac = 0.0F;
        } else if (frac > 1.0F) {
            frac = 1.0F;
        }
    }

    float f_0p1 = (float)k_lo->f_nom_0p1mm +
                  (frac * (float)((int32_t)k_hi->f_nom_0p1mm - (int32_t)k_lo->f_nom_0p1mm));
    float p_op = (float)k_lo->p_open +
                 (frac * (float)((int32_t)k_hi->p_open - (int32_t)k_lo->p_open));
    float p_mn = (float)k_lo->p_min +
                 (frac * (float)((int32_t)k_hi->p_min - (int32_t)k_lo->p_min));
    float efar = (float)k_lo->e_far_mech +
                 (frac * (float)((int32_t)k_hi->e_far_mech - (int32_t)k_lo->e_far_mech));
    float einf = (float)k_lo->e_inf_true +
                 (frac * (float)((int32_t)k_hi->e_inf_true - (int32_t)k_lo->e_inf_true));
    float enear = (float)k_lo->e_near_true +
                  (frac * (float)((int32_t)k_hi->e_near_true - (int32_t)k_lo->e_near_true));
    float mfd = (float)k_lo->mfd_mm +
                (frac * (float)((int32_t)k_hi->mfd_mm - (int32_t)k_lo->mfd_mm));

    out_eval->focal_mm = f_0p1 * DECIMM_TO_MM;
    out_eval->p_open = (uint16_t)(p_op + HALF_F);
    out_eval->p_min = (uint16_t)(p_mn + HALF_F);
    out_eval->e_far_mech = (int32_t)((efar >= 0.0F) ? (efar + HALF_F) : (efar - HALF_F));
    out_eval->e_inf_true = (int32_t)((einf >= 0.0F) ? (einf + HALF_F) : (einf - HALF_F));
    out_eval->e_near_true = (int32_t)((enear >= 0.0F) ? (enear + HALF_F) : (enear - HALF_F));
    out_eval->mfd_meters = mfd * MM_TO_M_F;
    for (size_t cidx = 0U; cidx < (size_t)E_CURVE_NODES_COUNT; ++cidx) {
        float cval = (float)k_lo->cam_se_fracs[cidx] +
                     (frac * (float)((int32_t)k_hi->cam_se_fracs[cidx] -
                                     (int32_t)k_lo->cam_se_fracs[cidx]));
        out_eval->cam_se_fracs[cidx] =
            (int32_t)((cval >= 0.0F) ? (cval + HALF_F) : (cval - HALF_F));
    }
}

uint32_t sigma_150600_thermal_vmax_tps(int8_t temp_c) {
    for (size_t idx = 0U; idx < (size_t)SIGMA_150600_THERMAL_ROW_COUNT; ++idx) {
        if ((temp_c >= g_sigma_150600_thermal_rows[idx].min_temp_c) &&
            (temp_c <= g_sigma_150600_thermal_rows[idx].max_temp_c)) {
            return g_sigma_150600_thermal_rows[idx].encoder_ticks_per_sec;
        }
    }
    return g_sigma_150600_thermal_rows[0].encoder_ticks_per_sec;
}

const sigma_150600_optics_knot_t *sigma_150600_get_optics_knots(size_t *out_count) {
    if (out_count != NULL) {
        *out_count = (size_t)SIGMA_150600_OPTICS_KNOT_COUNT;
    }
    return g_sigma_150600_optics_knots;
}

bool lens_session_cap_has(const adapter_ctx_t *ctx, uint8_t cmd) {
    static const uint32_t BIT_MOD_7 = 7UL;
    if ((cmd == 0U) || (cmd > CAP_CMD_MAX)) {
        return false;
    }
    uint32_t bit_idx = (uint32_t)cmd - 1UL;
    uint32_t byte_idx = bit_idx >> 3U;
    uint32_t shift = bit_idx & BIT_MOD_7;
    return (((uint32_t)ctx->cap_bitmap[byte_idx] >> shift) & 1UL) != 0UL;
}

static void clear_photo_capability(adapter_ctx_t *ctx) {
    ctx->vr_id19_cap14 = 0U;
    ctx->vr_id19_cap_valid = 0U;
}

bool lens_session_enqueue_tx(adapter_ctx_t *ctx, uint8_t cmd,
                             const uint8_t *payload, uint16_t payload_len) {
    if (payload_len > (uint16_t)E_TX_CMD_MAX_PAYLOAD) {
        return false;
    }
    uint8_t next_tail = (uint8_t)((ctx->tx_q_tail + 1U) % (uint8_t)E_TX_QUEUE_CAP);
    if (next_tail == ctx->tx_q_head) {
        return false;
    }
    e_tx_cmd_t *slot = &ctx->tx_queue[ctx->tx_q_tail];
    slot->cmd = cmd;
    slot->frame_type = EMOUNT_FRAME_TYPE_B2L;
    slot->payload_len = payload_len;
    ctx->e_tx_serial += 1U;
    if (ctx->e_tx_serial == 0U) {
        ctx->e_tx_serial = 1U;
    }
    slot->serial = ctx->e_tx_serial;
    if ((payload != NULL) && (payload_len > 0U)) {
        memcpy(slot->payload, payload, payload_len);
    }
    ctx->tx_q_tail = next_tail;
    return true;
}

bool lens_session_pop_tx(adapter_ctx_t *ctx, e_tx_cmd_t *out_cmd) {
    if (ctx->tx_q_head == ctx->tx_q_tail) {
        return false;
    }
    *out_cmd = ctx->tx_queue[ctx->tx_q_head];
    ctx->tx_q_head = (uint8_t)((ctx->tx_q_head + 1U) % (uint8_t)E_TX_QUEUE_CAP);
    return true;
}

static void update_id05_expected_len(e_id05_state_t *state) {
    uint16_t sum = 0U;
    for (uint32_t grp = 0U; grp < (uint32_t)E_ID05_GROUP_COUNT; ++grp) {
        if (((state->active_mask >> grp) & 1UL) != 0UL) {
            sum = (uint16_t)(sum + GRP05_LEN[grp]);
        }
    }
    state->expected_payload_bytes = sum;
}

static void reset_session_defaults(adapter_ctx_t *ctx) {
    ctx->active_vsync_hz = VSYNC_60HZ;
    ctx->uart_baud = BAUD_750K;
    ctx->tamron_session_mode = 2U;
    ctx->cmd15_stage = 3U;
    ctx->cmd15_polls = 0U;
    ctx->cmd15_open_ticks = 0ULL;
    ctx->id05.dist_code_x = DIST_INF_SENTINEL;
    ctx->id05.mag_code = MAG_INVALID_SENTINEL;
    ctx->id05.barrel_ois_on = 1U;
    ctx->id06.temp_celsius = DEFAULT_ROOM_TEMP_C;
    ctx->dist_last_f01 = 0U;
    ctx->dist_last_zoom = 0U;
    ctx->id05_fresh_since_06 = 0U;
}

static const uint32_t CORE_DIAG_Q_MASK = (uint32_t)CORE_DIAG_Q_CAP - 1UL;

/* NOLINTNEXTLINE(bugprone-easily-swappable-parameters) */
void lens_session_push_diag(adapter_ctx_t *ctx, uint16_t code,
                            uint32_t arg0, uint32_t arg1, uint32_t arg2, uint32_t arg3) {
    uint8_t next_head = (uint8_t)(((uint32_t)ctx->core_diag_head + 1UL) & CORE_DIAG_Q_MASK);
    if (next_head == ctx->core_diag_tail) {
        ctx->core_diag_tail = (uint8_t)(((uint32_t)ctx->core_diag_tail + 1UL) & CORE_DIAG_Q_MASK);
    }
    bb_event_input_t *slot = &ctx->core_diag_q[ctx->core_diag_head];
    slot->event_code = code;
    slot->lens_id = ctx->e_lens_id;
    slot->arg0 = arg0;
    slot->arg1 = arg1;
    slot->arg2 = arg2;
    slot->arg3 = arg3;
    ctx->core_diag_head = next_head;
}

bool lens_session_pop_diag(adapter_ctx_t *ctx, bb_event_input_t *out_evt) {
    if (ctx->core_diag_tail == ctx->core_diag_head) {
        return false;
    }
    *out_evt = ctx->core_diag_q[ctx->core_diag_tail];
    ctx->core_diag_tail = (uint8_t)(((uint32_t)ctx->core_diag_tail + 1UL) & CORE_DIAG_Q_MASK);
    return true;
}

static void reset_mount_defaults(adapter_ctx_t *ctx, optics_workspace_t *optics,
                                 e_session_state_t target_state) {
    static const uint16_t DECIMM_PER_MM = 10U;
    uint32_t next_epoch = ctx->session_epoch + 1U;
    uint8_t saved_sck_pdown = ctx->z_sck_powerdown_latched;
    uint8_t saved_retrigger = ctx->home_retrigger_armed;
    uint8_t saved_cmd15_seen = ctx->cmd15_seen_since_arm;
    uint8_t saved_dq_head = ctx->core_diag_head;
    uint8_t saved_dq_tail = ctx->core_diag_tail;
    uint8_t saved_retram_warm = ctx->retram_warm_hit;
    uint16_t saved_first_ready_ms_x10 = ctx->first_ready_ms_x10;
    bb_event_input_t saved_dq[CORE_DIAG_Q_CAP];
    memcpy(saved_dq, ctx->core_diag_q, sizeof(saved_dq));
    memset(ctx, 0, sizeof(*ctx));
    ctx->z_sck_powerdown_latched = saved_sck_pdown;
    ctx->home_retrigger_armed = saved_retrigger;
    ctx->cmd15_seen_since_arm = saved_cmd15_seen;
    ctx->core_diag_head = saved_dq_head;
    ctx->core_diag_tail = saved_dq_tail;
    ctx->retram_warm_hit = saved_retram_warm;
    ctx->first_ready_ms_x10 = saved_first_ready_ms_x10;
    memcpy(ctx->core_diag_q, saved_dq, sizeof(saved_dq));
    optics->sony_ds13_len = 0U;
    optics->sony_ds13_lid = 0U;
    optics->sony_ds13_valid = 0U;
    ctx->session_epoch = next_epoch;
    ctx->session_state = target_state;
    ctx->hs_step = HANDSHAKE_STEP_READY;
    ctx->e_lens_id = 0x0000U;
    ctx->cached_db_lid = (uint16_t)MASK_16BIT;
    ctx->cached_db_idx = -1;
    ctx->dist_cam_bias_valid = 0U;
    ctx->dist_cam_bias_117 = 0;
    ctx->e_lens_ver = DEFAULT_EMPTY_VER;
    ctx->z_primary_lens_id = DEFAULT_Z_GENERIC_ID;
    ctx->brand = LENS_BRAND_GENERIC;
    ctx->af_class = AF_CLASS_C_STD_1D_3C;
    ctx->f_min_mm = DEFAULT_EMPTY_FOCAL_MM;
    ctx->f_max_mm = DEFAULT_EMPTY_FOCAL_MM;
    ctx->f_cur_mm = DEFAULT_EMPTY_FOCAL_MM;
    reset_session_defaults(ctx);

    static const char empty_name[] = "BORYOZA E-Z 50mm F1.4 Manual";
    memcpy(ctx->lens_name_ascii, empty_name, sizeof(empty_name));
    memcpy(ctx->vendor_tag_5b, "NIKON", VENDOR_TAG_BYTES);

    ctx->id05.f_nom_0p1mm = (uint16_t)(DEFAULT_EMPTY_FOCAL_MM * DECIMM_PER_MM);
    ctx->id05.f_opt_0p1mm = (uint16_t)(DEFAULT_EMPTY_FOCAL_MM * DECIMM_PER_MM);
    bool manual_target = (bool)((target_state == E_SESSION_EMPTY_MOUNT) ||
                                (target_state == E_SESSION_PASSIVE_MOUNT));
    ctx->id05.is_mf_switch = manual_target ? 1U : 0U;
    ctx->ap.c_open_wide = DEFAULT_EMPTY_AV_OPEN;
    ctx->ap.c_open_tele = DEFAULT_EMPTY_AV_OPEN;
    ctx->ap.c_open_cur = DEFAULT_EMPTY_AV_OPEN;
    ctx->ap.c_min_cur = DEFAULT_EMPTY_AV_MIN;
    ctx->ap.c_cur = DEFAULT_EMPTY_AV_OPEN;

    af_servo_configure_for_lens(ctx);
    vr_controller_configure_for_lens(ctx);
    aperture_controls_configure_for_lens(ctx);
    optics_build_initial_tables(ctx, optics);
    ctx->identity_ready = 1U;
    z_snapshot_publish(ctx, optics);
}

void lens_session_init(adapter_ctx_t *ctx, optics_workspace_t *optics) {
    memset(ctx, 0, sizeof(*ctx));
    memset(optics, 0, sizeof(*optics));
    reset_mount_defaults(ctx, optics, E_SESSION_DETACHED);
    ctx->session_epoch = 1U;
    ctx->hs_step = HANDSHAKE_STEP_NONE;
}

void lens_session_enter_empty_mount(adapter_ctx_t *ctx, optics_workspace_t *optics) {
    reset_mount_defaults(ctx, optics, E_SESSION_EMPTY_MOUNT);
}

void lens_session_enter_detached(adapter_ctx_t *ctx, optics_workspace_t *optics) {
    reset_mount_defaults(ctx, optics, E_SESSION_DETACHED);
}

void lens_session_enter_passive_mount(adapter_ctx_t *ctx, optics_workspace_t *optics) {
    reset_mount_defaults(ctx, optics, E_SESSION_PASSIVE_MOUNT);
}

/* NOLINTBEGIN(readability-magic-numbers) — ID05 18-group field indices & E-mount handshake opcodes */
static void decode_typed_curve_slots(e_id05_state_t *state) {
    if (state->grp_present[11] == 0U) {
        return;
    }
    for (uint32_t slot = 0U; slot < 2U; ++slot) {
        uint8_t selector = state->grp[11][slot];
        if ((selector < 21U) || (selector > 23U) ||
            (state->grp_present[12U + slot] == 0U)) {
            continue;
        }
        uint32_t index = (uint32_t)selector - 21UL;
        uint32_t bit = 1UL << index;
        uint8_t *curve = state->optical_curve9[index];
        memcpy(curve, state->grp[12U + slot], E_ID05_CURVE_CORE_BYTES);
        if (state->grp_present[16U + slot] != 0U) {
            memcpy(&curve[E_ID05_CURVE_CORE_BYTES], state->grp[16U + slot],
                   E_ID05_CURVE_TAIL_BYTES);
            state->optical_curve_full = (uint8_t)((uint32_t)state->optical_curve_full | bit);
        } else {
            memset(&curve[E_ID05_CURVE_CORE_BYTES], 0, E_ID05_CURVE_TAIL_BYTES);
            state->optical_curve_full = (uint8_t)((uint32_t)state->optical_curve_full & ~bit);
        }
        state->optical_curve_valid = (uint8_t)((uint32_t)state->optical_curve_valid | bit);
    }
    if (state->curve_frames_seen < 2U) {
        state->curve_frames_seen = (uint8_t)(state->curve_frames_seen + 1U);
    }
}

static void decode_id05_curves(e_id05_state_t *state) {
    if (state->grp_present[5] != 0U) {
        state->delta_av = (int16_t)read_u16_le(&state->grp[5][0]);
        memcpy(state->vig_core6, &state->grp[5][2], E_ID05_CURVE_CORE_BYTES);
        memcpy(state->dist_core6, &state->grp[5][8], E_ID05_CURVE_CORE_BYTES);
        state->aberration_valid = 1U;
    }
    if (state->grp_present[11] != 0U) {
        memcpy(state->lca_types, state->grp[11], E_ID05_CURVE_CORE_BYTES);
    }
    if (state->grp_present[12] != 0U) {
        memcpy(state->lca_ch1_core6, state->grp[12], E_ID05_CURVE_CORE_BYTES);
    }
    if (state->grp_present[13] != 0U) {
        memcpy(state->lca_ch2_core6, state->grp[13], E_ID05_CURVE_CORE_BYTES);
    }
    if (state->grp_present[15] != 0U) {
        memcpy(state->vig_tail3, &state->grp[15][0], E_ID05_CURVE_TAIL_BYTES);
        memcpy(state->dist_tail3, &state->grp[15][3], E_ID05_CURVE_TAIL_BYTES);
    } else {
        memset(state->vig_tail3, 0, E_ID05_CURVE_TAIL_BYTES);
        memset(state->dist_tail3, 0, E_ID05_CURVE_TAIL_BYTES);
    }
    if (state->grp_present[16] != 0U) {
        memcpy(state->lca_ch1_tail3, state->grp[16], E_ID05_CURVE_TAIL_BYTES);
    } else {
        memset(state->lca_ch1_tail3, 0, E_ID05_CURVE_TAIL_BYTES);
    }
    if (state->grp_present[17] != 0U) {
        memcpy(state->lca_ch2_tail3, state->grp[17], E_ID05_CURVE_TAIL_BYTES);
    } else {
        memset(state->lca_ch2_tail3, 0, E_ID05_CURVE_TAIL_BYTES);
    }
    decode_typed_curve_slots(state);
}

static void decode_id05_groups(e_id05_state_t *state, bool curves_changed) {
    if (state->grp_present[0] != 0U) {
        state->p_cur = read_u16_le(&state->grp[0][0]);
    }
    if (state->grp_present[2] != 0U) {
        state->exit_pupil_raw = read_u16_le(&state->grp[2][0]);
    }
    if (state->grp_present[3] != 0U) {
        state->p_ring = read_u16_le(&state->grp[3][0]);
        state->manual_ring_active = (uint8_t)((uint32_t)state->grp[3][2] & 1UL);
    }
    if (state->grp_present[4] != 0U) {
        state->dist_code_x = read_u16_le(&state->grp[4][0]);
        state->mag_code = state->grp[4][3];
        state->f_opt_0p1mm = read_u16_le(&state->grp[4][4]);
        state->f_nom_0p1mm = read_u16_le(&state->grp[4][6]);
    }
    if (state->grp_present[7] != 0U) {
        state->mf_ring_dir = (int8_t)state->grp[7][0];
        state->mf_ring_step = state->grp[7][1];
        state->is_mf_switch = (uint8_t)(((uint32_t)state->grp[7][2] >> 1U) & 1UL);
        state->l_fn_pressed = (uint8_t)(((uint32_t)state->grp[7][4] >> 3U) & 1UL);
        state->barrel_ois_on = (uint8_t)(((uint32_t)state->grp[7][4] >> 4U) & 1UL);
        state->barrel_ois_mode = (uint8_t)(((uint32_t)state->grp[7][4] >> 5U) & 3UL);
    }
    if (state->grp_present[9] != 0U) {
        state->ois_active_fb = (uint8_t)((uint32_t)state->grp[9][0] & 3UL);
    }
    if (state->optics_skip != 0U) {
        state->optics_curve_dirty = 0U;
    } else if (curves_changed) {
        decode_id05_curves(state);
    } else if ((state->curve_frames_seen < 2U) && (state->optical_curve_valid != 0U)) {
        state->optics_curve_dirty = 1U;
        decode_id05_curves(state);
    }
}

static bool id05_group_affects_optics(uint32_t group) {
    switch (group) {
        case 5U:
        case 11U:
        case 12U:
        case 13U:
        case 15U:
        case 16U:
        case 17U:
            return true;
        default:
            return false;
    }
}

static bool id05_optical_group_changed(const e_id05_state_t *state, uint32_t grp,
                                       const uint8_t *incoming, uint8_t glen) {
    if (state->grp_present[grp] == 0U) {
        return true;
    }
    for (uint32_t idx = 0U; idx < (uint32_t)glen; ++idx) {
        if (state->grp[grp][idx] != incoming[idx]) {
            return true;
        }
    }
    return false;
}

bool lens_session_unpack_id05(e_id05_state_t *state, const uint8_t *payload, size_t payload_len) {
    if ((state->active_mask == 0UL) || (payload_len < (size_t)state->expected_payload_bytes)) {
        return false;
    }

    size_t cursor = 0U;
    bool curves_changed = false;
    for (uint32_t grp = 0U; grp < (uint32_t)E_ID05_GROUP_COUNT; ++grp) {
        bool optical_group = id05_group_affects_optics(grp);
        if (((state->active_mask >> grp) & 1UL) == 0UL) {
            if ((state->optics_skip == 0U) && optical_group && (state->grp_present[grp] != 0U)) {
                state->optics_curve_dirty = 1U;
                curves_changed = true;
            }
            state->grp_present[grp] = 0U;
            continue;
        }
        uint8_t glen = GRP05_LEN[grp];
        if ((cursor + (size_t)glen) > payload_len) {
            return false;
        }
        if ((state->optics_skip != 0U) && optical_group) {
            if (grp == 5U) {
                state->delta_av = (int16_t)read_u16_le(&payload[cursor]);
                state->aberration_valid = 1U;
                state->grp_present[grp] = 1U;
            }
            cursor += glen;
            continue;
        }
        if (optical_group && id05_optical_group_changed(state, grp, &payload[cursor], glen)) {
            state->optics_curve_dirty = 1U;
            curves_changed = true;
        }
        memcpy(state->grp[grp], &payload[cursor], glen);
        state->grp_present[grp] = 1U;
        cursor += glen;
    }

    decode_id05_groups(state, curves_changed);
    return true;
}

static size_t id06_position_offset(const adapter_ctx_t *ctx, size_t payload_len) {
    if (payload_len < 22U) {
        return 2U;
    }
    if (ctx->e_lens_id == ZEISS_BATIS40_ID) {
        return 20U;
    }
    if ((ctx->lens_family == 1U) && (read_u16_le(&ctx->tx07_raw[2]) > 0x0110U)) {
        return 20U;
    }
    return 2U;
}

static void decode_id06_positions_and_limits(const adapter_ctx_t *ctx, e_id06_state_t *st6,
                                             const uint8_t *payload, size_t payload_len) {
    size_t pos_off = id06_position_offset(ctx, payload_len);
    if ((ctx->af_class == AF_CLASS_B_LEGACY_F1_F2) || (ctx->brand == LENS_BRAND_TAMRON)) {
        st6->focus_pos_se = (int32_t)read_u16_le(&payload[pos_off]);
        st6->lim_far_se = (int32_t)read_u16_le(&payload[7]);
        st6->lim_near_se = (int32_t)read_u16_le(&payload[9]);
    } else {
        st6->focus_pos_se = (int32_t)(int16_t)read_u16_le(&payload[pos_off]);
        st6->lim_far_se = (int32_t)(int16_t)read_u16_le(&payload[7]);
        st6->lim_near_se = (int32_t)(int16_t)read_u16_le(&payload[9]);
    }
    if ((ctx->lens_family == 2U) && (st6->hw_moving == 0U)) {
        if (payload[0] == 0x28U) {
            st6->lim_near_se = st6->focus_pos_se;
        } else if (payload[0] == 0x10U) {
            st6->lim_far_se = st6->focus_pos_se;
        }
    }
    if (st6->lim_near_se >= st6->lim_far_se) {
        st6->at_far_end = (st6->focus_pos_se <= (st6->lim_far_se + ID06_LIMIT_MARGIN_SE)) ? 1U : 0U;
        st6->at_near_end = (st6->focus_pos_se >= (st6->lim_near_se - ID06_LIMIT_MARGIN_SE)) ? 1U : 0U;
    } else {
        st6->at_far_end = (st6->focus_pos_se >= (st6->lim_far_se - ID06_LIMIT_MARGIN_SE)) ? 1U : 0U;
        st6->at_near_end = (st6->focus_pos_se <= (st6->lim_near_se + ID06_LIMIT_MARGIN_SE)) ? 1U : 0U;
    }
}

bool lens_session_unpack_id06(adapter_ctx_t *ctx, uint64_t now_ticks,
                              const uint8_t *payload, size_t payload_len) {
    if (payload_len < 13U) {
        return false;
    }
    e_id06_state_t *st6 = &ctx->id06;
    size_t copy_len = (payload_len <= (size_t)E_ID06_MAX_BYTES) ? payload_len : (size_t)E_ID06_MAX_BYTES;
    memcpy(st6->raw_bytes, payload, copy_len);
    st6->raw_len = (uint16_t)copy_len;

    uint8_t had_prev = st6->valid;
    st6->focus_prev_se = st6->focus_pos_se;
    st6->t_prev_ticks = st6->t_sample_ticks;
    st6->t_sample_ticks = now_ticks;

    st6->hw_moving = (uint8_t)((((uint32_t)payload[1] & 0x07UL) != 0UL) ? 1U : 0U);
    decode_id06_positions_and_limits(ctx, st6, payload, payload_len);
    if ((had_prev != 0U) && (st6->hw_moving != 0U)) {
        ctx->af_predicted_se = st6->focus_pos_se + (st6->focus_pos_se - st6->focus_prev_se);
    } else {
        ctx->af_predicted_se = st6->focus_pos_se;
    }
    st6->mf_ring_byte = ((payload[1] == 1U) || (payload[1] == 2U)) ? payload[1] : 0U;
    if ((ctx->af.af_drive_active != 0U) || (((uint32_t)payload[6] & 0x10UL) != 0UL)) {
        st6->mf_ring_byte = 0U;
    }
    st6->zoom_norm = read_u16_le(&payload[11]);
    if (payload_len >= 15U) {
        st6->defocus_k_se = read_u16_le(&payload[13]);
    }
    if (payload_len >= 16U) {
        st6->temp_celsius = (int8_t)payload[15];
    }
    st6->valid = 1U;
    return true;
}

static void send_hs_step(e_handshake_step_t step, adapter_ctx_t *ctx, uint64_t now_ticks) {
    ctx->hs_step = step;
    ctx->state_enter_ticks = now_ticks;

    switch (step) {
        case HANDSHAKE_STEP_CMD01: {
            uint8_t pay[E_CAP_BITMAP_BYTES];
            memset(pay, 0, sizeof(pay));
            memset(pay, 0xFF, 7U);
            pay[7] = 0x7FU;
            pay[31] = 0x80U;
            (void)lens_session_enqueue_tx(ctx, 0x01U, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD07: {
            static const uint8_t pay[] = {0x08U};
            (void)lens_session_enqueue_tx(ctx, 0x07U, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD0C: {
            /* 1.5 Mbps UART switch request */
            static const uint8_t pay[] = {0x02U};
            (void)lens_session_enqueue_tx(ctx, 0x0CU, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD3F: {
            static const uint8_t pay[] = {0x00U};
            (void)lens_session_enqueue_tx(ctx, 0x3FU, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD3D: {
            uint8_t pay[23] = {0xBDU, 0xFFU, 0xB1U, 0xF1U, 0x10U};
            (void)lens_session_enqueue_tx(ctx, 0x3DU, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD08: {
            static const uint8_t pay[] = {0xC6U, 0xE1U, 0x00U, 0x00U, 0x00U, 0x07U, 0x62U, 0x02U};
            (void)lens_session_enqueue_tx(ctx, 0x08U, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD09: {
            static const uint8_t pay[] = {0x00U, 0x00U, 0x00U, 0x00U};
            (void)lens_session_enqueue_tx(ctx, 0x09U, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD0B: {
            static const uint8_t pay[] = {0x60U, 0x00U};
            (void)lens_session_enqueue_tx(ctx, 0x0BU, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD0D: {
            uint8_t pay[1] = {0x00U};
            (void)lens_session_enqueue_tx(ctx, 0x0DU, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD10: {
            ctx->home_retry_stage = 0U;
            ctx->home_poll_ticks = now_ticks;
            uint8_t pay[1] = {(ctx->lens_family == 2U) ? CMD10_HOME_FAM2 : CMD10_HOME_STD};
            (void)lens_session_enqueue_tx(ctx, 0x10U, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD32: {
            static const uint8_t pay[7] = {0x80U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
            (void)lens_session_enqueue_tx(ctx, 0x32U, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD5A: {
            static const uint8_t CMD5A_BODY_MASK[E_CAP_BITMAP_BYTES] = {
                0xFFU, 0x9FU, 0x7FU, 0x5FU, 0x80U, 0x07U, 0x0EU, 0x00U,
                0x00U, 0x00U, 0x00U, 0x02U, 0x00U, 0x00U, 0x00U, 0x00U,
                0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
                0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            };
            uint8_t pay[55];
            memset(pay, 0, sizeof(pay));
            for (size_t idx = 0U; idx < (size_t)E_CAP_BITMAP_BYTES; ++idx) {
                uint8_t masked = (uint8_t)((uint32_t)ctx->cap_bitmap[idx] &
                                           (uint32_t)CMD5A_BODY_MASK[idx]);
                pay[idx] = masked;
            }
            pay[E_CAP_BITMAP_BYTES] = 0x1FU;
            (void)lens_session_enqueue_tx(ctx, 0x5AU, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD3A_DS13: {
            uint8_t pay[23] = {0x0DU};
            (void)lens_session_enqueue_tx(ctx, 0x3AU, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD3B_DS13:
        case HANDSHAKE_STEP_CMD3B_LF: {
            uint32_t pnum = ctx->lf_next_page;
            uint8_t pay[7] = {
                0x00U,
                (uint8_t)(pnum & BYTE_MASK),
                (uint8_t)((pnum >> SHIFT_8) & BYTE_MASK),
                0U, 0U, 0U, 0U,
            };
            (void)lens_session_enqueue_tx(ctx, 0x3BU, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD3A_LF: {
            uint8_t pay[23] = {0x00U};
            (void)lens_session_enqueue_tx(ctx, 0x3AU, pay, (uint16_t)sizeof(pay));
            break;
        }
        case HANDSHAKE_STEP_CMD0A: {
            uint8_t pay[16] = {
                0xFFU, 0xFFU, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
                0x3FU, (ctx->has_ext_5a != 0U) ? 0x01U : 0x00U,
                0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            };
            ctx->id05.active_mask = ID05_FULL_18GRP_MASK;
            update_id05_expected_len(&ctx->id05);
            ctx->id06.active_mask = ID06_STD_6GRP_MASK;
            (void)lens_session_enqueue_tx(ctx, 0x0AU, pay, (uint16_t)sizeof(pay));
            break;
        }
        default:
            break;
    }
}

static e_handshake_step_t pick_next_hs_step(const adapter_ctx_t *ctx,
                                            const optics_workspace_t *optics,
                                            e_handshake_step_t completed) {
    static const struct {
        e_handshake_step_t step;
        uint8_t            cmd;
    } SIMPLE_STEPS[] = {
        {HANDSHAKE_STEP_CMD07, 0x07U},
        {HANDSHAKE_STEP_CMD0C, 0x0CU},
        {HANDSHAKE_STEP_CMD0B, 0x0BU},
        {HANDSHAKE_STEP_CMD3F, 0x3FU},
        {HANDSHAKE_STEP_CMD3D, 0x3DU},
        {HANDSHAKE_STEP_CMD32, 0x32U},
        {HANDSHAKE_STEP_CMD5A, 0x5AU},
        {HANDSHAKE_STEP_CMD08, 0x08U},
        {HANDSHAKE_STEP_CMD09, 0x09U},
        {HANDSHAKE_STEP_CMD0D, 0x0DU},
        {HANDSHAKE_STEP_CMD10, 0x10U},
    };

    bool ds13_cached = (bool)((optics != NULL) && (optics->sony_ds13_valid != 0U) &&
                              (optics->sony_ds13_lid == ctx->e_lens_id) &&
                              (ctx->e_lens_id != 0U));

    for (size_t idx = 0U; idx < (sizeof(SIMPLE_STEPS) / sizeof(SIMPLE_STEPS[0])); ++idx) {
        e_handshake_step_t cand = SIMPLE_STEPS[idx].step;
        if (completed >= cand) {
            continue;
        }
        if (((cand == HANDSHAKE_STEP_CMD32) || (cand == HANDSHAKE_STEP_CMD5A)) &&
            (ctx->has_ext_5a == 0U)) {
            continue;
        }
        if ((cand >= HANDSHAKE_STEP_CMD32) &&
            (completed < HANDSHAKE_STEP_CMD3A_DS13) &&
            (!ds13_cached) &&
            lens_session_cap_has(ctx, 0x3AU) && lens_session_cap_has(ctx, 0x3BU) &&
            (((uint32_t)ctx->divided_info_bits & DS13_BIT_MASK) != 0UL)) {
            return HANDSHAKE_STEP_CMD3A_DS13;
        }
        if (lens_session_cap_has(ctx, SIMPLE_STEPS[idx].cmd)) {
            return cand;
        }
    }

    bool has_3a3b = false;
    if (lens_session_cap_has(ctx, 0x3AU) && lens_session_cap_has(ctx, 0x3BU)) {
        has_3a3b = true;
    }
    if ((completed < HANDSHAKE_STEP_CMD3A_LF) && has_3a3b &&
        (((uint32_t)ctx->divided_info_bits & LF_DS0_BIT_MASK) != 0UL) &&
        (lens_db_lookup(ctx->e_lens_id) == NULL)) {
        return HANDSHAKE_STEP_CMD3A_LF;
    }
    if ((completed < HANDSHAKE_STEP_CMD0A) && lens_session_cap_has(ctx, 0x0AU)) {
        return HANDSHAKE_STEP_CMD0A;
    }
    return HANDSHAKE_STEP_WAIT_05_06;
}

static void advance_handshake_after(e_handshake_step_t completed, adapter_ctx_t *ctx,
                                    const optics_workspace_t *optics, uint64_t now_ticks) {
    e_handshake_step_t next_step = pick_next_hs_step(ctx, optics, completed);
    if (next_step == HANDSHAKE_STEP_WAIT_05_06) {
        ctx->hs_step = HANDSHAKE_STEP_WAIT_05_06;
        ctx->state_enter_ticks = now_ticks;
    } else {
        send_hs_step(next_step, ctx, now_ticks);
    }
}

static void apply_attached_lens_catalog(adapter_ctx_t *ctx) {
    const lens_db_entry_t *entry = lens_db_lookup_cached(ctx);
    if (entry != NULL) {
        ctx->z_primary_lens_id = entry->z_lens_id;
        ctx->brand = (lens_brand_t)entry->brand;
        ctx->f_min_mm = entry->f_min_mm;
        ctx->f_max_mm = entry->f_max_mm;
        ctx->f_cur_mm = entry->f_min_mm;
        ctx->ap.c_open_wide = entry->av_open_wide_c;
        ctx->ap.c_open_tele = entry->av_open_tele_c;
        ctx->ap.c_open_cur = entry->av_open_wide_c;
        ctx->ap.c_min_cur = DEFAULT_F22_AV_MIN;
        ctx->ap.c_cur = entry->av_open_wide_c;
        ctx->is_apsc = (uint8_t)(((uint32_t)entry->flags & LENS_FLAG_APSC) != 0UL ? 1U : 0U);
        ctx->is_macro = (uint8_t)(((uint32_t)entry->flags & LENS_FLAG_MACRO) != 0UL ? 1U : 0U);
        ctx->is_fisheye = (uint8_t)(((uint32_t)entry->flags & LENS_FLAG_FISHEYE) != 0UL ? 1U : 0U);
        ctx->is_dual_floating = (uint8_t)(((uint32_t)entry->flags & LENS_FLAG_DUAL_FLOATING) != 0UL ? 1U : 0U);
        ctx->supports_120hz = (uint8_t)(((uint32_t)entry->flags & LENS_FLAG_120HZ) != 0UL ? 1U : 0U);
        ctx->vr.lens_has_ois = (uint8_t)(((uint32_t)entry->flags & LENS_FLAG_OIS) != 0UL ? 1U : 0U);
        ctx->vr.lens_has_ois_switch = (uint8_t)(((uint32_t)entry->flags & LENS_FLAG_OIS_SWITCH) != 0UL ? 1U : 0U);
        memset(ctx->lens_name_ascii, 0, sizeof(ctx->lens_name_ascii));
        const char *ename = lens_db_entry_name(entry);
        size_t nlen = strlen(ename);
        if (nlen >= sizeof(ctx->lens_name_ascii)) {
            nlen = sizeof(ctx->lens_name_ascii) - 1U;
        }
        memcpy(ctx->lens_name_ascii, ename, nlen);
        memcpy(ctx->vendor_tag_5b, lens_db_vendor_tag5(entry->brand), VENDOR_TAG_BYTES);
    } else {
        ctx->z_primary_lens_id = (uint16_t)(DEFAULT_Z_ACTIVE_BASE_ID |
                                            ((uint32_t)ctx->e_lens_id & 0x03FFUL));
        if ((ctx->e_lens_id >= SONY_LENS_ID_MIN) && (ctx->e_lens_id <= SONY_LENS_ID_MAX)) {
            ctx->brand = LENS_BRAND_SONY;
            memcpy(ctx->vendor_tag_5b, "SONY\0", VENDOR_TAG_BYTES);
        } else {
            ctx->brand = LENS_BRAND_GENERIC;
            memcpy(ctx->vendor_tag_5b, "NIKON", VENDOR_TAG_BYTES);
        }
        ctx->f_min_mm = DEFAULT_EMPTY_FOCAL_MM;
        ctx->f_max_mm = DEFAULT_EMPTY_FOCAL_MM;
        ctx->f_cur_mm = DEFAULT_EMPTY_FOCAL_MM;
        ctx->ap.c_open_wide = DEFAULT_EMPTY_AV_OPEN;
        ctx->ap.c_open_tele = DEFAULT_EMPTY_AV_OPEN;
        ctx->ap.c_open_cur = DEFAULT_EMPTY_AV_OPEN;
        ctx->ap.c_min_cur = DEFAULT_EMPTY_AV_MIN;
        ctx->ap.c_cur = DEFAULT_EMPTY_AV_OPEN;
        ctx->is_apsc = 0U;
        ctx->is_macro = 0U;
        ctx->is_fisheye = 0U;
        ctx->is_dual_floating = 0U;
        ctx->supports_120hz = 0U;
        ctx->vr.lens_has_ois = 0U;
        ctx->vr.lens_has_ois_switch = 0U;
        static const char gen_name[] = "E-Mount AF Lens";
        memcpy(ctx->lens_name_ascii, gen_name, sizeof(gen_name));
    }

    if ((ctx->has_ext_5a == 0U) &&
        lens_session_cap_has(ctx, 0x41U) && lens_session_cap_has(ctx, 0x43U) &&
        lens_session_cap_has(ctx, 0x44U) && lens_session_cap_has(ctx, 0x49U)) {
        ctx->af_class = AF_CLASS_A_SONY_43_49;
    } else {
        ctx->af_class = AF_CLASS_C_STD_1D_3C;
    }
}

static void enqueue_active_extensions(adapter_ctx_t *ctx) {
    if ((ctx->af.linear_mf_code != 0U) && lens_session_cap_has(ctx, 0x22U)) {
        uint8_t pay22[2] = {0x01U, ctx->af.linear_mf_code};
        (void)lens_session_enqueue_tx(ctx, 0x22U, pay22, (uint16_t)sizeof(pay22));
    }
    if (ctx->brand != LENS_BRAND_SIGMA) {
        return;
    }
    if (lens_session_cap_has(ctx, 0x1FU)) {
        uint8_t pay1f[13] = {0x08U};
        ctx->af.id1f_param_requested = 1U;
        (void)lens_session_enqueue_tx(ctx, 0x1FU, pay1f, (uint16_t)sizeof(pay1f));
    }
    if (lens_session_cap_has(ctx, E_CMD_APERTURE_STATUS)) {
        (void)lens_session_enqueue_tx(ctx, E_CMD_APERTURE_STATUS, NULL, 0U);
    }
}

static void log_hs_identity_bb(adapter_ctx_t *ctx) {
    if ((ctx->bb_hs_id_logged == 0U) && (ctx->identity_ready != 0U) && (ctx->e_lens_id != 0U)) {
        ctx->bb_hs_id_logged = 1U;
        uint32_t arg0 = ((uint32_t)ctx->z_primary_lens_id) |
                        ((uint32_t)ctx->lens_family << 16U) |
                        ((uint32_t)ctx->af_class << 24U);
        uint32_t arg1 = ((uint32_t)ctx->f_min_mm) | ((uint32_t)ctx->f_max_mm << 16U);
        uint32_t arg2 = ((uint32_t)ctx->optics_ready) |
                        ((uint32_t)ctx->hs_tx08_done << 8U) |
                        ((ctx->uart_baud / 1000UL) << 16U);
        uint32_t arg3 = ((uint32_t)ctx->is_apsc) |
                        ((uint32_t)ctx->is_macro << 8U) |
                        ((uint32_t)ctx->is_fisheye << 16U) |
                        ((uint32_t)ctx->vr.lens_has_ois << 24U);
        lens_session_push_diag(ctx, BB_EVT_HS_IDENTITY, arg0, arg1, arg2, arg3);
    }
}

static void finalize_active_session(adapter_ctx_t *ctx, optics_workspace_t *optics) {
    if ((ctx->hs_cmd07_ok == 0U) || (ctx->e_lens_id == 0U)) {
        return;
    }
    if (ctx->id05.active_mask == 0UL) {
        ctx->id05.active_mask = ID05_STD_15GRP_MASK;
        update_id05_expected_len(&ctx->id05);
        ctx->id06.active_mask = ID06_STD_6GRP_MASK;
    }
    bool allow_120 = false;
    if ((ctx->uart_baud >= BAUD_1500K) &&
        (ctx->brand == LENS_BRAND_SIGMA) && (ctx->supports_120hz != 0U)) {
        allow_120 = true;
    }
    ctx->active_vsync_hz = allow_120 ? VSYNC_120HZ : VSYNC_60HZ;
    ctx->init_sync_rem = (ctx->lens_family == 2U) ? 0U : 2U;
    ctx->wake_rehandshake = 0U;
    ctx->wake_boot_prologue_seen = 0U;

    if (!lens_session_cap_has(ctx, E_CMD_IRIS_HOME_10)) {
        ctx->home_completed_ok = 1U;
        ctx->home_retrigger_armed = 0U;
        ctx->cmd15_seen_since_arm = 0U;
        if ((ctx->vr.lens_has_ois != 0U) && (ctx->vr_home_epoch != ctx->session_epoch)) {
            vr_controller_on_initialization_result(ctx, true);
        }
    }

    af_servo_configure_for_lens(ctx);
    vr_controller_configure_for_lens(ctx);
    aperture_controls_configure_for_lens(ctx);
    optics_build_initial_tables(ctx, optics);
    enqueue_active_extensions(ctx);

    ctx->id05_fresh_since_06 = (uint8_t)ID05_PAIR_EARLY_05;
    af_servo_update_telemetry(ctx, ctx->id06.t_sample_ticks);
    ctx->id05_fresh_since_06 = (uint8_t)ID05_PAIR_IDLE;
    aperture_controls_validate_distance(ctx);
    aperture_controls_update_from_id05(ctx, optics);
    af_servo_refresh_kz(ctx);
    vr_controller_refresh_state(ctx);
    optics_update_from_live_id05(ctx, optics);
    ctx->identity_ready = 1U;
    log_hs_identity_bb(ctx);
    z_snapshot_publish(ctx, optics);

    ctx->hs_step = HANDSHAKE_STEP_READY;
    ctx->session_state = E_SESSION_ACTIVE;
}

static bool native_home_pending(const adapter_ctx_t *ctx) {
    if ((ctx->brand == LENS_BRAND_SIGMA) &&
        ((ctx->hs_step == HANDSHAKE_STEP_CMD10) || (ctx->hs_step == HANDSHAKE_STEP_WAIT_HOME))) {
        return true;
    }
    return false;
}

void lens_session_force_active(adapter_ctx_t *ctx, optics_workspace_t *optics) {
    finalize_active_session(ctx, optics);
}

void lens_session_arm_home_retrigger(adapter_ctx_t *ctx) {
    ctx->home_retrigger_armed = HOME_RETRIGGER_BUDGET;
    ctx->cmd15_seen_since_arm = 0U;
}

bool lens_session_retrigger_homing(adapter_ctx_t *ctx, uint64_t now_ticks) {
    if ((ctx->home_retrigger_armed == 0U) && (ctx->z_sck_powerdown_latched != 0U)) {
        lens_session_arm_home_retrigger(ctx);
    }
    ctx->z_sck_powerdown_latched = 0U;
    if ((ctx->session_state == E_SESSION_POWERUP_WAIT) ||
        (ctx->session_state == E_SESSION_PARKED)) {
        return false;
    }
    if ((ctx->home_completed_ok != 0U) || (ctx->home_retrigger_armed == 0U)) {
        ctx->home_retrigger_armed = 0U;
        ctx->cmd15_seen_since_arm = 0U;
        return false;
    }
    if ((ctx->session_state == E_SESSION_HANDSHAKE) &&
        ((ctx->hs_step == HANDSHAKE_STEP_CMD10) || (ctx->hs_step == HANDSHAKE_STEP_WAIT_HOME))) {
        if (ctx->cmd15_seen_since_arm != 0U) {
            return false;
        }
        ctx->cmd15_seen_since_arm = 1U;
        ctx->home_retrigger_armed = (uint8_t)(ctx->home_retrigger_armed - 1U);
        ctx->tx_q_head = 0U;
        ctx->tx_q_tail = 0U;
        ctx->hs_start_ticks = now_ticks;
        send_hs_step(HANDSHAKE_STEP_CMD10, ctx, now_ticks);
        return true;
    }
    if ((ctx->session_state == E_SESSION_ACTIVE) &&
        lens_session_cap_has(ctx, E_CMD_IRIS_HOME_10)) {
        ctx->cmd15_seen_since_arm = 1U;
        ctx->home_retrigger_armed = (uint8_t)(ctx->home_retrigger_armed - 1U);
        ctx->tx_q_head = 0U;
        ctx->tx_q_tail = 0U;
        ctx->id05.grp_present[0] = 0U;
        ctx->id05.grp_present[7] = 0U;
        ctx->id05.is_mf_switch = 0U;
        ctx->id06.valid = 0U;
        ctx->hs_start_ticks = now_ticks;
        ctx->session_state = E_SESSION_HANDSHAKE;
        send_hs_step(HANDSHAKE_STEP_CMD10, ctx, now_ticks);
        return true;
    }
    return false;
}

bool lens_session_initial_probe_exhausted(const adapter_ctx_t *ctx, uint64_t now_ticks) {
    uint8_t max_retries = (ctx->wake_rehandshake != 0U) ? WAKE_CMD01_RETRIES : MAX_CMD01_RETRIES;
    if ((ctx->hs_step == HANDSHAKE_STEP_CMD01) && (ctx->wake_ready != 0U) &&
        (ctx->cmd01_retries >= max_retries) && (now_ticks >= ctx->state_enter_ticks) &&
        ((now_ticks - ctx->state_enter_ticks) >= CMD01_RETRY_TICKS)) {
        return true;
    }
    return false;
}

static void reset_wake_parfocal_state(adapter_ctx_t *ctx) {
    ctx->parfocal_hold_vsyncs = 0U;
    ctx->parfocal_cmd_valid = 0U;
    ctx->parfocal_cl_trim_cz = 0;
    ctx->parfocal_anchor_dist_x = 0U;
    ctx->parfocal_cl_cooldown = 0U;
    ctx->parfocal_cl_steps = 0U;
    ctx->parfocal_anchor_f01 = 0U;
    ctx->parfocal_trim_f01 = 0U;
    ctx->parfocal_min_rem_se = (uint16_t)MASK_16BIT;
    ctx->parfocal_anchor_refresh = 0U;
    ctx->parfocal_e0_coast_rem = 0U;
    ctx->id05_fresh_since_06 = (uint8_t)ID05_PAIR_IDLE;
    ctx->af_ext.parfocal_anchor_valid = 0U;
    ctx->af_ext.parfocal_wait_vsyncs = 0U;
    ctx->af_ext.parfocal_trim_pending = 0U;
}

void lens_session_start_mount_handshake(adapter_ctx_t *ctx, optics_workspace_t *optics, uint64_t now_ticks) {
    clear_photo_capability(ctx);
    ctx->vr_cmd3d_negotiated = 0U;
    if ((ctx->wake_rehandshake != 0U) && (ctx->identity_ready != 0U) && (ctx->e_lens_id != 0U)) {
        reset_wake_parfocal_state(ctx);
        ctx->tx_q_head = 0U;
        ctx->tx_q_tail = 0U;
        ctx->cmd01_retries = 0U;
        ctx->wake_ready = 0U;
        ctx->wake_retry_req = 0U;
        ctx->hs_cmd07_ok = 0U;
        ctx->hs_tx08_done = 0U;
        ctx->home_completed_ok = 0U;
        ctx->bb_hs_id_logged = 0U;
        ctx->bb_e_err_logged = 0U;
        ctx->bb_z_err_logged = 0U;
        ctx->bb_stall_logged = 0U;
        ctx->id05.grp_present[0] = 0U;
        ctx->id05.grp_present[7] = 0U;
        ctx->id05.is_mf_switch = 0U;
        ctx->id06.valid = 0U;
        ctx->uart_baud = BAUD_750K;
        ctx->active_vsync_hz = VSYNC_60HZ;
        ctx->tamron_session_mode = 2U;
        ctx->cmd24_read_mask = 0U;
        ctx->id05.optics_skip = 0U;
        ctx->optics_built_epoch = 0U;
        ctx->session_state = E_SESSION_HANDSHAKE;
        ctx->hs_start_ticks = now_ticks;
        send_hs_step(HANDSHAKE_STEP_CMD01, ctx, now_ticks);
        return;
    }
    uint16_t saved_ds13_len = optics->sony_ds13_len;
    uint16_t saved_ds13_lid = optics->sony_ds13_lid;
    uint8_t saved_ds13_valid = optics->sony_ds13_valid;
    reset_mount_defaults(ctx, optics, E_SESSION_HANDSHAKE);
    ctx->af_configured_lid = 0U;
    optics->sony_ds13_len = saved_ds13_len;
    optics->sony_ds13_lid = saved_ds13_lid;
    optics->sony_ds13_valid = saved_ds13_valid;
    ctx->session_state = E_SESSION_HANDSHAKE;
    ctx->hs_start_ticks = now_ticks;
    ctx->identity_ready = 0U;
    ctx->optics_ready = 0U;
    ctx->optics_disconnected = 0U;
    ctx->cmd24_read_mask = 0U;
    ctx->id05.optics_skip = 0U;
    ctx->optics_built_epoch = 0U;
    ctx->hs_cmd07_ok = 0U;
    ctx->hs_tx08_done = 0U;
    ctx->z_primary_lens_id = DEFAULT_Z_PROVISIONAL_ID;
    optics_build_initial_tables(ctx, optics);
    z_snapshot_publish(ctx, optics);
    send_hs_step(HANDSHAKE_STEP_CMD01, ctx, now_ticks);
}

static void handle_3a_3b_reply(adapter_ctx_t *ctx, optics_workspace_t *optics,
                               uint8_t cmd, const uint8_t *payload,
                               size_t payload_len, const uint64_t *now_ticks_ptr) {
    uint64_t now_ticks = *now_ticks_ptr;
    if (cmd == 0x3AU) {
        if (ctx->hs_step == HANDSHAKE_STEP_CMD3A_DS13) {
            optics->sony_ds13_len = 0U;
            optics->sony_ds13_lid = 0U;
            optics->sony_ds13_valid = 0U;
        }
        if ((payload_len >= 7U) && (payload[0] == 0U)) {
            ctx->lf_total_bytes = read_u32_le(&payload[1]);
            ctx->lf_total_pages = read_u16_le(&payload[5]);
            ctx->lf_next_page = 1U;
            if (ctx->lf_total_pages > 0U) {
                e_handshake_step_t pstep = (ctx->hs_step == HANDSHAKE_STEP_CMD3A_DS13)
                                               ? HANDSHAKE_STEP_CMD3B_DS13
                                               : HANDSHAKE_STEP_CMD3B_LF;
                send_hs_step(pstep, ctx, now_ticks);
                return;
            }
        }
        advance_handshake_after(ctx->hs_step, ctx, optics, now_ticks);
        return;
    }

    if (payload_len >= 3U) {
        uint16_t rx_page = read_u16_le(&payload[1]);
        const uint8_t *pdata = &payload[3];
        size_t plen = payload_len - 3U;
        if (ctx->hs_step == HANDSHAKE_STEP_CMD3B_DS13) {
            size_t cur_off = (size_t)optics->sony_ds13_len;
            if ((cur_off + plen) <= (size_t)E_SONY_DS13_MAX_BYTES) {
                memcpy(&optics->sony_ds13_buf[cur_off], pdata, plen);
                optics->sony_ds13_len = (uint16_t)(cur_off + plen);
                optics->sony_ds13_lid = ctx->e_lens_id;
                optics->sony_ds13_valid = 1U;
            }
        } else {
            optics_ingest_lf_page(ctx, optics, rx_page, pdata, plen);
        }
        if (ctx->lf_next_page < ctx->lf_total_pages) {
            ctx->lf_next_page = (uint16_t)(ctx->lf_next_page + 1U);
            send_hs_step(ctx->hs_step, ctx, now_ticks);
            return;
        }
    }
    e_handshake_step_t done = (ctx->hs_step == HANDSHAKE_STEP_CMD3B_DS13)
                                  ? HANDSHAKE_STEP_CMD3A_DS13
                                  : HANDSHAKE_STEP_CMD3A_LF;
#if !defined(__STDC_HOSTED__) || (__STDC_HOSTED__ == 0)
    if ((done == HANDSHAKE_STEP_CMD3A_DS13) && (optics->sony_ds13_valid != 0U) &&
        (optics->sony_ds13_len <= (uint16_t)RETRAM_DS13_CACHE_CAP)) {
        retram_warm_cache_store_ds13(ctx->e_lens_id, optics->sony_ds13_buf, optics->sony_ds13_len);
    }
#endif
    advance_handshake_after(done, ctx, optics, now_ticks);
}

static uint16_t decode_bcd4_be(const uint8_t *bcd2) {
    uint32_t dig0 = ((uint32_t)bcd2[0] >> 4U) & 0x0FUL;
    uint32_t dig1 = (uint32_t)bcd2[0] & 0x0FUL;
    uint32_t dig2 = ((uint32_t)bcd2[1] >> 4U) & 0x0FUL;
    uint32_t dig3 = (uint32_t)bcd2[1] & 0x0FUL;
    if ((dig0 > 9UL) || (dig1 > 9UL) || (dig2 > 9UL) || (dig3 > 9UL)) {
        return 0U;
    }
    return (uint16_t)((dig0 * 1000UL) + (dig1 * 100UL) + (dig2 * 10UL) + dig3);
}

static void parse_tx07_reply(adapter_ctx_t *ctx, const uint8_t *payload, size_t payload_len) {
    clear_photo_capability(ctx);
    size_t clen = (payload_len <= E_TX07_PAYLOAD_BYTES) ? payload_len : E_TX07_PAYLOAD_BYTES;
    memcpy(ctx->tx07_raw, payload, clen);
    if (payload_len >= 11U) {
        ctx->hs_cmd07_ok = 1U;
        ctx->lens_family = payload[0];
        ctx->e_lens_id = read_u16_le(&payload[9]);
        ctx->cached_db_lid = 0xFFFFU;
        ctx->cached_db_idx = -1;
        ctx->e_lens_ver = payload[6];
        bool fam1_32b = false;
        if ((payload[0] == 1U) && (((uint32_t)payload[5] & 0x02UL) == 0UL)) {
            fam1_32b = true;
        }
        ctx->id03_is_32b = (fam1_32b || (payload[5] == 1U)) ? 1U : 0U;
        ctx->has_ext_5a = (((payload[0] == 2U) && lens_session_cap_has(ctx, 0x5AU)) ||
                           (ctx->e_lens_id == SONY_LAEA5_ID) ||
                           (!lens_session_cap_has(ctx, 0x3DU) && lens_session_cap_has(ctx, 0x5AU)))
                              ? 1U : 0U;
        const lens_db_entry_t *entry = lens_db_lookup_cached(ctx);
        if (entry != NULL) {
            bool legacy_frame = (((uint32_t)entry->flags & LENS_FLAG_ID03_32B) != 0UL);
            if ((entry->brand == LENS_BRAND_SIGMA) || (entry->brand == LENS_BRAND_TAMRON) || legacy_frame) {
                ctx->id03_is_32b = legacy_frame ? 1U : 0U;
            }
        }
        apply_attached_lens_catalog(ctx);
    }
}

static void parse_tx3d_reply(adapter_ctx_t *ctx, const uint8_t *payload, size_t payload_len) {
    if (ctx->hs_step != HANDSHAKE_STEP_CMD3D) {
        return;
    }
    clear_photo_capability(ctx);
    if (payload_len >= 4U) {
        ctx->divided_info_bits = read_u16_le(&payload[0]);
    }
    uint8_t record_class = (uint8_t)((uint32_t)ctx->lens_family & TX07_CLASS_MASK);
    if ((payload_len >= TX3D_CAPABILITY_BYTES) && (ctx->hs_cmd07_ok != 0U) &&
        (ctx->vr_cmd3d_negotiated != 0U) &&
        ((record_class == 1U) || (record_class == 2U))) {
        ctx->vr_id19_cap_valid = 1U;
        ctx->vr_id19_cap14 =
            ((record_class == 1U) && (((uint32_t)payload[TX3D_CAP14_OFFSET] & TX3D_CAP14_MASK) != 0UL))
                ? 1U : 0U;
    }
}

static bool try_extract_cmd3f_lens_name(adapter_ctx_t *ctx, const uint8_t *payload,
                                        size_t payload_len) {
    if ((payload_len < 4U) || (lens_db_lookup_cached(ctx) != NULL)) {
        return false;
    }
    size_t start = ((payload[0] == 0x00U) && (payload_len > 4U)) ? 1U : 0U;
    size_t count = 0U;
    while (((start + count) < payload_len) &&
           (count < (sizeof(ctx->lens_name_ascii) - 1U))) {
        uint8_t cur_ch = payload[start + count];
        if ((cur_ch < 0x20U) || (cur_ch > 0x7EU)) {
            break;
        }
        count += 1U;
    }
    if (count >= 4U) {
        memset(ctx->lens_name_ascii, 0, sizeof(ctx->lens_name_ascii));
        memcpy(ctx->lens_name_ascii, &payload[start], count);
        return true;
    }
    return false;
}

static void parse_tx08_reply(adapter_ctx_t *ctx, const uint8_t *payload, size_t payload_len) {
    size_t clen = (payload_len <= E_TX08_PAYLOAD_BYTES) ? payload_len : E_TX08_PAYLOAD_BYTES;
    memcpy(ctx->tx08_raw, payload, clen);
    ctx->hs_tx08_done = 1U;
    if (payload_len >= 4U) {
        uint16_t p_open = read_u16_le(&payload[0]);
        uint16_t p_min = read_u16_le(&payload[2]);
        if (p_open >= 0x0E00U) {
            ctx->ap.raw_p_open_08 = p_open;
            ctx->id05.p_open = p_open;
        }
        if (p_min > p_open) {
            ctx->ap.raw_p_min_08 = p_min;
            ctx->id05.p_min = p_min;
        }
    }
    if (payload_len <= TX08_FISHEYE_OFFSET) {
        return;
    }
    if (ctx->e_lens_id == SONY_50150_ID) {
        ctx->vr.lens_has_ois = 0U;
        ctx->vr.lens_has_ois_switch = 0U;
    } else if (payload[TX08_OIS_OFFSET] != 0U) {
        ctx->vr.lens_has_ois = 1U;
    }
    if (ctx->brand == LENS_BRAND_SIGMA) {
        ctx->supports_120hz =
            (((uint32_t)payload[TX08_120HZ_OFFSET] & TX08_120HZ_BIT) != 0UL) ? 1U : 0U;
    }
    if ((lens_db_lookup_cached(ctx) == NULL) &&
        (((uint32_t)payload[TX08_FISHEYE_OFFSET] & TX08_FISHEYE_BIT) != 0UL)) {
        ctx->is_fisheye = 1U;
    }

    /* Populate optical metadata directly from TX08 for future uncataloged lenses */
    if (lens_db_lookup_cached(ctx) == NULL) {
        ctx->is_apsc = (uint8_t)((uint32_t)payload[TX08_APSC_OFFSET] & 1UL);
        uint16_t f_min = decode_bcd4_be(&payload[TX08_FMIN_BCD_OFFSET]);
        uint16_t f_max = decode_bcd4_be(&payload[TX08_FMAX_BCD_OFFSET]);
        if (f_min >= 4U) {
            ctx->f_min_mm = f_min;
            ctx->f_max_mm = (f_max >= f_min) ? f_max : f_min;
            ctx->f_cur_mm = f_min;
        }
        uint16_t p_open = read_u16_le(&payload[0]);
        uint16_t p_min = read_u16_le(&payload[2]);
        if (p_open >= 0x0E00U) {
            ctx->ap.c_open_wide = aperture_emount_p_to_z(p_open);
            ctx->ap.c_open_tele = ctx->ap.c_open_wide;
            ctx->ap.c_open_cur = ctx->ap.c_open_wide;
            ctx->ap.c_cur = ctx->ap.c_open_wide;
        }
        if (p_min > p_open) {
            ctx->ap.c_min_cur = aperture_emount_p_to_z(p_min);
        }
    }
}

static void parse_tx0a_reply(adapter_ctx_t *ctx, uint64_t now_ticks,
                             const uint8_t *payload, size_t payload_len) {
    if (payload_len < 4U) {
        return;
    }
    uint32_t lens_mask05 = read_u32_le(&payload[0]);
    if (lens_mask05 != 0UL) {
        ctx->id05.active_mask = ID05_FULL_18GRP_MASK & lens_mask05;
        update_id05_expected_len(&ctx->id05);
        ctx->tamron_session_mode = 1U;
        ctx->hs_step = HANDSHAKE_STEP_WAIT_05_06;
        ctx->state_enter_ticks = now_ticks;
    } else {
        ctx->id05.active_mask = 0UL;
        ctx->tamron_session_mode = 2U;
        vr_controller_on_emount_ack(ctx, 0x0AU, &now_ticks);
    }
}

static e_handshake_step_t map_ack_cmd_to_step(uint8_t cmd) {
    static const struct {
        uint8_t            cmd;
        e_handshake_step_t step;
    } ACK_MAP[] = {
        {0x4CU, HANDSHAKE_STEP_CMD4C},
        {0x4BU, HANDSHAKE_STEP_CMD4B},
        {0x3FU, HANDSHAKE_STEP_CMD3F},
        {0x09U, HANDSHAKE_STEP_CMD09},
        {0x0BU, HANDSHAKE_STEP_CMD0B},
        {0x0DU, HANDSHAKE_STEP_CMD0D},
        {0x32U, HANDSHAKE_STEP_CMD32},
        {0x5AU, HANDSHAKE_STEP_CMD5A},
    };
    for (size_t idx = 0U; idx < (sizeof(ACK_MAP) / sizeof(ACK_MAP[0])); ++idx) {
        if (ACK_MAP[idx].cmd == cmd) {
            return ACK_MAP[idx].step;
        }
    }
    return HANDSHAKE_STEP_NONE;
}

static void publish_early_handshake_identity(adapter_ctx_t *ctx, optics_workspace_t *optics) {
    af_servo_configure_for_lens(ctx);
    vr_controller_configure_for_lens(ctx);
    aperture_controls_configure_for_lens(ctx);
    optics_build_initial_tables(ctx, optics);
    if ((ctx->optics_ready != 0U) &&
        ((lens_db_lookup_cached(ctx) != NULL) || (ctx->hs_tx08_done != 0U))) {
        ctx->identity_ready = 1U;
        log_hs_identity_bb(ctx);
    }
    z_snapshot_publish(ctx, optics);
}

static bool handle_identity_reply(adapter_ctx_t *ctx, optics_workspace_t *optics,
                                  const uint8_t *payload, size_t payload_len) {
    if ((ctx->hs_step != HANDSHAKE_STEP_CMD07) || (payload_len < TX07_IDENTITY_MIN_BYTES) ||
        (read_u16_le(&payload[9]) == 0U)) {
        return false;
    }
    parse_tx07_reply(ctx, payload, payload_len);
    if ((optics != NULL) && (optics->sony_ds13_lid != ctx->e_lens_id)) {
        optics->sony_ds13_len = 0U;
        optics->sony_ds13_lid = 0U;
        optics->sony_ds13_valid = 0U;
#if !defined(__STDC_HOSTED__) || (__STDC_HOSTED__ == 0)
        uint16_t cached_len = 0U;
        if (retram_warm_cache_load_ds13(ctx->e_lens_id, optics->sony_ds13_buf, &cached_len)) {
            optics->sony_ds13_len = cached_len;
            optics->sony_ds13_lid = ctx->e_lens_id;
            optics->sony_ds13_valid = 1U;
            ctx->ds13_cache_hit = 1U;
        }
#endif
    }
    publish_early_handshake_identity(ctx, optics);
    return true;
}

static void finish_homing_ready(adapter_ctx_t *ctx, const optics_workspace_t *optics,
                                uint64_t now_ticks, bool init_ok) {
    if (init_ok) {
        ctx->home_completed_ok = 1U;
        ctx->home_retrigger_armed = 0U;
        ctx->cmd15_seen_since_arm = 0U;
        if (ctx->vr.lens_has_ois != 0U) {
            vr_controller_on_initialization_result(ctx, true);
        }
    }
    advance_handshake_after(HANDSHAKE_STEP_CMD10, ctx, optics, now_ticks);
}

static void handle_homing_reply(adapter_ctx_t *ctx, const optics_workspace_t *optics,
                                uint8_t cmd, const uint8_t *payload,
                                size_t payload_len, const uint64_t *now_ticks_ptr) {
    uint64_t now_ticks = *now_ticks_ptr;
    if (ctx->brand != LENS_BRAND_SIGMA) {
        if (ctx->hs_step != HANDSHAKE_STEP_CMD10) {
            return;
        }
        finish_homing_ready(ctx, optics, now_ticks, true);
        return;
    }
    if (cmd == 0x17U) {
        finish_homing_ready(ctx, optics, now_ticks, true);
        return;
    }
    if ((cmd != E_CMD_IRIS_HOME_10) || !native_home_pending(ctx)) {
        return;
    }
    bool step_ok = (bool)((payload_len == 0U) || (((uint32_t)payload[0] & 1UL) == 0UL));
    if (ctx->home_retry_stage == 0U) {
        if (step_ok) {
            finish_homing_ready(ctx, optics, now_ticks, true);
        } else {
            if (ctx->vr.lens_has_ois != 0U) {
                vr_controller_on_initialization_result(ctx, false);
            }
            ctx->home_retry_stage = 1U;
            ctx->hs_step = HANDSHAKE_STEP_WAIT_HOME;
            ctx->state_enter_ticks = now_ticks;
            uint8_t pay_iris[1] = {CMD10_HOME_IRIS_ONLY};
            (void)lens_session_enqueue_tx(ctx, E_CMD_IRIS_HOME_10, pay_iris, 1U);
        }
        return;
    }
    if (ctx->home_retry_stage == 1U) {
        ctx->home_retry_stage = step_ok ? 2U : 3U;
        ctx->state_enter_ticks = now_ticks;
        uint8_t pay_foc[1] = {CMD10_HOME_FOCUS_ONLY};
        (void)lens_session_enqueue_tx(ctx, E_CMD_IRIS_HOME_10, pay_foc, 1U);
        return;
    }
    finish_homing_ready(ctx, optics, now_ticks, (bool)((ctx->home_retry_stage == 2U) && step_ok));
}

static void handle_handshake_reply(adapter_ctx_t *ctx, optics_workspace_t *optics,
                                   uint8_t cmd, const uint8_t *payload,
                                   size_t payload_len, const uint64_t *now_ticks_ptr) {
    uint64_t now_ticks = *now_ticks_ptr;
    if ((cmd == 0x3AU) || (cmd == 0x3BU)) {
        handle_3a_3b_reply(ctx, optics, cmd, payload, payload_len, now_ticks_ptr);
        return;
    }
    if (cmd == 0x0AU) {
        parse_tx0a_reply(ctx, now_ticks, payload, payload_len);
        return;
    }
    if ((cmd == E_CMD_IRIS_HOME_10) || (cmd == 0x17U)) {
        handle_homing_reply(ctx, optics, cmd, payload, payload_len, now_ticks_ptr);
        return;
    }

    e_handshake_step_t step_done = map_ack_cmd_to_step(cmd);
    if ((cmd == 0x01U) && (payload_len >= E_CAP_BITMAP_BYTES)) {
        clear_photo_capability(ctx);
        ctx->optics_disconnected = 0U;
        memcpy(ctx->cap_bitmap, payload, E_CAP_BITMAP_BYTES);
        ctx->vr_cmd3d_negotiated = lens_session_cap_has(ctx, 0x3DU) ? 1U : 0U;
        step_done = HANDSHAKE_STEP_CMD01;
    } else if (cmd == 0x07U) {
        if (!handle_identity_reply(ctx, optics, payload, payload_len)) {
            return;
        }
        step_done = HANDSHAKE_STEP_CMD07;
    } else if (cmd == 0x0CU) {
        if ((payload_len >= 1U) && (((uint32_t)payload[0] & 1UL) != 0UL)) {
            ctx->uart_baud = BAUD_1500K;
        }
        step_done = HANDSHAKE_STEP_CMD0C;
    } else if (cmd == 0x08U) {
        parse_tx08_reply(ctx, payload, payload_len);
        publish_early_handshake_identity(ctx, optics);
        step_done = HANDSHAKE_STEP_CMD08;
    } else if (cmd == 0x3DU) {
        parse_tx3d_reply(ctx, payload, payload_len);
        step_done = HANDSHAKE_STEP_CMD3D;
    } else if (cmd == 0x3FU) {
        (void)try_extract_cmd3f_lens_name(ctx, payload, payload_len);
    }

    if ((step_done == HANDSHAKE_STEP_CMD5A) && (ctx->vr_cmd3d_negotiated == 0U)) {
        clear_photo_capability(ctx);
    }
    if ((step_done != HANDSHAKE_STEP_NONE) && (step_done == ctx->hs_step)) {
        advance_handshake_after(step_done, ctx, optics, now_ticks);
    }
}
/* NOLINTEND(readability-magic-numbers) */

static bool validate_emount_envelope(const uint8_t *wire, size_t wire_len) {
    if ((wire == NULL) || (wire_len < EMOUNT_ENVELOPE_OVHD)) {
        return false;
    }
    if ((wire[0] != EMOUNT_SOF) || (wire[wire_len - 1U] != EMOUNT_EOF)) {
        return false;
    }
    size_t declared_len = (size_t)read_u16_le(&wire[1]);
    if (declared_len != wire_len) {
        return false;
    }
    uint16_t expected_sum = read_u16_le(&wire[wire_len - 3U]);
    return expected_sum == boryoza_sum16_emount(wire, wire_len);
}

static bool handle_emount_id06_frame(adapter_ctx_t *ctx, optics_workspace_t *optics,
                                     const uint64_t *now_ticks_ptr,
                                     const uint8_t *payload, size_t payload_len) {
    uint64_t now_ticks = *now_ticks_ptr;
    if (!lens_session_unpack_id06(ctx, now_ticks, payload, payload_len)) {
        return false;
    }
    if (ctx->session_state == E_SESSION_ACTIVE) {
        uint8_t prev_pair = ctx->id05_fresh_since_06;
        ctx->id05_fresh_since_06 = (prev_pair == (uint8_t)ID05_PAIR_EARLY_05)
                                       ? (uint8_t)ID05_PAIR_EARLY_05
                                       : (uint8_t)ID05_PAIR_WAIT_05;
        af_servo_update_telemetry(ctx, now_ticks);
        if (prev_pair == (uint8_t)ID05_PAIR_EARLY_05) {
            ctx->id05_fresh_since_06 = (uint8_t)ID05_PAIR_IDLE;
        }
        if (ctx->dist_anchor_pending != 0U) {
            ctx->dist_anchor_cz = ctx->af.cur_cz;
            ctx->dist_anchor_pending = 0U;
        }
        aperture_controls_update_from_id05(ctx, optics);
        af_servo_refresh_kz(ctx);
        z_snapshot_publish(ctx, optics);
    } else if ((ctx->hs_step == HANDSHAKE_STEP_WAIT_05_06) && (ctx->id05.grp_present[0] != 0U)) {
        finalize_active_session(ctx, optics);
    }
    return true;
}

static void dispatch_active_id05_telemetry(adapter_ctx_t *ctx, optics_workspace_t *optics,
                                           uint64_t now_ticks, bool unpacked_06) {
    uint8_t prev_pair = ctx->id05_fresh_since_06;
    if (unpacked_06 || (ctx->id06.valid == 0U)) {
        ctx->id05_fresh_since_06 = (uint8_t)ID05_PAIR_EARLY_05;
        af_servo_update_telemetry(ctx, now_ticks);
        ctx->id05_fresh_since_06 = (uint8_t)ID05_PAIR_IDLE;
    } else {
        bool paired_after_06 = (bool)(prev_pair == (uint8_t)ID05_PAIR_WAIT_05);
        ctx->id05_fresh_since_06 = (uint8_t)ID05_PAIR_EARLY_05;
        af_servo_on_standalone_id05(ctx, paired_after_06);
        ctx->id05_fresh_since_06 = paired_after_06
                                       ? (uint8_t)ID05_PAIR_IDLE
                                       : (uint8_t)ID05_PAIR_EARLY_05;
    }
    if (unpacked_06 && (ctx->dist_anchor_pending != 0U)) {
        ctx->dist_anchor_cz = ctx->af.cur_cz;
        ctx->dist_anchor_pending = 0U;
    }
    aperture_controls_validate_distance(ctx);
    aperture_controls_update_from_id05(ctx, optics);
    if (unpacked_06) {
        ctx->dist_anchor_pending = 0U;
    }
    af_servo_refresh_kz(ctx);
    vr_controller_refresh_state(ctx);
    optics_update_from_live_id05(ctx, optics);
    z_snapshot_publish(ctx, optics);
}

static bool handle_emount_id05_frame(adapter_ctx_t *ctx, optics_workspace_t *optics,
                                     const uint64_t *now_ticks_ptr,
                                     const uint8_t *payload, size_t payload_len) {
    static const uint8_t E_CMD_ID06 = 0x06U;
    static const size_t ID05_EXT_18GRP_BYTES = 108U;
    static const size_t ID05_STD_15GRP_BYTES = 96U;
    static const size_t ID05_COMBO_MIN_BYTES = 110U;
    uint64_t now_ticks = *now_ticks_ptr;
    if ((ctx->id05.expected_payload_bytes == ID05_EXT_18GRP_BYTES) &&
        ((payload_len == ID05_STD_15GRP_BYTES) ||
         ((payload_len >= ID05_COMBO_MIN_BYTES) && (payload_len != ID05_EXT_18GRP_BYTES) &&
          ((payload_len <= ID05_EXT_18GRP_BYTES) || (payload[ID05_EXT_18GRP_BYTES] != E_CMD_ID06)) &&
          (payload[ID05_STD_15GRP_BYTES] == E_CMD_ID06)))) {
        ctx->id05.active_mask = ID05_STD_15GRP_MASK;
        update_id05_expected_len(&ctx->id05);
    }
    if (!lens_session_unpack_id05(&ctx->id05, payload, payload_len)) {
        return false;
    }
    bool unpacked_06 = false;
    size_t cursor = (size_t)ctx->id05.expected_payload_bytes;
    if ((payload_len > (cursor + 1U)) && (payload[cursor] == E_CMD_ID06)) {
        unpacked_06 = lens_session_unpack_id06(ctx, now_ticks,
                                               &payload[cursor + 1U],
                                               payload_len - (cursor + 1U));
    }
    if ((ctx->e_lens_id == SIGMA_150600_ID_REV2) || (ctx->e_lens_id == SIGMA_150600_ID_REV1)) {
        sigma_150600_zoom_eval_t zeval;
        sigma_150600_eval_zoom(ctx->id06.zoom_norm, &zeval);
        if (ctx->ap.raw_p_open_08 == 0U) {
            ctx->id05.p_open = zeval.p_open;
            ctx->id05.p_min = zeval.p_min;
        }
    }
    if (ctx->session_state == E_SESSION_ACTIVE) {
        dispatch_active_id05_telemetry(ctx, optics, now_ticks, unpacked_06);
    } else {
        ctx->id05_fresh_since_06 = (uint8_t)ID05_PAIR_EARLY_05;
        if ((ctx->hs_step == HANDSHAKE_STEP_WAIT_05_06) && (ctx->id06.valid != 0U)) {
            finalize_active_session(ctx, optics);
        }
    }
    return true;
}

static bool handle_aperture_status_reply(adapter_ctx_t *ctx, optics_workspace_t *optics,
                                         const uint8_t *payload, size_t payload_len) {
    if ((payload_len < APERTURE_STATUS_BYTES) || (ctx->brand != LENS_BRAND_SIGMA) ||
        (ctx->session_state != E_SESSION_ACTIVE)) {
        return false;
    }
    ctx->id05.p_ring = read_u16_le(payload);
    ctx->id05.manual_ring_active = (uint8_t)((uint32_t)payload[2] & 1UL);
    ctx->id05.grp_present[3] = 1U;
    uint16_t current_av = read_u16_le(&payload[3]);
    if (current_av >= E_AV_MIN_OPEN_P) {
        ctx->id05.p_cur = current_av;
    }
    aperture_controls_update_from_id05(ctx, optics);
    af_servo_refresh_kz(ctx);
    z_snapshot_publish(ctx, optics);
    return true;
}

bool lens_session_on_emount_frame(adapter_ctx_t *ctx, optics_workspace_t *optics,
                                  uint64_t now_ticks, const uint8_t *wire, size_t wire_len) {
    static const uint8_t E_CMD_ID05    = 0x05U;
    static const uint8_t E_CMD_ID06    = 0x06U;
    static const uint8_t E_CMD_ID0A    = 0x0AU;
    static const uint8_t E_CMD_ID16    = 0x16U;
    static const uint8_t E_CMD_ID3F    = 0x3FU;
    static const size_t  WIRE_OFF_CMD  = 5U;
    static const size_t  WIRE_OFF_PAY  = 6U;

    if (!validate_emount_envelope(wire, wire_len)) {
        return false;
    }
    ctx->last_emount_rx_ticks = now_ticks;
    ctx->pa3_debounce_cnt = 0U;

    uint8_t cmd = wire[WIRE_OFF_CMD];
    const uint8_t *payload = &wire[WIRE_OFF_PAY];
    size_t payload_len = wire_len - EMOUNT_ENVELOPE_OVHD;

    if (cmd == E_CMD_APERTURE_STATUS) {
        return handle_aperture_status_reply(ctx, optics, payload, payload_len);
    }

    if (cmd == E_CMD_ID05) {
        return handle_emount_id05_frame(ctx, optics, &now_ticks, payload, payload_len);
    }

    if (cmd == E_CMD_ID06) {
        return handle_emount_id06_frame(ctx, optics, &now_ticks, payload, payload_len);
    }

    if (cmd == E_CMD_ID16) {
        vr_controller_on_emount_ack(ctx, cmd, &now_ticks);
        return true;
    }

    if ((cmd == E_CMD_ID0A) && (ctx->session_state == E_SESSION_PARKING)) {
        parse_tx0a_reply(ctx, now_ticks, payload, payload_len);
        return true;
    }

    if (ctx->session_state == E_SESSION_HANDSHAKE) {
        handle_handshake_reply(ctx, optics, cmd, payload, payload_len, &now_ticks);
    } else if ((cmd == E_CMD_ID3F) && (ctx->session_state == E_SESSION_ACTIVE) &&
               try_extract_cmd3f_lens_name(ctx, payload, payload_len)) {
        z_snapshot_publish(ctx, optics);
    }
    return true;
}

static void step_active_handshake_timers(adapter_ctx_t *ctx, optics_workspace_t *optics,
                                         uint64_t now_ticks) {
    if (ctx->hs_step == HANDSHAKE_STEP_CMD01) {
        uint8_t max_retries = (ctx->wake_rehandshake != 0U) ? WAKE_CMD01_RETRIES : MAX_CMD01_RETRIES;
        if ((ctx->wake_ready != 0U) &&
            ((now_ticks - ctx->state_enter_ticks) >= CMD01_RETRY_TICKS) &&
            (ctx->cmd01_retries < max_retries)) {
            ctx->cmd01_retries = (uint8_t)(ctx->cmd01_retries + 1U);
            ctx->tx_q_head = 0U;
            ctx->tx_q_tail = 0U;
            ctx->wake_ready = 0U;
            ctx->wake_retry_req = 1U;
            send_hs_step(HANDSHAKE_STEP_CMD01, ctx, now_ticks);
        }
        return;
    }
    if (ctx->hs_step == HANDSHAKE_STEP_CMD07) {
        return;
    }
    if (ctx->hs_step == HANDSHAKE_STEP_CMD10) {
        if ((now_ticks - ctx->state_enter_ticks) > HS_HOME_TIMEOUT_TICKS) {
            advance_handshake_after(HANDSHAKE_STEP_CMD10, ctx, optics, now_ticks);
        } else if ((ctx->brand != LENS_BRAND_SIGMA) &&
                   ((now_ticks - ctx->home_poll_ticks) >= HOME_REPOLL_TICKS) &&
                   (ctx->tx_q_head == ctx->tx_q_tail)) {
            uint8_t pay10[1] = {(ctx->lens_family == 2U) ? CMD10_HOME_FAM2 : CMD10_HOME_STD};
            (void)lens_session_enqueue_tx(ctx, E_CMD_IRIS_HOME_10, pay10, (uint16_t)sizeof(pay10));
            ctx->home_poll_ticks = now_ticks;
        }
        return;
    }
    if (ctx->hs_step == HANDSHAKE_STEP_WAIT_HOME) {
        if ((now_ticks - ctx->state_enter_ticks) > HS_HOME_TIMEOUT_TICKS) {
            advance_handshake_after(HANDSHAKE_STEP_CMD10, ctx, optics, now_ticks);
        }
        return;
    }
    if (ctx->hs_step == HANDSHAKE_STEP_WAIT_05_06) {
        if ((now_ticks - ctx->state_enter_ticks) > WAIT_05_06_MAX_TICKS) {
            finalize_active_session(ctx, optics);
        }
        return;
    }
    if ((now_ticks - ctx->state_enter_ticks) > HS_STEP_TIMEOUT_TICKS) {
        advance_handshake_after(ctx->hs_step, ctx, optics, now_ticks);
    }
}

static bool is_emount_rx_silent_for_detach(const adapter_ctx_t *ctx, uint64_t now_ticks) {
    if (ctx->last_emount_rx_ticks == 0ULL) {
        return true;
    }
    if (now_ticks < ctx->last_emount_rx_ticks) {
        return false;
    }
    return (now_ticks - ctx->last_emount_rx_ticks) >= RX_SILENCE_DETACH_TKS;
}

static bool try_fast_remount_swap(adapter_ctx_t *ctx, optics_workspace_t *optics, uint64_t now_ticks) {
    bool swappable = (bool)((ctx->session_state == E_SESSION_PASSIVE_MOUNT) ||
                            ((ctx->hs_cmd07_ok == 0U) &&
                             ((ctx->session_state == E_SESSION_POWERUP_WAIT) ||
                              (ctx->session_state == E_SESSION_HANDSHAKE))));
    if ((!swappable) || (ctx->pa3_debounce_cnt < ATTACH_DEBOUNCE_THRESH)) {
        return false;
    }
    ctx->pa3_debounce_cnt = 0U;
    ctx->session_epoch += 1U;
    ctx->wake_rehandshake = 0U;
    ctx->wake_from_park_req = 0U;
    ctx->wake_boot_prologue_seen = 0U;
    ctx->identity_ready = 0U;
    ctx->optics_ready = 0U;
    ctx->z_primary_lens_id = DEFAULT_Z_PROVISIONAL_ID;
    optics_build_initial_tables(ctx, optics);
    z_snapshot_publish(ctx, optics);
    ctx->session_state = E_SESSION_POWERUP_WAIT;
    ctx->state_enter_ticks = now_ticks;
    ctx->hs_start_ticks = now_ticks;
    return true;
}

void lens_session_step(adapter_ctx_t *ctx, optics_workspace_t *optics,
                       uint8_t pa3_mounted, const uint32_t *vd_count_ptr, uint64_t now_ticks) {
    if ((ctx->session_state == E_SESSION_ACTIVE) &&
        is_emount_rx_silent_for_detach(ctx, now_ticks)) {
        vr_controller_invalidate_driver(ctx);
    }
    if (vr_controller_step(ctx, now_ticks)) {
        z_snapshot_publish(ctx, optics);
    }
    uint32_t vd_count = (vd_count_ptr != NULL) ? *vd_count_ptr : 0U;
    bool currently_attached = false;
    if ((ctx->session_state != E_SESSION_DETACHED) &&
        (ctx->session_state != E_SESSION_EMPTY_MOUNT)) {
        currently_attached = true;
    }
    if (currently_attached && (pa3_mounted == 0U)) {
        if (ctx->pa3_debounce_cnt < DETACH_DEBOUNCE_THRESH) {
            ctx->pa3_debounce_cnt = (uint8_t)(ctx->pa3_debounce_cnt + 1U);
        }
        if ((ctx->pa3_debounce_cnt >= DETACH_DEBOUNCE_THRESH) &&
            is_emount_rx_silent_for_detach(ctx, now_ticks)) {
            ctx->pa3_debounce_cnt = 0U;
            ctx->session_epoch += 1U;
            ctx->session_state = E_SESSION_DETACHED;
            return;
        }
    } else if ((!currently_attached) && (pa3_mounted != 0U)) {
        ctx->pa3_debounce_cnt = (uint8_t)(ctx->pa3_debounce_cnt + 1U);
        if (ctx->pa3_debounce_cnt >= ATTACH_DEBOUNCE_THRESH) {
            ctx->pa3_debounce_cnt = 0U;
            ctx->identity_ready = 0U;
            ctx->optics_ready = 0U;
            ctx->z_primary_lens_id = DEFAULT_Z_PROVISIONAL_ID;
            optics_build_initial_tables(ctx, optics);
            z_snapshot_publish(ctx, optics);
            ctx->session_state = E_SESSION_POWERUP_WAIT;
            ctx->vd_boot_start = vd_count;
            ctx->state_enter_ticks = now_ticks;
            ctx->hs_start_ticks = now_ticks;
            return;
        }
    } else {
        if (try_fast_remount_swap(ctx, optics, now_ticks)) {
            ctx->vd_boot_start = vd_count;
            return;
        }
        ctx->pa3_debounce_cnt = 0U;
    }

    if (ctx->session_state == E_SESSION_PASSIVE_MOUNT) {
        return;
    }
    if (ctx->session_state == E_SESSION_POWERUP_WAIT) {
        if ((vd_count - ctx->vd_boot_start) >= BOOT_VD_WAIT_CYCLES) {
            lens_session_start_mount_handshake(ctx, optics, now_ticks);
        }
        return;
    }
    if (ctx->session_state == E_SESSION_HANDSHAKE) {
        step_active_handshake_timers(ctx, optics, now_ticks);
    }
}

size_t lens_session_build_id03_wire(adapter_ctx_t *ctx, uint8_t *out_wire, uint8_t seq) {
    static const uint8_t E_CMD_ID03       = 0x03U;
    static const size_t  IDX_TGT_LO       = 3U;
    static const size_t  IDX_TGT_HI       = 4U;
    static const size_t  IDX_SEC_LO       = 5U;
    static const size_t  IDX_SEC_HI       = 6U;
    static const size_t  IDX_GEAR         = 7U;
    static const size_t  IDX_IRIS_EN      = 10U;
    static const size_t  IDX_TOGGLE       = 11U;
    static const size_t  IDX_OIS_B18      = 12U;
    static const size_t  IDX_OIS_W19      = 13U;
    static const size_t  IDX_OIS_W20      = 14U;
    static const size_t  IDX_OIS_B21      = 15U;
    static const size_t  IDX_SYNC_B22     = 16U;
    static const uint8_t OIS_B21_IDLE_EXT = 0x02U;

    uint8_t params[ID03_COMPOSITE_BYTES];
    memset(params, 0, sizeof(params));

    ctx->id03_frame_toggle = (uint8_t)((uint32_t)ctx->id03_frame_toggle ^ 1UL);
    uint8_t tog = (uint8_t)((uint32_t)ctx->id03_frame_toggle & 1UL);

    params[0] = (uint8_t)(ID03_IRIS_WINDOW_DEF & BYTE_MASK);
    params[1] = (uint8_t)((ID03_IRIS_WINDOW_DEF >> SHIFT_8) & BYTE_MASK);
    params[2] = (uint8_t)((ID03_IRIS_WINDOW_DEF >> SHIFT_16) & BYTE_MASK);

    uint32_t p_tgt = ctx->ap.p_target_emount;
    if (p_tgt < E_AV_MIN_OPEN_P) {
        p_tgt = aperture_z_to_emount_p(ctx->ap.c_open_cur);
    }
    uint32_t p_sec = ((ctx->id05.manual_ring_active == 0U) &&
                      (ctx->ap.body_a1_delta == 0) &&
                      (ctx->id05.p_open >= E_AV_MIN_OPEN_P))
                         ? (uint32_t)ctx->id05.p_open
                         : p_tgt;
    params[IDX_TGT_LO] = (uint8_t)(p_tgt & BYTE_MASK);
    params[IDX_TGT_HI] = (uint8_t)((p_tgt >> SHIFT_8) & BYTE_MASK);
    params[IDX_SEC_LO] = (uint8_t)(p_sec & BYTE_MASK);
    params[IDX_SEC_HI] = (uint8_t)((p_sec >> SHIFT_8) & BYTE_MASK);
    params[IDX_GEAR] = (ctx->has_ext_5a != 0U) ? ID03_WIRE13_EXT_GEAR : ID03_WIRE13_STD_GEAR;
    params[IDX_IRIS_EN] = 0x01U;
    params[IDX_TOGGLE] = tog;

    vr_controller_refresh_state(ctx);
    uint32_t w19 = ctx->vr.e_id03_w19;
    uint8_t b21 = ctx->vr.e_id03_b21;
    if ((ctx->vr.lens_has_ois == 0U) && ((ctx->lens_family == 2U) || (ctx->has_ext_5a != 0U))) {
        if (ctx->session_state == E_SESSION_HANDSHAKE) {
            w19 = 0x0002U;
        }
        b21 = OIS_B21_IDLE_EXT;
    }
    params[IDX_OIS_B18] = (uint8_t)(((uint32_t)ctx->vr.e_id03_b18 & CLEAR_BIT0_MASK) | (uint32_t)tog);
    params[IDX_OIS_W19] = (uint8_t)(w19 & BYTE_MASK);
    params[IDX_OIS_W20] = (uint8_t)((w19 >> SHIFT_8) & BYTE_MASK);
    params[IDX_OIS_B21] = b21;
    params[IDX_SYNC_B22] = 0x01U;

    size_t plen = (size_t)ID03_PARAM_BYTES;
    if (ctx->id03_is_32b != 0U) {
        params[ID03_PARAM_BYTES] = CMD2F_SUB0;
        if (ctx->brand == LENS_BRAND_TAMRON) {
            params[ID03_PARAM_BYTES + 1] = CMD2F_TAM_SUB1;
            params[ID03_PARAM_BYTES + 2] = CMD2F_TAM_SUB2;
        } else {
            params[ID03_PARAM_BYTES + 1] = CMD2F_STD_SUB1;
            params[ID03_PARAM_BYTES + 2] = (uint8_t)((uint32_t)CMD2F_STD_SUB2_BASE | (uint32_t)tog);
        }
        plen = (size_t)ID03_COMPOSITE_BYTES;
    }
    return boryoza_wrap_emount_frame(out_wire, EMOUNT_FRAME_TYPE_SYNC, &seq,
                                     E_CMD_ID03, params, plen);
}

size_t lens_session_build_id04_wire(adapter_ctx_t *ctx, uint8_t *out_wire, uint8_t seq) {
    static const uint8_t E_CMD_ID04     = 0x04U;
    static const uint8_t ID04_B8_MARKER = 0x19U;
    static const size_t  IDX_ID04_B16   = 10U;

    uint8_t params[ID04_MAX_PARAM_BYTES];
    memset(params, 0, sizeof(params));

    size_t sub_len = 0U;
    uint8_t hdr_b16 = 0U;
    uint8_t hdr_b9 = af_servo_build_id04_subrecord(
        ctx, &params[ID04_HDR_PARAM_BYTES], &sub_len, &hdr_b16);

    params[2] = ID04_B8_MARKER;
    params[3] = hdr_b9;
    params[IDX_ID04_B16] = hdr_b16;

    return boryoza_wrap_emount_frame(out_wire, EMOUNT_FRAME_TYPE_SYNC, &seq,
                                     E_CMD_ID04, params,
                                     (size_t)ID04_HDR_PARAM_BYTES + sub_len);
}
