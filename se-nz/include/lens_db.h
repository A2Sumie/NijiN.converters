#ifndef BORYOZA_LENS_DB_H
#define BORYOZA_LENS_DB_H

#include <stdint.h>
#include "boryoza_types.h"

#ifdef __cplusplus
extern "C" {
#endif

static const uint32_t LENS_FLAG_APSC          = (1UL << 0U);
static const uint32_t LENS_FLAG_MACRO         = (1UL << 1U);
static const uint32_t LENS_FLAG_FISHEYE       = (1UL << 2U);
static const uint32_t LENS_FLAG_OIS           = (1UL << 3U);
static const uint32_t LENS_FLAG_OIS_SWITCH    = (1UL << 4U);
static const uint32_t LENS_FLAG_DUAL_FLOATING = (1UL << 6U);
static const uint32_t LENS_FLAG_ID03_32B      = (1UL << 7U);

typedef struct {
    uint16_t e_lens_id;
    uint16_t z_lens_id;
    uint16_t f_min_mm;
    uint16_t f_max_mm;
    uint16_t av_open_wide_c;
    uint16_t av_open_tele_c;
    uint16_t mfd_mm;
    uint16_t mfd_tele_mm;
    uint16_t name_off;      /* Offset into g_lens_names_blob */
    uint16_t cam_e_inf;     /* 0 if dynamic/zoom cam */
    uint16_t cam_e_near;
    uint8_t  brand;
    uint8_t  flags;
    int16_t  spectral_idx;  /* -1 if neutral zero, >=0 index in g_spectral_81_db */
    int16_t  tamron_z_idx;  /* -1 if none, >=0 index in secondary optics table */
} lens_db_entry_t;

typedef struct {
    uint16_t zoom_raw;      /* 0..608 */
    uint16_t zoom_norm;     /* Normalized zoom coordinate */
    uint16_t f_nom_0p1mm;   /* 1500..6000 */
    uint16_t f_opt_0p1mm;   /* 1544..5778 */
    uint16_t exit_pupil;    /* 334..479 */
    uint16_t p_open;        /* Wide-open iris code */
    uint16_t p_min;         /* Minimum-aperture iris code */
    int16_t  e_far_mech;    /* Mechanical past-infinity stop (9093 -> 1280) */
    int16_t  e_inf_true;    /* True optical infinity (10293 -> 2453) */
    int16_t  e_near_true;   /* True MFD encoder (24052) */
    uint16_t mfd_mm;        /* 580..2800 mm across 150..600mm */
    uint16_t k_ref_u20000;  /* 97..104 */
    int16_t  cam_se_fracs[E_CURVE_NODES_COUNT];
} sigma_150600_zoom_knot_t;

typedef struct {
    int8_t   min_temp_c;
    int8_t   max_temp_c;
    uint16_t base_pps;
    uint32_t encoder_ticks_per_sec; /* base_pps * 16: 44800 .. 25600 */
} sigma_150600_thermal_row_t;

enum {
    Z_CMD24_A_NODE_SLOTS = 36,
};

typedef struct {
    uint16_t focal_mm;
    int16_t  vig_nodes8[E_CURVE_NODES_COUNT];
    int16_t  dist_b_nodes8[E_CURVE_NODES_COUNT];
    int16_t  lca_r_nodes8[E_CURVE_NODES_COUNT];
    int16_t  lca_b_nodes8[E_CURVE_NODES_COUNT];
} sigma_150600_optics_knot_t;

enum {
    OPT_U_AXIS_SPAN_0_1   = 0U,
    OPT_U_AXIS_RATIO_FMAX = 1U,
};

typedef struct {
    uint16_t z_lens_id;
    const uint8_t *cmd24_a_packed;
    uint16_t cmd24_a_packed_len;
    const uint8_t *cmd24_b_packed;
    uint16_t cmd24_b_packed_len;
    uint16_t a3_wide_q14;
    uint16_t a3_tele_q14;
    uint8_t  u_axis_mode;
} tamron_z_optics_profile_t;

typedef struct {
    float    focal_mm;
    uint16_t p_open;
    uint16_t p_min;
    int32_t  e_far_mech;
    int32_t  e_inf_true;
    int32_t  e_near_true;
    float    mfd_meters;
    int32_t  cam_se_fracs[E_CURVE_NODES_COUNT];
} sigma_150600_zoom_eval_t;

const lens_db_entry_t *lens_db_lookup(uint16_t e_lens_id);
const lens_db_entry_t *lens_db_lookup_cached(adapter_ctx_t *ctx);
const char *lens_db_entry_name(const lens_db_entry_t *entry);
const char *lens_db_vendor_tag5(uint8_t brand);
enum { SPECTRAL_81_POINTS = 81 };
bool lens_db_unpack_spectral81(int16_t spectral_idx, uint8_t out81[SPECTRAL_81_POINTS]);
void lens_db_get_opt_huff(const uint8_t **counts12, const uint8_t **syms, size_t *sym_count);
const tamron_z_optics_profile_t *lens_db_get_tamron_optics(int16_t tamron_z_idx);
size_t lens_db_get_tamron_optics_count(void);
void sigma_150600_eval_zoom(uint16_t zoom_norm, sigma_150600_zoom_eval_t *out_eval);
uint32_t sigma_150600_thermal_vmax_tps(int8_t temp_c);
const sigma_150600_optics_knot_t *sigma_150600_get_optics_knots(size_t *out_count);
const uint8_t *sigma_150600_get_optical_a_packed(size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* BORYOZA_LENS_DB_H */
