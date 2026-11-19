#ifndef BORYOZA_TYPES_H
#define BORYOZA_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Protocol & Buffer Sizing Constants
 * ========================================================================= */
enum {
    BORYOZA_FW_VER_MAJOR      = 2U,
    BORYOZA_FW_VER_MINOR      = 23U,
    BORYOZA_LF_BCD_MAJOR      = (((BORYOZA_FW_VER_MAJOR / 10U) << 4U) | (BORYOZA_FW_VER_MAJOR % 10U)),
    BORYOZA_LF_BCD_MINOR      = (((BORYOZA_FW_VER_MINOR / 10U) << 4U) | (BORYOZA_FW_VER_MINOR % 10U)),

    Z_CMD_HDR_BYTES           = 8,
    Z_CMD10_BYTES             = 28,
    Z_CMD11_BYTES             = 74,
    Z_CMD12_BYTES             = 66,
    Z_CMD20_00_BYTES          = 258,
    Z_CMD20_01_BYTES          = 24,
    Z_CMD24_A_BYTES           = 7426,
    Z_CMD24_A_STRIDE          = 7428,
    Z_CMD24_B_BYTES           = 3468,
    Z_HOTLINE_BYTES           = 42,
    Z_HOTLINE_STRIDE          = 44,
    Z_MOSI20_PREFIX_BYTES     = 52,
    Z_SPI3_SMALL_BUF_BYTES    = 516,
    Z_LARGE_ZERO_BUF_BYTES    = 1028,

    Z_CMD24_SPECTRAL_OFFSET   = 6977,
    Z_CMD24_SPECTRAL_TOTAL    = 179,
    Z_CMD24_SPECTRAL_POINTS   = 81,
    Z_CMD24_DISTORTION_OFFSET = 7169,
    Z_CMD24_DISTORTION_FLOATS = 54,
    Z_CMD24_FORMAT_OFFSET     = 7385,

    E_UART_RX_SLOTS           = 4,
    E_UART_RX_SLOT_BYTES      = 2048,
    E_UART_TX_BUF_BYTES       = 512,
    E_ID03_STD_BYTES          = 29,
    E_ID03_TAMRON_LEGACY_BYTES = 32,
    E_ID05_GROUP_COUNT        = 18,
    E_ID05_MAX_GROUP_BYTES    = 28,
    E_ID06_MAX_BYTES          = 64,
    E_CAP_BITMAP_BYTES        = 32,
    E_TX07_PAYLOAD_BYTES      = 35,
    E_TX08_PAYLOAD_BYTES      = 202,
    E_SONY_DS13_MAX_BYTES     = 3872,
    E_LF_WORKBUF_BYTES        = 36864,

    E_ID05_CURVE_CORE_BYTES   = 6,
    E_ID05_CURVE_TAIL_BYTES   = 3,
    E_CURVE_NODES_COUNT       = 8,
    E_SONY_PACKED9_BYTES      = 9,
    E_SONY_CURVE16_PTS        = 16,
    E_TX_CMD_MAX_PAYLOAD      = 56,
    E_TX_QUEUE_CAP            = 16,
    LENS_NAME_ASCII_CAP       = 64,
    VENDOR_TAG_BYTES          = 5,

    DIAG_RING_ENTRIES         = 512,
    BLACKBOX_SLOTS            = 256,
    BLACKBOX_RECORD_BYTES     = 32,
    JOURNAL_SLOTS_PER_SECTOR  = 64,
    JOURNAL_RECORD_BYTES      = 128,
    JOURNAL_RESERVED_BYTES    = 86,
};

/* =========================================================================
 * Lens Brand & Actuator Classification
 * ========================================================================= */
typedef enum {
    LENS_BRAND_GENERIC = 0,
    LENS_BRAND_SONY    = 1,
    LENS_BRAND_SIGMA   = 2,
    LENS_BRAND_TAMRON  = 3,
    LENS_BRAND_SAMYANG = 4,
    LENS_BRAND_ZEISS   = 5,
} lens_brand_t;

typedef enum {
    AF_CLASS_C_STD_1D_3C = 0, /* Standard signed step/scan sub-records */
    AF_CLASS_A_SONY_43_49 = 1, /* Extended focus capability set */
    AF_CLASS_B_LEGACY_F1_F2 = 2, /* Legacy unsigned u16 step commands */
} af_actuator_class_t;

typedef enum {
    ID05_PAIR_IDLE     = 0U,
    ID05_PAIR_EARLY_05 = 1U,
    ID05_PAIR_WAIT_05  = 2U,
} id05_pair_state_t;

typedef enum {
    E_SESSION_DETACHED        = 0,
    E_SESSION_POWERUP_WAIT    = 1,
    E_SESSION_HANDSHAKE       = 2,
    E_SESSION_ACTIVE          = 3,
    E_SESSION_PARKING         = 4,
    E_SESSION_PARKED          = 5,
    E_SESSION_EMPTY_MOUNT     = 6,
    E_SESSION_PASSIVE_MOUNT   = 7,
} e_session_state_t;

typedef enum {
    HANDSHAKE_STEP_NONE       = 0,
    HANDSHAKE_STEP_CMD01      = 1,
    HANDSHAKE_STEP_CMD07      = 2,
    HANDSHAKE_STEP_CMD0C      = 3,
    HANDSHAKE_STEP_CMD0B      = 4,
    HANDSHAKE_STEP_CMD4C      = 5,
    HANDSHAKE_STEP_CMD4B      = 6,
    HANDSHAKE_STEP_CMD3F      = 7,
    HANDSHAKE_STEP_CMD3D      = 8,
    HANDSHAKE_STEP_CMD3A_DS13 = 9,
    HANDSHAKE_STEP_CMD3B_DS13 = 10,
    HANDSHAKE_STEP_CMD32      = 11,
    HANDSHAKE_STEP_CMD5A      = 12,
    HANDSHAKE_STEP_CMD08      = 13,
    HANDSHAKE_STEP_CMD09      = 14,
    HANDSHAKE_STEP_CMD0D      = 15,
    HANDSHAKE_STEP_CMD10      = 16,
    HANDSHAKE_STEP_WAIT_HOME  = 17,
    HANDSHAKE_STEP_CMD3A_LF   = 18,
    HANDSHAKE_STEP_CMD3B_LF   = 19,
    HANDSHAKE_STEP_CMD0A      = 20,
    HANDSHAKE_STEP_WAIT_05_06 = 21,
    HANDSHAKE_STEP_SONY_POST  = 22,
    HANDSHAKE_STEP_READY      = 23,
} e_handshake_step_t;

typedef enum {
    PARK_IDLE                 = 0,
    PARK_WAIT_ID0A_ACK        = 1,
    PARK_WAIT_ID16_ACK        = 2,
    PARK_SETTLE_HOLD          = 3,
    PARK_COMPLETE             = 4,
    PARK_WAIT_VR_STOP         = 5,
} park_state_t;

typedef enum {
    VR_BODY_CMD_STANDBY_E3    = 0, /* Standby stabilization request */
    VR_BODY_CMD_OFF_A3        = 1, /* Stabilization disabled request */
    VR_BODY_CMD_NORMAL        = 2, /* Standard stabilization mode */
    VR_BODY_CMD_SPORT         = 3, /* Sport stabilization mode */
} vr_body_mode_t;

/* =========================================================================
 * Unpacked E-mount ID05 (18 Dynamic Groups) & ID06 Telemetry State
 * ========================================================================= */
typedef struct {
    uint32_t active_mask;
    uint16_t expected_payload_bytes;
    uint8_t  grp[E_ID05_GROUP_COUNT][E_ID05_MAX_GROUP_BYTES];
    uint8_t  grp_present[E_ID05_GROUP_COUNT];

    /* Decoded physical fields from active groups */
    uint16_t dist_code_x;        /* grp[4]+0..1: logarithmic focus distance code */
    uint16_t f_opt_0p1mm;        /* grp[4]+4..5 */
    uint16_t f_nom_0p1mm;        /* grp[4]+6..7 */
    uint16_t p_cur;              /* grp[0]+0..1: current iris code (256 steps/EV) */
    uint16_t p_open;             /* CMD08+0..1 + grp[5]+0..1 */
    uint16_t p_min;              /* CMD08+2..3 */
    uint8_t  mag_code;           /* grp[4]+3: beta = 2^(5 - code/16) when valid */
    uint16_t exit_pupil_raw;     /* grp[2]+0..1 */
    uint16_t p_ring;             /* grp[3]+0..1 */
    uint8_t  manual_ring_active; /* grp[3]+2.bit0 */
    int16_t  delta_av;           /* grp[5]+0..1 */
    uint8_t  vig_core6[E_ID05_CURVE_CORE_BYTES];
    uint8_t  dist_core6[E_ID05_CURVE_CORE_BYTES];
    uint8_t  vig_tail3[E_ID05_CURVE_TAIL_BYTES];
    uint8_t  dist_tail3[E_ID05_CURVE_TAIL_BYTES];
    uint8_t  lca_types[E_ID05_CURVE_CORE_BYTES];
    uint8_t  lca_ch1_core6[E_ID05_CURVE_CORE_BYTES];
    uint8_t  lca_ch2_core6[E_ID05_CURVE_CORE_BYTES];
    uint8_t  lca_ch1_tail3[E_ID05_CURVE_TAIL_BYTES];
    uint8_t  lca_ch2_tail3[E_ID05_CURVE_TAIL_BYTES];
    uint8_t  aberration_valid;
    int8_t   mf_ring_dir;        /* grp[7]+0 */
    uint8_t  mf_ring_step;       /* grp[7]+1 */
    uint8_t  is_mf_switch;       /* grp[7]+2.bit1 (0=AF, 1=MF) */
    uint8_t  l_fn_pressed;       /* grp[7]+4.bit3 (AFL / Focus Set) */
    uint8_t  barrel_ois_on;      /* grp[7]+4.bit4 (1=ON, 0=OFF) */
    uint8_t  barrel_ois_mode;    /* grp[7]+4.bits[6:5] */
    uint8_t  ois_active_fb;      /* grp[9]+0.bits[1:0] */
    uint8_t  optical_curve9[3][E_ID05_CURVE_CORE_BYTES + E_ID05_CURVE_TAIL_BYTES];
    uint8_t  optical_curve_valid;
    uint8_t  optical_curve_full;
    uint8_t  optics_curve_dirty;
    uint8_t  curve_frames_seen;
    uint8_t  optics_skip;
} e_id05_state_t;

typedef struct {
    uint8_t  active_mask;
    uint8_t  raw_bytes[E_ID06_MAX_BYTES];
    uint16_t raw_len;

    int32_t  focus_pos_se;       /* Signed/unsigned normalized E encoder position */
    int32_t  focus_prev_se;
    uint64_t t_sample_ticks;
    uint64_t t_prev_ticks;
    uint8_t  hw_moving;          /* Hardware motor moving flag */
    uint8_t  at_far_end;
    uint8_t  at_near_end;
    int32_t  lim_far_se;         /* Reported far limit in ID06 */
    int32_t  lim_near_se;        /* Reported near limit in ID06 */
    uint16_t zoom_norm;          /* Normalized zoom position */
    uint16_t defocus_k_se;       /* Lens-reported defocus sensitivity factor */
    int8_t   temp_celsius;       /* Lens internal temperature sensor */
    uint8_t  valid;
    uint8_t  mf_ring_byte;       /* Focus ring rotation state from status payload */
} e_id06_state_t;

/* =========================================================================
 * Outgoing E-mount Command Packet Item
 * ========================================================================= */
typedef struct {
    uint8_t  cmd;
    uint8_t  frame_type;
    uint16_t payload_len;
    uint8_t  payload[E_TX_CMD_MAX_PAYLOAD];
    uint32_t serial;
} e_tx_cmd_t;

/* =========================================================================
 * AF Servo & Coordinate Bridge State
 * ========================================================================= */
typedef struct {
    uint8_t  scale_exp_n;        /* Exponent n in {7, 9, 10}, packed into descriptor nibble */
    float    scale_const_a;      /* A = 2^(n+7): 16384.0f, 65536.0f, or 131072.0f */
    int32_t  mech_lz;            /* -512 */
    int32_t  mech_hz;            /* round(A / D_mfd_m) */

    int32_t  e_inf_true;         /* Latched/interpolated true infinity encoder S_E */
    int32_t  e_near_true;        /* Latched/interpolated true MFD encoder S_E */
    int32_t  dir_sign;           /* +1 if e_near_true >= e_inf_true, else -1 */
    float    mfd_meters;         /* Minimum focus distance at current focal length */
    uint8_t  barrel_limiter_on;  /* 1 if lens physical focus limiter narrows stroke */

    int32_t  cur_cz;             /* Current C_Z = Phi_z(S_E) */
    float    cur_dist_m;         /* Smooth distance in meters */
    float    cur_jacobian;       /* |dC_Z / dS_E| > 0 */
    uint8_t  kz_b0;              /* Focus sensitivity mantissa byte */
    uint8_t  kz_b1;              /* Focus sensitivity shift byte */

    /* Active Z AF request state */
    uint8_t  latest_cookie;
    uint8_t  arrival_confirmed;
    int32_t  target_cz;
    int32_t  target_se;
    float    cmd_vel_cz_pps;
    float    cmd_vel_se_pps;
    uint8_t  unlimited_speed;
    uint8_t  is_continuous_track;
    uint8_t  af_drive_active;
    uint8_t  af_cmd_pending;     /* Pending ID04 AF sub-record request */
    uint8_t  af_stop_req;        /* Explicit AF stop request (1C) */
    uint8_t  af_submode;         /* 0=idle, 1=1D step/target, 2=1C pre-stop, 3=3C scan */
    int8_t   motion_dir_cz;      /* +1 for +C_Z, -1 for -C_Z, 0 idle */
    uint8_t  settle_frames_rem;  /* Post-arrival settle hold frames */
    uint8_t  mf_hold_frames_rem; /* Manual focus ring activity hold frames */
    uint8_t  stable_idle_count;
    uint8_t  stall_vsyncs;       /* Non-moving VSync settle guard counter */
    uint8_t  s1_hold_vsyncs;     /* Half-press / recent-motion servo hold counter */
    uint64_t af_cmd_ticks;       /* Timestamp of latest Z AF command */
    uint8_t  seq_3c;             /* Rolling 2-bit counter in 3C byte1[3:2] */
    uint8_t  id1f_param_requested; /* Startup parameter requested */

    /* Linear manual focus configuration */
    uint8_t  linear_mf_code;
    int8_t   mf_ring_delta_bf;
    int8_t   prev_drive_dir_se;
    uint8_t  cushioned_near;
    uint8_t  cushioned_far;
    int32_t  step_accum_se;
    uint8_t  drive_vsyncs;
    uint8_t  e0_brake_rem;
    uint8_t  stalled_abort;
    uint8_t  prev_continuous;
    uint8_t  track_primed;
} af_servo_state_t;

typedef struct {
    int32_t  last_hot_cz;
    int32_t  active_step_se;
    int32_t  parfocal_cz_anchor;
    int32_t  parfocal_trim_se;
    float    ke_inf_ref;
    float    ke_first_obs;
    float    beta_first_obs;
    uint16_t prev_f_cur_mm;
    uint16_t parfocal_last_f_mm;
    uint16_t parfocal_last_zoom;
    uint8_t  hotline_seen;
    uint8_t  ke_inf_valid;
    uint8_t  ke_first_valid;
    uint8_t  kz_c0_b0;
    uint8_t  kz_c0_b1;
    uint8_t  parfocal_anchor_valid;
    uint8_t  parfocal_wait_vsyncs;
    uint8_t  parfocal_trim_pending;
    uint8_t  last_z_af_cmd;
} af_v218_ext_t;

typedef struct {
    uint32_t c11_identity_hash;
    uint16_t c10_z_lid;
    uint16_t c10_e_lid;
    uint16_t c10_e_ver;
    uint8_t  c10_flags;
    uint8_t  c10_scale_n;
    uint8_t  c10_state_class;
    uint8_t  c10_identity_ready;
    uint16_t c12_open_w;
    uint16_t c12_open_t;
    uint16_t c12_f_cur;
    uint16_t c12_f_min;
    uint16_t c12_f_max;
    uint8_t  c12_vr0;
    uint8_t  c12_vr1;
    uint8_t  c11_done_mask;
    uint8_t  c10_done_mask;
    uint8_t  c12_done_mask;
    uint8_t  dist_invalid;
} z_static_snap_cache_t;

/* =========================================================================
 * VR / OIS Duty & Shutdown Parking State
 * ========================================================================= */
typedef struct {
    uint8_t        lens_has_ois;
    uint8_t        lens_has_ois_switch;
    vr_body_mode_t body_mode;
    uint8_t        z_vr_cap0;         /* Primary VR capability byte */
    uint8_t        z_vr_cap1;         /* Secondary VR capability byte */
    uint8_t        z_df_state;        /* Configured VR status byte */
    uint8_t        stop_latched;
    uint16_t       cap_lens_lv;       /* 12800 or 0 */
    uint16_t       cap_lens_exp;      /* 12800 or 0 */
    uint16_t       cap_body_lv;       /* 0 or 12000 */
    uint16_t       cap_body_exp;      /* 0 or 12000 */
    uint8_t        e_id03_b18;        /* ID03 wire[18] */
    uint16_t       e_id03_w19;        /* ID03 wire[19..20] */
    uint8_t        e_id03_b21;        /* ID03 wire[21] */
    uint8_t        allow_id19;

    park_state_t   park_state;
    uint64_t       park_start_ticks;
    uint64_t       park_step_ticks;
    uint8_t        hotline_enabled;
    uint8_t        hotline_seq;
    uint8_t        internal_mode;
    uint8_t        d7_flags;
    uint8_t        profile_field;
    uint8_t        policy_field;
    uint8_t        phase_gate;
    uint8_t        transaction_df;
} vr_controller_state_t;

typedef struct {
    uint32_t lens_epoch;
    uint16_t lens_id;
    uint8_t  phase;
    uint8_t  busy;
    uint16_t prepare_250us;
    uint16_t resume_250us;
    uint16_t normal_offset_250us;
    uint16_t sport_offset_250us;
    uint16_t phase_remaining_250us;
    uint8_t  parameters_valid;
    uint8_t  state_valid;
} vr_driver_feedback_t;

typedef struct {
    vr_driver_feedback_t driver;
    uint64_t started_ticks;
    uint64_t last_tick_ticks;
    uint64_t deadline_ticks;
    uint16_t auxiliary_250us;
    uint16_t remaining_250us;
    uint32_t owner_epoch;
} vr_runtime_state_t;

/* =========================================================================
 * Aperture, Controls & EXIF Distance State
 * ========================================================================= */
typedef struct {
    uint16_t c_open_wide;
    uint16_t c_open_tele;
    uint16_t c_open_cur;
    uint16_t c_min_cur;
    uint16_t c_cur;
    uint16_t c_base_reported;    /* C_open in 'A' mode; C(P_ring) in manual ring mode */
    uint16_t cur_stopdown_delta; /* C_cur - C_open in 'A' mode; 0 in manual ring mode */
    uint16_t max_stopdown_delta; /* C_min - C_open in 'A' mode; 0 in manual ring mode */
    int16_t  body_a1_delta;      /* Signed 1/192 EV stopdown command from Z cmdA1 */
    uint16_t p_target_emount;    /* Target E-mount P written to ID03 wire[9..10] */
    uint16_t raw_p_open_08;      /* Wide max aperture P from CMD 08 payload[0..1] */
    uint16_t raw_p_min_08;       /* Min aperture P from CMD 08 payload[2..3] */
    uint16_t last_p_target;      /* Previous ID03 target P for toggle bit */
    uint8_t  id03_toggle;        /* Toggle bit written to ID03 wire[17] */
    uint16_t tuning_ref_2000;    /* Retained status tuning word for A0 subcommands */

    /* Synchronized EXIF & MF Distance Bar 4-gate fields */
    uint16_t z_dist_0x117;       /* Encoded focus distance word (max u16 at infinity) */
    uint16_t z_steps_0xbb;       /* 0 at inf, 1..255 finite */
    uint16_t z_alt_k_bucket;     /* 0 at inf, 1..255 finite distance bucket */
    uint8_t  z_mf_gate_2d;       /* Non-zero table pair T[trunc(2k/3)] */
    uint8_t  z_mf_gate_2e;
    uint16_t z_dnorm_0x31;       /* 0..4095 */
    uint16_t z_fnorm_0x33;       /* 0..4095 */
    uint16_t z_eff_focal_q4;     /* round(16.0f * f_eff_mm) */
    uint16_t z_mag_a3_q14;       /* round(16384.0f * beta) */
} aperture_exif_state_t;

/* =========================================================================
 * Lock-Free Double-Buffered Z Snapshot Bank (SRAM1)
 * ========================================================================= */
typedef struct {
    uint8_t cmd10[Z_CMD10_BYTES];
    uint8_t cmd11[Z_CMD11_BYTES];
    uint8_t cmd12[Z_CMD12_BYTES];
    uint8_t cmd20_00[Z_CMD20_00_BYTES];
    uint8_t cmd20_01[Z_CMD20_01_BYTES];
} z_fast_snapshots_t;

/* =========================================================================
 * Optics Large Table Workspace (Placed in SRAM2 on MCU)
 * ========================================================================= */
typedef struct {
    uint8_t  cmd24_sub00[2][Z_CMD24_A_STRIDE]; /* 14,856 B */
    uint8_t  cmd24_sub10[2][Z_CMD24_B_BYTES];  /*  6,936 B */
    uint8_t  sony_ds13_buf[E_SONY_DS13_MAX_BYTES]; /* 3,872 B */
    uint8_t  lf_work_buf[E_LF_WORKBUF_BYTES];  /* 36,864 B (36 KiB) */
    uint8_t  z_large_zero_buf[Z_LARGE_ZERO_BUF_BYTES]; /* 1,028 B */
    uint16_t sony_ds13_len;
    uint16_t sony_ds13_lid;
    uint8_t  sony_ds13_valid;
    uint8_t  active_optics_bank;
} optics_workspace_t;

typedef struct {
    uint16_t event_code;
    uint16_t lens_id;
    uint32_t arg0;
    uint32_t arg1;
    uint32_t arg2;
    uint32_t arg3;
} bb_event_input_t;

/* =========================================================================
 * Top-Level Adapter Session Context (Placed in SRAM1 on MCU)
 * ========================================================================= */
enum {
    CORE_DIAG_Q_CAP                = 8,
    RETRAM_WARM_HDR_EXPECT_BYTES   = 48,
    RETRAM_WARM_CACHE_EXPECT_BYTES = 3920,
};

typedef struct {
    uint32_t              session_epoch;
    e_session_state_t     session_state;
    e_handshake_step_t    hs_step;
    uint64_t              state_enter_ticks;
    uint32_t              vd_boot_start;

    /* Attached E-mount Lens Identity & Capabilities */
    uint16_t              e_lens_id;
    uint16_t              e_lens_ver;
    uint16_t              z_primary_lens_id;
    lens_brand_t          brand;
    af_actuator_class_t   af_class;
    char                  lens_name_ascii[LENS_NAME_ASCII_CAP];
    char                  vendor_tag_5b[VENDOR_TAG_BYTES];
    uint8_t               cap_bitmap[E_CAP_BITMAP_BYTES];
    uint8_t               tx07_raw[E_TX07_PAYLOAD_BYTES];
    uint8_t               tx08_raw[E_TX08_PAYLOAD_BYTES];

    uint16_t              f_min_mm;
    uint16_t              f_max_mm;
    uint16_t              f_cur_mm;
    uint8_t               lens_family;     /* CMD 07 payload[0] protocol family */
    uint8_t               id03_is_32b;
    uint8_t               has_ext_5a;      /* Extended capability negotiation active */
    uint8_t               init_sync_rem;   /* Remaining post-handshake ID04 sync frames */
    uint8_t               id03_frame_toggle;
    uint8_t               id04_idle_alt;
    uint8_t               vd_seq_latched;
    uint8_t               z_ring_heartbeat;
    uint8_t               is_apsc;
    uint8_t               is_macro;
    uint8_t               is_fisheye;
    uint8_t               is_dual_floating;
    uint8_t               supports_120hz;
    uint8_t               active_vsync_hz;
    uint32_t              uart_baud;
    uint8_t               tamron_session_mode; /* 2=setup/teardown, 1=active */
    uint8_t               e_tx_seq;
    uint8_t               pa3_debounce_cnt;
    uint64_t              last_ring_hb_ticks;

    /* Dataset 3A/3B streaming pagination state */
    uint16_t              lf_total_pages;
    uint16_t              lf_next_page;
    uint32_t              lf_total_bytes;
    uint16_t              divided_info_bits;

    /* Subsystem controllers */
    e_id05_state_t        id05;
    e_id06_state_t        id06;
    af_servo_state_t      af;
    vr_controller_state_t vr;
    aperture_exif_state_t ap;

    /* Retained 52B prefix of Z cmd20/00 MOSI */
    uint8_t               z_mosi20_retained[Z_MOSI20_PREFIX_BYTES];

    /* Lock-free double-buffered Z snapshots */
    z_fast_snapshots_t    z_banks[2];
    volatile uint8_t      active_z_bank;
    uint8_t               cmd15_stage;
    uint8_t               cmd15_polls;
    uint64_t              cmd15_open_ticks;

    /* Queued E-mount outgoing control commands (consumed per E4 / handshake) */
    e_tx_cmd_t            tx_queue[E_TX_QUEUE_CAP];
    uint8_t               tx_q_head;
    uint8_t               tx_q_tail;
    uint8_t               identity_ready;
    uint8_t               wake_ready;
    uint8_t               cmd01_retries;
    uint8_t               wake_retry_req;
    uint8_t               wake_from_park_req;
    uint8_t               wake_rehandshake;
    uint8_t               hs_cmd07_ok;
    uint8_t               park_after_handshake;
    uint8_t               af_limits_seen;
    uint64_t              hs_start_ticks;
    uint64_t              home_poll_ticks;
    uint64_t              last_emount_rx_ticks;
    vr_runtime_state_t    vr_runtime;
    uint32_t              vr_home_epoch;
    uint16_t              vr_home_lens_id;
    uint8_t               vr_stop_tx_phase;
    uint32_t              e_tx_serial;
    uint32_t              vr_stop_tx_serial;
    uint32_t              vr_photo_epoch;
    uint32_t              vr_photo_tx_serial;
    uint8_t               vr_photo_tx_phase;
    uint32_t              photo_owner_epoch;
    uint32_t              photo_owner_serial;
    uint64_t              photo_plan_start_ticks;
    uint8_t               photo_owner_phase;
    uint8_t               photo_release_flags;
    uint8_t               vr_id19_cap14;
    uint8_t               vr_id19_cap_valid;
    uint8_t               vr_cmd3d_negotiated;
    uint8_t               vr_photo_on_flags;
    uint8_t               vr_photo_off_flags;
    uint8_t               vr_stop_off_flags;
    uint32_t              vr_stop_epoch;
    int32_t               af_predicted_se;
    int32_t               af_drive_start_se;
    uint8_t               af_drive_moved;
    uint8_t               home_retry_stage;
    uint8_t               optics_ready;
    uint8_t               dist_anchor_valid;
    uint16_t              dist_anchor_x;
    int32_t               dist_anchor_cz;
    uint64_t              live_optics_wait_ticks;
    uint32_t              optics_built_epoch;
    uint16_t              optics_built_lid;
    uint8_t               optics_built_fisheye;
    uint8_t               vr_mode_settled;
    uint8_t               dist_anchor_pending;
    uint8_t               z_sck_powerdown_latched;
    uint8_t               home_completed_ok;
    uint8_t               home_retrigger_armed;
    uint8_t               cmd15_seen_since_arm;
    uint8_t               hs_tx08_done;
    uint8_t               optics_disconnected;
    uint8_t               cmd24_read_mask;
    uint8_t               dist_report_invalid;
    uint8_t               dist_verified_dynamic;
    uint8_t               dist_id05_samples;
    uint16_t              dist_first_x;
    int32_t               dist_min_se;
    int32_t               dist_max_se;
    uint16_t              diag_af_cmd_count;
    uint16_t              diag_af_stall_count;
    uint16_t              diag_z_hdr_err_count;
    uint8_t               bb_hs_id_logged;
    uint8_t               bb_e_err_logged;
    uint8_t               bb_z_err_logged;
    uint8_t               bb_stall_logged;
    uint8_t               core_diag_head;
    uint8_t               core_diag_tail;
    uint64_t              vr_mode_settle_ticks;
    bb_event_input_t      core_diag_q[CORE_DIAG_Q_CAP];
    af_v218_ext_t         af_ext;
    z_static_snap_cache_t z_snap_cache;
    int16_t               cached_db_idx;
    uint16_t              cached_db_lid;
    int16_t               dist_cam_bias_117;
    uint8_t               dist_cam_bias_valid;
    uint8_t               ctx_v220_pad;
    uint8_t               retram_warm_hit;
    uint8_t               ds13_cache_hit;
    uint16_t              first_ready_ms_x10;
    /* Tail state fields appended here to keep prior structure member offsets fixed */
    float                 parfocal_dist_m;
    int32_t               parfocal_last_cmd_se;
    uint32_t              af_configured_epoch;
    uint16_t              parfocal_last_f01;
    uint16_t              af_configured_lid;
    uint8_t               parfocal_hold_vsyncs;
    uint8_t               parfocal_cmd_valid;
    uint8_t               wake_boot_prologue_seen;
    int32_t               parfocal_cl_trim_cz;
    uint16_t              parfocal_anchor_dist_x;
    uint8_t               parfocal_cl_cooldown;
    uint8_t               parfocal_cl_steps;
    uint16_t              dist_last_f01;
    uint16_t              dist_last_zoom;
    uint16_t              parfocal_anchor_f01;
    uint16_t              parfocal_trim_f01;
    uint16_t              parfocal_min_rem_se;
    uint8_t               parfocal_anchor_refresh;
    uint8_t               parfocal_e0_coast_rem;
    uint8_t               id05_fresh_since_06;
} adapter_ctx_t;

/* =========================================================================
 * Blackbox (32B Records) & Dual-Sector Journal (128B Records)
 * ========================================================================= */
static const uint32_t BB_MAGIC_TAG           = 0x425A4242UL; /* "BZBB" */
static const uint16_t BB_EVT_BOOT_OK         = 0x0001U;
static const uint16_t BB_EVT_EMPTY_MOUNT_READY = 0x0002U;
static const uint16_t BB_EVT_E_BAUD_1M5      = 0x0003U;
static const uint16_t BB_EVT_E_LINK_UP       = 0x0004U;
static const uint16_t BB_EVT_E_120HZ_ACTIVE  = 0x0005U;
static const uint16_t BB_EVT_Z_FIRST_WIN     = 0x0006U;
static const uint16_t BB_EVT_HOTPLUG_DETACH  = 0x0007U;
static const uint16_t BB_EVT_SHUTDOWN_PARKED = 0x0008U;
static const uint16_t BB_EVT_RESET_SOURCE    = 0x0009U;
static const uint16_t BB_EVT_RESET_ROUTING   = 0x000AU;
static const uint16_t BB_EVT_HS_IDENTITY     = 0x000BU;
static const uint16_t BB_EVT_HS_TIMEOUT_FALLBACK = 0x000CU;
static const uint16_t BB_EVT_OPTICS_DISCONNECTED = 0x000DU;
static const uint16_t BB_EVT_DIST_FALLBACK   = 0x000EU;
static const uint16_t BB_EVT_SESSION_SUMMARY = 0x000FU;
static const uint16_t BB_EVT_PERF_OPTICS     = 0x0010U;
static const uint16_t BB_EVT_E_PROTO_ERR     = 0x00E1U;
static const uint16_t BB_EVT_Z_HDR_CSUM_ERR  = 0x00E2U;
static const uint16_t BB_EVT_AF_STALL_ABORT  = 0x00E3U;
static const uint16_t BB_EVT_HARDFAULT       = 0x00FFU;

static const uint32_t JOURNAL_MAGIC_TAG      = 0x425A4A4EUL; /* "BZJN" */
static const uint32_t JOURNAL_COMMIT_MARK    = 0x434F4D54UL; /* "COMT" */
static const uint32_t RETRAM_STATE_MAGIC     = 0x4252595AUL; /* "BRYZ" */
static const uint32_t RETRAM_WARM_MAGIC      = 0x425A5743UL; /* "BZWC" */

enum {
    RETRAM_DS13_CACHE_CAP = 3872,
};

typedef struct {
    uint32_t magic_tag;
    uint32_t ts_us;
    uint16_t event_code;
    uint16_t lens_id;
    uint32_t arg0;
    uint32_t arg1;
    uint32_t arg2;
    uint32_t arg3;
    uint32_t crc32;
} bb_record_t;

_Static_assert(sizeof(bb_record_t) == BLACKBOX_RECORD_BYTES, "bb_record_t must be 32 bytes");

typedef struct {
    uint32_t magic_tag;
    uint32_t version;
    uint64_t generation;
    uint16_t last_e_lens_id;
    uint16_t last_z_lens_id;
    int32_t  latched_e_inf;
    int32_t  latched_e_near;
    uint16_t last_focal_mm;
    uint16_t last_av_c;
    uint8_t  linear_mf_code;
    uint8_t  vr_last_mode;
    uint8_t  reserved_cfg[JOURNAL_RESERVED_BYTES];
    uint32_t payload_crc32;
    uint32_t commit_mark;
} journal_record_t;

_Static_assert(sizeof(journal_record_t) == JOURNAL_RECORD_BYTES, "journal_record_t must be 128 bytes");

typedef struct {
    uint32_t magic;
    uint32_t boot_count;
    uint16_t last_lens_id;
    uint16_t last_event;
    uint32_t fault_pc;
    uint32_t fault_lr;
    uint32_t fault_cfsr;
    uint32_t crc32;
} retram_state_t;

typedef struct {
    uint32_t magic;
    uint16_t bb_next_slot;
    uint16_t jnl_next_slot;
    uint64_t jnl_generation;
    uint32_t jnl_active_sector;
    int32_t  latched_e_inf;
    int32_t  latched_e_near;
    uint16_t cached_e_lens_id;
    uint16_t ds13_len;
    uint8_t  linear_mf_code;
    uint8_t  ds13_valid;
    uint16_t reserved_u16;
    uint32_t reserved_pad;
    uint32_t ds13_crc32;
    uint32_t hdr_crc32;
} retram_warm_hdr_t;

typedef struct {
    retram_warm_hdr_t hdr;
    uint32_t          ds13_words[RETRAM_DS13_CACHE_CAP / 4];
} retram_warm_cache_t;

_Static_assert(sizeof(retram_warm_hdr_t) == (size_t)RETRAM_WARM_HDR_EXPECT_BYTES, "retram_warm_hdr_t must be packed 48 bytes");
_Static_assert(sizeof(retram_warm_cache_t) == (size_t)RETRAM_WARM_CACHE_EXPECT_BYTES, "retram_warm_cache_t must be packed 3920 bytes");

typedef struct {
    uint32_t ts_ticks_lo;
    uint16_t tag;
    uint16_t len;
    uint32_t val0;
    uint32_t val1;
    uint32_t val2;
    uint32_t val3;
    uint32_t val4;
    uint32_t val5;
} diag_ring_entry_t;

typedef struct {
    uint8_t        cmd;
    uint8_t        sub;
    uint16_t       req_words;
    const uint8_t *mosi_buf;
    size_t         mosi_len;
    uint64_t       now_ticks;
} z_cmd_req_t;

/* Core Module Public APIs (Pure C17, Cross-Platform MCU + Host SIL) */
uint8_t boryoza_neg_sum8(const uint8_t *buf, size_t len_without_csum);
void boryoza_seal_z_frame(uint8_t *buf, size_t total_len);
uint16_t boryoza_sum16_emount(const uint8_t *wire, size_t wire_len);
size_t boryoza_wrap_emount_frame(uint8_t *out_wire, uint8_t frame_type, const uint8_t *seq_ptr,
                                 uint8_t cmd, const uint8_t *payload, size_t payload_len);
uint32_t boryoza_crc32_ieee(const void *data, size_t len);

/* lens_session.c */
void lens_session_init(adapter_ctx_t *ctx, optics_workspace_t *optics);
void lens_session_enter_empty_mount(adapter_ctx_t *ctx, optics_workspace_t *optics);
void lens_session_enter_detached(adapter_ctx_t *ctx, optics_workspace_t *optics);
void lens_session_enter_passive_mount(adapter_ctx_t *ctx, optics_workspace_t *optics);
void lens_session_force_active(adapter_ctx_t *ctx, optics_workspace_t *optics);
void lens_session_arm_home_retrigger(adapter_ctx_t *ctx);
bool lens_session_retrigger_homing(adapter_ctx_t *ctx, uint64_t now_ticks);
bool lens_session_initial_probe_exhausted(const adapter_ctx_t *ctx, uint64_t now_ticks);
void lens_session_start_mount_handshake(adapter_ctx_t *ctx, optics_workspace_t *optics, uint64_t now_ticks);
bool lens_session_cap_has(const adapter_ctx_t *ctx, uint8_t cmd);
bool lens_session_enqueue_tx(adapter_ctx_t *ctx, uint8_t cmd,
                             const uint8_t *payload, uint16_t payload_len);
bool lens_session_pop_tx(adapter_ctx_t *ctx, e_tx_cmd_t *out_cmd);
bool lens_session_unpack_id05(e_id05_state_t *state, const uint8_t *payload, size_t payload_len);
bool lens_session_unpack_id06(adapter_ctx_t *ctx, uint64_t now_ticks, const uint8_t *payload, size_t payload_len);
bool lens_session_on_emount_frame(adapter_ctx_t *ctx, optics_workspace_t *optics,
                                  uint64_t now_ticks, const uint8_t *wire, size_t wire_len);
void lens_session_step(adapter_ctx_t *ctx, optics_workspace_t *optics,
                       uint8_t pa3_mounted, const uint32_t *vd_count_ptr, uint64_t now_ticks);
size_t lens_session_build_id03_wire(adapter_ctx_t *ctx, uint8_t *out_wire, uint8_t seq);
size_t lens_session_build_id04_wire(adapter_ctx_t *ctx, uint8_t *out_wire, uint8_t seq);
void lens_session_push_diag(adapter_ctx_t *ctx, uint16_t code,
                            uint32_t arg0, uint32_t arg1, uint32_t arg2, uint32_t arg3);
bool lens_session_pop_diag(adapter_ctx_t *ctx, bb_event_input_t *out_evt);

/* af_servo_bridge.c */
void af_servo_configure_for_lens(adapter_ctx_t *ctx);
float af_servo_effective_mfd_meters(const adapter_ctx_t *ctx);
void af_servo_refresh_kz(adapter_ctx_t *ctx);
void af_servo_update_telemetry(adapter_ctx_t *ctx, uint64_t now_ticks);
void af_servo_on_standalone_id05(adapter_ctx_t *ctx, bool paired_after_06);
int32_t af_servo_se_to_cz(const adapter_ctx_t *ctx, int32_t pos_se);
int32_t af_servo_cz_to_se(const adapter_ctx_t *ctx, int32_t target_cz);
float af_servo_decode_z_speed_pps(uint8_t spd_lo, bool *out_unlimited, uint8_t spd_hi, int8_t *out_dir);
uint16_t af_servo_encode_kz_pair(float kz_val);
void af_servo_on_z_command(adapter_ctx_t *ctx, uint8_t cmd,
                           const uint8_t *mosi, size_t mosi_len);
void af_servo_emit_vsync_drive(adapter_ctx_t *ctx);
uint8_t af_servo_build_id04_subrecord(adapter_ctx_t *ctx, uint8_t *out_sub,
                                      size_t *out_sub_len, uint8_t *out_b16);
void af_servo_build_hotline_frame(adapter_ctx_t *ctx, uint8_t seq,
                                  uint8_t *out_hot42, uint64_t now_ticks);

/* vr_duty_controller.c */
void vr_controller_configure_for_lens(adapter_ctx_t *ctx);
void vr_controller_refresh_state(adapter_ctx_t *ctx);
bool vr_controller_on_driver_feedback(adapter_ctx_t *ctx, const vr_driver_feedback_t *feedback);
void vr_controller_on_initialization_result(adapter_ctx_t *ctx, bool completed);
void vr_controller_invalidate_driver(adapter_ctx_t *ctx);
void vr_controller_on_emount_tx_result(adapter_ctx_t *ctx, uint32_t lens_epoch,
                                       const uint8_t *wire, size_t wire_len,
                                       uint32_t serial, bool sent, uint64_t now_ticks);
bool vr_controller_step(adapter_ctx_t *ctx, uint64_t now_ticks);
void vr_controller_on_z_mosi20(adapter_ctx_t *ctx, const uint8_t *mosi, size_t mosi_len);
void vr_controller_on_z_command(adapter_ctx_t *ctx, uint8_t cmd,
                                const uint8_t *mosi, size_t mosi_len, const uint64_t *now_ticks_ptr);
void vr_controller_on_emount_ack(adapter_ctx_t *ctx, uint8_t ack_cmd, const uint64_t *now_ticks_ptr);
void vr_controller_step_parking(adapter_ctx_t *ctx, uint64_t now_ticks);
uint8_t vr_controller_poll_cmd51_status(adapter_ctx_t *ctx, uint64_t now_ticks);

/* aperture_controls.c */
uint16_t aperture_z_to_emount_p(uint16_t z_av_c);
uint16_t aperture_emount_p_to_z(uint16_t e_av_p);
uint16_t aperture_fnum_x100_to_z(uint16_t fnum_x100);
uint16_t exif_dist_x_to_z_0x117(uint16_t e_dist_x);
void aperture_controls_configure_for_lens(adapter_ctx_t *ctx);
void aperture_controls_validate_distance(adapter_ctx_t *ctx);
void aperture_controls_update_from_id05(adapter_ctx_t *ctx, const optics_workspace_t *optics);
void aperture_controls_on_z_a1(adapter_ctx_t *ctx, const uint8_t *mosi, size_t mosi_len);
void aperture_controls_fill_z_a0(const adapter_ctx_t *ctx, uint8_t sub, uint8_t *out_miso24);
void aperture_controls_on_z_mosi20(adapter_ctx_t *ctx, const uint8_t *mosi,
                                   size_t mosi_len, const uint64_t *now_ticks_ptr);

/* optics_xlate_fpu.c */
void optics_decode_sony_9b_16pt(const uint8_t packed9[E_SONY_PACKED9_BYTES],
                                uint8_t selector,
                                int16_t out16[E_SONY_CURVE16_PTS]);
void optics_decode_emount_9b(const uint8_t core6[E_ID05_CURVE_CORE_BYTES],
                             bool mask_bit7,
                             const uint8_t tail3[E_ID05_CURVE_TAIL_BYTES],
                             int32_t out_nodes8[E_CURVE_NODES_COUNT]);
void optics_build_initial_tables(adapter_ctx_t *ctx, optics_workspace_t *optics);
void optics_disconnect_post_init(adapter_ctx_t *ctx);
void optics_update_from_live_id05(adapter_ctx_t *ctx, optics_workspace_t *optics);
void optics_ingest_lf_page(adapter_ctx_t *ctx, optics_workspace_t *optics,
                           uint16_t page_idx, const uint8_t *page_data, size_t page_len);
float optics_eval_sony_ds13_focal_mm(const adapter_ctx_t *ctx, const optics_workspace_t *optics,
                                     float nominal_f_mm);
uint16_t optics_compute_a3_q14(const adapter_ctx_t *ctx, const optics_workspace_t *optics);

/* z_snapshot_builder.c */
void z_snapshot_publish(adapter_ctx_t *ctx, const optics_workspace_t *optics);
size_t z_dispatch_command_ram_safe(adapter_ctx_t *ctx, const optics_workspace_t *optics,
                                   const z_cmd_req_t *req, uint8_t *small_tx_buf,
                                   const uint8_t **out_direct_ptr);

#ifdef __cplusplus
}
#endif

#endif /* BORYOZA_TYPES_H */
