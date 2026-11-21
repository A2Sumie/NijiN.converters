#include "../../include/boryoza_types.h"
#include "../../include/boryoza_math_mem.h"

enum {
    ID0A_PARAM_LEN = 16,
};

static const uint8_t  Z_VR_CAP0_NONE       = 0x00U;
static const uint8_t  Z_VR_CAP1_NONE       = 0x44U;
static const uint8_t  Z_VR_CAP0_SPLIT_2_3  = 0x94U;
static const uint8_t  Z_VR_CAP1_SPLIT_2_3  = 0xC4U;

static const uint8_t  Z_DF_NON_VR          = 0x00U;
static const uint8_t  Z_DF_OFF             = 0x34U;
static const uint8_t  Z_DF_ACTIVE          = 0xB1U;
static const uint8_t  Z_DF_ACTIVE_READY    = 0x31U;
static const uint8_t  Z_DF_STANDBY         = 0xB4U;
static const uint8_t  Z_DF_STANDBY_READY   = 0x34U;
static const uint8_t  Z_DF_CONFIGURING     = 0xB8U;
static const uint8_t  Z_DF_CONFIG_READY    = 0x38U;
static const uint8_t  Z_DF_PREPARING       = 0xB2U;
static const uint8_t  Z_DF_PREPARED        = 0x32U;

static const uint16_t CAP_2PLUS3_LENS      = 12800U;
static const uint16_t CAP_0PLUS5_BODY      = 12000U;

static const uint8_t  E_ID03_OIS_OFF_B18   = 0x00U;
static const uint16_t E_ID03_OIS_OFF_W19   = 0x0033U;
static const uint8_t  E_ID03_OIS_OFF_B21   = 0x00U;

static const uint8_t  E_ID03_S1_B18        = 0x10U;
static const uint16_t E_ID03_PHOTO_W19      = 0x0003U;
static const uint16_t E_ID03_CAPTURE_W19    = 0x0004U;
static const uint16_t E_ID03_RECOVER_W19    = 0x000AU;
static const size_t   Z_MOSI20_RELEASE_OFFSET = 9U;
static const uint8_t  Z_MOSI20_RELEASE_MASK = 7U;
static const uint8_t  E_ID03_OIS_ENABLE     = 0x01U;
static const uint8_t  E_ID03_S1_GATE        = 0x02U;
static const uint8_t  E_ID03_MODE_SHIFT     = 2U;
static const uint8_t  OIS_MODE_INDEX_MAX    = 2U;

static const uint8_t  Z_CMD_HOTLINE_16     = 0x16U;
static const uint8_t  Z_CMD_PARK_50        = 0x50U;
static const uint8_t  Z_CMD_VR_D2          = 0xD2U;
static const uint8_t  Z_CMD_VR_D3          = 0xD3U;
static const uint8_t  Z_CMD_VR_D4          = 0xD4U;
static const uint8_t  Z_CMD_VR_D5          = 0xD5U;
static const uint8_t  Z_CMD_VR_D7          = 0xD7U;
static const uint8_t  D5_STOP_COMPLETE     = 0x03U;
static const uint8_t  D4_SUB_OFF_A3        = 0xA3U;
static const uint8_t  D4_SUB_STBY_E3       = 0xE3U;
static const uint8_t  D2_MODE_SPORT        = 0x04U;

static const uint8_t  CMD51_STATUS_BUSY         = 0x81U; /* Park shutdown still in progress */
static const uint8_t  CMD51_STATUS_READY        = 0x01U; /* Park shutdown completed */
static const uint64_t PARK_STEP_0A_TIMEOUT_TKS  = 120000ULL;  /* 40.0 ms @ 3.000 MHz */
static const uint64_t PARK_POST_16_SETTLE_TKS   = 360000ULL;  /* 120.0 ms post-lock ACK settle */
static const uint64_t PARK_MIN_TOTAL_TICKS      = 1440000ULL; /* 480.0 ms minimum park window */
static const uint64_t PARK_MAX_TIMEOUT_TICKS    = 1800000ULL; /* 600.0 ms safety upper bound */

static const uint8_t  E_CMD_ID0A                = 0x0AU;
static const uint8_t  E_CMD_ID03                = 0x03U;
static const uint8_t  E_CMD_ID16                = 0x16U;
static const uint8_t  E_CMD_ID19                = 0x19U;
static const size_t   E_WIRE_COMMAND_OFFSET     = 5U;
static const size_t   E_WIRE_PAYLOAD_OFFSET     = 6U;
static const size_t   E_STOP_WIRE_BYTES         = 10U;
static const size_t   E_ID03_W19_OFFSET         = 19U;
static const size_t   ID05_GRP_SWITCHES         = 7U;
static const size_t   D7_REQUEST_BYTES          = 10U;
static const uint64_t VR_QUARTER_MS_TICKS       = 750ULL;
static const uint64_t VR_MS_TICKS               = 3000ULL;
static const uint64_t VR_READY_PIPELINE_TICKS   = 1500ULL;
static const uint64_t VR_MODE_SETTLE_TICKS      = 15000ULL; /* 5.0 ms @ 3 MHz */
static const uint8_t  D3_MODE_SHIFT             = 6U;
static const uint8_t  D3_PROFILE_SHIFT          = 4U;
static const uint8_t  D3_PROFILE_MASK           = 3U;
static const uint8_t  D3_POLICY_MASK            = 15U;
static const uint8_t  D7_D3_ADMISSION_BIT        = 128U;
static const uint16_t VR_QUARTERS_PER_MS        = 4U;
static const uint8_t  VR_PHASE_PREDICATE[]       = {0U, 1U, 1U, 2U, 3U, 4U, 4U, 5U, 1U, 1U};

enum {
    VR_MODE_STOPPED = 0,
    VR_MODE_LIVE = 3,
    VR_MODE_D3_NORMAL = 4,
    VR_MODE_D3_SPORT = 5,
    VR_MODE_D3_ALTERNATE = 6,
    VR_MODE_D3_OFF = 7,
    VR_MODE_STANDBY = 8,
    VR_MODE_OFF = 9,
    VR_MODE_CONFIGURING = 11,
    VR_PHASE_COLD = 0,
    VR_PHASE_STARTING = 1,
    VR_PHASE_STOPPING = 2,
    VR_PHASE_READY = 3,
    VR_PHASE_TRANSITION = 4,
    VR_PHASE_RESETTING = 5,
    VR_DRIVER_READY_PHASE = 4,
    VR_GATE_DEADLINE = 0,
    VR_GATE_READY = 1,
    VR_GATE_HOLD = 2,
    VR_STOP_TX_IDLE = 0,
    VR_STOP_TX_PENDING = 1,
    VR_STOP_TX_QUEUED = 2,
    VR_STOP_TX_COMPLETE = 3,
    VR_PHOTO_IDLE = 0,
    VR_PHOTO_ON_PENDING = 1,
    VR_PHOTO_ON_QUEUED = 2,
    VR_PHOTO_ACTIVE = 3,
    VR_PHOTO_OFF_PENDING = 4,
    VR_PHOTO_OFF_QUEUED = 5,
    PHOTO_VIEW = 1,
    PHOTO_PLANNED = 2,
    PHOTO_CAPTURING = 3,
    PHOTO_RECOVERING = 4,
    PHOTO_STOPPED = 5,
    PHOTO_RESUMED = 6,
};

static void cancel_photo_transaction(adapter_ctx_t *ctx);
static void refresh_vr_mode(adapter_ctx_t *ctx);

static void transfer_photo_owner(adapter_ctx_t *ctx, uint8_t phase) {
    ctx->photo_owner_serial += 1U;
    if (ctx->photo_owner_serial == 0U) {
        ctx->photo_owner_serial = 1U;
    }
    ctx->photo_owner_epoch = ctx->session_epoch;
    ctx->photo_owner_phase = phase;
}

static void finish_photo_owner(adapter_ctx_t *ctx) {
    if ((ctx->photo_owner_epoch == ctx->session_epoch) &&
        ((ctx->photo_owner_phase == PHOTO_PLANNED) ||
         (ctx->photo_owner_phase == PHOTO_CAPTURING))) {
        transfer_photo_owner(ctx, PHOTO_RECOVERING);
    } else if (ctx->photo_owner_phase == PHOTO_STOPPED) {
        transfer_photo_owner(ctx, PHOTO_VIEW);
    }
}

static bool step_photo_owner(adapter_ctx_t *ctx, uint64_t now_ticks) {
    if ((ctx->photo_owner_epoch == ctx->session_epoch) &&
        (ctx->photo_owner_phase == PHOTO_PLANNED) &&
        (now_ticks >= ctx->photo_plan_start_ticks)) {
        ctx->photo_owner_phase = PHOTO_CAPTURING;
        vr_controller_refresh_state(ctx);
        return true;
    }
    return false;
}

static void refresh_photo_output(adapter_ctx_t *ctx) {
    if ((ctx->photo_owner_epoch != ctx->session_epoch) ||
        (ctx->session_state != E_SESSION_ACTIVE) ||
        (ctx->photo_owner_phase == PHOTO_STOPPED) ||
        (ctx->photo_owner_phase == 0U) ||
        ((ctx->photo_owner_phase == PHOTO_VIEW) && (ctx->vr.lens_has_ois != 0U))) {
        return;
    }
    uint16_t word = E_ID03_PHOTO_W19;
    if ((ctx->photo_owner_phase == PHOTO_PLANNED) ||
        (ctx->photo_owner_phase == PHOTO_CAPTURING)) {
        word = E_ID03_CAPTURE_W19;
    } else if (ctx->photo_owner_phase == PHOTO_RECOVERING) {
        word = E_ID03_RECOVER_W19;
    }
    ctx->vr.e_id03_w19 = word;
}

static void discard_queued_serial(adapter_ctx_t *ctx, uint32_t serial) {
    uint8_t read_idx = ctx->tx_q_head;
    uint8_t write_idx = read_idx;
    while (read_idx != ctx->tx_q_tail) {
        const e_tx_cmd_t *item = &ctx->tx_queue[read_idx];
        if (item->serial != serial) {
            if (write_idx != read_idx) {
                ctx->tx_queue[write_idx] = *item;
            }
            write_idx = (uint8_t)((write_idx + 1U) % E_TX_QUEUE_CAP);
        }
        read_idx = (uint8_t)((read_idx + 1U) % E_TX_QUEUE_CAP);
    }
    ctx->tx_q_tail = write_idx;
}

static void queue_stop_off(adapter_ctx_t *ctx) {
    if ((ctx->vr_stop_tx_phase == VR_STOP_TX_PENDING) &&
        (ctx->vr_stop_epoch == ctx->session_epoch) &&
        ((ctx->session_state == E_SESSION_ACTIVE) ||
         (ctx->session_state == E_SESSION_PARKING))) {
        const uint8_t flags = ctx->vr_stop_off_flags;
        uint8_t slot = ctx->tx_q_tail;
        if (lens_session_enqueue_tx(ctx, E_CMD_ID19, &flags, 1U)) {
            ctx->vr_stop_tx_serial = ctx->tx_queue[slot].serial;
            ctx->vr_stop_tx_phase = VR_STOP_TX_QUEUED;
        }
    }
}

static void request_stop(adapter_ctx_t *ctx) {
    bool photo_pending = (bool)((ctx->vr_photo_tx_phase != VR_PHOTO_IDLE) &&
                                (ctx->vr_photo_epoch == ctx->session_epoch));
    transfer_photo_owner(ctx, PHOTO_STOPPED);
    if ((ctx->vr.lens_has_ois != 0U) && (ctx->vr_stop_tx_phase == VR_STOP_TX_IDLE)) {
        ctx->vr_stop_off_flags = photo_pending ? ctx->vr_photo_off_flags : 0U;
        ctx->vr_stop_epoch = ctx->session_epoch;
        if (photo_pending || lens_session_cap_has(ctx, E_CMD_ID19)) {
            ctx->vr_stop_tx_phase = VR_STOP_TX_PENDING;
        } else {
            ctx->vr_stop_tx_phase = VR_STOP_TX_COMPLETE;
        }
    }
    cancel_photo_transaction(ctx);
    queue_stop_off(ctx);
    ctx->vr.internal_mode = VR_MODE_STOPPED;
    ctx->vr.stop_latched = 1U;
    ctx->vr_mode_settled = 0U;
    ctx->vr_mode_settle_ticks = 0ULL;
    vr_controller_refresh_state(ctx);
}

static void cancel_stop_transaction(adapter_ctx_t *ctx) {
    if (ctx->vr_stop_tx_phase == VR_STOP_TX_QUEUED) {
        discard_queued_serial(ctx, ctx->vr_stop_tx_serial);
    }
    ctx->vr_stop_tx_phase = VR_STOP_TX_IDLE;
    ctx->vr_stop_tx_serial = 0U;
    ctx->vr_stop_off_flags = 0U;
    ctx->vr_stop_epoch = 0U;
}

static void clear_completed_stop_transaction(adapter_ctx_t *ctx) {
    if ((ctx->vr_stop_tx_phase != VR_STOP_TX_PENDING) &&
        (ctx->vr_stop_tx_phase != VR_STOP_TX_QUEUED)) {
        cancel_stop_transaction(ctx);
    }
}

static void cancel_photo_transaction(adapter_ctx_t *ctx) {
    if ((ctx->vr_photo_tx_phase == VR_PHOTO_ON_QUEUED) ||
        (ctx->vr_photo_tx_phase == VR_PHOTO_OFF_QUEUED)) {
        discard_queued_serial(ctx, ctx->vr_photo_tx_serial);
    }
    ctx->vr_photo_tx_phase = VR_PHOTO_IDLE;
    ctx->vr_photo_tx_serial = 0U;
    ctx->vr_photo_epoch = 0U;
    ctx->vr_photo_on_flags = 0U;
    ctx->vr_photo_off_flags = 0U;
}

static void queue_photo_control(adapter_ctx_t *ctx) {
    uint8_t phase = ctx->vr_photo_tx_phase;
    if ((ctx->vr_photo_epoch != ctx->session_epoch) ||
        (ctx->session_state != E_SESSION_ACTIVE) ||
        ((phase != VR_PHOTO_ON_PENDING) && (phase != VR_PHOTO_OFF_PENDING))) {
        return;
    }
    uint8_t flags = (phase == VR_PHOTO_ON_PENDING)
                        ? ctx->vr_photo_on_flags : ctx->vr_photo_off_flags;
    uint8_t slot = ctx->tx_q_tail;
    if (lens_session_enqueue_tx(ctx, E_CMD_ID19, &flags, 1U)) {
        ctx->vr_photo_tx_serial = ctx->tx_queue[slot].serial;
        ctx->vr_photo_tx_phase = (phase == VR_PHOTO_ON_PENDING)
                                    ? VR_PHOTO_ON_QUEUED : VR_PHOTO_OFF_QUEUED;
    }
}

static void start_photo_transaction(adapter_ctx_t *ctx) {
    refresh_vr_mode(ctx);
    if ((ctx->vr.allow_id19 != 0U) &&
        (ctx->vr.internal_mode != VR_MODE_D3_OFF) &&
        lens_session_cap_has(ctx, E_CMD_ID19)) {
        cancel_stop_transaction(ctx);
        cancel_photo_transaction(ctx);
        ctx->vr_photo_epoch = ctx->session_epoch;
        ctx->vr_photo_on_flags = 3U;
        ctx->vr_photo_off_flags = 0U;
        ctx->vr_photo_tx_phase = VR_PHOTO_ON_PENDING;
        queue_photo_control(ctx);
    }
    vr_controller_refresh_state(ctx);
}

static void finish_photo_transaction(adapter_ctx_t *ctx) {
    finish_photo_owner(ctx);
    if ((ctx->vr_photo_tx_phase != VR_PHOTO_IDLE) &&
        (ctx->vr_photo_tx_phase <= VR_PHOTO_ACTIVE) &&
        (ctx->vr_photo_epoch == ctx->session_epoch)) {
        if (ctx->vr_photo_tx_phase == VR_PHOTO_ON_QUEUED) {
            discard_queued_serial(ctx, ctx->vr_photo_tx_serial);
        }
        ctx->vr_photo_tx_serial = 0U;
        ctx->vr_photo_tx_phase = VR_PHOTO_OFF_PENDING;
        queue_photo_control(ctx);
    }
}

void vr_controller_on_z_mosi20(adapter_ctx_t *ctx, const uint8_t *mosi, size_t mosi_len) {
    if ((mosi == NULL) || (mosi_len < (size_t)Z_MOSI20_PREFIX_BYTES) ||
        (ctx->session_state != E_SESSION_ACTIVE)) {
        return;
    }
    uint8_t flags = (uint8_t)((uint32_t)mosi[Z_MOSI20_RELEASE_OFFSET] & Z_MOSI20_RELEASE_MASK);
    if (flags != 0U) {
        ctx->v224.af_or_shutter_seen = 1U;
    }
    if ((ctx->photo_release_flags != 0U) && (flags == 0U)) {
        finish_photo_transaction(ctx);
    }
    ctx->photo_release_flags = flags;
}

static bool photo_restore_command(uint8_t cmd, const uint8_t *mosi, size_t mosi_len) {
    if (cmd == Z_CMD_VR_D2) {
        return mosi_len >= 3U;
    }
    if (cmd == Z_CMD_VR_D7) {
        return mosi_len >= D7_REQUEST_BYTES;
    }
    return (bool)((cmd == Z_CMD_VR_D4) && (mosi_len >= 3U) &&
                  ((mosi[1] == D4_SUB_OFF_A3) || (mosi[1] == D4_SUB_STBY_E3)));
}

static void enqueue_park_id0a(adapter_ctx_t *ctx, uint64_t now_ticks) {
    const uint8_t payload[ID0A_PARAM_LEN] = {0U};
    if (lens_session_enqueue_tx(ctx, E_CMD_ID0A, payload, (uint16_t)sizeof(payload))) {
        ctx->vr.park_step_ticks = now_ticks;
        ctx->vr.park_state = PARK_WAIT_ID0A_ACK;
    }
}

static void retain_stop_queue_entry(adapter_ctx_t *ctx) {
    e_tx_cmd_t item;
    e_tx_cmd_t stop_item;
    bool found = false;
    while (lens_session_pop_tx(ctx, &item)) {
        if ((ctx->vr_stop_tx_phase == VR_STOP_TX_QUEUED) &&
            (item.serial == ctx->vr_stop_tx_serial)) {
            stop_item = item;
            found = true;
        }
    }
    ctx->tx_q_head = 0U;
    ctx->tx_q_tail = 0U;
    if (found) {
        ctx->tx_queue[0] = stop_item;
        ctx->tx_q_tail = 1U;
    }
}

static bool d3_owns_status(const vr_controller_state_t *vr_ctl) {
    return (bool)((vr_ctl->internal_mode >= VR_MODE_D3_NORMAL) &&
                  (vr_ctl->internal_mode <= VR_MODE_D3_OFF));
}

static bool driver_state_matches(const adapter_ctx_t *ctx) {
    const vr_driver_feedback_t *driver = &ctx->vr_runtime.driver;
    return (bool)((driver->state_valid != 0U) &&
                  (driver->lens_epoch == ctx->session_epoch) &&
                  (driver->lens_id == ctx->e_lens_id) &&
                  (driver->lens_id != 0U));
}

static bool driver_feedback_matches(const adapter_ctx_t *ctx) {
    return (bool)(driver_state_matches(ctx) && (ctx->vr_runtime.driver.parameters_valid != 0U));
}

bool vr_controller_on_driver_feedback(adapter_ctx_t *ctx, const vr_driver_feedback_t *feedback) {
    if ((feedback == NULL) || (feedback->lens_epoch != ctx->session_epoch) ||
        (feedback->lens_id != ctx->e_lens_id) || (feedback->lens_id == 0U) ||
        ((size_t)feedback->phase >= sizeof(VR_PHASE_PREDICATE)) ||
        ((feedback->parameters_valid != 0U) && (feedback->prepare_250us == 0U) &&
         ((feedback->state_valid == 0U) ||
          (feedback->phase != VR_DRIVER_READY_PHASE)))) {
        return false;
    }
    ctx->vr_runtime.driver = *feedback;
    return true;
}

void vr_controller_invalidate_driver(adapter_ctx_t *ctx) {
    ctx->vr_home_epoch = 0U;
    ctx->vr_home_lens_id = 0U;
    ctx->vr_runtime.driver.parameters_valid = 0U;
    ctx->vr_runtime.driver.state_valid = 0U;
}

void vr_controller_on_initialization_result(adapter_ctx_t *ctx, bool completed) {
    vr_controller_invalidate_driver(ctx);
    if (!completed || (ctx->e_lens_id == 0U)) {
        return;
    }
    ctx->vr_home_epoch = ctx->session_epoch;
    ctx->vr_home_lens_id = ctx->e_lens_id;
    /* Completed actuator initialization needs no additional startup wait. */
    const vr_driver_feedback_t feedback = {
        .lens_epoch = ctx->session_epoch,
        .lens_id = ctx->e_lens_id,
        .phase = VR_DRIVER_READY_PHASE,
        .busy = 0U,
        .parameters_valid = 1U,
        .state_valid = 1U,
    };
    (void)vr_controller_on_driver_feedback(ctx, &feedback);
}

static bool complete_photo_control_frame(adapter_ctx_t *ctx, uint32_t lens_epoch,
                                          const uint8_t *wire, size_t wire_len,
                                          uint32_t serial, bool sent) {
    if (!sent || (wire_len < E_ID03_STD_BYTES) ||
        (wire[E_WIRE_COMMAND_OFFSET] != E_CMD_ID03) ||
        (ctx->photo_owner_epoch != lens_epoch) ||
        (serial != ctx->photo_owner_serial) ||
        (wire[E_ID03_W19_OFFSET + 1U] != 0U)) {
        return false;
    }
    if ((ctx->photo_owner_phase == PHOTO_RECOVERING) &&
        (wire[E_ID03_W19_OFFSET] == (uint8_t)E_ID03_RECOVER_W19)) {
        ctx->photo_owner_phase = PHOTO_RESUMED;
    } else if ((ctx->photo_owner_phase == PHOTO_RESUMED) &&
               (wire[E_ID03_W19_OFFSET] == (uint8_t)E_ID03_PHOTO_W19)) {
        ctx->photo_owner_phase = PHOTO_VIEW;
    } else {
        return false;
    }
    vr_controller_refresh_state(ctx);
    return true;
}

static bool complete_photo_tx(adapter_ctx_t *ctx, uint32_t lens_epoch,
                              const uint8_t *wire, size_t wire_len,
                              uint32_t serial, bool sent) {
    if ((wire_len == E_STOP_WIRE_BYTES) &&
        (wire[E_WIRE_COMMAND_OFFSET] == E_CMD_ID19) &&
        (ctx->vr_photo_epoch == lens_epoch) &&
        (serial == ctx->vr_photo_tx_serial) && (serial != 0U)) {
        uint8_t phase = ctx->vr_photo_tx_phase;
        bool starting = phase == VR_PHOTO_ON_QUEUED;
        uint8_t flags = starting ? ctx->vr_photo_on_flags : ctx->vr_photo_off_flags;
        if ((starting || (phase == VR_PHOTO_OFF_QUEUED)) &&
            (wire[E_WIRE_PAYLOAD_OFFSET] == flags)) {
            ctx->vr_photo_tx_serial = 0U;
            if (sent) {
                ctx->vr_photo_tx_phase = starting ? VR_PHOTO_ACTIVE : VR_PHOTO_IDLE;
                if (!starting) {
                    ctx->vr_photo_epoch = 0U;
                }
            } else {
                ctx->vr_photo_tx_phase = starting ? VR_PHOTO_ON_PENDING : VR_PHOTO_OFF_PENDING;
            }
            vr_controller_refresh_state(ctx);
            return true;
        }
    }
    return false;
}

void vr_controller_on_emount_tx_result(adapter_ctx_t *ctx, uint32_t lens_epoch,
                                       const uint8_t *wire, size_t wire_len,
                                       uint32_t serial, bool sent, uint64_t now_ticks) {
    if ((lens_epoch != ctx->session_epoch) || (wire == NULL)) {
        return;
    }
    if (complete_photo_control_frame(ctx, lens_epoch, wire, wire_len, serial, sent)) {
        return;
    }
    if (complete_photo_tx(ctx, lens_epoch, wire, wire_len, serial, sent)) {
        return;
    }
    if ((ctx->session_state == E_SESSION_PARKING) &&
        (ctx->vr.park_state == PARK_WAIT_VR_STOP) &&
        (ctx->vr_stop_tx_phase == VR_STOP_TX_COMPLETE) && sent &&
        (wire_len >= E_ID03_STD_BYTES) && (wire[E_WIRE_COMMAND_OFFSET] == E_CMD_ID03) &&
        (wire[E_ID03_W19_OFFSET] == (uint8_t)E_ID03_OIS_OFF_W19) &&
        (wire[E_ID03_W19_OFFSET + 1U] == 0U)) {
        enqueue_park_id0a(ctx, now_ticks);
        return;
    }
    if ((wire_len != E_STOP_WIRE_BYTES) ||
        (wire[E_WIRE_COMMAND_OFFSET] != E_CMD_ID19) ||
        (wire[E_WIRE_PAYLOAD_OFFSET] != ctx->vr_stop_off_flags) ||
        (ctx->vr_stop_epoch != lens_epoch) ||
        (ctx->vr_stop_tx_phase != VR_STOP_TX_QUEUED) ||
        (serial != ctx->vr_stop_tx_serial) || (serial == 0U)) {
        return;
    }
    ctx->vr_stop_tx_phase = sent ? VR_STOP_TX_COMPLETE : VR_STOP_TX_PENDING;
    ctx->vr_stop_tx_serial = 0U;
    vr_controller_refresh_state(ctx);
}

static bool vr_handshake_in_progress(const adapter_ctx_t *ctx) {
    return (bool)((ctx->session_state == E_SESSION_POWERUP_WAIT) ||
                  (ctx->session_state == E_SESSION_HANDSHAKE));
}

static bool try_settle_vr_mode_ready(adapter_ctx_t *ctx, uint64_t now_ticks) {
    vr_controller_state_t *vr_ctl = &ctx->vr;
    vr_runtime_state_t *runtime = &ctx->vr_runtime;
    if (vr_handshake_in_progress(ctx) || (vr_ctl->lens_has_ois == 0U) ||
        (ctx->vr_mode_settled != 0U) || (ctx->vr_mode_settle_ticks == 0ULL)) {
        return false;
    }
    bool settleable = (bool)((vr_ctl->internal_mode == VR_MODE_LIVE) ||
                             (vr_ctl->internal_mode == VR_MODE_STANDBY) ||
                             (vr_ctl->internal_mode == VR_MODE_OFF) ||
                             (vr_ctl->internal_mode == VR_MODE_CONFIGURING));
    if (!settleable) {
        return false;
    }
    uint8_t predicate = VR_PHASE_PREDICATE[runtime->driver.phase];
    if (!driver_state_matches(ctx) || (predicate != VR_PHASE_READY) ||
        (runtime->driver.busy != 0U) || (now_ticks < ctx->vr_mode_settle_ticks) ||
        ((now_ticks - ctx->vr_mode_settle_ticks) < VR_MODE_SETTLE_TICKS)) {
        return false;
    }
    ctx->vr_mode_settled = 1U;
    if (vr_ctl->internal_mode == VR_MODE_CONFIGURING) {
        vr_ctl->transaction_df = Z_DF_CONFIG_READY;
    }
    vr_controller_refresh_state(ctx);
    return true;
}

bool vr_controller_step(adapter_ctx_t *ctx, uint64_t now_ticks) {
    queue_stop_off(ctx);
    queue_photo_control(ctx);
    bool photo_changed = step_photo_owner(ctx, now_ticks);
    if (try_settle_vr_mode_ready(ctx, now_ticks)) {
        photo_changed = true;
    }
    vr_controller_state_t *vr_ctl = &ctx->vr;
    vr_runtime_state_t *runtime = &ctx->vr_runtime;
    if (!d3_owns_status(vr_ctl) || (runtime->owner_epoch != ctx->session_epoch) ||
        (now_ticks < runtime->last_tick_ticks)) {
        return photo_changed;
    }
    static const uint32_t VR_STEPS_SAT_MAX = 65535UL;
    uint64_t elapsed_ticks = now_ticks - runtime->last_tick_ticks;
    uint32_t steps = 0UL;
    if (elapsed_ticks >= ((uint64_t)VR_STEPS_SAT_MAX * VR_QUARTER_MS_TICKS)) {
        steps = VR_STEPS_SAT_MAX;
        runtime->last_tick_ticks = now_ticks;
    } else {
        uint32_t step_ticks = (uint32_t)VR_QUARTER_MS_TICKS;
        steps = (uint32_t)elapsed_ticks / step_ticks;
        if (steps == 0UL) {
            return photo_changed;
        }
        runtime->last_tick_ticks += (uint64_t)steps * VR_QUARTER_MS_TICKS;
    }
    runtime->remaining_250us = (steps >= runtime->remaining_250us)
                                  ? 0U : (uint16_t)(runtime->remaining_250us - steps);
    if (vr_ctl->internal_mode != VR_MODE_D3_OFF) {
        runtime->auxiliary_250us = (steps >= runtime->auxiliary_250us)
                                      ? 0U : (uint16_t)(runtime->auxiliary_250us - steps);
    }
    bool done = false;
    if ((vr_ctl->internal_mode != VR_MODE_D3_SPORT) &&
        (vr_ctl->phase_gate == VR_GATE_DEADLINE)) {
        done = now_ticks >= runtime->deadline_ticks;
    } else if ((vr_ctl->internal_mode == VR_MODE_D3_NORMAL) ||
               (vr_ctl->internal_mode == VR_MODE_D3_SPORT) ||
               (vr_ctl->phase_gate == VR_GATE_READY)) {
        uint8_t predicate = VR_PHASE_PREDICATE[runtime->driver.phase];
        done = (bool)(driver_state_matches(ctx) && (predicate == VR_PHASE_READY) &&
                      ((now_ticks - runtime->started_ticks) >= VR_READY_PIPELINE_TICKS));
    }
    if (done && (runtime->driver.busy == 0U) &&
        (vr_ctl->transaction_df == Z_DF_PREPARING)) {
        vr_ctl->transaction_df = Z_DF_PREPARED;
        vr_controller_refresh_state(ctx);
        return true;
    }
    return photo_changed;
}

static void restore_steady_z_vr_mode(adapter_ctx_t *ctx, const vr_controller_state_t *saved_vr,
                                     uint8_t saved_settled, const uint64_t *saved_ticks_ptr) {
    vr_controller_state_t *vr_ctl = &ctx->vr;
    uint64_t saved_settle_ticks = *saved_ticks_ptr;
    vr_ctl->body_mode = saved_vr->body_mode;
    vr_ctl->stop_latched = saved_vr->stop_latched;
    vr_ctl->d7_flags = saved_vr->d7_flags;
    if (d3_owns_status(saved_vr) || (saved_vr->internal_mode == VR_MODE_CONFIGURING)) {
        if (saved_vr->body_mode == VR_BODY_CMD_OFF_A3) {
            vr_ctl->internal_mode = VR_MODE_OFF;
        } else if (saved_vr->body_mode == VR_BODY_CMD_STANDBY_E3) {
            vr_ctl->internal_mode = VR_MODE_STANDBY;
        } else {
            vr_ctl->internal_mode = VR_MODE_LIVE;
        }
        vr_ctl->transaction_df = 0U;
        ctx->vr_mode_settled = 0U;
        ctx->vr_mode_settle_ticks = (saved_settle_ticks != 0ULL) ? saved_settle_ticks : 1ULL;
        return;
    }
    vr_ctl->internal_mode = saved_vr->internal_mode;
    vr_ctl->transaction_df = 0U;
    ctx->vr_mode_settled = saved_settled;
    ctx->vr_mode_settle_ticks = saved_settle_ticks;
}

void vr_controller_configure_for_lens(adapter_ctx_t *ctx) {
    vr_controller_state_t *vr_ctl = &ctx->vr;
    bool preserve_z_mode = (bool)((ctx->session_epoch != 0U) &&
                                  (ctx->photo_owner_epoch == ctx->session_epoch) &&
                                  (vr_ctl->internal_mode != VR_MODE_STOPPED));
    vr_controller_state_t saved_vr = *vr_ctl;
    uint8_t saved_settled = ctx->vr_mode_settled;
    uint64_t saved_settle_ticks = ctx->vr_mode_settle_ticks;
    uint8_t has_ois = vr_ctl->lens_has_ois;
    uint8_t has_sw = vr_ctl->lens_has_ois_switch;
    uint8_t hot_en = vr_ctl->hotline_enabled;

    memset(vr_ctl, 0, sizeof(*vr_ctl));
    memset(&ctx->vr_runtime, 0, sizeof(ctx->vr_runtime));
    cancel_stop_transaction(ctx);
    cancel_photo_transaction(ctx);
    transfer_photo_owner(ctx, PHOTO_VIEW);
    ctx->photo_plan_start_ticks = 0ULL;
    ctx->photo_release_flags = 0U;
    if ((ctx->vr_home_epoch == ctx->session_epoch) &&
        (ctx->vr_home_lens_id == ctx->e_lens_id) && (ctx->e_lens_id != 0U)) {
        vr_controller_on_initialization_result(ctx, true);
    } else {
        vr_controller_invalidate_driver(ctx);
    }
    vr_ctl->lens_has_ois = has_ois;
    vr_ctl->lens_has_ois_switch = has_sw;
    vr_ctl->hotline_enabled = hot_en;
    vr_ctl->park_state = PARK_IDLE;
    if (preserve_z_mode) {
        restore_steady_z_vr_mode(ctx, &saved_vr, saved_settled, &saved_settle_ticks);
    } else {
        vr_ctl->body_mode = VR_BODY_CMD_STANDBY_E3;
        vr_ctl->stop_latched = 1U;
        ctx->vr_mode_settled = 0U;
        ctx->vr_mode_settle_ticks = 0ULL;
    }

    if (has_ois != 0U) {
        vr_ctl->z_vr_cap0 = Z_VR_CAP0_SPLIT_2_3;
        vr_ctl->z_vr_cap1 = Z_VR_CAP1_SPLIT_2_3;
    } else {
        vr_ctl->z_vr_cap0 = Z_VR_CAP0_NONE;
        vr_ctl->z_vr_cap1 = Z_VR_CAP1_NONE;
    }
    vr_controller_refresh_state(ctx);
}

static void refresh_vr_mode(adapter_ctx_t *ctx) {
    vr_controller_state_t *vr_ctl = &ctx->vr;

    if (vr_ctl->lens_has_ois == 0U) {
        vr_ctl->z_vr_cap0 = Z_VR_CAP0_NONE;
        vr_ctl->z_vr_cap1 = Z_VR_CAP1_NONE;
        vr_ctl->z_df_state = Z_DF_NON_VR;
        vr_ctl->cap_lens_lv = 0U;
        vr_ctl->cap_lens_exp = 0U;
        vr_ctl->cap_body_lv = 0U;
        vr_ctl->cap_body_exp = 0U;
        vr_ctl->e_id03_b18 = E_ID03_OIS_OFF_B18;
        vr_ctl->e_id03_w19 = E_ID03_OIS_OFF_W19;
        vr_ctl->e_id03_b21 = E_ID03_OIS_OFF_B21;
        vr_ctl->allow_id19 = 0U;
        return;
    }

    vr_ctl->z_vr_cap0 = Z_VR_CAP0_SPLIT_2_3;
    vr_ctl->z_vr_cap1 = Z_VR_CAP1_SPLIT_2_3;

    if (vr_ctl->body_mode == VR_BODY_CMD_OFF_A3) {
        vr_ctl->z_df_state = (ctx->vr_mode_settled != 0U) ? Z_DF_STANDBY_READY : Z_DF_STANDBY;
        vr_ctl->cap_lens_lv = 0U;
        vr_ctl->cap_lens_exp = 0U;
        vr_ctl->cap_body_lv = 0U;
        vr_ctl->cap_body_exp = 0U;
        vr_ctl->e_id03_b18 = E_ID03_OIS_OFF_B18;
        vr_ctl->e_id03_w19 = E_ID03_OIS_OFF_W19;
        vr_ctl->e_id03_b21 = E_ID03_OIS_OFF_B21;
        vr_ctl->allow_id19 = 0U;
        return;
    }

    bool barrel_forced_off = false;
    if ((vr_ctl->lens_has_ois_switch != 0U) &&
        (ctx->id05.grp_present[ID05_GRP_SWITCHES] != 0U) &&
        (ctx->id05.barrel_ois_on == 0U)) {
        barrel_forced_off = true;
    }
    if (barrel_forced_off) {
        vr_ctl->z_df_state = Z_DF_OFF;
        vr_ctl->cap_lens_lv = 0U;
        vr_ctl->cap_lens_exp = 0U;
        vr_ctl->cap_body_lv = CAP_0PLUS5_BODY;
        vr_ctl->cap_body_exp = CAP_0PLUS5_BODY;
        vr_ctl->e_id03_b18 = E_ID03_OIS_OFF_B18;
        vr_ctl->e_id03_w19 = E_ID03_OIS_OFF_W19;
        vr_ctl->e_id03_b21 = E_ID03_OIS_OFF_B21;
        vr_ctl->allow_id19 = 0U;
        return;
    }

    if (vr_ctl->body_mode == VR_BODY_CMD_STANDBY_E3) {
        vr_ctl->z_df_state = (ctx->vr_mode_settled != 0U) ? Z_DF_STANDBY_READY : Z_DF_STANDBY;
        vr_ctl->cap_lens_lv = CAP_2PLUS3_LENS;
        vr_ctl->cap_lens_exp = CAP_2PLUS3_LENS;
        vr_ctl->cap_body_lv = 0U;
        vr_ctl->cap_body_exp = 0U;
        vr_ctl->e_id03_b18 = E_ID03_OIS_OFF_B18;
        vr_ctl->e_id03_w19 = E_ID03_OIS_OFF_W19;
        vr_ctl->e_id03_b21 = E_ID03_OIS_OFF_B21;
        vr_ctl->allow_id19 = 0U;
        return;
    }

    vr_ctl->z_df_state = (ctx->vr_mode_settled != 0U) ? Z_DF_ACTIVE_READY : Z_DF_ACTIVE;
    vr_ctl->cap_lens_lv = CAP_2PLUS3_LENS;
    vr_ctl->cap_lens_exp = CAP_2PLUS3_LENS;
    vr_ctl->cap_body_lv = 0U;
    vr_ctl->cap_body_exp = 0U;
    vr_ctl->allow_id19 = 1U;

    uint8_t lens_mode = 1U;
    if ((vr_ctl->lens_has_ois_switch != 0U) &&
        (ctx->id05.grp_present[ID05_GRP_SWITCHES] != 0U) &&
        (ctx->id05.barrel_ois_mode <= OIS_MODE_INDEX_MAX)) {
        lens_mode = (uint8_t)(ctx->id05.barrel_ois_mode + 1U);
    }
    vr_ctl->e_id03_b18 = E_ID03_S1_B18;
    vr_ctl->e_id03_w19 = E_ID03_PHOTO_W19;
    vr_ctl->e_id03_b21 = (uint8_t)(E_ID03_OIS_ENABLE |
                                 ((uint32_t)lens_mode << E_ID03_MODE_SHIFT));
    if (vr_ctl->body_mode != VR_BODY_CMD_SPORT) {
        vr_ctl->e_id03_b21 = (uint8_t)(vr_ctl->e_id03_b21 | E_ID03_S1_GATE);
    }
}

void vr_controller_refresh_state(adapter_ctx_t *ctx) {
    uint8_t previous_b18 = ctx->vr.e_id03_b18;
    uint16_t previous_w19 = ctx->vr.e_id03_w19;
    uint8_t previous_b21 = ctx->vr.e_id03_b21;
    refresh_vr_mode(ctx);
    if ((ctx->vr.lens_has_ois != 0U) &&
        (d3_owns_status(&ctx->vr) || (ctx->vr.internal_mode == VR_MODE_CONFIGURING))) {
        ctx->vr.z_df_state = ctx->vr.transaction_df;
    }
    if ((ctx->vr_stop_tx_phase == VR_STOP_TX_PENDING) ||
        (ctx->vr_stop_tx_phase == VR_STOP_TX_QUEUED) ||
        (ctx->vr_photo_tx_phase == VR_PHOTO_OFF_PENDING) ||
        (ctx->vr_photo_tx_phase == VR_PHOTO_OFF_QUEUED)) {
        ctx->vr.e_id03_b18 = previous_b18;
        ctx->vr.e_id03_w19 = previous_w19;
        ctx->vr.e_id03_b21 = previous_b21;
        ctx->vr.allow_id19 = 0U;
    } else if ((ctx->vr.stop_latched != 0U) &&
        ((ctx->vr_stop_tx_phase == VR_STOP_TX_IDLE) ||
         (ctx->vr_stop_tx_phase == VR_STOP_TX_COMPLETE))) {
        ctx->vr.e_id03_b18 = E_ID03_OIS_OFF_B18;
        ctx->vr.e_id03_w19 = E_ID03_OIS_OFF_W19;
        ctx->vr.e_id03_b21 = E_ID03_OIS_OFF_B21;
        ctx->vr.allow_id19 = 0U;
    }
    refresh_photo_output(ctx);
}

static void accept_d3_if_ready(adapter_ctx_t *ctx, uint64_t now_ticks,
                                const uint8_t *mosi, size_t mosi_len) {
    vr_controller_state_t *vr_ctl = &ctx->vr;
    vr_runtime_state_t *runtime = &ctx->vr_runtime;
    if (vr_handshake_in_progress(ctx) || (mosi_len < D7_REQUEST_BYTES) ||
        (vr_ctl->lens_has_ois == 0U) || d3_owns_status(vr_ctl) ||
        !driver_feedback_matches(ctx) ||
        ((vr_ctl->internal_mode == VR_MODE_CONFIGURING) &&
         (((uint32_t)vr_ctl->d7_flags & D7_D3_ADMISSION_BIT) == 0U))) {
        return;
    }
    uint8_t mode = (uint8_t)(VR_MODE_D3_NORMAL + ((uint32_t)mosi[1] >> D3_MODE_SHIFT));
    uint8_t predicate = VR_PHASE_PREDICATE[runtime->driver.phase];
    uint8_t gate = VR_GATE_DEADLINE;
    uint16_t remaining = (uint16_t)((uint16_t)mosi[3] * VR_QUARTERS_PER_MS);
    if ((predicate == VR_PHASE_COLD) || (predicate == VR_PHASE_RESETTING)) {
        if (remaining < runtime->driver.prepare_250us) {
            return;
        }
    } else if ((predicate == VR_PHASE_STARTING) || (predicate == VR_PHASE_STOPPING)) {
        gate = VR_GATE_READY;
    } else if (predicate == VR_PHASE_TRANSITION) {
        if (mode >= VR_MODE_D3_ALTERNATE) {
            gate = VR_GATE_HOLD;
        } else {
            uint32_t minimum = (uint32_t)runtime->driver.resume_250us +
                               runtime->driver.phase_remaining_250us;
            if ((uint32_t)remaining < minimum) {
                return;
            }
            gate = VR_GATE_READY;
        }
    }
    uint16_t offset = ((mode == VR_MODE_D3_SPORT) || (gate != VR_GATE_DEADLINE))
                          ? runtime->driver.sport_offset_250us
                          : runtime->driver.normal_offset_250us;
    runtime->started_ticks = now_ticks;
    runtime->last_tick_ticks = now_ticks;
    runtime->deadline_ticks = now_ticks + ((uint64_t)mosi[3] * VR_MS_TICKS);
    runtime->remaining_250us = remaining;
    runtime->auxiliary_250us = (uint16_t)(remaining - offset);
    runtime->owner_epoch = ctx->session_epoch;
    vr_ctl->internal_mode = mode;
    vr_ctl->profile_field = (uint8_t)(((uint32_t)mosi[1] >> D3_PROFILE_SHIFT) & D3_PROFILE_MASK);
    vr_ctl->policy_field = (uint8_t)((uint32_t)mosi[1] & D3_POLICY_MASK);
    vr_ctl->phase_gate = gate;
    vr_ctl->transaction_df = Z_DF_PREPARING;
    vr_ctl->stop_latched = 0U;
    ctx->vr_mode_settled = 0U;
    ctx->vr_mode_settle_ticks = 0ULL;
    transfer_photo_owner(ctx, PHOTO_PLANNED);
    ctx->photo_plan_start_ticks = runtime->deadline_ticks;
    start_photo_transaction(ctx);
}

static void enqueue_park_id16(adapter_ctx_t *ctx, uint64_t now_ticks) {
    static const uint8_t pay16[] = {0x01U};
    (void)lens_session_enqueue_tx(ctx, E_CMD_ID16, pay16, (uint16_t)sizeof(pay16));
    ctx->vr.park_step_ticks = now_ticks;
    ctx->vr.park_state = PARK_WAIT_ID16_ACK;
}

static void start_shutdown_parking_sequence(adapter_ctx_t *ctx, uint64_t now_ticks) {
    if ((ctx->session_state == E_SESSION_PARKING) ||
        (ctx->session_state == E_SESSION_PARKED) ||
        (ctx->park_after_handshake != 0U)) {
        return;
    }
    vr_controller_state_t *vr_ctl = &ctx->vr;
    ctx->v224.park_completed_by_timeout = 0U;
    ctx->af.af_drive_active = 0U;
    ctx->af.af_cmd_pending = 0U;
    ctx->af.af_stop_req = 0U;
    ctx->af.af_submode = 0U;
    vr_ctl->hotline_enabled = 0U;
    ctx->wake_from_park_req = 0U;
    ctx->wake_boot_prologue_seen = 0U;
    vr_ctl->park_start_ticks = now_ticks;
    vr_ctl->park_step_ticks = now_ticks;

    if ((ctx->session_state == E_SESSION_POWERUP_WAIT) ||
        (ctx->session_state == E_SESSION_HANDSHAKE)) {
        ctx->park_after_handshake = 1U;
        return;
    }
    if (ctx->session_state != E_SESSION_ACTIVE) {
        vr_ctl->park_state = PARK_COMPLETE;
        return;
    }

    ctx->session_state = E_SESSION_PARKING;
    vr_controller_invalidate_driver(ctx);
    ctx->v224.drive_ep_start_t256 = 0UL;
    retain_stop_queue_entry(ctx);
    request_stop(ctx);
    if (vr_ctl->lens_has_ois != 0U) {
        vr_ctl->park_state = PARK_WAIT_VR_STOP;
    } else {
        enqueue_park_id0a(ctx, now_ticks);
    }
}

static void arm_vr_mode_settle(adapter_ctx_t *ctx, uint64_t now_ticks) {
    clear_completed_stop_transaction(ctx);
    ctx->vr.stop_latched = 0U;
    ctx->vr_mode_settled = 0U;
    ctx->vr_mode_settle_ticks = (now_ticks != 0ULL) ? now_ticks : 1ULL;
}

static void apply_z_vr_mode_cmd(adapter_ctx_t *ctx, uint8_t cmd,
                                const uint8_t *mosi, size_t mosi_len,
                                const uint64_t *now_ticks_ptr) {
    uint64_t now_ticks = *now_ticks_ptr;
    vr_controller_state_t *vr_ctl = &ctx->vr;
    if ((cmd == Z_CMD_VR_D4) && (mosi_len >= 3U) &&
        ((mosi[1] == D4_SUB_OFF_A3) || (mosi[1] == D4_SUB_STBY_E3))) {
        arm_vr_mode_settle(ctx, now_ticks);
        if (mosi[1] == D4_SUB_OFF_A3) {
            vr_ctl->internal_mode = VR_MODE_OFF;
            vr_ctl->body_mode = VR_BODY_CMD_OFF_A3;
        } else {
            vr_ctl->internal_mode = VR_MODE_STANDBY;
            vr_ctl->body_mode = VR_BODY_CMD_STANDBY_E3;
        }
        vr_controller_refresh_state(ctx);
        return;
    }

    if ((cmd == Z_CMD_VR_D2) && (mosi_len >= 3U)) {
        arm_vr_mode_settle(ctx, now_ticks);
        vr_ctl->internal_mode = VR_MODE_LIVE;
        uint8_t mode_byte = mosi[1];
        if (mode_byte == D2_MODE_SPORT) {
            vr_ctl->body_mode = VR_BODY_CMD_SPORT;
        } else if (mode_byte == 0x00U) {
            vr_ctl->body_mode = VR_BODY_CMD_OFF_A3;
        } else {
            vr_ctl->body_mode = VR_BODY_CMD_NORMAL;
        }
        vr_controller_refresh_state(ctx);
        return;
    }

    if ((cmd == Z_CMD_VR_D7) && (mosi_len >= D7_REQUEST_BYTES)) {
        arm_vr_mode_settle(ctx, now_ticks);
        vr_ctl->d7_flags = mosi[1];
        vr_ctl->internal_mode = VR_MODE_CONFIGURING;
        vr_ctl->transaction_df = Z_DF_CONFIGURING;
        vr_controller_refresh_state(ctx);
    }
}

static void handle_vr_hotline_enable_cmd(adapter_ctx_t *ctx, vr_controller_state_t *vr_ctl,
                                         const uint8_t *mosi, size_t mosi_len) {
    if ((mosi_len < 2U) || (((uint32_t)mosi[1] & 0x01UL) == 0UL)) {
        return;
    }
    if ((ctx->session_state == E_SESSION_PARKING) ||
        (ctx->session_state == E_SESSION_PARKED)) {
        ctx->session_state = E_SESSION_PARKED;
        ctx->wake_from_park_req = 2U;
        ctx->wake_boot_prologue_seen = 1U;
    }
    ctx->z_sck_powerdown_latched = 0U;
    vr_ctl->hotline_enabled = 1U;
    vr_ctl->park_state = PARK_IDLE;
    if ((ctx->session_state == E_SESSION_ACTIVE) && (ctx->optics_ready != 0U)) {
        optics_disconnect_post_init(ctx);
    }
}

static void update_parked_d2_body_mode(vr_controller_state_t *vr_ctl, const uint8_t *mosi) {
    uint8_t mode_byte = mosi[1];
    if (mode_byte == D2_MODE_SPORT) {
        vr_ctl->body_mode = VR_BODY_CMD_SPORT;
    } else if (mode_byte == 0x00U) {
        vr_ctl->body_mode = VR_BODY_CMD_OFF_A3;
    } else {
        vr_ctl->body_mode = VR_BODY_CMD_NORMAL;
    }
}

void vr_controller_on_z_command(adapter_ctx_t *ctx, uint8_t cmd,
                                const uint8_t *mosi, size_t mosi_len,
                                const uint64_t *now_ticks_ptr) {
    vr_controller_state_t *vr_ctl = &ctx->vr;
    uint64_t now_ticks = (now_ticks_ptr != NULL) ? *now_ticks_ptr : 0ULL;
    (void)vr_controller_step(ctx, now_ticks);

    if (cmd == Z_CMD_VR_D3) {
        accept_d3_if_ready(ctx, now_ticks, mosi, mosi_len);
        return;
    }
    if (cmd == Z_CMD_HOTLINE_16) {
        handle_vr_hotline_enable_cmd(ctx, vr_ctl, mosi, mosi_len);
        return;
    }
    if (cmd == Z_CMD_PARK_50) {
        start_shutdown_parking_sequence(ctx, now_ticks);
        return;
    }
    if ((ctx->session_state == E_SESSION_PARKING) ||
        (ctx->session_state == E_SESSION_PARKED)) {
        if ((cmd == Z_CMD_VR_D2) && (mosi_len >= 3U)) {
            update_parked_d2_body_mode(vr_ctl, mosi);
        }
        return;
    }
    if (photo_restore_command(cmd, mosi, mosi_len)) {
        finish_photo_transaction(ctx);
    }
    if ((cmd == Z_CMD_VR_D5) && (mosi_len >= 3U) && (mosi[1] == D5_STOP_COMPLETE)) {
        request_stop(ctx);
        return;
    }
    apply_z_vr_mode_cmd(ctx, cmd, mosi, mosi_len, &now_ticks);
}

void vr_controller_on_emount_ack(adapter_ctx_t *ctx, uint8_t ack_cmd, const uint64_t *now_ticks_ptr) {
    uint64_t now_ticks = (now_ticks_ptr != NULL) ? *now_ticks_ptr : 0ULL;
    vr_controller_state_t *vr_ctl = &ctx->vr;
    if ((ack_cmd == E_CMD_ID0A) && (vr_ctl->park_state == PARK_WAIT_ID0A_ACK)) {
        enqueue_park_id16(ctx, now_ticks);
        return;
    }
    if ((ack_cmd == E_CMD_ID16) &&
        ((vr_ctl->park_state == PARK_WAIT_ID16_ACK) || (vr_ctl->park_state == PARK_WAIT_ID0A_ACK))) {
        vr_ctl->park_step_ticks = now_ticks;
        vr_ctl->park_state = PARK_SETTLE_HOLD;
    }
}

void vr_controller_step_parking(adapter_ctx_t *ctx, uint64_t now_ticks) {
    if (ctx->park_after_handshake != 0U) {
        if (ctx->session_state == E_SESSION_ACTIVE) {
            ctx->park_after_handshake = 0U;
            start_shutdown_parking_sequence(ctx, now_ticks);
        }
        return;
    }
    vr_controller_state_t *vr_ctl = &ctx->vr;
    if ((vr_ctl->park_state == PARK_IDLE) || (vr_ctl->park_state == PARK_COMPLETE)) {
        return;
    }
    uint64_t total_elapsed = (now_ticks >= vr_ctl->park_start_ticks)
                                 ? (now_ticks - vr_ctl->park_start_ticks)
                                 : 0ULL;
    uint64_t step_elapsed = (now_ticks >= vr_ctl->park_step_ticks)
                                ? (now_ticks - vr_ctl->park_step_ticks)
                                : 0ULL;

    if ((vr_ctl->park_state == PARK_WAIT_ID0A_ACK) &&
        (step_elapsed >= PARK_STEP_0A_TIMEOUT_TKS)) {
        enqueue_park_id16(ctx, now_ticks);
        return;
    }
    if ((vr_ctl->park_state == PARK_SETTLE_HOLD) &&
        (step_elapsed >= PARK_POST_16_SETTLE_TKS) &&
        (total_elapsed >= PARK_MIN_TOTAL_TICKS)) {
        vr_ctl->park_state = PARK_COMPLETE;
        ctx->session_state = E_SESSION_PARKED;
        return;
    }
    if (total_elapsed >= PARK_MAX_TIMEOUT_TICKS) {
        cancel_stop_transaction(ctx);
        ctx->v224.park_completed_by_timeout = 1U;
        vr_ctl->park_state = PARK_COMPLETE;
        ctx->session_state = E_SESSION_PARKED;
    }
}

uint8_t vr_controller_poll_cmd51_status(adapter_ctx_t *ctx, uint64_t now_ticks) {
    vr_controller_step_parking(ctx, now_ticks);
    if (ctx->park_after_handshake != 0U) {
        return CMD51_STATUS_BUSY;
    }
    vr_controller_state_t *vr_ctl = &ctx->vr;
    if ((vr_ctl->park_state == PARK_IDLE) || (vr_ctl->park_state == PARK_COMPLETE)) {
        return CMD51_STATUS_READY;
    }
    return CMD51_STATUS_BUSY;
}
