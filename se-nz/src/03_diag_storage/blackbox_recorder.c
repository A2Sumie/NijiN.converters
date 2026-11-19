#include "../../include/hc32f460_regs.h"
#include "../../include/boryoza_math_mem.h"

enum {
    WORDS_PER_BB_REC      = BLACKBOX_RECORD_BYTES / 4,
    BB_PENDING_CAP        = 32,
    BB_ROLLOVER_KEEP      = 8,
    RETRAM_STATE_WORDS    = sizeof(retram_state_t) / 4,
    RETRAM_WARM_HDR_WORDS = sizeof(retram_warm_hdr_t) / 4,
    RETRAM_DS13_WORDS     = RETRAM_DS13_CACHE_CAP / 4,
    DS13_CHUNK_WORDS      = 32,
    BB_CRC_NIB_LUT_LEN    = 16,
};

static const uint32_t BB_ERASED_U32     = 0xFFFFFFFFUL;
static const uint32_t CRC32_INIT_U32    = 0xFFFFFFFFUL;
static const uint32_t BYTES_IN_WORD     = 4UL;
static const uint32_t NIBBLE_LO_MASK    = 0x0FUL;
static const uint32_t BYTE_LO_MASK      = 0xFFUL;
static const uint32_t ALIGN32_ROUND_ADD = 3UL;
static const uint32_t SHIFT_4_BITS      = 4U;
static const uint32_t SHIFT_8_BITS      = 8U;
static const uint32_t RETRAM_HASH_WORDS = 6UL;
static const uint32_t BB_HASH_WORDS     = 7UL;
static const uint32_t WARM_HASH_WORDS   = (uint32_t)RETRAM_WARM_HDR_WORDS - 1UL;
static const uint32_t SHIFT_16_BITS     = 16U;
static const uint32_t AIRCR_SYSRESETREQ = 0x05FA0004UL;
static const uint32_t CFSR_BFAR_VALID   = 0x00008000UL;
static const uint32_t CFSR_MMAR_VALID   = 0x00000080UL;
static const uint32_t HF_IDX_LR         = 5UL;
static const uint32_t HF_IDX_PC         = 6UL;
static const size_t   BB_WIDX_CFSR      = 5U;
static const size_t   BB_WIDX_ZERO      = 6U;
static const size_t   BB_WIDX_CRC       = 7U;

BORYOZA_SRAM3_BUF static volatile diag_ring_entry_t s_diag_ring[DIAG_RING_ENTRIES];
BORYOZA_RETRAM_BUF static retram_state_t      s_retram_state;
BORYOZA_RETRAM_BUF static retram_warm_cache_t s_retram_warm;

/* NOLINTBEGIN(readability-magic-numbers) — IEEE 802.3 CRC32 half-byte lookup table */
static uint32_t s_crc32_nib_lut[BB_CRC_NIB_LUT_LEN] = {
    0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
    0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
    0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
    0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL,
};
/* NOLINTEND(readability-magic-numbers) */

static bb_record_t s_bb_pending[BB_PENDING_CAP];
static uint32_t    s_bb_rollover_keep[BB_ROLLOVER_KEEP][WORDS_PER_BB_REC];
static uint8_t     s_bb_pend_head      = 0U;
static uint8_t     s_bb_pend_count     = 0U;
static uint16_t    s_bb_next_slot      = 0U;
static uint16_t    s_diag_ring_idx     = 0U;
static uint8_t     s_bb_flash_degraded = 0U;
static uint8_t     s_retram_warm_hit   = 0U;

BORYOZA_RAMFUNC static uint32_t crc32_update_words(uint32_t crc_state, const uint32_t *words,
                                                   uint32_t word_count) {
    uint32_t crc = crc_state;
    for (uint32_t widx = 0U; widx < word_count; ++widx) {
        uint32_t val = words[widx];
        for (uint32_t bidx = 0U; bidx < BYTES_IN_WORD; ++bidx) {
            uint32_t byte_val = (val >> (bidx * SHIFT_8_BITS)) & BYTE_LO_MASK;
            crc = (crc >> SHIFT_4_BITS) ^ s_crc32_nib_lut[(crc ^ byte_val) & NIBBLE_LO_MASK];
            crc = (crc >> SHIFT_4_BITS) ^ s_crc32_nib_lut[(crc ^ (byte_val >> SHIFT_4_BITS)) & NIBBLE_LO_MASK];
        }
    }
    return crc;
}

BORYOZA_RAMFUNC static uint32_t crc32_ramfunc_safe(const uint32_t *words, uint32_t word_count) {
    return ~crc32_update_words(CRC32_INIT_U32, words, word_count);
}

BORYOZA_RAMFUNC static void retram_read_words(uint32_t *dst_words, const volatile uint32_t *src_words,
                                              uint32_t word_count) {
    for (uint32_t idx = 0U; idx < word_count; ++idx) {
        dst_words[idx] = src_words[idx];
    }
}

BORYOZA_RAMFUNC static void retram_write_words(volatile uint32_t *dst_words, const uint32_t *src_words,
                                               uint32_t word_count) {
    for (uint32_t idx = 0U; idx < word_count; ++idx) {
        dst_words[idx] = src_words[idx];
    }
}

BORYOZA_RAMFUNC static void retram_state_load(retram_state_t *dst) {
    uint32_t words[RETRAM_STATE_WORDS];
    retram_read_words(words, (const volatile uint32_t *)(const volatile void *)&s_retram_state,
                      (uint32_t)RETRAM_STATE_WORDS);
    memcpy(dst, words, sizeof(*dst));
}

BORYOZA_RAMFUNC static void retram_state_store(retram_state_t *state) {
    uint32_t words[RETRAM_STATE_WORDS];
    memcpy(words, state, sizeof(words));
    words[RETRAM_STATE_WORDS - 1U] = crc32_ramfunc_safe(words, RETRAM_HASH_WORDS);
    state->crc32 = words[RETRAM_STATE_WORDS - 1U];
    retram_write_words((volatile uint32_t *)(void *)&s_retram_state, words, (uint32_t)RETRAM_STATE_WORDS);
}

__attribute__((noinline)) static void retram_warm_hdr_load(retram_warm_hdr_t *dst) {
    uint32_t words[RETRAM_WARM_HDR_WORDS];
    retram_read_words(words, (const volatile uint32_t *)(const volatile void *)&s_retram_warm.hdr,
                      (uint32_t)RETRAM_WARM_HDR_WORDS);
    memcpy(dst, words, sizeof(*dst));
}

__attribute__((noinline)) static void retram_warm_hdr_store(retram_warm_hdr_t *warm_hdr) {
    uint32_t words[RETRAM_WARM_HDR_WORDS];
    memcpy(words, warm_hdr, sizeof(words));
    words[RETRAM_WARM_HDR_WORDS - 1U] = crc32_ramfunc_safe(words, WARM_HASH_WORDS);
    warm_hdr->hdr_crc32 = words[RETRAM_WARM_HDR_WORDS - 1U];
    retram_write_words((volatile uint32_t *)(void *)&s_retram_warm.hdr, words,
                       (uint32_t)RETRAM_WARM_HDR_WORDS);
}

static bool bb_slot_is_erased(uint32_t slot) {
    uint32_t addr = HC32_FLASH_BLACKBOX_BASE + (slot * (uint32_t)BLACKBOX_RECORD_BYTES);
    for (uint32_t widx = 0U; widx < (uint32_t)WORDS_PER_BB_REC; ++widx) {
        if (mmio_r32(addr + (widx * BYTES_IN_WORD)) != BB_ERASED_U32) {
            return false;
        }
    }
    return true;
}

static uint16_t scan_bb_next_slot_reverse(void) {
    uint16_t suffix_start = (uint16_t)BLACKBOX_SLOTS;
    for (int32_t slot = (int32_t)BLACKBOX_SLOTS - 1; slot >= 0; --slot) {
        if (!bb_slot_is_erased((uint32_t)slot)) {
            break;
        }
        suffix_start = (uint16_t)slot;
    }
    return suffix_start;
}

static bool bb_slot_boundary_ok(uint16_t next_slot) {
    if (next_slot > (uint16_t)BLACKBOX_SLOTS) {
        return false;
    }
    if ((next_slot < (uint16_t)BLACKBOX_SLOTS) && !bb_slot_is_erased((uint32_t)next_slot)) {
        return false;
    }
    if ((next_slot < (uint16_t)(BLACKBOX_SLOTS - 1U)) && !bb_slot_is_erased((uint32_t)BLACKBOX_SLOTS - 1U)) {
        return false;
    }
    if (next_slot > 0U) {
        uint32_t prev_addr = HC32_FLASH_BLACKBOX_BASE +
                             ((uint32_t)(next_slot - 1U) * (uint32_t)BLACKBOX_RECORD_BYTES);
        if (mmio_r32(prev_addr) == BB_ERASED_U32) {
            return false;
        }
    }
    return true;
}

void blackbox_init(void) {
    s_diag_ring_idx = 0U;
    s_bb_pend_head = 0U;
    s_bb_pend_count = 0U;
    s_bb_flash_degraded = 0U;
    s_retram_warm_hit = 0U;

    bool is_warm_reset = ((g_boot_reset_flags & RMU_RSTF0_PORF_BIT) == 0UL);
    retram_warm_hdr_t local_hdr;
    retram_warm_hdr_load(&local_hdr);
    uint32_t expected_warm_crc = crc32_ramfunc_safe((const uint32_t *)(const void *)&local_hdr,
                                                      WARM_HASH_WORDS);
    bool warm_crc_ok = (bool)((local_hdr.magic == RETRAM_WARM_MAGIC) &&
                              (local_hdr.hdr_crc32 == expected_warm_crc));
    if (is_warm_reset && warm_crc_ok && bb_slot_boundary_ok(local_hdr.bb_next_slot)) {
        s_bb_next_slot = local_hdr.bb_next_slot;
        s_retram_warm_hit = 1U;
    } else {
        s_bb_next_slot = scan_bb_next_slot_reverse();
        memset(&local_hdr, 0, sizeof(local_hdr));
        local_hdr.magic = RETRAM_WARM_MAGIC;
        local_hdr.bb_next_slot = s_bb_next_slot;
        retram_warm_hdr_store(&local_hdr);
    }

    retram_state_t local_state;
    retram_state_load(&local_state);
    uint32_t expected_crc = crc32_ramfunc_safe((const uint32_t *)(const void *)&local_state,
                                               RETRAM_HASH_WORDS);
    if ((local_state.magic != RETRAM_STATE_MAGIC) || (local_state.crc32 != expected_crc)) {
        memset(&local_state, 0, sizeof(local_state));
        local_state.magic = RETRAM_STATE_MAGIC;
        local_state.boot_count = 1U;
    } else {
        local_state.boot_count += 1U;
    }
    retram_state_store(&local_state);
}

static void diag_ring_push_evt(const bb_event_input_t *evt) {
    uint32_t ring_mask = (uint32_t)DIAG_RING_ENTRIES - 1UL;
    uint32_t idx = s_diag_ring_idx;
    s_diag_ring_idx = (uint16_t)((idx + 1UL) & ring_mask);
    volatile diag_ring_entry_t *ent = &s_diag_ring[idx];
    ent->ts_ticks_lo = (uint32_t)tmra1_now_ticks();
    ent->tag = evt->event_code;
    ent->len = evt->lens_id;
    ent->val0 = evt->arg0;
    ent->val1 = evt->arg1;
    ent->val2 = evt->arg2;
    ent->val3 = evt->arg3;
    ent->val4 = 0U;
    ent->val5 = 0U;
}

int blackbox_record_event(const bb_event_input_t *evt) {
    diag_ring_push_evt(evt);

    retram_state_t local_state;
    retram_state_load(&local_state);
    local_state.last_lens_id = evt->lens_id;
    local_state.last_event = evt->event_code;
    retram_state_store(&local_state);

    bb_record_t *rec = NULL;
    if (s_bb_pend_count >= (uint8_t)BB_PENDING_CAP) {
        uint8_t slot_idx = s_bb_pend_head;
        s_bb_pend_head = (uint8_t)((s_bb_pend_head + 1U) % (uint8_t)BB_PENDING_CAP);
        rec = &s_bb_pending[slot_idx];
    } else {
        uint8_t slot_idx = (uint8_t)(((uint32_t)s_bb_pend_head + (uint32_t)s_bb_pend_count) %
                                     (uint32_t)BB_PENDING_CAP);
        s_bb_pend_count = (uint8_t)(s_bb_pend_count + 1U);
        rec = &s_bb_pending[slot_idx];
    }

    rec->magic_tag = BB_MAGIC_TAG;
    rec->ts_us = tmra1_now_us32();
    rec->event_code = evt->event_code;
    rec->lens_id = evt->lens_id;
    rec->arg0 = evt->arg0;
    rec->arg1 = evt->arg1;
    rec->arg2 = evt->arg2;
    rec->arg3 = evt->arg3;
    rec->crc32 = 0U;
    return 0;
}

BORYOZA_RAMFUNC static bool blackbox_rollover_sector(void) {
    uint32_t keep_cnt = 0U;
    uint32_t start_slot = ((uint32_t)BLACKBOX_SLOTS > (uint32_t)BB_ROLLOVER_KEEP)
                              ? ((uint32_t)BLACKBOX_SLOTS - (uint32_t)BB_ROLLOVER_KEEP)
                              : 0UL;
    for (uint32_t slot = start_slot; slot < (uint32_t)BLACKBOX_SLOTS; ++slot) {
        uint32_t src_addr = HC32_FLASH_BLACKBOX_BASE + (slot * (uint32_t)BLACKBOX_RECORD_BYTES);
        for (uint32_t widx = 0U; widx < (uint32_t)WORDS_PER_BB_REC; ++widx) {
            s_bb_rollover_keep[keep_cnt][widx] = mmio_r32(src_addr + (widx * BYTES_IN_WORD));
        }
        if ((s_bb_rollover_keep[keep_cnt][0] == BB_MAGIC_TAG) &&
            (crc32_ramfunc_safe(s_bb_rollover_keep[keep_cnt], BB_HASH_WORDS) ==
             s_bb_rollover_keep[keep_cnt][BB_WIDX_CRC])) {
            keep_cnt += 1U;
        }
    }
    if (efm_erase_sector_ramfunc(HC32_FLASH_BLACKBOX_BASE) != 0) {
        return false;
    }
    uint32_t written_cnt = 0U;
    for (uint32_t kidx = 0U; kidx < keep_cnt; ++kidx) {
        uint32_t dst_addr = HC32_FLASH_BLACKBOX_BASE + (kidx * (uint32_t)BLACKBOX_RECORD_BYTES);
        bool slot_ok = true;
        for (uint32_t widx = 0U; widx < (uint32_t)WORDS_PER_BB_REC; ++widx) {
            if (efm_program_word_ramfunc(dst_addr + (widx * BYTES_IN_WORD),
                                         s_bb_rollover_keep[kidx][widx]) != 0) {
                slot_ok = false;
                break;
            }
        }
        if (!slot_ok) {
            s_bb_next_slot = (uint16_t)kidx + 1U;
            return false;
        }
        written_cnt = kidx + 1U;
    }
    s_bb_next_slot = (uint16_t)written_cnt;
    return bb_slot_is_erased((uint32_t)s_bb_next_slot);
}

static bool program_bb_record_at(uint16_t slot_idx, const bb_record_t *rec) {
    uint32_t slot_addr = HC32_FLASH_BLACKBOX_BASE +
                         ((uint32_t)slot_idx * (uint32_t)BLACKBOX_RECORD_BYTES);
    uint32_t words[WORDS_PER_BB_REC];
    memcpy(words, rec, sizeof(words));
    words[WORDS_PER_BB_REC - 1] = crc32_ramfunc_safe(words, BB_HASH_WORDS);
    for (uint32_t widx = 0U; widx < (uint32_t)WORDS_PER_BB_REC; ++widx) {
        if (efm_program_word_ramfunc(slot_addr + (widx * BYTES_IN_WORD), words[widx]) != 0) {
            s_bb_flash_degraded = 1U;
            return false;
        }
    }
    return true;
}

static bool write_one_pending_record(const bb_record_t *rec, bool *rolled_over) {
    while (s_bb_next_slot < (uint16_t)BLACKBOX_SLOTS) {
        uint16_t slot_idx = s_bb_next_slot;
        s_bb_next_slot = (uint16_t)(s_bb_next_slot + 1U);
        if (!bb_slot_is_erased((uint32_t)slot_idx)) {
            continue;
        }
        return program_bb_record_at(slot_idx, rec);
    }
    if (*rolled_over || !blackbox_rollover_sector()) {
        s_bb_flash_degraded = 1U;
        return false;
    }
    *rolled_over = true;
    if (s_bb_next_slot >= (uint16_t)BLACKBOX_SLOTS) {
        s_bb_flash_degraded = 1U;
        return false;
    }
    uint16_t slot_idx = s_bb_next_slot;
    s_bb_next_slot = (uint16_t)(s_bb_next_slot + 1U);
    return program_bb_record_at(slot_idx, rec);
}

void blackbox_flush_to_flash(void) {
    if (s_bb_pend_count == 0U) {
        return;
    }
    if (s_bb_flash_degraded != 0U) {
        s_bb_pend_head = 0U;
        s_bb_pend_count = 0U;
        return;
    }

    bool rolled_over = false;
    for (uint8_t pidx = 0U; pidx < s_bb_pend_count; ++pidx) {
        uint8_t buf_idx = (uint8_t)(((uint32_t)s_bb_pend_head + (uint32_t)pidx) %
                                    (uint32_t)BB_PENDING_CAP);
        if (!write_one_pending_record(&s_bb_pending[buf_idx], &rolled_over)) {
            break;
        }
    }
    s_bb_pend_head = 0U;
    s_bb_pend_count = 0U;
    retram_warm_cache_update_bb_slot(s_bb_next_slot);
}

BORYOZA_RAMFUNC static bool fault_frame_readable(const uint32_t *stack_frame, uint32_t cfsr) {
    static const uintptr_t SRAM_END = 0x20027000UL;
    static const uintptr_t FRAME_BYTES = 32UL;
    static const uint32_t STACK_ERROR_MASK = 0x00001010UL;
    uintptr_t addr = (uintptr_t)stack_frame;
    if (((addr & 3UL) != 0UL) || ((cfsr & STACK_ERROR_MASK) != 0UL)) {
        return false;
    }
    if ((addr >= HC32_SRAMH_BASE) && (addr <= SRAM_END - FRAME_BYTES)) {
        return true;
    }
    return false;
}

BORYOZA_RAMFUNC void boryoza_hardfault_c_handler(const uint32_t *stack_frame) {
    static volatile uint8_t s_in_hardfault = 0U;
    gpio_pa9_set_busy();
    gpio_pa2_set_low();
    if (s_in_hardfault != 0U) {
        mmio_w32(SCB_AIRCR_ADDR, AIRCR_SYSRESETREQ);
        for (;;) {
            cpu_nop();
        }
    }
    s_in_hardfault = 1U;

    uint32_t cfsr = mmio_r32(SCB_CFSR_ADDR);
    bool frame_valid = fault_frame_readable(stack_frame, cfsr);
    uint32_t fault_pc = frame_valid ? stack_frame[HF_IDX_PC] : 0U;
    uint32_t fault_lr = frame_valid ? stack_frame[HF_IDX_LR] : 0U;

    retram_state_t local_state;
    retram_state_load(&local_state);
    local_state.last_event = BB_EVT_HARDFAULT;
    local_state.fault_pc = fault_pc;
    local_state.fault_lr = fault_lr;
    local_state.fault_cfsr = cfsr;
    retram_state_store(&local_state);
    cpu_dsb_isb();

    if ((s_bb_flash_degraded == 0U) && (s_bb_next_slot < (uint16_t)BLACKBOX_SLOTS)) {
        uint32_t slot_addr = HC32_FLASH_BLACKBOX_BASE +
                             ((uint32_t)s_bb_next_slot * (uint32_t)BLACKBOX_RECORD_BYTES);
        if (mmio_r32(slot_addr) == BB_ERASED_U32) {
            uint32_t fault_aux = mmio_r32(SCB_HFSR_ADDR);
            if ((cfsr & CFSR_BFAR_VALID) != 0UL) {
                fault_aux = mmio_r32(SCB_BFAR_ADDR);
            } else if ((cfsr & CFSR_MMAR_VALID) != 0UL) {
                fault_aux = mmio_r32(SCB_MMFAR_ADDR);
            }
            uint32_t words[WORDS_PER_BB_REC];
            words[0] = BB_MAGIC_TAG;
            words[1] = tmra1_now_us32();
            words[2] = (uint32_t)BB_EVT_HARDFAULT |
                       ((uint32_t)local_state.last_lens_id << SHIFT_16_BITS);
            words[3] = fault_pc;
            words[4] = fault_lr;
            words[BB_WIDX_CFSR] = cfsr;
            words[BB_WIDX_ZERO] = fault_aux;
            words[BB_WIDX_CRC] = crc32_ramfunc_safe(words, BB_HASH_WORDS);
            for (uint32_t widx = 0U; widx < (uint32_t)WORDS_PER_BB_REC; ++widx) {
                (void)efm_program_word_ramfunc(slot_addr + (widx * BYTES_IN_WORD), words[widx]);
            }
        }
    }

    mmio_w32(SCB_AIRCR_ADDR, AIRCR_SYSRESETREQ);
    for (;;) {
        cpu_nop();
    }
}

bool retram_warm_cache_is_valid(void) {
    return s_retram_warm_hit != 0U;
}

const retram_warm_hdr_t *retram_warm_cache_get(void) {
    static retram_warm_hdr_t s_warm_copy;
    retram_warm_hdr_load(&s_warm_copy);
    uint32_t exp_crc = crc32_ramfunc_safe((const uint32_t *)(const void *)&s_warm_copy, WARM_HASH_WORDS);
    if ((s_warm_copy.magic != RETRAM_WARM_MAGIC) || (s_warm_copy.hdr_crc32 != exp_crc)) {
        return NULL;
    }
    return &s_warm_copy;
}

void retram_warm_cache_update_bb_slot(uint16_t next_slot) {
    retram_warm_hdr_t local_hdr;
    retram_warm_hdr_load(&local_hdr);
    if (local_hdr.magic != RETRAM_WARM_MAGIC) {
        memset(&local_hdr, 0, sizeof(local_hdr));
        local_hdr.magic = RETRAM_WARM_MAGIC;
    }
    local_hdr.bb_next_slot = next_slot;
    retram_warm_hdr_store(&local_hdr);
}

/* NOLINTNEXTLINE(bugprone-easily-swappable-parameters) */
void retram_warm_cache_update_journal(uint32_t active_sector, uint16_t next_slot, uint64_t generation,
                                      uint16_t e_lens_id, int32_t e_inf, int32_t e_near,
                                      uint8_t linear_mf_code) {
    retram_warm_hdr_t local_hdr;
    retram_warm_hdr_load(&local_hdr);
    if (local_hdr.magic != RETRAM_WARM_MAGIC) {
        memset(&local_hdr, 0, sizeof(local_hdr));
        local_hdr.magic = RETRAM_WARM_MAGIC;
    }
    local_hdr.jnl_active_sector = active_sector;
    local_hdr.jnl_next_slot = next_slot;
    local_hdr.jnl_generation = generation;
    local_hdr.latched_e_inf = e_inf;
    local_hdr.latched_e_near = e_near;
    local_hdr.linear_mf_code = linear_mf_code;
    if ((e_lens_id != 0U) && (local_hdr.cached_e_lens_id != e_lens_id)) {
        local_hdr.cached_e_lens_id = e_lens_id;
        local_hdr.ds13_len = 0U;
        local_hdr.ds13_valid = 0U;
        local_hdr.ds13_crc32 = 0U;
    }
    retram_warm_hdr_store(&local_hdr);
}

void retram_warm_cache_store_ds13(uint16_t e_lens_id, const uint8_t *buf, uint16_t len) {
    if ((e_lens_id == 0U) || (buf == NULL) || (len == 0U) || (len > (uint16_t)RETRAM_DS13_CACHE_CAP)) {
        return;
    }
    uint32_t word_cnt = ((uint32_t)len + ALIGN32_ROUND_ADD) / BYTES_IN_WORD;
    uint32_t crc_state = CRC32_INIT_U32;
    uint32_t chunk_w[DS13_CHUNK_WORDS];
    for (uint32_t base_w = 0U; base_w < word_cnt; base_w += (uint32_t)DS13_CHUNK_WORDS) {
        uint32_t rem_w = word_cnt - base_w;
        uint32_t cur_w = (rem_w > (uint32_t)DS13_CHUNK_WORDS) ? (uint32_t)DS13_CHUNK_WORDS : rem_w;
        uint32_t byte_off = base_w * BYTES_IN_WORD;
        uint32_t rem_bytes = ((uint32_t)len > byte_off) ? ((uint32_t)len - byte_off) : 0UL;
        uint32_t copy_bytes = cur_w * BYTES_IN_WORD;
        if (copy_bytes > rem_bytes) {
            copy_bytes = rem_bytes;
        }
        memset(chunk_w, 0, (size_t)cur_w * (size_t)BYTES_IN_WORD);
        memcpy(chunk_w, &buf[byte_off], copy_bytes);
        crc_state = crc32_update_words(crc_state, chunk_w, cur_w);
    }
    uint32_t new_ds13_crc = ~crc_state;

    retram_warm_hdr_t local_hdr;
    retram_warm_hdr_load(&local_hdr);
    if (local_hdr.magic != RETRAM_WARM_MAGIC) {
        memset(&local_hdr, 0, sizeof(local_hdr));
        local_hdr.magic = RETRAM_WARM_MAGIC;
    }
    if ((local_hdr.ds13_valid != 0U) && (local_hdr.cached_e_lens_id == e_lens_id) &&
        (local_hdr.ds13_len == len) && (local_hdr.ds13_crc32 == new_ds13_crc)) {
        return;
    }

    for (uint32_t base_w = 0U; base_w < word_cnt; base_w += (uint32_t)DS13_CHUNK_WORDS) {
        uint32_t rem_w = word_cnt - base_w;
        uint32_t cur_w = (rem_w > (uint32_t)DS13_CHUNK_WORDS) ? (uint32_t)DS13_CHUNK_WORDS : rem_w;
        uint32_t byte_off = base_w * BYTES_IN_WORD;
        uint32_t rem_bytes = ((uint32_t)len > byte_off) ? ((uint32_t)len - byte_off) : 0UL;
        uint32_t copy_bytes = cur_w * BYTES_IN_WORD;
        if (copy_bytes > rem_bytes) {
            copy_bytes = rem_bytes;
        }
        memset(chunk_w, 0, (size_t)cur_w * (size_t)BYTES_IN_WORD);
        memcpy(chunk_w, &buf[byte_off], copy_bytes);
        retram_write_words((volatile uint32_t *)(void *)&s_retram_warm.ds13_words[base_w], chunk_w, cur_w);
    }

    local_hdr.cached_e_lens_id = e_lens_id;
    local_hdr.ds13_len = len;
    local_hdr.ds13_valid = 1U;
    local_hdr.ds13_crc32 = new_ds13_crc;
    retram_warm_hdr_store(&local_hdr);
}

bool retram_warm_cache_load_ds13(uint16_t e_lens_id, uint8_t *out_buf, uint16_t *out_len) {
    if ((e_lens_id == 0U) || (out_buf == NULL) || (out_len == NULL)) {
        return false;
    }
    const retram_warm_hdr_t *hdr = retram_warm_cache_get();
    if ((hdr == NULL) || (hdr->ds13_valid == 0U) ||
        (hdr->cached_e_lens_id != e_lens_id) ||
        (hdr->ds13_len == 0U) || (hdr->ds13_len > (uint16_t)RETRAM_DS13_CACHE_CAP)) {
        return false;
    }
    uint32_t word_cnt = ((uint32_t)hdr->ds13_len + ALIGN32_ROUND_ADD) / BYTES_IN_WORD;
    uint32_t crc_state = CRC32_INIT_U32;
    uint32_t chunk_w[DS13_CHUNK_WORDS];
    for (uint32_t base_w = 0U; base_w < word_cnt; base_w += (uint32_t)DS13_CHUNK_WORDS) {
        uint32_t rem_w = word_cnt - base_w;
        uint32_t cur_w = (rem_w > (uint32_t)DS13_CHUNK_WORDS) ? (uint32_t)DS13_CHUNK_WORDS : rem_w;
        retram_read_words(chunk_w,
                          (const volatile uint32_t *)(const volatile void *)&s_retram_warm.ds13_words[base_w],
                          cur_w);
        crc_state = crc32_update_words(crc_state, chunk_w, cur_w);
    }
    if ((~crc_state) != hdr->ds13_crc32) {
        return false;
    }
    for (uint32_t base_w = 0U; base_w < word_cnt; base_w += (uint32_t)DS13_CHUNK_WORDS) {
        uint32_t rem_w = word_cnt - base_w;
        uint32_t cur_w = (rem_w > (uint32_t)DS13_CHUNK_WORDS) ? (uint32_t)DS13_CHUNK_WORDS : rem_w;
        retram_read_words(chunk_w,
                          (const volatile uint32_t *)(const volatile void *)&s_retram_warm.ds13_words[base_w],
                          cur_w);
        uint32_t byte_off = base_w * BYTES_IN_WORD;
        uint32_t rem_bytes = ((uint32_t)hdr->ds13_len > byte_off) ? ((uint32_t)hdr->ds13_len - byte_off) : 0UL;
        uint32_t copy_bytes = cur_w * BYTES_IN_WORD;
        if (copy_bytes > rem_bytes) {
            copy_bytes = rem_bytes;
        }
        memcpy(&out_buf[byte_off], chunk_w, copy_bytes);
    }
    *out_len = hdr->ds13_len;
    return true;
}
