#include "../../include/hc32f460_regs.h"
#include "../../include/boryoza_math_mem.h"

enum {
    WORDS_PER_JOURNAL_REC = JOURNAL_RECORD_BYTES / 4,
    PAYLOAD_WORDS_120B    = 30,
    PREFIX_WORDS_124B     = 31,
};

static const uint32_t JOURNAL_VERSION_V2  = 0x00020000UL;
static const uint32_t ERASED_WORD_VAL     = 0xFFFFFFFFUL;
static const uint32_t WORD_STRIDE_BYTES   = 4UL;
static const uint32_t COMMIT_WORD_OFFSET  = 124UL;
static const int      JNL_ERR_HOTLINE_ON  = -1;
static const int      JNL_ERR_SESSION_ACT = -2;
static const int      JNL_ERR_READBACK    = -5;
static const int      JNL_ERR_DEGRADED    = -6;

static uint32_t s_active_sector_base = HC32_FLASH_JOURNAL_A_BASE;
static uint16_t s_next_slot_idx      = 0U;
static uint64_t s_cur_generation     = 0ULL;
static uint8_t  s_jnl_flash_degraded = 0U;

static bool read_valid_slot(uint32_t sector_base, uint32_t slot_idx, journal_record_t *out_rec) {
    uint32_t addr = sector_base + (slot_idx * (uint32_t)JOURNAL_RECORD_BYTES);
    uint32_t raw_words[WORDS_PER_JOURNAL_REC];
    for (uint32_t widx = 0U; widx < (uint32_t)WORDS_PER_JOURNAL_REC; ++widx) {
        raw_words[widx] = mmio_r32(addr + (widx * WORD_STRIDE_BYTES));
    }
    memcpy(out_rec, raw_words, sizeof(*out_rec));
    if ((out_rec->magic_tag != JOURNAL_MAGIC_TAG) ||
        (out_rec->commit_mark != JOURNAL_COMMIT_MARK)) {
        return false;
    }
    uint32_t calc_crc = boryoza_crc32_ieee(raw_words, (size_t)PAYLOAD_WORDS_120B * 4U);
    return calc_crc == out_rec->payload_crc32;
}

static bool journal_slot_is_erased(uint32_t sector_base, uint32_t slot_idx) {
    uint32_t addr = sector_base + (slot_idx * (uint32_t)JOURNAL_RECORD_BYTES);
    for (uint32_t widx = 0U; widx < (uint32_t)WORDS_PER_JOURNAL_REC; ++widx) {
        if (mmio_r32(addr + (widx * WORD_STRIDE_BYTES)) != ERASED_WORD_VAL) {
            return false;
        }
    }
    return true;
}

static uint16_t find_first_erased_slot(uint32_t sector_base) {
    uint16_t suffix_start = (uint16_t)JOURNAL_SLOTS_PER_SECTOR;
    for (int32_t slot = (int32_t)JOURNAL_SLOTS_PER_SECTOR - 1; slot >= 0; --slot) {
        if (!journal_slot_is_erased(sector_base, (uint32_t)slot)) {
            break;
        }
        suffix_start = (uint16_t)slot;
    }
    return suffix_start;
}

void journal_init_and_load(adapter_ctx_t *ctx) {
    s_jnl_flash_degraded = 0U;
    const retram_warm_hdr_t *warm = retram_warm_cache_is_valid() ? retram_warm_cache_get() : NULL;
    if ((warm != NULL) &&
        ((warm->jnl_active_sector == HC32_FLASH_JOURNAL_A_BASE) ||
         (warm->jnl_active_sector == HC32_FLASH_JOURNAL_B_BASE)) &&
        (warm->jnl_next_slot <= (uint16_t)JOURNAL_SLOTS_PER_SECTOR)) {
        bool warm_ok = false;
        journal_record_t verify_rec;
        bool slot_boundary = (bool)((warm->jnl_next_slot == (uint16_t)JOURNAL_SLOTS_PER_SECTOR) ||
                                    journal_slot_is_erased(warm->jnl_active_sector,
                                                           (uint32_t)warm->jnl_next_slot));
        bool empty_warm = (bool)((warm->jnl_next_slot == 0U) && (warm->jnl_generation == 0ULL) &&
                                 journal_slot_is_erased(HC32_FLASH_JOURNAL_A_BASE, 0U) &&
                                 journal_slot_is_erased(HC32_FLASH_JOURNAL_B_BASE, 0U));
        bool valid_warm = (bool)((warm->jnl_next_slot > 0U) &&
                                 read_valid_slot(warm->jnl_active_sector,
                                                 (uint32_t)warm->jnl_next_slot - 1U, &verify_rec) &&
                                 (verify_rec.generation == warm->jnl_generation));
        if (slot_boundary && (empty_warm || valid_warm)) {
            warm_ok = true;
        }
        if (warm_ok) {
            s_active_sector_base = warm->jnl_active_sector;
            s_cur_generation = warm->jnl_generation;
            s_next_slot_idx = warm->jnl_next_slot;
            ctx->af.linear_mf_code = warm->linear_mf_code;
            return;
        }
    }

    journal_record_t best_rec;
    memset(&best_rec, 0, sizeof(best_rec));
    bool found_any = false;
    uint32_t best_sector = HC32_FLASH_JOURNAL_A_BASE;

    const uint32_t sectors[2] = {HC32_FLASH_JOURNAL_A_BASE, HC32_FLASH_JOURNAL_B_BASE};
    for (uint32_t sidx = 0U; sidx < 2U; ++sidx) {
        for (uint32_t slot = 0U; slot < (uint32_t)JOURNAL_SLOTS_PER_SECTOR; ++slot) {
            journal_record_t candidate;
            if (read_valid_slot(sectors[sidx], slot, &candidate) &&
                ((!found_any) || (candidate.generation > best_rec.generation))) {
                best_rec = candidate;
                best_sector = sectors[sidx];
                found_any = true;
            }
        }
    }

    if (found_any) {
        s_active_sector_base = best_sector;
        s_cur_generation = best_rec.generation;
        s_next_slot_idx = find_first_erased_slot(best_sector);
        ctx->af.linear_mf_code = best_rec.linear_mf_code;
        retram_warm_cache_update_journal(s_active_sector_base, s_next_slot_idx, s_cur_generation,
                                         best_rec.last_e_lens_id, best_rec.latched_e_inf,
                                         best_rec.latched_e_near, best_rec.linear_mf_code);
    } else {
        s_active_sector_base = HC32_FLASH_JOURNAL_A_BASE;
        s_cur_generation = 0ULL;
        s_next_slot_idx = find_first_erased_slot(HC32_FLASH_JOURNAL_A_BASE);
        retram_warm_cache_update_journal(s_active_sector_base, s_next_slot_idx, 0ULL,
                                         0U, 0, 0, 0U);
    }
}

int journal_commit_idle(const adapter_ctx_t *ctx) {
    if (s_jnl_flash_degraded != 0U) {
        return JNL_ERR_DEGRADED;
    }
    if (ctx->vr.hotline_enabled != 0U) {
        return JNL_ERR_HOTLINE_ON;
    }
    if ((ctx->session_state == E_SESSION_ACTIVE) ||
        (ctx->session_state == E_SESSION_HANDSHAKE) ||
        (ctx->session_state == E_SESSION_POWERUP_WAIT)) {
        return JNL_ERR_SESSION_ACT;
    }

    if (s_next_slot_idx >= (uint16_t)JOURNAL_SLOTS_PER_SECTOR) {
        uint32_t next_sector = (s_active_sector_base == HC32_FLASH_JOURNAL_A_BASE)
                                   ? HC32_FLASH_JOURNAL_B_BASE
                                   : HC32_FLASH_JOURNAL_A_BASE;
        int erase_status = efm_erase_sector_ramfunc(next_sector);
        if (erase_status != 0) {
            s_jnl_flash_degraded = 1U;
            return erase_status;
        }
        s_active_sector_base = next_sector;
        s_next_slot_idx = 0U;
    }

    journal_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic_tag = JOURNAL_MAGIC_TAG;
    rec.version = JOURNAL_VERSION_V2;
    rec.generation = s_cur_generation + 1ULL;
    rec.last_e_lens_id = ctx->e_lens_id;
    rec.last_z_lens_id = ctx->z_primary_lens_id;
    rec.latched_e_inf = ctx->af.e_inf_true;
    rec.latched_e_near = ctx->af.e_near_true;
    rec.last_focal_mm = ctx->f_cur_mm;
    rec.last_av_c = ctx->ap.c_cur;
    rec.linear_mf_code = ctx->af.linear_mf_code;
    rec.vr_last_mode = (uint8_t)ctx->vr.body_mode;
    rec.payload_crc32 = boryoza_crc32_ieee(&rec, (size_t)PAYLOAD_WORDS_120B * 4U);
    rec.commit_mark = JOURNAL_COMMIT_MARK;

    uint32_t words[WORDS_PER_JOURNAL_REC];
    memcpy(words, &rec, sizeof(words));

    uint32_t slot_addr = s_active_sector_base +
                         ((uint32_t)s_next_slot_idx * (uint32_t)JOURNAL_RECORD_BYTES);
    s_next_slot_idx = (uint16_t)(s_next_slot_idx + 1U);

    for (uint32_t widx = 0U; widx < (uint32_t)PREFIX_WORDS_124B; ++widx) {
        int prog_status = efm_program_word_ramfunc(slot_addr + (widx * WORD_STRIDE_BYTES), words[widx]);
        if (prog_status != 0) {
            s_jnl_flash_degraded = 1U;
            return prog_status;
        }
    }

    for (uint32_t widx = 0U; widx < (uint32_t)PREFIX_WORDS_124B; ++widx) {
        if (mmio_r32(slot_addr + (widx * WORD_STRIDE_BYTES)) != words[widx]) {
            s_jnl_flash_degraded = 1U;
            return JNL_ERR_READBACK;
        }
    }

    int commit_status = efm_program_word_ramfunc(slot_addr + COMMIT_WORD_OFFSET, words[PREFIX_WORDS_124B]);
    if (commit_status == 0) {
        s_cur_generation = rec.generation;
        retram_warm_cache_update_journal(s_active_sector_base, s_next_slot_idx, s_cur_generation,
                                         ctx->e_lens_id, ctx->af.e_inf_true, ctx->af.e_near_true,
                                         ctx->af.linear_mf_code);
    } else {
        s_jnl_flash_degraded = 1U;
    }
    return commit_status;
}
