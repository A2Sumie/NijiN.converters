#include "../../include/hc32f460_regs.h"
#include "../../include/boryoza_math_mem.h"

enum {
    WORDS_PER_JOURNAL_REC = JOURNAL_RECORD_BYTES / 4,
    PAYLOAD_WORDS_120B    = 30,
    PREFIX_WORDS_124B     = 31,
    FX_LRU_WORDS          = sizeof(v224_fx_lru_table_t) / 4,
    FX_LRU_HASH_WORDS     = FX_LRU_WORDS - 1,
};

_Static_assert((sizeof(v224_fx_lru_table_t) % 4U) == 0U, "v224_fx_lru_table_t must be word aligned");

static const uint32_t JOURNAL_VERSION_V2  = 0x00030000UL;
static const uint32_t ERASED_WORD_VAL     = 0xFFFFFFFFUL;
static const uint32_t WORD_STRIDE_BYTES   = 4UL;
static const uint32_t COMMIT_WORD_OFFSET  = 124UL;
static const int      JNL_ERR_HOTLINE_ON  = -1;
static const int      JNL_ERR_SESSION_ACT = -2;
static const int      JNL_ERR_READBACK    = -5;
static const int      JNL_ERR_DEGRADED    = -6;

BORYOZA_RETRAM_BUF static v224_fx_lru_table_t s_retram_fx_lru;

static uint32_t s_active_sector_base = HC32_FLASH_JOURNAL_A_BASE;
static uint16_t s_next_slot_idx      = 0U;
static uint64_t s_cur_generation     = 0ULL;
static uint8_t  s_jnl_flash_degraded = 0U;

uint16_t journal_sector_slot_cap(uint32_t sector_base) {
    if (sector_base == HC32_FLASH_JOURNAL_B_BASE) {
        return (uint16_t)JOURNAL_B_SLOTS_USABLE;
    }
    return (uint16_t)JOURNAL_SLOTS_PER_SECTOR;
}

static bool fx_lru_crc_valid(const v224_fx_lru_table_t *tbl) {
    if ((tbl->magic != FX_LRU_MAGIC_TAG) || (tbl->count > (uint8_t)V224_FX_LRU_SLOTS)) {
        return false;
    }
    uint32_t crc = boryoza_crc32_ieee(tbl, (size_t)FX_LRU_HASH_WORDS * 4U);
    return crc == tbl->crc32;
}

static void fx_lru_load(v224_fx_lru_table_t *dst) {
    uint32_t words[FX_LRU_WORDS];
    const volatile uint32_t *src =
        (const volatile uint32_t *)(const volatile void *)&s_retram_fx_lru;
    for (uint32_t idx = 0U; idx < (uint32_t)FX_LRU_WORDS; ++idx) {
        words[idx] = src[idx];
    }
    memcpy(dst, words, sizeof(*dst));
}

static void fx_lru_store(v224_fx_lru_table_t *src_tbl) {
    src_tbl->magic = FX_LRU_MAGIC_TAG;
    src_tbl->crc32 = boryoza_crc32_ieee(src_tbl, (size_t)FX_LRU_HASH_WORDS * 4U);
    uint32_t words[FX_LRU_WORDS];
    memcpy(words, src_tbl, sizeof(words));
    volatile uint32_t *dst = (volatile uint32_t *)(void *)&s_retram_fx_lru;
    for (uint32_t idx = 0U; idx < (uint32_t)FX_LRU_WORDS; ++idx) {
        dst[idx] = words[idx];
    }
}

void journal_fx_lru_init(void) {
    v224_fx_lru_table_t tbl;
    fx_lru_load(&tbl);
    if (retram_warm_cache_is_valid() && fx_lru_crc_valid(&tbl)) {
        return;
    }
    memset(&tbl, 0, sizeof(tbl));
    fx_lru_store(&tbl);
}

enum {
    FX_LRU_AGE_MAX        = 254U,
    JNL_SCAN_SECTOR_COUNT = 3U,
};

uint8_t journal_fx_lru_lookup(const v224_fx_lru_key_t *key) {
    if (key == NULL) {
        return 0U;
    }
    v224_fx_lru_table_t tbl;
    fx_lru_load(&tbl);
    if (!fx_lru_crc_valid(&tbl)) {
        return 0U;
    }
    for (uint8_t idx = 0U; (idx < tbl.count) && (idx < (uint8_t)V224_FX_LRU_SLOTS); ++idx) {
        const v224_fx_lru_entry_t *ent = &tbl.slots[idx];
        if ((ent->e_lid == key->e_lid) && (ent->f_min_mm == key->f_min_mm) &&
            (ent->av_open_x100 == key->av_open_x100)) {
            return ent->fx_enable;
        }
    }
    return 0U;
}

void journal_fx_lru_set(const v224_fx_lru_key_t *key, uint8_t fx_enable) {
    if (key == NULL) {
        return;
    }
    v224_fx_lru_table_t tbl;
    fx_lru_load(&tbl);
    if (!fx_lru_crc_valid(&tbl)) {
        memset(&tbl, 0, sizeof(tbl));
    }
    uint8_t target_idx = tbl.count;
    for (uint8_t idx = 0U; (idx < tbl.count) && (idx < (uint8_t)V224_FX_LRU_SLOTS); ++idx) {
        if ((tbl.slots[idx].e_lid == key->e_lid) &&
            (tbl.slots[idx].f_min_mm == key->f_min_mm) &&
            (tbl.slots[idx].av_open_x100 == key->av_open_x100)) {
            target_idx = idx;
            break;
        }
    }
    if (target_idx >= (uint8_t)V224_FX_LRU_SLOTS) {
        uint8_t oldest_idx = 0U;
        for (uint8_t idx = 1U; idx < (uint8_t)V224_FX_LRU_SLOTS; ++idx) {
            if (tbl.slots[idx].age_rank > tbl.slots[oldest_idx].age_rank) {
                oldest_idx = idx;
            }
        }
        target_idx = oldest_idx;
    } else if (target_idx >= tbl.count) {
        tbl.count = (uint8_t)(target_idx + 1U);
    }
    for (uint8_t idx = 0U; (idx < tbl.count) && (idx < (uint8_t)V224_FX_LRU_SLOTS); ++idx) {
        if ((idx != target_idx) && (tbl.slots[idx].age_rank < (uint8_t)FX_LRU_AGE_MAX)) {
            tbl.slots[idx].age_rank = (uint8_t)(tbl.slots[idx].age_rank + 1U);
        }
    }
    tbl.slots[target_idx].e_lid = key->e_lid;
    tbl.slots[target_idx].f_min_mm = key->f_min_mm;
    tbl.slots[target_idx].av_open_x100 = key->av_open_x100;
    tbl.slots[target_idx].fx_enable = (fx_enable != 0U) ? 1U : 0U;
    tbl.slots[target_idx].age_rank = 0U;
    tbl.seq = (uint16_t)(tbl.seq + 1U);
    fx_lru_store(&tbl);
}

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
    uint16_t cap = journal_sector_slot_cap(sector_base);
    uint16_t suffix_start = cap;
    for (int32_t slot = (int32_t)cap - 1; slot >= 0; --slot) {
        if (!journal_slot_is_erased(sector_base, (uint32_t)slot)) {
            break;
        }
        suffix_start = (uint16_t)slot;
    }
    return suffix_start;
}

static void load_fx_lru_from_record_or_clear(const journal_record_t *rec) {
    v224_fx_lru_table_t tbl;
    memcpy(&tbl, rec->reserved_cfg, sizeof(tbl));
    if (fx_lru_crc_valid(&tbl)) {
        fx_lru_store(&tbl);
    }
}

static bool try_load_warm_journal(adapter_ctx_t *ctx) {
    const retram_warm_hdr_t *warm = retram_warm_cache_is_valid() ? retram_warm_cache_get() : NULL;
    if ((warm == NULL) ||
        ((warm->jnl_active_sector != HC32_FLASH_JOURNAL_A_BASE) &&
         (warm->jnl_active_sector != HC32_FLASH_JOURNAL_PONG_BASE))) {
        return false;
    }
    uint16_t warm_cap = journal_sector_slot_cap(warm->jnl_active_sector);
    if (warm->jnl_next_slot > warm_cap) {
        return false;
    }
    journal_record_t verify_rec;
    memset(&verify_rec, 0, sizeof(verify_rec));
    bool slot_boundary = (bool)((warm->jnl_next_slot == warm_cap) ||
                                journal_slot_is_erased(warm->jnl_active_sector,
                                                       (uint32_t)warm->jnl_next_slot));
    bool empty_warm = (bool)((warm->jnl_next_slot == 0U) && (warm->jnl_generation == 0ULL) &&
                             journal_slot_is_erased(HC32_FLASH_JOURNAL_PONG_BASE, 0U) &&
                             journal_slot_is_erased(HC32_FLASH_JOURNAL_A_BASE, 0U) &&
                             journal_slot_is_erased(HC32_FLASH_JOURNAL_B_BASE, 0U));
    bool valid_warm = (bool)((warm->jnl_next_slot > 0U) &&
                             read_valid_slot(warm->jnl_active_sector,
                                             (uint32_t)warm->jnl_next_slot - 1U, &verify_rec) &&
                             (verify_rec.generation == warm->jnl_generation));
    if ((!slot_boundary) || ((!empty_warm) && (!valid_warm))) {
        return false;
    }
    s_active_sector_base = warm->jnl_active_sector;
    s_cur_generation = warm->jnl_generation;
    s_next_slot_idx = warm->jnl_next_slot;
    ctx->af.linear_mf_code = warm->linear_mf_code;
    v224_fx_lru_table_t rt_lru;
    fx_lru_load(&rt_lru);
    if ((!fx_lru_crc_valid(&rt_lru)) && valid_warm) {
        load_fx_lru_from_record_or_clear(&verify_rec);
    }
    return true;
}

static bool scan_best_flash_journal(journal_record_t *out_rec, uint32_t *out_sector) {
    const uint32_t sectors[JNL_SCAN_SECTOR_COUNT] = {
        HC32_FLASH_JOURNAL_A_BASE,
        HC32_FLASH_JOURNAL_PONG_BASE,
        HC32_FLASH_JOURNAL_B_BASE,
    };
    bool found_any = false;
    for (uint32_t sidx = 0U; sidx < (uint32_t)JNL_SCAN_SECTOR_COUNT; ++sidx) {
        uint16_t cap = journal_sector_slot_cap(sectors[sidx]);
        for (uint32_t slot = 0U; slot < (uint32_t)cap; ++slot) {
            journal_record_t candidate;
            if (read_valid_slot(sectors[sidx], slot, &candidate) &&
                ((!found_any) || (candidate.generation > out_rec->generation))) {
                *out_rec = candidate;
                *out_sector = sectors[sidx];
                found_any = true;
            }
        }
    }
    return found_any;
}

void journal_init_and_load(adapter_ctx_t *ctx) {
    s_jnl_flash_degraded = 0U;
    if (try_load_warm_journal(ctx)) {
        return;
    }

    journal_record_t best_rec;
    memset(&best_rec, 0, sizeof(best_rec));
    uint32_t best_sector = HC32_FLASH_JOURNAL_A_BASE;
    bool found_any = scan_best_flash_journal(&best_rec, &best_sector);

    if (found_any) {
        load_fx_lru_from_record_or_clear(&best_rec);
        if (best_sector == HC32_FLASH_JOURNAL_B_BASE) {
            /* Legacy secondary sector is read-only; continue active commits on Pong/A */
            s_active_sector_base = HC32_FLASH_JOURNAL_PONG_BASE;
            s_next_slot_idx = find_first_erased_slot(HC32_FLASH_JOURNAL_PONG_BASE);
        } else {
            s_active_sector_base = best_sector;
            s_next_slot_idx = find_first_erased_slot(best_sector);
        }
        s_cur_generation = best_rec.generation;
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

    if (s_next_slot_idx >= journal_sector_slot_cap(s_active_sector_base)) {
        uint32_t next_sector = (s_active_sector_base == HC32_FLASH_JOURNAL_A_BASE)
                                   ? HC32_FLASH_JOURNAL_PONG_BASE
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
    v224_fx_lru_table_t lru_copy;
    fx_lru_load(&lru_copy);
    if (!fx_lru_crc_valid(&lru_copy)) {
        memset(&lru_copy, 0, sizeof(lru_copy));
        lru_copy.magic = FX_LRU_MAGIC_TAG;
        lru_copy.crc32 = boryoza_crc32_ieee(&lru_copy, (size_t)FX_LRU_HASH_WORDS * 4U);
    }
    memcpy(rec.reserved_cfg, &lru_copy, sizeof(lru_copy));
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
