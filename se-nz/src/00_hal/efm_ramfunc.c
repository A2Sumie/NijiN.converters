#include "../../include/hc32f460_regs.h"

static const uint32_t EFM_WAIT_LIMIT    = 2000000UL;
static const uint32_t EFM_HOLD_MAX_LOOPS = 4000000UL;
static const uint32_t FLASH_DIAG_MIN    = 0x00078000UL;
static const uint32_t FLASH_DIAG_MAX    = 0x0007FFE0UL; /* Exclude top 32-byte reserved trim tail */
static const uint32_t WORD_ALIGN_MASK   = 0x00000003UL;
static const uint32_t ERASE_DUMMY_VALUE = 0x12345678UL; /* Sector-erase trigger word */
static const uint32_t EFM_DONE_MASK     = (0x00000100UL | 0x00000010UL); /* RDY | OPTEND */
static const int      EFM_ERR_PARAM     = -1;
static const int      EFM_ERR_UNLOCK    = -2;
static const int      EFM_ERR_STATUS    = -3;
static const int      EFM_ERR_VERIFY    = -4;
static const int      EFM_ERR_TIMEOUT   = -5;

volatile uint32_t g_efm_failure_stage;
volatile uint32_t g_efm_failure_fsr;

BORYOZA_RAMFUNC static void efm_hold_until_readable(uint32_t stage) {
    g_efm_failure_stage = stage;
    g_efm_failure_fsr = mmio_r32(EFM_FSR_ADDR);
    for (uint32_t idx = 0U; idx < EFM_HOLD_MAX_LOOPS; ++idx) {
        if ((mmio_r32(EFM_FSR_ADDR) & EFM_FSR_RDY_BIT) != 0UL) {
            return;
        }
        cpu_nop();
    }
}

BORYOZA_RAMFUNC static int efm_wait_ready_ramfunc(void) {
    for (uint32_t idx = 0U; idx < EFM_WAIT_LIMIT; ++idx) {
        if ((mmio_r32(EFM_FSR_ADDR) & EFM_FSR_RDY_BIT) != 0UL) {
            return 0;
        }
    }
    return EFM_ERR_TIMEOUT;
}

BORYOZA_RAMFUNC static int efm_wait_optend_and_ready_ramfunc(void) {
    for (uint32_t idx = 0U; idx < EFM_WAIT_LIMIT; ++idx) {
        if ((mmio_r32(EFM_FSR_ADDR) & EFM_DONE_MASK) == EFM_DONE_MASK) {
            return 0;
        }
    }
    return EFM_ERR_TIMEOUT;
}

BORYOZA_RAMFUNC int efm_program_word_ramfunc(uint32_t flash_addr, uint32_t word_val) {
    if ((flash_addr < FLASH_DIAG_MIN) || (flash_addr >= FLASH_DIAG_MAX) ||
        ((flash_addr & WORD_ALIGN_MASK) != 0UL)) {
        return EFM_ERR_PARAM;
    }

    uint32_t primask = cpu_irq_save();
    if ((mmio_r32(EFM_FAPRT_ADDR) & 1UL) == 0UL) {
        mmio_w32(EFM_FAPRT_ADDR, EFM_KEY1);
        mmio_w32(EFM_FAPRT_ADDR, EFM_KEY2);
    }
    if ((mmio_r32(EFM_FAPRT_ADDR) & 1UL) == 0UL) {
        cpu_irq_restore(primask);
        return EFM_ERR_UNLOCK;
    }

    mmio_w32(EFM_FSTP_ADDR, mmio_r32(EFM_FSTP_ADDR) & ~1UL);
    if (efm_wait_ready_ramfunc() != 0) {
        efm_hold_until_readable(1U);
        mmio_w32(EFM_FAPRT_ADDR, EFM_LOCK_VAL);
        mmio_w32(EFM_FAPRT_ADDR, EFM_LOCK_VAL);
        cpu_irq_restore(primask);
        return EFM_ERR_TIMEOUT;
    }
    uint32_t orig_frmc = mmio_r32(EFM_FRMC_ADDR);
    mmio_w32(EFM_FRMC_ADDR, orig_frmc & ~(EFM_FRMC_CACHE_BIT | EFM_FRMC_SLPMD_BIT));
    (void)mmio_r32(EFM_FRMC_ADDR);

    /* Two-step FWMC rule: PEMOD[6:4] requires PEMODE[0]=1 latched in a prior write */
    mmio_w32(EFM_FWMC_ADDR, EFM_FWMC_PEMODE_BIT);
    mmio_w32(EFM_FWMC_ADDR, EFM_FWMC_SINGLE_PROG);
    mmio_w32(EFM_FSCLR_ADDR, EFM_FSCLR_ALL);

    mmio_w32(flash_addr, word_val);
    int status = efm_wait_optend_and_ready_ramfunc();
    if (status != 0) {
        efm_hold_until_readable(2U);
    }
    uint32_t fsr = mmio_r32(EFM_FSR_ADDR);
    if ((status == 0) && ((fsr & EFM_FSR_ERR_MASK) != 0UL)) {
        status = EFM_ERR_STATUS;
    }

    mmio_w32(EFM_FSCLR_ADDR, EFM_FSCLR_ALL);
    /* Two-step teardown: clear PEMOD to Read-Only while PEMODE=1, then clear PEMODE=0 */
    mmio_w32(EFM_FWMC_ADDR, EFM_FWMC_PEMODE_BIT);
    mmio_w32(EFM_FWMC_ADDR, 0x00000000UL);
    if ((orig_frmc & EFM_FRMC_CACHE_BIT) != 0UL) {
        mmio_w32(EFM_FRMC_ADDR, (orig_frmc & ~EFM_FRMC_CACHE_BIT) | EFM_FRMC_CRST_BIT);
        (void)mmio_r32(EFM_FRMC_ADDR);
    }
    mmio_w32(EFM_FRMC_ADDR, orig_frmc);
    (void)mmio_r32(EFM_FRMC_ADDR);
    mmio_w32(EFM_FAPRT_ADDR, EFM_LOCK_VAL);
    mmio_w32(EFM_FAPRT_ADDR, EFM_LOCK_VAL);
    cpu_irq_restore(primask);

    if ((status == 0) && (mmio_r32(flash_addr) != word_val)) {
        return EFM_ERR_VERIFY;
    }
    return status;
}

BORYOZA_RAMFUNC int efm_erase_sector_ramfunc(uint32_t sector_addr) {
    if ((sector_addr != HC32_FLASH_JOURNAL_PONG_BASE) &&
        (sector_addr != HC32_FLASH_BLACKBOX_BASE) &&
        (sector_addr != HC32_FLASH_JOURNAL_A_BASE)) {
        return EFM_ERR_PARAM;
    }

    uint32_t primask = cpu_irq_save();
    if ((mmio_r32(EFM_FAPRT_ADDR) & 1UL) == 0UL) {
        mmio_w32(EFM_FAPRT_ADDR, EFM_KEY1);
        mmio_w32(EFM_FAPRT_ADDR, EFM_KEY2);
    }
    if ((mmio_r32(EFM_FAPRT_ADDR) & 1UL) == 0UL) {
        cpu_irq_restore(primask);
        return EFM_ERR_UNLOCK;
    }

    mmio_w32(EFM_FSTP_ADDR, mmio_r32(EFM_FSTP_ADDR) & ~1UL);
    if (efm_wait_ready_ramfunc() != 0) {
        efm_hold_until_readable(1U);
        mmio_w32(EFM_FAPRT_ADDR, EFM_LOCK_VAL);
        mmio_w32(EFM_FAPRT_ADDR, EFM_LOCK_VAL);
        cpu_irq_restore(primask);
        return EFM_ERR_TIMEOUT;
    }
    uint32_t orig_frmc = mmio_r32(EFM_FRMC_ADDR);
    mmio_w32(EFM_FRMC_ADDR, orig_frmc & ~(EFM_FRMC_CACHE_BIT | EFM_FRMC_SLPMD_BIT));
    (void)mmio_r32(EFM_FRMC_ADDR);

    /* Two-step FWMC unlock: latch PEMODE=1 first, then set PEMOD to sector erase */
    mmio_w32(EFM_FWMC_ADDR, EFM_FWMC_PEMODE_BIT);
    mmio_w32(EFM_FWMC_ADDR, EFM_FWMC_SECTOR_ERASE);
    mmio_w32(EFM_FSCLR_ADDR, EFM_FSCLR_ALL);

    mmio_w32(sector_addr, ERASE_DUMMY_VALUE);
    int status = efm_wait_optend_and_ready_ramfunc();
    if (status != 0) {
        efm_hold_until_readable(2U);
    }
    uint32_t fsr = mmio_r32(EFM_FSR_ADDR);
    if ((status == 0) && ((fsr & EFM_FSR_ERR_MASK) != 0UL)) {
        status = EFM_ERR_STATUS;
    }

    mmio_w32(EFM_FSCLR_ADDR, EFM_FSCLR_ALL);
    mmio_w32(EFM_FWMC_ADDR, EFM_FWMC_PEMODE_BIT);
    mmio_w32(EFM_FWMC_ADDR, 0x00000000UL);
    if ((orig_frmc & EFM_FRMC_CACHE_BIT) != 0UL) {
        mmio_w32(EFM_FRMC_ADDR, (orig_frmc & ~EFM_FRMC_CACHE_BIT) | EFM_FRMC_CRST_BIT);
        (void)mmio_r32(EFM_FRMC_ADDR);
    }
    mmio_w32(EFM_FRMC_ADDR, orig_frmc);
    (void)mmio_r32(EFM_FRMC_ADDR);
    mmio_w32(EFM_FAPRT_ADDR, EFM_LOCK_VAL);
    mmio_w32(EFM_FAPRT_ADDR, EFM_LOCK_VAL);
    cpu_irq_restore(primask);
    return status;
}
