#ifndef BORYOZA_MATH_MEM_H
#define BORYOZA_MATH_MEM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(__STDC_HOSTED__) && (__STDC_HOSTED__ == 1)
#include <math.h>
#include <string.h>

static inline bool boryoza_isfinite_f32(float val) {
    return isfinite(val) != 0;
}

static inline float boryoza_log2f(float val) {
    return log2f(val);
}

static inline float boryoza_exp2f(float val) {
    return exp2f(val);
}

static inline float boryoza_sqrtf(float val) {
    return sqrtf(val);
}

#else
/* Freestanding embedded C17 declarations (implemented in 00_hal / 02_core) */
void *memcpy(void *dst, const void *src, size_t len);
void *memset(void *dst, int byte_val, size_t len);
size_t strlen(const char *str);

bool boryoza_isfinite_f32(float val);
float boryoza_log2f(float val);
float boryoza_exp2f(float val);
float boryoza_sqrtf(float val);
#endif

/* NOLINTNEXTLINE(bugprone-easily-swappable-parameters) */
static inline uint32_t boryoza_ticks64_div_u16_lo32(uint64_t ticks, uint32_t div) {
    static const uint32_t LIMB_SHIFT_32 = 32U;
    static const uint32_t LIMB_SHIFT_16 = 16U;
    static const uint32_t LIMB_LO16_MSK = 0xFFFFUL;
    if (div == 0U) {
        return 0U;
    }
    uint32_t hi32 = (uint32_t)(ticks >> LIMB_SHIFT_32);
    uint32_t lo32 = (uint32_t)ticks;
    uint32_t rem_hi = hi32 % div;
    uint32_t mid = (rem_hi << LIMB_SHIFT_16) | (lo32 >> LIMB_SHIFT_16);
    uint32_t quo_hi16 = mid / div;
    uint32_t rem_mid = mid % div;
    uint32_t quo_lo16 = ((rem_mid << LIMB_SHIFT_16) | (lo32 & LIMB_LO16_MSK)) / div;
    return (quo_hi16 << LIMB_SHIFT_16) | quo_lo16;
}

#endif /* BORYOZA_MATH_MEM_H */
