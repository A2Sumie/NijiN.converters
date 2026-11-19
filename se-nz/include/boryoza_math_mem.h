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

#endif /* BORYOZA_MATH_MEM_H */
