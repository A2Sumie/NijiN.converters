#include "../../include/boryoza_types.h"
#include "../../include/boryoza_math_mem.h"
#include "../../include/lens_db.h"

#if !defined(__STDC_HOSTED__) || (__STDC_HOSTED__ == 0)
/* NOLINTBEGIN(readability-magic-numbers) */
typedef union {
    float    f;
    uint32_t u;
} f32_u32_bits_t;

bool boryoza_isfinite_f32(float val) {
    f32_u32_bits_t cvt;
    cvt.f = val;
    return ((cvt.u >> 23U) & 0xFFUL) != 0xFFUL;
}

float boryoza_log2f(float val) {
    f32_u32_bits_t cvt;
    cvt.f = val;
    uint32_t bits = cvt.u;
    if ((val <= 0.0F) || (((bits >> 23U) & 0xFFUL) == 0xFFUL)) {
        return -126.0F;
    }
    int32_t exp_unbiased = (int32_t)((bits >> 23U) & 0xFFUL) - 127;
    f32_u32_bits_t mant_cvt;
    mant_cvt.u = (bits & 0x007FFFFFUL) | 0x3F800000UL;
    float mant = mant_cvt.f;
    if (mant > 1.41421356F) {
        mant *= 0.5F;
        exp_unbiased += 1;
    }
    float num = mant - 1.0F;
    float den = mant + 1.0F;
    float ratio = num / den;
    float rsq = ratio * ratio;
    float poly = 2.88539008F + (rsq * (0.96179669F + (rsq * (0.57707802F + (rsq * 0.41219859F)))));
    return (float)exp_unbiased + (ratio * poly);
}

float boryoza_exp2f(float val) {
    if (val <= -126.0F) {
        return 0.0F;
    }
    if (val >= 127.0F) {
        val = 127.0F;
    }
    int32_t whole = (val >= 0.0F) ? (int32_t)val : ((int32_t)val - ((val == (float)(int32_t)val) ? 0 : 1));
    float frac = val - (float)whole;
    float poly = 1.0F + (frac * (0.69314718F + (frac * (0.24022651F +
                 (frac * (0.05550411F + (frac * (0.00961813F + (frac * 0.00133336F)))))))));
    f32_u32_bits_t scale_cvt;
    scale_cvt.u = (uint32_t)(whole + 127) << 23U;
    return poly * scale_cvt.f;
}

float boryoza_sqrtf(float val) {
    if (val <= 0.0F) {
        return 0.0F;
    }
#if defined(__ARM_FP) && ((__ARM_FP & 0x4) != 0)
    float res = 0.0F;
    __asm__("vsqrt.f32 %0, %1" : "=t"(res) : "t"(val));
    return res;
#else
    f32_u32_bits_t guess;
    guess.f = val;
    guess.u = (guess.u >> 1U) + 0x1FC00000UL;
    float x = guess.f;
    for (uint32_t iter = 0U; iter < 4U; ++iter) {
        x = 0.5F * (x + (val / x));
    }
    return x;
#endif
}
/* NOLINTEND(readability-magic-numbers) */
#endif

static const uint8_t  Z_SYNC_55                 = 0x55U;
static const uint8_t  CMD24_A_FLAG_QUARTIC_3    = 0x03U;
static const size_t   CMD24_A_NODE_START        = 2U;
static const size_t   CMD24_A_COEFF_START       = 38U;
static const size_t   CMD24_A_FLAG3_FLOATS      = 60U;
static const size_t   CMD24_A_GRP_STRIDE        = 15U;
static const uint8_t  CMD24_DIST_FORMAT_3       = 0x03U;
static const uint8_t  CMD24_DIST_FORMAT_NONE    = 0x00U;

static const uint8_t  CMD24_B_FLAG_STD          = 0x01U;
static const uint32_t CMD24_B_NODE_SLOTS        = 8UL;
static const size_t   CMD24_B_COEFF_START       = 10U;
static const size_t   CMD24_B_FLOATS_PER_NODE   = 108U;
static const size_t   CMD24_B_BLUE_OFF          = 54U;
static const size_t   DIST_RADIAL_STRIDE        = 18U;

static const uint16_t SIGMA_150600_REV2         = 0xC56EU;
static const uint16_t SIGMA_150600_REV1         = 0xC55FU;
static const uint8_t  MAG_INVALID_CODE          = 0xFFU;
static const int32_t  A3_MAX_Q14                = 32768;
static const int32_t  A3_TIER3_CAP_Q14          = 4915;
static const uint16_t DS13_MIN_BYTES            = 30U;
static const uint16_t DS13_STD_GRID_BYTES       = 3870U;
static const uint32_t DS13_ROWS_ZOOM            = 129UL;
static const uint32_t DS13_COLS_FOCUS           = 15UL;
static const size_t   LF_PAGE_PAYLOAD_BYTES     = 488U;

static const uint32_t SIGN_12BIT_MASK           = 0x0800UL;
static const uint32_t MASK_12BIT_U32            = 0x0FFFUL;
static const int32_t  SIGN_12BIT_SUB            = 0x1000;
static const uint32_t SIGN_4BIT_MASK            = 0x08UL;
static const int32_t  SIGN_4BIT_SUB             = 16;
static const uint32_t SIGN_8BIT_MASK            = 0x80UL;
static const int32_t  SIGN_8BIT_SUB             = 0x100;
static const uint32_t NIBBLE_MASK               = 0x0FUL;
static const uint32_t STRIP_BIT7_MASK           = 0x7FUL;
static const uint32_t BYTE_MASK_U32             = 0xFFUL;
static const uint32_t SHIFT_4                   = 4U;
static const uint32_t SHIFT_8                   = 8U;
static const uint32_t SHIFT_16                  = 16U;
static const uint32_t SHIFT_22                  = 22U;
static const uint32_t SHIFT_24                  = 24U;
static const uint32_t SHIFT_32                  = 32U;
static const int64_t  Q22_HALF_ROUND            = 2097152LL;
static const int32_t  VIG_FULL_Q14              = 16384;
static const int32_t  VIG_FLOOR_Q14             = 256;
static const float    VIG_FULL_Q14_F            = 16384.0F;
enum {
    SONY_CURVE_PTS    = 16,
    SONY_REL_PTS      = 15,
    SONY_PACKED_BYTES = 9,
    SONY_LCA_GROUPS   = 3,
    SONY_VIG_GROUPS   = 4,
    LIVE_NODE_COUNT    = 4,
    ID05_FOCAL_GRP_IDX = 4,
    ID05_LCA_GRP_IDX   = 11,
    ID05_TAIL_GRP_IDX  = 15,
};
static const uint8_t  ID05_MIN_CURVE_FRAMES     = 2U;
static const uint8_t  SONY_SEL_VIG_2            = 2U;
static const uint8_t  SONY_SEL_DIST_LEG_4       = 4U;
static const uint8_t  SONY_SEL_DIST_21          = 21U;
static const uint8_t  SONY_SEL_LCA_RED_22       = 22U;
static const uint8_t  SONY_SEL_LCA_BLU_23       = 23U;
static const uint8_t  CMD24_RADIAL_LOCKED_BIT   = 0x10U;
static const uint8_t  CMD24_RADIAL_SEL21_BIT    = 0x20U;
static const uint32_t CURVE_VALID_DIST_BIT      = 0x01UL;
static const uint32_t CURVE_VALID_LCA_MASK      = 0x06UL;
static const uint32_t CURVE_VALID_ALL_MASK      = 0x07UL;
static const float    DECIMM_TO_MM_F            = 0.1F;

static const float LCA_SCALE_NORM               = 1.0F / 131072.0F;
static const float FIT_R1_SQ                    = 0.390625F;
static const float FIT_R1_QUAD                  = 0.15258789F;
static const float NORM_COORD_MAX_F             = 4095.0F;
static const float Q12_SCALE_F                  = 4096.0F;
static const float Q12_INV_F                    = 1.0F / 4096.0F;
static const float QA_INV_F                     = 1.0F / 32768.0F;   /* 2^-15 */
static const float QB_INV_F                     = 1.0F / 1048576.0F; /* 2^-20 */
static const float Q14_SCALE_F                  = 16384.0F;
static const float DS13_MIN_SCALE               = 0.5F;
static const float DS13_MAX_SCALE               = 1.5F;
static const float MAG_CODE_EXP_BASE            = 5.0F;
static const float MAG_CODE_DIV                 = 16.0F;
static const float MM_TO_M_F                    = 0.001F;
static const float TWO_F                        = 2.0F;
static const float HALF_ROUND_F                 = 0.5F;

typedef struct {
    float c1_quad;
    float c2_quart;
} radial_2coeff_t;

typedef struct {
    float r_c1;
    float r_c2;
    float b_c1;
    float b_c2;
} lca_coeff_quad_t;

/* NOLINTBEGIN(readability-magic-numbers) */
static const int32_t INV_T3_Q22[3][3] = {
    {44739243, -16777216, 1398101},
    {-134217728, 83886080, -8388608},
    {89478485, -67108864, 11184811},
};
static const int32_t INV_T4_Q22[4][4] = {
    {67108864, -50331648, 22369621, -4194304},
    {-290805077, 318767104, -156587349, 30758229},
    {402653184, -536870912, 313174699, -67108864},
    {-178956971, 268435456, -178956971, 44739243},
};
static const int32_t INV_U5_Q22[5][5] = {
    {4194304, 0, 0, 0, 0},
    {-34952533, 67108864, -50331648, 22369621, -4194304},
    {97867093, -290805077, 318767104, -156587349, 30758229},
    {-111848107, 402653184, -536870912, 313174699, -67108864},
    {44739243, -178956971, 268435456, -178956971, 44739243},
};
static const int32_t INV_V3_Q22[3][3] = {
    {4194304, 0, 0},
    {-12582912, 16777216, -4194304},
    {8388608, -16777216, 8388608},
};
static const int32_t INV_TB3_Q22[3][3] = {
    {4194304, 0, 0},
    {-29360128, 33554432, -4194304},
    {25165824, -33554432, 8388608},
};
static const int32_t INV_U6_Q22[6][6] = {
    {4194304, 0, 0, 0, 0, 0},
    {-37748736, 62914560, -35951177, 13481691, -2995931, 299593},
    {90248273, -231435703, 207147259, -82881492, 18822653, -1900990},
    {-166570667, 571619556, -726617397, 415841524, -105455746, 11182730},
    {191146667, -728177778, 1066260317, -741180952, 240558730, -28606984},
    {-81269841, 325079365, -510839002, 394739229, -150929705, 23219955},
};

static const float VIG4_PINV[4][15] = {
    {3.43585816e+00F, 4.46446580e+00F, 3.93475237e+00F, 2.55874706e+00F, 9.11578856e-01F, -5.68523467e-01F, -1.58023136e+00F, -1.95911648e+00F, -1.67765071e+00F, -8.45206159e-01F, 2.91944867e-01F, 1.35062983e+00F, 1.81077598e+00F, 1.01541036e+00F, -1.82934024e+00F},
    {-1.71032211e+01F, -2.12803916e+01F, -1.73207539e+01F, -9.21403937e+00F, -1.50467988e-01F, 7.47925142e+00F, 1.20839213e+01F, 1.28718552e+01F, 9.85087819e+00F, 3.82832630e+00F, -3.58895309e+00F, -9.99460136e+00F, -1.21827487e+01F, -6.14801396e+00F, 1.29144951e+01F},
    {2.61730571e+01F, 3.16027852e+01F, 2.42403926e+01F, 1.06756219e+01F, -3.86324923e+00F, -1.55094085e+01F, -2.17575089e+01F, -2.14636687e+01F, -1.48454714e+01F, -3.48196573e+00F, 9.68633423e+00F, 2.03574492e+01F, 2.28679345e+01F, 1.01928804e+01F, -2.60540883e+01F},
    {-1.25776606e+01F, -1.48579542e+01F, -1.08868799e+01F, -4.00410632e+00F, 3.15702819e+00F, 8.66951586e+00F, 1.13126793e+01F, 1.05721716e+01F, 6.63997621e+00F, 4.14407042e-01F, -6.49989156e+00F, -1.17919448e+01F, -1.24444476e+01F, -4.73376410e+00F, 1.57700716e+01F},
};
static const float VIG3_PINV[3][15] = {
    {4.77497769e-01F, 1.56354281e+00F, 2.81272431e+00F, 3.87864687e+00F, 4.51393051e+00F, 4.57021070e+00F, 3.99813831e+00F, 2.84737966e+00F, 1.26661648e+00F, -4.96454074e-01F, -2.09511941e+00F, -3.08365151e+00F, -2.91730693e+00F, -9.52326818e-01F, 3.55406313e+00F},
    {-1.23202099e+00F, -4.00666905e+00F, -7.14396354e+00F, -9.73344616e+00F, -1.11341809e+01F, -1.09747539e+01F, -9.15327389e+00F, -5.83737169e+00F, -1.46420048e+00F, 3.25956426e+00F, 7.35772475e+00F, 9.58456090e+00F, 8.42483034e+00F, 2.09376837e+00F, -1.14629120e+01F},
    {7.67093796e-01F, 2.48187679e+00F, 4.39556242e+00F, 5.93397886e+00F, 6.69756897e+00F, 6.46139036e+00F, 5.17511532e+00F, 2.96303089e+00F, 1.24038802e-01F, -2.86834448e+00F, -5.36598779e+00F, -6.54614522e+00F, -5.41145619e+00F, -7.89945355e-01F, 8.66497731e+00F},
};
static const float DIST_PCT_PINV[3][15] = {
    {1.22359876e+00F, 1.61606116e+01F, 6.61118243e+01F, 1.64369962e+02F, 3.04789146e+02F, 4.57452711e+02F, 5.71439378e+02F, 5.84687797e+02F, 4.40959439e+02F, 1.13899854e+02F, -3.61801710e+02F, -8.52154334e+02F, -1.07850909e+03F, -5.65104333e+02F, 1.42048733e+03F},
    {-2.06057402e+00F, -2.72889165e+01F, -1.12019663e+02F, -2.79751323e+02F, -5.21885423e+02F, -7.90136231e+02F, -1.00061106e+03F, -1.04956114e+03F, -8.40803077e+02F, -3.24810898e+02F, 4.50521359e+02F, 1.27744644e+03F, 1.71426013e+03F, 9.99526869e+02F, -2.04536512e+03F},
    {8.48404518e-01F, 1.12740631e+01F, 4.64780740e+01F, 1.16717527e+02F, 2.19376448e+02F, 3.35698746e+02F, 4.32183171e+02F, 4.66640276e+02F, 3.98911386e+02F, 2.06249568e+02F, -9.66373794e+01F, -4.32881948e+02F, -6.34089921e+02F, -4.06311375e+02F, 7.08650327e+02F},
};
static const float LCA_PINV[2][15] = {
    {3.24375659e-07F, 1.19165366e-06F, 2.44306052e-06F, 3.91982277e-06F, 5.46316694e-06F, 6.91431956e-06F, 8.11450715e-06F, 8.90495625e-06F, 9.12689337e-06F, 8.62154504e-06F, 7.23013780e-06F, 4.79389818e-06F, 1.15405268e-06F, -3.84817214e-06F, -1.03715498e-05F},
    {-3.66149914e-07F, -1.34146459e-06F, -2.74124141e-06F, -4.38077778e-06F, -6.07537108e-06F, -7.64031872e-06F, -8.89091809e-06F, -9.64246658e-06F, -9.71026158e-06F, -8.90960049e-06F, -7.05578070e-06F, -3.96409961e-06F, 5.50145387e-07F, 6.67165690e-06F, 1.45851375e-05F},
};
/* NOLINTEND(readability-magic-numbers) */

static void write_f32_le(uint8_t *dst, float val) {
    float clean = boryoza_isfinite_f32(val) ? val : 0.0F;
    union {
        float    f;
        uint32_t u;
    } cvt;
    cvt.f = clean;
    dst[0] = (uint8_t)(cvt.u & BYTE_MASK_U32);
    dst[1] = (uint8_t)((cvt.u >> SHIFT_8) & BYTE_MASK_U32);
    dst[2] = (uint8_t)((cvt.u >> SHIFT_16) & BYTE_MASK_U32);
    dst[3] = (uint8_t)((cvt.u >> SHIFT_24) & BYTE_MASK_U32);
}

static int32_t clamp_s32(int32_t val, int32_t lo_val, int32_t hi_val) {
    if (val < lo_val) {
        return lo_val;
    }
    if (val > hi_val) {
        return hi_val;
    }
    return val;
}

static int32_t sign_ext4(uint32_t raw_u) {
    uint32_t masked = raw_u & NIBBLE_MASK;
    return ((masked & SIGN_4BIT_MASK) != 0UL) ? ((int32_t)masked - SIGN_4BIT_SUB) : (int32_t)masked;
}

static int32_t sign_ext8(uint32_t raw_u) {
    return (int32_t)(int8_t)(uint8_t)(raw_u & BYTE_MASK_U32);
}

static int32_t sign_ext12(uint32_t raw_u) {
    uint32_t masked = raw_u & MASK_12BIT_U32;
    return ((masked & SIGN_12BIT_MASK) != 0UL) ? ((int32_t)masked - SIGN_12BIT_SUB) : (int32_t)masked;
}

static int32_t sign_ext16(uint32_t raw_u) {
    return (int32_t)(int16_t)(uint16_t)raw_u;
}

static int64_t ashr_q22(int64_t acc) {
    if (acc >= 0) {
        return (int64_t)((uint64_t)acc >> SHIFT_22);
    }
    uint64_t inv = ~(uint64_t)acc;
    return (int64_t)(~(inv >> SHIFT_22));
}

static int32_t ashr_s32(int32_t val, uint32_t shift) {
    if (shift >= SHIFT_32) {
        return (val >= 0) ? 0 : -1;
    }
    if (val >= 0) {
        return (int32_t)((uint32_t)val >> shift);
    }
    uint32_t inv = ~(uint32_t)val;
    return (int32_t)(~(inv >> shift));
}

static int32_t trunc_div_pos2(int32_t num, int32_t den) {
    return (num >= 0) ? (num / den) : -((-num) / den);
}

typedef struct {
    const uint8_t *src;
    size_t         len;
    size_t         byte_pos;
    uint32_t       bitbuf;
    uint8_t        bitcnt;
} opt_bitstream_t;

/* NOLINTBEGIN(readability-magic-numbers) — optical Huffman and packed block tables */
static void bs_refill(opt_bitstream_t *bstream, uint8_t need_bits) {
    while ((bstream->bitcnt < need_bits) && (bstream->byte_pos < bstream->len)) {
        bstream->bitbuf = (bstream->bitbuf << SHIFT_8) | (uint32_t)bstream->src[bstream->byte_pos];
        bstream->byte_pos += 1U;
        bstream->bitcnt = (uint8_t)(bstream->bitcnt + 8U);
    }
}

static uint32_t bs_read_bits(opt_bitstream_t *bstream, uint8_t count) {
    if (count == 0U) {
        return 0UL;
    }
    uint32_t out = 0UL;
    if (count > 24U) {
        uint8_t high_bits = (uint8_t)(count - 16U);
        out = bs_read_bits(bstream, high_bits) << 16U;
        count = 16U;
    }
    bs_refill(bstream, count);
    if (bstream->bitcnt < count) {
        return out;
    }
    bstream->bitcnt = (uint8_t)(bstream->bitcnt - count);
    out |= (bstream->bitbuf >> bstream->bitcnt) & ((1UL << count) - 1UL);
    return out;
}

typedef struct {
    const uint8_t *counts12;
    const uint8_t *syms;
    size_t         sym_count;
    uint8_t        sym[256];
    uint8_t        len[256];
} opt_huff_lut8_t;

static opt_huff_lut8_t s_opt_huff_lut8;

static void ensure_opt_huff_lut8(const uint8_t *counts12, const uint8_t *syms, size_t sym_count) {
    if ((counts12 == NULL) || (syms == NULL)) {
        return;
    }
    if ((s_opt_huff_lut8.counts12 == counts12) &&
        (s_opt_huff_lut8.syms == syms) &&
        (s_opt_huff_lut8.sym_count == sym_count)) {
        return;
    }
    memset(&s_opt_huff_lut8, 0, sizeof(s_opt_huff_lut8));
    s_opt_huff_lut8.counts12 = counts12;
    s_opt_huff_lut8.syms = syms;
    s_opt_huff_lut8.sym_count = sym_count;
    uint32_t code = 0UL;
    size_t idx = 0U;
    for (uint32_t bits = 1U; bits <= 8U; ++bits) {
        uint32_t cnt = (uint32_t)counts12[bits];
        uint32_t span = 1UL << (8U - bits);
        for (uint32_t k = 0U; k < cnt; ++k) {
            if (idx < sym_count) {
                uint32_t base = code << (8U - bits);
                uint8_t sym_val = syms[idx];
                for (uint32_t j = 0U; j < span; ++j) {
                    s_opt_huff_lut8.sym[base + j] = sym_val;
                    s_opt_huff_lut8.len[base + j] = (uint8_t)bits;
                }
            }
            code += 1UL;
            idx += 1U;
        }
        code <<= 1U;
    }
}

static int32_t bs_read_huff_residual(opt_bitstream_t *bstream,
                                     const uint8_t *counts12,
                                     const uint8_t *syms,
                                     size_t sym_count) {
    if ((counts12 == NULL) || (syms == NULL)) {
        return 0;
    }
    uint8_t cat = 0U;
    bool found = false;
    if (s_opt_huff_lut8.counts12 == counts12) {
        bs_refill(bstream, 8U);
        if (bstream->bitcnt >= 8U) {
            uint32_t peek8 = (bstream->bitbuf >> (bstream->bitcnt - 8U)) & 0xFFUL;
            uint8_t lut_len = s_opt_huff_lut8.len[peek8];
            if (lut_len != 0U) {
                cat = s_opt_huff_lut8.sym[peek8];
                bstream->bitcnt = (uint8_t)(bstream->bitcnt - lut_len);
                found = true;
            }
        }
    }
    if (!found) {
        uint32_t code = 0UL;
        uint32_t first = 0UL;
        size_t idx = 0U;
        for (uint8_t len = 1U; len <= 11U; ++len) {
            bs_refill(bstream, 1U);
            if (bstream->bitcnt == 0U) {
                break;
            }
            bstream->bitcnt = (uint8_t)(bstream->bitcnt - 1U);
            uint32_t bit = (bstream->bitbuf >> bstream->bitcnt) & 1UL;
            code |= bit;
            uint32_t cnt = (uint32_t)counts12[len];
            if (code < (first + cnt)) {
                size_t sym_off = idx + (size_t)(code - first);
                if (sym_off < sym_count) {
                    cat = syms[sym_off];
                    found = true;
                }
                break;
            }
            idx += (size_t)cnt;
            first = (first + cnt) << 1U;
            code <<= 1U;
        }
    }
    if (!found || (cat == 0U)) {
        return 0;
    }
    uint8_t n_extra = (uint8_t)(cat - 1U);
    uint32_t low = bs_read_bits(bstream, n_extra);
    uint32_t zigzag = (1UL << n_extra) | low;
    return (int32_t)((zigzag >> 1U) ^ (0UL - (zigzag & 1UL)));
}

static uint64_t read_le56(const uint8_t *src) {
    uint64_t val = 0ULL;
    for (size_t idx = 0U; idx < 7U; ++idx) {
        val |= ((uint64_t)src[idx]) << (idx * 8U);
    }
    return val;
}

static void compute_cmd24_a_stage2(const int32_t prev[60], size_t n_rad, int64_t stage2[4][5][3]) {
    int64_t stage1[4][5][3] = {{{0}}};
    for (size_t ri = 0U; ri < n_rad; ++ri) {
        for (size_t ui = 0U; ui < 5U; ++ui) {
            const int32_t *row_ptr = &prev[(ri * 15U) + (ui * 3U)];
            for (size_t ci = 0U; ci < 3U; ++ci) {
                int64_t acc = Q22_HALF_ROUND;
                for (size_t vi = 0U; vi < 3U; ++vi) {
                    acc += (int64_t)INV_V3_Q22[ci][vi] * (int64_t)row_ptr[vi];
                }
                stage1[ri][ui][ci] = ashr_q22(acc);
            }
        }
        for (size_t bi = 0U; bi < 5U; ++bi) {
            for (size_t ci = 0U; ci < 3U; ++ci) {
                int64_t acc = Q22_HALF_ROUND;
                for (size_t ui = 0U; ui < 5U; ++ui) {
                    acc += (int64_t)INV_U5_Q22[bi][ui] * stage1[ri][ui][ci];
                }
                stage2[ri][bi][ci] = ashr_q22(acc);
            }
        }
    }
}

static void write_cmd24_a_stage3(const int64_t stage2[4][5][3], size_t n_rad, uint8_t *dst_node) {
    for (size_t ai = 0U; ai < n_rad; ++ai) {
        const int32_t *inv_row = (n_rad == 4U) ? INV_T4_Q22[ai] : INV_T3_Q22[ai];
        for (size_t bi = 0U; bi < 5U; ++bi) {
            for (size_t ci = 0U; ci < 3U; ++ci) {
                int64_t acc = Q22_HALF_ROUND;
                for (size_t ri = 0U; ri < n_rad; ++ri) {
                    acc += (int64_t)inv_row[ri] * stage2[ri][bi][ci];
                }
                int64_t stage3 = ashr_q22(acc);
                size_t f_idx = (((ai * 5U) + bi) * 3U) + ci;
                write_f32_le(&dst_node[f_idx * sizeof(float)], (float)stage3 * QA_INV_F);
            }
        }
    }
}

typedef struct {
    bool    exact_q12;
    bool    u_const;
    bool    sparse_r1_v0;
    uint8_t flag;
    size_t  n_rad;
    size_t  n_cap;
    size_t  per_node;
    size_t  vals_per_node;
} cmd24_a_cfg_t;

static size_t unpack_cmd24_a_masks(const uint8_t *packed, size_t packed_len,
                                   size_t cursor, uint8_t *buf_a) {
    uint64_t misc_mask = read_le56(&packed[cursor]);
    cursor += 7U;
    for (size_t idx = 0U; idx < 53U; ++idx) {
        if (((misc_mask >> idx) & 1ULL) == 0ULL) {
            continue;
        }
        if (cursor >= packed_len) {
            return 0U;
        }
        size_t dst_off = (idx < 13U) ? (7156U + idx) : (7385U + (idx - 13U));
        buf_a[dst_off] = packed[cursor];
        cursor += 1U;
    }

    if ((cursor + 7U) > packed_len) {
        return 0U;
    }
    uint64_t dist_mask = read_le56(&packed[cursor]);
    cursor += 7U;
    for (size_t idx = 0U; idx < 54U; ++idx) {
        if (((dist_mask >> idx) & 1ULL) == 0ULL) {
            continue;
        }
        if ((cursor + 4U) > packed_len) {
            return 0U;
        }
        memcpy(&buf_a[7169U + (idx * 4U)], &packed[cursor], 4U);
        cursor += 4U;
    }
    return cursor;
}

static void write_cmd24_a_node(const cmd24_a_cfg_t *cfg, size_t slot_idx,
                               const int32_t prev1[60], uint8_t *buf_a) {
    size_t node_off = CMD24_A_COEFF_START + (slot_idx * cfg->per_node * sizeof(float));
    if (cfg->exact_q12) {
        if (cfg->sparse_r1_v0) {
            for (size_t ki = 0U; ki < 15U; ++ki) {
                write_f32_le(&buf_a[node_off + ((15U + (ki * 3U)) * sizeof(float))],
                             (float)prev1[ki] * Q12_INV_F);
            }
        } else {
            for (size_t ki = 0U; ki < cfg->per_node; ++ki) {
                write_f32_le(&buf_a[node_off + (ki * sizeof(float))], (float)prev1[ki] * Q12_INV_F);
            }
        }
        return;
    }
    int32_t full[60] = {0};
    if (cfg->u_const) {
        for (size_t ri = 0U; ri < cfg->n_rad; ++ri) {
            for (size_t ui = 0U; ui < 5U; ++ui) {
                for (size_t vi = 0U; vi < 3U; ++vi) {
                    full[((((ri * 5U) + ui) * 3U) + vi)] = prev1[(ri * 3U) + vi];
                }
            }
        }
    } else {
        memcpy(full, prev1, cfg->per_node * sizeof(int32_t));
    }

    int64_t stage2[4][5][3] = {{{0}}};
    compute_cmd24_a_stage2(full, cfg->n_rad, stage2);
    write_cmd24_a_stage3(stage2, cfg->n_rad, &buf_a[node_off]);
}

typedef struct {
    const uint8_t *counts;
    const uint8_t *syms;
    size_t         sym_cnt;
} cmd24_a_huff_t;

typedef struct {
    const uint8_t *slots;
    const uint8_t *ap_coords;
    uint8_t        n_act;
} cmd24_a_nodes_t;

static void decode_cmd24_a_nodes(const cmd24_a_cfg_t *cfg, const cmd24_a_huff_t *huff,
                                 const cmd24_a_nodes_t *nodes, opt_bitstream_t *bstream,
                                 uint8_t *buf_a) {
    int32_t prev1[60] = {0};
    int32_t prev2[60] = {0};
    bool is_f3 = (bool)(cfg->flag == 3U);
    for (size_t k = 0U; k < (size_t)nodes->n_act; ++k) {
        bool has_prev2 = (bool)(k >= 2U);
        bool use_o2 = (bool)(((!cfg->exact_q12) || cfg->sparse_r1_v0) && is_f3 && has_prev2);
        int32_t da1 = 0;
        int32_t da0 = 1;
        if (use_o2) {
            da1 = (int32_t)nodes->ap_coords[k] - (int32_t)nodes->ap_coords[k - 1U];
            da0 = (int32_t)nodes->ap_coords[k - 1U] - (int32_t)nodes->ap_coords[k - 2U];
            if (da0 < 1) {
                da0 = 1;
            }
        }
        for (size_t ki = 0U; ki < cfg->vals_per_node; ++ki) {
            int32_t pred = prev1[ki];
            if (use_o2) {
                pred += ((prev1[ki] - prev2[ki]) * da1) / da0;
            }
            int32_t val = pred + bs_read_huff_residual(bstream, huff->counts, huff->syms, huff->sym_cnt);
            prev2[ki] = prev1[ki];
            prev1[ki] = val;
        }
        if (nodes->slots[k] < cfg->n_cap) {
            write_cmd24_a_node(cfg, (size_t)nodes->slots[k], prev1, buf_a);
        }
    }
}

static void unpack_cmd24_a_stream(const uint8_t *packed, size_t packed_len, uint8_t *buf_a) {
    if ((packed == NULL) || (packed_len < 16U)) {
        return;
    }
    uint8_t hdr_flags = packed[0];
    uint8_t n_act = packed[1];
    bool exact_q12 = ((hdr_flags & 0x80U) != 0U);
    bool u_const = ((hdr_flags & 0x40U) != 0U);
    bool has_sparse_r1 = ((hdr_flags & 0x20U) != 0U);
    bool sparse_r1_v0 = (bool)(exact_q12 && has_sparse_r1);
    uint8_t flag = (uint8_t)(hdr_flags & 0x0FU);
    size_t n_rad = (flag == 3U) ? 4U : 3U;
    size_t n_cap = (flag == 3U) ? 27U : 36U;
    size_t per_node = n_rad * 15U;
    size_t vals_per_node = per_node;
    if (sparse_r1_v0) {
        vals_per_node = 15U;
    } else if ((!exact_q12) && u_const) {
        vals_per_node = n_rad * 3U;
    }
    if (((size_t)n_act > n_cap) || (packed_len < (2U + ((size_t)n_act * 2U) + 14U))) {
        return;
    }

    memset(buf_a, 0, Z_CMD24_A_STRIDE);
    buf_a[0] = Z_SYNC_55;
    buf_a[1] = flag;
    memset(&buf_a[2], 0xFF, 36U);

    uint8_t slots[36] = {0};
    uint8_t ap_coords[36] = {0};
    size_t cursor = 2U;
    for (size_t k = 0U; k < (size_t)n_act; ++k) {
        uint8_t slot_idx = packed[cursor];
        uint8_t ap_val = packed[cursor + 1U];
        cursor += 2U;
        if (slot_idx < n_cap) {
            buf_a[2U + slot_idx] = ap_val;
        }
        slots[k] = slot_idx;
        ap_coords[k] = ap_val;
    }

    cursor = unpack_cmd24_a_masks(packed, packed_len, cursor, buf_a);
    if (cursor == 0U) {
        return;
    }

    cmd24_a_huff_t huff = {NULL, NULL, 0U};
    lens_db_get_opt_huff(&huff.counts, &huff.syms, &huff.sym_cnt);
    ensure_opt_huff_lut8(huff.counts, huff.syms, huff.sym_cnt);
    opt_bitstream_t bstream = {packed + cursor, packed_len - cursor, 0U, 0UL, 0U};
    cmd24_a_cfg_t cfg = {exact_q12, u_const, sparse_r1_v0, flag, n_rad, n_cap, per_node, vals_per_node};
    cmd24_a_nodes_t nodes = {slots, ap_coords, n_act};
    decode_cmd24_a_nodes(&cfg, &huff, &nodes, &bstream, buf_a);
}

static void compute_cmd24_b_stage2(const int32_t *plane54, int64_t stage2[3][6][3]) {
    int64_t stage1[3][6][3] = {{{0}}};
    for (size_t ri = 0U; ri < 3U; ++ri) {
        for (size_t ui = 0U; ui < 6U; ++ui) {
            const int32_t *row_ptr = &plane54[(ri * 18U) + (ui * 3U)];
            for (size_t ci = 0U; ci < 3U; ++ci) {
                int64_t acc = Q22_HALF_ROUND;
                for (size_t vi = 0U; vi < 3U; ++vi) {
                    acc += (int64_t)INV_V3_Q22[ci][vi] * (int64_t)row_ptr[vi];
                }
                stage1[ri][ui][ci] = ashr_q22(acc);
            }
        }
        for (size_t bi = 0U; bi < 6U; ++bi) {
            for (size_t ci = 0U; ci < 3U; ++ci) {
                int64_t acc = Q22_HALF_ROUND;
                for (size_t ui = 0U; ui < 6U; ++ui) {
                    acc += (int64_t)INV_U6_Q22[bi][ui] * stage1[ri][ui][ci];
                }
                stage2[ri][bi][ci] = ashr_q22(acc);
            }
        }
    }
}

static void write_cmd24_b_stage3(const int64_t stage2[3][6][3], uint8_t *dst_plane) {
    for (size_t ai = 0U; ai < 3U; ++ai) {
        for (size_t bi = 0U; bi < 6U; ++bi) {
            for (size_t ci = 0U; ci < 3U; ++ci) {
                int64_t acc = Q22_HALF_ROUND;
                for (size_t ri = 0U; ri < 3U; ++ri) {
                    acc += (int64_t)INV_TB3_Q22[ai][ri] * stage2[ri][bi][ci];
                }
                int64_t stage3 = ashr_q22(acc);
                size_t f_idx = (((ai * 6U) + bi) * 3U) + ci;
                write_f32_le(&dst_plane[f_idx * sizeof(float)], (float)stage3 * QB_INV_F);
            }
        }
    }
}

static void write_cmd24_b_node(bool u_const, const int32_t prev[108], uint8_t *dst_node) {
    int32_t plane54[54] = {0};
    for (size_t pi = 0U; pi < 2U; ++pi) {
        if (u_const) {
            for (size_t ti = 0U; ti < 3U; ++ti) {
                for (size_t ui = 0U; ui < 6U; ++ui) {
                    for (size_t vi = 0U; vi < 3U; ++vi) {
                        plane54[(((ti * 6U) + ui) * 3U) + vi] =
                            prev[(pi * 9U) + (ti * 3U) + vi];
                    }
                }
            }
        } else {
            memcpy(plane54, &prev[pi * 54U], 54U * sizeof(int32_t));
        }
        int64_t stage2[3][6][3] = {{{0}}};
        compute_cmd24_b_stage2(plane54, stage2);
        write_cmd24_b_stage3(stage2, &dst_node[pi * 54U * sizeof(float)]);
    }
}

static void unpack_cmd24_b_stream(const uint8_t *packed, size_t packed_len, uint8_t *buf_b) {
    if ((packed == NULL) || (packed_len < 3U)) {
        return;
    }
    memset(buf_b, 0, Z_CMD24_B_BYTES);
    buf_b[0] = Z_SYNC_55;
    uint8_t flags_b = packed[0];
    bool has_same_mask = ((flags_b & 0x80U) != 0U);
    bool u_const = ((flags_b & 0x40U) != 0U);
    buf_b[1] = (uint8_t)(flags_b & 0x3FU);
    buf_b[3466] = packed[1];
    uint8_t mask = packed[2];
    memset(&buf_b[2], 0xFF, 8U);

    size_t cursor = 3U;
    uint8_t slots_b[8] = {0};
    size_t n_act = 0U;
    for (size_t node = 0U; node < 8U; ++node) {
        if ((((uint32_t)mask >> node) & 1UL) == 0UL) {
            continue;
        }
        if (cursor >= packed_len) {
            return;
        }
        buf_b[2U + node] = packed[cursor];
        cursor += 1U;
        slots_b[n_act] = (uint8_t)node;
        n_act += 1U;
    }

    uint8_t same_mask = 0U;
    if (has_same_mask) {
        if (cursor >= packed_len) {
            return;
        }
        same_mask = packed[cursor];
        cursor += 1U;
    }

    const uint8_t *huff_counts = NULL;
    const uint8_t *huff_syms = NULL;
    size_t huff_sym_cnt = 0U;
    lens_db_get_opt_huff(&huff_counts, &huff_syms, &huff_sym_cnt);
    ensure_opt_huff_lut8(huff_counts, huff_syms, huff_sym_cnt);
    opt_bitstream_t bstream = {packed + cursor, packed_len - cursor, 0U, 0UL, 0U};

    size_t vals_per_node = u_const ? 18U : 108U;
    int32_t prev[108] = {0};
    size_t prev_node_off = CMD24_B_COEFF_START;
    for (size_t k = 0U; k < n_act; ++k) {
        size_t node_off = CMD24_B_COEFF_START + ((size_t)slots_b[k] * 108U * sizeof(float));
        if ((k >= 1U) && ((((uint32_t)same_mask >> k) & 1UL) != 0UL)) {
            memcpy(&buf_b[node_off], &buf_b[prev_node_off], 108U * sizeof(float));
            prev_node_off = node_off;
            continue;
        }
        for (size_t ki = 0U; ki < vals_per_node; ++ki) {
            prev[ki] += bs_read_huff_residual(&bstream, huff_counts, huff_syms, huff_sym_cnt);
        }
        write_cmd24_b_node(u_const, prev, &buf_b[node_off]);
        prev_node_off = node_off;
    }
}

void optics_decode_sony_9b_16pt(const uint8_t packed9[9], uint8_t selector, int16_t out16[16]) {
    int32_t exp_s4 = sign_ext4((uint32_t)packed9[0] >> SHIFT_4);
    int32_t pts[16] = {0};
    pts[10] = sign_ext12(((uint32_t)packed9[0] << SHIFT_8) | (uint32_t)packed9[1]);
    pts[8]  = sign_ext16((uint32_t)(pts[10] - sign_ext8(packed9[2])));
    pts[6]  = sign_ext16((uint32_t)(pts[8]  - sign_ext8(packed9[3])));
    pts[4]  = sign_ext16((uint32_t)(pts[6]  - sign_ext8(packed9[4])));
    pts[2]  = sign_ext16((uint32_t)(pts[4]  - sign_ext8(packed9[5])));
    pts[12] = sign_ext16((uint32_t)(pts[10] + sign_ext8(packed9[8])));
    pts[14] = sign_ext16((uint32_t)(pts[12] + sign_ext8(packed9[7])));
    pts[15] = sign_ext16((uint32_t)(pts[14] + sign_ext8(packed9[6])));

    uint32_t mode = 2UL;
    if ((selector <= 24U) && (((0x01E00078UL >> selector) & 1UL) != 0UL)) {
        mode = 1UL;
    } else if (selector == 2U) {
        mode = 0UL;
    }
    if (mode == 1UL) {
        pts[0] = sign_ext16((uint32_t)((3 * (pts[2] - pts[4])) + pts[6]));
    } else if (mode == 2UL) {
        int32_t old4 = pts[4];
        pts[0] = pts[2];
        pts[2] = old4;
        pts[4] = trunc_div_pos2(pts[6] + old4, 2);
    }
    pts[1] = sign_ext16((uint32_t)trunc_div_pos2((5 * (pts[0] - pts[4])) + (15 * pts[2]) + pts[6] + 8, 16));
    pts[13] = sign_ext16((uint32_t)trunc_div_pos2((5 * (pts[14] - pts[10])) + (15 * pts[12]) + pts[8] + 8, 16));
    static const uint8_t ODD_IDX[5] = {11U, 9U, 7U, 5U, 3U};
    for (size_t idx_k = 0U; idx_k < 5U; ++idx_k) {
        uint8_t idx = ODD_IDX[idx_k];
        pts[idx] = sign_ext16((uint32_t)trunc_div_pos2(
            (9 * (pts[idx - 1U] + pts[idx + 1U])) - pts[idx - 3U] - pts[idx + 3U] + 8, 16));
    }
    for (size_t idx = 0U; idx < 16U; ++idx) {
        int32_t val = ((mode == 0UL) && (pts[idx] < 0)) ? 0 : pts[idx];
        int32_t shifted = (exp_s4 >= 0)
                              ? ashr_s32(val, (uint32_t)exp_s4)
                              : (int32_t)((uint32_t)val << (uint32_t)(-exp_s4));
        out16[idx] = (int16_t)sign_ext16((uint32_t)shifted);
    }
}

void optics_decode_emount_9b(const uint8_t core6[E_ID05_CURVE_CORE_BYTES],
                             bool mask_bit7,
                             const uint8_t tail3[E_ID05_CURVE_TAIL_BYTES],
                             int32_t out_nodes8[E_CURVE_NODES_COUNT]) {
    uint32_t byte0 = mask_bit7 ? ((uint32_t)core6[0] & STRIP_BIT7_MASK) : (uint32_t)core6[0];
    int32_t exp_s4 = sign_ext4((byte0 >> SHIFT_4) & NIBBLE_MASK);
    uint32_t raw_u12 = ((byte0 & NIBBLE_MASK) << SHIFT_8) | (uint32_t)core6[1];
    int32_t anchor_c3 = ((raw_u12 & SIGN_12BIT_MASK) != 0UL)
                            ? ((int32_t)raw_u12 - SIGN_12BIT_SUB)
                            : (int32_t)raw_u12;

    int32_t diffs[7];
    for (size_t idx = 0U; idx < 4U; ++idx) {
        uint32_t raw_b = core6[2U + idx];
        diffs[idx] = ((raw_b & SIGN_8BIT_MASK) != 0UL)
                         ? ((int32_t)raw_b - SIGN_8BIT_SUB)
                         : (int32_t)raw_b;
    }
    for (size_t idx = 0U; idx < 3U; ++idx) {
        uint32_t raw_b = tail3[idx];
        diffs[4U + idx] = ((raw_b & SIGN_8BIT_MASK) != 0UL)
                              ? ((int32_t)raw_b - SIGN_8BIT_SUB)
                              : (int32_t)raw_b;
    }

    int32_t raw8[8];
    raw8[4] = anchor_c3;
    raw8[3] = sign_ext16((uint32_t)(raw8[4] - diffs[0]));
    raw8[2] = sign_ext16((uint32_t)(raw8[3] - diffs[1]));
    raw8[1] = sign_ext16((uint32_t)(raw8[2] - diffs[2]));
    raw8[0] = sign_ext16((uint32_t)(raw8[1] - diffs[3]));
    raw8[5] = sign_ext16((uint32_t)(raw8[4] + diffs[6]));
    raw8[6] = sign_ext16((uint32_t)(raw8[5] + diffs[5]));
    raw8[7] = sign_ext16((uint32_t)(raw8[6] + diffs[4]));

    for (size_t idx = 0U; idx < 8U; ++idx) {
        int32_t shifted = (exp_s4 >= 0)
                              ? ashr_s32(raw8[idx], (uint32_t)exp_s4)
                              : (int32_t)((uint32_t)raw8[idx] << (uint32_t)(-exp_s4));
        out_nodes8[idx] = sign_ext16((uint32_t)shifted);
    }
}
/* NOLINTEND(readability-magic-numbers) */

static radial_2coeff_t fit_radial_2coeff_zero_center(const int32_t nodes8[E_CURVE_NODES_COUNT],
                                                     float unit_scale) {
    static const size_t IDX_MID_NODE  = 4U;
    static const size_t IDX_EDGE_NODE = 7U;
    float center_y = (float)nodes8[0] * unit_scale;
    float y_mid = ((float)nodes8[IDX_MID_NODE] * unit_scale) - center_y;
    float y_edge = ((float)nodes8[IDX_EDGE_NODE] * unit_scale) - center_y;
    float denom = FIT_R1_SQ - FIT_R1_QUAD;
    radial_2coeff_t out;
    out.c1_quad = (y_mid - (FIT_R1_QUAD * y_edge)) / denom;
    out.c2_quart = y_edge - out.c1_quad;
    return out;
}

static void write_cmd24_b_node_lca(uint8_t *buf_b, size_t node_idx, const lca_coeff_quad_t *lca) {
    if (node_idx >= (size_t)CMD24_B_NODE_SLOTS) {
        return;
    }
    size_t base = CMD24_B_COEFF_START + (node_idx * CMD24_B_FLOATS_PER_NODE * sizeof(float));
    /* Keep slot 0 (Red c0) and slot 54 (Blue c0) strictly 0.0f so optical center shift == 0 */
    write_f32_le(&buf_b[base], 0.0F);
    write_f32_le(&buf_b[base + (DIST_RADIAL_STRIDE * sizeof(float))], lca->r_c1);
    write_f32_le(&buf_b[base + (2U * DIST_RADIAL_STRIDE * sizeof(float))], lca->r_c2);
    write_f32_le(&buf_b[base + (CMD24_B_BLUE_OFF * sizeof(float))], 0.0F);
    write_f32_le(&buf_b[base + ((CMD24_B_BLUE_OFF + DIST_RADIAL_STRIDE) * sizeof(float))], lca->b_c1);
    write_f32_le(&buf_b[base + ((CMD24_B_BLUE_OFF + (2U * DIST_RADIAL_STRIDE)) * sizeof(float))], lca->b_c2);
}

static void populate_spectral_179b(const adapter_ctx_t *ctx, uint8_t *buf_a) {
    memset(&buf_a[Z_CMD24_SPECTRAL_OFFSET], 0, Z_CMD24_SPECTRAL_TOTAL);
    const lens_db_entry_t *entry = lens_db_lookup(ctx->e_lens_id);
    if ((entry != NULL) && (entry->spectral_idx >= 0)) {
        (void)lens_db_unpack_spectral81(entry->spectral_idx, &buf_a[Z_CMD24_SPECTRAL_OFFSET]);
    }
}

static void build_sigma_150600_tables(uint8_t *buf_a, uint8_t *buf_b) {
    size_t packed_len = 0U;
    const uint8_t *packed_a = sigma_150600_get_optical_a_packed(&packed_len);
    unpack_cmd24_a_stream(packed_a, packed_len, buf_a);

    size_t kcount = 0U;
    const sigma_150600_optics_knot_t *knots = sigma_150600_get_optics_knots(&kcount);
    for (size_t kidx = 0U; kidx < kcount; ++kidx) {
        int32_t red8[E_CURVE_NODES_COUNT];
        int32_t blu8[E_CURVE_NODES_COUNT];
        for (size_t nidx = 0U; nidx < (size_t)E_CURVE_NODES_COUNT; ++nidx) {
            red8[nidx] = knots[kidx].lca_r_nodes8[nidx];
            blu8[nidx] = knots[kidx].lca_b_nodes8[nidx];
        }
        radial_2coeff_t rfit = fit_radial_2coeff_zero_center(red8, LCA_SCALE_NORM);
        radial_2coeff_t bfit = fit_radial_2coeff_zero_center(blu8, LCA_SCALE_NORM);
        lca_coeff_quad_t lca_quad = {rfit.c1_quad, rfit.c2_quart, bfit.c1_quad, bfit.c2_quart};
        write_cmd24_b_node_lca(buf_b, kidx, &lca_quad);
    }
}

static bool has_nonzero_distortion_54f(const uint8_t *buf_a) {
    for (size_t idx = 0U; idx < ((size_t)Z_CMD24_DISTORTION_FLOATS * sizeof(float)); ++idx) {
        if (buf_a[(size_t)Z_CMD24_DISTORTION_OFFSET + idx] != 0U) {
            return true;
        }
    }
    return false;
}

void optics_disconnect_post_init(adapter_ctx_t *ctx) {
    if (ctx->optics_disconnected != 0U) {
        return;
    }
    ctx->optics_disconnected = 1U;
    ctx->id05.optics_skip = 1U;
    ctx->id05.optics_curve_dirty = 0U;
    if ((ctx->session_state != E_SESSION_EMPTY_MOUNT) &&
        (ctx->session_state != E_SESSION_PASSIVE_MOUNT) &&
        (ctx->e_lens_id != 0U)) {
        static const uint32_t OPTICS_ZOOM_SHIFT = 16U;
        uint32_t arg0 = ((uint32_t)ctx->optics_ready) |
                        ((uint32_t)ctx->is_fisheye << SHIFT_8) |
                        (((ctx->f_max_mm > ctx->f_min_mm) ? 1UL : 0UL) << OPTICS_ZOOM_SHIFT);
        uint32_t arg2 = ((uint32_t)ctx->id05.optical_curve_valid) |
                        ((uint32_t)ctx->id05.curve_frames_seen << SHIFT_8);
        lens_session_push_diag(ctx, BB_EVT_OPTICS_DISCONNECTED,
                               arg0, ctx->id05.active_mask, arg2,
                               (uint32_t)ctx->cmd24_read_mask);
    }
}

void optics_build_initial_tables(adapter_ctx_t *ctx, optics_workspace_t *optics) {
    if ((ctx->session_epoch != 0U) &&
        (ctx->optics_built_epoch == ctx->session_epoch) &&
        (ctx->optics_built_lid == ctx->e_lens_id) &&
        (ctx->optics_built_fisheye == ctx->is_fisheye) &&
        ((ctx->optics_ready != 0U) || (ctx->f_max_mm <= ctx->f_min_mm)) &&
        (ctx->session_state != E_SESSION_EMPTY_MOUNT) &&
        (ctx->session_state != E_SESSION_PASSIVE_MOUNT)) {
        return;
    }
    uint8_t next_bank = (uint8_t)((uint32_t)optics->active_optics_bank ^ 1UL);
    uint8_t *buf_a = optics->cmd24_sub00[next_bank];
    uint8_t *buf_b = optics->cmd24_sub10[next_bank];

    memset(buf_a, 0, Z_CMD24_A_STRIDE);
    memset(buf_b, 0, Z_CMD24_B_BYTES);

    buf_a[0] = Z_SYNC_55;
    buf_a[1] = CMD24_A_FLAG_QUARTIC_3;
    memset(&buf_a[CMD24_A_NODE_START], 0xFF, Z_CMD24_A_NODE_SLOTS);

    static const uint8_t CMD24_B_NODES[8] = {
        0x00U, 0x0CU, 0x18U, 0x24U, 0x30U, 0x3CU, 0x48U, 0x4CU,
    };
    buf_b[0] = Z_SYNC_55;
    buf_b[1] = CMD24_B_FLAG_STD;
    memcpy(&buf_b[2], CMD24_B_NODES, sizeof(CMD24_B_NODES));

    const lens_db_entry_t *entry = lens_db_lookup(ctx->e_lens_id);
    const tamron_z_optics_profile_t *tam_prof =
        (entry != NULL) ? lens_db_get_tamron_optics(entry->tamron_z_idx) : NULL;

    bool static_optics = false;
    if (tam_prof != NULL) {
        unpack_cmd24_a_stream(tam_prof->cmd24_a_packed, tam_prof->cmd24_a_packed_len, buf_a);
        unpack_cmd24_b_stream(tam_prof->cmd24_b_packed, tam_prof->cmd24_b_packed_len, buf_b);
        static_optics = true;
    } else if ((ctx->e_lens_id == SIGMA_150600_REV2) || (ctx->e_lens_id == SIGMA_150600_REV1)) {
        build_sigma_150600_tables(buf_a, buf_b);
        static_optics = true;
    } else if (ctx->f_max_mm > ctx->f_min_mm) {
        static_optics = true;
    } else {
        static const uint8_t LIVE_A_NODES[4] = {0x00U, 0x0CU, 0x18U, 0x4CU};
        memcpy(&buf_a[CMD24_A_NODE_START], LIVE_A_NODES, sizeof(LIVE_A_NODES));
    }

    if (ctx->is_fisheye != 0U) {
        memset(&buf_a[Z_CMD24_DISTORTION_OFFSET], 0,
               (size_t)Z_CMD24_DISTORTION_FLOATS * sizeof(float));
    }

    populate_spectral_179b(ctx, buf_a);
    buf_a[Z_CMD24_FORMAT_OFFSET] =
        ((ctx->is_fisheye == 0U) && has_nonzero_distortion_54f(buf_a))
            ? CMD24_DIST_FORMAT_3
            : CMD24_DIST_FORMAT_NONE;

    boryoza_seal_z_frame(buf_a, Z_CMD24_A_BYTES);
    boryoza_seal_z_frame(buf_b, Z_CMD24_B_BYTES);
    optics->active_optics_bank = next_bank;
    ctx->optics_built_epoch = ctx->session_epoch;
    ctx->optics_built_lid = ctx->e_lens_id;
    ctx->optics_built_fisheye = ctx->is_fisheye;
    if (static_optics || (ctx->session_state == E_SESSION_EMPTY_MOUNT) ||
        (ctx->session_state == E_SESSION_PASSIVE_MOUNT)) {
        ctx->optics_ready = 1U;
        optics_disconnect_post_init(ctx);
    } else {
        ctx->optics_ready = 0U;
        ctx->optics_disconnected = 0U;
        ctx->id05.optics_skip = 0U;
    }
}

static void fit_live_vignette_quartic(const int16_t vig16[SONY_CURVE_PTS],
                                      float out_c[SONY_VIG_GROUPS]) {
    float targets[SONY_REL_PTS];
    for (size_t idx = 0U; idx < (size_t)SONY_REL_PTS; ++idx) {
        int32_t trans = VIG_FULL_Q14 - (int32_t)vig16[idx + 1U];
        if (trans < VIG_FLOOR_Q14) {
            trans = VIG_FLOOR_Q14;
        }
        targets[idx] = (VIG_FULL_Q14_F / (float)trans) - 1.0F;
    }
    for (size_t gi = 0U; gi < (size_t)SONY_VIG_GROUPS; ++gi) {
        float acc = 0.0F;
        for (size_t idx = 0U; idx < (size_t)SONY_REL_PTS; ++idx) {
            acc += VIG4_PINV[gi][idx] * targets[idx];
        }
        out_c[gi] = acc;
    }
    if (out_c[0] < 0.0F) {
        out_c[0] = 0.0F;
        for (size_t gi = 0U; gi < (size_t)SONY_LCA_GROUPS; ++gi) {
            float acc = 0.0F;
            for (size_t idx = 0U; idx < (size_t)SONY_REL_PTS; ++idx) {
                acc += VIG3_PINV[gi][idx] * targets[idx];
            }
            out_c[gi + 1U] = acc;
        }
    }
}

static void fit_live_distortion_abc_pct(const int16_t dist16[SONY_CURVE_PTS],
                                        float out_abc_pct[SONY_LCA_GROUPS]) {
    int32_t c0_den = VIG_FULL_Q14 - (int32_t)dist16[0];
    if (c0_den < VIG_FLOOR_Q14) {
        c0_den = VIG_FLOOR_Q14;
    }
    float k_center = VIG_FULL_Q14_F / (float)c0_den;
    float delta_rel[SONY_REL_PTS];
    for (size_t idx = 0U; idx < (size_t)SONY_REL_PTS; ++idx) {
        int32_t den = VIG_FULL_Q14 - (int32_t)dist16[idx + 1U];
        if (den < VIG_FLOOR_Q14) {
            den = VIG_FLOOR_Q14;
        }
        delta_rel[idx] = ((VIG_FULL_Q14_F / (float)den) / k_center) - 1.0F;
    }
    for (size_t gi = 0U; gi < (size_t)SONY_LCA_GROUPS; ++gi) {
        float acc = 0.0F;
        for (size_t idx = 0U; idx < (size_t)SONY_REL_PTS; ++idx) {
            acc += DIST_PCT_PINV[gi][idx] * delta_rel[idx];
        }
        out_abc_pct[gi] = acc;
    }
}

static void fit_live_lca_c1_c2(const int16_t lca16[SONY_CURVE_PTS], float *out_c1, float *out_c2) {
    float sum_c1 = 0.0F;
    float sum_c2 = 0.0F;
    int32_t base0 = lca16[0];
    for (size_t idx = 0U; idx < (size_t)SONY_REL_PTS; ++idx) {
        float diff = (float)((int32_t)lca16[idx + 1U] - base0);
        sum_c1 += LCA_PINV[0][idx] * diff;
        sum_c2 += LCA_PINV[1][idx] * diff;
    }
    *out_c1 = sum_c1;
    *out_c2 = sum_c2;
}

static bool bytes_any_nonzero(const uint8_t *buf, size_t len) {
    for (size_t idx = 0U; idx < len; ++idx) {
        if (buf[idx] != 0U) {
            return true;
        }
    }
    return false;
}

static void update_live_radial_tables(const adapter_ctx_t *ctx, uint8_t *buf_a) {
    static const uint8_t LIVE_A_NODES[LIVE_NODE_COUNT] = {0x00U, 0x0CU, 0x18U, 0x4CU};
    static const float STOPDOWN_FACTORS[LIVE_NODE_COUNT] = {1.00F, 0.65F, 0.42F, 0.25F};

    buf_a[1] = CMD24_A_FLAG_QUARTIC_3;
    memset(&buf_a[CMD24_A_NODE_START], 0xFF, Z_CMD24_A_NODE_SLOTS);
    memcpy(&buf_a[CMD24_A_NODE_START], LIVE_A_NODES, sizeof(LIVE_A_NODES));

    uint8_t vig9[SONY_PACKED_BYTES];
    memcpy(vig9, ctx->id05.vig_core6, E_ID05_CURVE_CORE_BYTES);
    memcpy(&vig9[E_ID05_CURVE_CORE_BYTES], ctx->id05.vig_tail3, E_ID05_CURVE_TAIL_BYTES);
    int16_t vig16[SONY_CURVE_PTS];
    optics_decode_sony_9b_16pt(vig9, SONY_SEL_VIG_2, vig16);
    float vig_c[SONY_VIG_GROUPS];
    fit_live_vignette_quartic(vig16, vig_c);

    for (size_t n_idx = 0U; n_idx < (size_t)LIVE_NODE_COUNT; ++n_idx) {
        size_t node_base = CMD24_A_COEFF_START + (n_idx * CMD24_A_FLAG3_FLOATS * sizeof(float));
        float scale = STOPDOWN_FACTORS[n_idx];
        for (size_t gi = 0U; gi < (size_t)SONY_VIG_GROUPS; ++gi) {
            write_f32_le(&buf_a[node_base + (gi * CMD24_A_GRP_STRIDE * sizeof(float))], vig_c[gi] * scale);
        }
    }

    memset(&buf_a[Z_CMD24_DISTORTION_OFFSET], 0,
           (size_t)Z_CMD24_DISTORTION_FLOATS * sizeof(float));
    if (ctx->is_fisheye != 0U) {
        buf_a[Z_CMD24_FORMAT_OFFSET] = CMD24_DIST_FORMAT_NONE;
        return;
    }

    bool have_sel21 = (bool)(((uint32_t)ctx->id05.optical_curve_valid & CURVE_VALID_DIST_BIT) != 0UL);
    bool have_legacy_dist = (bool)(bytes_any_nonzero(ctx->id05.dist_core6, E_ID05_CURVE_CORE_BYTES) ||
                                   bytes_any_nonzero(ctx->id05.dist_tail3, E_ID05_CURVE_TAIL_BYTES));
    if (!have_sel21 && !have_legacy_dist) {
        buf_a[Z_CMD24_FORMAT_OFFSET] = CMD24_DIST_FORMAT_NONE;
        return;
    }

    int16_t dist16[SONY_CURVE_PTS];
    if (have_sel21) {
        optics_decode_sony_9b_16pt(ctx->id05.optical_curve9[0], SONY_SEL_DIST_21, dist16);
    } else {
        uint8_t leg9[SONY_PACKED_BYTES];
        memcpy(leg9, ctx->id05.dist_core6, E_ID05_CURVE_CORE_BYTES);
        leg9[0] = (uint8_t)((uint32_t)leg9[0] & STRIP_BIT7_MASK);
        memcpy(&leg9[E_ID05_CURVE_CORE_BYTES], ctx->id05.dist_tail3, E_ID05_CURVE_TAIL_BYTES);
        optics_decode_sony_9b_16pt(leg9, SONY_SEL_DIST_LEG_4, dist16);
    }
    float abc_pct[SONY_LCA_GROUPS];
    fit_live_distortion_abc_pct(dist16, abc_pct);
    write_f32_le(&buf_a[Z_CMD24_DISTORTION_OFFSET], abc_pct[0]);
    write_f32_le(&buf_a[(size_t)Z_CMD24_DISTORTION_OFFSET + (DIST_RADIAL_STRIDE * sizeof(float))],
                 abc_pct[1]);
    write_f32_le(&buf_a[(size_t)Z_CMD24_DISTORTION_OFFSET + (2U * DIST_RADIAL_STRIDE * sizeof(float))],
                 abc_pct[2]);
    buf_a[Z_CMD24_FORMAT_OFFSET] = has_nonzero_distortion_54f(buf_a)
                                       ? CMD24_DIST_FORMAT_3
                                       : CMD24_DIST_FORMAT_NONE;
}

static bool update_live_lca_table(const adapter_ctx_t *ctx, uint8_t *buf_b) {
    enum {
        LEGACY_LCA_GRP_B = 12,
        LEGACY_LCA_GRP_C = 13,
    };
    static const uint8_t LEGACY_LCA_TYPE_RED = 5U;
    static const uint8_t LEGACY_LCA_TYPE_BLU = 6U;
    bool typed_lca_present =
        (bool)(((uint32_t)ctx->id05.optical_curve_valid & CURVE_VALID_LCA_MASK) == CURVE_VALID_LCA_MASK);
    bool legacy_lca_present = (bool)((!typed_lca_present) &&
                                     (ctx->id05.grp_present[ID05_LCA_GRP_IDX] != 0U) &&
                                     (ctx->id05.grp_present[LEGACY_LCA_GRP_B] != 0U) &&
                                     (ctx->id05.grp_present[LEGACY_LCA_GRP_C] != 0U) &&
                                     (ctx->id05.lca_types[0] == LEGACY_LCA_TYPE_RED) &&
                                     (ctx->id05.lca_types[1] == LEGACY_LCA_TYPE_BLU));
    if ((!typed_lca_present) && (!legacy_lca_present)) {
        return false;
    }
    int16_t red16[SONY_CURVE_PTS];
    int16_t blu16[SONY_CURVE_PTS];
    if (typed_lca_present) {
        optics_decode_sony_9b_16pt(ctx->id05.optical_curve9[1], SONY_SEL_LCA_RED_22, red16);
        optics_decode_sony_9b_16pt(ctx->id05.optical_curve9[2], SONY_SEL_LCA_BLU_23, blu16);
    } else {
        uint8_t ch1_9[SONY_PACKED_BYTES];
        uint8_t ch2_9[SONY_PACKED_BYTES];
        memcpy(ch1_9, ctx->id05.lca_ch1_core6, E_ID05_CURVE_CORE_BYTES);
        memcpy(&ch1_9[E_ID05_CURVE_CORE_BYTES], ctx->id05.lca_ch1_tail3, E_ID05_CURVE_TAIL_BYTES);
        memcpy(ch2_9, ctx->id05.lca_ch2_core6, E_ID05_CURVE_CORE_BYTES);
        memcpy(&ch2_9[E_ID05_CURVE_CORE_BYTES], ctx->id05.lca_ch2_tail3, E_ID05_CURVE_TAIL_BYTES);
        optics_decode_sony_9b_16pt(ch1_9, ctx->id05.lca_types[0], red16);
        optics_decode_sony_9b_16pt(ch2_9, ctx->id05.lca_types[1], blu16);
    }
    lca_coeff_quad_t lca_quad = {0.0F, 0.0F, 0.0F, 0.0F};
    fit_live_lca_c1_c2(red16, &lca_quad.r_c1, &lca_quad.r_c2);
    fit_live_lca_c1_c2(blu16, &lca_quad.b_c1, &lca_quad.b_c2);
    for (size_t idx = 0U; idx < (size_t)CMD24_B_NODE_SLOTS; ++idx) {
        write_cmd24_b_node_lca(buf_b, idx, &lca_quad);
    }
    return true;
}

static float compute_linear_zoom_frac(const adapter_ctx_t *ctx) {
    if (ctx->f_max_mm <= ctx->f_min_mm) {
        return 0.0F;
    }
    float f_cur = (float)ctx->f_cur_mm;
    if ((ctx->id05.grp_present[ID05_FOCAL_GRP_IDX] != 0U) && (ctx->id05.f_nom_0p1mm > 0U)) {
        f_cur = (float)ctx->id05.f_nom_0p1mm * DECIMM_TO_MM_F;
    }
    float span = (float)(ctx->f_max_mm - ctx->f_min_mm);
    float frac = (f_cur - (float)ctx->f_min_mm) / span;
    if (frac < 0.0F) {
        return 0.0F;
    }
    if (frac > 1.0F) {
        return 1.0F;
    }
    return frac;
}

static bool live_prime_optics_complete(const adapter_ctx_t *ctx, bool radial_done, bool lca_updated) {
    bool is_15grp_frame = (bool)(ctx->id05.grp_present[ID05_TAIL_GRP_IDX] == 0U);
    bool no_lca_15grp = (bool)(is_15grp_frame &&
                               ((ctx->id05.grp_present[ID05_LCA_GRP_IDX] == 0U) ||
                                ((ctx->id05.lca_types[0] == 0U) && (ctx->id05.lca_types[1] == 0U))));
    bool typed_ready = (bool)((((uint32_t)ctx->id05.optical_curve_valid & CURVE_VALID_ALL_MASK) == CURVE_VALID_ALL_MASK) ||
                              ((ctx->id05.curve_frames_seen >= ID05_MIN_CURVE_FRAMES) &&
                               (ctx->id05.optical_curve_valid != 0U)));
    return (bool)(typed_ready || (lca_updated && radial_done) || no_lca_15grp);
}

void optics_update_from_live_id05(adapter_ctx_t *ctx, optics_workspace_t *optics) {
    if ((ctx->optics_disconnected != 0U) || (ctx->id05.optics_skip != 0U) ||
        (ctx->f_max_mm > ctx->f_min_mm)) {
        ctx->id05.optics_curve_dirty = 0U;
        if (ctx->f_max_mm > ctx->f_min_mm) {
            ctx->optics_ready = 1U;
            optics_disconnect_post_init(ctx);
        }
        return;
    }
    if ((ctx->id05.aberration_valid == 0U) || (ctx->id05.optics_curve_dirty == 0U) ||
        (ctx->e_lens_id == SIGMA_150600_REV2) || (ctx->e_lens_id == SIGMA_150600_REV1)) {
        return;
    }
    const lens_db_entry_t *entry = lens_db_lookup_cached(ctx);
    if ((entry != NULL) && (entry->tamron_z_idx >= 0)) {
        return;
    }
    uint8_t next_bank = (uint8_t)((uint32_t)optics->active_optics_bank ^ 1UL);
    uint8_t *buf_a = optics->cmd24_sub00[next_bank];
    uint8_t *buf_b = optics->cmd24_sub10[next_bank];
    memcpy(buf_a, optics->cmd24_sub00[optics->active_optics_bank], Z_CMD24_A_BYTES);
    memcpy(buf_b, optics->cmd24_sub10[optics->active_optics_bank], Z_CMD24_B_BYTES);

    bool have_sel21 = (bool)(((uint32_t)ctx->id05.optical_curve_valid & CURVE_VALID_DIST_BIT) != 0UL);
    bool legacy_dist_present = (bool)(bytes_any_nonzero(ctx->id05.dist_core6, E_ID05_CURVE_CORE_BYTES) ||
                                      bytes_any_nonzero(ctx->id05.dist_tail3, E_ID05_CURVE_TAIL_BYTES));
    bool radial_locked = (bool)((((uint32_t)ctx->cmd24_read_mask & CMD24_RADIAL_LOCKED_BIT) != 0UL) &&
                                ((!have_sel21) || (((uint32_t)ctx->cmd24_read_mask & CMD24_RADIAL_SEL21_BIT) != 0UL)));
    if (!radial_locked) {
        update_live_radial_tables(ctx, buf_a);
        if (have_sel21 || legacy_dist_present) {
            ctx->cmd24_read_mask = (uint8_t)((uint32_t)ctx->cmd24_read_mask | CMD24_RADIAL_LOCKED_BIT);
            if (have_sel21) {
                ctx->cmd24_read_mask = (uint8_t)((uint32_t)ctx->cmd24_read_mask | CMD24_RADIAL_SEL21_BIT);
            }
        }
    }

    bool lca_updated = update_live_lca_table(ctx, buf_b);
    bool radial_done = (bool)((((uint32_t)ctx->cmd24_read_mask & CMD24_RADIAL_LOCKED_BIT) != 0UL) ||
                              legacy_dist_present || have_sel21);
    if (live_prime_optics_complete(ctx, radial_done, lca_updated)) {
        ctx->optics_ready = 1U;
    }

    boryoza_seal_z_frame(buf_a, Z_CMD24_A_BYTES);
    boryoza_seal_z_frame(buf_b, Z_CMD24_B_BYTES);
    optics->active_optics_bank = next_bank;
    ctx->id05.optics_curve_dirty = 0U;
    if ((ctx->optics_ready != 0U) && (radial_done || lca_updated || (ctx->id05.optical_curve_valid != 0U))) {
        optics_disconnect_post_init(ctx);
    }
}

void optics_ingest_lf_page(adapter_ctx_t *ctx, optics_workspace_t *optics,
                           uint16_t page_idx, const uint8_t *page_data, size_t page_len) {
    (void)ctx;
    if ((page_idx == 0U) || (page_data == NULL) || (page_len == 0U)) {
        return;
    }
    size_t logical_off = (((size_t)page_idx - 1U) * LF_PAGE_PAYLOAD_BYTES) %
                         (size_t)E_LF_WORKBUF_BYTES;
    size_t room = (size_t)E_LF_WORKBUF_BYTES - logical_off;
    size_t copy_n = (page_len <= room) ? page_len : room;
    memcpy(&optics->lf_work_buf[logical_off], page_data, copy_n);
}

float optics_eval_sony_ds13_focal_mm(const adapter_ctx_t *ctx, const optics_workspace_t *optics,
                                     float nominal_f_mm) {
    if ((optics->sony_ds13_valid == 0U) || (optics->sony_ds13_len < DS13_MIN_BYTES)) {
        return nominal_f_mm;
    }

    uint32_t rows = (optics->sony_ds13_len >= DS13_STD_GRID_BYTES) ? DS13_ROWS_ZOOM : 1UL;
    float z_frac = compute_linear_zoom_frac(ctx);
    float d_frac = (float)ctx->ap.z_dnorm_0x31 / NORM_COORD_MAX_F;
    if (d_frac < 0.0F) {
        d_frac = 0.0F;
    } else if (d_frac > 1.0F) {
        d_frac = 1.0F;
    }

    float r_pos = z_frac * (float)(rows - 1UL);
    uint32_t row0 = (uint32_t)r_pos;
    uint32_t row1 = ((row0 + 1UL) < rows) ? (row0 + 1UL) : row0;
    float row_t = r_pos - (float)row0;

    float c_pos = d_frac * (float)(DS13_COLS_FOCUS - 1UL);
    uint32_t col0 = (uint32_t)c_pos;
    uint32_t col1 = ((col0 + 1UL) < DS13_COLS_FOCUS) ? (col0 + 1UL) : col0;
    float col_t = c_pos - (float)col0;

    const uint8_t *raw = optics->sony_ds13_buf;
    size_t idx00 = (((size_t)row0 * (size_t)DS13_COLS_FOCUS) + (size_t)col0) * 2U;
    size_t idx01 = (((size_t)row0 * (size_t)DS13_COLS_FOCUS) + (size_t)col1) * 2U;
    size_t idx10 = (((size_t)row1 * (size_t)DS13_COLS_FOCUS) + (size_t)col0) * 2U;
    size_t idx11 = (((size_t)row1 * (size_t)DS13_COLS_FOCUS) + (size_t)col1) * 2U;

    uint16_t val00 = (uint16_t)((uint32_t)raw[idx00] | ((uint32_t)raw[idx00 + 1U] << SHIFT_8));
    uint16_t val01 = (uint16_t)((uint32_t)raw[idx01] | ((uint32_t)raw[idx01 + 1U] << SHIFT_8));
    uint16_t val10 = (uint16_t)((uint32_t)raw[idx10] | ((uint32_t)raw[idx10 + 1U] << SHIFT_8));
    uint16_t val11 = (uint16_t)((uint32_t)raw[idx11] | ((uint32_t)raw[idx11 + 1U] << SHIFT_8));

    float top = ((float)val00 * (1.0F - col_t)) + ((float)val01 * col_t);
    float bot = ((float)val10 * (1.0F - col_t)) + ((float)val11 * col_t);
    float q12_scale = ((top * (1.0F - row_t)) + (bot * row_t)) / Q12_SCALE_F;
    if ((q12_scale < DS13_MIN_SCALE) || (q12_scale > DS13_MAX_SCALE)) {
        return nominal_f_mm;
    }
    return nominal_f_mm * q12_scale;
}

uint16_t optics_compute_a3_q14(const adapter_ctx_t *ctx, const optics_workspace_t *optics) {
    (void)optics;
    if (ctx->dist_report_invalid != 0U) {
        return 0U;
    }
    if ((ctx->id05.grp_present[4] != 0U) &&
        (ctx->id05.mag_code != MAG_INVALID_CODE) && (ctx->id05.mag_code != 0U)) {
        float exp_val = MAG_CODE_EXP_BASE - ((float)ctx->id05.mag_code / MAG_CODE_DIV);
        float beta = boryoza_exp2f(exp_val);
        int32_t q14 = (int32_t)((Q14_SCALE_F * beta) + HALF_ROUND_F);
        return (uint16_t)clamp_s32(q14, 0, A3_MAX_Q14);
    }

    const lens_db_entry_t *entry = lens_db_lookup(ctx->e_lens_id);
    const tamron_z_optics_profile_t *tam_prof =
        (entry != NULL) ? lens_db_get_tamron_optics(entry->tamron_z_idx) : NULL;
    if (tam_prof != NULL) {
        float z_frac = compute_linear_zoom_frac(ctx);
        float d_frac = (float)ctx->ap.z_dnorm_0x31 / NORM_COORD_MAX_F;
        float mfd_q14 = (float)tam_prof->a3_wide_q14 +
                        (z_frac * (float)((int32_t)tam_prof->a3_tele_q14 - (int32_t)tam_prof->a3_wide_q14));
        int32_t q14 = (int32_t)((mfd_q14 * d_frac) + HALF_ROUND_F);
        return (uint16_t)clamp_s32(q14, 0, A3_MAX_Q14);
    }

    if (ctx->af.cur_cz <= 0) {
        return 0U;
    }
    float focal_m = (float)ctx->f_cur_mm * MM_TO_M_F;
    float denom = ctx->af.cur_dist_m - (TWO_F * focal_m);
    if (denom < focal_m) {
        denom = focal_m;
    }
    float beta = focal_m / denom;
    int32_t cap = (ctx->is_macro != 0U) ? A3_MAX_Q14 : A3_TIER3_CAP_Q14;
    int32_t q14 = (int32_t)((Q14_SCALE_F * beta) + HALF_ROUND_F);
    return (uint16_t)clamp_s32(q14, 0, cap);
}
