/*
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "config.h"
#include "libavutil/attributes.h"
#include "libavutil/cpu.h"
#include "libavutil/x86/cpu.h"
#include "libavfilter/vca_dct.h"

void ff_dct8_8bit_avx2(const int16_t *src, int16_t *dst, intptr_t srcStride);
void ff_dct8_10bit_avx2(const int16_t *src, int16_t *dst, intptr_t srcStride);
void ff_dct8_12bit_avx2(const int16_t *src, int16_t *dst, intptr_t srcStride);

void ff_dct16_8bit_avx2(const int16_t *src, int16_t *dst, intptr_t srcStride);
void ff_dct16_10bit_avx2(const int16_t *src, int16_t *dst, intptr_t srcStride);
void ff_dct16_12bit_avx2(const int16_t *src, int16_t *dst, intptr_t srcStride);

void ff_dct32_8bit_avx2(const int16_t *src, int16_t *dst, intptr_t srcStride);
void ff_dct32_10bit_avx2(const int16_t *src, int16_t *dst, intptr_t srcStride);
void ff_dct32_12bit_avx2(const int16_t *src, int16_t *dst, intptr_t srcStride);

 
#if HAVE_X86ASM

static void ff_dct8_avx2(const int16_t *src, int16_t *dst, int bit_depth) {
    switch (bit_depth) {
        case 8:
            ff_dct8_8bit_avx2(src, dst, 8);
            break;
        case 10:
            ff_dct8_10bit_avx2(src, dst, 8);
            break;
        case 12:
            ff_dct8_12bit_avx2(src, dst, 8);
            break;
        default:
            ff_vca_dct8_c(src, dst, bit_depth);
            break;
    }
}

static void ff_dct16_avx2(const int16_t *src, int16_t *dst, int bit_depth) {
    switch (bit_depth) {
        case 8:
            ff_dct16_8bit_avx2(src, dst, 16);
            break;
        case 10:
            ff_dct16_10bit_avx2(src, dst, 16);
            break;
        case 12:
            ff_dct16_12bit_avx2(src, dst, 16);
            break;
        default:
            ff_vca_dct16_c(src, dst, bit_depth);
            break;
    }        
}

static void ff_dct32_avx2(const int16_t *src, int16_t *dst, int bit_depth) {
    switch (bit_depth) {
        case 8:
            ff_dct32_8bit_avx2(src, dst, 32);
            break;
        case 10:
            ff_dct32_10bit_avx2(src, dst, 32);
            break;
        case 12:
            ff_dct32_12bit_avx2(src, dst, 32);
            break;
        default:
            ff_vca_dct32_c(src, dst, bit_depth);
            break;
    }
}

static void ff_lowpass_dct16_avx2(const int16_t *src, int16_t *dst, int bit_depth) {
    ALIGN_VAR_32(int16_t, coef[8 * 8]);
    ALIGN_VAR_32(int16_t, avg_block[8 * 8]);

    int32_t totalSum = 0;
    int16_t sum = 0;
    for (int i = 0; i < 8; i++)
        for (int j =0; j < 8; j++)
        {
            sum = src[2*i*16 + 2*j] + src[2*i*16 + 2*j + 1]
                    + src[(2*i+1)*16 + 2*j] + src[(2*i+1)*16 + 2*j + 1];
            avg_block[i*8 + j] = sum >> 2;

            totalSum += sum;
        }

    switch (bit_depth) {
        case 8:
            ff_dct8_8bit_avx2(avg_block, coef, 8);
            break;
        case 10:
            ff_dct8_10bit_avx2(avg_block, coef, 8);
            break;
        case 12:
            ff_dct8_12bit_avx2(avg_block, coef, 8);
            break;
        default:
            ff_vca_dct8_c(avg_block, coef, bit_depth);
            break;
    }   
    
    memset(dst, 0, 256 * sizeof(int16_t));
    for (int i = 0; i < 8; i++)
    {
        memcpy(&dst[i * 16], &coef[i * 8], 8 * sizeof(int16_t));
    }
    dst[0] = (int16_t)(totalSum >> 1);

}

static void ff_lowpass_dct32_avx2(const int16_t *src, int16_t *dst, int bit_depth) {
    ALIGN_VAR_32(int16_t, coef[16 * 16]);
    ALIGN_VAR_32(int16_t, avg_block[16 * 16]);
   
    int32_t totalSum = 0;
    int16_t sum = 0;
    for (int i = 0; i < 16; i++)
        for (int j =0; j < 16; j++)
        {
            sum = src[2*i*32 + 2*j] + src[2*i*32 + 2*j + 1]
                    + src[(2*i+1)*32 + 2*j] + src[(2*i+1)*32 + 2*j + 1];
            avg_block[i*16 + j] = sum >> 2;

            totalSum += sum;
        }
    
    switch (bit_depth) {
        case 8:
            ff_dct16_8bit_avx2(avg_block, coef, 16);
            break;
        case 10:
            ff_dct16_10bit_avx2(avg_block, coef, 16);
            break;
        case 12:
            ff_dct16_12bit_avx2(avg_block, coef, 16);
            break;
        default:
            ff_vca_dct16_c(avg_block, coef, bit_depth);
            break;
    }  

    memset(dst, 0, 1024 * sizeof(int16_t));
    for (int i = 0; i < 16; i++)
    {
        memcpy(&dst[i * 32], &coef[i * 16], 16 * sizeof(int16_t));
    }
    dst[0] = (int16_t)(totalSum >> 3);
}
#endif /* HAVE_X86ASM */

av_cold int ff_vca_dct_init_x86(VCAContext *v) {
#if HAVE_X86ASM
    int cpu_flags = av_get_cpu_flags();

    if (EXTERNAL_AVX2(cpu_flags)) {
        if(v->enable_lowpass) {
            switch (v->blocksize) {
                case 32:
                    v->perform_dct = ff_lowpass_dct32_avx2;
                    return 0;
                case 16:
                    v->perform_dct = ff_lowpass_dct16_avx2;
                    return 0;
                case 8:
                    v->perform_dct = ff_vca_lowpass_dct8_c;
                    return 0;
                default:
                    return AVERROR(AVERROR_INVALIDDATA);
            }
        }
        else {            
            switch (v->blocksize) {
                case 32:
                    v->perform_dct = ff_dct32_avx2;
                    return 0;
                case 16:
                    v->perform_dct = ff_dct16_avx2;
                    return 0;
                case 8:
                    v->perform_dct = ff_dct8_avx2;
                    return 0;
                default:
                    return AVERROR(AVERROR_INVALIDDATA);
            }
        }
    }
#endif /* HAVE_X86ASM */
return 0;
}
