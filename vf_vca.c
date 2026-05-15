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

/**
 * @file
 * Calculate frame scores using Video Complexity Analyzer (VCA)
 */


#include "libavutil/timestamp.h"
#include "libavutil/mathematics.h"

#include "libavutil/avassert.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"


#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "video.h"

#include "vca_dct.h"

typedef struct ThreadData {
    void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth);
    void (*calc_vca_slice)(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, 
                        int enable_lowpass, int slice_start, int slice_end, ResultSums *partial_sum,
                        void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth));
    int stride;
    int blocksize;

    int enable_lowpass;

    uint8_t *src;

    VCAPlaneInfo *plane;
    VCAResults *result;
    
    ResultSums **partial_sums;
} ThreadData;

#define OFFSET(x) offsetof(VCAContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption vca_options[] = {
    // Analysis config                                      
    { "blocksize", "Set size of block", OFFSET(blocksize), AV_OPT_TYPE_INT, {.i64=32}, 8, 32, FLAGS },
    { "n", "Set the frames batch size, -1 to process all", OFFSET(n_frames), AV_OPT_TYPE_INT, {.i64=-1}, -1, INT_MAX, FLAGS },
    // Performance
    { "lowpass", "Enable low-pass DCT", OFFSET(enable_lowpass), AV_OPT_TYPE_BOOL, { .i64=1 }, 0, 1, FLAGS },
    { "simd", "Enable hardware acceralation with SIMD", OFFSET(enable_simd), AV_OPT_TYPE_BOOL, { .i64=1 }, 0, 1, FLAGS },
    { "brightness", "Enable brightness infomation", OFFSET(enable_brightness), AV_OPT_TYPE_BOOL, { .i64=0 }, 0, 1, FLAGS },
    { "chroma", "Enable analysis of chroma channels", OFFSET(enable_chroma), AV_OPT_TYPE_BOOL, { .i64=0 }, 0, 1, FLAGS },
    // Output
    { "file", "Set file where to print analysis information", OFFSET(file_str), AV_OPT_TYPE_STRING, {.str=NULL}, 0, 0, FLAGS },
    { "yuview", "Produce a detailed blockwise output for YUView", OFFSET(yuview), AV_OPT_TYPE_BOOL, { .i64=0 }, 0, 1, FLAGS },
    { NULL }
};

static const double E_norm_factor = 90;
static const double h_norm_factor = 18;

AVFILTER_DEFINE_CLASS(vca);

static const enum AVPixelFormat pxl_fmts[] = {
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P,
    AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10,
    AV_PIX_FMT_NONE
};

#define WRITE_YUVIEW_BRIGHTNESS_1_1 \
        result->brightness[block_i] = (uint32_t)sqrt(out_buffer[0]); 

#define WRITE_YUVIEW_BRIGHTNESS_1_0
#define WRITE_YUVIEW_BRIGHTNESS_0_1
#define WRITE_YUVIEW_BRIGHTNESS_0_0

#define WRITE_YUVIEW_BRIGHTNESS(IS_BRIGHTNESS, IS_YUVIEW) WRITE_YUVIEW_BRIGHTNESS_##IS_BRIGHTNESS##_##IS_YUVIEW

#define WRITE_YUVIEW_ENERGY_DIF_1_1 \
        result->energy_dif[block_i] = abs((int)result->energy[block_i] - (int)result->energy_prev[block_i]);

#define WRITE_YUVIEW_ENERGY_DIF_1_0
#define WRITE_YUVIEW_ENERGY_DIF_0_1
#define WRITE_YUVIEW_ENERGY_DIF_0_0

#define WRITE_YUVIEW_ENERGY_DIF(IS_NOT_FIRST, IS_YUVIEW) WRITE_YUVIEW_ENERGY_DIF_##IS_NOT_FIRST##_##IS_YUVIEW

#define ENERGY_DIF_1 \
        partial_sum->h += abs((int)result->energy[block_i] - (int)result->energy_prev[block_i]); 

#define ENERGY_DIF_0 

#define ENERGY_DIF(IS_NOT_FIRST) ENERGY_DIF_##IS_NOT_FIRST

#define BRIGHTNESS_1 \
        partial_sum->L += (uint32_t)sqrt(out_buffer[0]);

#define BRIGHTNESS_0 

#define BRIGHTNESS(IS_BRIGHTNESS) BRIGHTNESS_##IS_BRIGHTNESS

#define DEFINE_CALC_ENERGY_SLICE(BLOCKSIZE, IS_NOT_FIRST, IS_YUVIEW, IS_BRIGHNTESS)                 \
static void calc_vca_##BLOCKSIZE##_isnf##IS_NOT_FIRST##_brig##IS_BRIGHNTESS##_yuview##IS_YUVIEW##_slice(\
    int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result,                              \
    int enable_lowpass, int slice_start, int slice_end, ResultSums *partial_sum,                      \
    void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth)) {                       \
    int block_i = (slice_start / BLOCKSIZE) * plane->w_blocks;                                      \
    ALIGN_VAR_32(int16_t, block_buffer[BLOCKSIZE * BLOCKSIZE]);                                     \
    ALIGN_VAR_32(int16_t, out_buffer[BLOCKSIZE * BLOCKSIZE]);                                       \
    const unsigned bit_depth = plane->bit_depth;                                                    \
    for (unsigned blockY = slice_start; blockY < slice_end; blockY += BLOCKSIZE) {                  \
        int padding_b = FFMAX(((int)(blockY + BLOCKSIZE) - (int)(plane->h_pxls_src)), 0);           \
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += BLOCKSIZE) {                    \
            int offset = blockX * plane->pxl_depth + (blockY * stride);                             \
            int padding_r = FFMAX((int)(blockX + BLOCKSIZE) - (int)(plane->w_pxls_src), 0);         \
            /* Copy values to block buffer */                                                       \
            ff_copy_vals_buffer(plane->pxl_depth, offset, BLOCKSIZE, src, stride,                   \
                             block_buffer, padding_r, padding_b);                                   \
            perform_dct(block_buffer, out_buffer, bit_depth);                                       \
            /* Calculate energy */                                                                  \
            result->energy[block_i] = ff_calc_weighted_coeff(BLOCKSIZE, out_buffer, enable_lowpass);\
            partial_sum->E += result->energy[block_i];                                              \
            BRIGHTNESS(IS_BRIGHNTESS)                                                               \
            ENERGY_DIF(IS_NOT_FIRST)                                                                \
            WRITE_YUVIEW_BRIGHTNESS(IS_BRIGHNTESS, IS_YUVIEW)                                       \
            WRITE_YUVIEW_ENERGY_DIF(IS_NOT_FIRST, IS_YUVIEW)                                        \
            block_i++;                                                                              \
        }                                                                                           \
    }                                                                                               \
}

#define FUNCTION_LIST(X) \
    X(8,0,0,0)  X(16,0,0,0)  X(32,0,0,0) X(8,1,0,0)  X(16,1,0,0)  X(32,1,0,0) \
    X(8,0,1,0)  X(16,0,1,0)  X(32,0,1,0) X(8,1,1,0)  X(16,1,1,0)  X(32,1,1,0) \
    X(8,0,0,1)  X(16,0,0,1)  X(32,0,0,1) X(8,1,0,1)  X(16,1,0,1)  X(32,1,0,1) \
    X(8,0,1,1)  X(16,0,1,1)  X(32,0,1,1) X(8,1,1,1)  X(16,1,1,1)  X(32,1,1,1) 

FUNCTION_LIST(DEFINE_CALC_ENERGY_SLICE)

#define FN(blsz, isnf, brig, yuv) \
    calc_vca_##blsz##_isnf##isnf##_brig##brig##_yuview##yuv##_slice
    
#define ISNF(blsz, brig, yuv) \
    { FN(blsz,0,brig,yuv), FN(blsz,1,brig,yuv) }

#define YUVIEW(blsz, brig) \
    { ISNF(blsz, brig, 0), ISNF(blsz, brig, 1) }

#define BRIGHT(blsz) \
    { YUVIEW(blsz, 0), YUVIEW(blsz, 1) }

static void* calc_fn_table[3][2][2][2] = {
    BRIGHT(8),
    BRIGHT(16),
    BRIGHT(32)
};


static void print_log(AVFilterContext *ctx, int lvl, const char *msg, ...)
{
    va_list argument_list;

    va_start(argument_list, msg);
    if (msg)
        av_vlog(ctx, lvl, msg, argument_list);
    va_end(argument_list);
}

static void print_file(AVFilterContext *ctx, int lvl, const char *msg, ...)
{
    VCAContext *v = ctx->priv;
    va_list argument_list;

    va_start(argument_list, msg);
    if (msg) {
        char buf[128];
        int ret = vsnprintf(buf, sizeof(buf), msg, argument_list);
        avio_write(v->avio_context, buf, ret);
    }
    va_end(argument_list);
}

static int calc_energy_filter_slice(AVFilterContext *ctx, void *arg, int job, int nb_jobs){
    ThreadData *th = arg;

    int block_row_start = (th->plane->h_blocks * job)     / nb_jobs;
    int block_row_end   = (th->plane->h_blocks * (job+1)) / nb_jobs;

    int slice_start = block_row_start * th->blocksize;
    int slice_end   = block_row_end   * th->blocksize;

    th->calc_vca_slice(
        th->stride, th->src, th->plane, th->result,
        th->enable_lowpass, slice_start, slice_end,
        th->partial_sums[job], th->perform_dct
    );

    return 0;
}

static void perform_vca(AVFilterContext *ctx, AVFilterLink *inlink, AVFrame *in, FilterLink *inl ,
    VCAContext *v, int plane_i, double* h, uint32_t* E, uint32_t* L){
    VCAPlaneInfo* plane = v->plane[plane_i];

    int stride = in->linesize[plane_i] / plane->pxl_depth;
    int nb_threads = ff_filter_get_nb_threads(ctx);
    void* calc_vca_slice;

    ResultSums** partial_sums = av_calloc(nb_threads, sizeof(ResultSums*));
    for(int j = 0; j < nb_threads; j++)
        partial_sums[j] = av_calloc(1, sizeof(ResultSums));

    if (v->n_frames_processed == 0)
        calc_vca_slice = v->calc_vca_slice_isnf0;
    else 
        calc_vca_slice = v->calc_vca_slice_isnf1;

    ThreadData th = {
        .stride = stride,
        .blocksize = v->blocksize,
        .enable_lowpass = v->enable_lowpass,
        .src = in->data[plane_i],
        .plane = plane,
        .result =  v->result[plane_i],
        .partial_sums = partial_sums,
        .perform_dct = v->perform_dct,
        .calc_vca_slice = calc_vca_slice,
    };

    ff_filter_execute(ctx, calc_energy_filter_slice, &th, NULL, FFMIN(plane->h_blocks, nb_threads));

    for (int i = 0; i < nb_threads; i++){
        E[plane_i] += th.partial_sums[i]->E;
        L[plane_i] += th.partial_sums[i]->L;
        h[plane_i] += th.partial_sums[i]->h;
    }

    E[plane_i] /= (plane->n_blocks * E_norm_factor); 
    L[plane_i] /= (plane->n_blocks); 
    h[plane_i] /= (plane->n_blocks * h_norm_factor); 

    av_free(th.partial_sums);

    // At the end copy current energy to the previous
    memcpy(v->result[plane_i]->energy_prev ,v->result[plane_i]->energy, v->plane[plane_i]->n_blocks * sizeof(uint32_t));
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    VCAContext *v = ctx->priv;
    FilterLink *inl = ff_filter_link(inlink);
    int planes = v->enable_chroma ? 3 : 1;
 
    if (v->n_frames_processed >= v->n_frames && v->n_frames != -1)
        return ff_filter_frame(inlink->dst->outputs[0], in);

    uint32_t E[3] =  {0,0,0};
    uint32_t L[3] =  {0.0,0.0,0.0};
    double h[3]   =  {0,0,0};
    
    for(int i = 0; i < planes; i++){
        if (v->plane[i]->bit_depth != 8 
            && v->plane[i]->bit_depth != 10 
            && v->plane[i]->bit_depth != 12)                                       
            return AVERROR(AVERROR_INVALIDDATA);   
        perform_vca(ctx, inlink, in, inl, v, i, h, E, L);
    }

    v->n_frames_processed++;

    // Dump info;
    if (v->yuview) {
        int block_i = 0;
        for (unsigned y = 0; y < v->plane[0]->h_blocks; y ++) {
            for (unsigned x = 0; x < v->plane[0]->w_blocks; x ++) {
                v->print(ctx, AV_LOG_INFO, "%d;%d;%d;%d;%d;%d;%d\n", inl->frame_count_out, 
                        x * v->blocksize, y * v->blocksize, v->blocksize, v->blocksize, 
                        0, v->result[0]->energy_prev[block_i]);
                        block_i++;
            } 
        }
        block_i = 0;
        for (unsigned y = 0; y < v->plane[0]->h_blocks; y ++) {
            for (unsigned x = 0; x < v->plane[0]->w_blocks; x ++) {
                v->print(ctx, AV_LOG_INFO, "%d;%d;%d;%d;%d;%d;%.0f\n", inl->frame_count_out,
                        x * v->blocksize, y * v->blocksize, v->blocksize, v->blocksize, 
                        1, v->result[0]->energy_dif[block_i]);
                        block_i++;
            } 
        }
    } else {
        v->print(ctx, AV_LOG_INFO,
            "%4"PRId64,
            inl->frame_count_out);
        v->print(ctx, AV_LOG_INFO,
                ",%d,%f",
                E[0], h[0]);
        if(v->enable_brightness)
                v->print(ctx, AV_LOG_INFO,",%d",L[0]);
        if (v->enable_chroma) {
            v->print(ctx, AV_LOG_INFO,
                ",%d,%f",
                E[1], h[1]);
            v->print(ctx, AV_LOG_INFO,
                ",%d,%f",
                E[2], h[2]);
            if(v->enable_brightness)
                v->print(ctx, AV_LOG_INFO,",%d,%d",L[1],L[2]);
        }
    }

    v->print(ctx, AV_LOG_INFO, "\n");
    return ff_filter_frame(inlink->dst->outputs[0], in);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    VCAContext *v = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int max_pixsteps[4];
    int planes;

    v->plane[0]->w_pxls_src = inlink->w;
    v->plane[0]->h_pxls_src = inlink->h;

    if (v->enable_chroma){
        v->plane[1]->w_pxls_src = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
        v->plane[1]->h_pxls_src = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);

        v->plane[2]->w_pxls_src = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
        v->plane[2]->h_pxls_src = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);

        planes = 3;
    } else 
        planes = 1;

    for(int i = 0; i < planes; i++){
        // Inference of bit depth 
        v->plane[i]->bit_depth = desc->comp[i].depth;
        // Inference of pixel depth
        av_image_fill_max_pixsteps(max_pixsteps, NULL, desc);
        v->plane[i]->pxl_depth = max_pixsteps[i];

        v->plane[i]->w_blocks = (v->plane[i]->w_pxls_src + v->blocksize - 1) / v->blocksize;
        v->plane[i]->h_blocks = (v->plane[i]->h_pxls_src + v->blocksize - 1) / v->blocksize;

        v->plane[i]->n_blocks = v->plane[i]->w_blocks * v->plane[i]->h_blocks;    
                
        v->plane[i]->w_pxls = v->plane[i]->w_blocks * v->blocksize;
        v->plane[i]->h_pxls = v->plane[i]->h_blocks * v->blocksize;

        // Free previous buffers in case they are allocated already
        av_freep(&v->result[i]->energy_prev);
        av_freep(&v->result[i]->energy_dif);
        av_freep(&v->result[i]->energy);
        av_freep(&v->result[i]->brightness);

        v->result[i]->energy = av_malloc(v->plane[i]->n_blocks * sizeof(uint32_t));
        v->result[i]->energy_prev = av_malloc(v->plane[i]->n_blocks * sizeof(uint32_t));
        if (!v->result[i]->energy || ! v->result[i]->energy_prev)
            return AVERROR(ENOMEM);

        if(v->yuview){
            v->result[i]->energy_dif = av_malloc(v->plane[i]->n_blocks * sizeof(double));
            v->result[i]->brightness = av_malloc(v->plane[i]->n_blocks * sizeof(uint32_t));
            if (!v->result[i]->energy_dif || !v->result[i]->brightness)
                return AVERROR(ENOMEM);
        }
    }

    if (v->yuview) {
        v->print(ctx, AV_LOG_INFO, "%%;%%;Written by VCA for YUView\n");
        v->print(ctx, AV_LOG_INFO, "%%;syntax-version;v1.22\n");
        v->print(ctx, AV_LOG_INFO, "%%;%%;POC;X-position of the left top pixel in the block;Y-position of the left top pixel in the block;");
        v->print(ctx, AV_LOG_INFO, "Width of the block;Height of the block; Type-ID;Type specific value\n");
        v->print(ctx, AV_LOG_INFO, "%%;seq-specs;%s;%s;%d;%d;%d\n", "file", "layer0", v->plane[0]->w_pxls, v->plane[0]->h_pxls, 24);
        v->print(ctx, AV_LOG_INFO, "%%;type;0;BlockEnergy;range\n");
        v->print(ctx, AV_LOG_INFO, "%%;defaultRange;0;10000;heat\n");
        v->print(ctx, AV_LOG_INFO, "%%;type;1;TempEnergyDiff;range\n");
        v->print(ctx, AV_LOG_INFO, "%%;defaultRange;0;3000;heat\n");
    } else {
        v->print(ctx, AV_LOG_INFO, "POC,E,h");
        if(v->enable_brightness)
            v->print(ctx, AV_LOG_INFO, ",L");
        if (v->enable_chroma)
            v->print(ctx, AV_LOG_INFO, ",EV,LV,hV,EU,LU,hE");
        if(v->enable_brightness && v->enable_chroma)
            v->print(ctx, AV_LOG_INFO, ",LV,LU");

        v->print(ctx, AV_LOG_INFO, "\n");
    }
    
    av_log(ctx, AV_LOG_INFO, "threads: %d\n", ff_filter_get_nb_threads(ctx));

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    // User options but no input data
    VCAContext *v = ctx->priv;
    int ret;
    int planes = v->enable_chroma ? 3 : 1;

    // allocate arrays of pointers
    v->result = av_calloc(planes, sizeof(*v->result));
    v->plane  = av_calloc(planes, sizeof(*v->plane));
    if (!v->result || !v->plane)
        return AVERROR(ENOMEM);

    // allocate each plane/result struct
    for (int i = 0; i < planes; i++) {
        v->result[i] = av_mallocz(sizeof(*v->result[i]));
        v->plane[i]  = av_mallocz(sizeof(*v->plane[i]));

        if (!v->result[i] || !v->plane[i]) 
            return AVERROR(ENOMEM);
    }
    
    v->n_frames_processed = 0;

    if (v->file_str) {
        v->print = print_file;
    } else {
        v->print = print_log;
    }
        
    if (v->enable_lowpass) {
        switch (v->blocksize) {
            case 32: v->perform_dct = ff_vca_lowpass_dct32_c; break;
            case 16: v->perform_dct = ff_vca_lowpass_dct16_c; break;
            case 8: v->perform_dct = ff_vca_lowpass_dct8_c; break;
            default: 
                av_log(ctx, AV_LOG_ERROR, "Unallowed blocksize: %d\n", v->blocksize);
                return AVERROR(AVERROR_INVALIDDATA);
        }
    } else {            
        switch (v->blocksize) {
            case 32: v->perform_dct = ff_vca_dct32_c; break;
            case 16: v->perform_dct = ff_vca_dct16_c; break;
            case 8: v->perform_dct = ff_vca_dct8_c; break;
            default:
                av_log(ctx, AV_LOG_ERROR, "Unallowed blocksize: %d\n", v->blocksize);    
                return AVERROR(AVERROR_INVALIDDATA);
        }
    }

    if (v->enable_simd) {
        #if ARCH_X86 && HAVE_X86ASM
        ret = ff_vca_dct_init_x86(v);
        if (ret != 0) {
            return ret;
        }
        #endif
    }

    int b = 0;
    switch (v->blocksize) {
        case 8:  b = 0; break;
        case 16: b = 1; break;
        case 32: b = 2; break;
        default: 
            av_log(ctx, AV_LOG_ERROR, "Unallowed blocksize: %d\n", v->blocksize);    
            return AVERROR(AVERROR_INVALIDDATA);
    }

    v->calc_vca_slice_isnf0 = calc_fn_table[b][v->enable_brightness][v->yuview][0];
    v->calc_vca_slice_isnf1 = calc_fn_table[b][v->enable_brightness][v->yuview][1];

    v->avio_context = NULL;
    if (v->file_str) {
        ret = avio_open(&v->avio_context, v->file_str, AVIO_FLAG_WRITE);

        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "Could not open %s: %s\n",
                   v->file_str, av_err2str(ret));
            return ret;
        }
    }
    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    VCAContext *v = ctx->priv;
    int planes = v->enable_chroma ? 3 : 1;

    for(int plane = 0; plane < planes; plane++) {
        av_freep(&v->result[plane]->energy);
        av_freep(&v->result[plane]->energy_prev);
        av_freep(&v->result[plane]->energy_dif);
        av_freep(&v->result[plane]->brightness);
    }

    if (v->avio_context) {
        avio_closep(&v->avio_context);
    }
}

static const AVFilterPad avfilter_vf_vca_inputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
        .filter_frame     = filter_frame,
        .config_props     = config_input,
    },
};

const FFFilter ff_vf_vca = {
    .p.name        = "vca",
    .p.description = NULL_IF_CONFIG_SMALL("Perform VCA analysis."),
    .p.priv_class  = &vca_class,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS,
    .priv_size     = sizeof(VCAContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_PIXFMTS_ARRAY(pxl_fmts),
    FILTER_INPUTS(avfilter_vf_vca_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
};  