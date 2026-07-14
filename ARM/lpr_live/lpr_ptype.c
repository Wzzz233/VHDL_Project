// SPDX-License-Identifier: GPL-2.0
/* Plate type classifier RKNN: BGPEY route override helper. */

#include "lpr_ptype.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char *lpr_ptype_class_str(int cls)
{
    static const char *const names[LPR_PTYPE_CLASS_COUNT] = {
        "blue", "green", "yellow", "white", "black"
    };
    if (cls >= 0 && cls < LPR_PTYPE_CLASS_COUNT)
        return names[cls];
    return "unknown";
}

static void resize_bilinear(const uint8_t *src, int sw, int sh, uint8_t *dst, int dw, int dh)
{
    int x, y;
    if (sw <= 0 || sh <= 0 || dw <= 0 || dh <= 0)
        return;
    if (sw == 1 || sh == 1) {
        for (y = 0; y < dh; y++) {
            int sy = y * sh / dh;
            for (x = 0; x < dw; x++) {
                int sx = x * sw / dw;
                memcpy(dst + ((size_t)y * dw + x) * 3U,
                       src + ((size_t)sy * sw + sx) * 3U, 3);
            }
        }
        return;
    }
    for (y = 0; y < dh; y++) {
        float fy = ((float)y + 0.5f) * (float)sh / (float)dh - 0.5f;
        int y0 = (int)floorf(fy);
        int y1;
        float wy;
        if (y0 < 0) y0 = 0;
        if (y0 > sh - 1) y0 = sh - 1;
        y1 = y0 + 1;
        if (y1 > sh - 1) y1 = sh - 1;
        wy = fy - (float)y0;
        if (wy < 0.0f) wy = 0.0f;
        if (wy > 1.0f) wy = 1.0f;
        for (x = 0; x < dw; x++) {
            float fx = ((float)x + 0.5f) * (float)sw / (float)dw - 0.5f;
            int x0 = (int)floorf(fx);
            int x1;
            float wx;
            int c;
            if (x0 < 0) x0 = 0;
            if (x0 > sw - 1) x0 = sw - 1;
            x1 = x0 + 1;
            if (x1 > sw - 1) x1 = sw - 1;
            wx = fx - (float)x0;
            if (wx < 0.0f) wx = 0.0f;
            if (wx > 1.0f) wx = 1.0f;
            for (c = 0; c < 3; c++) {
                float p00 = src[((size_t)y0 * sw + x0) * 3U + c];
                float p01 = src[((size_t)y0 * sw + x1) * 3U + c];
                float p10 = src[((size_t)y1 * sw + x0) * 3U + c];
                float p11 = src[((size_t)y1 * sw + x1) * 3U + c];
                float v0 = p00 * (1.0f - wx) + p01 * wx;
                float v1 = p10 * (1.0f - wx) + p11 * wx;
                int iv = (int)(v0 * (1.0f - wy) + v1 * wy + 0.5f);
                if (iv < 0) iv = 0;
                if (iv > 255) iv = 255;
                dst[((size_t)y * dw + x) * 3U + c] = (uint8_t)iv;
            }
        }
    }
}

static int tensor_elem_count(const rknn_tensor_attr *a)
{
    uint32_t n = 1;
    if (!a || a->n_dims == 0)
        return 0;
    for (uint32_t i = 0; i < a->n_dims; i++) {
        if (a->dims[i] == 0)
            return 0;
        n *= a->dims[i];
    }
    return (int)n;
}

int lpr_ptype_run(struct rknn_model *m,
                  const uint8_t *plate_rgb, int plate_w, int plate_h,
                  int *cls_out, float *conf_out, double *timing_ms)
{
    uint8_t *input = NULL;
    rknn_input in;
    rknn_output out;
    int ret;
    int elem_count;
    int best = -1;
    float best_logit = -INFINITY;
    float max_logit = -INFINITY;
    float sum_exp = 0.0f;
    int64_t t0, t1;

    if (cls_out) *cls_out = LPR_PTYPE_UNKNOWN;
    if (conf_out) *conf_out = 0.0f;
    if (timing_ms) *timing_ms = 0.0;
    if (!m || !m->ctx || !plate_rgb || !cls_out || !conf_out)
        return -1;
    if (m->in_w == 0 || m->in_h == 0 || m->in_c != 3 || plate_w <= 0 || plate_h <= 0)
        return -1;

    t0 = lpr_mono_us();
    input = malloc((size_t)m->in_w * (size_t)m->in_h * 3U);
    if (!input)
        return -1;
    resize_bilinear(plate_rgb, plate_w, plate_h, input, (int)m->in_w, (int)m->in_h);

    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf = input;
    in.size = m->in_w * m->in_h * 3U;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    ret = rknn_inputs_set(m->ctx, 1, &in);
    if (ret < 0)
        goto out_free;
    ret = rknn_run(m->ctx, NULL);
    if (ret < 0)
        goto out_free;

    memset(&out, 0, sizeof(out));
    out.want_float = 1;
    ret = rknn_outputs_get(m->ctx, 1, &out, NULL);
    if (ret < 0)
        goto out_free;

    elem_count = tensor_elem_count(&m->output_attrs[0]);
    if (elem_count < LPR_PTYPE_CLASS_COUNT) {
        ret = -1;
        goto out_release;
    }
    for (int i = 0; i < LPR_PTYPE_CLASS_COUNT; i++) {
        float v = ((const float *)out.buf)[i];
        if (v > best_logit) {
            best_logit = v;
            best = i;
        }
        if (v > max_logit)
            max_logit = v;
    }
    if (best >= 0) {
        for (int i = 0; i < LPR_PTYPE_CLASS_COUNT; i++)
            sum_exp += expf(((const float *)out.buf)[i] - max_logit);
        *cls_out = best;
        *conf_out = (sum_exp > 0.0f) ? expf(best_logit - max_logit) / sum_exp : 0.0f;
        ret = 0;
    } else {
        ret = -1;
    }

out_release:
    rknn_outputs_release(m->ctx, 1, &out);
out_free:
    t1 = lpr_mono_us();
    if (timing_ms)
        *timing_ms = (double)(t1 - t0) / 1000.0;
    free(input);
    return ret;
}
