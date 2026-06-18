// SPDX-License-Identifier: GPL-2.0
/* PPLCNet RKNN OCR module. */

#include "lpr_ocr.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------------- RKNN model load/release ---------------- */

static const char *tensor_fmt_name(int fmt)
{
    return fmt == RKNN_TENSOR_NCHW ? "NCHW" : (fmt == RKNN_TENSOR_NHWC ? "NHWC" : "OTHER");
}

int lpr_model_load(struct rknn_model *m, const char *name, const char *path)
{
    void *data = NULL;
    uint32_t sz = 0;
    uint32_t i;
    memset(m, 0, sizeof(*m));
    m->name = name;
    m->path = path;
    if (lpr_load_file(path, &data, &sz) < 0)
        return -1;
    if (rknn_init(&m->ctx, data, sz, 0, NULL) < 0) {
        free(data);
        return -1;
    }
    free(data);
    if (rknn_query(m->ctx, RKNN_QUERY_IN_OUT_NUM, &m->io_num, sizeof(m->io_num)) < 0)
        return -1;
    if (m->io_num.n_output == 0 || m->io_num.n_output > 8)
        return -1;
    memset(&m->input_attr, 0, sizeof(m->input_attr));
    m->input_attr.index = 0;
    if (rknn_query(m->ctx, RKNN_QUERY_INPUT_ATTR, &m->input_attr, sizeof(m->input_attr)) < 0)
        return -1;
    if (m->input_attr.fmt == RKNN_TENSOR_NCHW) {
        m->in_c = m->input_attr.dims[1];
        m->in_h = m->input_attr.dims[2];
        m->in_w = m->input_attr.dims[3];
    } else {
        m->in_h = m->input_attr.dims[1];
        m->in_w = m->input_attr.dims[2];
        m->in_c = m->input_attr.dims[3];
    }
    for (i = 0; i < m->io_num.n_output; i++) {
        memset(&m->output_attrs[i], 0, sizeof(m->output_attrs[i]));
        m->output_attrs[i].index = i;
        if (rknn_query(m->ctx, RKNN_QUERY_OUTPUT_ATTR, &m->output_attrs[i], sizeof(m->output_attrs[i])) < 0)
            return -1;
    }
    fprintf(stderr, "[%s] input=%ux%ux%u fmt=%s outputs=%u\n", name, m->in_w, m->in_h, m->in_c,
            tensor_fmt_name(m->input_attr.fmt), m->io_num.n_output);
    for (i = 0; i < m->io_num.n_output; i++) {
        const rknn_tensor_attr *a = &m->output_attrs[i];
        fprintf(stderr, "[%s] out[%u] n_dims=%u dims=[%u,%u,%u,%u] fmt=%s type=%d\n",
                name, i, a->n_dims, a->dims[0], a->dims[1], a->dims[2], a->dims[3],
                tensor_fmt_name(a->fmt), a->type);
    }
    return 0;
}

void lpr_model_release(struct rknn_model *m)
{
    if (m && m->ctx)
        rknn_destroy(m->ctx);
    if (m)
        memset(m, 0, sizeof(*m));
}

/* ---------------- OCR input prep ---------------- */

static void resize_nn(const uint8_t *src, int sw, int sh, uint8_t *dst, int dw, int dh)
{
    int x, y;
    for (y = 0; y < dh; y++) {
        int sy = y * sh / dh;
        for (x = 0; x < dw; x++) {
            int sx = x * sw / dw;
            memcpy(dst + ((size_t)y * dw + x) * 3U, src + ((size_t)sy * sw + sx) * 3U, 3);
        }
    }
}

static void resize_bilinear(const uint8_t *src, int sw, int sh, uint8_t *dst, int dw, int dh)
{
    int x, y;
    if (sw <= 1 || sh <= 1) {
        resize_nn(src, sw, sh, dst, dw, dh);
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

static void rgb_to_bgr_inplace(uint8_t *buf, int w, int h)
{
    size_t n = (size_t)w * h;
    for (size_t i = 0; i < n; i++) {
        uint8_t t = buf[i * 3U];
        buf[i * 3U] = buf[i * 3U + 2];
        buf[i * 3U + 2] = t;
    }
}

static void apply_gray3(uint8_t *buf, int w, int h)
{
    size_t n = (size_t)w * h;
    for (size_t i = 0; i < n; i++) {
        uint8_t b = buf[i * 3U];
        uint8_t g = buf[i * 3U + 1];
        uint8_t r = buf[i * 3U + 2];
        uint8_t y = (uint8_t)((114 * b + 587 * g + 299 * r) / 1000);
        buf[i * 3U] = y;
        buf[i * 3U + 1] = y;
        buf[i * 3U + 2] = y;
    }
}

static void apply_bin(uint8_t *buf, int w, int h)
{
    size_t n = (size_t)w * h;
    apply_gray3(buf, w, h);
    for (size_t i = 0; i < n; i++) {
        uint8_t y = buf[i * 3U] > 127 ? 255 : 0;
        buf[i * 3U] = y;
        buf[i * 3U + 1] = y;
        buf[i * 3U + 2] = y;
    }
}

/* ---------------- OCR output layout autodetect ---------------- */

static bool dim_matches_class(int dim, int key_count)
{
    return dim > 1 && key_count > 0 && (dim == key_count || dim == key_count + 1);
}

static void set_layout_ct(int c, int t, int *t_size, int *c_size, int *t_stride, int *c_stride)
{
    *c_size = c; *t_size = t; *t_stride = 1; *c_stride = *t_size;
}

static void set_layout_tc(int t, int c, int *t_size, int *c_size, int *t_stride, int *c_stride)
{
    *t_size = t; *c_size = c; *t_stride = *c_size; *c_stride = 1;
}

static bool build_ocr_layout(const rknn_tensor_attr *a, int key_count,
                             int *t_size, int *c_size, int *t_stride, int *c_stride)
{
    if (a->n_dims == 3) {
        int d1 = (int)a->dims[1], d2 = (int)a->dims[2];
        bool d1c = dim_matches_class(d1, key_count), d2c = dim_matches_class(d2, key_count);
        if (d1c && !d2c) set_layout_ct(d1, d2, t_size, c_size, t_stride, c_stride);
        else if (d2c && !d1c) set_layout_tc(d1, d2, t_size, c_size, t_stride, c_stride);
        else if (a->fmt == RKNN_TENSOR_NCHW) set_layout_ct(d1, d2, t_size, c_size, t_stride, c_stride);
        else set_layout_tc(d1, d2, t_size, c_size, t_stride, c_stride);
        return true;
    }
    if (a->n_dims == 4) {
        int d1 = (int)a->dims[1], d2 = (int)a->dims[2], d3 = (int)a->dims[3];
        if (dim_matches_class(d1, key_count)) set_layout_ct(d1, d2 * d3, t_size, c_size, t_stride, c_stride);
        else if (dim_matches_class(d3, key_count)) set_layout_tc(d1 * d2, d3, t_size, c_size, t_stride, c_stride);
        else if (a->fmt == RKNN_TENSOR_NCHW) set_layout_ct(d1, d2 * d3, t_size, c_size, t_stride, c_stride);
        else set_layout_tc(d1 * d2, d3, t_size, c_size, t_stride, c_stride);
        return true;
    }
    if (a->n_dims == 2) {
        int d0 = (int)a->dims[0], d1 = (int)a->dims[1];
        if (dim_matches_class(d0, key_count)) set_layout_ct(d0, d1, t_size, c_size, t_stride, c_stride);
        else set_layout_tc(d0, d1, t_size, c_size, t_stride, c_stride);
        return true;
    }
    return false;
}

/* ---------------- OCR forward + decode ---------------- */

int lpr_ocr_run(struct rknn_model *m, const struct ocr_keys *keys,
                enum ocr_preproc_mode preproc, enum ocr_decode_family family,
                const uint8_t *crop_rgb, int crop_w, int crop_h,
                char *text, size_t text_len, float *conf,
                struct ocr_decode_diag *diag, struct ocr_timing *timing)
{
    uint8_t *input;
    rknn_input in;
    rknn_output out;
    int ret;
    int t_size = 0, c_size = 0, t_stride = 0, c_stride = 0;
    int blank;
    int64_t t0, t1, t2, t3, t4, t5;
    if (timing)
        memset(timing, 0, sizeof(*timing));
    t0 = lpr_mono_us();
    input = malloc((size_t)m->in_w * m->in_h * 3U);
    if (!input)
        return -1;
    resize_bilinear(crop_rgb, crop_w, crop_h, input, (int)m->in_w, (int)m->in_h);
    rgb_to_bgr_inplace(input, (int)m->in_w, (int)m->in_h);
    if (preproc == OCR_PREPROC_GRAY)
        apply_gray3(input, (int)m->in_w, (int)m->in_h);
    else if (preproc == OCR_PREPROC_BIN)
        apply_bin(input, (int)m->in_w, (int)m->in_h);
    t1 = lpr_mono_us();
    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf = input;
    in.size = m->in_w * m->in_h * 3U;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    ret = rknn_inputs_set(m->ctx, 1, &in);
    t2 = lpr_mono_us();
    if (ret < 0) { free(input); return ret; }
    ret = rknn_run(m->ctx, NULL);
    t3 = lpr_mono_us();
    if (ret < 0) { free(input); return ret; }
    memset(&out, 0, sizeof(out));
    out.want_float = 1;
    ret = rknn_outputs_get(m->ctx, 1, &out, NULL);
    t4 = lpr_mono_us();
    free(input);
    if (ret < 0)
        return ret;
    if (!build_ocr_layout(&m->output_attrs[0], keys->count, &t_size, &c_size, &t_stride, &c_stride)) {
        rknn_outputs_release(m->ctx, 1, &out);
        return -1;
    }
    blank = (c_size == keys->count + 1) ? keys->count : c_size - 1;
    {
        const char *key_ptrs[MAX_OCR_KEYS];
        for (int i = 0; i < keys->count; i++)
            key_ptrs[i] = keys->keys[i];
        ret = ocr_decode_logits((const float *)out.buf, t_size, c_size, t_stride, c_stride,
                                key_ptrs, keys->count, blank, family,
                                text, text_len, conf, diag);
    }
    t5 = lpr_mono_us();
    if (timing) {
        timing->prep_ms = (double)(t1 - t0) / 1000.0;
        timing->input_ms = (double)(t2 - t1) / 1000.0;
        timing->run_ms = (double)(t3 - t2) / 1000.0;
        timing->output_ms = (double)(t4 - t3) / 1000.0;
        timing->decode_ms = (double)(t5 - t4) / 1000.0;
    }
    rknn_outputs_release(m->ctx, 1, &out);
    return ret;
}

void lpr_ocr_log_contract(const char *route_name,
                          const struct rknn_model *m,
                          const struct ocr_keys *keys)
{
    int t_size = 0, c_size = 0, t_stride = 0, c_stride = 0;
    int blank = -1;
    const char *k0 = (keys && keys->count > 0) ? keys->keys[0] : "<none>";
    const char *k11 = (keys && keys->count > 11) ? keys->keys[11] : "<none>";
    const char *klast = (keys && keys->count > 0) ? keys->keys[keys->count - 1] : "<none>";
    if (!m || !keys) return;
    if (build_ocr_layout(&m->output_attrs[0], keys->count, &t_size, &c_size, &t_stride, &c_stride))
        blank = (c_size == keys->count + 1) ? keys->count : c_size - 1;
    fprintf(stderr,
            "[bgp-live] route=%s keys=%d first=%s idx12=%s last=%s ocr_layout=t%d c%d t_stride=%d c_stride=%d blank=%d\n",
            route_name ? route_name : "?", keys->count, k0, k11, klast, t_size, c_size, t_stride, c_stride, blank);
}
