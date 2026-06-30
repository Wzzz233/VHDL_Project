// SPDX-License-Identifier: GPL-2.0
/* YOLOv8n-pose plate detector with NMS and quad output. */

#include "lpr_detector.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct tensor_cn_view {
    const float *buf;
    int c;
    int n;
    bool c_major;
};

struct letterbox_meta {
    float scale;
    int pad_x;
    int pad_y;
    int src_w;
    int src_h;
    int dst_w;
    int dst_h;
    bool valid;
};

/* ---------------- Image resize helpers ---------------- */

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

static void resize_nn_bgrx(const uint8_t *src, int sw, int sh, uint8_t *dst, int dw, int dh)
{
    int x, y;
    for (y = 0; y < dh; y++) {
        int sy = y * sh / dh;
        for (x = 0; x < dw; x++) {
            int sx = x * sw / dw;
            lpr_bgrx_pixel_rgb(src, sw, sx, sy, dst + ((size_t)y * dw + x) * 3U);
        }
    }
}

static void prepare_detect_input_bgrx(const uint8_t *src, int sw, int sh, uint8_t *dst,
                                      enum det_resize_mode mode, struct letterbox_meta *meta)
{
    memset(meta, 0, sizeof(*meta));
    meta->src_w = sw;
    meta->src_h = sh;
    meta->dst_w = ALGO_STREAM_SIZE;
    meta->dst_h = ALGO_STREAM_SIZE;
    if (mode == DET_RESIZE_STRETCH) {
        resize_nn_bgrx(src, sw, sh, dst, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE);
        meta->scale = 1.0f;
        meta->valid = false;
        return;
    }
    {
        float sx = (float)ALGO_STREAM_SIZE / (float)sw;
        float sy = (float)ALGO_STREAM_SIZE / (float)sh;
        float scale = sx < sy ? sx : sy;
        int nw = (int)((float)sw * scale + 0.5f);
        int nh = (int)((float)sh * scale + 0.5f);
        int off_x = (ALGO_STREAM_SIZE - nw) / 2;
        int off_y = (ALGO_STREAM_SIZE - nh) / 2;
        uint8_t *tmp;
        memset(dst, 114, (size_t)ALGO_STREAM_SIZE * ALGO_STREAM_SIZE * 3U);
        if (nw < 1) nw = 1;
        if (nh < 1) nh = 1;
        tmp = malloc((size_t)nw * nh * 3U);
        if (!tmp)
            return;
        resize_nn_bgrx(src, sw, sh, tmp, nw, nh);
        for (int y = 0; y < nh; y++)
            memcpy(dst + ((size_t)(off_y + y) * ALGO_STREAM_SIZE + off_x) * 3U,
                   tmp + (size_t)y * nw * 3U, (size_t)nw * 3U);
        free(tmp);
        meta->scale = scale;
        meta->pad_x = off_x;
        meta->pad_y = off_y;
        meta->valid = true;
    }
}

static void prepare_detect_input(const uint8_t *src, int sw, int sh, uint8_t *dst,
                                 enum det_resize_mode mode, struct letterbox_meta *meta)
{
    memset(meta, 0, sizeof(*meta));
    meta->src_w = sw;
    meta->src_h = sh;
    meta->dst_w = ALGO_STREAM_SIZE;
    meta->dst_h = ALGO_STREAM_SIZE;
    if (mode == DET_RESIZE_STRETCH) {
        resize_nn(src, sw, sh, dst, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE);
        meta->scale = 1.0f;
        meta->valid = false;
        return;
    }
    {
        float sx = (float)ALGO_STREAM_SIZE / (float)sw;
        float sy = (float)ALGO_STREAM_SIZE / (float)sh;
        float scale = sx < sy ? sx : sy;
        int nw = (int)((float)sw * scale + 0.5f);
        int nh = (int)((float)sh * scale + 0.5f);
        int off_x = (ALGO_STREAM_SIZE - nw) / 2;
        int off_y = (ALGO_STREAM_SIZE - nh) / 2;
        uint8_t *tmp;
        memset(dst, 114, (size_t)ALGO_STREAM_SIZE * ALGO_STREAM_SIZE * 3U);
        if (nw < 1) nw = 1;
        if (nh < 1) nh = 1;
        tmp = malloc((size_t)nw * nh * 3U);
        if (!tmp)
            return;
        resize_nn(src, sw, sh, tmp, nw, nh);
        for (int y = 0; y < nh; y++)
            memcpy(dst + ((size_t)(off_y + y) * ALGO_STREAM_SIZE + off_x) * 3U,
                   tmp + (size_t)y * nw * 3U, (size_t)nw * 3U);
        free(tmp);
        meta->scale = scale;
        meta->pad_x = off_x;
        meta->pad_y = off_y;
        meta->valid = true;
    }
}

static void map_point_from_detect(float *x, float *y, enum det_resize_mode mode,
                                  const struct letterbox_meta *m, int sw, int sh)
{
    if (mode == DET_RESIZE_LETTERBOX && m->valid) {
        *x = (*x - (float)m->pad_x) / m->scale;
        *y = (*y - (float)m->pad_y) / m->scale;
    } else {
        *x = *x * (float)sw / (float)ALGO_STREAM_SIZE;
        *y = *y * (float)sh / (float)ALGO_STREAM_SIZE;
    }
    if (*x < 0.0f) *x = 0.0f;
    if (*y < 0.0f) *y = 0.0f;
    if (*x > (float)(sw - 1)) *x = (float)(sw - 1);
    if (*y > (float)(sh - 1)) *y = (float)(sh - 1);
}

static void bbox_from_quad(struct det_box *d, int sw, int sh)
{
    float minx = d->quad[0], maxx = d->quad[0], miny = d->quad[1], maxy = d->quad[1];
    int i;
    for (i = 1; i < 4; i++) {
        float x = d->quad[i * 2];
        float y = d->quad[i * 2 + 1];
        if (x < minx) minx = x;
        if (x > maxx) maxx = x;
        if (y < miny) miny = y;
        if (y > maxy) maxy = y;
    }
    d->x1 = (int)floorf(minx);
    d->y1 = (int)floorf(miny);
    d->x2 = (int)ceilf(maxx);
    d->y2 = (int)ceilf(maxy);
    if (d->x1 < 0) d->x1 = 0;
    if (d->y1 < 0) d->y1 = 0;
    if (d->x2 >= sw) d->x2 = sw - 1;
    if (d->y2 >= sh) d->y2 = sh - 1;
}

/* ---------------- NMS ---------------- */

static float box_iou(const struct det_box *a, const struct det_box *b)
{
    int x1 = a->x1 > b->x1 ? a->x1 : b->x1;
    int y1 = a->y1 > b->y1 ? a->y1 : b->y1;
    int x2 = a->x2 < b->x2 ? a->x2 : b->x2;
    int y2 = a->y2 < b->y2 ? a->y2 : b->y2;
    int iw = x2 - x1 + 1;
    int ih = y2 - y1 + 1;
    float inter, aa, ba;
    if (iw <= 0 || ih <= 0)
        return 0.0f;
    inter = (float)iw * (float)ih;
    aa = (float)(a->x2 - a->x1 + 1) * (float)(a->y2 - a->y1 + 1);
    ba = (float)(b->x2 - b->x1 + 1) * (float)(b->y2 - b->y1 + 1);
    return inter / fmaxf(1.0f, aa + ba - inter);
}

static int compare_det_desc(const void *pa, const void *pb)
{
    const struct det_box *a = (const struct det_box *)pa;
    const struct det_box *b = (const struct det_box *)pb;
    if (a->conf < b->conf) return 1;
    if (a->conf > b->conf) return -1;
    return 0;
}

static void nms(struct det_box *dets, int *count, float thr, int max_det)
{
    struct det_box kept[MAX_DETS];
    int kept_n = 0;
    int i, j;
    qsort(dets, (size_t)*count, sizeof(dets[0]), compare_det_desc);
    for (i = 0; i < *count && kept_n < max_det; i++) {
        bool drop = false;
        for (j = 0; j < kept_n; j++) {
            if (box_iou(&dets[i], &kept[j]) > thr) {
                drop = true;
                break;
            }
        }
        if (!drop)
            kept[kept_n++] = dets[i];
    }
    memcpy(dets, kept, (size_t)kept_n * sizeof(dets[0]));
    *count = kept_n;
}

/* ---------------- Tensor view (NCHW vs NHWC autodetect) ---------------- */

static bool build_tensor_cn_view(const rknn_tensor_attr *a, const float *buf, struct tensor_cn_view *tv)
{
    int dims[4];
    int k = 0;
    memset(tv, 0, sizeof(*tv));
    if (a->n_dims == 3) {
        int d1 = (int)a->dims[1];
        int d2 = (int)a->dims[2];
        if (d2 == OBB_POINT_COUNT) { tv->buf = buf; tv->c = d1; tv->n = d2; tv->c_major = true; return true; }
        if (d1 == OBB_POINT_COUNT) { tv->buf = buf; tv->c = d2; tv->n = d1; tv->c_major = false; return true; }
        return false;
    }
    for (uint32_t i = 1; i < a->n_dims && i < 4; i++) {
        int d = (int)a->dims[i];
        if (d > 1) dims[k++] = d;
    }
    if (k != 2)
        return false;
    if (dims[1] == OBB_POINT_COUNT) { tv->buf = buf; tv->c = dims[0]; tv->n = dims[1]; tv->c_major = true; return true; }
    if (dims[0] == OBB_POINT_COUNT) { tv->buf = buf; tv->c = dims[1]; tv->n = dims[0]; tv->c_major = false; return true; }
    return false;
}

static float tensor_read(const struct tensor_cn_view *tv, int c, int n)
{
    if (tv->c_major)
        return tv->buf[(size_t)c * tv->n + n];
    return tv->buf[(size_t)n * tv->c + c];
}

int lpr_detector_pose_nc(const struct rknn_model *m)
{
    uint32_t i;
    for (i = 0; i < m->io_num.n_output; i++) {
        struct tensor_cn_view tv;
        if (build_tensor_cn_view(&m->output_attrs[i], NULL, &tv) && tv.n == OBB_POINT_COUNT && tv.c >= POSE_MIN_CHANNELS)
            return tv.c - POSE_BOX_CHANNELS - POSE_KPT_CHANNELS;
    }
    return 0;
}

/* ---------------- Pose decode ---------------- */

static int decode_pose_outputs(const struct rknn_model *m, const rknn_output *outs,
                               int pose_nc, int class_filter, float conf_thr,
                               int img_w, int img_h, enum det_resize_mode resize_mode,
                               const struct letterbox_meta *lb,
                               struct det_box *out, int *out_count)
{
    struct tensor_cn_view tv;
    int out_idx = -1;
    int count = 0;
    uint32_t oi;
    *out_count = 0;
    for (oi = 0; oi < m->io_num.n_output; oi++) {
        if (build_tensor_cn_view(&m->output_attrs[oi], (const float *)outs[oi].buf, &tv) &&
            tv.n == OBB_POINT_COUNT && tv.c >= POSE_MIN_CHANNELS) {
            out_idx = (int)oi;
            break;
        }
    }
    if (out_idx < 0)
        return -1;
    (void)out_idx;
    if (pose_nc <= 0)
        pose_nc = tv.c - POSE_BOX_CHANNELS - POSE_KPT_CHANNELS;
    for (int i = 0; i < tv.n && count < MAX_DETS; i++) {
        int best_cls = 0;
        float best_score;
        int kpt_base;
        struct det_box d;
        if (pose_nc == 1) {
            best_score = tensor_read(&tv, 4, i);
        } else {
            best_score = -1.0f;
            for (int c = 0; c < pose_nc; c++) {
                float s = tensor_read(&tv, 4 + c, i);
                if (s > best_score) { best_score = s; best_cls = c; }
            }
        }
        if (!isfinite(best_score))
            continue;
        if (best_score < 0.0f || best_score > 1.0f)
            best_score = lpr_sigmoidf(best_score);
        if (best_score < conf_thr)
            continue;
        if (class_filter >= 0 && best_cls != class_filter)
            continue;
        memset(&d, 0, sizeof(d));
        d.conf = best_score;
        d.cls = best_cls;
        kpt_base = POSE_BOX_CHANNELS + pose_nc;
        bool valid = true;
        for (int k = 0; k < POSE_KPT_COUNT; k++) {
            float x = tensor_read(&tv, kpt_base + k * POSE_KPT_DIMS + 0, i);
            float y = tensor_read(&tv, kpt_base + k * POSE_KPT_DIMS + 1, i);
            float v = tensor_read(&tv, kpt_base + k * POSE_KPT_DIMS + 2, i);
            if (!isfinite(x) || !isfinite(y) || !isfinite(v)) { valid = false; break; }
            map_point_from_detect(&x, &y, resize_mode, lb, img_w, img_h);
            d.quad[k * 2] = x;
            d.quad[k * 2 + 1] = y;
        }
        if (!valid)
            continue;
        bbox_from_quad(&d, img_w, img_h);
        if (d.x2 <= d.x1 || d.y2 <= d.y1)
            continue;
        out[count++] = d;
    }
    nms(out, &count, 0.45f, MAX_DETS);
    *out_count = count;
    return 0;
}

int lpr_detector_run(struct rknn_model *m, const uint8_t *rgb,
                     int img_w, int img_h, uint8_t *input,
                     enum det_resize_mode resize_mode,
                     int pose_nc, int class_filter,
                     float conf_thr, float nms_iou, int max_det,
                     struct det_box *dets, int *det_count)
{
    struct letterbox_meta lb;
    rknn_input in;
    rknn_output outs[8];
    int ret;
    prepare_detect_input(rgb, img_w, img_h, input, resize_mode, &lb);
    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf = input;
    in.size = m->in_w * m->in_h * 3U;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    ret = rknn_inputs_set(m->ctx, 1, &in);
    if (ret < 0) return ret;
    ret = rknn_run(m->ctx, NULL);
    if (ret < 0) return ret;
    memset(outs, 0, sizeof(outs));
    for (uint32_t i = 0; i < m->io_num.n_output; i++)
        outs[i].want_float = 1;
    ret = rknn_outputs_get(m->ctx, m->io_num.n_output, outs, NULL);
    if (ret < 0) return ret;
    ret = decode_pose_outputs(m, outs, pose_nc, class_filter, conf_thr, img_w, img_h,
                              resize_mode, &lb, dets, det_count);
    rknn_outputs_release(m->ctx, m->io_num.n_output, outs);
    if (ret == 0)
        nms(dets, det_count, nms_iou, max_det);
    return ret;
}

int lpr_detector_pick_best(const struct det_box *dets, int count)
{
    int best = -1;
    float best_conf = -1.0f;
    for (int i = 0; i < count; i++) {
        if (dets[i].conf > best_conf) {
            best_conf = dets[i].conf;
            best = i;
        }
    }
    return best;
}

int lpr_detector_run_bgrx(struct rknn_model *m, const uint8_t *bgrx,
                          int img_w, int img_h, uint8_t *input,
                          enum det_resize_mode resize_mode,
                          int pose_nc, int class_filter,
                          float conf_thr, float nms_iou, int max_det,
                          struct det_box *dets, int *det_count)
{
    struct letterbox_meta lb;
    rknn_input in;
    rknn_output outs[8];
    int ret;
    prepare_detect_input_bgrx(bgrx, img_w, img_h, input, resize_mode, &lb);
    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf = input;
    in.size = m->in_w * m->in_h * 3U;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    ret = rknn_inputs_set(m->ctx, 1, &in);
    if (ret < 0) return ret;
    ret = rknn_run(m->ctx, NULL);
    if (ret < 0) return ret;
    memset(outs, 0, sizeof(outs));
    for (uint32_t i = 0; i < m->io_num.n_output; i++)
        outs[i].want_float = 1;
    ret = rknn_outputs_get(m->ctx, m->io_num.n_output, outs, NULL);
    if (ret < 0) return ret;
    ret = decode_pose_outputs(m, outs, pose_nc, class_filter, conf_thr, img_w, img_h,
                              resize_mode, &lb, dets, det_count);
    rknn_outputs_release(m->ctx, m->io_num.n_output, outs);
    if (ret == 0)
        nms(dets, det_count, nms_iou, max_det);
    return ret;
}
