/*
 * FPGA Bird Detection Display Application
 *
 * Display path:
 *   /dev/fpga_dma0 -> appsrc(BGR16) -> queue(leaky) -> kmssink
 *
 * Inference path:
 *   Dedicated worker thread converts BGR565/BGRX8888 to RGB888,
 *   letterbox-resizes to 640x640 with pad_value=114, runs
 *   standard COCO-80 YOLOv8n detection RKNN, filters class_id=16 (bird),
 *   and publishes boxes back to the display thread for overlay.
 *
 * Model contract:
 *   Input:  NHWC uint8, 640x640x3, letterbox pad=114
 *   Output: one tensor [1,84,8400] (box=4 + cls=80), coords already in
 *           640x640 pixel space (strides baked into ONNX graph),
 *           class scores already sigmoided.
 *
 * Architecture mirrors fpga_lpr_display.c but stripped of OCR, OBB,
 * A-channel fusion, refiner and offline paths.
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <rknn_api.h>

#include "pcie_fpga_dma.h"

#define DEFAULT_DEVICE "/dev/" FPGA_DMA_DEV_NAME
#define DEFAULT_DRM_CARD "/dev/dri/card0"
#define DEFAULT_FPS 15
#define DEFAULT_TIMEOUT_MS 5000
#define DEFAULT_STATS_INTERVAL 1
#define DEFAULT_COPY_BUFFERS 2
#define DEFAULT_QUEUE_DEPTH 1
#define MIN_COPY_BUFFERS 2
#define MAX_COPY_BUFFERS 6

#define BIRD_INPUT_SIZE       640
#define BIRD_PAD_VALUE        114
#define BIRD_GRID_POINTS      8400
#define BIRD_COCO_CLASSES     80
#define BIRD_TOTAL_CHANNELS   (4 + BIRD_COCO_CLASSES)   /* 84 */
#define BIRD_CLASS_ID         16                         /* COCO "bird" */

#define MAX_DETS              128
#define PRE_NMS_CAP           (MAX_DETS * 4)

#define COLOR_YELLOW_565      0xFFE0
#define COLOR_CYAN_565        0x07FF
#define COLOR_RED_565         0xF800
#define COLOR_GREEN_565       0x07E0
#define COLOR_MAGENTA_565     0xF81F
#define OVERLAY_TEXT_SCALE    3

enum pixel_order {
    PIXEL_ORDER_BGR565 = 0,
    PIXEL_ORDER_RGB565,
};

struct options {
    const char *device_path;
    const char *drm_card_path;
    const char *bird_model_path;
    int connector_id;
    int fps;
    enum pixel_order pixel_order;
    int timeout_ms;
    int stats_interval;
    int copy_buffers;
    int queue_depth;
    float min_conf;
    float nms_iou;
    int max_det;
    int class_id;                 /* -1 = any; default 16 bird */
    bool swap16;
};

struct det_box {
    int   x1, y1, x2, y2;
    float cx, cy, w, h;
    float conf;
    int   cls;
};

struct letterbox_meta {
    float scale;
    int   pad_x, pad_y;
    int   src_w, src_h;
    int   dst_w, dst_h;
    bool  valid;
};

struct yolo_model {
    const char *name;
    const char *path;
    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr input_attr;
    rknn_tensor_attr output_attrs[8];
    uint32_t in_w, in_h, in_c;
};

struct bird_results {
    struct det_box dets[MAX_DETS];
    int   det_count;
    int   det_raw_count;
    uint64_t frame_seq;
    double   infer_ms_last;
    uint64_t infer_frames_total;
    double   infer_ms_total;
};

struct frame_slot {
    uint8_t *data;
    bool     in_use;
    uint64_t generation;
};

struct slot_ticket {
    int idx;
    uint64_t generation;
};

struct app_ctx {
    struct options opt;
    int dev_fd;
    int drm_fd;
    void *dma_map;
    size_t dma_map_size;
    uint8_t *dma_copy;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t src_frame_bpp;
    size_t   src_frame_size;
    bool     src_is_bgrx;
    uint32_t frame_bpp;
    size_t   frame_size;

    struct frame_slot *slots;
    int slot_count;
    GMutex slots_lock;
    GCond  slots_cond;

    GstElement *pipeline;
    GstElement *appsrc;
    GstElement *queue;
    GstElement *sink;
    GstBus     *bus;

    bool running;
    uint64_t captured_frames;
    uint64_t pushed_frames;
    uint64_t released_frames;
    uint64_t next_pts_ns;
    int64_t  last_stats_us;
    uint64_t last_stats_cap;
    uint64_t last_stats_rel;
    uint64_t last_stats_infer;
    uint64_t slot_wait_timeout_count;
    uint64_t infer_overwrite_count;

    pthread_t infer_thread;
    pthread_mutex_t infer_lock;
    pthread_cond_t  infer_cond;
    uint8_t *infer_latest_raw;
    bool     infer_has_new;
    uint64_t infer_frame_seq;

    pthread_mutex_t result_lock;
    struct bird_results results;

    struct yolo_model bird_model;
};

struct frame_cookie {
    struct app_ctx *ctx;
    struct slot_ticket ticket;
};

static volatile sig_atomic_t g_stop = 0;

static int64_t mono_us(void) { return g_get_monotonic_time(); }

static void signal_handler(int signo)
{
    if (signo == SIGINT || signo == SIGTERM)
        g_stop = 1;
}

/* ---------------- pixel format helpers ---------------- */

static void decode_pixel565(const struct options *opt, uint8_t lo_in, uint8_t hi_in,
                            uint8_t *r8, uint8_t *g8, uint8_t *b8)
{
    uint8_t lo = lo_in, hi = hi_in;
    uint16_t pix;
    uint8_t r5, g6, b5;
    if (opt->swap16) { uint8_t t = lo; lo = hi; hi = t; }
    pix = (uint16_t)lo | ((uint16_t)hi << 8);
    if (opt->pixel_order == PIXEL_ORDER_BGR565) {
        r5 =  pix        & 0x1F;
        g6 = (pix >> 5)  & 0x3F;
        b5 = (pix >> 11) & 0x1F;
    } else {
        r5 = (pix >> 11) & 0x1F;
        g6 = (pix >> 5)  & 0x3F;
        b5 =  pix        & 0x1F;
    }
    *r8 = (uint8_t)((r5 << 3) | (r5 >> 2));
    *g8 = (uint8_t)((g6 << 2) | (g6 >> 4));
    *b8 = (uint8_t)((b5 << 3) | (b5 >> 2));
}

static void raw565_to_rgb888_full(struct app_ctx *ctx, const uint8_t *raw, uint8_t *rgb)
{
    size_t pixels = (size_t)ctx->frame_width * ctx->frame_height;
    for (size_t i = 0; i < pixels; i++) {
        uint8_t r, g, b;
        decode_pixel565(&ctx->opt, raw[i * 2], raw[i * 2 + 1], &r, &g, &b);
        rgb[i * 3 + 0] = r;
        rgb[i * 3 + 1] = g;
        rgb[i * 3 + 2] = b;
    }
}

static void bgrx8888_to_rgb888(const uint8_t *src, int w, int h, uint8_t *dst)
{
    size_t pixels = (size_t)w * h;
    for (size_t i = 0; i < pixels; i++) {
        const uint8_t *p = src + i * 4U;
        uint8_t *q = dst + i * 3U;
        q[0] = p[2];  /* R */
        q[1] = p[1];  /* G */
        q[2] = p[0];  /* B */
    }
}

/* ---------------- resize / letterbox ---------------- */

static void resize_rgb888_nn(const uint8_t *src, int sw, int sh,
                             uint8_t *dst, int dw, int dh)
{
    for (int y = 0; y < dh; y++) {
        int sy = (y * sh) / dh;
        const uint8_t *srow = src + (size_t)sy * sw * 3;
        uint8_t *drow = dst + (size_t)y * dw * 3;
        for (int x = 0; x < dw; x++) {
            int sx = (x * sw) / dw;
            const uint8_t *p = srow + sx * 3;
            drow[x * 3 + 0] = p[0];
            drow[x * 3 + 1] = p[1];
            drow[x * 3 + 2] = p[2];
        }
    }
}

/* YOLOv8 contract: letterbox resize, center-pad with 114 (gray). */
static void resize_rgb888_letterbox_pad(const uint8_t *src, int sw, int sh,
                                        uint8_t *dst, int dw, int dh,
                                        uint8_t pad, struct letterbox_meta *meta)
{
    if (meta) {
        memset(meta, 0, sizeof(*meta));
        meta->src_w = sw; meta->src_h = sh;
        meta->dst_w = dw; meta->dst_h = dh;
    }
    if (sw <= 0 || sh <= 0 || dw <= 0 || dh <= 0)
        return;

    size_t dst_sz = (size_t)dw * dh * 3U;
    memset(dst, pad, dst_sz);

    float fx = (float)dw / (float)sw;
    float fy = (float)dh / (float)sh;
    float scale = (fx < fy) ? fx : fy;
    if (scale <= 0.0f)
        return;

    int scaled_w = (int)((float)sw * scale + 0.5f);
    int scaled_h = (int)((float)sh * scale + 0.5f);
    if (scaled_w < 1) scaled_w = 1;
    if (scaled_h < 1) scaled_h = 1;
    if (scaled_w > dw) scaled_w = dw;
    if (scaled_h > dh) scaled_h = dh;

    int off_x = (dw - scaled_w) / 2;
    int off_y = (dh - scaled_h) / 2;

    uint8_t *tmp = malloc((size_t)scaled_w * scaled_h * 3U);
    if (!tmp)
        return;
    resize_rgb888_nn(src, sw, sh, tmp, scaled_w, scaled_h);
    for (int y = 0; y < scaled_h; y++) {
        const uint8_t *srow = tmp + (size_t)y * scaled_w * 3U;
        uint8_t *drow = dst + ((size_t)(off_y + y) * dw + (size_t)off_x) * 3U;
        memcpy(drow, srow, (size_t)scaled_w * 3U);
    }
    free(tmp);

    if (meta) {
        meta->scale = scale;
        meta->pad_x = off_x;
        meta->pad_y = off_y;
        meta->valid = true;
    }
}

static void map_box_from_letterbox(struct det_box *b, const struct letterbox_meta *lb)
{
    if (!lb || !lb->valid || lb->scale <= 0.0f)
        return;
    float x1 = ((float)b->x1 - (float)lb->pad_x) / lb->scale;
    float x2 = ((float)b->x2 - (float)lb->pad_x) / lb->scale;
    float y1 = ((float)b->y1 - (float)lb->pad_y) / lb->scale;
    float y2 = ((float)b->y2 - (float)lb->pad_y) / lb->scale;
    b->x1 = (int)lroundf(x1);
    b->x2 = (int)lroundf(x2);
    b->y1 = (int)lroundf(y1);
    b->y2 = (int)lroundf(y2);
    if (b->x1 < 0) b->x1 = 0;
    if (b->y1 < 0) b->y1 = 0;
    if (b->x2 >= lb->src_w) b->x2 = lb->src_w - 1;
    if (b->y2 >= lb->src_h) b->y2 = lb->src_h - 1;
    if (b->x2 < b->x1) b->x2 = b->x1;
    if (b->y2 < b->y1) b->y2 = b->y1;
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
    if (iw <= 0 || ih <= 0) return 0.0f;
    int ia = iw * ih;
    int ua = (a->x2 - a->x1 + 1) * (a->y2 - a->y1 + 1)
           + (b->x2 - b->x1 + 1) * (b->y2 - b->y1 + 1) - ia;
    if (ua <= 0) return 0.0f;
    return (float)ia / (float)ua;
}

static int det_conf_cmp(const void *pa, const void *pb)
{
    const struct det_box *a = pa;
    const struct det_box *b = pb;
    if (a->conf < b->conf) return 1;
    if (a->conf > b->conf) return -1;
    return 0;
}

static void nms_inplace(struct det_box *dets, int *count, float iou_thr)
{
    int n = *count;
    if (n <= 0) { *count = 0; return; }
    if (n > PRE_NMS_CAP) n = PRE_NMS_CAP;
    bool removed[PRE_NMS_CAP];
    memset(removed, 0, sizeof(removed[0]) * (size_t)n);
    qsort(dets, (size_t)n, sizeof(dets[0]), det_conf_cmp);
    int out = 0;
    for (int i = 0; i < n; i++) {
        if (removed[i]) continue;
        dets[out++] = dets[i];
        for (int j = i + 1; j < n; j++) {
            if (removed[j] || dets[i].cls != dets[j].cls) continue;
            if (box_iou(&dets[i], &dets[j]) > iou_thr)
                removed[j] = true;
        }
    }
    *count = out;
}

/* ---------------- YOLOv8 COCO-80 decode ----------------
 * Output tensor [1, 84, 8400]  (NCHW-friendly):
 *   rows 0..3  = cx, cy, w, h  (already in 640x640 pixel space)
 *   rows 4..83 = 80 per-class scores (already sigmoided by the ONNX graph)
 * stride multiplication is baked into the export.
 * Depending on the RKNN layout the tensor may also come back as
 * [1, 8400, 84] (c_major=false); we handle both.
 */
struct tensor_view84 {
    const float *buf;
    int c;     /* 84 */
    int n;     /* 8400 */
    bool c_major;
};

static bool build_tensor_view84(const rknn_tensor_attr *a, const float *buf,
                                struct tensor_view84 *tv)
{
    if (!a || !buf || !tv) return false;
    memset(tv, 0, sizeof(*tv));

    /* Collect the non-batch non-unit dims; expect two of size 84 and 8400. */
    int dims[8] = {0};
    int k = 0;
    for (uint32_t i = 0; i < a->n_dims && k < 8; i++) {
        int d = (int)a->dims[i];
        if (d > 1) dims[k++] = d;
    }
    if (k != 2) return false;

    if (dims[0] == BIRD_TOTAL_CHANNELS && dims[1] > 0) {
        tv->buf = buf; tv->c = dims[0]; tv->n = dims[1]; tv->c_major = true;
        return true;
    }
    if (dims[1] == BIRD_TOTAL_CHANNELS && dims[0] > 0) {
        tv->buf = buf; tv->c = dims[1]; tv->n = dims[0]; tv->c_major = false;
        return true;
    }
    return false;
}

static inline float tv84_read(const struct tensor_view84 *tv, int c, int n)
{
    if (tv->c_major)
        return tv->buf[(size_t)c * (size_t)tv->n + (size_t)n];
    return tv->buf[(size_t)n * (size_t)tv->c + (size_t)c];
}

static int decode_yolov8_det_cls80(const struct yolo_model *m,
                                   const rknn_output *outs,
                                   float conf_thr,
                                   int target_class,   /* -1 = any */
                                   int in_w, int in_h, /* model input canvas */
                                   struct det_box *out, int *out_count,
                                   float nms_iou, int max_det)
{
    *out_count = 0;
    if (m->io_num.n_output != 1)
        return -1;
    struct tensor_view84 tv;
    if (!build_tensor_view84(&m->output_attrs[0], (const float *)outs[0].buf, &tv))
        return -1;
    if (tv.c != BIRD_TOTAL_CHANNELS || tv.n <= 0)
        return -1;

    struct det_box cand[PRE_NMS_CAP];
    int count = 0;
    const int grid = tv.n;   /* 2100 for 320x320, 8400 for 640x640 */

    for (int i = 0; i < grid; i++) {
        /* pick max-score class (or just the target class if filtering) */
        float best_score;
        int best_cls;
        if (target_class >= 0) {
            best_cls = target_class;
            best_score = tv84_read(&tv, 4 + target_class, i);
            if (best_score < conf_thr)
                continue;
            /* Also ensure no other class dominates (optional strict mode).
             * Here we trust argmax==target by checking it is the top score. */
            for (int c = 0; c < BIRD_COCO_CLASSES; c++) {
                if (c == target_class) continue;
                float s = tv84_read(&tv, 4 + c, i);
                if (s > best_score) { best_cls = -1; break; }
            }
            if (best_cls < 0)
                continue;
        } else {
            best_cls = 0;
            best_score = tv84_read(&tv, 4, i);
            for (int c = 1; c < BIRD_COCO_CLASSES; c++) {
                float s = tv84_read(&tv, 4 + c, i);
                if (s > best_score) { best_score = s; best_cls = c; }
            }
            if (best_score < conf_thr)
                continue;
        }

        float cx = tv84_read(&tv, 0, i);
        float cy = tv84_read(&tv, 1, i);
        float w  = tv84_read(&tv, 2, i);
        float h  = tv84_read(&tv, 3, i);
        if (w <= 0.0f || h <= 0.0f)
            continue;

        struct det_box det;
        memset(&det, 0, sizeof(det));
        det.cx = cx; det.cy = cy; det.w = w; det.h = h;
        det.x1 = (int)(cx - w * 0.5f + 0.5f);
        det.y1 = (int)(cy - h * 0.5f + 0.5f);
        det.x2 = (int)(cx + w * 0.5f + 0.5f);
        det.y2 = (int)(cy + h * 0.5f + 0.5f);
        if (det.x1 < 0) det.x1 = 0;
        if (det.y1 < 0) det.y1 = 0;
        if (det.x2 >= in_w) det.x2 = in_w - 1;
        if (det.y2 >= in_h) det.y2 = in_h - 1;
        if (det.x2 <= det.x1 || det.y2 <= det.y1)
            continue;
        det.conf = best_score;
        det.cls  = best_cls;

        if (count < PRE_NMS_CAP) {
            cand[count++] = det;
        } else {
            /* replace the lowest-conf slot */
            int min_i = 0;
            float min_conf = cand[0].conf;
            for (int k = 1; k < PRE_NMS_CAP; k++) {
                if (cand[k].conf < min_conf) {
                    min_conf = cand[k].conf;
                    min_i = k;
                }
            }
            if (det.conf > min_conf)
                cand[min_i] = det;
        }
    }

    nms_inplace(cand, &count, nms_iou);
    if (max_det > 0 && count > max_det) count = max_det;
    if (count > MAX_DETS) count = MAX_DETS;
    memcpy(out, cand, (size_t)count * sizeof(out[0]));
    *out_count = count;
    return 0;
}

/* ---------------- RKNN model lifecycle ---------------- */

static const char *tensor_fmt_name(rknn_tensor_format fmt)
{
    switch (fmt) {
    case RKNN_TENSOR_NCHW: return "NCHW";
    case RKNN_TENSOR_NHWC: return "NHWC";
    default: return "UNSPEC";
    }
}

static int rknn_model_load(struct yolo_model *m, const char *name, const char *path)
{
    memset(m, 0, sizeof(*m));
    m->name = name;
    m->path = path;

    FILE *fp = fopen(path, "rb");
    if (!fp) return -1;
    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    void *data = malloc((size_t)sz);
    if (!data) { fclose(fp); return -1; }
    if (fread(data, 1, (size_t)sz, fp) != (size_t)sz) {
        fclose(fp); free(data); return -1;
    }
    fclose(fp);

    if (rknn_init(&m->ctx, data, (uint32_t)sz, 0, NULL) < 0) {
        free(data); return -1;
    }
    free(data);

    if (rknn_query(m->ctx, RKNN_QUERY_IN_OUT_NUM, &m->io_num, sizeof(m->io_num)) < 0)
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
    if (m->io_num.n_output > 8) return -1;
    for (uint32_t i = 0; i < m->io_num.n_output; i++) {
        memset(&m->output_attrs[i], 0, sizeof(m->output_attrs[i]));
        m->output_attrs[i].index = i;
        if (rknn_query(m->ctx, RKNN_QUERY_OUTPUT_ATTR,
                       &m->output_attrs[i], sizeof(m->output_attrs[i])) < 0)
            return -1;
    }
    fprintf(stderr, "[%s] loaded input=%ux%ux%u outputs=%u\n",
            name, m->in_w, m->in_h, m->in_c, m->io_num.n_output);
    for (uint32_t i = 0; i < m->io_num.n_output; i++) {
        const rknn_tensor_attr *a = &m->output_attrs[i];
        fprintf(stderr,
                "  out[%u]: dims=%u x %u x %u x %u n_dims=%u fmt=%s\n",
                i, a->dims[0], a->dims[1], a->dims[2], a->dims[3],
                a->n_dims, tensor_fmt_name(a->fmt));
    }
    if (m->in_w <= 0 || m->in_h <= 0 || m->in_c != 3) {
        fprintf(stderr, "[%s] WARN invalid input layout %ux%ux%u\n",
                name, m->in_w, m->in_h, m->in_c);
    }
    return 0;
}

static void rknn_model_release(struct yolo_model *m)
{
    if (m->ctx) rknn_destroy(m->ctx);
    memset(m, 0, sizeof(*m));
}

/* ---------------- FPGA DMA init / frame copy ---------------- */

static int init_fpga_dma(struct app_ctx *ctx)
{
    struct fpga_info info;
    struct buffer_map map;
    uint32_t inferred_format;

    ctx->dev_fd = open(ctx->opt.device_path, O_RDWR | O_CLOEXEC);
    if (ctx->dev_fd < 0) return -1;
    if (ioctl(ctx->dev_fd, FPGA_DMA_GET_INFO, &info) < 0) return -1;
    if (info.frame_width != 1280 || info.frame_height != 720) return -1;

    inferred_format = info.pixel_format;
    if (inferred_format != FPGA_PIXEL_FORMAT_BGR565 &&
        inferred_format != FPGA_PIXEL_FORMAT_BGRX8888) {
        inferred_format = (info.frame_bpp == 4) ? FPGA_PIXEL_FORMAT_BGRX8888
                                                : FPGA_PIXEL_FORMAT_BGR565;
    }
    info.frame_bpp = (inferred_format == FPGA_PIXEL_FORMAT_BGRX8888) ? 4 : 2;

    ctx->frame_width  = info.frame_width;
    ctx->frame_height = info.frame_height;
    ctx->src_frame_bpp = info.frame_bpp;
    ctx->src_is_bgrx = (inferred_format == FPGA_PIXEL_FORMAT_BGRX8888);
    ctx->src_frame_size = (size_t)ctx->frame_width * ctx->frame_height * ctx->src_frame_bpp;
    ctx->frame_bpp  = 2;  /* display is always BGR565 */
    ctx->frame_size = (size_t)ctx->frame_width * ctx->frame_height * ctx->frame_bpp;
    if (ctx->src_is_bgrx) ctx->opt.swap16 = false;

    memset(&map, 0, sizeof(map));
    map.index = 0;
    if (ioctl(ctx->dev_fd, FPGA_DMA_MAP_BUFFER, &map) < 0) return -1;
    if (map.size < ctx->src_frame_size) return -1;
    ctx->dma_map_size = map.size;
    ctx->dma_map = mmap(NULL, ctx->dma_map_size, PROT_READ, MAP_SHARED, ctx->dev_fd, 0);
    if (ctx->dma_map == MAP_FAILED) { ctx->dma_map = NULL; return -1; }

    ctx->dma_copy = malloc(ctx->src_frame_size);
    return ctx->dma_copy ? 0 : -1;
}

static int trigger_frame_dma(struct app_ctx *ctx)
{
    struct dma_transfer t;
    memset(&t, 0, sizeof(t));
    t.size = (uint32_t)ctx->src_frame_size;
    t.user_buf = (uint64_t)(uintptr_t)ctx->dma_copy;
    if (ioctl(ctx->dev_fd, FPGA_DMA_READ_FRAME, &t) < 0) return -1;
    return t.result == 0 ? 0 : -1;
}

static int init_copy_slots(struct app_ctx *ctx)
{
    ctx->slots = calloc((size_t)ctx->opt.copy_buffers, sizeof(*ctx->slots));
    if (!ctx->slots) return -1;
    ctx->slot_count = ctx->opt.copy_buffers;
    for (int i = 0; i < ctx->slot_count; i++) {
        ctx->slots[i].data = malloc(ctx->frame_size);
        if (!ctx->slots[i].data) return -1;
    }
    return 0;
}

static void copy_frame_to_slot565(struct app_ctx *ctx, uint8_t *dst, const uint8_t *src)
{
    size_t pixels = (size_t)ctx->frame_width * ctx->frame_height;
    if (ctx->src_is_bgrx) {
        for (size_t i = 0; i < pixels; i++) {
            const uint8_t *p = src + i * 4U;
            uint8_t b = p[0], g = p[1], r = p[2];
            uint16_t pix565;
            if (ctx->opt.pixel_order == PIXEL_ORDER_BGR565)
                pix565 = (uint16_t)(((b >> 3) << 11) | ((g >> 2) << 5) | (r >> 3));
            else
                pix565 = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
            dst[i * 2U + 0] = (uint8_t)(pix565 & 0xFF);
            dst[i * 2U + 1] = (uint8_t)(pix565 >> 8);
        }
        return;
    }
    if (!ctx->opt.swap16) { memcpy(dst, src, ctx->frame_size); return; }
    for (size_t i = 0; i + 1 < ctx->frame_size; i += 2) {
        dst[i] = src[i + 1];
        dst[i + 1] = src[i];
    }
}

/* ---------------- slot / buffer helpers ---------------- */

static void release_slot_ticket(struct app_ctx *ctx, const struct slot_ticket *ticket, bool count_release)
{
    g_mutex_lock(&ctx->slots_lock);
    if (ticket->idx >= 0 && ticket->idx < ctx->slot_count &&
        ctx->slots[ticket->idx].in_use &&
        ctx->slots[ticket->idx].generation == ticket->generation) {
        ctx->slots[ticket->idx].in_use = false;
        if (count_release) ctx->released_frames++;
        g_cond_signal(&ctx->slots_cond);
    }
    g_mutex_unlock(&ctx->slots_lock);
}

static void frame_release_notify(gpointer user_data)
{
    struct frame_cookie *cookie = (struct frame_cookie *)user_data;
    release_slot_ticket(cookie->ctx, &cookie->ticket, true);
    g_free(cookie);
}

static int acquire_free_slot(struct app_ctx *ctx, struct slot_ticket *ticket)
{
    int64_t deadline_us = mono_us() + (int64_t)ctx->opt.timeout_ms * 1000LL;
    ticket->idx = -1;
    ticket->generation = 0;
    for (;;) {
        g_mutex_lock(&ctx->slots_lock);
        for (int i = 0; i < ctx->slot_count; i++) {
            if (!ctx->slots[i].in_use) {
                ctx->slots[i].in_use = true;
                ctx->slots[i].generation++;
                ticket->idx = i;
                ticket->generation = ctx->slots[i].generation;
                g_mutex_unlock(&ctx->slots_lock);
                return 0;
            }
        }
        int64_t now = mono_us();
        if (now >= deadline_us) {
            g_mutex_unlock(&ctx->slots_lock);
            ctx->slot_wait_timeout_count++;
            return -1;
        }
        int64_t wake_us = now + 20000;
        if (wake_us > deadline_us) wake_us = deadline_us;
        g_cond_wait_until(&ctx->slots_cond, &ctx->slots_lock, wake_us);
        g_mutex_unlock(&ctx->slots_lock);
    }
}

static GstBuffer *build_frame_buffer(struct app_ctx *ctx, const struct slot_ticket *ticket)
{
    struct frame_cookie *cookie = g_new0(struct frame_cookie, 1);
    if (!cookie) return NULL;
    cookie->ctx = ctx;
    cookie->ticket = *ticket;
    GstBuffer *buf = gst_buffer_new_wrapped_full((GstMemoryFlags)0,
                                                 ctx->slots[ticket->idx].data, ctx->frame_size,
                                                 0, ctx->frame_size, cookie, frame_release_notify);
    if (!buf) {
        release_slot_ticket(ctx, ticket, false);
        g_free(cookie);
        return NULL;
    }
    GST_BUFFER_PTS(buf) = ctx->next_pts_ns;
    GST_BUFFER_DURATION(buf) = (guint64)(GST_SECOND / ctx->opt.fps);
    ctx->next_pts_ns += GST_BUFFER_DURATION(buf);
    return buf;
}

static int handle_bus_messages(struct app_ctx *ctx)
{
    GstMessage *msg;
    while ((msg = gst_bus_pop(ctx->bus)) != NULL) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError *err = NULL;
            gchar *dbg = NULL;
            gst_message_parse_error(msg, &err, &dbg);
            fprintf(stderr, "GStreamer ERROR: %s\n", err ? err->message : "unknown");
            if (dbg) fprintf(stderr, "  debug: %s\n", dbg);
            if (err) g_error_free(err);
            g_free(dbg);
            ctx->running = false;
            gst_message_unref(msg);
            return -1;
        }
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
            ctx->running = false;
            gst_message_unref(msg);
            return -1;
        }
        gst_message_unref(msg);
    }
    return 0;
}

static int build_pipeline(struct app_ctx *ctx)
{
    const char *fmt = (ctx->opt.pixel_order == PIXEL_ORDER_BGR565) ? "BGR16" : "RGB16";
    ctx->pipeline = gst_pipeline_new("fpga-bird");
    ctx->appsrc = gst_element_factory_make("appsrc", "src");
    ctx->queue  = gst_element_factory_make("queue", "latency_queue");
    ctx->sink   = gst_element_factory_make("kmssink", "sink");
    if (!ctx->pipeline || !ctx->appsrc || !ctx->queue || !ctx->sink) return -1;

    gst_bin_add_many(GST_BIN(ctx->pipeline), ctx->appsrc, ctx->queue, ctx->sink, NULL);
    if (!gst_element_link_many(ctx->appsrc, ctx->queue, ctx->sink, NULL)) return -1;

    GstCaps *caps = gst_caps_new_simple("video/x-raw",
                                        "format",    G_TYPE_STRING, fmt,
                                        "width",     G_TYPE_INT, (int)ctx->frame_width,
                                        "height",    G_TYPE_INT, (int)ctx->frame_height,
                                        "framerate", GST_TYPE_FRACTION, ctx->opt.fps, 1,
                                        NULL);
    if (!caps) return -1;
    g_object_set(ctx->appsrc,
                 "caps", caps,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "format", GST_FORMAT_TIME,
                 "block", FALSE,
                 "max-bytes", (guint64)ctx->frame_size * (guint64)ctx->opt.queue_depth,
                 NULL);
    gst_caps_unref(caps);
    g_object_set(ctx->queue, "max-size-buffers", 1, "max-size-bytes", 0,
                 "max-size-time", (guint64)0, "leaky", 2, NULL);
    g_object_set(ctx->sink, "sync", FALSE, NULL);
    if (ctx->opt.connector_id >= 0)
        g_object_set(ctx->sink, "connector-id", ctx->opt.connector_id, NULL);
    if (ctx->drm_fd >= 0)
        g_object_set(ctx->sink, "fd", ctx->drm_fd, NULL);

    ctx->bus = gst_element_get_bus(ctx->pipeline);
    GstStateChangeReturn sret = gst_element_set_state(ctx->pipeline, GST_STATE_PLAYING);
    if (sret == GST_STATE_CHANGE_FAILURE) return -1;
    sret = gst_element_get_state(ctx->pipeline, NULL, NULL, 5 * GST_SECOND);
    if (sret == GST_STATE_CHANGE_FAILURE) return -1;
    fprintf(stderr, "Pipeline started: appsrc(%s)->queue(leaky)->kmssink\n", fmt);
    return 0;
}

/* ---------------- overlay drawing ---------------- */

static void draw_hline_565(uint16_t *pix, int w, int h, int x1, int x2, int y, uint16_t c)
{
    if (y < 0 || y >= h) return;
    if (x1 > x2) { int t = x1; x1 = x2; x2 = t; }
    if (x1 < 0) x1 = 0;
    if (x2 >= w) x2 = w - 1;
    for (int x = x1; x <= x2; x++) pix[y * w + x] = c;
}

static void draw_vline_565(uint16_t *pix, int w, int h, int x, int y1, int y2, uint16_t c)
{
    if (x < 0 || x >= w) return;
    if (y1 > y2) { int t = y1; y1 = y2; y2 = t; }
    if (y1 < 0) y1 = 0;
    if (y2 >= h) y2 = h - 1;
    for (int y = y1; y <= y2; y++) pix[y * w + x] = c;
}

static void draw_rect_565(uint16_t *pix, int w, int h, const struct det_box *b, uint16_t c)
{
    for (int t = 0; t < 2; t++) {
        draw_hline_565(pix, w, h, b->x1, b->x2, b->y1 + t, c);
        draw_hline_565(pix, w, h, b->x1, b->x2, b->y2 - t, c);
        draw_vline_565(pix, w, h, b->x1 + t, b->y1, b->y2, c);
        draw_vline_565(pix, w, h, b->x2 - t, b->y1, b->y2, c);
    }
}

static uint8_t glyph5x7(char ch, int row)
{
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
    switch (ch) {
    case '0': { static const uint8_t g[7]={0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; return g[row]; }
    case '1': { static const uint8_t g[7]={0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
    case '2': { static const uint8_t g[7]={0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}; return g[row]; }
    case '3': { static const uint8_t g[7]={0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case '4': { static const uint8_t g[7]={0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}; return g[row]; }
    case '5': { static const uint8_t g[7]={0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}; return g[row]; }
    case '6': { static const uint8_t g[7]={0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}; return g[row]; }
    case '7': { static const uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x08,0x08}; return g[row]; }
    case '8': { static const uint8_t g[7]={0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}; return g[row]; }
    case '9': { static const uint8_t g[7]={0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}; return g[row]; }
    case 'A': { static const uint8_t g[7]={0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'B': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}; return g[row]; }
    case 'C': { static const uint8_t g[7]={0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}; return g[row]; }
    case 'D': { static const uint8_t g[7]={0x1C,0x12,0x11,0x11,0x11,0x12,0x1C}; return g[row]; }
    case 'E': { static const uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}; return g[row]; }
    case 'F': { static const uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'G': { static const uint8_t g[7]={0x0F,0x10,0x10,0x13,0x11,0x11,0x0F}; return g[row]; }
    case 'H': { static const uint8_t g[7]={0x11,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'I': { static const uint8_t g[7]={0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
    case 'J': { static const uint8_t g[7]={0x01,0x01,0x01,0x01,0x11,0x11,0x0E}; return g[row]; }
    case 'K': { static const uint8_t g[7]={0x11,0x12,0x14,0x18,0x14,0x12,0x11}; return g[row]; }
    case 'L': { static const uint8_t g[7]={0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return g[row]; }
    case 'M': { static const uint8_t g[7]={0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; return g[row]; }
    case 'N': { static const uint8_t g[7]={0x11,0x19,0x15,0x13,0x11,0x11,0x11}; return g[row]; }
    case 'O': { static const uint8_t g[7]={0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'P': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'Q': { static const uint8_t g[7]={0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}; return g[row]; }
    case 'R': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return g[row]; }
    case 'S': { static const uint8_t g[7]={0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case 'T': { static const uint8_t g[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'U': { static const uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'V': { static const uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x0A,0x04}; return g[row]; }
    case 'W': { static const uint8_t g[7]={0x11,0x11,0x11,0x15,0x15,0x1B,0x11}; return g[row]; }
    case 'X': { static const uint8_t g[7]={0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}; return g[row]; }
    case 'Y': { static const uint8_t g[7]={0x11,0x11,0x0A,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'Z': { static const uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}; return g[row]; }
    case '.': { static const uint8_t g[7]={0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}; return g[row]; }
    case '-': { static const uint8_t g[7]={0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; return g[row]; }
    case ':': { static const uint8_t g[7]={0x00,0x04,0x04,0x00,0x04,0x04,0x00}; return g[row]; }
    case ' ': { static const uint8_t g[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00}; return g[row]; }
    default:  return 0;
    }
}

static void draw_text_565(uint16_t *pix, int w, int h, int x, int y,
                          const char *s, uint16_t c, int scale)
{
    if (!s || scale < 1) return;
    int advance = 6 * scale;
    for (int i = 0; s[i] != '\0'; i++) {
        int ox = x + i * advance;
        for (int row = 0; row < 7; row++) {
            uint8_t bits = glyph5x7(s[i], row);
            for (int col = 0; col < 5; col++) {
                if (!(bits & (1U << (4 - col)))) continue;
                int px0 = ox + col * scale;
                int py0 = y + row * scale;
                for (int sy = 0; sy < scale; sy++) {
                    int py = py0 + sy;
                    if (py < 0 || py >= h) continue;
                    for (int sx = 0; sx < scale; sx++) {
                        int px = px0 + sx;
                        if (px >= 0 && px < w) pix[py * w + px] = c;
                    }
                }
            }
        }
    }
}

/* ---------------- inference thread ---------------- */

static int run_bird_detect(struct app_ctx *ctx, const uint8_t *src_rgb, int src_w, int src_h,
                           float conf_thr, uint8_t *model_in,
                           struct det_box *out, int *out_count, struct letterbox_meta *lb)
{
    struct yolo_model *m = &ctx->bird_model;
    int in_w = (int)m->in_w;
    int in_h = (int)m->in_h;

    /* letterbox 1280x720 (or similar) -> in_w x in_h with pad_value=114 */
    resize_rgb888_letterbox_pad(src_rgb, src_w, src_h, model_in, in_w, in_h,
                                BIRD_PAD_VALUE, lb);

    rknn_input in;
    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf   = model_in;
    in.size  = (uint32_t)(in_w * in_h * 3);
    in.type  = RKNN_TENSOR_UINT8;
    in.fmt   = RKNN_TENSOR_NHWC;

    int ret = rknn_inputs_set(m->ctx, 1, &in);
    if (ret < 0) return ret;
    ret = rknn_run(m->ctx, NULL);
    if (ret < 0) return ret;

    rknn_output outs[8];
    memset(outs, 0, sizeof(outs));
    for (uint32_t i = 0; i < m->io_num.n_output; i++)
        outs[i].want_float = 1;
    ret = rknn_outputs_get(m->ctx, m->io_num.n_output, outs, NULL);
    if (ret < 0) return ret;

    ret = decode_yolov8_det_cls80(m, outs, conf_thr, ctx->opt.class_id,
                                  in_w, in_h,
                                  out, out_count, ctx->opt.nms_iou, ctx->opt.max_det);
    rknn_outputs_release(m->ctx, m->io_num.n_output, outs);

    if (ret == 0) {
        for (int i = 0; i < *out_count; i++)
            map_box_from_letterbox(&out[i], lb);
    }
    return ret;
}

static void *infer_thread_main(void *arg)
{
    struct app_ctx *ctx = (struct app_ctx *)arg;
    uint8_t *raw_local = malloc(ctx->src_frame_size);
    uint8_t *rgb_full  = malloc((size_t)ctx->frame_width * ctx->frame_height * 3U);
    int model_buf_sz = (int)(ctx->bird_model.in_w * ctx->bird_model.in_h * 3);
    uint8_t *model_in  = malloc((size_t)model_buf_sz);

    if (!raw_local || !rgb_full || !model_in) {
        free(raw_local); free(rgb_full); free(model_in);
        return NULL;
    }

    while (ctx->running) {
        struct det_box dets[MAX_DETS];
        int det_count = 0;
        struct letterbox_meta lb;
        uint64_t seq;

        pthread_mutex_lock(&ctx->infer_lock);
        while (ctx->running && !ctx->infer_has_new)
            pthread_cond_wait(&ctx->infer_cond, &ctx->infer_lock);
        if (!ctx->running) {
            pthread_mutex_unlock(&ctx->infer_lock);
            break;
        }
        memcpy(raw_local, ctx->infer_latest_raw, ctx->src_frame_size);
        seq = ctx->infer_frame_seq;
        ctx->infer_has_new = false;
        pthread_mutex_unlock(&ctx->infer_lock);

        int64_t t0 = mono_us();
        if (ctx->src_is_bgrx) {
            bgrx8888_to_rgb888(raw_local, (int)ctx->frame_width, (int)ctx->frame_height, rgb_full);
        } else {
            raw565_to_rgb888_full(ctx, raw_local, rgb_full);
        }

        if (run_bird_detect(ctx, rgb_full, (int)ctx->frame_width, (int)ctx->frame_height,
                            ctx->opt.min_conf, model_in, dets, &det_count, &lb) < 0) {
            det_count = 0;
        }
        int64_t t1 = mono_us();

        struct bird_results r;
        memset(&r, 0, sizeof(r));
        r.det_raw_count = det_count;
        for (int i = 0; i < det_count && r.det_count < MAX_DETS; i++)
            r.dets[r.det_count++] = dets[i];
        r.frame_seq = seq;
        r.infer_ms_last = (double)(t1 - t0) / 1000.0;

        pthread_mutex_lock(&ctx->result_lock);
        r.infer_frames_total = ctx->results.infer_frames_total + 1;
        r.infer_ms_total = ctx->results.infer_ms_total + r.infer_ms_last;
        ctx->results = r;
        pthread_mutex_unlock(&ctx->result_lock);

        if (det_count > 0) {
            fprintf(stderr, "[bird] frame=%" PRIu64 " n=%d ms=%.2f",
                    seq, det_count, r.infer_ms_last);
            for (int i = 0; i < det_count && i < 4; i++) {
                fprintf(stderr, " [%d:%.2f,%d,%d,%d,%d]",
                        dets[i].cls, dets[i].conf,
                        dets[i].x1, dets[i].y1, dets[i].x2, dets[i].y2);
            }
            fprintf(stderr, "\n");
        }
    }

    free(raw_local); free(rgb_full); free(model_in);
    return NULL;
}

static void push_latest_to_infer(struct app_ctx *ctx, const uint8_t *raw)
{
    pthread_mutex_lock(&ctx->infer_lock);
    if (ctx->infer_has_new) ctx->infer_overwrite_count++;
    memcpy(ctx->infer_latest_raw, raw, ctx->src_frame_size);
    ctx->infer_frame_seq++;
    ctx->infer_has_new = true;
    pthread_cond_signal(&ctx->infer_cond);
    pthread_mutex_unlock(&ctx->infer_lock);
}

static void overlay_results_on_slot(struct app_ctx *ctx, uint8_t *slot_data)
{
    struct bird_results r;
    uint16_t *pix = (uint16_t *)slot_data;
    pthread_mutex_lock(&ctx->result_lock);
    r = ctx->results;
    pthread_mutex_unlock(&ctx->result_lock);

    for (int i = 0; i < r.det_count; i++) {
        char txt[48];
        const struct det_box *b = &r.dets[i];
        int text_scale = OVERLAY_TEXT_SCALE;
        int text_h = 7 * text_scale;
        int ty = b->y1 - (text_h + 3);
        if (ty < 0) ty = b->y1 + 3;
        if (ty + text_h >= (int)ctx->frame_height)
            ty = (int)ctx->frame_height - text_h - 1;
        if (ty < 0) ty = 0;
        snprintf(txt, sizeof(txt), "BIRD %d%%", (int)(b->conf * 100.0f + 0.5f));
        draw_rect_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, b, COLOR_MAGENTA_565);
        draw_text_565(pix, (int)ctx->frame_width, (int)ctx->frame_height,
                      b->x1, ty, txt, COLOR_MAGENTA_565, text_scale);
    }
}

static void print_stats(struct app_ctx *ctx)
{
    int64_t now = mono_us();
    int64_t dt = now - ctx->last_stats_us;
    if (dt < (int64_t)ctx->opt.stats_interval * 1000000LL)
        return;
    struct bird_results r;
    pthread_mutex_lock(&ctx->result_lock);
    r = ctx->results;
    pthread_mutex_unlock(&ctx->result_lock);
    fprintf(stderr,
            "[stats] cap=%" PRIu64 " push=%" PRIu64 " rel=%" PRIu64
            " infer=%" PRIu64 " infer_ms=%.2f birds=%d(raw=%d) drop=%" PRIu64
            " cap_fps=%.2f disp_fps=%.2f infer_fps=%.2f\n",
            ctx->captured_frames, ctx->pushed_frames, ctx->released_frames,
            r.infer_frames_total, r.infer_ms_last,
            r.det_count, r.det_raw_count,
            ctx->infer_overwrite_count,
            (double)(ctx->captured_frames - ctx->last_stats_cap) * 1000000.0 / (double)dt,
            (double)(ctx->released_frames - ctx->last_stats_rel) * 1000000.0 / (double)dt,
            (double)(r.infer_frames_total - ctx->last_stats_infer) * 1000000.0 / (double)dt);
    ctx->last_stats_cap = ctx->captured_frames;
    ctx->last_stats_rel = ctx->released_frames;
    ctx->last_stats_infer = r.infer_frames_total;
    ctx->last_stats_us = now;
}

/* ---------------- options ---------------- */

static void print_usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s --bird-model <path> [OPTIONS]\n"
        "  --device <path>         FPGA device (default: %s)\n"
        "  --drm-card <path>       DRM card (default: %s)\n"
        "  --bird-model <path>     Bird (COCO YOLOv8n) RKNN model path [required]\n"
        "  --connector-id <id>     Optional KMS connector id\n"
        "  --fps <num>             Target FPS (default: %d)\n"
        "  --pixel-order <mode>    bgr565|rgb565 (default: bgr565)\n"
        "  --swap16 <0|1>          Swap bytes per 16-bit pixel (default: 1)\n"
        "  --timeout-ms <ms>       Frame timeout (default: %d)\n"
        "  --stats-interval <sec>  Stats print interval (default: %d)\n"
        "  --copy-buffers <num>    Copy ring size (default: %d)\n"
        "  --queue-depth <num>     appsrc queue depth (default: %d)\n"
        "  --min-conf <v>          Detection confidence threshold (default: 0.30)\n"
        "  --nms-iou <v>           NMS IoU threshold (default: 0.45)\n"
        "  --max-det <n>           Max detections after NMS (default: 128)\n"
        "  --class-id <n>          COCO class id to keep (-1=any, default: %d=bird)\n"
        "  --help                  Show this help\n",
        prog, DEFAULT_DEVICE, DEFAULT_DRM_CARD, DEFAULT_FPS, DEFAULT_TIMEOUT_MS,
        DEFAULT_STATS_INTERVAL, DEFAULT_COPY_BUFFERS, DEFAULT_QUEUE_DEPTH, BIRD_CLASS_ID);
}

static int parse_options(int argc, char **argv, struct options *opt)
{
    static const struct option long_opts[] = {
        {"device",        required_argument, NULL, 1},
        {"drm-card",      required_argument, NULL, 2},
        {"bird-model",    required_argument, NULL, 3},
        {"connector-id",  required_argument, NULL, 4},
        {"fps",           required_argument, NULL, 5},
        {"pixel-order",   required_argument, NULL, 6},
        {"swap16",        required_argument, NULL, 7},
        {"timeout-ms",    required_argument, NULL, 8},
        {"stats-interval",required_argument, NULL, 9},
        {"copy-buffers",  required_argument, NULL, 10},
        {"queue-depth",   required_argument, NULL, 11},
        {"min-conf",      required_argument, NULL, 12},
        {"nms-iou",       required_argument, NULL, 13},
        {"max-det",       required_argument, NULL, 14},
        {"class-id",      required_argument, NULL, 15},
        {"help",          no_argument,       NULL, 'h'},
        {0, 0, 0, 0}
    };

    memset(opt, 0, sizeof(*opt));
    opt->device_path    = DEFAULT_DEVICE;
    opt->drm_card_path  = DEFAULT_DRM_CARD;
    opt->bird_model_path = NULL;
    opt->connector_id   = -1;
    opt->fps            = DEFAULT_FPS;
    opt->pixel_order    = PIXEL_ORDER_BGR565;
    opt->timeout_ms     = DEFAULT_TIMEOUT_MS;
    opt->stats_interval = DEFAULT_STATS_INTERVAL;
    opt->copy_buffers   = DEFAULT_COPY_BUFFERS;
    opt->queue_depth    = DEFAULT_QUEUE_DEPTH;
    opt->min_conf       = 0.30f;
    opt->nms_iou        = 0.45f;
    opt->max_det        = MAX_DETS;
    opt->class_id       = BIRD_CLASS_ID;
    opt->swap16         = true;

    int c;
    while ((c = getopt_long(argc, argv, "h", long_opts, NULL)) != -1) {
        switch (c) {
        case 1:  opt->device_path    = optarg; break;
        case 2:  opt->drm_card_path  = optarg; break;
        case 3:  opt->bird_model_path = optarg; break;
        case 4:  opt->connector_id   = atoi(optarg); break;
        case 5:  opt->fps            = atoi(optarg); break;
        case 6:
            if (strcmp(optarg, "bgr565") == 0) opt->pixel_order = PIXEL_ORDER_BGR565;
            else if (strcmp(optarg, "rgb565") == 0) opt->pixel_order = PIXEL_ORDER_RGB565;
            else return -1;
            break;
        case 7:  opt->swap16 = atoi(optarg) ? true : false; break;
        case 8:  opt->timeout_ms     = atoi(optarg); break;
        case 9:  opt->stats_interval = atoi(optarg); break;
        case 10: opt->copy_buffers   = atoi(optarg); break;
        case 11: opt->queue_depth    = atoi(optarg); break;
        case 12: opt->min_conf       = (float)atof(optarg); break;
        case 13: opt->nms_iou        = (float)atof(optarg); break;
        case 14: opt->max_det        = atoi(optarg); break;
        case 15: opt->class_id       = atoi(optarg); break;
        case 'h':
        default: return -1;
        }
    }

    if (!opt->bird_model_path || opt->bird_model_path[0] == '\0') {
        fprintf(stderr, "Missing required --bird-model\n");
        return -1;
    }
    if (opt->copy_buffers < MIN_COPY_BUFFERS) opt->copy_buffers = MIN_COPY_BUFFERS;
    if (opt->copy_buffers > MAX_COPY_BUFFERS) opt->copy_buffers = MAX_COPY_BUFFERS;
    if (opt->fps <= 0) opt->fps = DEFAULT_FPS;
    if (opt->queue_depth < 1) opt->queue_depth = 1;
    if (opt->max_det <= 0) opt->max_det = MAX_DETS;
    if (opt->nms_iou <= 0.0f) opt->nms_iou = 0.45f;
    return 0;
}

/* ---------------- cleanup / main ---------------- */

static void cleanup(struct app_ctx *ctx)
{
    ctx->running = false;
    pthread_mutex_lock(&ctx->infer_lock);
    pthread_cond_broadcast(&ctx->infer_cond);
    pthread_mutex_unlock(&ctx->infer_lock);
    if (ctx->infer_thread)
        pthread_join(ctx->infer_thread, NULL);

    rknn_model_release(&ctx->bird_model);

    if (ctx->appsrc)
        gst_app_src_end_of_stream(GST_APP_SRC(ctx->appsrc));
    if (ctx->pipeline) {
        gst_element_set_state(ctx->pipeline, GST_STATE_NULL);
        gst_object_unref(ctx->pipeline);
    }
    if (ctx->bus)
        gst_object_unref(ctx->bus);

    if (ctx->slots) {
        for (int i = 0; i < ctx->slot_count; i++)
            free(ctx->slots[i].data);
        free(ctx->slots);
    }
    if (ctx->dma_map && ctx->dma_map != MAP_FAILED)
        munmap(ctx->dma_map, ctx->dma_map_size);
    if (ctx->dev_fd >= 0) close(ctx->dev_fd);
    if (ctx->drm_fd >= 0) close(ctx->drm_fd);
    free(ctx->dma_copy);
    free(ctx->infer_latest_raw);

    pthread_mutex_destroy(&ctx->infer_lock);
    pthread_cond_destroy(&ctx->infer_cond);
    pthread_mutex_destroy(&ctx->result_lock);
    g_mutex_clear(&ctx->slots_lock);
    g_cond_clear(&ctx->slots_cond);
}

int main(int argc, char **argv)
{
    struct app_ctx ctx;
    int ret = 1;

    memset(&ctx, 0, sizeof(ctx));
    ctx.dev_fd = -1;
    ctx.drm_fd = -1;
    ctx.running = true;

    g_mutex_init(&ctx.slots_lock);
    g_cond_init(&ctx.slots_cond);
    pthread_mutex_init(&ctx.infer_lock, NULL);
    pthread_cond_init(&ctx.infer_cond, NULL);
    pthread_mutex_init(&ctx.result_lock, NULL);

    if (parse_options(argc, argv, &ctx.opt) < 0) {
        print_usage(argv[0]);
        goto out;
    }

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    gst_init(&argc, &argv);

    ctx.drm_fd = open(ctx.opt.drm_card_path, O_RDWR | O_CLOEXEC);
    if (ctx.drm_fd < 0) {
        fprintf(stderr, "Open DRM card failed: %s\n", strerror(errno));
        goto out;
    }
    if (init_fpga_dma(&ctx) < 0) {
        fprintf(stderr, "FPGA DMA init failed\n");
        goto out;
    }
    if (init_copy_slots(&ctx) < 0) {
        fprintf(stderr, "Alloc copy slots failed\n");
        goto out;
    }
    if (rknn_model_load(&ctx.bird_model, "bird", ctx.opt.bird_model_path) < 0) {
        fprintf(stderr, "Load bird model failed: %s\n", ctx.opt.bird_model_path);
        goto out;
    }

    ctx.infer_latest_raw = malloc(ctx.src_frame_size);
    if (!ctx.infer_latest_raw) goto out;
    if (build_pipeline(&ctx) < 0) {
        fprintf(stderr, "Build GStreamer pipeline failed\n");
        goto out;
    }
    if (pthread_create(&ctx.infer_thread, NULL, infer_thread_main, &ctx) != 0) {
        fprintf(stderr, "Start infer thread failed\n");
        goto out;
    }

    fprintf(stderr,
            "Start bird loop: fps=%d src=%s pixel=%s swap16=%s min_conf=%.2f nms_iou=%.2f max_det=%d class=%d\n",
            ctx.opt.fps,
            ctx.src_is_bgrx ? "bgrx8888" : "bgr565",
            (ctx.opt.pixel_order == PIXEL_ORDER_BGR565) ? "bgr565" : "rgb565",
            ctx.opt.swap16 ? "on" : "off",
            ctx.opt.min_conf, ctx.opt.nms_iou, ctx.opt.max_det, ctx.opt.class_id);

    ctx.last_stats_us = mono_us();

    while (ctx.running) {
        struct slot_ticket ticket;
        GstBuffer *buf;
        GstFlowReturn flow;
        int64_t t0 = mono_us();
        int64_t target_us = 1000000LL / ctx.opt.fps;

        if (g_stop) ctx.running = false;
        if (!ctx.running) break;
        if (handle_bus_messages(&ctx) < 0) break;

        if (trigger_frame_dma(&ctx) < 0) break;
        ctx.captured_frames++;

        if (acquire_free_slot(&ctx, &ticket) < 0) break;
        copy_frame_to_slot565(&ctx, ctx.slots[ticket.idx].data, ctx.dma_copy);
        push_latest_to_infer(&ctx, ctx.dma_copy);
        overlay_results_on_slot(&ctx, ctx.slots[ticket.idx].data);

        buf = build_frame_buffer(&ctx, &ticket);
        if (!buf) break;
        flow = gst_app_src_push_buffer(GST_APP_SRC(ctx.appsrc), buf);
        if (flow != GST_FLOW_OK) {
            release_slot_ticket(&ctx, &ticket, false);
            break;
        }
        ctx.pushed_frames++;
        print_stats(&ctx);

        int64_t loop_us = mono_us() - t0;
        if (loop_us < target_us)
            usleep((useconds_t)(target_us - loop_us));
    }

    fprintf(stderr, "Exit: cap=%" PRIu64 " push=%" PRIu64 " rel=%" PRIu64 "\n",
            ctx.captured_frames, ctx.pushed_frames, ctx.released_frames);
    ret = 0;
out:
    cleanup(&ctx);
    return ret;
}
