// SPDX-License-Identifier: GPL-2.0
/*
 * Standalone green PPLCNet live validation driver.
 *
 * Pipeline:
 *   FPGA DMA frame -> YOLOv8n-pose plate quad -> quad warp crop -> green PPLCNet CTC.
 * This program is intentionally separate from fpga_lpr_display.c and loads only one OCR model.
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <rknn_api.h>

#include "ocr_decode.h"
#include "pcie_fpga_dma.h"

#define DEFAULT_DEVICE "/dev/" FPGA_DMA_DEV_NAME
#define ALGO_STREAM_SIZE 640
#define OBB_POINT_COUNT 8400
#define POSE_KPT_COUNT 4
#define POSE_KPT_DIMS 3
#define POSE_BOX_CHANNELS 4
#define POSE_KPT_CHANNELS (POSE_KPT_COUNT * POSE_KPT_DIMS)
#define POSE_MIN_CHANNELS (POSE_BOX_CHANNELS + 1 + POSE_KPT_CHANNELS)
#define MAX_DETS 128
#define MAX_OCR_KEYS 128
#define MAX_OCR_KEY_LEN 16

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum pixel_order {
    PIXEL_ORDER_BGR565 = 0,
    PIXEL_ORDER_RGB565,
};

enum det_resize_mode {
    DET_RESIZE_STRETCH = 0,
    DET_RESIZE_LETTERBOX,
};

enum ocr_preproc_mode {
    OCR_PREPROC_NONE = 0,
    OCR_PREPROC_GRAY,
    OCR_PREPROC_BIN,
};

struct live_options {
    const char *device_path;
    const char *plate_model_path;
    const char *ocr_model_path;
    const char *keys_path;
    int frames;
    int fps;
    float min_conf;
    float nms_iou;
    int max_det;
    int class_filter;
    bool auto_green_filter;
    enum det_resize_mode det_resize_mode;
    enum ocr_preproc_mode ocr_preproc_mode;
    enum pixel_order pixel_order;
    bool swap16;
};

struct det_box {
    int x1;
    int y1;
    int x2;
    int y2;
    float conf;
    int cls;
    float quad[8];
};

struct rknn_model {
    const char *name;
    const char *path;
    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr input_attr;
    rknn_tensor_attr output_attrs[8];
    uint32_t in_w;
    uint32_t in_h;
    uint32_t in_c;
};

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

struct dma_state {
    int fd;
    void *map;
    size_t map_size;
    uint8_t *copy;
    uint32_t frame_w;
    uint32_t frame_h;
    uint32_t frame_bpp;
    size_t frame_size;
    bool src_is_bgrx;
};

struct ocr_keys {
    char keys[MAX_OCR_KEYS][MAX_OCR_KEY_LEN];
    int count;
};

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static int64_t mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
}

static float sigmoidf_local(float x)
{
    if (x >= 0.0f) {
        float z = expf(-x);
        return 1.0f / (1.0f + z);
    }
    float z = expf(x);
    return z / (1.0f + z);
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s --plate-model yolov8n_pos.rknn --ocr-green-model green.rknn --ocr-keys keys.txt [opts]\n"
            "\n"
            "Required:\n"
            "  --plate-model <path>       YOLOv8n pose RKNN plate detector\n"
            "  --ocr-green-model <path>   Green PPLCNet OCR RKNN\n"
            "  --ocr-keys <path>          OCR keys file\n"
            "\n"
            "Options:\n"
            "  --device <path>            FPGA DMA device (default: /dev/fpga_dma0)\n"
            "  --frames <n>               Frames to process, 0 means forever (default: 0)\n"
            "  --fps <n>                  Capture throttle FPS (default: 10)\n"
            "  --min-plate-conf <v>       Detector threshold (default: 0.50)\n"
            "  --plate-nms-iou <v>        NMS IoU (default: 0.45)\n"
            "  --plate-max-det <n>        Max detections per frame (default: 8)\n"
            "  --class-filter <id>        Detector class filter; auto green class when pose_nc>=5\n"
            "  --det-resize <stretch|letterbox> Detector mapping (default: letterbox)\n"
            "  --ocr-preproc <none|gray|bin> OCR preproc (default: gray)\n"
            "  --pixel-order <bgr565|rgb565> Raw 565 order (default: bgr565)\n"
            "  --swap16 <0|1>             Swap raw 565 bytes (default: 0)\n",
            prog);
}

static void defaults(struct live_options *o)
{
    memset(o, 0, sizeof(*o));
    o->device_path = DEFAULT_DEVICE;
    o->frames = 0;
    o->fps = 10;
    o->min_conf = 0.50f;
    o->nms_iou = 0.45f;
    o->max_det = 8;
    o->class_filter = -1;
    o->auto_green_filter = true;
    o->det_resize_mode = DET_RESIZE_LETTERBOX;
    o->ocr_preproc_mode = OCR_PREPROC_GRAY;
    o->pixel_order = PIXEL_ORDER_BGR565;
    o->swap16 = false;
}

static int parse_options(int argc, char **argv, struct live_options *o)
{
    static const struct option opts[] = {
        {"device", required_argument, NULL, 1},
        {"plate-model", required_argument, NULL, 2},
        {"ocr-green-model", required_argument, NULL, 3},
        {"ocr-keys", required_argument, NULL, 4},
        {"frames", required_argument, NULL, 5},
        {"fps", required_argument, NULL, 6},
        {"min-plate-conf", required_argument, NULL, 7},
        {"plate-nms-iou", required_argument, NULL, 8},
        {"plate-max-det", required_argument, NULL, 9},
        {"class-filter", required_argument, NULL, 10},
        {"det-resize", required_argument, NULL, 11},
        {"ocr-preproc", required_argument, NULL, 12},
        {"pixel-order", required_argument, NULL, 13},
        {"swap16", required_argument, NULL, 14},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0},
    };
    int c;
    defaults(o);
    while ((c = getopt_long(argc, argv, "h", opts, NULL)) != -1) {
        switch (c) {
        case 1: o->device_path = optarg; break;
        case 2: o->plate_model_path = optarg; break;
        case 3: o->ocr_model_path = optarg; break;
        case 4: o->keys_path = optarg; break;
        case 5: o->frames = atoi(optarg); break;
        case 6: o->fps = atoi(optarg); break;
        case 7: o->min_conf = strtof(optarg, NULL); break;
        case 8: o->nms_iou = strtof(optarg, NULL); break;
        case 9: o->max_det = atoi(optarg); break;
        case 10:
            o->class_filter = atoi(optarg);
            o->auto_green_filter = false;
            break;
        case 11:
            if (strcmp(optarg, "stretch") == 0) o->det_resize_mode = DET_RESIZE_STRETCH;
            else if (strcmp(optarg, "letterbox") == 0) o->det_resize_mode = DET_RESIZE_LETTERBOX;
            else return -1;
            break;
        case 12:
            if (strcmp(optarg, "none") == 0) o->ocr_preproc_mode = OCR_PREPROC_NONE;
            else if (strcmp(optarg, "gray") == 0 || strcmp(optarg, "gray3") == 0) o->ocr_preproc_mode = OCR_PREPROC_GRAY;
            else if (strcmp(optarg, "bin") == 0) o->ocr_preproc_mode = OCR_PREPROC_BIN;
            else return -1;
            break;
        case 13:
            if (strcmp(optarg, "bgr565") == 0) o->pixel_order = PIXEL_ORDER_BGR565;
            else if (strcmp(optarg, "rgb565") == 0) o->pixel_order = PIXEL_ORDER_RGB565;
            else return -1;
            break;
        case 14:
            o->swap16 = (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0 || strcmp(optarg, "on") == 0);
            break;
        case 'h': return 1;
        default: return -1;
        }
    }
    if (!o->plate_model_path || !o->ocr_model_path || !o->keys_path)
        return -1;
    if (o->fps <= 0 || o->fps > 120 || o->frames < 0 || o->max_det <= 0 || o->max_det > MAX_DETS)
        return -1;
    return 0;
}

static int load_file(const char *path, void **data_out, uint32_t *size_out)
{
    FILE *fp;
    long sz;
    void *data;
    fp = fopen(path, "rb");
    if (!fp)
        return -1;
    if (fseek(fp, 0, SEEK_END) < 0) { fclose(fp); return -1; }
    sz = ftell(fp);
    if (sz <= 0) { fclose(fp); return -1; }
    rewind(fp);
    data = malloc((size_t)sz);
    if (!data) { fclose(fp); return -1; }
    if (fread(data, 1, (size_t)sz, fp) != (size_t)sz) {
        free(data);
        fclose(fp);
        return -1;
    }
    fclose(fp);
    *data_out = data;
    *size_out = (uint32_t)sz;
    return 0;
}

static int load_keys(const char *path, struct ocr_keys *keys)
{
    FILE *fp;
    char line[256];
    int n = 0;
    memset(keys, 0, sizeof(*keys));
    fp = fopen(path, "r");
    if (!fp)
        return -1;
    while (fgets(line, sizeof(line), fp) && n < MAX_OCR_KEYS) {
        char *s = line;
        char *nl;
        size_t len;
        nl = strchr(s, '\n'); if (nl) *nl = '\0';
        nl = strchr(s, '\r'); if (nl) *nl = '\0';
        if ((unsigned char)s[0] == 0xEF && (unsigned char)s[1] == 0xBB && (unsigned char)s[2] == 0xBF)
            s += 3;
        if (s[0] == '\0' || s[0] == '#')
            continue;
        len = strnlen(s, MAX_OCR_KEY_LEN - 1);
        memcpy(keys->keys[n], s, len);
        keys->keys[n][len] = '\0';
        n++;
    }
    fclose(fp);
    keys->count = n;
    return n > 0 ? 0 : -1;
}

static const char *tensor_fmt_name(int fmt)
{
    return fmt == RKNN_TENSOR_NCHW ? "NCHW" : (fmt == RKNN_TENSOR_NHWC ? "NHWC" : "OTHER");
}

static int model_load(struct rknn_model *m, const char *name, const char *path)
{
    void *data = NULL;
    uint32_t sz = 0;
    uint32_t i;
    memset(m, 0, sizeof(*m));
    m->name = name;
    m->path = path;
    if (load_file(path, &data, &sz) < 0)
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

static void model_release(struct rknn_model *m)
{
    if (m && m->ctx)
        rknn_destroy(m->ctx);
    if (m)
        memset(m, 0, sizeof(*m));
}

static int init_dma(struct dma_state *d, const struct live_options *opt)
{
    struct fpga_info info;
    struct buffer_map map;
    uint32_t fmt;
    memset(d, 0, sizeof(*d));
    d->fd = -1;
    d->fd = open(opt->device_path, O_RDWR | O_CLOEXEC);
    if (d->fd < 0)
        return -1;
    if (ioctl(d->fd, FPGA_DMA_GET_INFO, &info) < 0)
        return -1;
    fmt = info.pixel_format;
    if (fmt != FPGA_PIXEL_FORMAT_BGR565 && fmt != FPGA_PIXEL_FORMAT_BGRX8888)
        fmt = (info.frame_bpp == 4) ? FPGA_PIXEL_FORMAT_BGRX8888 : FPGA_PIXEL_FORMAT_BGR565;
    info.frame_bpp = (fmt == FPGA_PIXEL_FORMAT_BGRX8888) ? 4 : 2;
    d->frame_w = info.frame_width;
    d->frame_h = info.frame_height;
    d->frame_bpp = info.frame_bpp;
    d->src_is_bgrx = fmt == FPGA_PIXEL_FORMAT_BGRX8888;
    d->frame_size = (size_t)d->frame_w * d->frame_h * d->frame_bpp;
    memset(&map, 0, sizeof(map));
    map.index = 0;
    if (ioctl(d->fd, FPGA_DMA_MAP_BUFFER, &map) < 0)
        return -1;
    if (map.size < d->frame_size)
        return -1;
    d->map_size = map.size;
    d->map = mmap(NULL, d->map_size, PROT_READ, MAP_SHARED, d->fd, 0);
    if (d->map == MAP_FAILED) {
        d->map = NULL;
        return -1;
    }
    d->copy = malloc(d->frame_size);
    if (!d->copy)
        return -1;
    return 0;
}

static void release_dma(struct dma_state *d)
{
    if (d->map)
        munmap(d->map, d->map_size);
    free(d->copy);
    if (d->fd >= 0)
        close(d->fd);
    memset(d, 0, sizeof(*d));
    d->fd = -1;
}

static int read_frame(struct dma_state *d)
{
    struct dma_transfer t;
    memset(&t, 0, sizeof(t));
    t.size = (uint32_t)d->frame_size;
    t.user_buf = (uint64_t)(uintptr_t)d->copy;
    if (ioctl(d->fd, FPGA_DMA_READ_FRAME, &t) < 0)
        return -1;
    return t.result == 0 ? 0 : -1;
}

static void decode_pixel565(enum pixel_order order, bool swap16, uint8_t lo_in, uint8_t hi_in,
                            uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t lo = swap16 ? hi_in : lo_in;
    uint8_t hi = swap16 ? lo_in : hi_in;
    uint16_t v = (uint16_t)lo | ((uint16_t)hi << 8);
    uint8_t c0 = (uint8_t)((v >> 11) & 0x1F);
    uint8_t c1 = (uint8_t)((v >> 5) & 0x3F);
    uint8_t c2 = (uint8_t)(v & 0x1F);
    if (order == PIXEL_ORDER_BGR565) {
        *b = (uint8_t)((c0 << 3) | (c0 >> 2));
        *g = (uint8_t)((c1 << 2) | (c1 >> 4));
        *r = (uint8_t)((c2 << 3) | (c2 >> 2));
    } else {
        *r = (uint8_t)((c0 << 3) | (c0 >> 2));
        *g = (uint8_t)((c1 << 2) | (c1 >> 4));
        *b = (uint8_t)((c2 << 3) | (c2 >> 2));
    }
}

static void frame_to_rgb888(const struct dma_state *d, const struct live_options *opt, uint8_t *rgb)
{
    size_t pixels = (size_t)d->frame_w * d->frame_h;
    size_t i;
    if (d->src_is_bgrx) {
        for (i = 0; i < pixels; i++) {
            const uint8_t *p = d->copy + i * 4U;
            rgb[i * 3U + 0] = p[2];
            rgb[i * 3U + 1] = p[1];
            rgb[i * 3U + 2] = p[0];
        }
        return;
    }
    for (i = 0; i < pixels; i++) {
        uint8_t r, g, b;
        decode_pixel565(opt->pixel_order, opt->swap16, d->copy[i * 2U], d->copy[i * 2U + 1], &r, &g, &b);
        rgb[i * 3U + 0] = r;
        rgb[i * 3U + 1] = g;
        rgb[i * 3U + 2] = b;
    }
}

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

static int detect_pose_nc_from_attrs(const struct rknn_model *m)
{
    uint32_t i;
    for (i = 0; i < m->io_num.n_output; i++) {
        struct tensor_cn_view tv;
        if (build_tensor_cn_view(&m->output_attrs[i], NULL, &tv) && tv.n == OBB_POINT_COUNT && tv.c >= POSE_MIN_CHANNELS)
            return tv.c - POSE_BOX_CHANNELS - POSE_KPT_CHANNELS;
    }
    return 0;
}

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
            best_score = sigmoidf_local(best_score);
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

static int run_detector(struct rknn_model *m, const uint8_t *rgb, int img_w, int img_h,
                        uint8_t *input, enum det_resize_mode resize_mode,
                        int pose_nc, int class_filter, float conf_thr, float nms_iou, int max_det,
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

static void order_quad(const float in[8], float out[8])
{
    float cx = 0.0f, cy = 0.0f;
    int tl = 0, tr = 0, br = 0, bl = 0;
    float best_tl = 1e30f, best_tr = 1e30f, best_br = -1e30f, best_bl = 1e30f;
    for (int i = 0; i < 4; i++) { cx += in[i * 2]; cy += in[i * 2 + 1]; }
    cx *= 0.25f; cy *= 0.25f;
    (void)cx; (void)cy;
    for (int i = 0; i < 4; i++) {
        float x = in[i * 2], y = in[i * 2 + 1];
        float sum = x + y;
        float diff = y - x;
        if (sum < best_tl) { best_tl = sum; tl = i; }
        if (diff < best_tr) { best_tr = diff; tr = i; }
        if (sum > best_br) { best_br = sum; br = i; }
        if (diff > best_bl) { best_bl = diff; bl = i; }
    }
    out[0] = in[tl * 2]; out[1] = in[tl * 2 + 1];
    out[2] = in[tr * 2]; out[3] = in[tr * 2 + 1];
    out[4] = in[br * 2]; out[5] = in[br * 2 + 1];
    out[6] = in[bl * 2]; out[7] = in[bl * 2 + 1];
}

static void sample_bilinear(const uint8_t *rgb, int w, int h, float x, float y, uint8_t p[3])
{
    int x0, y0, x1, y1;
    float wx, wy;
    if (x < 0.0f) x = 0.0f;
    if (y < 0.0f) y = 0.0f;
    if (x > (float)(w - 1)) x = (float)(w - 1);
    if (y > (float)(h - 1)) y = (float)(h - 1);
    x0 = (int)floorf(x); y0 = (int)floorf(y);
    x1 = x0 + 1; y1 = y0 + 1;
    if (x1 >= w) x1 = w - 1;
    if (y1 >= h) y1 = h - 1;
    wx = x - (float)x0; wy = y - (float)y0;
    for (int c = 0; c < 3; c++) {
        float p00 = rgb[((size_t)y0 * w + x0) * 3U + c];
        float p01 = rgb[((size_t)y0 * w + x1) * 3U + c];
        float p10 = rgb[((size_t)y1 * w + x0) * 3U + c];
        float p11 = rgb[((size_t)y1 * w + x1) * 3U + c];
        float v0 = p00 * (1.0f - wx) + p01 * wx;
        float v1 = p10 * (1.0f - wx) + p11 * wx;
        int iv = (int)(v0 * (1.0f - wy) + v1 * wy + 0.5f);
        if (iv < 0) iv = 0;
        if (iv > 255) iv = 255;
        p[c] = (uint8_t)iv;
    }
}

static bool warp_quad_piecewise(const uint8_t *rgb, int img_w, int img_h, const float quad_in[8],
                                uint8_t *dst, int cap_w, int cap_h, int *out_w, int *out_h)
{
    float q[8];
    float top_w, bot_w, left_h, right_h;
    int dw, dh;
    order_quad(quad_in, q);
    top_w = hypotf(q[2] - q[0], q[3] - q[1]);
    bot_w = hypotf(q[4] - q[6], q[5] - q[7]);
    left_h = hypotf(q[6] - q[0], q[7] - q[1]);
    right_h = hypotf(q[4] - q[2], q[5] - q[3]);
    dw = (int)(fmaxf(top_w, bot_w) + 0.5f);
    dh = (int)(fmaxf(left_h, right_h) + 0.5f);
    if (dw < 1) dw = 1;
    if (dh < 1) dh = 1;
    if (dw > cap_w) dw = cap_w;
    if (dh > cap_h) dh = cap_h;
    if (dw <= 0 || dh <= 0)
        return false;
    for (int y = 0; y < dh; y++) {
        float ty = dh > 1 ? (float)y / (float)(dh - 1) : 0.0f;
        float lx = q[0] + (q[6] - q[0]) * ty;
        float ly = q[1] + (q[7] - q[1]) * ty;
        float rx = q[2] + (q[4] - q[2]) * ty;
        float ry = q[3] + (q[5] - q[3]) * ty;
        for (int x = 0; x < dw; x++) {
            float tx = dw > 1 ? (float)x / (float)(dw - 1) : 0.0f;
            uint8_t pix[3];
            sample_bilinear(rgb, img_w, img_h, lx + (rx - lx) * tx, ly + (ry - ly) * tx, pix);
            memcpy(dst + ((size_t)y * dw + x) * 3U, pix, 3);
        }
    }
    *out_w = dw;
    *out_h = dh;
    return true;
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

static int run_ocr(struct rknn_model *m, const struct ocr_keys *keys,
                   enum ocr_preproc_mode preproc,
                   const uint8_t *crop_rgb, int crop_w, int crop_h,
                   char *text, size_t text_len, float *conf, struct ocr_decode_diag *diag)
{
    uint8_t *input;
    rknn_input in;
    rknn_output out;
    int ret;
    int t_size = 0, c_size = 0, t_stride = 0, c_stride = 0;
    int blank;
    input = malloc((size_t)m->in_w * m->in_h * 3U);
    if (!input)
        return -1;
    resize_bilinear(crop_rgb, crop_w, crop_h, input, (int)m->in_w, (int)m->in_h);
    rgb_to_bgr_inplace(input, (int)m->in_w, (int)m->in_h);
    if (preproc == OCR_PREPROC_GRAY)
        apply_gray3(input, (int)m->in_w, (int)m->in_h);
    else if (preproc == OCR_PREPROC_BIN)
        apply_bin(input, (int)m->in_w, (int)m->in_h);
    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf = input;
    in.size = m->in_w * m->in_h * 3U;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    ret = rknn_inputs_set(m->ctx, 1, &in);
    if (ret < 0) { free(input); return ret; }
    ret = rknn_run(m->ctx, NULL);
    if (ret < 0) { free(input); return ret; }
    memset(&out, 0, sizeof(out));
    out.want_float = 1;
    ret = rknn_outputs_get(m->ctx, 1, &out, NULL);
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
                                key_ptrs, keys->count, blank, OCR_DECODE_FAMILY_GREEN8,
                                text, text_len, conf, diag);
    }
    rknn_outputs_release(m->ctx, 1, &out);
    return ret;
}

static int pick_best(const struct det_box *dets, int count)
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

int main(int argc, char **argv)
{
    struct live_options opt;
    struct dma_state dma;
    struct rknn_model det_model;
    struct rknn_model ocr_model;
    struct ocr_keys keys;
    uint8_t *rgb = NULL;
    uint8_t *det_input = NULL;
    uint8_t *crop = NULL;
    int pose_nc;
    int class_filter;
    int ret = 1;
    int parsed;
    int64_t target_us;

    parsed = parse_options(argc, argv, &opt);
    if (parsed != 0) {
        usage(argv[0]);
        return parsed > 0 ? 0 : 1;
    }

    memset(&dma, 0, sizeof(dma)); dma.fd = -1;
    memset(&det_model, 0, sizeof(det_model));
    memset(&ocr_model, 0, sizeof(ocr_model));
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (load_keys(opt.keys_path, &keys) < 0) {
        fprintf(stderr, "[green-live] failed to load keys: %s\n", opt.keys_path);
        goto out;
    }
    if (init_dma(&dma, &opt) < 0) {
        fprintf(stderr, "[green-live] failed to init DMA: %s: %s\n", opt.device_path, strerror(errno));
        goto out;
    }
    if (model_load(&det_model, "yolov8n_pose", opt.plate_model_path) < 0) {
        fprintf(stderr, "[green-live] failed to load detector: %s\n", opt.plate_model_path);
        goto out;
    }
    if (model_load(&ocr_model, "pplcnet_green", opt.ocr_model_path) < 0) {
        fprintf(stderr, "[green-live] failed to load OCR: %s\n", opt.ocr_model_path);
        goto out;
    }
    if (det_model.in_w != ALGO_STREAM_SIZE || det_model.in_h != ALGO_STREAM_SIZE || det_model.in_c != 3) {
        fprintf(stderr, "[green-live] detector input must be 640x640x3, got %ux%ux%u\n",
                det_model.in_w, det_model.in_h, det_model.in_c);
        goto out;
    }
    if (ocr_model.in_c != 3) {
        fprintf(stderr, "[green-live] OCR input must have 3 channels, got %u\n", ocr_model.in_c);
        goto out;
    }

    pose_nc = detect_pose_nc_from_attrs(&det_model);
    class_filter = opt.class_filter;
    if (opt.auto_green_filter && pose_nc >= 5)
        class_filter = 1;

    rgb = malloc((size_t)dma.frame_w * dma.frame_h * 3U);
    det_input = malloc((size_t)det_model.in_w * det_model.in_h * 3U);
    crop = malloc((size_t)dma.frame_w * dma.frame_h * 3U);
    if (!rgb || !det_input || !crop)
        goto out;

    fprintf(stderr,
            "[green-live] start frame=%ux%u src=%s frames=%d fps=%d pose_nc=%d class_filter=%d "
            "det_resize=%s ocr=%ux%u preproc=%s\n",
            dma.frame_w, dma.frame_h, dma.src_is_bgrx ? "bgrx8888" : "bgr565",
            opt.frames, opt.fps, pose_nc, class_filter,
            opt.det_resize_mode == DET_RESIZE_LETTERBOX ? "letterbox" : "stretch",
            ocr_model.in_w, ocr_model.in_h,
            opt.ocr_preproc_mode == OCR_PREPROC_GRAY ? "gray" : (opt.ocr_preproc_mode == OCR_PREPROC_BIN ? "bin" : "none"));

    target_us = 1000000LL / opt.fps;
    for (int frame = 0; !g_stop && (opt.frames == 0 || frame < opt.frames); frame++) {
        struct det_box dets[MAX_DETS];
        int det_count = 0;
        int best;
        int crop_w = 0, crop_h = 0;
        char text[64] = "";
        float conf = 0.0f;
        struct ocr_decode_diag diag;
        int64_t t0 = mono_us(), t1, t2, t3;
        if (read_frame(&dma) < 0) {
            fprintf(stderr, "[green-live] DMA frame read failed\n");
            goto out;
        }
        frame_to_rgb888(&dma, &opt, rgb);
        t1 = mono_us();
        if (run_detector(&det_model, rgb, (int)dma.frame_w, (int)dma.frame_h, det_input,
                         opt.det_resize_mode, pose_nc, class_filter, opt.min_conf, opt.nms_iou, opt.max_det,
                         dets, &det_count) < 0) {
            fprintf(stderr, "[green-live] frame=%d detector failed\n", frame);
            continue;
        }
        t2 = mono_us();
        best = pick_best(dets, det_count);
        if (best < 0) {
            printf("[green-live] frame=%d det=0 dma_ms=%.1f det_ms=%.1f\n",
                   frame, (double)(t1 - t0) / 1000.0, (double)(t2 - t1) / 1000.0);
        } else if (!warp_quad_piecewise(rgb, (int)dma.frame_w, (int)dma.frame_h, dets[best].quad,
                                        crop, (int)dma.frame_w, (int)dma.frame_h, &crop_w, &crop_h)) {
            printf("[green-live] frame=%d det=%d best=%d warp=fail\n", frame, det_count, best);
        } else {
            memset(&diag, 0, sizeof(diag));
            if (run_ocr(&ocr_model, &keys, opt.ocr_preproc_mode, crop, crop_w, crop_h,
                        text, sizeof(text), &conf, &diag) < 0) {
                snprintf(text, sizeof(text), "UNK");
                conf = 0.0f;
            }
            t3 = mono_us();
            printf("[green-live] frame=%d det=%d best=%d cls=%d box=[%d,%d,%d,%d] crop=%dx%d "
                   "text=%s conf=%.3f blank=%.3f dma_ms=%.1f det_ms=%.1f ocr_ms=%.1f total_ms=%.1f\n",
                   frame, det_count, best, dets[best].cls,
                   dets[best].x1, dets[best].y1, dets[best].x2, dets[best].y2,
                   crop_w, crop_h, text, conf, diag.blank_top1_ratio,
                   (double)(t1 - t0) / 1000.0,
                   (double)(t2 - t1) / 1000.0,
                   (double)(t3 - t2) / 1000.0,
                   (double)(t3 - t0) / 1000.0);
        }
        fflush(stdout);
        {
            int64_t used = mono_us() - t0;
            if (used < target_us)
                usleep((useconds_t)(target_us - used));
        }
    }
    ret = 0;

out:
    free(rgb);
    free(det_input);
    free(crop);
    model_release(&det_model);
    model_release(&ocr_model);
    release_dma(&dma);
    return ret;
}
