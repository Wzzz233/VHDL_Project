// SPDX-License-Identifier: GPL-2.0
/*
 * Standalone blue/green PPLCNet live validation driver.
 *
 * Pipeline:
 *   FPGA DMA frame -> YOLOv8n-pose plate quad -> color route -> quad warp crop
 *   -> blue or green PPLCNet CTC -> HDMI/KMS display overlay.
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
#include <time.h>
#include <unistd.h>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <rknn_api.h>

#include "ocr_decode.h"
#include "pcie_fpga_dma.h"

#define DEFAULT_DEVICE "/dev/" FPGA_DMA_DEV_NAME
#define DEFAULT_DRM_CARD "/dev/dri/card0"
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
#define OVERLAY_TEXT_SCALE 2
#define COLOR_CYAN_565 0x07FF
#define COLOR_RED_565 0xF800
#define COLOR_WHITE_565 0xFFFF

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

enum plate_color {
    PLATE_COLOR_UNKNOWN = 0,
    PLATE_COLOR_BLUE,
    PLATE_COLOR_GREEN,
    PLATE_COLOR_YELLOW,
};

struct live_options {
    const char *device_path;
    const char *plate_model_path;
    const char *ocr_blue_model_path;
    const char *ocr_green_model_path;
    const char *keys_path;
    const char *drm_card_path;
    int connector_id;
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
    bool display;
    bool display_sync;
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

struct ocr_timing {
    double prep_ms;
    double input_ms;
    double run_ms;
    double output_ms;
    double decode_ms;
};

struct display_state {
    bool enabled;
    int drm_fd;
    uint32_t w;
    uint32_t h;
    int fps;
    int connector_id;
    bool sync;
    size_t frame_size;
    uint64_t next_pts_ns;
    GstElement *pipeline;
    GstElement *appsrc;
    GstElement *queue;
    GstElement *sink;
    GstBus *bus;
};

struct live_result {
    bool valid;
    uint64_t seq;
    struct det_box box;
    int det_count;
    int best;
    int crop_w;
    int crop_h;
    enum plate_color color;
    char route_name[8];
    char text[64];
    float conf;
    float blank_ratio;
};

struct infer_state {
    pthread_t thread;
    bool thread_started;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    pthread_mutex_t result_lock;
    bool running;
    bool has_new;
    uint64_t seq;
    uint64_t overwrite_count;
    uint64_t infer_count;
    uint8_t *latest_rgb;
    size_t rgb_size;
    int frame_w;
    int frame_h;
    struct live_result result;

    const struct live_options *opt;
    struct rknn_model *det_model;
    struct rknn_model *ocr_blue_model;
    struct rknn_model *ocr_green_model;
    const struct ocr_keys *keys;
    int pose_nc;
    int class_filter;
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
            "Usage: %s --plate-model yolov8n_pos.rknn --ocr-blue-model blue.rknn --ocr-green-model green.rknn --ocr-keys keys.txt [opts]\n"
            "\n"
            "Required:\n"
            "  --plate-model <path>       YOLOv8n pose RKNN plate detector\n"
            "  --ocr-blue-model <path>    Blue PPLCNet OCR RKNN\n"
            "  --ocr-green-model <path>   Green PPLCNet OCR RKNN\n"
            "  --ocr-keys <path>          OCR keys file\n"
            "\n"
            "Options:\n"
            "  --device <path>            FPGA DMA device (default: /dev/fpga_dma0)\n"
            "  --drm-card <path>          DRM card for display (default: /dev/dri/card0)\n"
            "  --connector-id <id>        Optional KMS connector id\n"
            "  --no-display               Disable HDMI/KMS display output\n"
            "  --display-sync <0|1>       kmssink sync to display clock (default: 0)\n"
            "  --frames <n>               Frames to process, 0 means forever (default: 0)\n"
            "  --fps <n>                  Capture throttle FPS (default: 10)\n"
            "  --min-plate-conf <v>       Detector threshold (default: 0.50)\n"
            "  --plate-nms-iou <v>        NMS IoU (default: 0.45)\n"
            "  --plate-max-det <n>        Max detections per frame (default: 8)\n"
            "  --class-filter <id>        Detector class filter; -1 disables (default: -1)\n"
            "  --auto-green-filter <0|1>  Auto use detector class 1 when pose_nc>=5 (default: 0)\n"
            "  --det-resize <stretch|letterbox> Detector mapping (default: stretch)\n"
            "  --ocr-preproc <none|gray|bin> OCR preproc (default: gray)\n"
            "  --pixel-order <bgr565|rgb565> Raw 565 order (default: bgr565)\n"
            "  --swap16 <0|1>             Swap raw 565 bytes (default: 0)\n",
            prog);
}

static void defaults(struct live_options *o)
{
    memset(o, 0, sizeof(*o));
    o->device_path = DEFAULT_DEVICE;
    o->drm_card_path = DEFAULT_DRM_CARD;
    o->frames = 0;
    o->fps = 10;
    o->min_conf = 0.50f;
    o->nms_iou = 0.45f;
    o->max_det = 8;
    o->class_filter = -1;
    o->connector_id = -1;
    o->auto_green_filter = false;
    o->det_resize_mode = DET_RESIZE_STRETCH;
    o->ocr_preproc_mode = OCR_PREPROC_GRAY;
    o->pixel_order = PIXEL_ORDER_BGR565;
    o->swap16 = false;
    o->display = true;
    o->display_sync = false;
}

static int parse_options(int argc, char **argv, struct live_options *o)
{
    static const struct option opts[] = {
        {"device", required_argument, NULL, 1},
        {"plate-model", required_argument, NULL, 2},
        {"ocr-green-model", required_argument, NULL, 3},
        {"ocr-keys", required_argument, NULL, 4},
        {"ocr-blue-model", required_argument, NULL, 20},
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
        {"drm-card", required_argument, NULL, 15},
        {"connector-id", required_argument, NULL, 16},
        {"no-display", no_argument, NULL, 17},
        {"display-sync", required_argument, NULL, 18},
        {"auto-green-filter", required_argument, NULL, 19},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0},
    };
    int c;
    defaults(o);
    while ((c = getopt_long(argc, argv, "h", opts, NULL)) != -1) {
        switch (c) {
        case 1: o->device_path = optarg; break;
        case 2: o->plate_model_path = optarg; break;
        case 3: o->ocr_green_model_path = optarg; break;
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
        case 15: o->drm_card_path = optarg; break;
        case 16: o->connector_id = atoi(optarg); break;
        case 17: o->display = false; break;
        case 18:
            o->display_sync = (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0 || strcmp(optarg, "on") == 0);
            break;
        case 19:
            o->auto_green_filter = (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0 || strcmp(optarg, "on") == 0);
            break;
        case 20: o->ocr_blue_model_path = optarg; break;
        case 'h': return 1;
        default: return -1;
        }
    }
    if (!o->plate_model_path || !o->ocr_blue_model_path || !o->ocr_green_model_path || !o->keys_path)
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


static void rgb888_to_rgb565_frame(const uint8_t *rgb, uint16_t *dst, int w, int h)
{
    size_t n = (size_t)w * (size_t)h;
    for (size_t i = 0; i < n; i++) {
        uint8_t r = rgb[i * 3U + 0];
        uint8_t g = rgb[i * 3U + 1];
        uint8_t b = rgb[i * 3U + 2];
        dst[i] = (uint16_t)(((uint16_t)(r >> 3) << 11) |
                            ((uint16_t)(g >> 2) << 5) |
                            ((uint16_t)(b >> 3)));
    }
}

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
    case '-': { static const uint8_t g[7]={0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; return g[row]; }
    case '.': { static const uint8_t g[7]={0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}; return g[row]; }
    case ':': { static const uint8_t g[7]={0x00,0x04,0x04,0x00,0x04,0x04,0x00}; return g[row]; }
    case ' ': { static const uint8_t g[7]={0,0,0,0,0,0,0}; return g[row]; }
    default: return 0;
    }
}

static void draw_text_565(uint16_t *pix, int w, int h, int x, int y, const char *s, uint16_t c, int scale)
{
    if (!s || scale < 1) return;
    for (int i = 0; s[i] != '\0'; i++) {
        int ox = x + i * 6 * scale;
        for (int row = 0; row < 7; row++) {
            uint8_t bits = glyph5x7(s[i], row);
            for (int col = 0; col < 5; col++) {
                if (!(bits & (1U << (4 - col)))) continue;
                for (int sy = 0; sy < scale; sy++) {
                    int py = y + row * scale + sy;
                    if (py < 0 || py >= h) continue;
                    for (int sx = 0; sx < scale; sx++) {
                        int px = ox + col * scale + sx;
                        if (px >= 0 && px < w) pix[py * w + px] = c;
                    }
                }
            }
        }
    }
}

static void overlay_ascii_from_text(const char *text, char *out, size_t out_len)
{
    size_t j = 0;
    if (!out || out_len == 0) return;
    if (!text) text = "";
    for (size_t i = 0; text[i] && j + 1 < out_len; i++) {
        unsigned char ch = (unsigned char)text[i];
        if ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z'))
            out[j++] = (char)ch;
    }
    out[j] = '\0';
}

static int display_start(struct display_state *d, const struct live_options *opt, uint32_t w, uint32_t h)
{
    GstCaps *caps;
    GstStateChangeReturn sret;
    memset(d, 0, sizeof(*d));
    d->enabled = opt->display;
    d->drm_fd = -1;
    if (!d->enabled) return 0;
    d->w = w; d->h = h; d->fps = opt->fps; d->connector_id = opt->connector_id; d->sync = opt->display_sync;
    d->frame_size = (size_t)w * (size_t)h * 2U;
    if (opt->drm_card_path && opt->drm_card_path[0]) {
        d->drm_fd = open(opt->drm_card_path, O_RDWR | O_CLOEXEC);
        if (d->drm_fd < 0)
            fprintf(stderr, "[display] warning: failed to open %s: %s\n", opt->drm_card_path, strerror(errno));
    }
    d->pipeline = gst_pipeline_new("pplcnet-green-live");
    d->appsrc = gst_element_factory_make("appsrc", "src");
    d->queue = gst_element_factory_make("queue", "queue");
    d->sink = gst_element_factory_make("kmssink", "sink");
    if (!d->pipeline || !d->appsrc || !d->queue || !d->sink) {
        fprintf(stderr, "[display] failed to create appsrc/queue/kmssink\n");
        return -1;
    }
    gst_bin_add_many(GST_BIN(d->pipeline), d->appsrc, d->queue, d->sink, NULL);
    if (!gst_element_link_many(d->appsrc, d->queue, d->sink, NULL)) {
        fprintf(stderr, "[display] failed to link appsrc -> queue -> kmssink\n");
        return -1;
    }
    caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "RGB16",
                               "width", G_TYPE_INT, (int)w, "height", G_TYPE_INT, (int)h,
                               "framerate", GST_TYPE_FRACTION, opt->fps, 1, NULL);
    if (!caps) return -1;
    g_object_set(d->appsrc, "caps", caps, "is-live", TRUE, "do-timestamp", TRUE,
                 "format", GST_FORMAT_TIME, "block", FALSE,
                 "max-bytes", (guint64)d->frame_size * 2U, NULL);
    gst_caps_unref(caps);
    g_object_set(d->queue, "max-size-buffers", 1, "max-size-bytes", 0,
                 "max-size-time", (guint64)0, "leaky", 2, NULL);
    g_object_set(d->sink, "sync", d->sync ? TRUE : FALSE, NULL);
    if (d->connector_id >= 0) g_object_set(d->sink, "connector-id", d->connector_id, NULL);
    if (d->drm_fd >= 0) g_object_set(d->sink, "fd", d->drm_fd, NULL);
    d->bus = gst_element_get_bus(d->pipeline);
    sret = gst_element_set_state(d->pipeline, GST_STATE_PLAYING);
    if (sret == GST_STATE_CHANGE_FAILURE) return -1;
    sret = gst_element_get_state(d->pipeline, NULL, NULL, 5 * GST_SECOND);
    if (sret == GST_STATE_CHANGE_FAILURE) return -1;
    fprintf(stderr, "[display] started appsrc RGB16 %ux%u -> kmssink sync=%d connector=%d\n",
            w, h, d->sync ? 1 : 0, d->connector_id);
    return 0;
}

static void display_stop(struct display_state *d)
{
    if (!d || !d->enabled) return;
    if (d->appsrc) gst_app_src_end_of_stream(GST_APP_SRC(d->appsrc));
    if (d->pipeline) gst_element_set_state(d->pipeline, GST_STATE_NULL);
    if (d->bus) gst_object_unref(d->bus);
    if (d->pipeline) gst_object_unref(d->pipeline);
    if (d->drm_fd >= 0) close(d->drm_fd);
    memset(d, 0, sizeof(*d));
    d->drm_fd = -1;
}

static int display_handle_bus(struct display_state *d)
{
    GstMessage *msg;
    if (!d || !d->enabled || !d->bus) return 0;
    while ((msg = gst_bus_pop(d->bus)) != NULL) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError *err = NULL; gchar *dbg = NULL;
            gst_message_parse_error(msg, &err, &dbg);
            fprintf(stderr, "[display] error: %s\n", err ? err->message : "unknown");
            if (err) g_error_free(err);
            g_free(dbg);
            gst_message_unref(msg);
            return -1;
        }
        gst_message_unref(msg);
    }
    return 0;
}

static int display_push(struct display_state *d, const uint16_t *frame)
{
    uint8_t *copy;
    GstBuffer *buf;
    GstFlowReturn flow;
    if (!d || !d->enabled) return 0;
    if (display_handle_bus(d) < 0) return -1;
    copy = g_malloc(d->frame_size);
    if (!copy) return -1;
    memcpy(copy, frame, d->frame_size);
    buf = gst_buffer_new_wrapped_full((GstMemoryFlags)0, copy, d->frame_size, 0, d->frame_size, copy, g_free);
    if (!buf) { g_free(copy); return -1; }
    GST_BUFFER_PTS(buf) = d->next_pts_ns;
    GST_BUFFER_DURATION(buf) = (guint64)(GST_SECOND / d->fps);
    d->next_pts_ns += GST_BUFFER_DURATION(buf);
    flow = gst_app_src_push_buffer(GST_APP_SRC(d->appsrc), buf);
    return flow == GST_FLOW_OK ? 0 : -1;
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

static float quad_area8(const float q[8])
{
    float area = 0.0f;
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) & 3;
        area += q[i * 2] * q[j * 2 + 1] - q[j * 2] * q[i * 2 + 1];
    }
    return 0.5f * fabsf(area);
}

static bool quad_is_convex8(const float q[8])
{
    float prev = 0.0f;
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) & 3;
        int k = (i + 2) & 3;
        float ax = q[j * 2] - q[i * 2];
        float ay = q[j * 2 + 1] - q[i * 2 + 1];
        float bx = q[k * 2] - q[j * 2];
        float by = q[k * 2 + 1] - q[j * 2 + 1];
        float cross = ax * by - ay * bx;
        if (fabsf(cross) < 1e-5f)
            continue;
        if (prev != 0.0f && cross * prev < 0.0f)
            return false;
        prev = cross;
    }
    return prev != 0.0f;
}

static void order_quad(const float in[8], float out[8])
{
    struct quad_pt { float x; float y; float a; } pts[4];
    int idx[4] = {0, 1, 2, 3};
    float cx = 0.0f, cy = 0.0f;
    if (!in || !out) return;
    for (int i = 0; i < 4; i++) {
        pts[i].x = in[i * 2];
        pts[i].y = in[i * 2 + 1];
        cx += pts[i].x;
        cy += pts[i].y;
    }
    cx *= 0.25f;
    cy *= 0.25f;
    for (int i = 0; i < 4; i++)
        pts[i].a = atan2f(pts[i].y - cy, pts[i].x - cx);
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
            if (pts[idx[j]].a < pts[idx[i]].a) {
                int t = idx[i]; idx[i] = idx[j]; idx[j] = t;
            }
        }
    }
    int top_edge = 0;
    float best_y = 0.5f * (pts[idx[0]].y + pts[idx[1]].y);
    for (int i = 1; i < 4; i++) {
        float edge_y = 0.5f * (pts[idx[i]].y + pts[idx[(i + 1) & 3]].y);
        if (edge_y < best_y) { best_y = edge_y; top_edge = i; }
    }
    if (pts[idx[top_edge]].x <= pts[idx[(top_edge + 1) & 3]].x) {
        for (int i = 0; i < 4; i++) {
            int pos = idx[(top_edge + i) & 3];
            out[i * 2] = pts[pos].x;
            out[i * 2 + 1] = pts[pos].y;
        }
    } else {
        for (int i = 0; i < 4; i++) {
            int pos = idx[(top_edge + 1 - i + 4) & 3];
            out[i * 2] = pts[pos].x;
            out[i * 2 + 1] = pts[pos].y;
        }
    }
    if (quad_area8(out) < 4.0f || !quad_is_convex8(out))
        memcpy(out, in, sizeof(float) * 8U);
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

static bool solve_linear_8x8(float a[8][9], float x[8])
{
    for (int i = 0; i < 8; i++) {
        int piv = i;
        float max_v = fabsf(a[i][i]);
        for (int j = i + 1; j < 8; j++) {
            float v = fabsf(a[j][i]);
            if (v > max_v) { max_v = v; piv = j; }
        }
        if (max_v < 1e-8f) return false;
        if (piv != i) {
            for (int k = i; k < 9; k++) { float t = a[i][k]; a[i][k] = a[piv][k]; a[piv][k] = t; }
        }
        float div = a[i][i];
        for (int k = i; k < 9; k++) a[i][k] /= div;
        for (int j = 0; j < 8; j++) {
            if (j == i) continue;
            float mul = a[j][i];
            if (fabsf(mul) < 1e-8f) continue;
            for (int k = i; k < 9; k++) a[j][k] -= mul * a[i][k];
        }
    }
    for (int i = 0; i < 8; i++) x[i] = a[i][8];
    return true;
}

static bool get_homography_4pt(const float src[8], const float dst[8], float h[9])
{
    float mat[8][9];
    float sol[8];
    memset(mat, 0, sizeof(mat));
    for (int i = 0; i < 4; i++) {
        float x = src[i * 2], y = src[i * 2 + 1];
        float u = dst[i * 2], v = dst[i * 2 + 1];
        int r0 = i * 2, r1 = r0 + 1;
        mat[r0][0] = x; mat[r0][1] = y; mat[r0][2] = 1.0f;
        mat[r0][6] = -u * x; mat[r0][7] = -u * y; mat[r0][8] = u;
        mat[r1][3] = x; mat[r1][4] = y; mat[r1][5] = 1.0f;
        mat[r1][6] = -v * x; mat[r1][7] = -v * y; mat[r1][8] = v;
    }
    if (!solve_linear_8x8(mat, sol)) return false;
    h[0] = sol[0]; h[1] = sol[1]; h[2] = sol[2];
    h[3] = sol[3]; h[4] = sol[4]; h[5] = sol[5];
    h[6] = sol[6]; h[7] = sol[7]; h[8] = 1.0f;
    return true;
}

static bool invert_homography(const float h[9], float inv[9])
{
    float det = h[0] * (h[4] * h[8] - h[5] * h[7]) -
                h[1] * (h[3] * h[8] - h[5] * h[6]) +
                h[2] * (h[3] * h[7] - h[4] * h[6]);
    if (fabsf(det) < 1e-8f) return false;
    inv[0] =  (h[4] * h[8] - h[5] * h[7]) / det;
    inv[1] = -(h[1] * h[8] - h[2] * h[7]) / det;
    inv[2] =  (h[1] * h[5] - h[2] * h[4]) / det;
    inv[3] = -(h[3] * h[8] - h[5] * h[6]) / det;
    inv[4] =  (h[0] * h[8] - h[2] * h[6]) / det;
    inv[5] = -(h[0] * h[5] - h[2] * h[3]) / det;
    inv[6] =  (h[3] * h[7] - h[4] * h[6]) / det;
    inv[7] = -(h[0] * h[7] - h[1] * h[6]) / det;
    inv[8] =  (h[0] * h[4] - h[1] * h[3]) / det;
    return true;
}

static bool warp_quad_homography(const uint8_t *rgb, int img_w, int img_h, const float quad_in[8],
                                 uint8_t *dst, int cap_w, int cap_h, int *out_w, int *out_h)
{
    float q[8], dst_quad[8], h[9], inv_h[9];
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
    if (dw <= 0 || dh <= 0) return false;
    dst_quad[0] = 0.0f;             dst_quad[1] = 0.0f;
    dst_quad[2] = (float)dw - 1.0f; dst_quad[3] = 0.0f;
    dst_quad[4] = (float)dw - 1.0f; dst_quad[5] = (float)dh - 1.0f;
    dst_quad[6] = 0.0f;             dst_quad[7] = (float)dh - 1.0f;
    if (!get_homography_4pt(q, dst_quad, h)) return false;
    if (!invert_homography(h, inv_h)) return false;
    for (int y = 0; y < dh; y++) {
        for (int x = 0; x < dw; x++) {
            float fx = (float)x, fy = (float)y;
            float den = inv_h[6] * fx + inv_h[7] * fy + inv_h[8];
            float sx, sy;
            uint8_t pix[3];
            if (fabsf(den) < 1e-8f) den = den >= 0.0f ? 1e-8f : -1e-8f;
            sx = (inv_h[0] * fx + inv_h[1] * fy + inv_h[2]) / den;
            sy = (inv_h[3] * fx + inv_h[4] * fy + inv_h[5]) / den;
            sample_bilinear(rgb, img_w, img_h, sx, sy, pix);
            memcpy(dst + ((size_t)y * dw + x) * 3U, pix, 3);
        }
    }
    *out_w = dw;
    *out_h = dh;
    return true;
}

static bool __attribute__((unused)) warp_quad_piecewise(const uint8_t *rgb, int img_w, int img_h, const float quad_in[8],
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
                   enum ocr_preproc_mode preproc, enum ocr_decode_family family,
                   const uint8_t *crop_rgb, int crop_w, int crop_h,
                   char *text, size_t text_len, float *conf, struct ocr_decode_diag *diag,
                   struct ocr_timing *timing)
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
    t0 = mono_us();
    input = malloc((size_t)m->in_w * m->in_h * 3U);
    if (!input)
        return -1;
    resize_bilinear(crop_rgb, crop_w, crop_h, input, (int)m->in_w, (int)m->in_h);
    rgb_to_bgr_inplace(input, (int)m->in_w, (int)m->in_h);
    if (preproc == OCR_PREPROC_GRAY)
        apply_gray3(input, (int)m->in_w, (int)m->in_h);
    else if (preproc == OCR_PREPROC_BIN)
        apply_bin(input, (int)m->in_w, (int)m->in_h);
    t1 = mono_us();
    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf = input;
    in.size = m->in_w * m->in_h * 3U;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    ret = rknn_inputs_set(m->ctx, 1, &in);
    t2 = mono_us();
    if (ret < 0) { free(input); return ret; }
    ret = rknn_run(m->ctx, NULL);
    t3 = mono_us();
    if (ret < 0) { free(input); return ret; }
    memset(&out, 0, sizeof(out));
    out.want_float = 1;
    ret = rknn_outputs_get(m->ctx, 1, &out, NULL);
    t4 = mono_us();
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
    t5 = mono_us();
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


static enum plate_color classify_plate_color_rgb(const uint8_t *rgb, int w, int h, const struct det_box *b)
{
    int x1 = b->x1 + (b->x2 - b->x1) / 6;
    int x2 = b->x2 - (b->x2 - b->x1) / 6;
    int y1 = b->y1 + (b->y2 - b->y1) / 6;
    int y2 = b->y2 - (b->y2 - b->y1) / 6;
    int total = 0, blue_cnt = 0, green_cnt = 0, yellow_cnt = 0, dark_cnt = 0;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 >= w) x2 = w - 1;
    if (y2 >= h) y2 = h - 1;
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            const uint8_t *p = rgb + (y * w + x) * 3;
            float r = p[0] / 255.0f;
            float g = p[1] / 255.0f;
            float bch = p[2] / 255.0f;
            float mx = fmaxf(r, fmaxf(g, bch));
            float mn = fminf(r, fminf(g, bch));
            float d = mx - mn;
            float h_deg = 0.0f;
            float s = (mx == 0.0f) ? 0.0f : (d / mx);
            float v = mx;
            if (v < 0.20f) dark_cnt++;
            if (d > 1e-6f) {
                if (mx == r) h_deg = 60.0f * fmodf((g - bch) / d, 6.0f);
                else if (mx == g) h_deg = 60.0f * (((bch - r) / d) + 2.0f);
                else h_deg = 60.0f * (((r - g) / d) + 4.0f);
            }
            if (h_deg < 0.0f) h_deg += 360.0f;
            total++;
            if (h_deg >= 190.0f && h_deg <= 260.0f && s > 0.23f && v > 0.16f) blue_cnt++;
            else if (h_deg >= 75.0f && h_deg <= 155.0f && s > 0.20f && v > 0.16f) green_cnt++;
            else if (h_deg >= 15.0f && h_deg <= 55.0f && s > 0.15f && v > 0.16f) yellow_cnt++;
        }
    }
    if (total == 0) return PLATE_COLOR_UNKNOWN;
    if ((float)blue_cnt / (float)total >= 0.20f && blue_cnt > green_cnt + (int)(0.05f * total))
        return PLATE_COLOR_BLUE;
    if ((float)green_cnt / (float)total >= 0.20f && green_cnt > blue_cnt + (int)(0.05f * total))
        return PLATE_COLOR_GREEN;
    if ((float)yellow_cnt / (float)total >= 0.18f && (float)dark_cnt / (float)total < 0.50f)
        return PLATE_COLOR_YELLOW;
    return PLATE_COLOR_UNKNOWN;
}

static const char *plate_color_str(enum plate_color c)
{
    if (c == PLATE_COLOR_BLUE) return "BLUE";
    if (c == PLATE_COLOR_GREEN) return "GREEN";
    if (c == PLATE_COLOR_YELLOW) return "YELLOW";
    return "UNK";
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

static void log_ocr_contract(const char *route_name, const struct rknn_model *m, const struct ocr_keys *keys)
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
            "[bg-live] route=%s keys=%d first=%s idx12=%s last=%s ocr_layout=t%d c%d t_stride=%d c_stride=%d blank=%d\n",
            route_name ? route_name : "?", keys->count, k0, k11, klast, t_size, c_size, t_stride, c_stride, blank);
}


static void infer_publish_result(struct infer_state *st, const struct live_result *res)
{
    pthread_mutex_lock(&st->result_lock);
    st->result = *res;
    pthread_mutex_unlock(&st->result_lock);
}

static void infer_get_result(struct infer_state *st, struct live_result *res)
{
    pthread_mutex_lock(&st->result_lock);
    *res = st->result;
    pthread_mutex_unlock(&st->result_lock);
}

static void infer_submit_latest(struct infer_state *st, const uint8_t *rgb)
{
    pthread_mutex_lock(&st->lock);
    if (st->has_new)
        st->overwrite_count++;
    memcpy(st->latest_rgb, rgb, st->rgb_size);
    st->seq++;
    st->has_new = true;
    pthread_cond_signal(&st->cond);
    pthread_mutex_unlock(&st->lock);
}

static void *infer_thread_main(void *arg)
{
    struct infer_state *st = (struct infer_state *)arg;
    uint8_t *rgb = malloc(st->rgb_size);
    uint8_t *det_input = malloc((size_t)st->det_model->in_w * st->det_model->in_h * 3U);
    uint8_t *crop = malloc(st->rgb_size);
    if (!rgb || !det_input || !crop) {
        fprintf(stderr, "[bg-live] infer thread alloc failed\n");
        free(rgb); free(det_input); free(crop);
        return NULL;
    }

    while (1) {
        uint64_t seq;
        struct det_box dets[MAX_DETS];
        int det_count = 0;
        int best = -1;
        int crop_w = 0, crop_h = 0;
        char text[64] = "";
        float conf = 0.0f;
        struct ocr_decode_diag diag;
        struct ocr_timing ocr_timing;
        enum plate_color color = PLATE_COLOR_UNKNOWN;
        const char *route_name = "blue";
        double warp_ms = 0.0, color_ms = 0.0;
        int64_t t0, t1, t2;
        struct live_result res;

        pthread_mutex_lock(&st->lock);
        while (st->running && !st->has_new)
            pthread_cond_wait(&st->cond, &st->lock);
        if (!st->running && !st->has_new) {
            pthread_mutex_unlock(&st->lock);
            break;
        }
        memcpy(rgb, st->latest_rgb, st->rgb_size);
        seq = st->seq;
        st->has_new = false;
        pthread_mutex_unlock(&st->lock);

        memset(&res, 0, sizeof(res));
        res.seq = seq;
        t0 = mono_us();
        if (run_detector(st->det_model, rgb, st->frame_w, st->frame_h, det_input,
                         st->opt->det_resize_mode, st->pose_nc, st->class_filter,
                         st->opt->min_conf, st->opt->nms_iou, st->opt->max_det,
                         dets, &det_count) < 0) {
            fprintf(stderr, "[bg-live] infer seq=%" PRIu64 " detector failed\n", seq);
            continue;
        }
        t1 = mono_us();
        best = pick_best(dets, det_count);
        res.det_count = det_count;
        res.best = best;
        if (best >= 0) {
            int64_t tw0 = mono_us();
            bool warp_ok = warp_quad_homography(rgb, st->frame_w, st->frame_h, dets[best].quad,
                                                crop, st->frame_w, st->frame_h, &crop_w, &crop_h);
            int64_t tw1 = mono_us();
            warp_ms = (double)(tw1 - tw0) / 1000.0;
            if (warp_ok) {
                struct rknn_model *ocr_route = st->ocr_blue_model;
                enum ocr_decode_family decode_family = OCR_DECODE_FAMILY_NORMAL7;
                int64_t tc0 = mono_us();
                color = classify_plate_color_rgb(rgb, st->frame_w, st->frame_h, &dets[best]);
                int64_t tc1 = mono_us();
                color_ms = (double)(tc1 - tc0) / 1000.0;
                if (color == PLATE_COLOR_GREEN) {
                    ocr_route = st->ocr_green_model;
                    decode_family = OCR_DECODE_FAMILY_GREEN8;
                    route_name = "green";
                }
                memset(&diag, 0, sizeof(diag));
                memset(&ocr_timing, 0, sizeof(ocr_timing));
                if (run_ocr(ocr_route, st->keys, st->opt->ocr_preproc_mode, decode_family,
                            crop, crop_w, crop_h, text, sizeof(text), &conf, &diag, &ocr_timing) < 0) {
                    snprintf(text, sizeof(text), "UNK");
                    conf = 0.0f;
                }
                res.valid = true;
                res.box = dets[best];
                res.crop_w = crop_w;
                res.crop_h = crop_h;
                res.color = color;
                snprintf(res.route_name, sizeof(res.route_name), "%s", route_name);
                snprintf(res.text, sizeof(res.text), "%s", text);
                res.conf = conf;
                res.blank_ratio = diag.blank_top1_ratio;
            }
        }
        t2 = mono_us();
        st->infer_count++;
        infer_publish_result(st, &res);
        if (res.valid) {
            printf("[bg-live] infer_seq=%" PRIu64 " det=%d best=%d cls=%d color=%s route=%s box=[%d,%d,%d,%d] crop=%dx%d "
                   "text=%s conf=%.3f blank=%.3f detocr_ms=%.1f det_ms=%.1f ocr_ms=%.1f "
                   "warp_ms=%.1f color_ms=%.1f prep_ms=%.1f in_ms=%.1f run_ms=%.1f out_ms=%.1f dec_ms=%.1f overwritten=%" PRIu64 "\n",
                   seq, det_count, best, dets[best].cls, plate_color_str(color), route_name,
                   dets[best].x1, dets[best].y1, dets[best].x2, dets[best].y2,
                   crop_w, crop_h, text, conf, diag.blank_top1_ratio,
                   (double)(t2 - t0) / 1000.0,
                   (double)(t1 - t0) / 1000.0,
                   (double)(t2 - t1) / 1000.0,
                   warp_ms, color_ms, ocr_timing.prep_ms, ocr_timing.input_ms,
                   ocr_timing.run_ms, ocr_timing.output_ms, ocr_timing.decode_ms,
                   st->overwrite_count);
        } else {
            printf("[bg-live] infer_seq=%" PRIu64 " det=%d best=%d det_ms=%.1f overwritten=%" PRIu64 "\n",
                   seq, det_count, best, (double)(t1 - t0) / 1000.0, st->overwrite_count);
        }
        fflush(stdout);
    }

    free(rgb); free(det_input); free(crop);
    return NULL;
}

static int infer_start(struct infer_state *st, const struct live_options *opt,
                       struct rknn_model *det_model, struct rknn_model *ocr_blue_model,
                       struct rknn_model *ocr_green_model, const struct ocr_keys *keys,
                       int pose_nc, int class_filter, int frame_w, int frame_h)
{
    memset(st, 0, sizeof(*st));
    st->opt = opt;
    st->det_model = det_model;
    st->ocr_blue_model = ocr_blue_model;
    st->ocr_green_model = ocr_green_model;
    st->keys = keys;
    st->pose_nc = pose_nc;
    st->class_filter = class_filter;
    st->frame_w = frame_w;
    st->frame_h = frame_h;
    st->rgb_size = (size_t)frame_w * (size_t)frame_h * 3U;
    st->latest_rgb = malloc(st->rgb_size);
    if (!st->latest_rgb)
        return -1;
    pthread_mutex_init(&st->lock, NULL);
    pthread_cond_init(&st->cond, NULL);
    pthread_mutex_init(&st->result_lock, NULL);
    st->running = true;
    if (pthread_create(&st->thread, NULL, infer_thread_main, st) != 0)
        return -1;
    st->thread_started = true;
    return 0;
}

static void infer_stop(struct infer_state *st)
{
    if (!st)
        return;
    pthread_mutex_lock(&st->lock);
    st->running = false;
    pthread_cond_broadcast(&st->cond);
    pthread_mutex_unlock(&st->lock);
    if (st->thread_started)
        pthread_join(st->thread, NULL);
    free(st->latest_rgb);
    pthread_mutex_destroy(&st->lock);
    pthread_cond_destroy(&st->cond);
    pthread_mutex_destroy(&st->result_lock);
    memset(st, 0, sizeof(*st));
}

int main(int argc, char **argv)
{
    struct live_options opt;
    struct dma_state dma;
    struct rknn_model det_model;
    struct rknn_model ocr_blue_model;
    struct rknn_model ocr_green_model;
    struct ocr_keys keys;
    struct display_state display;
    struct infer_state infer;
    uint8_t *rgb = NULL;
    uint8_t *det_input = NULL;
    uint8_t *crop = NULL;
    uint16_t *display_frame = NULL;
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
    memset(&display, 0, sizeof(display)); display.drm_fd = -1;
    memset(&infer, 0, sizeof(infer));
    memset(&det_model, 0, sizeof(det_model));
    memset(&ocr_blue_model, 0, sizeof(ocr_blue_model));
    memset(&ocr_green_model, 0, sizeof(ocr_green_model));
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    if (opt.display)
        gst_init(NULL, NULL);

    if (load_keys(opt.keys_path, &keys) < 0) {
        fprintf(stderr, "[bg-live] failed to load keys: %s\n", opt.keys_path);
        goto out;
    }
    if (init_dma(&dma, &opt) < 0) {
        fprintf(stderr, "[bg-live] failed to init DMA: %s: %s\n", opt.device_path, strerror(errno));
        goto out;
    }
    if (display_start(&display, &opt, dma.frame_w, dma.frame_h) < 0) {
        fprintf(stderr, "[bg-live] failed to start display\n");
        goto out;
    }
    if (model_load(&det_model, "yolov8n_pose", opt.plate_model_path) < 0) {
        fprintf(stderr, "[bg-live] failed to load detector: %s\n", opt.plate_model_path);
        goto out;
    }
    if (model_load(&ocr_blue_model, "pplcnet_blue", opt.ocr_blue_model_path) < 0) {
        fprintf(stderr, "[bg-live] failed to load blue OCR: %s\n", opt.ocr_blue_model_path);
        goto out;
    }
    if (model_load(&ocr_green_model, "pplcnet_green", opt.ocr_green_model_path) < 0) {
        fprintf(stderr, "[bg-live] failed to load green OCR: %s\n", opt.ocr_green_model_path);
        goto out;
    }
    if (det_model.in_w != ALGO_STREAM_SIZE || det_model.in_h != ALGO_STREAM_SIZE || det_model.in_c != 3) {
        fprintf(stderr, "[bg-live] detector input must be 640x640x3, got %ux%ux%u\n",
                det_model.in_w, det_model.in_h, det_model.in_c);
        goto out;
    }
    if (ocr_blue_model.in_c != 3 || ocr_green_model.in_c != 3) {
        fprintf(stderr, "[bg-live] OCR input must have 3 channels, got blue=%u green=%u\n",
                ocr_blue_model.in_c, ocr_green_model.in_c);
        goto out;
    }

    log_ocr_contract("blue", &ocr_blue_model, &keys);
    log_ocr_contract("green", &ocr_green_model, &keys);

    pose_nc = detect_pose_nc_from_attrs(&det_model);
    class_filter = opt.class_filter;
    if (opt.auto_green_filter && pose_nc >= 5)
        class_filter = 1;

    rgb = malloc((size_t)dma.frame_w * dma.frame_h * 3U);
    if (opt.display)
        display_frame = malloc((size_t)dma.frame_w * dma.frame_h * 2U);
    if (!rgb || (opt.display && !display_frame))
        goto out;

    fprintf(stderr,
            "[bg-live] start frame=%ux%u src=%s frames=%d fps=%d pose_nc=%d class_filter=%d "
            "det_resize=%s blue_ocr=%ux%u green_ocr=%ux%u preproc=%s display=%d auto_green_filter=%d async_infer=1\n",
            dma.frame_w, dma.frame_h, dma.src_is_bgrx ? "bgrx8888" : "bgr565",
            opt.frames, opt.fps, pose_nc, class_filter,
            opt.det_resize_mode == DET_RESIZE_LETTERBOX ? "letterbox" : "stretch",
            ocr_blue_model.in_w, ocr_blue_model.in_h, ocr_green_model.in_w, ocr_green_model.in_h,
            opt.ocr_preproc_mode == OCR_PREPROC_GRAY ? "gray" : (opt.ocr_preproc_mode == OCR_PREPROC_BIN ? "bin" : "none"),
            opt.display ? 1 : 0, opt.auto_green_filter ? 1 : 0);

    if (infer_start(&infer, &opt, &det_model, &ocr_blue_model, &ocr_green_model, &keys,
                    pose_nc, class_filter, (int)dma.frame_w, (int)dma.frame_h) < 0) {
        fprintf(stderr, "[bg-live] failed to start infer thread\n");
        goto out;
    }

    target_us = 1000000LL / opt.fps;
    for (int frame = 0; !g_stop && (opt.frames == 0 || frame < opt.frames); frame++) {
        struct live_result latest;
        int64_t t0 = mono_us();
        if (read_frame(&dma) < 0) {
            fprintf(stderr, "[bg-live] DMA frame read failed\n");
            goto out;
        }
        frame_to_rgb888(&dma, &opt, rgb);
        infer_submit_latest(&infer, rgb);

        if (display_frame) {
            rgb888_to_rgb565_frame(rgb, display_frame, (int)dma.frame_w, (int)dma.frame_h);
            infer_get_result(&infer, &latest);
            if (latest.valid) {
                char ascii[32];
                char overlay[64];
                int ty = latest.box.y1 - (7 * OVERLAY_TEXT_SCALE + 3);
                if (ty < 0) ty = latest.box.y1 + 3;
                overlay_ascii_from_text(latest.text, ascii, sizeof(ascii));
                snprintf(overlay, sizeof(overlay), "%s %c %.2f", ascii[0] ? ascii : "OCR",
                         latest.route_name[0] == 'g' ? 'G' : 'B', latest.conf);
                draw_rect_565(display_frame, (int)dma.frame_w, (int)dma.frame_h,
                              &latest.box, COLOR_CYAN_565);
                draw_text_565(display_frame, (int)dma.frame_w, (int)dma.frame_h,
                              latest.box.x1, ty, overlay, COLOR_CYAN_565, OVERLAY_TEXT_SCALE);
            }
            if (display_push(&display, display_frame) < 0)
                goto out;
        }
        {
            int64_t used = mono_us() - t0;
            if (used < target_us)
                usleep((useconds_t)(target_us - used));
        }
    }
    ret = 0;

out:
    infer_stop(&infer);
    free(rgb);
    free(det_input);
    free(crop);
    free(display_frame);
    display_stop(&display);
    model_release(&det_model);
    model_release(&ocr_blue_model);
    model_release(&ocr_green_model);
    release_dma(&dma);
    return ret;
}
