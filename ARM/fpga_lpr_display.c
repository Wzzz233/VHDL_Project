// SPDX-License-Identifier: GPL-2.0
/*
 * FPGA LPR Display Application (Phase 4)
 *
 * Display path:
 *   /dev/fpga_dma0 -> appsrc(BGR16) -> queue(leaky) -> kmssink
 *
 * Inference path:
 *   Dedicated worker thread converts BGR565 to RGB888 for RKNN only.
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

#define MAX_LABELS 256
#define MAX_LABEL_LEN 64
#define MAX_DETS 128
#define MAX_OCR_KEYS 128
#define MAX_OCR_KEY_LEN 16
#define ALGO_STREAM_SIZE 640
#define OCR_CROP_WIDTH 150
#define OCR_CROP_HEIGHT 50

#define COLOR_YELLOW_565 0xFFE0
#define COLOR_CYAN_565 0x07FF
#define COLOR_RED_565 0xF800
#define COLOR_GREEN_565 0x07E0

enum pixel_order {
    PIXEL_ORDER_BGR565 = 0,
    PIXEL_ORDER_RGB565,
};

enum plate_color {
    PLATE_COLOR_UNKNOWN = 0,
    PLATE_COLOR_BLUE,
    PLATE_COLOR_GREEN,
    PLATE_COLOR_YELLOW,
};

enum plate_type {
    PLATE_TYPE_COMMON_BLUE = 0,
    PLATE_TYPE_COMMON_GREEN,
    PLATE_TYPE_YELLOW,
    PLATE_TYPE_POLICE,
    PLATE_TYPE_TRAILER,
    PLATE_TYPE_EMBASSY_CONSULATE,
    PLATE_TYPE_UNKNOWN,
};

struct options {
    const char *device_path;
    const char *drm_card_path;
    const char *veh_model_path;
    const char *plate_model_path;
    const char *ocr_model_path;
    const char *ocr_keys_path;
    const char *labels_path;
    const char *pred_log_path;
    int connector_id;
    int fps;
    enum pixel_order pixel_order;
    int timeout_ms;
    int stats_interval;
    int copy_buffers;
    int queue_depth;
    float min_car_conf;
    float min_plate_conf;
    int plate_on_car_only;
    int plate_only;
    int sw_preproc;
    int fpga_a_mask;
    float a_proj_ratio;
    float a_roi_iou_min;
    int ped_event;
    int red_stable_frames;
    float red_ratio_thr;
    float stopline_ratio;
    bool swap16;
};

struct det_box {
    int x1;
    int y1;
    int x2;
    int y2;
    float conf;
    int cls;
};

struct plate_det {
    struct det_box box;
    struct det_box crop_box;
    enum plate_color color;
    enum plate_type type;
    int parent_car;
    char ocr_text[24];
    float ocr_conf;
};

struct frame_slot {
    uint8_t *data;
    bool in_use;
    uint64_t generation;
};

struct lpr_results {
    struct det_box cars[MAX_DETS];
    int car_count;
    int car_raw_count;
    struct det_box persons[MAX_DETS];
    int person_count;
    int person_raw_count;
    struct plate_det plates[MAX_DETS];
    int plate_count;
    int plate_raw_count;
    struct det_box a_roi;
    int a_roi_valid;
    int light_red;
    uint64_t ped_event_total;
    uint64_t ped_event_last_frame;
    uint64_t frame_seq;
    double infer_ms_last;
    uint64_t infer_frames_total;
    double infer_ms_total;
};

struct yolo_model {
    const char *name;
    const char *path;
    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr input_attr;
    rknn_tensor_attr output_attrs[8];
    uint32_t in_w;
    uint32_t in_h;
    uint32_t in_c;
    int class_count;
};

struct ocr_model {
    const char *name;
    const char *path;
    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr input_attr;
    rknn_tensor_attr output_attrs[4];
    uint32_t in_w;
    uint32_t in_h;
    uint32_t in_c;
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
    size_t src_frame_size;
    bool src_is_bgrx;
    uint32_t frame_bpp;
    size_t frame_size;

    struct frame_slot *slots;
    int slot_count;
    GMutex slots_lock;
    GCond slots_cond;

    GstElement *pipeline;
    GstElement *appsrc;
    GstElement *queue;
    GstElement *sink;
    GstBus *bus;

    bool running;
    uint64_t captured_frames;
    uint64_t pushed_frames;
    uint64_t released_frames;
    uint64_t next_pts_ns;
    int64_t last_stats_us;
    uint64_t last_stats_cap;
    uint64_t last_stats_rel;
    uint64_t last_stats_infer;
    uint64_t slot_wait_timeout_count;
    uint64_t infer_overwrite_count;

    pthread_t infer_thread;
    pthread_mutex_t infer_lock;
    pthread_cond_t infer_cond;
    uint8_t *infer_latest_raw;
    bool infer_has_new;
    uint64_t infer_frame_seq;

    pthread_mutex_t result_lock;
    struct lpr_results results;

    struct yolo_model veh_model;
    struct yolo_model plate_model;
    struct ocr_model ocr_model;
    char ocr_keys[MAX_OCR_KEYS][MAX_OCR_KEY_LEN];
    int ocr_key_count;
    int ocr_blank_index;
    FILE *pred_log_fp;
    pthread_mutex_t pred_log_lock;
    char labels[MAX_LABELS][MAX_LABEL_LEN];
    int label_count;
    int car_class_id;
    int person_class_id;

    struct det_box plate_hist1[MAX_DETS];
    int plate_hist1_count;
    struct det_box plate_hist2[MAX_DETS];
    int plate_hist2_count;

    uint64_t pred_rows_total;
    uint64_t gate_plate_raw_positive_frames;
    uint64_t gate_plate_raw_positive_streak;

    struct det_box ped_track_box[MAX_DETS];
    int ped_track_id[MAX_DETS];
    int ped_track_ttl[MAX_DETS];
    int ped_track_count;
    int ped_next_track_id;
    int ped_red_streak;
};

struct slot_ticket {
    int idx;
    uint64_t generation;
};

struct frame_cookie {
    struct app_ctx *ctx;
    struct slot_ticket ticket;
};

static volatile sig_atomic_t g_stop = 0;

static void resize_rgb888_nn(const uint8_t *src, int sw, int sh, uint8_t *dst, int dw, int dh);
static float box_iou(const struct det_box *a, const struct det_box *b);

static int64_t mono_us(void)
{
    return g_get_monotonic_time();
}

static void signal_handler(int signo)
{
    if (signo == SIGINT || signo == SIGTERM)
        g_stop = 1;
}

static void print_usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s [OPTIONS]\n"
            "  --device <path>         FPGA device (default: %s)\n"
            "  --drm-card <path>       DRM card (default: %s)\n"
            "  --veh-model <path>      Vehicle RKNN model path (required)\n"
            "  --plate-model <path>    Plate RKNN model path (required)\n"
            "  --ocr-model <path>      OCR RKNN model path (required)\n"
            "  --ocr-keys <path>       OCR keys file path (required)\n"
            "  --labels <path>         Labels file path (required)\n"
            "  --pred-log <path>       Prediction CSV output path (optional)\n"
            "  --connector-id <id>     Optional KMS connector id\n"
            "  --fps <num>             Target FPS (default: %d)\n"
            "  --pixel-order <mode>    bgr565|rgb565 (default: bgr565)\n"
            "  --swap16 <0|1>          Swap bytes per 16-bit pixel (default: 1)\n"
            "  --timeout-ms <ms>       Frame timeout (default: %d)\n"
            "  --stats-interval <sec>  Stats print interval (default: %d)\n"
            "  --copy-buffers <num>    Copy ring size (default: %d)\n"
            "  --queue-depth <num>     appsrc max frame queue (default: %d)\n"
            "  --min-car-conf <v>      Car confidence threshold (default: 0.35)\n"
            "  --min-plate-conf <v>    Plate confidence threshold (default: 0.45)\n"
            "  --plate-on-car-only <0|1>  Reserve switch (default: 0)\n"
            "  --plate-only <0|1>      Disable vehicle dependency for plate output (default: 1)\n"
            "  --sw-preproc <0|1>      Enable software preproc A/B path (default: 0)\n"
            "  --fpga-a-mask <0|1>     Enable FPGA A-channel ROI fusion (default: 0)\n"
            "  --a-proj-ratio <v>      A-channel projection threshold ratio (default: 0.35)\n"
            "  --a-roi-iou-min <v>     Min IoU for A-ROI filtering (default: 0.05)\n"
            "  --ped-event <0|1>       Enable pedestrian red-light event (default: 0)\n"
            "  --red-stable-frames <n> Red light debounce frames (default: 5)\n"
            "  --red-ratio-thr <v>     A-channel red ratio threshold (default: 0.002)\n"
            "  --stopline-ratio <v>    Stopline Y ratio [0,1] (default: 0.55)\n"
            "  --help                  Show this help\n",
            prog, DEFAULT_DEVICE, DEFAULT_DRM_CARD, DEFAULT_FPS, DEFAULT_TIMEOUT_MS,
            DEFAULT_STATS_INTERVAL, DEFAULT_COPY_BUFFERS, DEFAULT_QUEUE_DEPTH);
}

static int parse_options(int argc, char **argv, struct options *opt)
{
    static const struct option long_opts[] = {
        {"device", required_argument, NULL, 1},
        {"drm-card", required_argument, NULL, 2},
        {"veh-model", required_argument, NULL, 3},
        {"plate-model", required_argument, NULL, 4},
        {"ocr-model", required_argument, NULL, 5},
        {"ocr-keys", required_argument, NULL, 6},
        {"labels", required_argument, NULL, 7},
        {"pred-log", required_argument, NULL, 8},
        {"connector-id", required_argument, NULL, 9},
        {"fps", required_argument, NULL, 10},
        {"pixel-order", required_argument, NULL, 11},
        {"swap16", required_argument, NULL, 12},
        {"timeout-ms", required_argument, NULL, 13},
        {"stats-interval", required_argument, NULL, 14},
        {"copy-buffers", required_argument, NULL, 15},
        {"queue-depth", required_argument, NULL, 16},
        {"min-car-conf", required_argument, NULL, 17},
        {"min-plate-conf", required_argument, NULL, 18},
        {"plate-on-car-only", required_argument, NULL, 19},
        {"plate-only", required_argument, NULL, 20},
        {"sw-preproc", required_argument, NULL, 21},
        {"fpga-a-mask", required_argument, NULL, 22},
        {"a-proj-ratio", required_argument, NULL, 23},
        {"a-roi-iou-min", required_argument, NULL, 24},
        {"ped-event", required_argument, NULL, 25},
        {"red-stable-frames", required_argument, NULL, 26},
        {"red-ratio-thr", required_argument, NULL, 27},
        {"stopline-ratio", required_argument, NULL, 28},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };
    int c;

    memset(opt, 0, sizeof(*opt));
    opt->device_path = DEFAULT_DEVICE;
    opt->drm_card_path = DEFAULT_DRM_CARD;
    opt->connector_id = -1;
    opt->fps = DEFAULT_FPS;
    opt->pixel_order = PIXEL_ORDER_BGR565;
    opt->swap16 = true;
    opt->timeout_ms = DEFAULT_TIMEOUT_MS;
    opt->stats_interval = DEFAULT_STATS_INTERVAL;
    opt->copy_buffers = DEFAULT_COPY_BUFFERS;
    opt->queue_depth = DEFAULT_QUEUE_DEPTH;
    opt->min_car_conf = 0.35f;
    opt->min_plate_conf = 0.45f;
    opt->plate_on_car_only = 0;
    opt->plate_only = 1;
    opt->sw_preproc = 0;
    opt->fpga_a_mask = 0;
    opt->a_proj_ratio = 0.35f;
    opt->a_roi_iou_min = 0.05f;
    opt->ped_event = 0;
    opt->red_stable_frames = 5;
    opt->red_ratio_thr = 0.002f;
    opt->stopline_ratio = 0.55f;

    while ((c = getopt_long(argc, argv, "h", long_opts, NULL)) != -1) {
        switch (c) {
        case 1: opt->device_path = optarg; break;
        case 2: opt->drm_card_path = optarg; break;
        case 3: opt->veh_model_path = optarg; break;
        case 4: opt->plate_model_path = optarg; break;
        case 5: opt->ocr_model_path = optarg; break;
        case 6: opt->ocr_keys_path = optarg; break;
        case 7: opt->labels_path = optarg; break;
        case 8: opt->pred_log_path = optarg; break;
        case 9: opt->connector_id = atoi(optarg); break;
        case 10: opt->fps = atoi(optarg); break;
        case 11:
            if (strcmp(optarg, "bgr565") == 0)
                opt->pixel_order = PIXEL_ORDER_BGR565;
            else if (strcmp(optarg, "rgb565") == 0)
                opt->pixel_order = PIXEL_ORDER_RGB565;
            else
                return -1;
            break;
        case 12: opt->swap16 = atoi(optarg) ? true : false; break;
        case 13: opt->timeout_ms = atoi(optarg); break;
        case 14: opt->stats_interval = atoi(optarg); break;
        case 15: opt->copy_buffers = atoi(optarg); break;
        case 16: opt->queue_depth = atoi(optarg); break;
        case 17: opt->min_car_conf = (float)atof(optarg); break;
        case 18: opt->min_plate_conf = (float)atof(optarg); break;
        case 19: opt->plate_on_car_only = atoi(optarg) ? 1 : 0; break;
        case 20: opt->plate_only = atoi(optarg) ? 1 : 0; break;
        case 21: opt->sw_preproc = atoi(optarg) ? 1 : 0; break;
        case 22: opt->fpga_a_mask = atoi(optarg) ? 1 : 0; break;
        case 23: opt->a_proj_ratio = (float)atof(optarg); break;
        case 24: opt->a_roi_iou_min = (float)atof(optarg); break;
        case 25: opt->ped_event = atoi(optarg) ? 1 : 0; break;
        case 26: opt->red_stable_frames = atoi(optarg); break;
        case 27: opt->red_ratio_thr = (float)atof(optarg); break;
        case 28: opt->stopline_ratio = (float)atof(optarg); break;
        case 'h':
            print_usage(argv[0]);
            exit(0);
        default:
            return -1;
        }
    }

    if (opt->fps <= 0 || opt->timeout_ms <= 0 || opt->stats_interval <= 0)
        return -1;
    if (opt->copy_buffers < MIN_COPY_BUFFERS || opt->copy_buffers > MAX_COPY_BUFFERS)
        return -1;
    if (opt->queue_depth <= 0)
        return -1;
    if (opt->a_proj_ratio <= 0.0f || opt->a_proj_ratio >= 1.0f)
        return -1;
    if (opt->a_roi_iou_min < 0.0f || opt->a_roi_iou_min > 1.0f)
        return -1;
    if (opt->red_stable_frames <= 0 || opt->red_stable_frames > 120)
        return -1;
    if (opt->red_ratio_thr < 0.0f || opt->red_ratio_thr > 1.0f)
        return -1;
    if (opt->stopline_ratio <= 0.05f || opt->stopline_ratio >= 0.95f)
        return -1;
    if (!opt->veh_model_path || !opt->plate_model_path ||
        !opt->ocr_model_path || !opt->ocr_keys_path || !opt->labels_path)
        return -1;
    return 0;
}

static void decode_pixel565(const struct options *opt, uint8_t lo_in, uint8_t hi_in,
                            uint8_t *r8, uint8_t *g8, uint8_t *b8)
{
    uint8_t lo = lo_in;
    uint8_t hi = hi_in;
    uint16_t pix;
    uint8_t r5, g6, b5;
    if (opt->swap16) {
        uint8_t t = lo;
        lo = hi;
        hi = t;
    }
    pix = (uint16_t)lo | ((uint16_t)hi << 8);
    if (opt->pixel_order == PIXEL_ORDER_BGR565) {
        r5 = pix & 0x1F;
        g6 = (pix >> 5) & 0x3F;
        b5 = (pix >> 11) & 0x1F;
    } else {
        r5 = (pix >> 11) & 0x1F;
        g6 = (pix >> 5) & 0x3F;
        b5 = pix & 0x1F;
    }
    *r8 = (uint8_t)((r5 << 3) | (r5 >> 2));
    *g8 = (uint8_t)((g6 << 2) | (g6 >> 4));
    *b8 = (uint8_t)((b5 << 3) | (b5 >> 2));
}

static int load_labels(struct app_ctx *ctx, const char *path)
{
    FILE *fp = fopen(path, "r");
    char line[256];
    int idx = 0;
    if (!fp) {
        fprintf(stderr, "Open labels failed: %s\n", strerror(errno));
        return -1;
    }
    while (fgets(line, sizeof(line), fp) && idx < MAX_LABELS) {
        char *nl = strchr(line, '\n');
        size_t n;
        if (nl) *nl = '\0';
        if (line[0] == '\0')
            continue;
        n = strnlen(line, MAX_LABEL_LEN - 1);
        memcpy(ctx->labels[idx], line, n);
        ctx->labels[idx][n] = '\0';
        idx++;
    }
    fclose(fp);
    ctx->label_count = idx;
    ctx->car_class_id = 2;
    ctx->person_class_id = 0;
    for (idx = 0; idx < ctx->label_count; idx++) {
        if (strcmp(ctx->labels[idx], "car") == 0) {
            ctx->car_class_id = idx;
        } else if (strcmp(ctx->labels[idx], "person") == 0) {
            ctx->person_class_id = idx;
        }
    }
    return 0;
}

static int load_ocr_keys(struct app_ctx *ctx, const char *path)
{
    FILE *fp = fopen(path, "r");
    char line[256];
    int idx = 0;
    if (!fp) {
        fprintf(stderr, "Open OCR keys failed: %s\n", strerror(errno));
        return -1;
    }
    while (fgets(line, sizeof(line), fp) && idx < MAX_OCR_KEYS) {
        char *nl = strchr(line, '\n');
        size_t n;
        if (nl) *nl = '\0';
        nl = strchr(line, '\r');
        if (nl) *nl = '\0';
        if (line[0] == '\0' || line[0] == '#')
            continue;
        n = strnlen(line, MAX_OCR_KEY_LEN - 1);
        memcpy(ctx->ocr_keys[idx], line, n);
        ctx->ocr_keys[idx][n] = '\0';
        idx++;
    }
    fclose(fp);
    if (idx <= 0)
        return -1;
    ctx->ocr_key_count = idx;
    ctx->ocr_blank_index = idx;
    fprintf(stderr, "[ocr] loaded %d keys from %s\n", ctx->ocr_key_count, path);
    return 0;
}

static int rknn_ocr_model_load(struct ocr_model *m, const char *name, const char *path)
{
    FILE *fp;
    long sz;
    void *data;
    uint32_t i;
    memset(m, 0, sizeof(*m));
    m->name = name;
    m->path = path;

    fp = fopen(path, "rb");
    if (!fp) return -1;
    fseek(fp, 0, SEEK_END);
    sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    data = malloc((size_t)sz);
    if (!data) { fclose(fp); return -1; }
    if (fread(data, 1, (size_t)sz, fp) != (size_t)sz) { fclose(fp); free(data); return -1; }
    fclose(fp);

    if (rknn_init(&m->ctx, data, (uint32_t)sz, 0, NULL) < 0) { free(data); return -1; }
    free(data);
    if (rknn_query(m->ctx, RKNN_QUERY_IN_OUT_NUM, &m->io_num, sizeof(m->io_num)) < 0)
        return -1;
    if (m->io_num.n_output == 0 || m->io_num.n_output > 4)
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

    fprintf(stderr, "[%s] loaded input=%ux%ux%u outputs=%u\n",
            name, m->in_w, m->in_h, m->in_c, m->io_num.n_output);
    return 0;
}

static void rknn_ocr_model_release(struct ocr_model *m)
{
    if (m->ctx)
        rknn_destroy(m->ctx);
    memset(m, 0, sizeof(*m));
}

static bool build_ocr_layout(const rknn_tensor_attr *a, int *t_size, int *c_size, int *t_stride, int *c_stride)
{
    if (a->n_dims == 2) {
        *t_size = (int)a->dims[0];
        *c_size = (int)a->dims[1];
        *t_stride = *c_size;
        *c_stride = 1;
        return (*t_size > 0 && *c_size > 1);
    }
    if (a->n_dims == 3) {
        int d1 = (int)a->dims[1];
        int d2 = (int)a->dims[2];
        if (d1 <= 0 || d2 <= 1)
            return false;
        if (d1 <= d2) {
            *t_size = d1;
            *c_size = d2;
            *t_stride = *c_size;
            *c_stride = 1;
        } else {
            *t_size = d2;
            *c_size = d1;
            *t_stride = 1;
            *c_stride = *t_size;
        }
        return true;
    }
    if (a->n_dims == 4) {
        if (a->fmt == RKNN_TENSOR_NCHW) {
            *c_size = (int)a->dims[1];
            *t_size = (int)a->dims[2] * (int)a->dims[3];
            *t_stride = 1;
            *c_stride = *t_size;
        } else {
            *t_size = (int)a->dims[1] * (int)a->dims[2];
            *c_size = (int)a->dims[3];
            *t_stride = *c_size;
            *c_stride = 1;
        }
        return (*t_size > 0 && *c_size > 1);
    }
    return false;
}

static int ctc_decode_logits(const float *buf, int t_size, int c_size, int t_stride, int c_stride,
                             struct app_ctx *ctx, char *text, size_t text_len, float *conf_out)
{
    int t;
    int prev = -1;
    int emitted = 0;
    float conf_sum = 0.0f;
    int blank_idx = ctx->ocr_blank_index;

    if (text_len == 0)
        return -1;
    text[0] = '\0';

    if (blank_idx < 0 || blank_idx >= c_size) {
        if (c_size == ctx->ocr_key_count + 1)
            blank_idx = ctx->ocr_key_count;
        else
            blank_idx = c_size - 1;
    }

    for (t = 0; t < t_size; t++) {
        int c;
        int best_c = 0;
        float best_logit = -1e30f;
        float max_logit = -1e30f;
        float exp_sum = 0.0f;
        const float *row = buf + (size_t)t * (size_t)t_stride;

        for (c = 0; c < c_size; c++) {
            float v = row[(size_t)c * (size_t)c_stride];
            if (v > best_logit) {
                best_logit = v;
                best_c = c;
            }
            if (v > max_logit)
                max_logit = v;
        }
        for (c = 0; c < c_size; c++) {
            float v = row[(size_t)c * (size_t)c_stride];
            exp_sum += expf(v - max_logit);
        }
        if (best_c == blank_idx || best_c == prev) {
            prev = best_c;
            continue;
        }
        if (best_c >= 0 && best_c < ctx->ocr_key_count) {
            float prob = 0.0f;
            size_t left;
            exp_sum = (exp_sum > 1e-8f) ? exp_sum : 1e-8f;
            prob = expf(best_logit - max_logit) / exp_sum;
            left = text_len - strnlen(text, text_len);
            if (left > 1) {
                strncat(text, ctx->ocr_keys[best_c], left - 1);
                emitted++;
                conf_sum += prob;
            }
        }
        prev = best_c;
    }
    *conf_out = (emitted > 0) ? (conf_sum / (float)emitted) : 0.0f;
    return 0;
}

static int run_model_ocr(struct app_ctx *ctx, const uint8_t *crop_rgb, int crop_w, int crop_h,
                         char *text, size_t text_len, float *conf_out)
{
    struct ocr_model *m = &ctx->ocr_model;
    rknn_input in;
    rknn_output outs[4];
    uint8_t *ocr_in = NULL;
    const rknn_tensor_attr *out_attr;
    int t_size, c_size, t_stride, c_stride;
    int ret = -1;
    uint32_t i;

    ocr_in = malloc((size_t)m->in_w * m->in_h * 3U);
    if (!ocr_in)
        return -1;

    resize_rgb888_nn(crop_rgb, crop_w, crop_h, ocr_in, (int)m->in_w, (int)m->in_h);
    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf = (void *)ocr_in;
    in.size = m->in_w * m->in_h * 3;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    ret = rknn_inputs_set(m->ctx, 1, &in);
    if (ret < 0)
        goto out;
    ret = rknn_run(m->ctx, NULL);
    if (ret < 0)
        goto out;

    memset(outs, 0, sizeof(outs));
    for (i = 0; i < m->io_num.n_output; i++)
        outs[i].want_float = 1;
    ret = rknn_outputs_get(m->ctx, m->io_num.n_output, outs, NULL);
    if (ret < 0)
        goto out;

    out_attr = &m->output_attrs[0];
    if (!build_ocr_layout(out_attr, &t_size, &c_size, &t_stride, &c_stride)) {
        ret = -1;
        goto out_release;
    }
    if (ctx->ocr_blank_index < 0 || ctx->ocr_blank_index >= c_size) {
        if (c_size == ctx->ocr_key_count + 1)
            ctx->ocr_blank_index = ctx->ocr_key_count;
        else
            ctx->ocr_blank_index = c_size - 1;
    }
    ret = ctc_decode_logits((const float *)outs[0].buf, t_size, c_size, t_stride, c_stride,
                            ctx, text, text_len, conf_out);

out_release:
    rknn_outputs_release(m->ctx, m->io_num.n_output, outs);
out:
    free(ocr_in);
    return ret;
}

static int init_fpga_dma(struct app_ctx *ctx)
{
    struct fpga_info info;
    struct buffer_map map;
    uint32_t inferred_format;

    ctx->dev_fd = open(ctx->opt.device_path, O_RDWR | O_CLOEXEC);
    if (ctx->dev_fd < 0)
        return -1;
    if (ioctl(ctx->dev_fd, FPGA_DMA_GET_INFO, &info) < 0)
        return -1;
    if (info.frame_width != 1280 || info.frame_height != 720)
        return -1;

    inferred_format = info.pixel_format;

    if (inferred_format != FPGA_PIXEL_FORMAT_BGR565 &&
        inferred_format != FPGA_PIXEL_FORMAT_BGRX8888) {
        if (info.frame_bpp == 4)
            inferred_format = FPGA_PIXEL_FORMAT_BGRX8888;
        else
            inferred_format = FPGA_PIXEL_FORMAT_BGR565;
    }
    if (inferred_format == FPGA_PIXEL_FORMAT_BGRX8888)
        info.frame_bpp = 4;
    else
        info.frame_bpp = 2;

    ctx->frame_width = info.frame_width;
    ctx->frame_height = info.frame_height;
    ctx->src_frame_bpp = info.frame_bpp;
    ctx->src_is_bgrx = (inferred_format == FPGA_PIXEL_FORMAT_BGRX8888) || (info.frame_bpp == 4);
    ctx->src_frame_size = (size_t)ctx->frame_width * ctx->frame_height * ctx->src_frame_bpp;
    /* Internal pipeline keeps BGR565 for overlay and drawing. */
    ctx->frame_bpp = 2;
    ctx->frame_size = (size_t)ctx->frame_width * ctx->frame_height * ctx->frame_bpp;

    if (ctx->src_is_bgrx)
        ctx->opt.swap16 = false;

    memset(&map, 0, sizeof(map));
    map.index = 0;
    if (ioctl(ctx->dev_fd, FPGA_DMA_MAP_BUFFER, &map) < 0)
        return -1;
    if (map.size < ctx->src_frame_size)
        return -1;
    ctx->dma_map_size = map.size;
    ctx->dma_map = mmap(NULL, ctx->dma_map_size, PROT_READ, MAP_SHARED, ctx->dev_fd, 0);
    if (ctx->dma_map == MAP_FAILED) {
        ctx->dma_map = NULL;
        return -1;
    }

    ctx->dma_copy = malloc(ctx->src_frame_size);
    if (!ctx->dma_copy)
        return -1;
    return 0;
}

static int trigger_frame_dma(struct app_ctx *ctx)
{
    struct dma_transfer t;
    memset(&t, 0, sizeof(t));
    t.size = (uint32_t)ctx->src_frame_size;
    t.user_buf = (uint64_t)(uintptr_t)ctx->dma_copy;
    if (ioctl(ctx->dev_fd, FPGA_DMA_READ_FRAME, &t) < 0)
        return -1;
    return t.result == 0 ? 0 : -1;
}

static int init_copy_slots(struct app_ctx *ctx)
{
    int i;
    ctx->slots = calloc((size_t)ctx->opt.copy_buffers, sizeof(*ctx->slots));
    if (!ctx->slots)
        return -1;
    ctx->slot_count = ctx->opt.copy_buffers;
    for (i = 0; i < ctx->slot_count; i++) {
        ctx->slots[i].data = malloc(ctx->frame_size);
        if (!ctx->slots[i].data)
            return -1;
    }
    return 0;
}

static void copy_frame_to_slot565(struct app_ctx *ctx, uint8_t *dst, const uint8_t *src)
{
    size_t i;
    size_t pixels = (size_t)ctx->frame_width * ctx->frame_height;

    if (ctx->src_is_bgrx) {
        for (i = 0; i < pixels; i++) {
            const uint8_t *p = src + i * 4U;
            uint8_t b = p[0];
            uint8_t g = p[1];
            uint8_t r = p[2];
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

    if (!ctx->opt.swap16) {
        memcpy(dst, src, ctx->frame_size);
        return;
    }
    for (i = 0; i + 1 < ctx->frame_size; i += 2) {
        dst[i] = src[i + 1];
        dst[i + 1] = src[i];
    }
}

static void draw_hline_565(uint16_t *pix, int w, int h, int x1, int x2, int y, uint16_t c)
{
    int x;
    if (y < 0 || y >= h)
        return;
    if (x1 > x2) { int t = x1; x1 = x2; x2 = t; }
    if (x1 < 0) x1 = 0;
    if (x2 >= w) x2 = w - 1;
    for (x = x1; x <= x2; x++)
        pix[y * w + x] = c;
}

static void draw_vline_565(uint16_t *pix, int w, int h, int x, int y1, int y2, uint16_t c)
{
    int y;
    if (x < 0 || x >= w)
        return;
    if (y1 > y2) { int t = y1; y1 = y2; y2 = t; }
    if (y1 < 0) y1 = 0;
    if (y2 >= h) y2 = h - 1;
    for (y = y1; y <= y2; y++)
        pix[y * w + x] = c;
}

static void draw_rect_565(uint16_t *pix, int w, int h, const struct det_box *b, uint16_t c)
{
    int t;
    for (t = 0; t < 2; t++) {
        draw_hline_565(pix, w, h, b->x1, b->x2, b->y1 + t, c);
        draw_hline_565(pix, w, h, b->x1, b->x2, b->y2 - t, c);
        draw_vline_565(pix, w, h, b->x1 + t, b->y1, b->y2, c);
        draw_vline_565(pix, w, h, b->x2 - t, b->y1, b->y2, c);
    }
}

static uint8_t glyph5x7(char ch, int row)
{
    switch (ch) {
    case 'B': { static const uint8_t g[7]={0x1E,0x11,0x1E,0x11,0x11,0x11,0x1E}; return g[row]; }
    case 'L': { static const uint8_t g[7]={0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return g[row]; }
    case 'U': { static const uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x11,0x1F}; return g[row]; }
    case 'E': { static const uint8_t g[7]={0x1F,0x10,0x1E,0x10,0x10,0x10,0x1F}; return g[row]; }
    case 'G': { static const uint8_t g[7]={0x0F,0x10,0x10,0x13,0x11,0x11,0x0F}; return g[row]; }
    case 'R': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return g[row]; }
    case 'N': { static const uint8_t g[7]={0x11,0x19,0x15,0x13,0x11,0x11,0x11}; return g[row]; }
    case 'K': { static const uint8_t g[7]={0x11,0x12,0x14,0x18,0x14,0x12,0x11}; return g[row]; }
    default: return 0;
    }
}

static void draw_text_565(uint16_t *pix, int w, int h, int x, int y, const char *s, uint16_t c)
{
    int i;
    for (i = 0; s[i] != '\0'; i++) {
        int row;
        int col;
        int ox = x + i * 6;
        for (row = 0; row < 7; row++) {
            uint8_t bits = glyph5x7(s[i], row);
            for (col = 0; col < 5; col++) {
                if (bits & (1U << (4 - col))) {
                    int px = ox + col;
                    int py = y + row;
                    if (px >= 0 && px < w && py >= 0 && py < h)
                        pix[py * w + px] = c;
                }
            }
        }
    }
}

static void release_slot_ticket(struct app_ctx *ctx, const struct slot_ticket *ticket, bool count_release)
{
    g_mutex_lock(&ctx->slots_lock);
    if (ticket->idx >= 0 && ticket->idx < ctx->slot_count &&
        ctx->slots[ticket->idx].in_use &&
        ctx->slots[ticket->idx].generation == ticket->generation) {
        ctx->slots[ticket->idx].in_use = false;
        if (count_release)
            ctx->released_frames++;
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
        int i;
        int64_t now;
        int64_t wake_us;
        g_mutex_lock(&ctx->slots_lock);
        for (i = 0; i < ctx->slot_count; i++) {
            if (!ctx->slots[i].in_use) {
                ctx->slots[i].in_use = true;
                ctx->slots[i].generation++;
                ticket->idx = i;
                ticket->generation = ctx->slots[i].generation;
                g_mutex_unlock(&ctx->slots_lock);
                return 0;
            }
        }
        now = mono_us();
        if (now >= deadline_us) {
            g_mutex_unlock(&ctx->slots_lock);
            ctx->slot_wait_timeout_count++;
            return -1;
        }
        wake_us = now + 20000;
        if (wake_us > deadline_us)
            wake_us = deadline_us;
        g_cond_wait_until(&ctx->slots_cond, &ctx->slots_lock, wake_us);
        g_mutex_unlock(&ctx->slots_lock);
    }
}

static GstBuffer *build_frame_buffer(struct app_ctx *ctx, const struct slot_ticket *ticket)
{
    struct frame_cookie *cookie = g_new0(struct frame_cookie, 1);
    GstBuffer *buf;
    if (!cookie)
        return NULL;
    cookie->ctx = ctx;
    cookie->ticket = *ticket;
    buf = gst_buffer_new_wrapped_full((GstMemoryFlags)0,
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
            if (dbg)
                fprintf(stderr, "  debug: %s\n", dbg);
            if (err)
                g_error_free(err);
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

static void print_pad_caps(const char *label, GstElement *elem, const char *pad_name)
{
    GstPad *pad = gst_element_get_static_pad(elem, pad_name);
    GstCaps *caps;
    gchar *s;
    if (!pad)
        return;
    caps = gst_pad_get_current_caps(pad);
    if (!caps)
        caps = gst_pad_query_caps(pad, NULL);
    if (caps) {
        s = gst_caps_to_string(caps);
        fprintf(stderr, "%s caps: %s\n", label, s ? s : "<null>");
        g_free(s);
        gst_caps_unref(caps);
    }
    gst_object_unref(pad);
}

static int build_pipeline(struct app_ctx *ctx)
{
    GstCaps *caps;
    GstStateChangeReturn sret;
    const char *fmt = (ctx->opt.pixel_order == PIXEL_ORDER_BGR565) ? "BGR16" : "RGB16";
    ctx->pipeline = gst_pipeline_new("fpga-lpr");
    ctx->appsrc = gst_element_factory_make("appsrc", "src");
    ctx->queue = gst_element_factory_make("queue", "latency_queue");
    ctx->sink = gst_element_factory_make("kmssink", "sink");
    if (!ctx->pipeline || !ctx->appsrc || !ctx->queue || !ctx->sink)
        return -1;

    gst_bin_add_many(GST_BIN(ctx->pipeline), ctx->appsrc, ctx->queue, ctx->sink, NULL);
    if (!gst_element_link_many(ctx->appsrc, ctx->queue, ctx->sink, NULL))
        return -1;

    caps = gst_caps_new_simple("video/x-raw",
                               "format", G_TYPE_STRING, fmt,
                               "width", G_TYPE_INT, (int)ctx->frame_width,
                               "height", G_TYPE_INT, (int)ctx->frame_height,
                               "framerate", GST_TYPE_FRACTION, ctx->opt.fps, 1,
                               NULL);
    if (!caps)
        return -1;
    g_object_set(ctx->appsrc,
                 "caps", caps,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "format", GST_FORMAT_TIME,
                 "block", FALSE,
                 "max-bytes", (guint64)ctx->frame_size * (guint64)ctx->opt.queue_depth,
                 NULL);
    gst_caps_unref(caps);

    g_object_set(ctx->queue,
                 "max-size-buffers", 1,
                 "max-size-bytes", 0,
                 "max-size-time", (guint64)0,
                 "leaky", 2,
                 NULL);
    g_object_set(ctx->sink, "sync", FALSE, NULL);
    if (ctx->opt.connector_id >= 0)
        g_object_set(ctx->sink, "connector-id", ctx->opt.connector_id, NULL);
    if (ctx->drm_fd >= 0)
        g_object_set(ctx->sink, "fd", ctx->drm_fd, NULL);

    ctx->bus = gst_element_get_bus(ctx->pipeline);
    sret = gst_element_set_state(ctx->pipeline, GST_STATE_PLAYING);
    if (sret == GST_STATE_CHANGE_FAILURE)
        return -1;
    sret = gst_element_get_state(ctx->pipeline, NULL, NULL, 5 * GST_SECOND);
    if (sret == GST_STATE_CHANGE_FAILURE)
        return -1;
    fprintf(stderr, "Pipeline started: appsrc(%s)->queue(leaky)->kmssink\n", fmt);
    print_pad_caps("appsrc:src", ctx->appsrc, "src");
    print_pad_caps("kmssink:sink", ctx->sink, "sink");
    return 0;
}

static float sigmoidf_local(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

static float box_iou(const struct det_box *a, const struct det_box *b)
{
    int x1 = a->x1 > b->x1 ? a->x1 : b->x1;
    int y1 = a->y1 > b->y1 ? a->y1 : b->y1;
    int x2 = a->x2 < b->x2 ? a->x2 : b->x2;
    int y2 = a->y2 < b->y2 ? a->y2 : b->y2;
    int iw = x2 - x1 + 1;
    int ih = y2 - y1 + 1;
    int ia;
    int ua;
    if (iw <= 0 || ih <= 0)
        return 0.0f;
    ia = iw * ih;
    ua = (a->x2 - a->x1 + 1) * (a->y2 - a->y1 + 1) +
         (b->x2 - b->x1 + 1) * (b->y2 - b->y1 + 1) - ia;
    if (ua <= 0)
        return 0.0f;
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
    bool removed[MAX_DETS];
    int i;
    int j;
    int out = 0;
    memset(removed, 0, sizeof(removed));
    qsort(dets, (size_t)(*count), sizeof(dets[0]), det_conf_cmp);
    for (i = 0; i < *count; i++) {
        if (removed[i]) continue;
        dets[out++] = dets[i];
        for (j = i + 1; j < *count; j++) {
            if (removed[j] || dets[i].cls != dets[j].cls)
                continue;
            if (box_iou(&dets[i], &dets[j]) > iou_thr)
                removed[j] = true;
        }
    }
    *count = out;
}

static const char *tensor_fmt_name(rknn_tensor_format fmt)
{
    switch (fmt) {
    case RKNN_TENSOR_NCHW: return "NCHW";
    case RKNN_TENSOR_NHWC: return "NHWC";
    default: return "UNSPEC";
    }
}

static const char *tensor_type_name(rknn_tensor_type t)
{
    switch (t) {
#ifdef RKNN_TENSOR_FLOAT32
    case RKNN_TENSOR_FLOAT32: return "f32";
#endif
#ifdef RKNN_TENSOR_FLOAT16
    case RKNN_TENSOR_FLOAT16: return "f16";
#endif
#ifdef RKNN_TENSOR_INT8
    case RKNN_TENSOR_INT8: return "i8";
#endif
#ifdef RKNN_TENSOR_UINT8
    case RKNN_TENSOR_UINT8: return "u8";
#endif
#ifdef RKNN_TENSOR_INT16
    case RKNN_TENSOR_INT16: return "i16";
#endif
#ifdef RKNN_TENSOR_UINT16
    case RKNN_TENSOR_UINT16: return "u16";
#endif
#ifdef RKNN_TENSOR_INT32
    case RKNN_TENSOR_INT32: return "i32";
#endif
#ifdef RKNN_TENSOR_UINT32
    case RKNN_TENSOR_UINT32: return "u32";
#endif
    default: return "other";
    }
}

static int rknn_model_load(struct yolo_model *m, const char *name, const char *path, int class_count)
{
    FILE *fp;
    long sz;
    void *data;
    uint32_t i;
    memset(m, 0, sizeof(*m));
    m->name = name;
    m->path = path;
    m->class_count = class_count;

    fp = fopen(path, "rb");
    if (!fp) return -1;
    fseek(fp, 0, SEEK_END);
    sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    data = malloc((size_t)sz);
    if (!data) { fclose(fp); return -1; }
    if (fread(data, 1, (size_t)sz, fp) != (size_t)sz) { fclose(fp); free(data); return -1; }
    fclose(fp);

    if (rknn_init(&m->ctx, data, (uint32_t)sz, 0, NULL) < 0) { free(data); return -1; }
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
    for (i = 0; i < m->io_num.n_output; i++) {
        memset(&m->output_attrs[i], 0, sizeof(m->output_attrs[i]));
        m->output_attrs[i].index = i;
        if (rknn_query(m->ctx, RKNN_QUERY_OUTPUT_ATTR, &m->output_attrs[i], sizeof(m->output_attrs[i])) < 0)
            return -1;
    }
    fprintf(stderr, "[%s] loaded input=%ux%ux%u outputs=%u\n",
            name, m->in_w, m->in_h, m->in_c, m->io_num.n_output);
    for (i = 0; i < m->io_num.n_output; i++) {
        const rknn_tensor_attr *a = &m->output_attrs[i];
        fprintf(stderr,
                "  out[%u]: dims=%u x %u x %u x %u n_dims=%u fmt=%s type=%s qnt=%d zp=%d scale=%.6f\n",
                i,
                a->dims[0], a->dims[1], a->dims[2], a->dims[3], a->n_dims,
                tensor_fmt_name(a->fmt), tensor_type_name(a->type), a->qnt_type, a->zp, a->scale);
    }
    return 0;
}

static void rknn_model_release(struct yolo_model *m)
{
    if (m->ctx)
        rknn_destroy(m->ctx);
    memset(m, 0, sizeof(*m));
}

static void resize_rgb888_nn(const uint8_t *src, int sw, int sh, uint8_t *dst, int dw, int dh)
{
    int x, y;
    for (y = 0; y < dh; y++) {
        int sy = (y * sh) / dh;
        for (x = 0; x < dw; x++) {
            int sx = (x * sw) / dw;
            const uint8_t *p = src + (sy * sw + sx) * 3;
            uint8_t *q = dst + (y * dw + x) * 3;
            q[0] = p[0];
            q[1] = p[1];
            q[2] = p[2];
        }
    }
}

static void raw565_to_rgb888_full(struct app_ctx *ctx, const uint8_t *raw, uint8_t *rgb)
{
    size_t i;
    size_t pixels = (size_t)ctx->frame_width * ctx->frame_height;
    for (i = 0; i < pixels; i++) {
        uint8_t r, g, b;
        decode_pixel565(&ctx->opt, raw[i * 2], raw[i * 2 + 1], &r, &g, &b);
        rgb[i * 3 + 0] = r;
        rgb[i * 3 + 1] = g;
        rgb[i * 3 + 2] = b;
    }
}

static void bgrx8888_to_rgb888_and_a(const uint8_t *src_bgrx, int w, int h,
                                     uint8_t *dst_rgb, uint8_t *dst_a)
{
    size_t i;
    size_t pixels = (size_t)w * h;
    for (i = 0; i < pixels; i++) {
        const uint8_t *p = src_bgrx + i * 4U;
        uint8_t *q = dst_rgb + i * 3U;
        q[0] = p[2];
        q[1] = p[1];
        q[2] = p[0];
        if (dst_a)
            dst_a[i] = p[3];
    }
}

static inline uint8_t clip_u8(int v)
{
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

static void sw_preprocess_rgb888(uint8_t *rgb, int w, int h)
{
    size_t pixels = (size_t)w * h;
    uint8_t *gray = malloc(pixels);
    uint8_t *filt = malloc(pixels);
    uint8_t *edge = malloc(pixels);
    int x, y;
    if (!gray || !filt || !edge) {
        free(gray);
        free(filt);
        free(edge);
        return;
    }

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            const uint8_t *p = rgb + ((size_t)y * w + x) * 3U;
            int g = (77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8;
            gray[(size_t)y * w + x] = (uint8_t)g;
        }
    }
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            int s = 0;
            int c = 0;
            int ky, kx;
            for (ky = -1; ky <= 1; ky++) {
                int yy = y + ky;
                if (yy < 0 || yy >= h)
                    continue;
                for (kx = -1; kx <= 1; kx++) {
                    int xx = x + kx;
                    if (xx < 0 || xx >= w)
                        continue;
                    s += gray[(size_t)yy * w + xx];
                    c++;
                }
            }
            filt[(size_t)y * w + x] = (uint8_t)(s / (c > 0 ? c : 1));
        }
    }
    for (y = 1; y < h - 1; y++) {
        for (x = 1; x < w - 1; x++) {
            int gx =
                -filt[(size_t)(y - 1) * w + (x - 1)] + filt[(size_t)(y - 1) * w + (x + 1)] +
                -2 * filt[(size_t)y * w + (x - 1)]     + 2 * filt[(size_t)y * w + (x + 1)] +
                -filt[(size_t)(y + 1) * w + (x - 1)] + filt[(size_t)(y + 1) * w + (x + 1)];
            int gy =
                -filt[(size_t)(y - 1) * w + (x - 1)] - 2 * filt[(size_t)(y - 1) * w + x] - filt[(size_t)(y - 1) * w + (x + 1)] +
                 filt[(size_t)(y + 1) * w + (x - 1)] + 2 * filt[(size_t)(y + 1) * w + x] + filt[(size_t)(y + 1) * w + (x + 1)];
            int mag = (abs(gx) + abs(gy)) >> 2;
            edge[(size_t)y * w + x] = clip_u8(mag);
        }
    }
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            uint8_t g = filt[(size_t)y * w + x];
            uint8_t e = edge[(size_t)y * w + x];
            uint8_t enh = clip_u8((int)g + ((int)e >> 1));
            uint8_t *q = rgb + ((size_t)y * w + x) * 3U;
            q[0] = enh;
            q[1] = enh;
            q[2] = enh;
        }
    }

    free(gray);
    free(filt);
    free(edge);
}

static int extract_a_channel_roi(const uint8_t *a_map, int w, int h, float proj_ratio,
                                 struct det_box *roi, float *red_ratio_out)
{
    int *hist_x = calloc((size_t)w, sizeof(int));
    int *hist_y = calloc((size_t)h, sizeof(int));
    int x, y;
    int edge_total = 0;
    int valid_total = 0;
    int red_total = 0;
    int max_x = 0, max_y = 0;
    int x1 = -1, x2 = -1, y1 = -1, y2 = -1;

    if (!hist_x || !hist_y) {
        free(hist_x);
        free(hist_y);
        return 0;
    }

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            size_t idx = (size_t)y * w + x;
            uint8_t a = a_map[idx];
            uint8_t preproc_valid = (a >> 7) & 0x1;
            uint8_t edge_flag = (a >> 6) & 0x1;
            uint8_t color_class = (a >> 2) & 0x3;
            if (x == 0 && y == 0)
                continue; /* Pixel[0,0] may be watermark */
            if (!preproc_valid)
                continue;
            valid_total++;
            if (color_class == 0x3)
                red_total++;
            if (edge_flag) {
                hist_x[x]++;
                hist_y[y]++;
                edge_total++;
            }
        }
    }
    if (red_ratio_out) {
        *red_ratio_out = (valid_total > 0) ? ((float)red_total / (float)valid_total) : 0.0f;
    }
    if (edge_total <= 0) {
        free(hist_x);
        free(hist_y);
        return 0;
    }

    for (x = 0; x < w; x++)
        if (hist_x[x] > max_x) max_x = hist_x[x];
    for (y = 0; y < h; y++)
        if (hist_y[y] > max_y) max_y = hist_y[y];

    if (max_x <= 0 || max_y <= 0) {
        free(hist_x);
        free(hist_y);
        return 0;
    }

    {
        int tx = (int)((float)max_x * proj_ratio);
        int ty = (int)((float)max_y * proj_ratio);
        for (x = 0; x < w; x++) {
            if (hist_x[x] >= tx) {
                if (x1 < 0) x1 = x;
                x2 = x;
            }
        }
        for (y = 0; y < h; y++) {
            if (hist_y[y] >= ty) {
                if (y1 < 0) y1 = y;
                y2 = y;
            }
        }
    }

    free(hist_x);
    free(hist_y);

    if (x1 < 0 || y1 < 0 || x2 <= x1 || y2 <= y1)
        return 0;

    roi->x1 = x1;
    roi->y1 = y1;
    roi->x2 = x2;
    roi->y2 = y2;
    roi->conf = 1.0f;
    roi->cls = 0;
    return 1;
}

static int filter_boxes_by_roi(const struct det_box *in, int in_count, const struct det_box *roi,
                               float iou_thr, struct det_box *out, int out_cap)
{
    int i;
    int n = 0;
    for (i = 0; i < in_count && n < out_cap; i++) {
        const struct det_box *b = &in[i];
        int cx = (b->x1 + b->x2) / 2;
        int cy = (b->y1 + b->y2) / 2;
        bool center_inside = (cx >= roi->x1 && cx <= roi->x2 && cy >= roi->y1 && cy <= roi->y2);
        if (center_inside || box_iou(b, roi) >= iou_thr)
            out[n++] = *b;
    }
    return n;
}

static int update_ped_tracks_nn(struct app_ctx *ctx,
                                const struct det_box *persons, int person_count,
                                bool light_red, uint64_t frame_seq,
                                struct det_box *out_persons, int *out_person_count)
{
    int i;
    int j;
    int events = 0;
    int stopline_y = (int)((float)ctx->frame_height * ctx->opt.stopline_ratio);
    bool used_det[MAX_DETS] = {0};
    (void)frame_seq;

    for (i = 0; i < ctx->ped_track_count; i++)
        ctx->ped_track_ttl[i]--;

    for (i = 0; i < ctx->ped_track_count; i++) {
        int best = -1;
        int best_d2 = 0x7fffffff;
        int tcx = (ctx->ped_track_box[i].x1 + ctx->ped_track_box[i].x2) / 2;
        int tcy = (ctx->ped_track_box[i].y1 + ctx->ped_track_box[i].y2) / 2;
        for (j = 0; j < person_count; j++) {
            int dcx, dcy, d2;
            if (used_det[j])
                continue;
            dcx = (persons[j].x1 + persons[j].x2) / 2 - tcx;
            dcy = (persons[j].y1 + persons[j].y2) / 2 - tcy;
            d2 = dcx * dcx + dcy * dcy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best = j;
            }
        }
        if (best >= 0 && best_d2 <= (96 * 96)) {
            int old_cy = (ctx->ped_track_box[i].y1 + ctx->ped_track_box[i].y2) / 2;
            int new_cy = (persons[best].y1 + persons[best].y2) / 2;
            bool crossed = (old_cy < stopline_y) && (new_cy >= stopline_y);
            ctx->ped_track_box[i] = persons[best];
            ctx->ped_track_ttl[i] = 8;
            used_det[best] = true;
            if (light_red && crossed) {
                events++;
            }
        }
    }

    for (j = 0; j < person_count && ctx->ped_track_count < MAX_DETS; j++) {
        if (used_det[j])
            continue;
        ctx->ped_track_box[ctx->ped_track_count] = persons[j];
        ctx->ped_track_id[ctx->ped_track_count] = ctx->ped_next_track_id++;
        ctx->ped_track_ttl[ctx->ped_track_count] = 8;
        ctx->ped_track_count++;
    }

    i = 0;
    while (i < ctx->ped_track_count) {
        if (ctx->ped_track_ttl[i] > 0) {
            i++;
            continue;
        }
        ctx->ped_track_box[i] = ctx->ped_track_box[ctx->ped_track_count - 1];
        ctx->ped_track_id[i] = ctx->ped_track_id[ctx->ped_track_count - 1];
        ctx->ped_track_ttl[i] = ctx->ped_track_ttl[ctx->ped_track_count - 1];
        ctx->ped_track_count--;
    }

    *out_person_count = 0;
    for (i = 0; i < person_count && *out_person_count < MAX_DETS; i++)
        out_persons[(*out_person_count)++] = persons[i];

    return events;
}

static void clamp_box(struct det_box *b, int w, int h)
{
    if (b->x1 < 0) b->x1 = 0;
    if (b->y1 < 0) b->y1 = 0;
    if (b->x2 >= w) b->x2 = w - 1;
    if (b->y2 >= h) b->y2 = h - 1;
    if (b->x2 < b->x1) b->x2 = b->x1;
    if (b->y2 < b->y1) b->y2 = b->y1;
}

static void map_box_between_spaces(struct det_box *b, int src_w, int src_h, int dst_w, int dst_h)
{
    b->x1 = (int)((int64_t)b->x1 * dst_w / src_w);
    b->x2 = (int)((int64_t)b->x2 * dst_w / src_w);
    b->y1 = (int)((int64_t)b->y1 * dst_h / src_h);
    b->y2 = (int)((int64_t)b->y2 * dst_h / src_h);
    clamp_box(b, dst_w, dst_h);
}

static void compute_center_crop_box(const struct det_box *src, int img_w, int img_h,
                                    int crop_w, int crop_h, struct det_box *crop)
{
    int cx = (src->x1 + src->x2) / 2;
    int cy = (src->y1 + src->y2) / 2;
    int x1 = cx - crop_w / 2;
    int y1 = cy - crop_h / 2;

    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x1 + crop_w > img_w) x1 = img_w - crop_w;
    if (y1 + crop_h > img_h) y1 = img_h - crop_h;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;

    crop->x1 = x1;
    crop->y1 = y1;
    crop->x2 = x1 + crop_w - 1;
    crop->y2 = y1 + crop_h - 1;
    clamp_box(crop, img_w, img_h);
}

static void copy_crop_rgb888(const uint8_t *rgb, int img_w, const struct det_box *crop, uint8_t *dst)
{
    int y;
    int crop_w = crop->x2 - crop->x1 + 1;
    int crop_h = crop->y2 - crop->y1 + 1;
    for (y = 0; y < crop_h; y++) {
        const uint8_t *src_row = rgb + ((crop->y1 + y) * img_w + crop->x1) * 3;
        uint8_t *dst_row = dst + y * crop_w * 3;
        memcpy(dst_row, src_row, (size_t)crop_w * 3U);
    }
}

static void decode_rows_output(const float *rows, int n_rows, int n_cols, int class_count,
                               float conf_thr, int src_w, int src_h, int in_w, int in_h,
                               struct det_box *out, int *out_count)
{
    int i;
    int cls_lim = class_count;
    if (n_cols < 6)
        return;
    if (cls_lim > n_cols - 5)
        cls_lim = n_cols - 5;
    for (i = 0; i < n_rows && *out_count < MAX_DETS; i++) {
        const float *r = rows + i * n_cols;
        float obj = r[4];
        float best = (cls_lim > 0) ? 0.0f : 1.0f;
        int best_id = 0;
        int c;
        float cx = r[0], cy = r[1], bw = r[2], bh = r[3];
        struct det_box b;
        for (c = 0; c < cls_lim; c++) {
            float p = r[5 + c];
            if (p > best) { best = p; best_id = c; }
        }
        if (obj <= 1.0f) obj = sigmoidf_local(obj);
        if (best <= 1.0f) best = sigmoidf_local(best);
        if (obj * best < conf_thr)
            continue;
        if (bw <= 2.0f && bh <= 2.0f) {
            cx *= (float)in_w; cy *= (float)in_h; bw *= (float)in_w; bh *= (float)in_h;
        }
        b.x1 = (int)((cx - bw * 0.5f) * ((float)src_w / (float)in_w));
        b.y1 = (int)((cy - bh * 0.5f) * ((float)src_h / (float)in_h));
        b.x2 = (int)((cx + bw * 0.5f) * ((float)src_w / (float)in_w));
        b.y2 = (int)((cy + bh * 0.5f) * ((float)src_h / (float)in_h));
        b.conf = obj * best;
        b.cls = best_id;
        clamp_box(&b, src_w, src_h);
        out[(*out_count)++] = b;
    }
}

static void decode_rows_output_transposed(const float *rows_t, int n_cols, int n_rows, int class_count,
                                          float conf_thr, int src_w, int src_h, int in_w, int in_h,
                                          struct det_box *out, int *out_count)
{
    int i;
    int c;
    int cls_lim = class_count;
    if (n_cols < 6)
        return;
    if (cls_lim > n_cols - 5)
        cls_lim = n_cols - 5;
    for (i = 0; i < n_rows && *out_count < MAX_DETS; i++) {
        float obj = rows_t[4 * n_rows + i];
        float best = (cls_lim > 0) ? 0.0f : 1.0f;
        int best_id = 0;
        float cx = rows_t[0 * n_rows + i];
        float cy = rows_t[1 * n_rows + i];
        float bw = rows_t[2 * n_rows + i];
        float bh = rows_t[3 * n_rows + i];
        struct det_box b;
        for (c = 0; c < cls_lim; c++) {
            float p = rows_t[(5 + c) * n_rows + i];
            if (p > best) { best = p; best_id = c; }
        }
        if (obj <= 1.0f) obj = sigmoidf_local(obj);
        if (best <= 1.0f) best = sigmoidf_local(best);
        if (obj * best < conf_thr)
            continue;
        if (bw <= 2.0f && bh <= 2.0f) {
            cx *= (float)in_w; cy *= (float)in_h; bw *= (float)in_w; bh *= (float)in_h;
        }
        b.x1 = (int)((cx - bw * 0.5f) * ((float)src_w / (float)in_w));
        b.y1 = (int)((cy - bh * 0.5f) * ((float)src_h / (float)in_h));
        b.x2 = (int)((cx + bw * 0.5f) * ((float)src_w / (float)in_w));
        b.y2 = (int)((cy + bh * 0.5f) * ((float)src_h / (float)in_h));
        b.conf = obj * best;
        b.cls = best_id;
        clamp_box(&b, src_w, src_h);
        out[(*out_count)++] = b;
    }
}

static void decode_rows_tensor_output(const rknn_tensor_attr *a, const float *buf, int class_count,
                                      float conf_thr, int src_w, int src_h, int in_w, int in_h,
                                      struct det_box *out, int *out_count)
{
    int n1 = (int)a->dims[1];
    int n2 = (int)a->dims[2];
    if (a->n_dims != 3)
        return;
    if (n2 >= 6 && n2 <= 512) {
        decode_rows_output(buf, n1, n2, class_count, conf_thr, src_w, src_h, in_w, in_h, out, out_count);
        return;
    }
    if (n1 >= 6 && n1 <= 512) {
        decode_rows_output_transposed(buf, n1, n2, class_count, conf_thr, src_w, src_h, in_w, in_h, out, out_count);
    }
}

struct yolo_head_view {
    uint32_t out_idx;
    int h;
    int w;
    int c;
    int stride;
    bool nchw;
};

static bool parse_yolo_head_view(const struct yolo_model *m, uint32_t out_idx, struct yolo_head_view *hv)
{
    const rknn_tensor_attr *a = &m->output_attrs[out_idx];
    int h = 0;
    int w = 0;
    int c = 0;
    bool nchw = false;

    if (a->n_dims != 4)
        return false;
    if (a->fmt == RKNN_TENSOR_NCHW) {
        c = (int)a->dims[1];
        h = (int)a->dims[2];
        w = (int)a->dims[3];
        nchw = true;
    } else if (a->fmt == RKNN_TENSOR_NHWC) {
        h = (int)a->dims[1];
        w = (int)a->dims[2];
        c = (int)a->dims[3];
        nchw = false;
    } else {
        if (a->dims[2] == a->dims[3] && a->dims[1] >= 18) {
            c = (int)a->dims[1];
            h = (int)a->dims[2];
            w = (int)a->dims[3];
            nchw = true;
        } else if (a->dims[1] == a->dims[2] && a->dims[3] >= 18) {
            h = (int)a->dims[1];
            w = (int)a->dims[2];
            c = (int)a->dims[3];
            nchw = false;
        } else {
            return false;
        }
    }

    if (h <= 0 || w <= 0 || c <= 0 || c % 3 != 0 || c / 3 < 6)
        return false;

    hv->out_idx = out_idx;
    hv->h = h;
    hv->w = w;
    hv->c = c;
    hv->nchw = nchw;
    hv->stride = (h > 0) ? ((int)m->in_h / h) : 0;
    return hv->stride > 0;
}

static void sort_heads_by_stride(struct yolo_head_view *heads, int n)
{
    int i;
    int j;
    for (i = 0; i < n; i++) {
        for (j = i + 1; j < n; j++) {
            if (heads[i].stride > heads[j].stride) {
                struct yolo_head_view t = heads[i];
                heads[i] = heads[j];
                heads[j] = t;
            }
        }
    }
}

static float head_read(const float *buf, const struct yolo_head_view *hv, int a, int gy, int gx, int k)
{
    int attrs = hv->c / 3;
    int ch = a * attrs + k;
    if (hv->nchw)
        return buf[((ch * hv->h + gy) * hv->w) + gx];
    return buf[((gy * hv->w + gx) * hv->c) + ch];
}

static void decode_yolo_head_output(const float *buf, const struct yolo_head_view *hv,
                                    const float anchors[3][2], int class_count, float conf_thr,
                                    int src_w, int src_h, int in_w, int in_h,
                                    struct det_box *out, int *out_count)
{
    int gy;
    int gx;
    int a;
    int attrs = hv->c / 3;
    int classes = attrs - 5;
    int cls_lim = class_count;
    if (classes <= 0)
        return;
    if (cls_lim > classes)
        cls_lim = classes;

    for (gy = 0; gy < hv->h && *out_count < MAX_DETS; gy++) {
        for (gx = 0; gx < hv->w && *out_count < MAX_DETS; gx++) {
            for (a = 0; a < 3 && *out_count < MAX_DETS; a++) {
                float tx = head_read(buf, hv, a, gy, gx, 0);
                float ty = head_read(buf, hv, a, gy, gx, 1);
                float tw = head_read(buf, hv, a, gy, gx, 2);
                float th = head_read(buf, hv, a, gy, gx, 3);
                float to = head_read(buf, hv, a, gy, gx, 4);
                float obj = sigmoidf_local(to);
                float best = (cls_lim > 0) ? 0.0f : 1.0f;
                int best_id = 0;
                int c;
                struct det_box b;
                float bx;
                float by;
                float bw;
                float bh;
                float conf;

                if (obj < conf_thr * 0.5f)
                    continue;

                for (c = 0; c < cls_lim; c++) {
                    float p = sigmoidf_local(head_read(buf, hv, a, gy, gx, 5 + c));
                    if (p > best) {
                        best = p;
                        best_id = c;
                    }
                }

                conf = obj * best;
                if (conf < conf_thr)
                    continue;

                bx = ((sigmoidf_local(tx) * 2.0f - 0.5f) + (float)gx) * (float)hv->stride;
                by = ((sigmoidf_local(ty) * 2.0f - 0.5f) + (float)gy) * (float)hv->stride;
                bw = powf(sigmoidf_local(tw) * 2.0f, 2.0f) * anchors[a][0];
                bh = powf(sigmoidf_local(th) * 2.0f, 2.0f) * anchors[a][1];

                b.x1 = (int)((bx - bw * 0.5f) * ((float)src_w / (float)in_w));
                b.y1 = (int)((by - bh * 0.5f) * ((float)src_h / (float)in_h));
                b.x2 = (int)((bx + bw * 0.5f) * ((float)src_w / (float)in_w));
                b.y2 = (int)((by + bh * 0.5f) * ((float)src_h / (float)in_h));
                b.conf = conf;
                b.cls = best_id;
                clamp_box(&b, src_w, src_h);
                out[(*out_count)++] = b;
            }
        }
    }
}

static void decode_yolo_heads_outputs(const struct yolo_model *m, const rknn_output *outs,
                                      float conf_thr, int src_w, int src_h,
                                      struct det_box *out, int *out_count)
{
    static const float anchors_p5[3][3][2] = {
        {{10.0f, 13.0f}, {16.0f, 30.0f}, {33.0f, 23.0f}},
        {{30.0f, 61.0f}, {62.0f, 45.0f}, {59.0f, 119.0f}},
        {{116.0f, 90.0f}, {156.0f, 198.0f}, {373.0f, 326.0f}},
    };
    static const float anchors_p6[4][3][2] = {
        {{19.0f, 27.0f}, {44.0f, 40.0f}, {38.0f, 94.0f}},
        {{96.0f, 68.0f}, {86.0f, 152.0f}, {180.0f, 137.0f}},
        {{140.0f, 301.0f}, {303.0f, 264.0f}, {238.0f, 542.0f}},
        {{436.0f, 615.0f}, {739.0f, 380.0f}, {925.0f, 792.0f}},
    };
    struct yolo_head_view heads[8];
    int head_count = 0;
    uint32_t i;

    for (i = 0; i < m->io_num.n_output && head_count < 8; i++) {
        if (parse_yolo_head_view(m, i, &heads[head_count]))
            head_count++;
    }
    if (head_count == 0)
        return;

    sort_heads_by_stride(heads, head_count);
    for (i = 0; i < (uint32_t)head_count && *out_count < MAX_DETS; i++) {
        const float (*anchors)[2] = NULL;
        if (head_count == 3 && i < 3)
            anchors = anchors_p5[i];
        else if (head_count == 4 && i < 4)
            anchors = anchors_p6[i];
        else if (i < 3)
            anchors = anchors_p5[i];
        if (!anchors)
            continue;

        decode_yolo_head_output((const float *)outs[heads[i].out_idx].buf, &heads[i], anchors,
                                m->class_count, conf_thr, src_w, src_h,
                                (int)m->in_w, (int)m->in_h, out, out_count);
    }
}

static int run_model_detect(struct yolo_model *m, const uint8_t *in_rgb, int src_w, int src_h,
                            float conf_thr, struct det_box *out, int *out_count)
{
    rknn_input in;
    rknn_output outs[8];
    uint32_t i;
    int ret;
    *out_count = 0;

    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf = (void *)in_rgb;
    in.size = m->in_w * m->in_h * 3;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    ret = rknn_inputs_set(m->ctx, 1, &in);
    if (ret < 0) return ret;
    ret = rknn_run(m->ctx, NULL);
    if (ret < 0) return ret;

    memset(outs, 0, sizeof(outs));
    for (i = 0; i < m->io_num.n_output; i++)
        outs[i].want_float = 1;
    ret = rknn_outputs_get(m->ctx, m->io_num.n_output, outs, NULL);
    if (ret < 0) return ret;

    for (i = 0; i < m->io_num.n_output; i++) {
        rknn_tensor_attr *a = &m->output_attrs[i];
        if (a->n_dims == 3)
            decode_rows_tensor_output(a, (const float *)outs[i].buf, m->class_count,
                                      conf_thr, src_w, src_h, (int)m->in_w, (int)m->in_h,
                                      out, out_count);
    }
    if (*out_count == 0)
        decode_yolo_heads_outputs(m, outs, conf_thr, src_w, src_h, out, out_count);

    if (*out_count > MAX_DETS) *out_count = MAX_DETS;
    nms_inplace(out, out_count, 0.45f);
    rknn_outputs_release(m->ctx, m->io_num.n_output, outs);
    return 0;
}

static enum plate_color classify_plate_color_rgb(const uint8_t *rgb, int w, int h, const struct det_box *b)
{
    int x1 = b->x1 + (b->x2 - b->x1) / 6;
    int x2 = b->x2 - (b->x2 - b->x1) / 6;
    int y1 = b->y1 + (b->y2 - b->y1) / 6;
    int y2 = b->y2 - (b->y2 - b->y1) / 6;
    int x, y;
    int total = 0, blue_cnt = 0, green_cnt = 0, yellow_cnt = 0;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 >= w) x2 = w - 1;
    if (y2 >= h) y2 = h - 1;
    for (y = y1; y <= y2; y++) {
        for (x = x1; x <= x2; x++) {
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
            if (d > 1e-6f) {
                if (mx == r) h_deg = 60.0f * fmodf((g - bch) / d, 6.0f);
                else if (mx == g) h_deg = 60.0f * (((bch - r) / d) + 2.0f);
                else h_deg = 60.0f * (((r - g) / d) + 4.0f);
            }
            if (h_deg < 0.0f) h_deg += 360.0f;
            total++;
            if (h_deg >= 90.0f && h_deg <= 130.0f && s > 0.23f && v > 0.16f) blue_cnt++;
            else if (h_deg >= 35.0f && h_deg <= 90.0f && s > 0.20f && v > 0.16f) green_cnt++;
            else if (h_deg >= 15.0f && h_deg <= 55.0f && s > 0.20f && v > 0.16f) yellow_cnt++;
        }
    }
    if (total == 0) return PLATE_COLOR_UNKNOWN;
    if ((float)blue_cnt / (float)total >= 0.20f && blue_cnt > green_cnt + (int)(0.05f * total))
        return PLATE_COLOR_BLUE;
    if ((float)green_cnt / (float)total >= 0.20f && green_cnt > blue_cnt + (int)(0.05f * total))
        return PLATE_COLOR_GREEN;
    if ((float)yellow_cnt / (float)total >= 0.18f)
        return PLATE_COLOR_YELLOW;
    return PLATE_COLOR_UNKNOWN;
}

static int find_parent_car(const struct det_box *plate, const struct det_box *cars, int car_count)
{
    int i;
    int cx = (plate->x1 + plate->x2) / 2;
    int cy = (plate->y1 + plate->y2) / 2;
    int best = -1;
    float best_ratio = 0.0f;
    for (i = 0; i < car_count; i++) {
        const struct det_box *c = &cars[i];
        int x1 = plate->x1 > c->x1 ? plate->x1 : c->x1;
        int y1 = plate->y1 > c->y1 ? plate->y1 : c->y1;
        int x2 = plate->x2 < c->x2 ? plate->x2 : c->x2;
        int y2 = plate->y2 < c->y2 ? plate->y2 : c->y2;
        bool center_inside = (cx >= c->x1 && cx <= c->x2 && cy >= c->y1 && cy <= c->y2);
        float ratio = 0.0f;
        if (x2 >= x1 && y2 >= y1) {
            int ia = (x2 - x1 + 1) * (y2 - y1 + 1);
            int pa = (plate->x2 - plate->x1 + 1) * (plate->y2 - plate->y1 + 1);
            ratio = pa > 0 ? (float)ia / (float)pa : 0.0f;
        }
        if (center_inside)
            return i;
        if (ratio > 0.70f && ratio > best_ratio) {
            best_ratio = ratio;
            best = i;
        }
    }
    return best;
}

static const char *plate_color_str(enum plate_color c)
{
    if (c == PLATE_COLOR_BLUE) return "BLUE";
    if (c == PLATE_COLOR_GREEN) return "GREEN";
    if (c == PLATE_COLOR_YELLOW) return "YELLOW";
    return "UNK";
}

static enum plate_type classify_plate_type(enum plate_color color, const char *text)
{
    const char *utf8_police = "\xE8\xAD\xA6";
    const char *utf8_trailer = "\xE6\x8C\x82";
    const char *utf8_embassy = "\xE4\xBD\xBF";
    const char *utf8_consulate = "\xE9\xA2\x86";

    if (text && strstr(text, utf8_police))
        return PLATE_TYPE_POLICE;
    if (text && strstr(text, utf8_trailer))
        return PLATE_TYPE_TRAILER;
    if (text && (strstr(text, utf8_embassy) || strstr(text, utf8_consulate)))
        return PLATE_TYPE_EMBASSY_CONSULATE;
    if (text && strncmp(text, "WJ", 2) == 0)
        return PLATE_TYPE_POLICE;

    if (color == PLATE_COLOR_GREEN)
        return PLATE_TYPE_COMMON_GREEN;
    if (color == PLATE_COLOR_BLUE)
        return PLATE_TYPE_COMMON_BLUE;
    if (color == PLATE_COLOR_YELLOW)
        return PLATE_TYPE_YELLOW;
    return PLATE_TYPE_UNKNOWN;
}

static const char *plate_type_str(enum plate_type t)
{
    switch (t) {
    case PLATE_TYPE_COMMON_BLUE: return "common_blue";
    case PLATE_TYPE_COMMON_GREEN: return "common_green";
    case PLATE_TYPE_YELLOW: return "yellow";
    case PLATE_TYPE_POLICE: return "police";
    case PLATE_TYPE_TRAILER: return "trailer";
    case PLATE_TYPE_EMBASSY_CONSULATE: return "embassy_consulate";
    default: return "unknown";
    }
}

static bool plate_box_pass_rules(const struct det_box *b, int frame_w, int frame_h)
{
    int bw = b->x2 - b->x1 + 1;
    int bh = b->y2 - b->y1 + 1;
    int cy = (b->y1 + b->y2) / 2;
    float aspect;
    float area;
    float min_area = (float)frame_w * (float)frame_h * 0.0016f;

    if (bw <= 0 || bh <= 0)
        return false;
    if (bw < 56 || bh < 18)
        return false;
    aspect = (float)bw / (float)bh;
    if (aspect < 2.4f || aspect > 5.8f)
        return false;
    area = (float)bw * (float)bh;
    if (area < min_area)
        return false;
    if (cy < (int)(0.12f * (float)frame_h) || cy > (int)(0.95f * (float)frame_h))
        return false;
    return true;
}

static bool has_iou_match(const struct det_box *cur, const struct det_box *hist, int hist_count, float iou_thr)
{
    int i;
    for (i = 0; i < hist_count; i++) {
        if (box_iou(cur, &hist[i]) >= iou_thr)
            return true;
    }
    return false;
}

static void temporal_confirm_and_update(struct app_ctx *ctx,
                                        const struct det_box *filtered, int filtered_count,
                                        struct det_box *confirmed, int *confirmed_count)
{
    int i;
    *confirmed_count = 0;
    if (ctx->plate_hist1_count > 0 && ctx->plate_hist2_count > 0) {
        for (i = 0; i < filtered_count && *confirmed_count < MAX_DETS; i++) {
            if (has_iou_match(&filtered[i], ctx->plate_hist1, ctx->plate_hist1_count, 0.35f) &&
                has_iou_match(&filtered[i], ctx->plate_hist2, ctx->plate_hist2_count, 0.30f)) {
                confirmed[(*confirmed_count)++] = filtered[i];
            }
        }
    }
    memcpy(ctx->plate_hist2, ctx->plate_hist1, sizeof(ctx->plate_hist1));
    ctx->plate_hist2_count = ctx->plate_hist1_count;
    memcpy(ctx->plate_hist1, filtered, (size_t)filtered_count * sizeof(filtered[0]));
    ctx->plate_hist1_count = filtered_count;
}

static void csv_safe_text(const char *in, char *out, size_t out_len)
{
    size_t i = 0;
    if (out_len == 0)
        return;
    if (!in) {
        out[0] = '\0';
        return;
    }
    while (*in && i + 1 < out_len) {
        char ch = *in++;
        if (ch == ',' || ch == '\n' || ch == '\r')
            ch = '_';
        out[i++] = ch;
    }
    out[i] = '\0';
}

static void log_prediction_row(struct app_ctx *ctx, uint64_t frame_id, int64_t ts_us,
                               const struct plate_det *pd)
{
    char safe_text[64];
    if (!ctx->pred_log_fp)
        return;
    csv_safe_text(pd->ocr_text, safe_text, sizeof(safe_text));
    pthread_mutex_lock(&ctx->pred_log_lock);
    fprintf(ctx->pred_log_fp,
            "%" PRIu64 ",%s,%s,%.4f,%d,%d,%d,%d,%" PRId64 "\n",
            frame_id,
            safe_text,
            plate_type_str(pd->type),
            pd->ocr_conf,
            pd->box.x1, pd->box.y1, pd->box.x2, pd->box.y2,
            ts_us);
    fflush(ctx->pred_log_fp);
    pthread_mutex_unlock(&ctx->pred_log_lock);
}

static void build_overlay_ascii_text(const struct plate_det *pd, char *out, size_t out_len)
{
    size_t i = 0;
    const char *s = pd->ocr_text;
    if (out_len == 0)
        return;
    while (s && *s && i + 1 < out_len) {
        unsigned char ch = (unsigned char)*s++;
        if ((ch >= '0' && ch <= '9') ||
            (ch >= 'A' && ch <= 'Z') ||
            (ch >= 'a' && ch <= 'z') ||
            ch == '-') {
            out[i++] = (char)ch;
        }
    }
    if (i == 0) {
        const char *fb = plate_type_str(pd->type);
        while (*fb && i + 1 < out_len) {
            unsigned char ch = (unsigned char)*fb++;
            if ((ch >= 'a' && ch <= 'z') || ch == '_')
                out[i++] = (char)ch;
        }
        if (i == 0) {
            out[0] = 'U';
            if (out_len > 1) out[1] = 'N';
            if (out_len > 2) out[2] = 'K';
            i = (out_len > 2) ? 3 : out_len - 1;
        }
    }
    out[i] = '\0';
}

static void *infer_thread_main(void *arg)
{
    struct app_ctx *ctx = (struct app_ctx *)arg;
    uint8_t *raw_local = malloc(ctx->src_frame_size);
    uint8_t *rgb_full = malloc((size_t)ctx->frame_width * ctx->frame_height * 3U);
    uint8_t *a_map = malloc((size_t)ctx->frame_width * ctx->frame_height);
    uint8_t *algo_rgb = malloc((size_t)ALGO_STREAM_SIZE * ALGO_STREAM_SIZE * 3U);
    uint8_t *veh_in = malloc((size_t)ctx->veh_model.in_w * ctx->veh_model.in_h * 3U);
    uint8_t *plate_in = malloc((size_t)ctx->plate_model.in_w * ctx->plate_model.in_h * 3U);
    uint8_t *plate_crop = malloc((size_t)OCR_CROP_WIDTH * OCR_CROP_HEIGHT * 3U);
    if (!raw_local || !rgb_full || !a_map || !algo_rgb || !veh_in || !plate_in || !plate_crop) {
        free(raw_local); free(rgb_full); free(a_map); free(algo_rgb);
        free(veh_in); free(plate_in); free(plate_crop);
        return NULL;
    }

    while (ctx->running) {
        struct det_box cars[MAX_DETS];
        struct det_box raw_plates[MAX_DETS];
        struct det_box filtered_plates[MAX_DETS];
        struct det_box roi_plates[MAX_DETS];
        struct det_box stable_plates[MAX_DETS];
        struct det_box persons[MAX_DETS];
        struct det_box tracked_persons[MAX_DETS];
        struct lpr_results r;
        int car_count = 0;
        int raw_plate_count = 0;
        int filtered_plate_count = 0;
        int stable_plate_count = 0;
        int person_count = 0;
        int tracked_person_count = 0;
        int ped_events = 0;
        bool light_red = false;
        float red_ratio = 0.0f;
        struct det_box a_roi = {0};
        bool a_roi_valid = false;
        int i;
        int64_t t0, t1;
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

        t0 = mono_us();
        if (ctx->src_is_bgrx) {
            bgrx8888_to_rgb888_and_a(raw_local, (int)ctx->frame_width, (int)ctx->frame_height, rgb_full, a_map);
        } else {
            raw565_to_rgb888_full(ctx, raw_local, rgb_full);
            memset(a_map, 0, (size_t)ctx->frame_width * ctx->frame_height);
        }
        if (ctx->opt.sw_preproc)
            sw_preprocess_rgb888(rgb_full, (int)ctx->frame_width, (int)ctx->frame_height);

        if (ctx->opt.fpga_a_mask && ctx->src_is_bgrx) {
            a_roi_valid = extract_a_channel_roi(a_map, (int)ctx->frame_width, (int)ctx->frame_height,
                                                ctx->opt.a_proj_ratio, &a_roi, &red_ratio) ? true : false;
            if (red_ratio >= ctx->opt.red_ratio_thr)
                ctx->ped_red_streak++;
            else
                ctx->ped_red_streak = 0;
            light_red = (ctx->ped_red_streak >= ctx->opt.red_stable_frames);
        } else {
            ctx->ped_red_streak = 0;
            light_red = false;
        }

        resize_rgb888_nn(rgb_full, (int)ctx->frame_width, (int)ctx->frame_height,
                         algo_rgb, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE);

        if (ctx->veh_model.in_w == ALGO_STREAM_SIZE && ctx->veh_model.in_h == ALGO_STREAM_SIZE)
            memcpy(veh_in, algo_rgb, (size_t)ALGO_STREAM_SIZE * ALGO_STREAM_SIZE * 3U);
        else
            resize_rgb888_nn(algo_rgb, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE,
                             veh_in, (int)ctx->veh_model.in_w, (int)ctx->veh_model.in_h);

        if (ctx->plate_model.in_w == ALGO_STREAM_SIZE && ctx->plate_model.in_h == ALGO_STREAM_SIZE)
            memcpy(plate_in, algo_rgb, (size_t)ALGO_STREAM_SIZE * ALGO_STREAM_SIZE * 3U);
        else
            resize_rgb888_nn(algo_rgb, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE,
                             plate_in, (int)ctx->plate_model.in_w, (int)ctx->plate_model.in_h);

        if (!ctx->opt.plate_only || ctx->opt.ped_event) {
            if (run_model_detect(&ctx->veh_model, veh_in, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE,
                                 ctx->opt.min_car_conf, cars, &car_count) < 0)
                car_count = 0;
        }
        {
            float plate_thr = ctx->opt.min_plate_conf;
            if (ctx->opt.fpga_a_mask && a_roi_valid)
                plate_thr = fmaxf(0.05f, plate_thr - 0.05f);
            if (run_model_detect(&ctx->plate_model, plate_in, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE,
                                 plate_thr, raw_plates, &raw_plate_count) < 0)
                raw_plate_count = 0;
        }
        if (raw_plate_count > 0) {
            ctx->gate_plate_raw_positive_frames++;
            ctx->gate_plate_raw_positive_streak++;
        } else {
            ctx->gate_plate_raw_positive_streak = 0;
        }

        for (i = 0; i < car_count; i++)
            map_box_between_spaces(&cars[i], ALGO_STREAM_SIZE, ALGO_STREAM_SIZE,
                                   (int)ctx->frame_width, (int)ctx->frame_height);
        for (i = 0; i < car_count && person_count < MAX_DETS; i++) {
            if (cars[i].cls == ctx->person_class_id)
                persons[person_count++] = cars[i];
        }
        if (ctx->opt.ped_event)
            ped_events = update_ped_tracks_nn(ctx, persons, person_count, light_red, seq,
                                              tracked_persons, &tracked_person_count);

        for (i = 0; i < raw_plate_count; i++) {
            map_box_between_spaces(&raw_plates[i], ALGO_STREAM_SIZE, ALGO_STREAM_SIZE,
                                   (int)ctx->frame_width, (int)ctx->frame_height);
            if (plate_box_pass_rules(&raw_plates[i], (int)ctx->frame_width, (int)ctx->frame_height))
                filtered_plates[filtered_plate_count++] = raw_plates[i];
        }
        if (ctx->opt.fpga_a_mask && a_roi_valid) {
            int roi_count = filter_boxes_by_roi(filtered_plates, filtered_plate_count, &a_roi,
                                                ctx->opt.a_roi_iou_min, roi_plates, MAX_DETS);
            if (roi_count > 0) {
                memcpy(filtered_plates, roi_plates, (size_t)roi_count * sizeof(roi_plates[0]));
                filtered_plate_count = roi_count;
            }
        }

        temporal_confirm_and_update(ctx, filtered_plates, filtered_plate_count,
                                    stable_plates, &stable_plate_count);
        t1 = mono_us();

        memset(&r, 0, sizeof(r));
        r.car_raw_count = car_count;
        r.person_raw_count = person_count;
        r.plate_raw_count = raw_plate_count;
        r.a_roi_valid = a_roi_valid ? 1 : 0;
        r.a_roi = a_roi;
        r.light_red = light_red ? 1 : 0;
        for (i = 0; i < car_count && r.car_count < MAX_DETS; i++) {
            if (cars[i].cls == ctx->car_class_id)
                r.cars[r.car_count++] = cars[i];
        }
        if (!ctx->opt.ped_event) {
            for (i = 0; i < person_count && r.person_count < MAX_DETS; i++)
                r.persons[r.person_count++] = persons[i];
        } else {
            for (i = 0; i < tracked_person_count && r.person_count < MAX_DETS; i++)
                r.persons[r.person_count++] = tracked_persons[i];
        }

        for (i = 0; i < stable_plate_count && r.plate_count < MAX_DETS; i++) {
            struct plate_det pd;
            int parent = -1;
            int crop_w;
            int crop_h;
            pd.box = stable_plates[i];
            if (!ctx->opt.plate_only)
                parent = find_parent_car(&pd.box, r.cars, r.car_count);
            if (!ctx->opt.plate_only && ctx->opt.plate_on_car_only && parent < 0)
                continue;
            pd.parent_car = parent;
            pd.color = classify_plate_color_rgb(rgb_full, (int)ctx->frame_width, (int)ctx->frame_height, &pd.box);
            compute_center_crop_box(&pd.box, (int)ctx->frame_width, (int)ctx->frame_height,
                                    OCR_CROP_WIDTH, OCR_CROP_HEIGHT, &pd.crop_box);
            crop_w = pd.crop_box.x2 - pd.crop_box.x1 + 1;
            crop_h = pd.crop_box.y2 - pd.crop_box.y1 + 1;
            copy_crop_rgb888(rgb_full, (int)ctx->frame_width, &pd.crop_box, plate_crop);
            if (run_model_ocr(ctx, plate_crop, crop_w, crop_h,
                              pd.ocr_text, sizeof(pd.ocr_text), &pd.ocr_conf) < 0) {
                snprintf(pd.ocr_text, sizeof(pd.ocr_text), "UNK");
                pd.ocr_conf = 0.0f;
            }
            pd.type = classify_plate_type(pd.color, pd.ocr_text);
            fprintf(stderr,
                    "[pred] frame=%" PRIu64 " ts_us=%" PRId64 " bbox=[%d,%d,%d,%d] text=%s conf=%.2f type=%s color=%s\n",
                    seq,
                    mono_us(),
                    pd.box.x1, pd.box.y1, pd.box.x2, pd.box.y2,
                    pd.ocr_text,
                    pd.ocr_conf,
                    plate_type_str(pd.type),
                    plate_color_str(pd.color));
            log_prediction_row(ctx, seq, mono_us(), &pd);
            ctx->pred_rows_total++;
            r.plates[r.plate_count++] = pd;
        }
        r.frame_seq = seq;
        r.infer_ms_last = (double)(t1 - t0) / 1000.0;
        pthread_mutex_lock(&ctx->result_lock);
        r.infer_frames_total = ctx->results.infer_frames_total + 1;
        r.infer_ms_total = ctx->results.infer_ms_total + r.infer_ms_last;
        r.ped_event_total = ctx->results.ped_event_total + (uint64_t)ped_events;
        r.ped_event_last_frame = (ped_events > 0) ? seq : ctx->results.ped_event_last_frame;
        ctx->results = r;
        pthread_mutex_unlock(&ctx->result_lock);
    }

    free(raw_local); free(rgb_full); free(a_map); free(algo_rgb);
    free(veh_in); free(plate_in); free(plate_crop);
    return NULL;
}

static void overlay_results_on_slot(struct app_ctx *ctx, uint8_t *slot_data)
{
    struct lpr_results r;
    uint16_t *pix = (uint16_t *)slot_data;
    int i;
    int stopline_y = (int)((float)ctx->frame_height * ctx->opt.stopline_ratio);
    pthread_mutex_lock(&ctx->result_lock);
    r = ctx->results;
    pthread_mutex_unlock(&ctx->result_lock);

    for (i = 0; i < r.car_count; i++)
        draw_rect_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, &r.cars[i], COLOR_YELLOW_565);
    for (i = 0; i < r.person_count; i++)
        draw_rect_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, &r.persons[i], COLOR_GREEN_565);

    for (i = 0; i < r.plate_count; i++) {
        char txt[32];
        int tx = r.plates[i].box.x1;
        int ty = r.plates[i].box.y1 - 10;
        if (ty < 0) ty = r.plates[i].box.y1 + 2;
        build_overlay_ascii_text(&r.plates[i], txt, sizeof(txt));
        draw_rect_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, &r.plates[i].box, COLOR_CYAN_565);
        draw_text_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, tx, ty, txt, COLOR_CYAN_565);
    }

    if (ctx->opt.fpga_a_mask && r.a_roi_valid)
        draw_rect_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, &r.a_roi, COLOR_GREEN_565);
    if (ctx->opt.ped_event) {
        draw_hline_565(pix, (int)ctx->frame_width, (int)ctx->frame_height,
                       0, (int)ctx->frame_width - 1, stopline_y,
                       r.light_red ? COLOR_RED_565 : COLOR_GREEN_565);
    }
}

static void push_latest_to_infer(struct app_ctx *ctx, const uint8_t *raw)
{
    pthread_mutex_lock(&ctx->infer_lock);
    if (ctx->infer_has_new)
        ctx->infer_overwrite_count++;
    memcpy(ctx->infer_latest_raw, raw, ctx->src_frame_size);
    ctx->infer_frame_seq++;
    ctx->infer_has_new = true;
    pthread_cond_signal(&ctx->infer_cond);
    pthread_mutex_unlock(&ctx->infer_lock);
}

static void print_stats(struct app_ctx *ctx)
{
    int64_t now = mono_us();
    int64_t dt = now - ctx->last_stats_us;
    struct lpr_results r;
    if (dt < (int64_t)ctx->opt.stats_interval * 1000000LL)
        return;
    pthread_mutex_lock(&ctx->result_lock);
    r = ctx->results;
    pthread_mutex_unlock(&ctx->result_lock);
    fprintf(stderr,
            "[stats] cap=%" PRIu64 " push=%" PRIu64 " rel=%" PRIu64
            " infer=%" PRIu64 " infer_ms=%.2f cars=%d(raw=%d) persons=%d(raw=%d)"
            " plates=%d(raw=%d) aroi=%d red=%d ped_evt=%" PRIu64
            " gate_raw_pos=%" PRIu64 " gate_streak=%" PRIu64 " pred_rows=%" PRIu64 " drop=%" PRIu64
            " cap_fps=%.2f disp_fps=%.2f infer_fps=%.2f\n",
            ctx->captured_frames, ctx->pushed_frames, ctx->released_frames,
            r.infer_frames_total, r.infer_ms_last,
            r.car_count, r.car_raw_count,
            r.person_count, r.person_raw_count,
            r.plate_count, r.plate_raw_count,
            r.a_roi_valid, r.light_red, r.ped_event_total,
            ctx->gate_plate_raw_positive_frames, ctx->gate_plate_raw_positive_streak, ctx->pred_rows_total,
            ctx->infer_overwrite_count,
            (double)(ctx->captured_frames - ctx->last_stats_cap) * 1000000.0 / (double)dt,
            (double)(ctx->released_frames - ctx->last_stats_rel) * 1000000.0 / (double)dt,
            (double)(r.infer_frames_total - ctx->last_stats_infer) * 1000000.0 / (double)dt);
    ctx->last_stats_cap = ctx->captured_frames;
    ctx->last_stats_rel = ctx->released_frames;
    ctx->last_stats_infer = r.infer_frames_total;
    ctx->last_stats_us = now;
}

static void cleanup(struct app_ctx *ctx)
{
    int i;
    ctx->running = false;
    pthread_mutex_lock(&ctx->infer_lock);
    pthread_cond_broadcast(&ctx->infer_cond);
    pthread_mutex_unlock(&ctx->infer_lock);
    if (ctx->infer_thread)
        pthread_join(ctx->infer_thread, NULL);

    rknn_model_release(&ctx->veh_model);
    rknn_model_release(&ctx->plate_model);
    rknn_ocr_model_release(&ctx->ocr_model);

    if (ctx->pred_log_fp) {
        fclose(ctx->pred_log_fp);
        ctx->pred_log_fp = NULL;
    }

    if (ctx->appsrc)
        gst_app_src_end_of_stream(GST_APP_SRC(ctx->appsrc));
    if (ctx->pipeline)
        gst_element_set_state(ctx->pipeline, GST_STATE_NULL);
    if (ctx->bus)
        gst_object_unref(ctx->bus);
    if (ctx->pipeline)
        gst_object_unref(ctx->pipeline);

    if (ctx->dma_map)
        munmap(ctx->dma_map, ctx->dma_map_size);
    free(ctx->dma_copy);
    free(ctx->infer_latest_raw);

    if (ctx->slots) {
        for (i = 0; i < ctx->slot_count; i++)
            free(ctx->slots[i].data);
        free(ctx->slots);
    }

    if (ctx->dev_fd >= 0)
        close(ctx->dev_fd);
    if (ctx->drm_fd >= 0)
        close(ctx->drm_fd);

    pthread_mutex_destroy(&ctx->infer_lock);
    pthread_cond_destroy(&ctx->infer_cond);
    pthread_mutex_destroy(&ctx->result_lock);
    pthread_mutex_destroy(&ctx->pred_log_lock);
    g_cond_clear(&ctx->slots_cond);
    g_mutex_clear(&ctx->slots_lock);
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
    pthread_mutex_init(&ctx.pred_log_lock, NULL);

    if (parse_options(argc, argv, &ctx.opt) < 0) {
        print_usage(argv[0]);
        goto out;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    gst_init(&argc, &argv);

    ctx.drm_fd = open(ctx.opt.drm_card_path, O_RDWR | O_CLOEXEC);
    if (ctx.drm_fd < 0)
        goto out;
    if (load_labels(&ctx, ctx.opt.labels_path) < 0)
        goto out;
    if (load_ocr_keys(&ctx, ctx.opt.ocr_keys_path) < 0)
        goto out;
    if (init_fpga_dma(&ctx) < 0)
        goto out;
    if (init_copy_slots(&ctx) < 0)
        goto out;
    if (rknn_model_load(&ctx.veh_model, "vehicle", ctx.opt.veh_model_path, ctx.label_count) < 0)
        goto out;
    if (rknn_model_load(&ctx.plate_model, "plate", ctx.opt.plate_model_path, 1) < 0)
        goto out;
    if (rknn_ocr_model_load(&ctx.ocr_model, "ocr", ctx.opt.ocr_model_path) < 0)
        goto out;

    if (ctx.opt.pred_log_path && ctx.opt.pred_log_path[0] != '\0') {
        ctx.pred_log_fp = fopen(ctx.opt.pred_log_path, "w");
        if (!ctx.pred_log_fp)
            goto out;
        fprintf(ctx.pred_log_fp, "frame_id,plate_text_pred,plate_type_pred,conf,x1,y1,x2,y2,ts_us\n");
        fflush(ctx.pred_log_fp);
    }

    ctx.infer_latest_raw = malloc(ctx.src_frame_size);
    if (!ctx.infer_latest_raw)
        goto out;
    if (build_pipeline(&ctx) < 0)
        goto out;
    if (pthread_create(&ctx.infer_thread, NULL, infer_thread_main, &ctx) != 0)
        goto out;

    fprintf(stderr,
            "Start LPR loop: fps=%d src=%s pixel=%s swap16=%s min_car=%.2f min_plate=%.2f plate_only=%d "
            "sw_preproc=%d fpga_a_mask=%d ped_event=%d pred_log=%s\n",
            ctx.opt.fps,
            ctx.src_is_bgrx ? "bgrx8888" : "bgr565",
            (ctx.opt.pixel_order == PIXEL_ORDER_BGR565) ? "bgr565" : "rgb565",
            ctx.opt.swap16 ? "on" : "off",
            ctx.opt.min_car_conf, ctx.opt.min_plate_conf,
            ctx.opt.plate_only,
            ctx.opt.sw_preproc,
            ctx.opt.fpga_a_mask,
            ctx.opt.ped_event,
            ctx.opt.pred_log_path ? ctx.opt.pred_log_path : "<off>");

    ctx.last_stats_us = mono_us();

    while (ctx.running) {
        struct slot_ticket ticket;
        GstBuffer *buf;
        GstFlowReturn flow;
        int64_t t0;
        int64_t loop_us;
        int64_t target_us = 1000000LL / ctx.opt.fps;

        if (g_stop)
            ctx.running = false;
        if (!ctx.running)
            break;
        if (handle_bus_messages(&ctx) < 0)
            break;

        t0 = mono_us();
        if (trigger_frame_dma(&ctx) < 0)
            break;
        ctx.captured_frames++;

        if (acquire_free_slot(&ctx, &ticket) < 0)
            break;
        copy_frame_to_slot565(&ctx, ctx.slots[ticket.idx].data, ctx.dma_copy);
        push_latest_to_infer(&ctx, ctx.dma_copy);
        overlay_results_on_slot(&ctx, ctx.slots[ticket.idx].data);

        buf = build_frame_buffer(&ctx, &ticket);
        if (!buf)
            break;
        flow = gst_app_src_push_buffer(GST_APP_SRC(ctx.appsrc), buf);
        if (flow != GST_FLOW_OK) {
            release_slot_ticket(&ctx, &ticket, false);
            break;
        }
        ctx.pushed_frames++;
        print_stats(&ctx);

        loop_us = mono_us() - t0;
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
