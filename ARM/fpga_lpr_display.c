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

#define COLOR_YELLOW_565 0xFFE0
#define COLOR_CYAN_565 0x07FF

enum pixel_order {
    PIXEL_ORDER_BGR565 = 0,
    PIXEL_ORDER_RGB565,
};

enum plate_color {
    PLATE_COLOR_UNKNOWN = 0,
    PLATE_COLOR_BLUE,
    PLATE_COLOR_GREEN,
};

struct options {
    const char *device_path;
    const char *drm_card_path;
    const char *veh_model_path;
    const char *plate_model_path;
    const char *labels_path;
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
    enum plate_color color;
    int parent_car;
};

struct frame_slot {
    uint8_t *data;
    bool in_use;
    uint64_t generation;
};

struct lpr_results {
    struct det_box cars[MAX_DETS];
    int car_count;
    struct plate_det plates[MAX_DETS];
    int plate_count;
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

struct app_ctx {
    struct options opt;
    int dev_fd;
    int drm_fd;
    void *dma_map;
    size_t dma_map_size;
    uint8_t *dma_copy;
    uint32_t frame_width;
    uint32_t frame_height;
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
    char labels[MAX_LABELS][MAX_LABEL_LEN];
    int label_count;
    int car_class_id;
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
            "  --labels <path>         Labels file path (required)\n"
            "  --connector-id <id>     Optional KMS connector id\n"
            "  --fps <num>             Target FPS (default: %d)\n"
            "  --pixel-order <mode>    bgr565|rgb565 (default: bgr565)\n"
            "  --swap16 <0|1>          Swap bytes per 16-bit pixel (default: 1)\n"
            "  --timeout-ms <ms>       Frame timeout (default: %d)\n"
            "  --stats-interval <sec>  Stats print interval (default: %d)\n"
            "  --copy-buffers <num>    Copy ring size (default: %d)\n"
            "  --queue-depth <num>     appsrc max frame queue (default: %d)\n"
            "  --min-car-conf <v>      Car confidence threshold (default: 0.35)\n"
            "  --min-plate-conf <v>    Plate confidence threshold (default: 0.35)\n"
            "  --plate-on-car-only <0|1>  Reserve switch (default: 0)\n"
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
        {"labels", required_argument, NULL, 5},
        {"connector-id", required_argument, NULL, 6},
        {"fps", required_argument, NULL, 7},
        {"pixel-order", required_argument, NULL, 8},
        {"swap16", required_argument, NULL, 9},
        {"timeout-ms", required_argument, NULL, 10},
        {"stats-interval", required_argument, NULL, 11},
        {"copy-buffers", required_argument, NULL, 12},
        {"queue-depth", required_argument, NULL, 13},
        {"min-car-conf", required_argument, NULL, 14},
        {"min-plate-conf", required_argument, NULL, 15},
        {"plate-on-car-only", required_argument, NULL, 16},
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
    opt->min_plate_conf = 0.35f;
    opt->plate_on_car_only = 0;

    while ((c = getopt_long(argc, argv, "h", long_opts, NULL)) != -1) {
        switch (c) {
        case 1: opt->device_path = optarg; break;
        case 2: opt->drm_card_path = optarg; break;
        case 3: opt->veh_model_path = optarg; break;
        case 4: opt->plate_model_path = optarg; break;
        case 5: opt->labels_path = optarg; break;
        case 6: opt->connector_id = atoi(optarg); break;
        case 7: opt->fps = atoi(optarg); break;
        case 8:
            if (strcmp(optarg, "bgr565") == 0)
                opt->pixel_order = PIXEL_ORDER_BGR565;
            else if (strcmp(optarg, "rgb565") == 0)
                opt->pixel_order = PIXEL_ORDER_RGB565;
            else
                return -1;
            break;
        case 9: opt->swap16 = atoi(optarg) ? true : false; break;
        case 10: opt->timeout_ms = atoi(optarg); break;
        case 11: opt->stats_interval = atoi(optarg); break;
        case 12: opt->copy_buffers = atoi(optarg); break;
        case 13: opt->queue_depth = atoi(optarg); break;
        case 14: opt->min_car_conf = (float)atof(optarg); break;
        case 15: opt->min_plate_conf = (float)atof(optarg); break;
        case 16: opt->plate_on_car_only = atoi(optarg) ? 1 : 0; break;
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
    if (!opt->veh_model_path || !opt->plate_model_path || !opt->labels_path)
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
        if (nl) *nl = '\0';
        if (line[0] == '\0')
            continue;
        snprintf(ctx->labels[idx], MAX_LABEL_LEN, "%s", line);
        idx++;
    }
    fclose(fp);
    ctx->label_count = idx;
    ctx->car_class_id = 2;
    for (idx = 0; idx < ctx->label_count; idx++) {
        if (strcmp(ctx->labels[idx], "car") == 0) {
            ctx->car_class_id = idx;
            break;
        }
    }
    return 0;
}

static int init_fpga_dma(struct app_ctx *ctx)
{
    struct fpga_info info;
    struct buffer_map map;
    ctx->dev_fd = open(ctx->opt.device_path, O_RDWR | O_CLOEXEC);
    if (ctx->dev_fd < 0)
        return -1;
    if (ioctl(ctx->dev_fd, FPGA_DMA_GET_INFO, &info) < 0)
        return -1;
    if (info.frame_width != 1280 || info.frame_height != 720 || info.frame_bpp != 2)
        return -1;
    ctx->frame_width = info.frame_width;
    ctx->frame_height = info.frame_height;
    ctx->frame_bpp = info.frame_bpp;
    ctx->frame_size = (size_t)ctx->frame_width * ctx->frame_height * ctx->frame_bpp;

    memset(&map, 0, sizeof(map));
    map.index = 0;
    if (ioctl(ctx->dev_fd, FPGA_DMA_MAP_BUFFER, &map) < 0)
        return -1;
    ctx->dma_map_size = map.size;
    ctx->dma_map = mmap(NULL, ctx->dma_map_size, PROT_READ, MAP_SHARED, ctx->dev_fd, 0);
    if (ctx->dma_map == MAP_FAILED) {
        ctx->dma_map = NULL;
        return -1;
    }

    ctx->dma_copy = malloc(ctx->frame_size);
    if (!ctx->dma_copy)
        return -1;
    return 0;
}

static int trigger_frame_dma(struct app_ctx *ctx)
{
    struct dma_transfer t;
    memset(&t, 0, sizeof(t));
    t.size = (uint32_t)ctx->frame_size;
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

static void clamp_box(struct det_box *b, int w, int h)
{
    if (b->x1 < 0) b->x1 = 0;
    if (b->y1 < 0) b->y1 = 0;
    if (b->x2 >= w) b->x2 = w - 1;
    if (b->y2 >= h) b->y2 = h - 1;
    if (b->x2 < b->x1) b->x2 = b->x1;
    if (b->y2 < b->y1) b->y2 = b->y1;
}

static void decode_rows_output(const float *rows, int n_rows, int n_cols, int class_count,
                               float conf_thr, int src_w, int src_h, int in_w, int in_h,
                               struct det_box *out, int *out_count)
{
    int i;
    for (i = 0; i < n_rows && *out_count < MAX_DETS; i++) {
        const float *r = rows + i * n_cols;
        float obj = r[4];
        float best = 0.0f;
        int best_id = 0;
        int c;
        float cx = r[0], cy = r[1], bw = r[2], bh = r[3];
        struct det_box b;
        for (c = 0; c < class_count; c++) {
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
        if (a->n_dims == 3 && a->dims[2] >= 6)
            decode_rows_output((float *)outs[i].buf, (int)a->dims[1], (int)a->dims[2],
                               m->class_count, conf_thr, src_w, src_h,
                               (int)m->in_w, (int)m->in_h, out, out_count);
    }

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
    int total = 0, blue_cnt = 0, green_cnt = 0;
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
        }
    }
    if (total == 0) return PLATE_COLOR_UNKNOWN;
    if ((float)blue_cnt / (float)total >= 0.20f && blue_cnt > green_cnt + (int)(0.05f * total))
        return PLATE_COLOR_BLUE;
    if ((float)green_cnt / (float)total >= 0.20f && green_cnt > blue_cnt + (int)(0.05f * total))
        return PLATE_COLOR_GREEN;
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
    return "UNK";
}

static void *infer_thread_main(void *arg)
{
    struct app_ctx *ctx = (struct app_ctx *)arg;
    uint8_t *raw_local = malloc(ctx->frame_size);
    uint8_t *rgb_full = malloc((size_t)ctx->frame_width * ctx->frame_height * 3U);
    uint8_t *veh_in = malloc((size_t)ctx->veh_model.in_w * ctx->veh_model.in_h * 3U);
    uint8_t *plate_in = malloc((size_t)ctx->plate_model.in_w * ctx->plate_model.in_h * 3U);
    if (!raw_local || !rgb_full || !veh_in || !plate_in) {
        free(raw_local); free(rgb_full); free(veh_in); free(plate_in);
        return NULL;
    }

    while (ctx->running) {
        struct det_box cars[MAX_DETS];
        struct det_box plates[MAX_DETS];
        struct lpr_results r;
        int car_count = 0, plate_count = 0, i;
        int64_t t0, t1;
        uint64_t seq;

        pthread_mutex_lock(&ctx->infer_lock);
        while (ctx->running && !ctx->infer_has_new)
            pthread_cond_wait(&ctx->infer_cond, &ctx->infer_lock);
        if (!ctx->running) {
            pthread_mutex_unlock(&ctx->infer_lock);
            break;
        }
        memcpy(raw_local, ctx->infer_latest_raw, ctx->frame_size);
        seq = ctx->infer_frame_seq;
        ctx->infer_has_new = false;
        pthread_mutex_unlock(&ctx->infer_lock);

        t0 = mono_us();
        raw565_to_rgb888_full(ctx, raw_local, rgb_full);
        resize_rgb888_nn(rgb_full, (int)ctx->frame_width, (int)ctx->frame_height,
                         veh_in, (int)ctx->veh_model.in_w, (int)ctx->veh_model.in_h);
        resize_rgb888_nn(rgb_full, (int)ctx->frame_width, (int)ctx->frame_height,
                         plate_in, (int)ctx->plate_model.in_w, (int)ctx->plate_model.in_h);

        if (run_model_detect(&ctx->veh_model, veh_in, (int)ctx->frame_width, (int)ctx->frame_height,
                             ctx->opt.min_car_conf, cars, &car_count) < 0)
            car_count = 0;
        if (run_model_detect(&ctx->plate_model, plate_in, (int)ctx->frame_width, (int)ctx->frame_height,
                             ctx->opt.min_plate_conf, plates, &plate_count) < 0)
            plate_count = 0;
        t1 = mono_us();

        memset(&r, 0, sizeof(r));
        for (i = 0; i < car_count && r.car_count < MAX_DETS; i++) {
            if (cars[i].cls == ctx->car_class_id)
                r.cars[r.car_count++] = cars[i];
        }
        for (i = 0; i < plate_count && r.plate_count < MAX_DETS; i++) {
            struct plate_det pd;
            int parent = find_parent_car(&plates[i], r.cars, r.car_count);
            if (parent < 0)
                continue;
            pd.box = plates[i];
            pd.parent_car = parent;
            pd.color = classify_plate_color_rgb(rgb_full, (int)ctx->frame_width, (int)ctx->frame_height, &plates[i]);
            r.plates[r.plate_count++] = pd;
        }
        r.frame_seq = seq;
        r.infer_ms_last = (double)(t1 - t0) / 1000.0;
        pthread_mutex_lock(&ctx->result_lock);
        r.infer_frames_total = ctx->results.infer_frames_total + 1;
        r.infer_ms_total = ctx->results.infer_ms_total + r.infer_ms_last;
        ctx->results = r;
        pthread_mutex_unlock(&ctx->result_lock);
    }

    free(raw_local); free(rgb_full); free(veh_in); free(plate_in);
    return NULL;
}

static void overlay_results_on_slot(struct app_ctx *ctx, uint8_t *slot_data)
{
    struct lpr_results r;
    uint16_t *pix = (uint16_t *)slot_data;
    int i;
    pthread_mutex_lock(&ctx->result_lock);
    r = ctx->results;
    pthread_mutex_unlock(&ctx->result_lock);

    for (i = 0; i < r.car_count; i++)
        draw_rect_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, &r.cars[i], COLOR_YELLOW_565);

    for (i = 0; i < r.plate_count; i++) {
        const char *txt = plate_color_str(r.plates[i].color);
        int tx = r.plates[i].box.x1;
        int ty = r.plates[i].box.y1 - 10;
        if (ty < 0) ty = r.plates[i].box.y1 + 2;
        draw_rect_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, &r.plates[i].box, COLOR_CYAN_565);
        draw_text_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, tx, ty, txt, COLOR_CYAN_565);
    }
}

static void push_latest_to_infer(struct app_ctx *ctx, const uint8_t *raw)
{
    pthread_mutex_lock(&ctx->infer_lock);
    if (ctx->infer_has_new)
        ctx->infer_overwrite_count++;
    memcpy(ctx->infer_latest_raw, raw, ctx->frame_size);
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
            " infer=%" PRIu64 " infer_ms=%.2f drop=%" PRIu64 " cap_fps=%.2f disp_fps=%.2f infer_fps=%.2f\n",
            ctx->captured_frames, ctx->pushed_frames, ctx->released_frames,
            r.infer_frames_total, r.infer_ms_last, ctx->infer_overwrite_count,
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
    if (init_fpga_dma(&ctx) < 0)
        goto out;
    if (init_copy_slots(&ctx) < 0)
        goto out;
    if (rknn_model_load(&ctx.veh_model, "vehicle", ctx.opt.veh_model_path, ctx.label_count) < 0)
        goto out;
    if (rknn_model_load(&ctx.plate_model, "plate", ctx.opt.plate_model_path, 1) < 0)
        goto out;

    ctx.infer_latest_raw = malloc(ctx.frame_size);
    if (!ctx.infer_latest_raw)
        goto out;
    if (build_pipeline(&ctx) < 0)
        goto out;
    if (pthread_create(&ctx.infer_thread, NULL, infer_thread_main, &ctx) != 0)
        goto out;

    fprintf(stderr,
            "Start LPR loop: fps=%d pixel=%s swap16=%s min_car=%.2f min_plate=%.2f\n",
            ctx.opt.fps,
            (ctx.opt.pixel_order == PIXEL_ORDER_BGR565) ? "bgr565" : "rgb565",
            ctx.opt.swap16 ? "on" : "off",
            ctx.opt.min_car_conf, ctx.opt.min_plate_conf);

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
        push_latest_to_infer(&ctx, ctx.dma_copy);

        if (acquire_free_slot(&ctx, &ticket) < 0)
            break;
        copy_frame_to_slot565(&ctx, ctx.slots[ticket.idx].data, ctx.dma_copy);
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

