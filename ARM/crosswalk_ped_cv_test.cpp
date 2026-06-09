// SPDX-License-Identifier: GPL-2.0
/*
 * Independent crosswalk / road / sidewalk CV test with pedestrian RKNN.
 *
 * This intentionally does not link against fpga_lpr_display.c or OCR code.
 * It uses the FPGA DMA userspace ABI directly, runs one pedestrian detector,
 * estimates scene regions with OpenCV, and overlays the test state via KMS.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <getopt.h>
#include <inttypes.h>
#include <map>
#include <numeric>
#include <signal.h>
#include <string>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rknn_api.h>

#include "pcie_fpga_dma.h"

#define DEFAULT_DEVICE "/dev/" FPGA_DMA_DEV_NAME
#define DEFAULT_DRM_CARD "/dev/dri/card0"
#define MAX_DETS 128

enum pixel_order {
    PIXEL_ORDER_BGR565 = 0,
    PIXEL_ORDER_RGB565,
};

struct options {
    const char *device_path = DEFAULT_DEVICE;
    const char *drm_card_path = DEFAULT_DRM_CARD;
    const char *ped_model_path = nullptr;
    const char *labels_path = nullptr;
    int connector_id = -1;
    int fps = 15;
    enum pixel_order pixel_order = PIXEL_ORDER_BGR565;
    bool swap16 = true;
    int timeout_ms = 5000;
    int stats_interval = 1;
    int copy_buffers = 3;
    int queue_depth = 1;
    float min_person_conf = 0.35f;
    int cv_every_n = 3;
    int scene_smooth = 10;
    const char *debug_dump_dir = nullptr;
    int debug_dump_every_n = 30;
    int debug_dump_max = 30;
};

struct det_box {
    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;
    float conf = 0.0f;
    int cls = -1;
};

struct letterbox_meta {
    float scale = 1.0f;
    int pad_x = 0;
    int pad_y = 0;
    int src_w = 0;
    int src_h = 0;
    int dst_w = 0;
    int dst_h = 0;
};

struct yolo_model {
    const char *path = nullptr;
    rknn_context ctx = 0;
    rknn_input_output_num io_num{};
    rknn_tensor_attr input_attr{};
    rknn_tensor_attr output_attrs[8]{};
    uint32_t in_w = 0;
    uint32_t in_h = 0;
    uint32_t in_c = 0;
    int class_count = 80;
    int person_class_id = 0;
};

enum region_state {
    REGION_UNKNOWN = 0,
    REGION_SIDEWALK,
    REGION_ROAD,
    REGION_CROSSWALK,
};

struct track_state {
    cv::Rect box;
    cv::Point foot;
    region_state last_region = REGION_UNKNOWN;
    bool came_from_sidewalk = false;
    int ttl = 0;
};

struct scene_state {
    std::vector<cv::Point> crosswalk_poly;
    std::vector<cv::Point> road_poly;
    bool crosswalk_valid = false;
    bool crosswalk_hold = false;
    bool road_valid = false;
    bool road_hold = false;
    int crosswalk_ttl = 0;
    int road_ttl = 0;
};

enum output_layout_kind {
    OUTPUT_LAYOUT_UNKNOWN = 0,
    OUTPUT_LAYOUT_YOLO_OBJ_CLASSES,
    OUTPUT_LAYOUT_YOLO_CLASSES,
    OUTPUT_LAYOUT_SINGLE_CLASS_CONF,
};

struct app_ctx {
    options opt;
    int dev_fd = -1;
    int drm_fd = -1;
    uint32_t frame_width = 0;
    uint32_t frame_height = 0;
    uint32_t frame_bpp = 0;
    uint32_t frame_stride = 0;
    uint32_t pixel_format = FPGA_PIXEL_FORMAT_BGR565;
    size_t frame_size = 0;
    bool source_is_bgrx = false;
    std::vector<uint8_t> dma_copy;
    std::vector<uint8_t> display_bgrx;
    std::vector<std::string> labels;
    yolo_model ped_model;
    scene_state scene;
    std::vector<track_state> tracks;
    GstElement *pipeline = nullptr;
    GstElement *appsrc = nullptr;
    GstElement *queue = nullptr;
    GstElement *sink = nullptr;
    GstBus *bus = nullptr;
    uint64_t frame_seq = 0;
    uint64_t pushed_frames = 0;
    uint64_t crossing_total = 0;
    int debug_dumped = 0;
    int64_t last_stats_us = 0;
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

static float sigmoidf_local(float x)
{
    return 1.0f / (1.0f + std::exp(-x));
}

static void print_usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s --ped-model <path> --labels <path> [OPTIONS]\n"
            "  --device <path>          FPGA device (default: %s)\n"
            "  --drm-card <path>        DRM card (default: %s)\n"
            "  --ped-model <path>       Pedestrian RKNN model path\n"
            "  --labels <path>          Labels file path\n"
            "  --connector-id <id>      Optional KMS connector id\n"
            "  --fps <num>              Target FPS (default: 15)\n"
            "  --pixel-order <mode>     bgr565|rgb565 (default: bgr565)\n"
            "  --swap16 <0|1>           Swap bytes in 16-bit source pixels (default: 1)\n"
            "  --timeout-ms <ms>        Reserved wait timeout (default: 5000)\n"
            "  --stats-interval <sec>   Stats interval (default: 1)\n"
            "  --copy-buffers <num>     Reserved copy buffers (default: 3)\n"
            "  --queue-depth <num>      appsrc queue depth (default: 1)\n"
            "  --min-person-conf <v>    Person confidence threshold (default: 0.35)\n"
            "  --cv-every-n <n>         Run CV every N frames (default: 3)\n"
            "  --scene-smooth <n>       ROI hold TTL in CV samples (default: 10)\n"
            "  --debug-dump-dir <path>  Dump raw/overlay PPM and metadata text (default: off)\n"
            "  --debug-dump-every-n <n> Dump every N frames when enabled (default: 30)\n"
            "  --debug-dump-max <n>     Max debug samples to dump (default: 30)\n",
            prog, DEFAULT_DEVICE, DEFAULT_DRM_CARD);
}

static int parse_bool(const char *s, bool *out)
{
    if (!strcmp(s, "1") || !strcasecmp(s, "true") || !strcasecmp(s, "on")) {
        *out = true;
        return 0;
    }
    if (!strcmp(s, "0") || !strcasecmp(s, "false") || !strcasecmp(s, "off")) {
        *out = false;
        return 0;
    }
    return -1;
}

static int parse_options(int argc, char **argv, options *opt)
{
    static const struct option long_opts[] = {
        {"device", required_argument, NULL, 1},
        {"drm-card", required_argument, NULL, 2},
        {"ped-model", required_argument, NULL, 3},
        {"labels", required_argument, NULL, 4},
        {"connector-id", required_argument, NULL, 5},
        {"fps", required_argument, NULL, 6},
        {"pixel-order", required_argument, NULL, 7},
        {"swap16", required_argument, NULL, 8},
        {"timeout-ms", required_argument, NULL, 9},
        {"stats-interval", required_argument, NULL, 10},
        {"copy-buffers", required_argument, NULL, 11},
        {"queue-depth", required_argument, NULL, 12},
        {"min-person-conf", required_argument, NULL, 13},
        {"cv-every-n", required_argument, NULL, 14},
        {"scene-smooth", required_argument, NULL, 15},
        {"debug-dump-dir", required_argument, NULL, 16},
        {"debug-dump-every-n", required_argument, NULL, 17},
        {"debug-dump-max", required_argument, NULL, 18},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };

    int c;
    while ((c = getopt_long(argc, argv, "h", long_opts, NULL)) != -1) {
        switch (c) {
        case 1: opt->device_path = optarg; break;
        case 2: opt->drm_card_path = optarg; break;
        case 3: opt->ped_model_path = optarg; break;
        case 4: opt->labels_path = optarg; break;
        case 5: opt->connector_id = atoi(optarg); break;
        case 6: opt->fps = atoi(optarg); break;
        case 7:
            if (!strcmp(optarg, "bgr565")) opt->pixel_order = PIXEL_ORDER_BGR565;
            else if (!strcmp(optarg, "rgb565")) opt->pixel_order = PIXEL_ORDER_RGB565;
            else { fprintf(stderr, "Invalid --pixel-order: %s\n", optarg); return -1; }
            break;
        case 8:
            if (parse_bool(optarg, &opt->swap16) < 0) {
                fprintf(stderr, "Invalid --swap16: %s\n", optarg);
                return -1;
            }
            break;
        case 9: opt->timeout_ms = atoi(optarg); break;
        case 10: opt->stats_interval = atoi(optarg); break;
        case 11: opt->copy_buffers = atoi(optarg); break;
        case 12: opt->queue_depth = atoi(optarg); break;
        case 13: opt->min_person_conf = (float)atof(optarg); break;
        case 14: opt->cv_every_n = atoi(optarg); break;
        case 15: opt->scene_smooth = atoi(optarg); break;
        case 16: opt->debug_dump_dir = optarg; break;
        case 17: opt->debug_dump_every_n = atoi(optarg); break;
        case 18: opt->debug_dump_max = atoi(optarg); break;
        case 'h': print_usage(argv[0]); exit(0);
        default: return -1;
        }
    }

    if (!opt->ped_model_path || !opt->labels_path) {
        fprintf(stderr, "Missing required --ped-model and --labels\n");
        return -1;
    }
    if (opt->fps <= 0 || opt->fps > 120) return -1;
    if (opt->timeout_ms <= 0) return -1;
    if (opt->stats_interval <= 0) return -1;
    if (opt->copy_buffers < 1 || opt->copy_buffers > 8) return -1;
    if (opt->queue_depth <= 0) return -1;
    if (opt->min_person_conf < 0.0f || opt->min_person_conf > 1.0f) return -1;
    if (opt->cv_every_n <= 0 || opt->cv_every_n > 120) return -1;
    if (opt->scene_smooth <= 0 || opt->scene_smooth > 300) return -1;
    if (opt->debug_dump_every_n <= 0 || opt->debug_dump_every_n > 10000) return -1;
    if (opt->debug_dump_max < 0 || opt->debug_dump_max > 100000) return -1;
    return 0;
}

static int load_labels(app_ctx *ctx)
{
    std::ifstream in(ctx->opt.labels_path);
    std::string line;
    if (!in)
        return -1;
    while (std::getline(in, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' '))
            line.pop_back();
        if (!line.empty())
            ctx->labels.push_back(line);
    }
    if (ctx->labels.empty())
        return -1;
    ctx->ped_model.class_count = (int)ctx->labels.size();
    ctx->ped_model.person_class_id = 0;
    for (size_t i = 0; i < ctx->labels.size(); i++) {
        if (ctx->labels[i] == "person") {
            ctx->ped_model.person_class_id = (int)i;
            break;
        }
    }
    fprintf(stderr, "Labels loaded: %zu person_class_id=%d\n",
            ctx->labels.size(), ctx->ped_model.person_class_id);
    return 0;
}

static const char *tensor_fmt_name(int fmt)
{
    switch (fmt) {
    case RKNN_TENSOR_NCHW: return "NCHW";
    case RKNN_TENSOR_NHWC: return "NHWC";
    default: return "OTHER";
    }
}

static int rknn_model_load(yolo_model *m, const char *path)
{
    FILE *fp = fopen(path, "rb");
    long sz;
    void *data;
    if (!fp)
        return -1;
    fseek(fp, 0, SEEK_END);
    sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (sz <= 0) {
        fclose(fp);
        return -1;
    }
    data = malloc((size_t)sz);
    if (!data) {
        fclose(fp);
        return -1;
    }
    if (fread(data, 1, (size_t)sz, fp) != (size_t)sz) {
        fclose(fp);
        free(data);
        return -1;
    }
    fclose(fp);

    m->path = path;
    if (rknn_init(&m->ctx, data, (uint32_t)sz, 0, NULL) < 0) {
        free(data);
        return -1;
    }
    free(data);

    if (rknn_query(m->ctx, RKNN_QUERY_IN_OUT_NUM, &m->io_num, sizeof(m->io_num)) < 0)
        return -1;
    if (m->io_num.n_output > 8)
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
    if (m->in_c != 3 || m->in_w <= 0 || m->in_h <= 0)
        return -1;

    for (uint32_t i = 0; i < m->io_num.n_output; i++) {
        memset(&m->output_attrs[i], 0, sizeof(m->output_attrs[i]));
        m->output_attrs[i].index = i;
        if (rknn_query(m->ctx, RKNN_QUERY_OUTPUT_ATTR, &m->output_attrs[i], sizeof(m->output_attrs[i])) < 0)
            return -1;
    }

    fprintf(stderr, "ped model loaded: input=%ux%u fmt=%s outputs=%u classes=%d\n",
            m->in_w, m->in_h, tensor_fmt_name(m->input_attr.fmt),
            m->io_num.n_output, m->class_count);
    for (uint32_t i = 0; i < m->io_num.n_output; i++) {
        const rknn_tensor_attr *a = &m->output_attrs[i];
        fprintf(stderr, "  out[%u]: n_dims=%u dims=%u,%u,%u,%u fmt=%s\n",
                i, a->n_dims, a->dims[0], a->dims[1], a->dims[2], a->dims[3],
                tensor_fmt_name(a->fmt));
        for (uint32_t j = 0; j < a->n_dims; j++) {
            if (a->dims[j] == 5 && m->class_count != 1) {
                fprintf(stderr,
                        "  warn: output stride 5 is single-class [x,y,w,h,conf]; "
                        "labels has %d entries. Prefer a one-line labels file containing person.\n",
                        m->class_count);
                break;
            }
        }
    }
    return 0;
}

static void rknn_model_release(yolo_model *m)
{
    if (m->ctx)
        rknn_destroy(m->ctx);
    m->ctx = 0;
}

static int init_fpga_dma(app_ctx *ctx)
{
    struct fpga_info info{};
    ctx->dev_fd = open(ctx->opt.device_path, O_RDWR | O_CLOEXEC);
    if (ctx->dev_fd < 0) {
        perror("open fpga device");
        return -1;
    }
    if (ioctl(ctx->dev_fd, FPGA_DMA_GET_INFO, &info) < 0) {
        perror("FPGA_DMA_GET_INFO");
        return -1;
    }
    if (info.pixel_format == FPGA_PIXEL_FORMAT_BGRX8888)
        info.frame_bpp = 4;
    else if (info.pixel_format == FPGA_PIXEL_FORMAT_BGR565)
        info.frame_bpp = 2;
    else if (info.frame_bpp == 4)
        info.pixel_format = FPGA_PIXEL_FORMAT_BGRX8888;
    else
        info.pixel_format = FPGA_PIXEL_FORMAT_BGR565;

    if (info.frame_stride < info.frame_width * info.frame_bpp)
        info.frame_stride = info.frame_width * info.frame_bpp;
    if (info.frame_bpp != 2 && info.frame_bpp != 4) {
        fprintf(stderr, "Unsupported frame_bpp=%u\n", info.frame_bpp);
        return -1;
    }

    ctx->frame_width = info.frame_width;
    ctx->frame_height = info.frame_height;
    ctx->frame_bpp = info.frame_bpp;
    ctx->frame_stride = info.frame_stride;
    ctx->pixel_format = info.pixel_format;
    ctx->frame_size = (size_t)ctx->frame_stride * ctx->frame_height;
    ctx->source_is_bgrx = (ctx->pixel_format == FPGA_PIXEL_FORMAT_BGRX8888) || (ctx->frame_bpp == 4);
    ctx->dma_copy.resize(ctx->frame_size);
    ctx->display_bgrx.resize((size_t)ctx->frame_width * ctx->frame_height * 4U);

    fprintf(stderr, "FPGA DMA ready: %ux%u bpp=%u stride=%u frame=%zu source=%s\n",
            ctx->frame_width, ctx->frame_height, ctx->frame_bpp, ctx->frame_stride,
            ctx->frame_size, ctx->source_is_bgrx ? "BGRX" : "565");
    return 0;
}

static int trigger_frame_dma(app_ctx *ctx)
{
    struct dma_transfer transfer{};
    transfer.size = (uint32_t)ctx->frame_size;
    transfer.user_buf = (uint64_t)(uintptr_t)ctx->dma_copy.data();
    if (ioctl(ctx->dev_fd, FPGA_DMA_READ_FRAME, &transfer) < 0) {
        perror("FPGA_DMA_READ_FRAME");
        return -1;
    }
    if (transfer.result != 0) {
        fprintf(stderr, "FPGA_DMA_READ_FRAME result=%u\n", transfer.result);
        return -1;
    }
    return 0;
}

static cv::Mat frame_to_bgr(app_ctx *ctx)
{
    cv::Mat bgr((int)ctx->frame_height, (int)ctx->frame_width, CV_8UC3);
    const uint8_t *src = ctx->dma_copy.data();
    if (ctx->source_is_bgrx) {
        for (uint32_t y = 0; y < ctx->frame_height; y++) {
            const uint8_t *row = src + (size_t)y * ctx->frame_stride;
            cv::Vec3b *dst = bgr.ptr<cv::Vec3b>((int)y);
            for (uint32_t x = 0; x < ctx->frame_width; x++) {
                const uint8_t *p = row + (size_t)x * 4U;
                dst[x] = cv::Vec3b(p[0], p[1], p[2]);
            }
        }
        return bgr;
    }

    for (uint32_t y = 0; y < ctx->frame_height; y++) {
        const uint8_t *row = src + (size_t)y * ctx->frame_stride;
        cv::Vec3b *dst = bgr.ptr<cv::Vec3b>((int)y);
        for (uint32_t x = 0; x < ctx->frame_width; x++) {
            uint8_t lo = row[(size_t)x * 2U];
            uint8_t hi = row[(size_t)x * 2U + 1U];
            if (ctx->opt.swap16)
                std::swap(lo, hi);
            uint16_t pix = (uint16_t)lo | ((uint16_t)hi << 8);
            uint8_t r5, g6, b5;
            if (ctx->opt.pixel_order == PIXEL_ORDER_BGR565) {
                r5 = pix & 0x1F;
                g6 = (pix >> 5) & 0x3F;
                b5 = (pix >> 11) & 0x1F;
            } else {
                r5 = (pix >> 11) & 0x1F;
                g6 = (pix >> 5) & 0x3F;
                b5 = pix & 0x1F;
            }
            dst[x] = cv::Vec3b((uint8_t)((b5 << 3) | (b5 >> 2)),
                               (uint8_t)((g6 << 2) | (g6 >> 4)),
                               (uint8_t)((r5 << 3) | (r5 >> 2)));
        }
    }
    return bgr;
}

static int build_pipeline(app_ctx *ctx)
{
    ctx->drm_fd = open(ctx->opt.drm_card_path, O_RDWR | O_CLOEXEC);
    if (ctx->drm_fd < 0) {
        perror("open drm card");
        return -1;
    }

    ctx->pipeline = gst_pipeline_new("crosswalk-cv-test");
    ctx->appsrc = gst_element_factory_make("appsrc", "src");
    ctx->queue = gst_element_factory_make("queue", "latency_queue");
    ctx->sink = gst_element_factory_make("kmssink", "sink");
    if (!ctx->pipeline || !ctx->appsrc || !ctx->queue || !ctx->sink)
        return -1;
    gst_bin_add_many(GST_BIN(ctx->pipeline), ctx->appsrc, ctx->queue, ctx->sink, NULL);
    if (!gst_element_link_many(ctx->appsrc, ctx->queue, ctx->sink, NULL))
        return -1;

    GstCaps *caps = gst_caps_new_simple("video/x-raw",
                                        "format", G_TYPE_STRING, "BGRx",
                                        "width", G_TYPE_INT, (int)ctx->frame_width,
                                        "height", G_TYPE_INT, (int)ctx->frame_height,
                                        "framerate", GST_TYPE_FRACTION, ctx->opt.fps, 1,
                                        NULL);
    g_object_set(ctx->appsrc,
                 "caps", caps,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "format", GST_FORMAT_TIME,
                 "block", FALSE,
                 "max-bytes", (guint64)ctx->display_bgrx.size() * (guint64)ctx->opt.queue_depth,
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
    if (gst_element_set_state(ctx->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        return -1;
    if (gst_element_get_state(ctx->pipeline, NULL, NULL, 5 * GST_SECOND) == GST_STATE_CHANGE_FAILURE)
        return -1;
    fprintf(stderr, "Pipeline started: appsrc(BGRx)->queue(leaky)->kmssink\n");
    return 0;
}

static bool handle_bus(app_ctx *ctx)
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
            gst_message_unref(msg);
            return false;
        }
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
            gst_message_unref(msg);
            return false;
        }
        gst_message_unref(msg);
    }
    return true;
}

static float box_iou(const det_box &a, const det_box &b)
{
    int x1 = std::max(a.x1, b.x1);
    int y1 = std::max(a.y1, b.y1);
    int x2 = std::min(a.x2, b.x2);
    int y2 = std::min(a.y2, b.y2);
    int iw = x2 - x1 + 1;
    int ih = y2 - y1 + 1;
    if (iw <= 0 || ih <= 0)
        return 0.0f;
    int ia = iw * ih;
    int ua = (a.x2 - a.x1 + 1) * (a.y2 - a.y1 + 1) +
             (b.x2 - b.x1 + 1) * (b.y2 - b.y1 + 1) - ia;
    return ua > 0 ? (float)ia / (float)ua : 0.0f;
}

static void clamp_box(det_box &b, int w, int h)
{
    b.x1 = std::max(0, std::min(w - 1, b.x1));
    b.x2 = std::max(0, std::min(w - 1, b.x2));
    b.y1 = std::max(0, std::min(h - 1, b.y1));
    b.y2 = std::max(0, std::min(h - 1, b.y2));
    if (b.x2 < b.x1) b.x2 = b.x1;
    if (b.y2 < b.y1) b.y2 = b.y1;
}

static cv::Mat letterbox_rgb(const cv::Mat &rgb, int dst_w, int dst_h, letterbox_meta *meta)
{
    float scale = std::min((float)dst_w / (float)rgb.cols, (float)dst_h / (float)rgb.rows);
    int sw = std::max(1, (int)std::round(rgb.cols * scale));
    int sh = std::max(1, (int)std::round(rgb.rows * scale));
    cv::Mat out(dst_h, dst_w, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(sw, sh), 0, 0, cv::INTER_LINEAR);
    int ox = (dst_w - sw) / 2;
    int oy = (dst_h - sh) / 2;
    resized.copyTo(out(cv::Rect(ox, oy, sw, sh)));
    if (meta) {
        meta->scale = scale;
        meta->pad_x = ox;
        meta->pad_y = oy;
        meta->src_w = rgb.cols;
        meta->src_h = rgb.rows;
        meta->dst_w = dst_w;
        meta->dst_h = dst_h;
    }
    return out;
}

static void map_from_letterbox(det_box &b, const letterbox_meta &lb)
{
    b.x1 = (int)std::round(((float)b.x1 - lb.pad_x) / lb.scale);
    b.x2 = (int)std::round(((float)b.x2 - lb.pad_x) / lb.scale);
    b.y1 = (int)std::round(((float)b.y1 - lb.pad_y) / lb.scale);
    b.y2 = (int)std::round(((float)b.y2 - lb.pad_y) / lb.scale);
    clamp_box(b, lb.src_w, lb.src_h);
}

static bool choose_stride_layout(const rknn_tensor_attr &attr, int class_count,
                                 bool *channel_first, int *stride, int *count,
                                 output_layout_kind *layout)
{
    size_t elems = 1;
    std::vector<int> dims;
    for (uint32_t i = 0; i < attr.n_dims; i++) {
        if (attr.dims[i] > 0) {
            dims.push_back((int)attr.dims[i]);
            elems *= attr.dims[i];
        }
    }
    int y5 = 5 + class_count;
    int y8 = 4 + class_count;

    for (int d : dims) {
        if ((d == y8 || d == y5) && elems % (size_t)d == 0) {
            *stride = d;
            *count = (int)(elems / (size_t)d);
            *channel_first = (dims.size() >= 3 && dims[1] == d && dims.back() != d);
            *layout = (d == y5) ? OUTPUT_LAYOUT_YOLO_OBJ_CLASSES : OUTPUT_LAYOUT_YOLO_CLASSES;
            return true;
        }
    }
    for (int d : dims) {
        if (d == 5 && elems % (size_t)d == 0) {
            *stride = d;
            *count = (int)(elems / (size_t)d);
            *channel_first = (dims.size() >= 3 && dims[1] == d && dims.back() != d);
            *layout = OUTPUT_LAYOUT_SINGLE_CLASS_CONF;
            return true;
        }
    }
    for (int d : dims) {
        if (d >= 6 && d <= 256 && elems % (size_t)d == 0) {
            *stride = d;
            *count = (int)(elems / (size_t)d);
            *channel_first = (dims.size() >= 3 && dims[1] == d && dims.back() != d);
            *layout = (d >= 5 + class_count) ? OUTPUT_LAYOUT_YOLO_OBJ_CLASSES : OUTPUT_LAYOUT_YOLO_CLASSES;
            return true;
        }
    }
    return false;
}

static float output_value(const float *buf, bool channel_first, int stride, int count, int row, int col)
{
    return channel_first ? buf[col * count + row] : buf[row * stride + col];
}

static void decode_output(const float *buf, const rknn_tensor_attr &attr, const yolo_model &m,
                          float conf_thr, std::vector<det_box> *out)
{
    bool channel_first = false;
    int stride = 0;
    int count = 0;
    output_layout_kind layout = OUTPUT_LAYOUT_UNKNOWN;
    if (!choose_stride_layout(attr, m.class_count, &channel_first, &stride, &count, &layout))
        return;

    bool single_class_conf = (layout == OUTPUT_LAYOUT_SINGLE_CLASS_CONF);
    bool has_obj = (layout == OUTPUT_LAYOUT_YOLO_OBJ_CLASSES);
    int cls_base = has_obj ? 5 : 4;
    int cls_n = single_class_conf ? 0 : std::min(m.class_count, stride - cls_base);
    if (!single_class_conf && cls_n <= 0)
        return;

    for (int i = 0; i < count; i++) {
        float cx = output_value(buf, channel_first, stride, count, i, 0);
        float cy = output_value(buf, channel_first, stride, count, i, 1);
        float bw = output_value(buf, channel_first, stride, count, i, 2);
        float bh = output_value(buf, channel_first, stride, count, i, 3);
        float obj = (has_obj || single_class_conf) ?
            output_value(buf, channel_first, stride, count, i, 4) : 1.0f;
        if (obj > 1.0f || obj < 0.0f)
            obj = sigmoidf_local(obj);

        int best_cls = m.person_class_id;
        float best_score = 1.0f;
        if (!single_class_conf) {
            best_cls = -1;
            best_score = 0.0f;
            for (int c = 0; c < cls_n; c++) {
                float s = output_value(buf, channel_first, stride, count, i, cls_base + c);
                if (s > 1.0f || s < 0.0f)
                    s = sigmoidf_local(s);
                if (s > best_score) {
                    best_score = s;
                    best_cls = c;
                }
            }
        }
        float conf = obj * best_score;
        if (best_cls != m.person_class_id || conf < conf_thr)
            continue;
        if (cx >= 0.0f && cx <= 1.5f && cy >= 0.0f && cy <= 1.5f &&
            bw > 0.0f && bw <= 1.5f && bh > 0.0f && bh <= 1.5f) {
            cx *= (float)m.in_w;
            bw *= (float)m.in_w;
            cy *= (float)m.in_h;
            bh *= (float)m.in_h;
        }

        det_box b;
        b.x1 = (int)std::round(cx - bw * 0.5f);
        b.y1 = (int)std::round(cy - bh * 0.5f);
        b.x2 = (int)std::round(cx + bw * 0.5f);
        b.y2 = (int)std::round(cy + bh * 0.5f);
        b.conf = conf;
        b.cls = best_cls;
        clamp_box(b, (int)m.in_w, (int)m.in_h);
        if (b.x2 > b.x1 && b.y2 > b.y1)
            out->push_back(b);
    }
}

static int run_ped_detect(app_ctx *ctx, const cv::Mat &bgr, std::vector<det_box> *persons)
{
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    letterbox_meta lb;
    cv::Mat input = letterbox_rgb(rgb, (int)ctx->ped_model.in_w, (int)ctx->ped_model.in_h, &lb);

    rknn_input in{};
    in.index = 0;
    in.buf = input.data;
    in.size = (uint32_t)(input.total() * input.elemSize());
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    int ret = rknn_inputs_set(ctx->ped_model.ctx, 1, &in);
    if (ret < 0) return ret;
    ret = rknn_run(ctx->ped_model.ctx, NULL);
    if (ret < 0) return ret;

    rknn_output outs[8]{};
    for (uint32_t i = 0; i < ctx->ped_model.io_num.n_output; i++)
        outs[i].want_float = 1;
    ret = rknn_outputs_get(ctx->ped_model.ctx, ctx->ped_model.io_num.n_output, outs, NULL);
    if (ret < 0) return ret;

    std::vector<det_box> raw;
    for (uint32_t i = 0; i < ctx->ped_model.io_num.n_output; i++)
        decode_output((const float *)outs[i].buf, ctx->ped_model.output_attrs[i],
                      ctx->ped_model, ctx->opt.min_person_conf, &raw);
    rknn_outputs_release(ctx->ped_model.ctx, ctx->ped_model.io_num.n_output, outs);

    std::sort(raw.begin(), raw.end(), [](const det_box &a, const det_box &b) {
        return a.conf > b.conf;
    });
    std::vector<bool> suppressed(raw.size(), false);
    persons->clear();
    for (size_t i = 0; i < raw.size() && persons->size() < MAX_DETS; i++) {
        if (suppressed[i])
            continue;
        det_box b = raw[i];
        map_from_letterbox(b, lb);
        persons->push_back(b);
        for (size_t j = i + 1; j < raw.size(); j++) {
            if (!suppressed[j] && box_iou(raw[i], raw[j]) > 0.45f)
                suppressed[j] = true;
        }
    }
    return 0;
}

static bool point_in_poly(const std::vector<cv::Point> &poly, const cv::Point &p)
{
    if (poly.size() < 3)
        return false;
    return cv::pointPolygonTest(poly, p, false) >= 0.0;
}

static float polygon_area_abs(const std::vector<cv::Point> &poly)
{
    if (poly.size() < 3)
        return 0.0f;
    return (float)std::fabs(cv::contourArea(poly));
}

static bool frame_has_active_scene(const cv::Mat &bgr)
{
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Scalar mean = cv::mean(gray);
    cv::Mat bright;
    cv::threshold(gray, bright, 80, 255, cv::THRESH_BINARY);
    double bright_ratio = (double)cv::countNonZero(bright) / (double)gray.total();

    return mean[0] >= 45.0 && bright_ratio >= 0.12;
}

static void update_scene_hold(scene_state *scene, int ttl)
{
    if (scene->crosswalk_ttl > 0) {
        scene->crosswalk_ttl--;
        scene->crosswalk_valid = true;
        scene->crosswalk_hold = true;
    } else {
        scene->crosswalk_valid = false;
        scene->crosswalk_hold = false;
        scene->crosswalk_poly.clear();
    }
    if (scene->road_ttl > 0) {
        scene->road_ttl--;
        scene->road_valid = true;
        scene->road_hold = true;
    } else {
        scene->road_valid = false;
        scene->road_hold = false;
        scene->road_poly.clear();
    }
    (void)ttl;
}

static bool detect_crosswalk(const cv::Mat &bgr, std::vector<cv::Point> *poly)
{
    struct stripe_seg {
        int x1;
        int y1;
        int x2;
        int y2;
        float len;
        float angle;
        float cx;
        float cy;
    };

    cv::Mat small, hsv, roi, mask, morph, edges;
    double scale = 640.0 / (double)bgr.cols;
    cv::resize(bgr, small, cv::Size(), scale, scale, cv::INTER_AREA);
    int h = small.rows;
    int w = small.cols;
    int y0 = (int)std::round((double)h * 0.50);
    int y1 = (int)std::round((double)h * 0.98);
    cv::cvtColor(small, hsv, cv::COLOR_BGR2HSV);
    roi = hsv(cv::Rect(0, y0, w, y1 - y0));
    cv::inRange(roi, cv::Scalar(0, 0, 115), cv::Scalar(179, 155, 255), mask);
    cv::morphologyEx(mask, morph, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 3)));
    cv::Canny(morph, edges, 50, 140);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180.0, 25, 25, 15);
    std::vector<stripe_seg> segs;
    for (const auto &l : lines) {
        int x1l = l[0];
        int y1l = l[1] + y0;
        int x2l = l[2];
        int y2l = l[3] + y0;
        float dx = (float)(x2l - x1l);
        float dy = (float)(y2l - y1l);
        float len = std::hypot(dx, dy);
        if (len < 25.0f || len > (float)w * 0.55f)
            continue;
        float angle = std::atan2(dy, dx) * 180.0f / (float)CV_PI;
        if (angle < -90.0f) angle += 180.0f;
        if (angle > 90.0f) angle -= 180.0f;
        if (std::fabs(angle) > 22.0f)
            continue;
        float cy = ((float)y1l + (float)y2l) * 0.5f;
        float cx = ((float)x1l + (float)x2l) * 0.5f;
        if (cy < (float)h * 0.53f || cy > (float)h * 0.95f)
            continue;
        if (len > (float)w * 0.45f && cy < (float)h * 0.62f)
            continue;
        segs.push_back({x1l, y1l, x2l, y2l, len, angle, cx, cy});
    }
    if (segs.size() < 3)
        return false;

    float best_score = 0.0f;
    cv::Rect best_rect;
    for (const auto &s : segs) {
        std::vector<stripe_seg> cluster;
        for (const auto &t : segs) {
            if (std::fabs(t.angle - s.angle) > 10.0f)
                continue;
            if (std::fabs(t.cy - s.cy) > (float)h * 0.30f)
                continue;
            if (std::fabs(t.cx - s.cx) > (float)w * 0.36f)
                continue;
            cluster.push_back(t);
        }
        if (cluster.size() < 3)
            continue;

        int min_x = w - 1, min_y = h - 1, max_x = 0, max_y = 0;
        float len_sum = 0.0f;
        for (const auto &c : cluster) {
            min_x = std::min(min_x, std::min(c.x1, c.x2));
            min_y = std::min(min_y, std::min(c.y1, c.y2));
            max_x = std::max(max_x, std::max(c.x1, c.x2));
            max_y = std::max(max_y, std::max(c.y1, c.y2));
            len_sum += c.len;
        }
        int bw = max_x - min_x;
        int bh = max_y - min_y;
        if (bw < (int)((float)w * 0.10f) || bh < (int)((float)h * 0.03f))
            continue;
        if (bw > (int)((float)w * 0.62f) || bh > (int)((float)h * 0.35f))
            continue;
        float score = (float)cluster.size() * 1000.0f + len_sum +
                      (float)min_x * 0.1f + (float)min_y * 0.2f;
        if (score > best_score) {
            best_score = score;
            best_rect = cv::Rect(cv::Point(min_x, min_y), cv::Point(max_x, max_y));
        }
    }
    if (best_score <= 0.0f)
        return false;

    int pad = 14;
    int x0 = std::max(0, best_rect.x - pad);
    int yy0 = std::max(0, best_rect.y - pad);
    int x1b = std::min(w - 1, best_rect.x + best_rect.width + pad);
    int yy1 = std::min(h - 1, best_rect.y + best_rect.height + pad);

    poly->clear();
    poly->emplace_back((int)std::round((double)x0 / scale), (int)std::round((double)yy0 / scale));
    poly->emplace_back((int)std::round((double)x1b / scale), (int)std::round((double)yy0 / scale));
    poly->emplace_back((int)std::round((double)x1b / scale), (int)std::round((double)yy1 / scale));
    poly->emplace_back((int)std::round((double)x0 / scale), (int)std::round((double)yy1 / scale));
    return true;
}

static bool detect_road(const cv::Mat &bgr, std::vector<cv::Point> *poly)
{
    cv::Mat small, gray, blur, edges;
    double scale = 640.0 / (double)bgr.cols;
    cv::resize(bgr, small, cv::Size(), scale, scale, cv::INTER_AREA);
    cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
    cv::Canny(blur, edges, 70, 180);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180.0, 55, 55, 25);
    std::vector<cv::Vec4i> left, right;
    int h = small.rows;
    int w = small.cols;
    for (const auto &l : lines) {
        float dx = (float)(l[2] - l[0]);
        float dy = (float)(l[3] - l[1]);
        if (std::fabs(dx) < 1.0f)
            continue;
        float slope = dy / dx;
        float len = std::hypot(dx, dy);
        if (len < 55.0f || std::fabs(slope) < 0.35f)
            continue;
        int cy = (l[1] + l[3]) / 2;
        if (cy < h / 3)
            continue;
        if (slope < 0)
            left.push_back(l);
        else
            right.push_back(l);
    }
    if (left.empty() || right.empty())
        return false;

    auto fit_side = [](const std::vector<cv::Vec4i> &ls) {
        std::vector<cv::Point2f> pts;
        for (const auto &l : ls) {
            pts.emplace_back((float)l[0], (float)l[1]);
            pts.emplace_back((float)l[2], (float)l[3]);
        }
        cv::Vec4f line;
        cv::fitLine(pts, line, cv::DIST_L2, 0, 0.01, 0.01);
        return line;
    };
    cv::Vec4f lf = fit_side(left);
    cv::Vec4f rf = fit_side(right);

    auto x_at_y = [](const cv::Vec4f &line, float y) {
        float vx = line[0], vy = line[1], x0 = line[2], y0 = line[3];
        if (std::fabs(vy) < 1e-3f)
            return x0;
        return x0 + (y - y0) * vx / vy;
    };

    float y_top = (float)h * 0.42f;
    float y_bot = (float)h * 0.98f;
    float lx_top = x_at_y(lf, y_top);
    float lx_bot = x_at_y(lf, y_bot);
    float rx_top = x_at_y(rf, y_top);
    float rx_bot = x_at_y(rf, y_bot);
    if (lx_top > rx_top) std::swap(lx_top, rx_top);
    if (lx_bot > rx_bot) std::swap(lx_bot, rx_bot);
    if ((rx_bot - lx_bot) < (float)w * 0.18f)
        return false;

    auto clampx = [w](float x) { return std::max(0.0f, std::min((float)(w - 1), x)); };
    poly->clear();
    poly->emplace_back((int)std::round(clampx(lx_top) / scale), (int)std::round(y_top / scale));
    poly->emplace_back((int)std::round(clampx(rx_top) / scale), (int)std::round(y_top / scale));
    poly->emplace_back((int)std::round(clampx(rx_bot) / scale), (int)std::round(y_bot / scale));
    poly->emplace_back((int)std::round(clampx(lx_bot) / scale), (int)std::round(y_bot / scale));
    if (polygon_area_abs(*poly) > (float)bgr.cols * (float)bgr.rows * 0.55f)
        return false;
    if (std::abs(poly->at(1).x - poly->at(0).x) > (int)((float)bgr.cols * 0.85f))
        return false;
    return true;
}

static void run_scene_cv(app_ctx *ctx, const cv::Mat &bgr)
{
    std::vector<cv::Point> poly;
    if (!frame_has_active_scene(bgr)) {
        ctx->scene = scene_state{};
        return;
    }

    update_scene_hold(&ctx->scene, ctx->opt.scene_smooth);
    if (detect_crosswalk(bgr, &poly)) {
        ctx->scene.crosswalk_poly = poly;
        ctx->scene.crosswalk_valid = true;
        ctx->scene.crosswalk_hold = false;
        ctx->scene.crosswalk_ttl = ctx->opt.scene_smooth;
    }
    if (detect_road(bgr, &poly)) {
        ctx->scene.road_poly = poly;
        ctx->scene.road_valid = true;
        ctx->scene.road_hold = false;
        ctx->scene.road_ttl = ctx->opt.scene_smooth;
    }
}

static region_state classify_region(const scene_state &scene, const cv::Point &foot)
{
    if (scene.crosswalk_valid && point_in_poly(scene.crosswalk_poly, foot))
        return REGION_CROSSWALK;
    if (scene.road_valid && point_in_poly(scene.road_poly, foot))
        return REGION_ROAD;
    if (scene.road_valid)
        return REGION_SIDEWALK;
    return REGION_UNKNOWN;
}

static const char *region_name(region_state r)
{
    switch (r) {
    case REGION_SIDEWALK: return "SIDEWALK";
    case REGION_ROAD: return "ROAD";
    case REGION_CROSSWALK: return "CROSSWALK";
    default: return "UNK";
    }
}

static int update_tracks_and_count(app_ctx *ctx, const std::vector<det_box> &persons,
                                   std::vector<region_state> *regions,
                                   std::vector<bool> *crossing)
{
    int events = 0;
    for (auto &tr : ctx->tracks)
        tr.ttl--;

    regions->assign(persons.size(), REGION_UNKNOWN);
    crossing->assign(persons.size(), false);

    for (size_t i = 0; i < persons.size(); i++) {
        cv::Point foot((persons[i].x1 + persons[i].x2) / 2, persons[i].y2);
        region_state r = classify_region(ctx->scene, foot);
        (*regions)[i] = r;

        int best = -1;
        int best_d2 = 96 * 96;
        for (size_t j = 0; j < ctx->tracks.size(); j++) {
            if (ctx->tracks[j].ttl <= 0)
                continue;
            int dx = ctx->tracks[j].foot.x - foot.x;
            int dy = ctx->tracks[j].foot.y - foot.y;
            int d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best = (int)j;
            }
        }
        if (best < 0) {
            track_state tr;
            tr.box = cv::Rect(cv::Point(persons[i].x1, persons[i].y1),
                              cv::Point(persons[i].x2, persons[i].y2));
            tr.foot = foot;
            tr.last_region = r;
            tr.came_from_sidewalk = (r == REGION_SIDEWALK);
            tr.ttl = 10;
            ctx->tracks.push_back(tr);
        } else {
            track_state &tr = ctx->tracks[(size_t)best];
            bool entered = tr.came_from_sidewalk &&
                (r == REGION_ROAD || r == REGION_CROSSWALK) &&
                tr.last_region == REGION_SIDEWALK;
            tr.box = cv::Rect(cv::Point(persons[i].x1, persons[i].y1),
                              cv::Point(persons[i].x2, persons[i].y2));
            tr.foot = foot;
            tr.last_region = r;
            if (r == REGION_SIDEWALK)
                tr.came_from_sidewalk = true;
            tr.ttl = 10;
            if (entered) {
                (*crossing)[i] = true;
                events++;
            }
        }
    }

    ctx->tracks.erase(std::remove_if(ctx->tracks.begin(), ctx->tracks.end(),
                                     [](const track_state &tr) { return tr.ttl <= 0; }),
                      ctx->tracks.end());
    return events;
}

static void draw_poly(cv::Mat &bgr, const std::vector<cv::Point> &poly, const cv::Scalar &color,
                      const char *label)
{
    if (poly.size() < 3)
        return;
    std::vector<std::vector<cv::Point>> polys{poly};
    cv::polylines(bgr, polys, true, color, 3, cv::LINE_AA);
    cv::Mat overlay = bgr.clone();
    cv::fillPoly(overlay, polys, color);
    cv::addWeighted(overlay, 0.15, bgr, 0.85, 0.0, bgr);
    cv::putText(bgr, label, poly[0], cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv::LINE_AA);
}

static void overlay_results(app_ctx *ctx, cv::Mat &bgr, const std::vector<det_box> &persons,
                            const std::vector<region_state> &regions,
                            const std::vector<bool> &crossing)
{
    draw_poly(bgr, ctx->scene.road_poly, cv::Scalar(255, 180, 0),
              ctx->scene.road_valid ? (ctx->scene.road_hold ? "ROAD HOLD" : "ROAD") : "ROAD LOST");
    draw_poly(bgr, ctx->scene.crosswalk_poly, cv::Scalar(0, 255, 255),
              ctx->scene.crosswalk_valid ? (ctx->scene.crosswalk_hold ? "CROSSWALK HOLD" : "CROSSWALK") : "CROSSWALK LOST");

    for (size_t i = 0; i < persons.size(); i++) {
        const det_box &p = persons[i];
        cv::Scalar color(0, 255, 0);
        const char *state = region_name(regions[i]);
        if (regions[i] == REGION_CROSSWALK) {
            color = cv::Scalar(0, 255, 255);
            state = "CROSSWALK_OK";
        } else if (regions[i] == REGION_ROAD) {
            color = cv::Scalar(0, 0, 255);
            state = "JAYWALK?";
        }
        cv::rectangle(bgr, cv::Rect(cv::Point(p.x1, p.y1), cv::Point(p.x2, p.y2)), color, 2);
        cv::Point foot((p.x1 + p.x2) / 2, p.y2);
        cv::circle(bgr, foot, 5, color, -1, cv::LINE_AA);
        std::string text = std::string(state) + (crossing[i] ? " ENTER" : "");
        cv::putText(bgr, text, cv::Point(p.x1, std::max(20, p.y1 - 8)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv::LINE_AA);
    }

    char status[160];
    snprintf(status, sizeof(status), "frame=%" PRIu64 " persons=%zu crosswalk=%s road=%s crossings=%" PRIu64,
             ctx->frame_seq, persons.size(),
             ctx->scene.crosswalk_valid ? (ctx->scene.crosswalk_hold ? "HOLD" : "OK") : "LOST",
             ctx->scene.road_valid ? (ctx->scene.road_hold ? "HOLD" : "OK") : "LOST",
             ctx->crossing_total);
    cv::putText(bgr, status, cv::Point(16, 32), cv::FONT_HERSHEY_SIMPLEX,
                0.75, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
}

static void write_poly(FILE *fp, const char *name, bool valid, bool hold,
                       const std::vector<cv::Point> &poly)
{
    fprintf(fp, "%s_valid=%d\n", name, valid ? 1 : 0);
    fprintf(fp, "%s_hold=%d\n", name, hold ? 1 : 0);
    fprintf(fp, "%s_points=%zu", name, poly.size());
    for (const auto &p : poly)
        fprintf(fp, " %d,%d", p.x, p.y);
    fprintf(fp, "\n");
}

static int write_ppm_bgr(const char *path, const cv::Mat &bgr)
{
    FILE *fp;
    if (bgr.empty() || bgr.type() != CV_8UC3)
        return -1;
    fp = fopen(path, "wb");
    if (!fp)
        return -1;
    fprintf(fp, "P6\n%d %d\n255\n", bgr.cols, bgr.rows);
    for (int y = 0; y < bgr.rows; y++) {
        const cv::Vec3b *row = bgr.ptr<cv::Vec3b>(y);
        for (int x = 0; x < bgr.cols; x++) {
            uint8_t rgb[3] = { row[x][2], row[x][1], row[x][0] };
            fwrite(rgb, 1, 3, fp);
        }
    }
    fclose(fp);
    return 0;
}

static void maybe_dump_debug(app_ctx *ctx, const cv::Mat &raw_bgr, const cv::Mat &overlay_bgr,
                             const std::vector<det_box> &persons,
                             const std::vector<region_state> &regions,
                             const std::vector<bool> &crossing,
                             int crossing_count)
{
    char raw_path[512], overlay_path[512], meta_path[512];
    FILE *fp;

    if (!ctx->opt.debug_dump_dir || ctx->opt.debug_dump_max <= 0)
        return;
    if (ctx->debug_dumped >= ctx->opt.debug_dump_max)
        return;
    if ((ctx->frame_seq % (uint64_t)ctx->opt.debug_dump_every_n) != 0)
        return;

    mkdir(ctx->opt.debug_dump_dir, 0777);
    snprintf(raw_path, sizeof(raw_path), "%s/raw_%06" PRIu64 ".ppm",
             ctx->opt.debug_dump_dir, ctx->frame_seq);
    snprintf(overlay_path, sizeof(overlay_path), "%s/overlay_%06" PRIu64 ".ppm",
             ctx->opt.debug_dump_dir, ctx->frame_seq);
    snprintf(meta_path, sizeof(meta_path), "%s/meta_%06" PRIu64 ".txt",
             ctx->opt.debug_dump_dir, ctx->frame_seq);

    if (write_ppm_bgr(raw_path, raw_bgr) < 0)
        fprintf(stderr, "[dump] failed raw=%s\n", raw_path);
    if (write_ppm_bgr(overlay_path, overlay_bgr) < 0)
        fprintf(stderr, "[dump] failed overlay=%s\n", overlay_path);

    fp = fopen(meta_path, "w");
    if (fp) {
        fprintf(fp, "frame=%" PRIu64 "\n", ctx->frame_seq);
        fprintf(fp, "image=%dx%d\n", raw_bgr.cols, raw_bgr.rows);
        fprintf(fp, "persons=%zu\n", persons.size());
        fprintf(fp, "crossing_count=%d\n", crossing_count);
        write_poly(fp, "crosswalk", ctx->scene.crosswalk_valid, ctx->scene.crosswalk_hold,
                   ctx->scene.crosswalk_poly);
        write_poly(fp, "road", ctx->scene.road_valid, ctx->scene.road_hold,
                   ctx->scene.road_poly);
        for (size_t i = 0; i < persons.size(); i++) {
            cv::Point foot((persons[i].x1 + persons[i].x2) / 2, persons[i].y2);
            fprintf(fp,
                    "person[%zu]=bbox:%d,%d,%d,%d conf:%.4f foot:%d,%d region:%s crossing:%d\n",
                    i, persons[i].x1, persons[i].y1, persons[i].x2, persons[i].y2,
                    persons[i].conf, foot.x, foot.y,
                    (i < regions.size()) ? region_name(regions[i]) : "NA",
                    (i < crossing.size() && crossing[i]) ? 1 : 0);
        }
        fclose(fp);
    } else {
        fprintf(stderr, "[dump] failed meta=%s\n", meta_path);
    }

    fprintf(stderr, "[dump] frame=%" PRIu64 " raw=%s overlay=%s meta=%s\n",
            ctx->frame_seq, raw_path, overlay_path, meta_path);
    ctx->debug_dumped++;
}

static int push_frame(app_ctx *ctx, const cv::Mat &bgr)
{
    cv::Mat bgrx;
    cv::cvtColor(bgr, bgrx, cv::COLOR_BGR2BGRA);
    GstBuffer *buf = gst_buffer_new_allocate(NULL, bgrx.total() * bgrx.elemSize(), NULL);
    if (!buf)
        return -1;
    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_WRITE)) {
        gst_buffer_unref(buf);
        return -1;
    }
    memcpy(map.data, bgrx.data, map.size);
    gst_buffer_unmap(buf, &map);
    GST_BUFFER_PTS(buf) = gst_util_uint64_scale(ctx->pushed_frames, GST_SECOND, ctx->opt.fps);
    GST_BUFFER_DURATION(buf) = gst_util_uint64_scale(1, GST_SECOND, ctx->opt.fps);
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(ctx->appsrc), buf);
    if (ret != GST_FLOW_OK) {
        fprintf(stderr, "gst_app_src_push_buffer failed: %d\n", ret);
        return -1;
    }
    ctx->pushed_frames++;
    return 0;
}

static void print_stats(app_ctx *ctx, int person_count, int crossing_count)
{
    int64_t now = mono_us();
    if (ctx->last_stats_us == 0)
        ctx->last_stats_us = now;
    if (now - ctx->last_stats_us < (int64_t)ctx->opt.stats_interval * 1000000LL)
        return;
    fprintf(stderr,
            "[stats] frame=%" PRIu64 " pushed=%" PRIu64 " persons=%d crosswalk_valid=%d road_valid=%d crossing_count=%d total=%" PRIu64 "\n",
            ctx->frame_seq, ctx->pushed_frames, person_count,
            ctx->scene.crosswalk_valid ? 1 : 0,
            ctx->scene.road_valid ? 1 : 0,
            crossing_count,
            ctx->crossing_total);
    ctx->last_stats_us = now;
}

static void cleanup(app_ctx *ctx)
{
    if (ctx->pipeline) {
        gst_element_set_state(ctx->pipeline, GST_STATE_NULL);
        gst_object_unref(ctx->pipeline);
    }
    if (ctx->bus)
        gst_object_unref(ctx->bus);
    rknn_model_release(&ctx->ped_model);
    if (ctx->dev_fd >= 0)
        close(ctx->dev_fd);
    if (ctx->drm_fd >= 0)
        close(ctx->drm_fd);
}

int main(int argc, char **argv)
{
    app_ctx ctx;
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (parse_options(argc, argv, &ctx.opt) < 0) {
        print_usage(argv[0]);
        return 1;
    }

    gst_init(&argc, &argv);

    if (load_labels(&ctx) < 0) {
        fprintf(stderr, "Failed to load labels: %s\n", ctx.opt.labels_path);
        return 2;
    }
    ctx.ped_model.class_count = (int)ctx.labels.size();
    if (rknn_model_load(&ctx.ped_model, ctx.opt.ped_model_path) < 0) {
        fprintf(stderr, "Failed to load ped model: %s\n", ctx.opt.ped_model_path);
        cleanup(&ctx);
        return 2;
    }
    if (init_fpga_dma(&ctx) < 0) {
        cleanup(&ctx);
        return 3;
    }
    if (build_pipeline(&ctx) < 0) {
        fprintf(stderr, "Failed to start KMS pipeline\n");
        cleanup(&ctx);
        return 4;
    }

    fprintf(stderr,
            "Start crosswalk CV test: fps=%d min_person=%.2f cv_every_n=%d scene_smooth=%d pixel=%s swap16=%d\n",
            ctx.opt.fps, ctx.opt.min_person_conf, ctx.opt.cv_every_n, ctx.opt.scene_smooth,
            ctx.opt.pixel_order == PIXEL_ORDER_BGR565 ? "bgr565" : "rgb565",
            ctx.opt.swap16 ? 1 : 0);

    while (!g_stop) {
        if (!handle_bus(&ctx))
            break;
        if (trigger_frame_dma(&ctx) < 0)
            break;

        cv::Mat bgr = frame_to_bgr(&ctx);
        if (ctx.frame_seq % (uint64_t)ctx.opt.cv_every_n == 0)
            run_scene_cv(&ctx, bgr);

        std::vector<det_box> persons;
        if (run_ped_detect(&ctx, bgr, &persons) < 0)
            persons.clear();

        std::vector<region_state> regions;
        std::vector<bool> crossing;
        int crossing_count = update_tracks_and_count(&ctx, persons, &regions, &crossing);
        ctx.crossing_total += (uint64_t)crossing_count;

        cv::Mat raw_for_dump;
        if (ctx.opt.debug_dump_dir)
            raw_for_dump = bgr.clone();
        overlay_results(&ctx, bgr, persons, regions, crossing);
        maybe_dump_debug(&ctx, ctx.opt.debug_dump_dir ? raw_for_dump : bgr, bgr,
                         persons, regions, crossing, crossing_count);
        if (push_frame(&ctx, bgr) < 0)
            break;
        print_stats(&ctx, (int)persons.size(), crossing_count);
        ctx.frame_seq++;
    }

    fprintf(stderr, "Stop crosswalk CV test: frame=%" PRIu64 " pushed=%" PRIu64 " crossings=%" PRIu64 "\n",
            ctx.frame_seq, ctx.pushed_frames, ctx.crossing_total);
    cleanup(&ctx);
    return 0;
}
