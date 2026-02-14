// SPDX-License-Identifier: GPL-2.0
/*
 * FPGA HDMI KMS Display Application
 *
 * Capture frames from /dev/fpga_dma0 and render to HDMI via
 * GStreamer appsrc -> kmssink.
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <linux/input.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>
#include <signal.h>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include "pcie_fpga_dma.h"

#define DEFAULT_DEVICE "/dev/" FPGA_DMA_DEV_NAME
#define DEFAULT_DRM_CARD "/dev/dri/card0"
#define DEFAULT_FPS 10
#define DEFAULT_TIMEOUT_MS 5000
#define DEFAULT_STATS_INTERVAL 1

enum pixel_order {
    PIXEL_ORDER_BGR565 = 0,
    PIXEL_ORDER_RGB565,
};

struct options {
    const char *device_path;
    const char *drm_card_path;
    const char *input_path;
    int connector_id;
    int fps;
    enum pixel_order pixel_order;
    int timeout_ms;
    int stats_interval;
};

struct app_ctx {
    struct options opt;

    int dev_fd;
    int drm_fd;
    int input_fd;
    int epoll_fd;

    void *dma_map;
    size_t dma_map_size;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_bpp;
    size_t frame_size;

    GstElement *pipeline;
    GstElement *appsrc;
    GstElement *sink;
    GstBus *bus;

    GMutex frame_lock;
    GCond frame_cond;
    bool frame_in_flight;
    uint64_t frame_id;

    bool running;
    uint64_t captured_frames;
    uint64_t pushed_frames;
    uint64_t released_frames;
    uint64_t next_pts_ns;

    double total_loop_ms;
    uint64_t loop_samples;

    int64_t start_us;
    int64_t last_stats_us;
    uint64_t last_stats_captured;
    uint64_t last_stats_released;
};

struct frame_cookie {
    struct app_ctx *ctx;
    uint64_t id;
};

static volatile sig_atomic_t g_stop = 0;

static int64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
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
            "Options:\n"
            "  --device <path>         FPGA device (default: %s)\n"
            "  --drm-card <path>       DRM card (default: %s)\n"
            "  --connector-id <id>     Optional KMS connector id\n"
            "  --input <event>         Optional input event device\n"
            "  --fps <num>             Target FPS (default: %d)\n"
            "  --pixel-order <mode>    bgr565|rgb565 (default: bgr565)\n"
            "  --timeout-ms <ms>       Frame timeout (default: %d)\n"
            "  --stats-interval <sec>  Stats print interval (default: %d)\n"
            "  --help                  Show this message\n",
            prog,
            DEFAULT_DEVICE,
            DEFAULT_DRM_CARD,
            DEFAULT_FPS,
            DEFAULT_TIMEOUT_MS,
            DEFAULT_STATS_INTERVAL);
}

static int parse_options(int argc, char **argv, struct options *opt)
{
    static const struct option long_opts[] = {
        {"device", required_argument, NULL, 1},
        {"drm-card", required_argument, NULL, 2},
        {"connector-id", required_argument, NULL, 3},
        {"input", required_argument, NULL, 4},
        {"fps", required_argument, NULL, 5},
        {"pixel-order", required_argument, NULL, 6},
        {"timeout-ms", required_argument, NULL, 7},
        {"stats-interval", required_argument, NULL, 8},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };

    int c;

    opt->device_path = DEFAULT_DEVICE;
    opt->drm_card_path = DEFAULT_DRM_CARD;
    opt->input_path = NULL;
    opt->connector_id = -1;
    opt->fps = DEFAULT_FPS;
    opt->pixel_order = PIXEL_ORDER_BGR565;
    opt->timeout_ms = DEFAULT_TIMEOUT_MS;
    opt->stats_interval = DEFAULT_STATS_INTERVAL;

    while ((c = getopt_long(argc, argv, "h", long_opts, NULL)) != -1) {
        switch (c) {
        case 1:
            opt->device_path = optarg;
            break;
        case 2:
            opt->drm_card_path = optarg;
            break;
        case 3:
            opt->connector_id = atoi(optarg);
            break;
        case 4:
            opt->input_path = optarg;
            break;
        case 5:
            opt->fps = atoi(optarg);
            if (opt->fps <= 0) {
                fprintf(stderr, "Invalid --fps: %s\n", optarg);
                return -1;
            }
            break;
        case 6:
            if (strcmp(optarg, "bgr565") == 0) {
                opt->pixel_order = PIXEL_ORDER_BGR565;
            } else if (strcmp(optarg, "rgb565") == 0) {
                opt->pixel_order = PIXEL_ORDER_RGB565;
            } else {
                fprintf(stderr, "Invalid --pixel-order: %s\n", optarg);
                return -1;
            }
            break;
        case 7:
            opt->timeout_ms = atoi(optarg);
            if (opt->timeout_ms <= 0) {
                fprintf(stderr, "Invalid --timeout-ms: %s\n", optarg);
                return -1;
            }
            break;
        case 8:
            opt->stats_interval = atoi(optarg);
            if (opt->stats_interval <= 0) {
                fprintf(stderr, "Invalid --stats-interval: %s\n", optarg);
                return -1;
            }
            break;
        case 'h':
            print_usage(argv[0]);
            exit(0);
        default:
            print_usage(argv[0]);
            return -1;
        }
    }

    return 0;
}

static int test_bit_ul(const unsigned long *bits, int bit)
{
    int idx = bit / (int)(sizeof(unsigned long) * 8U);
    int shift = bit % (int)(sizeof(unsigned long) * 8U);
    return (int)((bits[idx] >> shift) & 1UL);
}

static int input_device_has_exit_keys(int fd)
{
    unsigned long ev_bits[(EV_MAX / (int)(sizeof(unsigned long) * 8U)) + 2];
    unsigned long key_bits[(KEY_MAX / (int)(sizeof(unsigned long) * 8U)) + 2];

    memset(ev_bits, 0, sizeof(ev_bits));
    memset(key_bits, 0, sizeof(key_bits));

    if (ioctl(fd, EVIOCGBIT(0, sizeof(ev_bits)), ev_bits) < 0)
        return 0;
    if (!test_bit_ul(ev_bits, EV_KEY))
        return 0;
    if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits) < 0)
        return 0;

    return test_bit_ul(key_bits, KEY_ESC) || test_bit_ul(key_bits, KEY_Q);
}

static int open_input_auto(void)
{
    DIR *dir;
    struct dirent *ent;

    dir = opendir("/dev/input");
    if (!dir)
        return -1;

    while ((ent = readdir(dir)) != NULL) {
        char path[256];
        int fd;

        if (strncmp(ent->d_name, "event", 5) != 0)
            continue;

        snprintf(path, sizeof(path), "/dev/input/%s", ent->d_name);
        fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0)
            continue;

        if (input_device_has_exit_keys(fd)) {
            fprintf(stderr, "Using input device: %s\n", path);
            closedir(dir);
            return fd;
        }

        close(fd);
    }

    closedir(dir);
    return -1;
}

static int setup_input(struct app_ctx *ctx)
{
    struct epoll_event ev;

    ctx->input_fd = -1;
    ctx->epoll_fd = -1;

    if (ctx->opt.input_path) {
        ctx->input_fd = open(ctx->opt.input_path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (ctx->input_fd < 0) {
            fprintf(stderr, "Failed to open input %s: %s\n",
                    ctx->opt.input_path, strerror(errno));
            return -1;
        }
    } else {
        ctx->input_fd = open_input_auto();
        if (ctx->input_fd < 0)
            fprintf(stderr, "Warning: no input device found, ESC/Q control disabled\n");
    }

    if (ctx->input_fd < 0)
        return 0;

    ctx->epoll_fd = epoll_create1(EPOLL_CLOEXEC);
    if (ctx->epoll_fd < 0) {
        fprintf(stderr, "epoll_create1 failed: %s\n", strerror(errno));
        return -1;
    }

    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLIN;
    ev.data.fd = ctx->input_fd;
    if (epoll_ctl(ctx->epoll_fd, EPOLL_CTL_ADD, ctx->input_fd, &ev) < 0) {
        fprintf(stderr, "epoll_ctl add input failed: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}

static void drain_input_events(struct app_ctx *ctx)
{
    struct input_event ev[32];
    ssize_t n;

    if (ctx->input_fd < 0)
        return;

    for (;;) {
        n = read(ctx->input_fd, ev, sizeof(ev));
        if (n <= 0)
            break;

        size_t count = (size_t)n / sizeof(ev[0]);
        size_t i;
        for (i = 0; i < count; i++) {
            if (ev[i].type == EV_KEY && ev[i].value == 1 &&
                (ev[i].code == KEY_ESC || ev[i].code == KEY_Q)) {
                fprintf(stderr, "Exit key pressed, stopping...\n");
                ctx->running = false;
            }
        }
    }

    if (n < 0 && errno != EAGAIN)
        fprintf(stderr, "Input read error: %s\n", strerror(errno));
}

static int init_fpga_dma(struct app_ctx *ctx)
{
    struct fpga_info info;
    struct buffer_map map;

    ctx->dev_fd = open(ctx->opt.device_path, O_RDWR | O_CLOEXEC);
    if (ctx->dev_fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", ctx->opt.device_path, strerror(errno));
        return -1;
    }

    if (ioctl(ctx->dev_fd, FPGA_DMA_GET_INFO, &info) < 0) {
        fprintf(stderr, "FPGA_DMA_GET_INFO failed: %s\n", strerror(errno));
        return -1;
    }

    if (info.frame_width != 1280 || info.frame_height != 720 || info.frame_bpp != 2) {
        fprintf(stderr,
                "Unsupported frame format: %ux%u bpp=%u (expected 1280x720 bpp=2)\n",
                info.frame_width, info.frame_height, info.frame_bpp);
        return -1;
    }

    ctx->frame_width = info.frame_width;
    ctx->frame_height = info.frame_height;
    ctx->frame_bpp = info.frame_bpp;
    ctx->frame_size = (size_t)ctx->frame_width * ctx->frame_height * ctx->frame_bpp;

    memset(&map, 0, sizeof(map));
    map.index = 0;
    if (ioctl(ctx->dev_fd, FPGA_DMA_MAP_BUFFER, &map) < 0) {
        fprintf(stderr, "FPGA_DMA_MAP_BUFFER failed: %s\n", strerror(errno));
        return -1;
    }

    if (map.size < ctx->frame_size) {
        fprintf(stderr, "Mapped DMA buffer too small: %u < %zu\n", map.size, ctx->frame_size);
        return -1;
    }

    ctx->dma_map_size = map.size;
    ctx->dma_map = mmap(NULL, ctx->dma_map_size, PROT_READ, MAP_SHARED, ctx->dev_fd, 0);
    if (ctx->dma_map == MAP_FAILED) {
        ctx->dma_map = NULL;
        fprintf(stderr, "mmap DMA buffer failed: %s\n", strerror(errno));
        return -1;
    }

    fprintf(stderr, "FPGA DMA ready: %ux%u bpp=%u frame=%zu bytes\n",
            ctx->frame_width, ctx->frame_height, ctx->frame_bpp, ctx->frame_size);
    return 0;
}

static int trigger_frame_dma(struct app_ctx *ctx)
{
    struct dma_transfer transfer;

    memset(&transfer, 0, sizeof(transfer));
    transfer.size = (uint32_t)ctx->frame_size;
    transfer.user_buf = 0;

    if (ioctl(ctx->dev_fd, FPGA_DMA_READ_FRAME, &transfer) < 0) {
        fprintf(stderr, "FPGA_DMA_READ_FRAME failed: %s\n", strerror(errno));
        return -1;
    }

    if (transfer.result != 0) {
        fprintf(stderr, "FPGA_DMA_READ_FRAME result error: %u\n", transfer.result);
        return -1;
    }

    return 0;
}

static void frame_release_notify(gpointer user_data)
{
    struct frame_cookie *cookie = (struct frame_cookie *)user_data;
    struct app_ctx *ctx = cookie->ctx;

    g_mutex_lock(&ctx->frame_lock);
    if (ctx->frame_in_flight && ctx->frame_id == cookie->id) {
        ctx->frame_in_flight = false;
        ctx->released_frames++;
        g_cond_signal(&ctx->frame_cond);
    }
    g_mutex_unlock(&ctx->frame_lock);

    g_free(cookie);
}

static GstBuffer *build_frame_buffer(struct app_ctx *ctx)
{
    struct frame_cookie *cookie;
    GstBuffer *buf;

    cookie = g_new0(struct frame_cookie, 1);
    if (!cookie)
        return NULL;

    cookie->ctx = ctx;

    g_mutex_lock(&ctx->frame_lock);
    ctx->frame_id++;
    cookie->id = ctx->frame_id;
    ctx->frame_in_flight = true;
    g_mutex_unlock(&ctx->frame_lock);

    buf = gst_buffer_new_wrapped_full((GstMemoryFlags)0,
                                      ctx->dma_map,
                                      ctx->frame_size,
                                      0,
                                      ctx->frame_size,
                                      cookie,
                                      frame_release_notify);
    if (!buf) {
        g_mutex_lock(&ctx->frame_lock);
        ctx->frame_in_flight = false;
        g_cond_signal(&ctx->frame_cond);
        g_mutex_unlock(&ctx->frame_lock);
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
        switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
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
        case GST_MESSAGE_EOS:
            fprintf(stderr, "GStreamer EOS\n");
            ctx->running = false;
            gst_message_unref(msg);
            return -1;
        case GST_MESSAGE_WARNING: {
            GError *err = NULL;
            gchar *dbg = NULL;
            gst_message_parse_warning(msg, &err, &dbg);
            fprintf(stderr, "GStreamer WARNING: %s\n", err ? err->message : "unknown");
            if (dbg)
                fprintf(stderr, "  debug: %s\n", dbg);
            if (err)
                g_error_free(err);
            g_free(dbg);
            break;
        }
        default:
            break;
        }
        gst_message_unref(msg);
    }

    return 0;
}

static int process_events(struct app_ctx *ctx, int wait_ms)
{
    struct epoll_event ev[4];
    int n;

    if (g_stop)
        ctx->running = false;

    if (handle_bus_messages(ctx) < 0)
        return -1;

    if (ctx->epoll_fd < 0)
        return 0;

    n = epoll_wait(ctx->epoll_fd, ev, 4, wait_ms);
    if (n < 0) {
        if (errno == EINTR)
            return 0;
        fprintf(stderr, "epoll_wait failed: %s\n", strerror(errno));
        return -1;
    }

    if (n > 0)
        drain_input_events(ctx);

    return 0;
}

static int wait_previous_frame_done(struct app_ctx *ctx)
{
    int64_t deadline_us = now_us() + (int64_t)ctx->opt.timeout_ms * 1000LL;

    for (;;) {
        bool in_flight;
        int64_t slice_deadline;

        if (process_events(ctx, 10) < 0)
            return -1;
        if (!ctx->running)
            return -1;

        g_mutex_lock(&ctx->frame_lock);
        in_flight = ctx->frame_in_flight;
        g_mutex_unlock(&ctx->frame_lock);
        if (!in_flight)
            return 0;

        if (now_us() >= deadline_us) {
            fprintf(stderr, "Timeout waiting previous frame release (%d ms)\n", ctx->opt.timeout_ms);
            return -1;
        }

        slice_deadline = now_us() + 20000; /* 20ms */
        if (slice_deadline > deadline_us)
            slice_deadline = deadline_us;

        g_mutex_lock(&ctx->frame_lock);
        if (ctx->frame_in_flight)
            g_cond_wait_until(&ctx->frame_cond, &ctx->frame_lock, slice_deadline);
        g_mutex_unlock(&ctx->frame_lock);
    }
}

static void print_stats(struct app_ctx *ctx)
{
    int64_t now = now_us();
    int64_t dt = now - ctx->last_stats_us;
    double avg_ms;

    if (dt < (int64_t)ctx->opt.stats_interval * 1000000LL)
        return;

    avg_ms = ctx->loop_samples ? (ctx->total_loop_ms / (double)ctx->loop_samples) : 0.0;

    fprintf(stderr,
            "[stats] cap=%" PRIu64 " push=%" PRIu64 " rel=%" PRIu64
            "  interval_fps=%.2f rel_fps=%.2f avg_loop=%.2fms\n",
            ctx->captured_frames,
            ctx->pushed_frames,
            ctx->released_frames,
            (double)(ctx->captured_frames - ctx->last_stats_captured) * 1000000.0 / (double)dt,
            (double)(ctx->released_frames - ctx->last_stats_released) * 1000000.0 / (double)dt,
            avg_ms);

    ctx->last_stats_captured = ctx->captured_frames;
    ctx->last_stats_released = ctx->released_frames;
    ctx->last_stats_us = now;
}

static int build_pipeline(struct app_ctx *ctx)
{
    GstCaps *caps;
    GstStateChangeReturn sret;
    const char *fmt = (ctx->opt.pixel_order == PIXEL_ORDER_BGR565) ? "BGR16" : "RGB16";

    ctx->pipeline = gst_pipeline_new("fpga-hdmi");
    ctx->appsrc = gst_element_factory_make("appsrc", "src");
    ctx->sink = gst_element_factory_make("kmssink", "sink");
    if (!ctx->pipeline || !ctx->appsrc || !ctx->sink) {
        fprintf(stderr, "Failed to create GStreamer elements (appsrc/kmssink)\n");
        return -1;
    }

    gst_bin_add_many(GST_BIN(ctx->pipeline), ctx->appsrc, ctx->sink, NULL);
    if (!gst_element_link(ctx->appsrc, ctx->sink)) {
        fprintf(stderr, "Failed to link appsrc -> kmssink\n");
        return -1;
    }

    caps = gst_caps_new_simple("video/x-raw",
                               "format", G_TYPE_STRING, fmt,
                               "width", G_TYPE_INT, (int)ctx->frame_width,
                               "height", G_TYPE_INT, (int)ctx->frame_height,
                               "framerate", GST_TYPE_FRACTION, ctx->opt.fps, 1,
                               NULL);
    if (!caps) {
        fprintf(stderr, "Failed to create caps\n");
        return -1;
    }

    g_object_set(ctx->appsrc,
                 "caps", caps,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "format", GST_FORMAT_TIME,
                 "block", TRUE,
                 NULL);
    gst_caps_unref(caps);

    g_object_set(ctx->sink, "sync", FALSE, NULL);
    if (ctx->opt.connector_id >= 0)
        g_object_set(ctx->sink, "connector-id", ctx->opt.connector_id, NULL);

    if (ctx->drm_fd >= 0)
        g_object_set(ctx->sink, "fd", ctx->drm_fd, NULL);

    ctx->bus = gst_element_get_bus(ctx->pipeline);

    sret = gst_element_set_state(ctx->pipeline, GST_STATE_PLAYING);
    if (sret == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "Failed to set pipeline PLAYING\n");
        return -1;
    }

    sret = gst_element_get_state(ctx->pipeline, NULL, NULL, 5 * GST_SECOND);
    if (sret == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "Pipeline failed during state transition (caps/sink negotiation)\n");
        return -1;
    }

    fprintf(stderr, "Pipeline started: appsrc(format=%s) -> kmssink\n", fmt);
    return 0;
}

static void cleanup(struct app_ctx *ctx)
{
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

    if (ctx->dev_fd >= 0)
        close(ctx->dev_fd);
    if (ctx->drm_fd >= 0)
        close(ctx->drm_fd);
    if (ctx->input_fd >= 0)
        close(ctx->input_fd);
    if (ctx->epoll_fd >= 0)
        close(ctx->epoll_fd);

    g_cond_clear(&ctx->frame_cond);
    g_mutex_clear(&ctx->frame_lock);
}

int main(int argc, char **argv)
{
    struct app_ctx ctx;
    int ret = 1;

    memset(&ctx, 0, sizeof(ctx));
    ctx.dev_fd = -1;
    ctx.drm_fd = -1;
    ctx.input_fd = -1;
    ctx.epoll_fd = -1;
    ctx.running = true;

    g_mutex_init(&ctx.frame_lock);
    g_cond_init(&ctx.frame_cond);

    if (parse_options(argc, argv, &ctx.opt) < 0)
        goto out;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    gst_init(&argc, &argv);

    ctx.drm_fd = open(ctx.opt.drm_card_path, O_RDWR | O_CLOEXEC);
    if (ctx.drm_fd < 0) {
        fprintf(stderr, "Failed to open DRM card %s: %s\n",
                ctx.opt.drm_card_path, strerror(errno));
        goto out;
    }

    if (setup_input(&ctx) < 0)
        goto out;

    if (init_fpga_dma(&ctx) < 0)
        goto out;

    if (build_pipeline(&ctx) < 0)
        goto out;

    fprintf(stderr,
            "Start display loop: fps=%d pixel-order=%s timeout=%dms\n",
            ctx.opt.fps,
            (ctx.opt.pixel_order == PIXEL_ORDER_BGR565) ? "bgr565" : "rgb565",
            ctx.opt.timeout_ms);

    ctx.start_us = now_us();
    ctx.last_stats_us = ctx.start_us;

    while (ctx.running) {
        GstBuffer *buf;
        GstFlowReturn flow;
        int64_t t0;
        int64_t t1;
        int64_t target_us = 1000000LL / ctx.opt.fps;

        if (wait_previous_frame_done(&ctx) < 0)
            break;

        if (process_events(&ctx, 0) < 0)
            break;
        if (!ctx.running)
            break;

        t0 = now_us();

        if (trigger_frame_dma(&ctx) < 0) {
            fprintf(stderr, "DMA trigger failed\n");
            break;
        }
        ctx.captured_frames++;

        buf = build_frame_buffer(&ctx);
        if (!buf) {
            fprintf(stderr, "Failed to wrap DMA frame into GstBuffer\n");
            break;
        }

        flow = gst_app_src_push_buffer(GST_APP_SRC(ctx.appsrc), buf);
        if (flow != GST_FLOW_OK) {
            fprintf(stderr, "gst_app_src_push_buffer failed: %d\n", flow);
            g_mutex_lock(&ctx.frame_lock);
            ctx.frame_in_flight = false;
            g_cond_signal(&ctx.frame_cond);
            g_mutex_unlock(&ctx.frame_lock);
            break;
        }
        ctx.pushed_frames++;

        t1 = now_us();
        ctx.total_loop_ms += (double)(t1 - t0) / 1000.0;
        ctx.loop_samples++;

        print_stats(&ctx);

        if ((t1 - t0) < target_us)
            usleep((useconds_t)(target_us - (t1 - t0)));
    }

    fprintf(stderr,
            "Exit: captured=%" PRIu64 " pushed=%" PRIu64 " released=%" PRIu64 "\n",
            ctx.captured_frames,
            ctx.pushed_frames,
            ctx.released_frames);

    ret = 0;

out:
    cleanup(&ctx);
    return ret;
}
