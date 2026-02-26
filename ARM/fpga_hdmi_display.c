// SPDX-License-Identifier: GPL-2.0
/*
 * FPGA HDMI KMS Display Application
 *
 * Capture frames from /dev/fpga_dma0 and render to HDMI via
 * GStreamer appsrc -> queue -> kmssink.
 */

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <linux/input.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include "pcie_fpga_dma.h"

#define DEFAULT_DEVICE "/dev/" FPGA_DMA_DEV_NAME
#define DEFAULT_DRM_CARD "/dev/dri/card0"
#define DEFAULT_FPS 10
#define DEFAULT_TIMEOUT_MS 5000
#define DEFAULT_STATS_INTERVAL 1
#define DEFAULT_COPY_BUFFERS 3
#define DEFAULT_QUEUE_DEPTH 2
#define MIN_COPY_BUFFERS 2
#define MAX_COPY_BUFFERS 6

enum pixel_order {
    PIXEL_ORDER_BGR565 = 0,
    PIXEL_ORDER_RGB565,
};

enum io_mode {
    IO_MODE_MMAP = 0,
    IO_MODE_COPY,
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
    int copy_buffers;
    int queue_depth;
    enum io_mode io_mode;
    bool swap16;
};

struct frame_slot {
    uint8_t *data;
    bool owns_data;
    bool in_use;
    uint64_t generation;
};

struct app_ctx {
    struct options opt;

    int dev_fd;
    int drm_fd;
    int input_fd;
    int epoll_fd;

    void *dma_map;
    size_t dma_map_size;
    uint8_t *dma_copy;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_bpp;
    uint32_t frame_stride;
    uint32_t pixel_format;
    size_t frame_size;
    size_t display_frame_size;
    bool source_is_bgrx;
    bool zero_copy_mode;

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

    double total_loop_ms;
    uint64_t loop_samples;

    uint64_t slot_wait_timeout_count;
    uint64_t slot_wait_total_us;
    uint64_t slot_wait_samples;

    int64_t start_us;
    int64_t last_stats_us;
    uint64_t last_stats_captured;
    uint64_t last_stats_released;
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
            "Options:\n"
            "  --device <path>         FPGA device (default: %s)\n"
            "  --drm-card <path>       DRM card (default: %s)\n"
            "  --connector-id <id>     Optional KMS connector id\n"
            "  --input <event>         Optional input event device\n"
            "  --fps <num>             Target FPS (default: %d)\n"
            "  --pixel-order <mode>    bgr565|rgb565 (default: bgr565)\n"
            "  --timeout-ms <ms>       Frame timeout (default: %d)\n"
            "  --stats-interval <sec>  Stats print interval (default: %d)\n"
            "  --copy-buffers <num>    Copy ring size (default: %d, range: %d..%d)\n"
            "  --queue-depth <num>     appsrc max frame queue (default: %d)\n"
            "  --io-mode <mode>        mmap|copy (default: mmap)\n"
            "  --swap16 <0|1>          Swap bytes in each 16-bit pixel (default: 1)\n"
            "  --help                  Show this message\n",
            prog,
            DEFAULT_DEVICE,
            DEFAULT_DRM_CARD,
            DEFAULT_FPS,
            DEFAULT_TIMEOUT_MS,
            DEFAULT_STATS_INTERVAL,
            DEFAULT_COPY_BUFFERS,
            MIN_COPY_BUFFERS,
            MAX_COPY_BUFFERS,
            DEFAULT_QUEUE_DEPTH);
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
        {"copy-buffers", required_argument, NULL, 9},
        {"queue-depth", required_argument, NULL, 10},
        {"io-mode", required_argument, NULL, 11},
        {"swap16", required_argument, NULL, 12},
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
    opt->copy_buffers = DEFAULT_COPY_BUFFERS;
    opt->queue_depth = DEFAULT_QUEUE_DEPTH;
    opt->io_mode = IO_MODE_MMAP;
    opt->swap16 = true;

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
        case 9:
            opt->copy_buffers = atoi(optarg);
            if (opt->copy_buffers < MIN_COPY_BUFFERS || opt->copy_buffers > MAX_COPY_BUFFERS) {
                fprintf(stderr, "Invalid --copy-buffers: %s (range %d..%d)\n",
                        optarg, MIN_COPY_BUFFERS, MAX_COPY_BUFFERS);
                return -1;
            }
            break;
        case 10:
            opt->queue_depth = atoi(optarg);
            if (opt->queue_depth <= 0) {
                fprintf(stderr, "Invalid --queue-depth: %s\n", optarg);
                return -1;
            }
            break;
        case 11:
            if (strcmp(optarg, "mmap") == 0) {
                opt->io_mode = IO_MODE_MMAP;
            } else if (strcmp(optarg, "copy") == 0) {
                opt->io_mode = IO_MODE_COPY;
            } else {
                fprintf(stderr, "Invalid --io-mode: %s\n", optarg);
                return -1;
            }
            break;
        case 12:
            if (strcmp(optarg, "1") == 0 || strcasecmp(optarg, "on") == 0 ||
                strcasecmp(optarg, "true") == 0) {
                opt->swap16 = true;
            } else if (strcmp(optarg, "0") == 0 || strcasecmp(optarg, "off") == 0 ||
                       strcasecmp(optarg, "false") == 0) {
                opt->swap16 = false;
            } else {
                fprintf(stderr, "Invalid --swap16: %s (use 0|1)\n", optarg);
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

        if (snprintf(path, sizeof(path), "/dev/input/%.244s", ent->d_name) >= (int)sizeof(path))
            continue;

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
        size_t count;
        size_t i;

        n = read(ctx->input_fd, ev, sizeof(ev));
        if (n <= 0)
            break;

        count = (size_t)n / sizeof(ev[0]);
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

static const char *pixel_format_name(uint32_t pixel_format)
{
    switch (pixel_format) {
    case FPGA_PIXEL_FORMAT_BGRX8888:
        return "BGRX8888";
    case FPGA_PIXEL_FORMAT_BGR565:
        return "BGR565";
    default:
        return "UNKNOWN";
    }
}

static int init_fpga_dma(struct app_ctx *ctx)
{
    struct fpga_info info;
    uint32_t min_stride;

    ctx->dev_fd = open(ctx->opt.device_path, O_RDWR | O_CLOEXEC);
    if (ctx->dev_fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", ctx->opt.device_path, strerror(errno));
        return -1;
    }

    if (ioctl(ctx->dev_fd, FPGA_DMA_GET_INFO, &info) < 0) {
        fprintf(stderr, "FPGA_DMA_GET_INFO failed: %s\n", strerror(errno));
        return -1;
    }

    if (info.frame_width != 1280 || info.frame_height != 720) {
        fprintf(stderr,
                "Unsupported frame geometry: %ux%u (expected 1280x720)\n",
                info.frame_width, info.frame_height);
        return -1;
    }

    if (info.pixel_format == FPGA_PIXEL_FORMAT_BGRX8888)
        info.frame_bpp = 4;
    else if (info.pixel_format == FPGA_PIXEL_FORMAT_BGR565)
        info.frame_bpp = 2;
    else if (info.frame_bpp == 4)
        info.pixel_format = FPGA_PIXEL_FORMAT_BGRX8888;
    else if (info.frame_bpp == 2)
        info.pixel_format = FPGA_PIXEL_FORMAT_BGR565;

    if (info.frame_bpp != 2 && info.frame_bpp != 4) {
        fprintf(stderr, "Unsupported frame_bpp=%u (expected 2 or 4)\n", info.frame_bpp);
        return -1;
    }

    min_stride = info.frame_width * info.frame_bpp;
    if (info.frame_stride < min_stride)
        info.frame_stride = min_stride;

    ctx->frame_width = info.frame_width;
    ctx->frame_height = info.frame_height;
    ctx->frame_bpp = info.frame_bpp;
    ctx->frame_stride = info.frame_stride;
    ctx->pixel_format = info.pixel_format;
    ctx->frame_size = (size_t)ctx->frame_stride * ctx->frame_height;
    ctx->source_is_bgrx = (ctx->pixel_format == FPGA_PIXEL_FORMAT_BGRX8888) || (ctx->frame_bpp == 4);
    ctx->display_frame_size = ctx->source_is_bgrx
        ? ctx->frame_size
        : ((size_t)ctx->frame_width * ctx->frame_height * 4U);
    ctx->zero_copy_mode = ctx->source_is_bgrx && (ctx->opt.io_mode == IO_MODE_MMAP);

    if (ctx->opt.io_mode == IO_MODE_MMAP) {
        struct buffer_map map;

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
    } else {
        ctx->dma_copy = malloc(ctx->frame_size);
        if (!ctx->dma_copy) {
            fprintf(stderr, "Failed to allocate copy IO frame buffer (%zu bytes)\n", ctx->frame_size);
            return -1;
        }
    }

    if (ctx->source_is_bgrx && (ctx->opt.swap16 || ctx->opt.pixel_order != PIXEL_ORDER_BGR565))
        fprintf(stderr, "Note: --pixel-order/--swap16 are ignored for BGRX source frames\n");

    fprintf(stderr,
            "FPGA DMA ready: %ux%u fmt=%s bpp=%u stride=%u frame=%zu bytes (io-mode=%s zero-copy=%s)\n",
            ctx->frame_width,
            ctx->frame_height,
            pixel_format_name(ctx->pixel_format),
            ctx->frame_bpp,
            ctx->frame_stride,
            ctx->frame_size,
            (ctx->opt.io_mode == IO_MODE_MMAP) ? "mmap" : "copy",
            ctx->zero_copy_mode ? "on" : "off");
    return 0;
}

static int init_copy_slots(struct app_ctx *ctx)
{
    int i;

    if (ctx->zero_copy_mode)
        ctx->slot_count = 1;
    else
        ctx->slot_count = ctx->opt.copy_buffers;

    ctx->slots = calloc((size_t)ctx->slot_count, sizeof(*ctx->slots));
    if (!ctx->slots) {
        fprintf(stderr, "Failed to allocate slot metadata\n");
        return -1;
    }

    if (ctx->zero_copy_mode) {
        ctx->slots[0].data = (uint8_t *)ctx->dma_map;
        ctx->slots[0].owns_data = false;
        return 0;
    }

    for (i = 0; i < ctx->slot_count; i++) {
        ctx->slots[i].data = malloc(ctx->display_frame_size);
        if (!ctx->slots[i].data) {
            fprintf(stderr, "Failed to allocate copy slot %d\n", i);
            return -1;
        }
        ctx->slots[i].owns_data = true;
    }

    return 0;
}

static int trigger_frame_dma(struct app_ctx *ctx)
{
    struct dma_transfer transfer;

    memset(&transfer, 0, sizeof(transfer));
    transfer.size = (uint32_t)ctx->frame_size;
    transfer.user_buf = (ctx->opt.io_mode == IO_MODE_COPY)
        ? (uint64_t)(uintptr_t)ctx->dma_copy
        : 0;

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

static void convert_frame_to_bgrx(struct app_ctx *ctx, uint8_t *dst, const uint8_t *src)
{
    size_t i;
    size_t pixel_count = (size_t)ctx->frame_width * ctx->frame_height;

    for (i = 0; i < pixel_count; i++) {
        uint8_t lo = src[(i << 1)];
        uint8_t hi = src[(i << 1) + 1];
        uint16_t pix;
        uint8_t r5;
        uint8_t g6;
        uint8_t b5;
        uint8_t r8;
        uint8_t g8;
        uint8_t b8;
        size_t out = i << 2;

        if (ctx->opt.swap16) {
            uint8_t tmp = lo;
            lo = hi;
            hi = tmp;
        }

        pix = (uint16_t)lo | ((uint16_t)hi << 8);
        if (ctx->opt.pixel_order == PIXEL_ORDER_BGR565) {
            r5 = pix & 0x1F;
            g6 = (pix >> 5) & 0x3F;
            b5 = (pix >> 11) & 0x1F;
        } else {
            r5 = (pix >> 11) & 0x1F;
            g6 = (pix >> 5) & 0x3F;
            b5 = pix & 0x1F;
        }

        r8 = (uint8_t)((r5 << 3) | (r5 >> 2));
        g8 = (uint8_t)((g6 << 2) | (g6 >> 4));
        b8 = (uint8_t)((b5 << 3) | (b5 >> 2));

        dst[out + 0] = b8;
        dst[out + 1] = g8;
        dst[out + 2] = r8;
        dst[out + 3] = 0xFF;
    }
}

static void prepare_display_frame(struct app_ctx *ctx, uint8_t *dst, const uint8_t *src)
{
    if (ctx->source_is_bgrx) {
        if (dst != src)
            memcpy(dst, src, ctx->display_frame_size);
        return;
    }
    convert_frame_to_bgrx(ctx, dst, src);
}

static void release_slot_ticket(struct app_ctx *ctx, const struct slot_ticket *ticket, bool count_release)
{
    if (!ctx || !ticket || ticket->idx < 0 || ticket->idx >= ctx->slot_count)
        return;

    g_mutex_lock(&ctx->slots_lock);
    if (ctx->slots[ticket->idx].in_use &&
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
    int64_t wait_start = mono_us();

    memset(ticket, 0, sizeof(*ticket));
    ticket->idx = -1;

    for (;;) {
        int i;
        int64_t now;
        int64_t wake_us;

        if (process_events(ctx, 10) < 0)
            return -1;
        if (!ctx->running)
            return -1;

        g_mutex_lock(&ctx->slots_lock);
        for (i = 0; i < ctx->slot_count; i++) {
            if (!ctx->slots[i].in_use) {
                ctx->slots[i].in_use = true;
                ctx->slots[i].generation++;
                ticket->idx = i;
                ticket->generation = ctx->slots[i].generation;
                g_mutex_unlock(&ctx->slots_lock);

                ctx->slot_wait_total_us += (uint64_t)(mono_us() - wait_start);
                ctx->slot_wait_samples++;
                return 0;
            }
        }

        now = mono_us();
        if (now >= deadline_us) {
            ctx->slot_wait_timeout_count++;
            g_mutex_unlock(&ctx->slots_lock);
            fprintf(stderr, "Timeout waiting free copy slot (%d ms)\n", ctx->opt.timeout_ms);
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
    struct frame_cookie *cookie;
    GstBuffer *buf;

    cookie = g_new0(struct frame_cookie, 1);
    if (!cookie)
        return NULL;

    cookie->ctx = ctx;
    cookie->ticket = *ticket;

    buf = gst_buffer_new_wrapped_full((GstMemoryFlags)0,
                                      ctx->slots[ticket->idx].data,
                                      ctx->display_frame_size,
                                      0,
                                      ctx->display_frame_size,
                                      cookie,
                                      frame_release_notify);
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

static void get_slot_counts(struct app_ctx *ctx, int *free_slots, int *used_slots)
{
    int i;
    int free_cnt = 0;
    int used_cnt = 0;

    g_mutex_lock(&ctx->slots_lock);
    for (i = 0; i < ctx->slot_count; i++) {
        if (ctx->slots[i].in_use)
            used_cnt++;
        else
            free_cnt++;
    }
    g_mutex_unlock(&ctx->slots_lock);

    if (free_slots)
        *free_slots = free_cnt;
    if (used_slots)
        *used_slots = used_cnt;
}

static void print_stats(struct app_ctx *ctx)
{
    int64_t now = mono_us();
    int64_t dt = now - ctx->last_stats_us;
    double avg_ms;
    double avg_slot_wait_ms;
    int free_slots;
    int used_slots;

    if (dt < (int64_t)ctx->opt.stats_interval * 1000000LL)
        return;

    avg_ms = ctx->loop_samples ? (ctx->total_loop_ms / (double)ctx->loop_samples) : 0.0;
    avg_slot_wait_ms = ctx->slot_wait_samples
        ? ((double)ctx->slot_wait_total_us / (double)ctx->slot_wait_samples / 1000.0)
        : 0.0;

    get_slot_counts(ctx, &free_slots, &used_slots);

    fprintf(stderr,
            "[stats] cap=%" PRIu64 " push=%" PRIu64 " rel=%" PRIu64
            " free=%d used=%d timeout=%" PRIu64
            " fps=%.2f rel_fps=%.2f avg_loop=%.2fms avg_slot_wait=%.2fms\n",
            ctx->captured_frames,
            ctx->pushed_frames,
            ctx->released_frames,
            free_slots,
            used_slots,
            ctx->slot_wait_timeout_count,
            (double)(ctx->captured_frames - ctx->last_stats_captured) * 1000000.0 / (double)dt,
            (double)(ctx->released_frames - ctx->last_stats_released) * 1000000.0 / (double)dt,
            avg_ms,
            avg_slot_wait_ms);

    ctx->last_stats_captured = ctx->captured_frames;
    ctx->last_stats_released = ctx->released_frames;
    ctx->last_stats_us = now;
}

static void print_pad_caps(const char *label, GstElement *elem, const char *pad_name)
{
    GstPad *pad;
    GstCaps *caps;
    gchar *caps_str;

    pad = gst_element_get_static_pad(elem, pad_name);
    if (!pad) {
        fprintf(stderr, "%s caps: <no pad '%s'>\n", label, pad_name);
        return;
    }

    caps = gst_pad_get_current_caps(pad);
    if (!caps)
        caps = gst_pad_query_caps(pad, NULL);

    if (!caps) {
        fprintf(stderr, "%s caps: <none>\n", label);
        gst_object_unref(pad);
        return;
    }

    caps_str = gst_caps_to_string(caps);
    fprintf(stderr, "%s caps: %s\n", label, caps_str ? caps_str : "<null>");
    g_free(caps_str);
    gst_caps_unref(caps);
    gst_object_unref(pad);
}

static int build_pipeline(struct app_ctx *ctx)
{
    GstCaps *caps;
    GstStateChangeReturn sret;
    const char *fmt = "BGRx";

    ctx->pipeline = gst_pipeline_new("fpga-hdmi");
    ctx->appsrc = gst_element_factory_make("appsrc", "src");
    ctx->queue = gst_element_factory_make("queue", "latency_queue");
    ctx->sink = gst_element_factory_make("kmssink", "sink");
    if (!ctx->pipeline || !ctx->appsrc || !ctx->queue || !ctx->sink) {
        fprintf(stderr, "Failed to create GStreamer elements (appsrc/queue/kmssink)\n");
        return -1;
    }

    gst_bin_add_many(GST_BIN(ctx->pipeline), ctx->appsrc, ctx->queue, ctx->sink, NULL);
    if (!gst_element_link_many(ctx->appsrc, ctx->queue, ctx->sink, NULL)) {
        fprintf(stderr, "Failed to link appsrc -> queue -> kmssink\n");
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
                 "block", FALSE,
                 "max-bytes", (guint64)ctx->display_frame_size * (guint64)ctx->opt.queue_depth,
                 NULL);
    gst_caps_unref(caps);

    /* Drop stale frames aggressively to keep live latency bounded. */
    g_object_set(ctx->queue,
                 "max-size-buffers", 1,
                 "max-size-bytes", 0,
                 "max-size-time", (guint64)0,
                 "leaky", 2,  /* downstream */
                 NULL);

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

    fprintf(stderr,
            "Pipeline started: appsrc(format=%s,block=false) -> queue(leaky=downstream,1) -> kmssink (copy_buffers=%d queue_depth=%d)\n",
            fmt, ctx->opt.copy_buffers, ctx->opt.queue_depth);
    print_pad_caps("appsrc:src", ctx->appsrc, "src");
    print_pad_caps("kmssink:sink", ctx->sink, "sink");
    return 0;
}

static void cleanup(struct app_ctx *ctx)
{
    int i;

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
    if (ctx->dma_copy)
        free(ctx->dma_copy);

    if (ctx->slots) {
        for (i = 0; i < ctx->slot_count; i++)
            if (ctx->slots[i].owns_data)
                free(ctx->slots[i].data);
        free(ctx->slots);
    }

    if (ctx->dev_fd >= 0)
        close(ctx->dev_fd);
    if (ctx->drm_fd >= 0)
        close(ctx->drm_fd);
    if (ctx->input_fd >= 0)
        close(ctx->input_fd);
    if (ctx->epoll_fd >= 0)
        close(ctx->epoll_fd);

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
    ctx.input_fd = -1;
    ctx.epoll_fd = -1;
    ctx.running = true;

    g_mutex_init(&ctx.slots_lock);
    g_cond_init(&ctx.slots_cond);

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

    if (init_copy_slots(&ctx) < 0)
        goto out;

    if (build_pipeline(&ctx) < 0)
        goto out;

    fprintf(stderr,
            "Start display loop: fps=%d src_fmt=%s io-mode=%s zero-copy=%s pixel-order=%s swap16=%s timeout=%dms copy_buffers=%d queue_depth=%d\n",
            ctx.opt.fps,
            pixel_format_name(ctx.pixel_format),
            (ctx.opt.io_mode == IO_MODE_MMAP) ? "mmap" : "copy",
            ctx.zero_copy_mode ? "on" : "off",
            (ctx.opt.pixel_order == PIXEL_ORDER_BGR565) ? "bgr565" : "rgb565",
            ctx.opt.swap16 ? "on" : "off",
            ctx.opt.timeout_ms,
            ctx.opt.copy_buffers,
            ctx.opt.queue_depth);

    ctx.start_us = mono_us();
    ctx.last_stats_us = ctx.start_us;

    while (ctx.running) {
        struct slot_ticket ticket;
        GstBuffer *buf;
        GstFlowReturn flow;
        const uint8_t *frame_src;
        int64_t t0;
        int64_t t1;
        int64_t target_us = 1000000LL / ctx.opt.fps;

        if (process_events(&ctx, 0) < 0)
            break;
        if (!ctx.running)
            break;

        t0 = mono_us();

        if (ctx.zero_copy_mode) {
            if (acquire_free_slot(&ctx, &ticket) < 0)
                break;

            if (trigger_frame_dma(&ctx) < 0) {
                fprintf(stderr, "DMA trigger failed\n");
                release_slot_ticket(&ctx, &ticket, false);
                break;
            }
            ctx.captured_frames++;

            frame_src = (const uint8_t *)ctx.dma_map;
            if (!frame_src) {
                fprintf(stderr, "Frame source is null in io-mode=mmap\n");
                release_slot_ticket(&ctx, &ticket, false);
                break;
            }
            prepare_display_frame(&ctx, ctx.slots[ticket.idx].data, frame_src);
        } else {
            if (trigger_frame_dma(&ctx) < 0) {
                fprintf(stderr, "DMA trigger failed\n");
                break;
            }
            ctx.captured_frames++;

            if (acquire_free_slot(&ctx, &ticket) < 0)
                break;

            frame_src = (ctx.opt.io_mode == IO_MODE_COPY) ? ctx.dma_copy : (const uint8_t *)ctx.dma_map;
            if (!frame_src) {
                fprintf(stderr, "Frame source is null in io-mode=%s\n",
                        (ctx.opt.io_mode == IO_MODE_MMAP) ? "mmap" : "copy");
                release_slot_ticket(&ctx, &ticket, false);
                break;
            }
            prepare_display_frame(&ctx, ctx.slots[ticket.idx].data, frame_src);
        }

        buf = build_frame_buffer(&ctx, &ticket);
        if (!buf) {
            fprintf(stderr, "Failed to build GstBuffer for slot %d\n", ticket.idx);
            break;
        }

        flow = gst_app_src_push_buffer(GST_APP_SRC(ctx.appsrc), buf);
        if (flow != GST_FLOW_OK) {
            fprintf(stderr, "gst_app_src_push_buffer failed: %d\n", flow);
            release_slot_ticket(&ctx, &ticket, false);
            break;
        }
        ctx.pushed_frames++;

        t1 = mono_us();
        ctx.total_loop_ms += (double)(t1 - t0) / 1000.0;
        ctx.loop_samples++;

        print_stats(&ctx);

        if ((t1 - t0) < target_us)
            usleep((useconds_t)(target_us - (t1 - t0)));
    }

    fprintf(stderr,
            "Exit: captured=%" PRIu64 " pushed=%" PRIu64 " released=%" PRIu64 " slot_timeout=%" PRIu64 "\n",
            ctx.captured_frames,
            ctx.pushed_frames,
            ctx.released_frames,
            ctx.slot_wait_timeout_count);

    ret = 0;

out:
    cleanup(&ctx);
    return ret;
}
