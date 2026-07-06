// SPDX-License-Identifier: GPL-2.0
/* Minimal direct DRM/KMS display diagnostic for FPGA BGRX frames. */

#include "pcie_fpga_dma.h"

#include <drm_fourcc.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#define DEFAULT_DEVICE "/dev/" FPGA_DMA_DEV_NAME
#define DEFAULT_DRM_CARD "/dev/dri/card0"
#define DEFAULT_FPS 30
#define DRM_BUFFER_COUNT 3

struct options {
    const char *device_path;
    const char *drm_card_path;
    int connector_id;
    int fps;
    int frames;
    bool wait_new_frame;
};

struct drm_fb {
    uint32_t handle;
    uint32_t fb_id;
    uint32_t pitch;
    uint64_t size;
    uint8_t *map;
};

struct drm_state {
    int fd;
    uint32_t crtc_id;
    uint32_t connector_id;
    drmModeModeInfo mode;
    drmModeCrtc *saved_crtc;
    struct drm_fb fb[DRM_BUFFER_COUNT];
};

struct interval_stats {
    int samples;
    int64_t min_us;
    int64_t max_us;
    int64_t sum_us;
    int over_40ms;
    int over_50ms;
};

static volatile sig_atomic_t g_stop;

static int64_t mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
}

static void sleep_until_us(int64_t target_us)
{
    struct timespec ts;

    if (target_us <= mono_us())
        return;
    ts.tv_sec = (time_t)(target_us / 1000000LL);
    ts.tv_nsec = (long)((target_us % 1000000LL) * 1000LL);
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) < 0 && errno == EINTR)
        ;
}

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static void stats_update(struct interval_stats *stats, int64_t delta_us)
{
    if (!stats || delta_us < 0)
        return;
    if (stats->samples == 0 || delta_us < stats->min_us)
        stats->min_us = delta_us;
    if (delta_us > stats->max_us)
        stats->max_us = delta_us;
    stats->sum_us += delta_us;
    stats->samples++;
    if (delta_us > 40000)
        stats->over_40ms++;
    if (delta_us > 50000)
        stats->over_50ms++;
}

static void stats_print(const char *name, const struct interval_stats *stats)
{
    if (!stats || stats->samples <= 0)
        return;
    fprintf(stderr,
            "[drm-display] %s interval: samples=%d avg_ms=%.2f min_ms=%.2f max_ms=%.2f over40ms=%d over50ms=%d\n",
            name, stats->samples,
            (double)stats->sum_us / (double)stats->samples / 1000.0,
            (double)stats->min_us / 1000.0,
            (double)stats->max_us / 1000.0,
            stats->over_40ms, stats->over_50ms);
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s [OPTIONS]\n"
            "  --device <path>       FPGA DMA device (default: %s)\n"
            "  --drm-card <path>     DRM card (default: %s)\n"
            "  --connector-id <id>   Optional connector id\n"
            "  --fps <num>           Target fps (default: %d)\n"
            "  --frames <num>        Stop after N frames (default: 0, run until Ctrl+C)\n"
            "  --wait-new-frame <0|1> Wait for FPGA frame status before read (default: 1)\n",
            prog, DEFAULT_DEVICE, DEFAULT_DRM_CARD, DEFAULT_FPS);
}

static int parse_bool(const char *s, bool *out)
{
    if (strcmp(s, "1") == 0 || strcasecmp(s, "true") == 0 || strcasecmp(s, "on") == 0) {
        *out = true;
        return 0;
    }
    if (strcmp(s, "0") == 0 || strcasecmp(s, "false") == 0 || strcasecmp(s, "off") == 0) {
        *out = false;
        return 0;
    }
    return -1;
}

static int parse_options(int argc, char **argv, struct options *opt)
{
    static const struct option long_opts[] = {
        {"device", required_argument, NULL, 1},
        {"drm-card", required_argument, NULL, 2},
        {"connector-id", required_argument, NULL, 3},
        {"fps", required_argument, NULL, 4},
        {"frames", required_argument, NULL, 5},
        {"wait-new-frame", required_argument, NULL, 6},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };
    int c;

    opt->device_path = DEFAULT_DEVICE;
    opt->drm_card_path = DEFAULT_DRM_CARD;
    opt->connector_id = -1;
    opt->fps = DEFAULT_FPS;
    opt->frames = 0;
    opt->wait_new_frame = true;

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
            opt->fps = atoi(optarg);
            if (opt->fps <= 0)
                return -1;
            break;
        case 5:
            opt->frames = atoi(optarg);
            if (opt->frames < 0)
                return -1;
            break;
        case 6:
            if (parse_bool(optarg, &opt->wait_new_frame) < 0)
                return -1;
            break;
        case 'h':
            usage(argv[0]);
            exit(0);
        default:
            return -1;
        }
    }
    return 0;
}

static int dma_get_status(int fd, struct fpga_frame_status *status)
{
    memset(status, 0, sizeof(*status));
    if (ioctl(fd, FPGA_DMA_GET_FRAME_STATUS, status) < 0)
        return -1;
    return status->magic == FPGA_FRAME_STATUS_MAGIC ? 0 : -1;
}

static int dma_wait_new_frame(int fd, uint32_t *last_change_count, int timeout_ms)
{
    int waited_ms = 0;

    while (timeout_ms <= 0 || waited_ms < timeout_ms) {
        struct fpga_frame_status status;
        if (dma_get_status(fd, &status) < 0)
            return -1;
        if (status.frame_change_count != *last_change_count) {
            *last_change_count = status.frame_change_count;
            return 0;
        }
        usleep(1000);
        waited_ms++;
    }
    return -1;
}

static int dma_read_frame(int fd, uint32_t slot, void *dst, size_t size)
{
    struct dma_transfer t;

    memset(&t, 0, sizeof(t));
    t.size = (uint32_t)size;
    t.offset = slot;
    t.user_buf = (uint64_t)(uintptr_t)dst;
    if (ioctl(fd, FPGA_DMA_READ_FRAME, &t) < 0)
        return -1;
    return t.result == 0 ? 0 : -1;
}

static drmModeConnector *find_connector(int fd, drmModeRes *res, int connector_id)
{
    for (int i = 0; i < res->count_connectors; i++) {
        drmModeConnector *conn = drmModeGetConnector(fd, res->connectors[i]);
        if (!conn)
            continue;
        if ((connector_id < 0 || (int)conn->connector_id == connector_id) &&
            conn->connection == DRM_MODE_CONNECTED && conn->count_modes > 0)
            return conn;
        drmModeFreeConnector(conn);
    }
    return NULL;
}

static int choose_mode(const drmModeConnector *conn, uint32_t w, uint32_t h, drmModeModeInfo *mode)
{
    int best = -1;

    for (int i = 0; i < conn->count_modes; i++) {
        if ((uint32_t)conn->modes[i].hdisplay == w && (uint32_t)conn->modes[i].vdisplay == h) {
            if (best < 0 || (conn->modes[i].type & DRM_MODE_TYPE_PREFERRED))
                best = i;
        }
    }
    if (best < 0)
        return -1;
    *mode = conn->modes[best];
    return 0;
}

static uint32_t find_crtc(int fd, drmModeRes *res, drmModeConnector *conn)
{
    drmModeEncoder *enc;

    if (conn->encoder_id) {
        enc = drmModeGetEncoder(fd, conn->encoder_id);
        if (enc) {
            uint32_t crtc_id = enc->crtc_id;
            drmModeFreeEncoder(enc);
            if (crtc_id)
                return crtc_id;
        }
    }

    for (int i = 0; i < conn->count_encoders; i++) {
        enc = drmModeGetEncoder(fd, conn->encoders[i]);
        if (!enc)
            continue;
        for (int j = 0; j < res->count_crtcs; j++) {
            if (enc->possible_crtcs & (1 << j)) {
                uint32_t crtc_id = res->crtcs[j];
                drmModeFreeEncoder(enc);
                return crtc_id;
            }
        }
        drmModeFreeEncoder(enc);
    }
    return 0;
}

static int drm_create_fb(int fd, uint32_t w, uint32_t h, struct drm_fb *fb)
{
    struct drm_mode_create_dumb creq;
    struct drm_mode_map_dumb mreq;
    uint32_t handles[4] = {0};
    uint32_t pitches[4] = {0};
    uint32_t offsets[4] = {0};

    memset(fb, 0, sizeof(*fb));
    memset(&creq, 0, sizeof(creq));
    creq.width = w;
    creq.height = h;
    creq.bpp = 32;
    if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0)
        return -1;

    fb->handle = creq.handle;
    fb->pitch = creq.pitch;
    fb->size = creq.size;
    handles[0] = fb->handle;
    pitches[0] = fb->pitch;
    if (drmModeAddFB2(fd, w, h, DRM_FORMAT_XRGB8888, handles, pitches, offsets, &fb->fb_id, 0) < 0)
        return -1;

    memset(&mreq, 0, sizeof(mreq));
    mreq.handle = fb->handle;
    if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) < 0)
        return -1;
    fb->map = mmap(NULL, fb->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)mreq.offset);
    if (fb->map == MAP_FAILED) {
        fb->map = NULL;
        return -1;
    }
    memset(fb->map, 0, fb->size);
    return 0;
}

static void drm_destroy_fb(int fd, struct drm_fb *fb)
{
    struct drm_mode_destroy_dumb dreq;

    if (!fb)
        return;
    if (fb->map)
        munmap(fb->map, fb->size);
    if (fb->fb_id)
        drmModeRmFB(fd, fb->fb_id);
    if (fb->handle) {
        memset(&dreq, 0, sizeof(dreq));
        dreq.handle = fb->handle;
        ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
    }
    memset(fb, 0, sizeof(*fb));
}

static int drm_init(struct drm_state *drm, const struct options *opt, uint32_t w, uint32_t h)
{
    drmModeRes *res;
    drmModeConnector *conn;
    uint32_t conn_id;

    memset(drm, 0, sizeof(*drm));
    drm->fd = -1;
    drm->fd = open(opt->drm_card_path, O_RDWR | O_CLOEXEC);
    if (drm->fd < 0)
        return -1;

    res = drmModeGetResources(drm->fd);
    if (!res)
        return -1;
    conn = find_connector(drm->fd, res, opt->connector_id);
    if (!conn) {
        drmModeFreeResources(res);
        errno = ENODEV;
        return -1;
    }
    if (choose_mode(conn, w, h, &drm->mode) < 0) {
        fprintf(stderr, "[drm-display] no %ux%u mode on connector %u\n", w, h, conn->connector_id);
        drmModeFreeConnector(conn);
        drmModeFreeResources(res);
        errno = EINVAL;
        return -1;
    }
    drm->crtc_id = find_crtc(drm->fd, res, conn);
    drm->connector_id = conn->connector_id;
    conn_id = conn->connector_id;
    drmModeFreeConnector(conn);
    drmModeFreeResources(res);
    if (!drm->crtc_id) {
        errno = ENODEV;
        return -1;
    }

    drm->saved_crtc = drmModeGetCrtc(drm->fd, drm->crtc_id);
    for (int i = 0; i < DRM_BUFFER_COUNT; i++) {
        if (drm_create_fb(drm->fd, w, h, &drm->fb[i]) < 0)
            return -1;
    }
    fprintf(stderr, "[drm-display] connector=%u crtc=%u mode=%ux%u@%u\n",
            conn_id, drm->crtc_id, drm->mode.hdisplay, drm->mode.vdisplay, drm->mode.vrefresh);
    return 0;
}

static void drm_cleanup(struct drm_state *drm)
{
    if (!drm)
        return;
    if (drm->fd >= 0 && drm->saved_crtc) {
        drmModeSetCrtc(drm->fd, drm->saved_crtc->crtc_id, drm->saved_crtc->buffer_id,
                       drm->saved_crtc->x, drm->saved_crtc->y,
                       &drm->connector_id, 1, &drm->saved_crtc->mode);
        drmModeFreeCrtc(drm->saved_crtc);
    }
    if (drm->fd >= 0) {
        for (int i = 0; i < DRM_BUFFER_COUNT; i++)
            drm_destroy_fb(drm->fd, &drm->fb[i]);
        close(drm->fd);
    }
    memset(drm, 0, sizeof(*drm));
    drm->fd = -1;
}

static void page_flip_handler(int fd, unsigned int frame, unsigned int sec,
                              unsigned int usec, void *data)
{
    bool *waiting = (bool *)data;
    (void)fd;
    (void)frame;
    (void)sec;
    (void)usec;
    *waiting = false;
}

static int wait_flip(int fd, bool *waiting)
{
    drmEventContext ev;

    memset(&ev, 0, sizeof(ev));
    ev.version = DRM_EVENT_CONTEXT_VERSION;
    ev.page_flip_handler = page_flip_handler;
    while (*waiting && !g_stop) {
        fd_set fds;
        int ret;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        ret = select(fd + 1, &fds, NULL, NULL, NULL);
        if (ret < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (drmHandleEvent(fd, &ev) < 0)
            return -1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    struct options opt;
    struct drm_state drm;
    struct fpga_info info;
    struct fpga_frame_status status;
    struct interval_stats read_intervals = {0};
    struct interval_stats flip_intervals = {0};
    int dma_fd = -1;
    size_t frame_size;
    uint32_t last_change = 0;
    int ret = 1;
    int current_fb = 0;
    int64_t next_us;
    int64_t prev_read_done = 0;
    int64_t prev_flip_done = 0;
    uint64_t frames = 0;

    if (parse_options(argc, argv, &opt) < 0) {
        usage(argv[0]);
        return 1;
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    dma_fd = open(opt.device_path, O_RDWR | O_CLOEXEC);
    if (dma_fd < 0) {
        perror("open fpga dma");
        goto out;
    }
    if (ioctl(dma_fd, FPGA_DMA_GET_INFO, &info) < 0) {
        perror("FPGA_DMA_GET_INFO");
        goto out;
    }
    if (info.pixel_format != FPGA_PIXEL_FORMAT_BGRX8888 && info.frame_bpp != 4) {
        fprintf(stderr, "[drm-display] BGRX8888 source required, pixel_format=%u bpp=%u\n",
                info.pixel_format, info.frame_bpp);
        goto out;
    }
    info.frame_bpp = 4;
    if (info.frame_stride < info.frame_width * info.frame_bpp)
        info.frame_stride = info.frame_width * info.frame_bpp;
    frame_size = (size_t)info.frame_stride * info.frame_height;
    fprintf(stderr, "[drm-display] DMA frame=%ux%u stride=%u size=%zu fps=%d wait_new_frame=%d\n",
            info.frame_width, info.frame_height, info.frame_stride, frame_size,
            opt.fps, opt.wait_new_frame ? 1 : 0);

    if (drm_init(&drm, &opt, info.frame_width, info.frame_height) < 0) {
        perror("drm init");
        goto out;
    }
    if (opt.wait_new_frame) {
        if (dma_get_status(dma_fd, &status) < 0) {
            fprintf(stderr, "[drm-display] frame status unavailable\n");
            goto out;
        }
        last_change = status.frame_change_count;
    }

    next_us = mono_us();
    for (int frame = 0; !g_stop && (opt.frames == 0 || frame < opt.frames); frame++) {
        int fb_index;
        int64_t read_done;

        if (opt.wait_new_frame) {
            if (dma_wait_new_frame(dma_fd, &last_change, 1000) < 0) {
                fprintf(stderr, "[drm-display] wait new frame failed\n");
                goto out;
            }
        } else {
            sleep_until_us(next_us);
            next_us += 1000000LL / opt.fps;
        }

        fb_index = (frame == 0) ? 0 : ((current_fb + 1) % DRM_BUFFER_COUNT);
        if (dma_read_frame(dma_fd, 0, drm.fb[fb_index].map, frame_size) < 0) {
            perror("FPGA_DMA_READ_FRAME");
            goto out;
        }
        read_done = mono_us();
        if (prev_read_done > 0)
            stats_update(&read_intervals, read_done - prev_read_done);
        prev_read_done = read_done;

        if (frame == 0) {
            if (drmModeSetCrtc(drm.fd, drm.crtc_id, drm.fb[fb_index].fb_id, 0, 0,
                               &drm.connector_id, 1, &drm.mode) < 0) {
                perror("drmModeSetCrtc");
                goto out;
            }
            current_fb = fb_index;
        } else {
            bool waiting = true;
            if (drmModePageFlip(drm.fd, drm.crtc_id, drm.fb[fb_index].fb_id,
                                DRM_MODE_PAGE_FLIP_EVENT, &waiting) < 0) {
                perror("drmModePageFlip");
                goto out;
            }
            if (wait_flip(drm.fd, &waiting) < 0) {
                perror("wait page flip");
                goto out;
            }
            current_fb = fb_index;
            if (prev_flip_done > 0)
                stats_update(&flip_intervals, mono_us() - prev_flip_done);
            prev_flip_done = mono_us();
        }
        frames++;
        if (frames % 60U == 0)
            fprintf(stderr, "[drm-display] frames=%llu\n", (unsigned long long)frames);
    }
    ret = 0;

out:
    fprintf(stderr, "[drm-display] exit frames=%llu\n", (unsigned long long)frames);
    stats_print("read-done", &read_intervals);
    stats_print("page-flip", &flip_intervals);
    drm_cleanup(&drm);
    if (dma_fd >= 0)
        close(dma_fd);
    return ret;
}
