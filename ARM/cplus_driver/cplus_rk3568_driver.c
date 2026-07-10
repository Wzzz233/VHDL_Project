// SPDX-License-Identifier: MIT
#define _GNU_SOURCE
/* FPGA/DMA entry point for the C+ pedestrian crossing driver. */

#include "cplus_async.h"
#include "cplus_display_async.h"
#include "cplus_frame_pool.h"
#include "cplus_rknn.h"
#include "../pcie_fpga_dma.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

struct options {
    const char *detector_model;
    const char *segmenter_model;
    const char *device_path;
    const char *input_bgrx_path;
    const char *dump_bgrx_path;
    const char *drm_card_path;
    int connector_id;
    int display;
    int width;
    int height;
    int frames;
    int fps;
    int always_segment;
};

static volatile sig_atomic_t stop_requested;

static void handle_signal(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static int64_t monotonic_us(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)now.tv_sec * 1000000LL + now.tv_nsec / 1000;
}

static void sleep_until_us(int64_t deadline_us)
{
    struct timespec deadline;
    int result;
    deadline.tv_sec = (time_t)(deadline_us / 1000000LL);
    deadline.tv_nsec = (long)(deadline_us % 1000000LL) * 1000L;
    do {
        result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
    } while (result == EINTR && !stop_requested);
}

static void usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s --det-model detector.rknn --seg-model segmenter.rknn [options]\n"
            "  --device PATH          FPGA DMA device (default /dev/fpga_dma0)\n"
            "  --frames N             Frames to capture; 0 means continuous (default 1)\n"
            "  --fps N                Live capture/display rate, 1-120 (default 30)\n"
            "  --input-bgrx PATH      Offline BGRX8888 frame; needs --width and --height\n"
            "  --dump-bgrx PATH       Save the first captured BGRX8888 frame\n"
            "  --width N --height N   Offline BGRX frame dimensions\n"
            "  --always-segment       Run segmenter even when no ordinary person remains\n"
            "  --display 0|1          HDMI display with result overlay (default 1)\n"
            "  --drm-card PATH        DRM card for HDMI display (default /dev/dri/card0)\n"
            "  --connector-id N       Optional DRM connector id\n",
            program);
}

static int parse_options(int argc, char **argv, struct options *options)
{
    static const struct option long_options[] = {
        {"det-model", required_argument, NULL, 'd'},
        {"seg-model", required_argument, NULL, 's'},
        {"device", required_argument, NULL, 'D'},
        {"frames", required_argument, NULL, 'n'},
        {"fps", required_argument, NULL, 'F'},
        {"input-bgrx", required_argument, NULL, 'i'},
        {"dump-bgrx", required_argument, NULL, 'o'},
        {"width", required_argument, NULL, 'w'},
        {"height", required_argument, NULL, 'h'},
        {"always-segment", no_argument, NULL, 'a'},
        {"display", required_argument, NULL, 'p'},
        {"drm-card", required_argument, NULL, 'r'},
        {"connector-id", required_argument, NULL, 'c'},
        {"help", no_argument, NULL, '?'},
        {0, 0, 0, 0},
    };
    int argument;
    memset(options, 0, sizeof(*options));
    options->device_path = "/dev/fpga_dma0";
    options->frames = 1;
    options->fps = 30;
    options->display = 1;
    options->drm_card_path = "/dev/dri/card0";
    options->connector_id = -1;
    while ((argument = getopt_long(argc, argv, "d:s:D:n:F:i:o:w:h:ap:r:c:?",
                                   long_options, NULL)) != -1) {
        switch (argument) {
        case 'd': options->detector_model = optarg; break;
        case 's': options->segmenter_model = optarg; break;
        case 'D': options->device_path = optarg; break;
        case 'n': options->frames = atoi(optarg); break;
        case 'F': options->fps = atoi(optarg); break;
        case 'i': options->input_bgrx_path = optarg; break;
        case 'o': options->dump_bgrx_path = optarg; break;
        case 'w': options->width = atoi(optarg); break;
        case 'h': options->height = atoi(optarg); break;
        case 'a': options->always_segment = 1; break;
        case 'p': options->display = atoi(optarg); break;
        case 'r': options->drm_card_path = optarg; break;
        case 'c': options->connector_id = atoi(optarg); break;
        default: return -1;
        }
    }
    if (!options->detector_model || !options->segmenter_model || options->frames < 0 ||
        options->fps < 1 || options->fps > 120 ||
        (options->input_bgrx_path && (options->width <= 0 || options->height <= 0)) ||
        (options->display != 0 && options->display != 1))
        return -1;
    return 0;
}

static int read_file_exact(const char *path, uint8_t *buffer, size_t size)
{
    FILE *file = fopen(path, "rb");
    int result = -1;
    if (!file) return -1;
    if (fread(buffer, 1, size, file) == size && fgetc(file) == EOF) result = 0;
    fclose(file);
    return result;
}

static int write_file_exact(const char *path, const uint8_t *buffer, size_t size)
{
    FILE *file = fopen(path, "wb");
    int result = -1;
    if (!file) return -1;
    if (fwrite(buffer, 1, size, file) == size && fflush(file) == 0) result = 0;
    fclose(file);
    return result;
}

static int open_dma_frame(const char *path, int *fd, int *width, int *height, size_t *size)
{
    struct fpga_info info;
    *fd = open(path, O_RDWR | O_CLOEXEC);
    if (*fd < 0 || ioctl(*fd, FPGA_DMA_GET_INFO, &info) < 0 ||
        info.pixel_format != FPGA_PIXEL_FORMAT_BGRX8888 || info.frame_bpp != 4) {
        if (*fd >= 0) close(*fd);
        *fd = -1;
        return -1;
    }
    *width = (int)info.frame_width;
    *height = (int)info.frame_height;
    *size = (size_t)info.frame_width * info.frame_height * 4U;
    return 0;
}

static int read_dma_frame(int fd, uint8_t *frame, size_t size)
{
    struct dma_transfer transfer;
    memset(&transfer, 0, sizeof(transfer));
    transfer.size = (uint32_t)size;
    transfer.user_buf = (uint64_t)(uintptr_t)frame;
    return ioctl(fd, FPGA_DMA_READ_FRAME, &transfer) == 0 && transfer.result == 0 ? 0 : -1;
}

int main(int argc, char **argv)
{
    struct options options;
    struct cplus_runtime_config config;
    struct cplus_rknn_model detector = {0};
    struct cplus_rknn_model segmenter = {0};
    struct cplus_frame_pool pool = {0};
    struct cplus_display_async display = { .drm = { .fd = -1, .active_fb = -1 } };
    struct cplus_async_infer infer = {0};
    struct cplus_async_result latest = {0};
    struct cplus_async_stats infer_stats = {0};
    uint64_t display_presented = 0;
    uint64_t display_replaced = 0;
    int fd = -1;
    int width;
    int height;
    int frame_count;
    int frame_index;
    int final_slot = -1;
    size_t frame_size;
    bool dump_written = false;
    int pool_initialized = 0;
    int status = 1;
    int64_t next_frame_us;
    int64_t frame_period_us;

    if (parse_options(argc, argv, &options) < 0) {
        usage(argv[0]);
        return 2;
    }
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    if (options.input_bgrx_path) {
        width = options.width;
        height = options.height;
        frame_size = (size_t)width * height * 4U;
        frame_count = 1;
    } else if (open_dma_frame(options.device_path, &fd, &width, &height, &frame_size) == 0) {
        frame_count = options.frames;
    } else {
        fprintf(stderr, "Cannot open BGRX8888 FPGA DMA source: %s\n", options.device_path);
        return 1;
    }
    if (cplus_frame_pool_init(&pool, frame_size) < 0) {
        fprintf(stderr, "Shared frame pool initialization failed\n");
        goto done;
    }
    pool_initialized = 1;
    if (cplus_rknn_model_load(&detector, "detector", options.detector_model) < 0 ||
        cplus_rknn_model_load(&segmenter, "segmenter", options.segmenter_model) < 0) {
        fprintf(stderr, "Model initialization failed\n");
        goto done;
    }
    if (detector.input_width != CPLUS_MODEL_WIDTH || detector.input_height != CPLUS_MODEL_HEIGHT ||
        segmenter.input_width != CPLUS_MODEL_WIDTH || segmenter.input_height != CPLUS_MODEL_HEIGHT) {
        fprintf(stderr, "Both models must use 640x640 inputs\n");
        goto done;
    }
    cplus_default_runtime_config(&config);
    if (options.display &&
        cplus_display_async_start(&display, &pool, options.drm_card_path,
                                  options.connector_id, width, height) < 0) {
        fprintf(stderr, "Asynchronous HDMI display initialization failed: %s\n",
                strerror(errno));
        goto done;
    }
    if (cplus_async_infer_start(&infer, &pool, width, height, &detector, &segmenter,
                                &config, options.always_segment != 0) < 0) {
        fprintf(stderr, "Background inference initialization failed\n");
        goto done;
    }
    fprintf(stderr, "[pipeline] asynchronous capture/display/inference started: "
                    "frame=%dx%d fps=%d slots=%d\n",
            width, height, options.fps, CPLUS_FRAME_POOL_SLOTS);
    frame_period_us = 1000000LL / options.fps;
    next_frame_us = monotonic_us();

    for (frame_index = 0; !stop_requested &&
         (frame_count == 0 || frame_index < frame_count); ++frame_index) {
        struct cplus_async_result overlay = {0};
        uint8_t *frame = NULL;
        int slot = -1;
        int acquire_status;
        bool have_result;
        bool keep_final;

        if (!options.input_bgrx_path && frame_index > 0) {
            next_frame_us += frame_period_us;
            sleep_until_us(next_frame_us);
            if (stop_requested) break;
            if (monotonic_us() > next_frame_us + frame_period_us)
                next_frame_us = monotonic_us();
        }
        acquire_status = cplus_frame_pool_acquire(&pool, &slot, &frame);
        if (acquire_status != 0) {
            fprintf(stderr, "Shared frame pool exhausted at frame %d\n", frame_index);
            goto done;
        }
        if ((options.input_bgrx_path && read_file_exact(options.input_bgrx_path, frame, frame_size) < 0) ||
            (!options.input_bgrx_path && read_dma_frame(fd, frame, frame_size) < 0)) {
            cplus_frame_pool_release(&pool, slot);
            fprintf(stderr, "Unable to read frame %d\n", frame_index);
            goto done;
        }
        if (options.dump_bgrx_path && !dump_written) {
            if (write_file_exact(options.dump_bgrx_path, frame, frame_size) < 0) {
                cplus_frame_pool_release(&pool, slot);
                fprintf(stderr, "Unable to save BGRX frame to %s\n", options.dump_bgrx_path);
                goto done;
            }
            dump_written = true;
        }
        have_result = cplus_async_get_result(&infer, &overlay);
        if (cplus_async_submit_frame(&infer, slot, (uint64_t)frame_index) < 0 ||
            (options.display && cplus_display_async_submit(&display, slot,
                                                           overlay.results, overlay.count,
                                                           have_result) < 0)) {
            cplus_frame_pool_release(&pool, slot);
            fprintf(stderr, "Unable to submit frame %d to asynchronous pipeline\n", frame_index);
            goto done;
        }
        keep_final = frame_count > 0 && frame_index + 1 == frame_count;
        if (keep_final) {
            final_slot = slot;
        } else {
            cplus_frame_pool_release(&pool, slot);
        }
        if (cplus_async_failed(&infer) ||
            (options.display && cplus_display_async_failed(&display))) {
            fprintf(stderr, "Asynchronous worker failed\n");
            goto done;
        }
    }
    if (!stop_requested && frame_count > 0 && frame_index > 0) {
        uint64_t final_frame = (uint64_t)(frame_index - 1);
        if (cplus_async_wait_for_frame(&infer, final_frame) < 0 ||
            !cplus_async_get_result(&infer, &latest)) {
            fprintf(stderr, "Final background inference failed\n");
            goto done;
        }
        if (options.display &&
            cplus_display_async_submit(&display, final_slot, latest.results,
                                       latest.count, true) < 0) {
            fprintf(stderr, "Unable to submit final result to HDMI display\n");
            goto done;
        }
    }
    status = 0;

done:
    if (final_slot >= 0) {
        cplus_frame_pool_release(&pool, final_slot);
        final_slot = -1;
    }
    cplus_async_get_stats(&infer, &infer_stats);
    cplus_display_async_stats(&display, &display_presented, &display_replaced);
    if (infer.thread_started) {
        fprintf(stderr, "[pipeline] inference submitted=%llu completed=%llu replaced=%llu "
                        "displayed=%llu display_replaced=%llu\n",
                (unsigned long long)infer_stats.submitted,
                (unsigned long long)infer_stats.completed,
                (unsigned long long)infer_stats.replaced,
                (unsigned long long)display_presented,
                (unsigned long long)display_replaced);
    }
    cplus_display_async_stop(&display);
    cplus_async_infer_stop(&infer);
    if (fd >= 0) close(fd);
    cplus_rknn_model_release(&detector);
    cplus_rknn_model_release(&segmenter);
    if (pool_initialized && cplus_frame_pool_destroy(&pool) < 0) {
        fprintf(stderr, "Shared frame pool still had outstanding references during shutdown\n");
        status = 1;
    }
    return status;
}
