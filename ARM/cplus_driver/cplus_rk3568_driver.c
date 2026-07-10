// SPDX-License-Identifier: MIT
#define _GNU_SOURCE
/* FPGA/DMA entry point for the C+ pedestrian crossing driver. */

#include "cplus_async.h"
#include "cplus_display.h"
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
    int always_segment;
};

static volatile sig_atomic_t stop_requested;

static void handle_signal(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static void usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s --det-model detector.rknn --seg-model segmenter.rknn [options]\n"
            "  --device PATH          FPGA DMA device (default /dev/fpga_dma0)\n"
            "  --frames N             Frames to display; 0 means continuous (default 1)\n"
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
    options->display = 1;
    options->drm_card_path = "/dev/dri/card0";
    options->connector_id = -1;
    while ((argument = getopt_long(argc, argv, "d:s:D:n:i:o:w:h:ap:r:c:?", long_options, NULL)) != -1) {
        switch (argument) {
        case 'd': options->detector_model = optarg; break;
        case 's': options->segmenter_model = optarg; break;
        case 'D': options->device_path = optarg; break;
        case 'n': options->frames = atoi(optarg); break;
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

static int present_frame(struct cplus_display *display, const uint8_t *frame,
                         const struct cplus_async_result *result, bool result_available)
{
    if (!display->started) return 0;
    return cplus_display_present(display, frame,
                                 result_available ? result->results : NULL,
                                 result_available ? result->count : 0,
                                 result_available);
}

int main(int argc, char **argv)
{
    struct options options;
    struct cplus_runtime_config config;
    struct cplus_rknn_model detector = {0};
    struct cplus_rknn_model segmenter = {0};
    struct cplus_display display = { .fd = -1, .active_fb = -1 };
    struct cplus_async_infer infer = { .pending_slot = -1 };
    struct cplus_async_result latest = {0};
    struct cplus_async_stats stats = {0};
    uint8_t *last_frame = NULL;
    int fd = -1;
    int width;
    int height;
    int frame_count;
    int frame_index;
    size_t frame_size;
    bool dump_written = false;
    int status = 1;

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
    if (options.display && cplus_display_start(&display, options.drm_card_path,
                                                options.connector_id, width, height) < 0) {
        fprintf(stderr, "HDMI display initialization failed: %s\n", strerror(errno));
        goto done;
    }
    if (cplus_async_infer_start(&infer, width, height, &detector, &segmenter,
                                &config, options.always_segment != 0) < 0) {
        fprintf(stderr, "Background inference initialization failed\n");
        goto done;
    }
    fprintf(stderr, "[async] display and inference started: frame=%dx%d queue=latest-only\n",
            width, height);

    for (frame_index = 0; !stop_requested &&
         (frame_count == 0 || frame_index < frame_count); ++frame_index) {
        int slot = -1;
        int submit_status;
        uint8_t *frame = cplus_async_acquire_frame(&infer, &slot);
        bool have_result;

        if (!frame) {
            fprintf(stderr, "Background inference stopped before frame %d\n", frame_index);
            goto done;
        }
        if ((options.input_bgrx_path && read_file_exact(options.input_bgrx_path, frame, frame_size) < 0) ||
            (!options.input_bgrx_path && read_dma_frame(fd, frame, frame_size) < 0)) {
            cplus_async_discard_frame(&infer, slot);
            fprintf(stderr, "Unable to read frame %d\n", frame_index);
            goto done;
        }
        if (options.dump_bgrx_path && !dump_written) {
            if (write_file_exact(options.dump_bgrx_path, frame, frame_size) < 0) {
                cplus_async_discard_frame(&infer, slot);
                fprintf(stderr, "Unable to save BGRX frame to %s\n", options.dump_bgrx_path);
                goto done;
            }
            dump_written = true;
        }
        have_result = cplus_async_get_result(&infer, &latest);
        if (present_frame(&display, frame, &latest, have_result) < 0) {
            cplus_async_discard_frame(&infer, slot);
            fprintf(stderr, "HDMI display update failed: %s\n", strerror(errno));
            goto done;
        }
        last_frame = frame;
        submit_status = cplus_async_submit_frame(&infer, slot, (uint64_t)frame_index);
        if (submit_status < 0) {
            fprintf(stderr, "Unable to queue frame %d for background inference\n", frame_index);
            goto done;
        }
        if (cplus_async_failed(&infer)) {
            fprintf(stderr, "Background inference failed\n");
            goto done;
        }
    }
    if (!stop_requested && cplus_async_wait_idle(&infer) < 0) {
        fprintf(stderr, "Background inference failed\n");
        goto done;
    }
    if (!stop_requested && cplus_async_get_result(&infer, &latest)) {
        if (last_frame && latest.source_frame == (uint64_t)(frame_index - 1) &&
            present_frame(&display, last_frame, &latest, true) < 0) {
            fprintf(stderr, "Final HDMI display update failed: %s\n", strerror(errno));
            goto done;
        }
    }
    status = 0;
done:
    cplus_async_get_stats(&infer, &stats);
    if (infer.thread_started) {
        fprintf(stderr, "[async] submitted=%llu completed=%llu dropped=%llu\n",
                (unsigned long long)stats.submitted, (unsigned long long)stats.completed,
                (unsigned long long)stats.dropped);
    }
    cplus_async_infer_stop(&infer);
    if (fd >= 0) close(fd);
    cplus_display_stop(&display);
    cplus_rknn_model_release(&detector);
    cplus_rknn_model_release(&segmenter);
    return status;
}
