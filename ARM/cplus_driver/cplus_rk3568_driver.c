// SPDX-License-Identifier: MIT
#define _GNU_SOURCE
/* FPGA/DMA entry point for the C+ pedestrian crossing driver. */

#include "cplus_core.h"
#include "cplus_display.h"
#include "cplus_rknn.h"
#include "../pcie_fpga_dma.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <signal.h>
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

static void usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s --det-model detector.rknn --seg-model segmenter.rknn [options]\n"
            "  --device PATH          FPGA DMA device (default /dev/fpga_dma0)\n"
            "  --frames N             DMA frames to inspect (default 1)\n"
            "  --input-bgrx PATH      Offline BGRX8888 frame; needs --width and --height\n"
            "  --dump-bgrx PATH       Save the captured BGRX8888 frame for inspection\n"
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
        (options->display != 0 && options->display != 1)) return -1;
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

static void bgrx_to_rgb(const uint8_t *bgrx, int width, int height, uint8_t *rgb)
{
    size_t pixels = (size_t)width * height;
    size_t index;
    for (index = 0; index < pixels; ++index) {
        rgb[index * 3U] = bgrx[index * 4U + 2U];
        rgb[index * 3U + 1U] = bgrx[index * 4U + 1U];
        rgb[index * 3U + 2U] = bgrx[index * 4U];
    }
}

static volatile sig_atomic_t stop_requested;

static void handle_signal(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static int ordinary_people_count(const struct cplus_detection *detections, int count)
{
    int index, ordinary = 0;
    for (index = 0; index < count; ++index)
        if (detections[index].box.class_id == CPLUS_COCO_PERSON &&
            detections[index].target_type == CPLUS_TARGET_PEDESTRIAN) ++ordinary;
    return ordinary;
}

static void print_results(int frame_index, const struct cplus_person_result *results, int count)
{
    int index;
    printf("{\"frame\":%d,\"targets\":[", frame_index);
    for (index = 0; index < count; ++index) {
        const struct cplus_person_result *result = &results[index];
        if (index) putchar(',');
        printf("{\"type\":\"%s\",\"score\":%.3f,\"decision\":\"%s\",\"reason\":\"%s\","
               "\"box\":[%.1f,%.1f,%.1f,%.1f],\"ground\":{\"road\":%.4f,\"sidewalk\":%.4f,\"zebra\":%.4f}}",
               cplus_target_type_name(result->detection.target_type), result->detection.box.score,
               cplus_decision_name(result->decision), cplus_reason_name(result->reason),
               result->detection.box.x1, result->detection.box.y1,
               result->detection.box.x2, result->detection.box.y2,
               result->ground.road, result->ground.sidewalk, result->ground.zebra);
    }
    puts("]}");
}

int main(int argc, char **argv)
{
    struct options options;
    struct cplus_runtime_config config;
    struct cplus_rknn_model detector = {0}, segmenter = {0};
    struct cplus_display display = { .fd = -1, .active_fb = -1 };
    struct cplus_detection detections[CPLUS_MAX_DETECTIONS];
    struct cplus_person_result results[CPLUS_MAX_DETECTIONS];
    uint8_t *frame = NULL, *rgb = NULL, *detector_rgb = NULL, *segmenter_rgb = NULL, *mask = NULL;
    struct cplus_letterbox letterbox;
    const float *output;
    size_t output_count;
    int fd = -1, width, height, frame_count, frame_index, detection_count, result_count;
    size_t frame_size;
    int status = 1;

    if (parse_options(argc, argv, &options) < 0) { usage(argv[0]); return 2; }
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    if (options.input_bgrx_path) {
        width = options.width; height = options.height; frame_size = (size_t)width * height * 4U; frame_count = 1;
    } else if (open_dma_frame(options.device_path, &fd, &width, &height, &frame_size) == 0) {
        frame_count = options.frames;
    } else {
        fprintf(stderr, "Cannot open BGRX8888 FPGA DMA source: %s\n", options.device_path);
        return 1;
    }
    frame = malloc(frame_size); rgb = malloc((size_t)width * height * 3U);
    detector_rgb = malloc((size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT * 3U);
    segmenter_rgb = malloc((size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT * 3U);
    mask = malloc((size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT);
    if (!frame || !rgb || !detector_rgb || !segmenter_rgb || !mask ||
        cplus_rknn_model_load(&detector, "detector", options.detector_model) < 0 ||
        cplus_rknn_model_load(&segmenter, "segmenter", options.segmenter_model) < 0) {
        fprintf(stderr, "Model or buffer initialization failed\n");
        goto done;
    }
    if (detector.input_width != CPLUS_MODEL_WIDTH || detector.input_height != CPLUS_MODEL_HEIGHT ||
        segmenter.input_width != CPLUS_MODEL_WIDTH || segmenter.input_height != CPLUS_MODEL_HEIGHT) {
        fprintf(stderr, "Both models must use 640x640 inputs\n");
        goto done;
    }
    if (options.display && cplus_display_start(&display, options.drm_card_path, options.connector_id, width, height) < 0) {
        fprintf(stderr, "HDMI display initialization failed: %s\n", strerror(errno));
        goto done;
    }
    cplus_default_runtime_config(&config);
    for (frame_index = 0; !stop_requested && (frame_count == 0 || frame_index < frame_count); ++frame_index) {
        struct cplus_mask_stats mask_stats;
        if ((options.input_bgrx_path && read_file_exact(options.input_bgrx_path, frame, frame_size) < 0) ||
            (!options.input_bgrx_path && read_dma_frame(fd, frame, frame_size) < 0)) {
            fprintf(stderr, "Unable to read frame %d\n", frame_index);
            goto done;
        }
        if (options.dump_bgrx_path && write_file_exact(options.dump_bgrx_path, frame, frame_size) < 0) {
            fprintf(stderr, "Unable to save BGRX frame to %s\n", options.dump_bgrx_path);
            goto done;
        }
        bgrx_to_rgb(frame, width, height, rgb);
        fprintf(stderr, "[frame %d/%d] captured %dx%d\n", frame_index + 1, frame_count, width, height);
        cplus_prepare_detector_rgb(rgb, width, height, detector_rgb, &letterbox);
        if (cplus_rknn_infer_rgb(&detector, detector_rgb, CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT, &output, &output_count) < 0) {
            fprintf(stderr, "Detector inference failed\n"); goto done;
        }
        detection_count = cplus_decode_yolo(output, output_count, &letterbox, &config, detections, CPLUS_MAX_DETECTIONS);
        cplus_rknn_release_output(&detector);
        if (detection_count < 0) { fprintf(stderr, "Unexpected detector output shape\n"); goto done; }
        cplus_assign_riders(detections, detection_count, width, height, &config);
        fprintf(stderr, "[frame %d/%d] detector targets=%d ordinary_pedestrians=%d\n",
                frame_index + 1, frame_count, detection_count, ordinary_people_count(detections, detection_count));
        if (ordinary_people_count(detections, detection_count) || options.always_segment) {
            fprintf(stderr, "[frame %d/%d] running segmenter and ground processing\n", frame_index + 1, frame_count);
            cplus_prepare_segmenter_rgb(rgb, width, height, segmenter_rgb);
            if (cplus_rknn_infer_rgb(&segmenter, segmenter_rgb, CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT, &output, &output_count) < 0 ||
                cplus_mask_argmax(output, output_count, mask) < 0) {
                fprintf(stderr, "Segmenter inference failed or had an unexpected output shape\n"); goto done;
            }
            cplus_rknn_release_output(&segmenter);
            fprintf(stderr, "[frame %d/%d] segmenter done; applying Candidate C\n", frame_index + 1, frame_count);
            if (cplus_postprocess_mask_candidate_c(mask, CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT, &mask_stats) < 0) {
                fprintf(stderr, "Candidate C mask processing failed\n"); goto done;
            }
        } else {
            memset(mask, CPLUS_MASK_OTHER, (size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT);
        }
        result_count = cplus_evaluate_detections(detections, detection_count, mask,
                                                 CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT,
                                                 width, height, results, CPLUS_MAX_DETECTIONS);
        if (result_count < 0) { fprintf(stderr, "Rule evaluation failed\n"); goto done; }
        cplus_overlay_results(frame, width, height, results, result_count);
        if (display.started && cplus_display_present(&display, frame) < 0) {
            fprintf(stderr, "HDMI display update failed: %s\n", strerror(errno));
            goto done;
        }
        print_results(frame_index, results, result_count);
        fflush(stdout);
    }
    status = 0;
done:
    if (fd >= 0) close(fd);
    cplus_display_stop(&display);
    cplus_rknn_model_release(&detector); cplus_rknn_model_release(&segmenter);
    free(frame); free(rgb); free(detector_rgb); free(segmenter_rgb); free(mask);
    return status;
}
