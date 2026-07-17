// SPDX-License-Identifier: MIT
#define _GNU_SOURCE
/* FPGA/DMA entry point for the C+ pedestrian crossing driver. */

#include "cplus_async.h"
#include "cplus_display_async.h"
#include "cplus_frame_pool.h"
#include "cplus_rga.h"
#include "cplus_rknn.h"
#include "../pcie_fpga_dma.h"

#include <dirent.h>
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
    const char *input_frames_dir;
    const char *fixed_mask_path;
    const char *generate_fixed_mask_path;
    const char *dump_bgrx_path;
    const char *output_mask_bgrx_path;
    const char *output_frames_dir;
    const char *drm_card_path;
    int connector_id;
    int display;
    int width;
    int height;
    int frames;
    int fps;
    int always_segment;
    int input_bgrx_stream;
    int input_nv12_stream;
    int src_width;
    int src_height;
    int hold_display;
    int display_mask;
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
            "  --input-frames-dir PATH Offline BGRX8888 frame directory (frameNNNNN.bgrx);\n"
            "                          loads models once and processes every frame\n"
            "  --input-bgrx-stream    Read consecutive BGRX8888 frames from stdin\n"
            "  --input-nv12-stream    Read consecutive NV12 frames from stdin (RGA hardware\n"
            "                          converts to BGRX with letterbox scaling; needs\n"
            "                          --src-width/--src-height and --width/--height)\n"
            "  --src-width N --src-height N  Source video resolution for --input-nv12-stream\n"
            "  --fixed-mask PATH      Reuse a 640x640 semantic mask; detector only\n"
            "  --generate-fixed-mask PATH  Generate and save a 640x640 mask from --input-bgrx\n"
            "  --hold-display         Keep the final offline frame on HDMI until stopped\n"
            "  --display-mask 0|1    Draw the full semantic mask on HDMI (default 1)\n"
            "  --dump-bgrx PATH       Save the first captured BGRX8888 frame\n"
            "  --output-mask-bgrx PATH  Save the offline frame with the colored mask overlay\n"
            "  --output-frames-dir PATH  Save every processed frame with mask+box overlay\n"
            "                            as frameNNNNN.bgrx (one file per input frame)\n"
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
        {"input-frames-dir", required_argument, NULL, 'I'},
        {"input-bgrx-stream", no_argument, NULL, 'S'},
        {"input-nv12-stream", no_argument, NULL, 'N'},
        {"src-width", required_argument, NULL, 'X'},
        {"src-height", required_argument, NULL, 'Y'},
        {"fixed-mask", required_argument, NULL, 'm'},
        {"generate-fixed-mask", required_argument, NULL, 'g'},
        {"hold-display", no_argument, NULL, 'H'},
        {"display-mask", required_argument, NULL, 'M'},
        {"dump-bgrx", required_argument, NULL, 'o'},
        {"output-mask-bgrx", required_argument, NULL, 'O'},
        {"output-frames-dir", required_argument, NULL, 'E'},
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
    options->display_mask = 1;
    options->drm_card_path = "/dev/dri/card0";
    options->connector_id = -1;
    while ((argument = getopt_long(argc, argv, "d:s:D:n:F:i:I:SN:X:Y:m:g:HM:o:O:E:w:h:ap:r:c:?",
                                   long_options, NULL)) != -1) {
        switch (argument) {
        case 'd': options->detector_model = optarg; break;
        case 's': options->segmenter_model = optarg; break;
        case 'D': options->device_path = optarg; break;
        case 'n': options->frames = atoi(optarg); break;
        case 'F': options->fps = atoi(optarg); break;
        case 'i': options->input_bgrx_path = optarg; break;
        case 'I': options->input_frames_dir = optarg; break;
        case 'S': options->input_bgrx_stream = 1; break;
        case 'N': options->input_nv12_stream = 1; break;
        case 'X': options->src_width = atoi(optarg); break;
        case 'Y': options->src_height = atoi(optarg); break;
        case 'm': options->fixed_mask_path = optarg; break;
        case 'g': options->generate_fixed_mask_path = optarg; break;
        case 'H': options->hold_display = 1; break;
        case 'M': options->display_mask = atoi(optarg); break;
        case 'o': options->dump_bgrx_path = optarg; break;
        case 'O': options->output_mask_bgrx_path = optarg; break;
        case 'E': options->output_frames_dir = optarg; break;
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
        (options->input_frames_dir && (options->width <= 0 || options->height <= 0)) ||
        (options->input_bgrx_stream && (options->width <= 0 || options->height <= 0)) ||
        (options->input_nv12_stream && (options->width <= 0 || options->height <= 0 ||
                                       options->src_width <= 0 || options->src_height <= 0)) ||
        ((options->input_bgrx_path ? 1 : 0) +
         (options->input_frames_dir ? 1 : 0) +
         options->input_bgrx_stream + options->input_nv12_stream > 1) ||
        (options->output_mask_bgrx_path && !options->input_bgrx_path) ||
        (options->output_frames_dir && !options->input_frames_dir) ||
        (options->generate_fixed_mask_path && !options->input_bgrx_path) ||
        (options->fixed_mask_path && options->generate_fixed_mask_path) ||
        (options->hold_display && !options->display) ||
        (options->display_mask != 0 && options->display_mask != 1) ||
        (options->display != 0 && options->display != 1))
        return -1;
    return 0;
}

static int read_stream_frame(int fd, uint8_t *buffer, size_t size)
{
    size_t used = 0;
    while (used < size) {
        ssize_t count = read(fd, buffer + used, size - used);
        if (count < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (count == 0)
            return used == 0 ? 1 : -1;
        used += (size_t)count;
    }
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

/* Collect sorted frame*.bgrx entries from dir into names[] (capacity cap).
 * Returns the count, or -1 on error. Names are heap-allocated and owned by
 * the caller; free with free_frame_names. */
static int collect_frame_files(const char *dir, char **names, int cap)
{
    DIR *handle = opendir(dir);
    struct dirent *entry;
    int count = 0;
    if (!handle) return -1;
    while ((entry = readdir(handle)) != NULL) {
        const char *dot = strrchr(entry->d_name, '.');
        if (!dot || strcmp(dot, ".bgrx") != 0) continue;
        if (strncmp(entry->d_name, "frame", 5) != 0) continue;
        if (count >= cap) {
            closedir(handle);
            return -1;
        }
        {
            size_t need = strlen(dir) + 1 + strlen(entry->d_name) + 1;
            char *full = (char *)malloc(need);
            if (!full) {
                closedir(handle);
                return -1;
            }
            snprintf(full, need, "%s/%s", dir, entry->d_name);
            names[count++] = full;
        }
    }
    closedir(handle);
    /* Simple insertion sort by filename so frame00000 < frame00001 < ... */
    {
        int i, j;
        for (i = 1; i < count; ++i) {
            char *value = names[i];
            const char *base_i = strrchr(value, '/') ? strrchr(value, '/') + 1 : value;
            for (j = i - 1; j >= 0; --j) {
                const char *base_j = strrchr(names[j], '/') ? strrchr(names[j], '/') + 1 : names[j];
                if (strcmp(base_j, base_i) > 0) {
                    names[j + 1] = names[j];
                } else {
                    break;
                }
            }
            names[j + 1] = value;
        }
    }
    return count;
}

static void free_frame_names(char **names, int count)
{
    int i;
    for (i = 0; i < count; ++i) free(names[i]);
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

static void report_npu_frequency(void)
{
    static const char *paths[] = {
        "/sys/kernel/debug/rknpu/freq",
        "/proc/rknpu/freq",
        "/sys/class/devfreq/fde40000.npu/cur_freq",
    };
    size_t index;
    for (index = 0; index < sizeof(paths) / sizeof(paths[0]); ++index) {
        FILE *file = fopen(paths[index], "r");
        unsigned long frequency;
        if (!file) continue;
        if (fscanf(file, "%lu", &frequency) == 1) {
            fprintf(stderr, "[npu] frequency=%.0fMHz source=%s\n",
                    (double)frequency / 1000000.0, paths[index]);
            fclose(file);
            return;
        }
        fclose(file);
    }
    fprintf(stderr, "[npu] frequency unavailable from debugfs/procfs/devfreq\n");
}

static bool fp_output_type(rknn_tensor_type type)
{
    return type == RKNN_TENSOR_FLOAT16 || type == RKNN_TENSOR_FLOAT32;
}

static int validate_model_contracts(const struct cplus_rknn_model *detector,
                                    const struct cplus_rknn_model *segmenter)
{
    const rknn_tensor_attr *detector_output = &detector->output_attr;
    const rknn_tensor_attr *segmenter_output = &segmenter->output_attr;
    if (detector_output->n_dims != 3 ||
        detector_output->dims[0] != 1 ||
        detector_output->dims[1] != CPLUS_YOLO_CHANNELS ||
        detector_output->dims[2] != CPLUS_YOLO_PREDICTIONS ||
        detector_output->n_elems !=
            (uint32_t)(CPLUS_YOLO_CHANNELS * CPLUS_YOLO_PREDICTIONS) ||
        !fp_output_type(detector_output->type)) {
        fprintf(stderr, "Detector output must be [1,84,8400] in FP16 or FP32\n");
        return -1;
    }
    if (segmenter_output->n_dims != 4 ||
        segmenter_output->dims[0] != 1 ||
        segmenter_output->dims[1] != 4 ||
        segmenter_output->dims[2] != CPLUS_MODEL_HEIGHT ||
        segmenter_output->dims[3] != CPLUS_MODEL_WIDTH ||
        segmenter_output->n_elems != (uint32_t)(4U * CPLUS_MODEL_PIXELS) ||
        segmenter_output->fmt != RKNN_TENSOR_NCHW ||
        !fp_output_type(segmenter_output->type)) {
        fprintf(stderr,
                "Segmenter output must be NCHW [1,4,640,640] in FP16 or FP32\n");
        return -1;
    }
    return 0;
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
    char *frame_names[1024];
    int frame_names_count = 0;
    bool fixed_mask_mode;
    int64_t stream_stats_last_us = 0;
    uint64_t stream_stats_last_input = 0;
    uint64_t stream_stats_last_displayed = 0;
    uint64_t stream_stats_last_inferred = 0;
    struct cplus_rga *rga = NULL;
    uint8_t *nv12_input = NULL;
    size_t nv12_size = 0;

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
    } else if (options.input_frames_dir) {
        width = options.width;
        height = options.height;
        frame_size = (size_t)width * height * 4U;
        frame_names_count = collect_frame_files(options.input_frames_dir,
                                                 frame_names, 1024);
        if (frame_names_count <= 0) {
            fprintf(stderr, "No frame*.bgrx files found in %s\n", options.input_frames_dir);
            return 1;
        }
        frame_count = frame_names_count;
    } else if (options.input_bgrx_stream) {
        width = options.width;
        height = options.height;
        frame_size = (size_t)width * height * 4U;
        frame_count = 0;
    } else if (options.input_nv12_stream) {
        width = options.width;
        height = options.height;
        /* The pool holds BGRX output frames (after RGA conversion); the raw
         * NV12 input is read into a separate scratch buffer. */
        frame_size = (size_t)width * height * 4U;
        frame_count = 0;
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
    if (options.input_nv12_stream) {
        nv12_size = (size_t)options.src_width * options.src_height * 3U / 2U;
        nv12_input = malloc(nv12_size);
        if (!nv12_input) {
            fprintf(stderr, "NV12 input buffer allocation failed\n");
            goto done;
        }
        if (cplus_rga_init(&rga, options.src_width, options.src_height,
                           width, height) < 0 || !rga) {
            fprintf(stderr, "RGA hardware conversion initialization failed\n");
            goto done;
        }
    }
    if (cplus_rknn_model_load(&detector, "detector", options.detector_model) < 0 ||
        cplus_rknn_model_load(&segmenter, "segmenter", options.segmenter_model) < 0) {
        fprintf(stderr, "Model initialization failed\n");
        goto done;
    }
    if (detector.input_width != CPLUS_MODEL_WIDTH ||
        detector.input_height != CPLUS_MODEL_HEIGHT ||
        detector.input_channels != 3 ||
        segmenter.input_width != CPLUS_MODEL_WIDTH ||
        segmenter.input_height != CPLUS_MODEL_HEIGHT ||
        segmenter.input_channels != 3) {
        fprintf(stderr, "Both models must use three-channel 640x640 inputs\n");
        goto done;
    }
    if (validate_model_contracts(&detector, &segmenter) < 0) goto done;
    report_npu_frequency();
    cplus_default_runtime_config(&config);
    if (options.display &&
        cplus_display_async_start(&display, &pool, options.drm_card_path,
                                  options.connector_id, width, height) < 0) {
        fprintf(stderr, "Asynchronous HDMI display initialization failed: %s\n",
                strerror(errno));
        goto done;
    }
    fixed_mask_mode = options.fixed_mask_path ||
                      options.generate_fixed_mask_path;
    if (cplus_async_infer_start(&infer, &pool, width, height, &detector, &segmenter,
                                &config, options.always_segment != 0,
                                fixed_mask_mode) < 0) {
        fprintf(stderr, "Background inference initialization failed\n");
        goto done;
    }
    if (options.fixed_mask_path) {
        uint8_t fixed_mask[CPLUS_MODEL_PIXELS];
        if (read_file_exact(options.fixed_mask_path, fixed_mask,
                            sizeof(fixed_mask)) < 0 ||
            cplus_async_set_fixed_mask(&infer, fixed_mask,
                                       sizeof(fixed_mask), 0) < 0) {
            fprintf(stderr, "Unable to load fixed mask %s\n",
                    options.fixed_mask_path);
            goto done;
        }
    } else if (options.generate_fixed_mask_path &&
               cplus_async_request_fixed_mask(&infer) < 0) {
        fprintf(stderr, "Unable to request fixed mask generation\n");
        goto done;
    }
    fprintf(stderr, "[pipeline] asynchronous capture/display/inference started: "
                    "frame=%dx%d fps=%d slots=%d\n",
            width, height, options.fps, CPLUS_FRAME_POOL_SLOTS);
    frame_period_us = 1000000LL / options.fps;
    next_frame_us = monotonic_us();
    stream_stats_last_us = next_frame_us;

    for (frame_index = 0; !stop_requested &&
         (frame_count == 0 || frame_index < frame_count); ++frame_index) {
        uint8_t *frame = NULL;
        int slot = -1;
        int acquire_status;
        bool have_result;
        bool keep_final;

        if (!options.input_bgrx_path && !options.input_frames_dir &&
            !options.input_bgrx_stream && !options.input_nv12_stream &&
            frame_index > 0) {
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
        if (options.input_bgrx_stream) {
            int read_status = read_stream_frame(STDIN_FILENO, frame, frame_size);
            if (read_status == 1) {
                cplus_frame_pool_release(&pool, slot);
                break;
            }
            if (read_status < 0) {
                cplus_frame_pool_release(&pool, slot);
                fprintf(stderr, "Unable to read streamed frame %d\n", frame_index);
                goto done;
            }
        } else if (options.input_nv12_stream) {
            /* Read raw NV12 into the scratch buffer, then let RGA hardware
             * convert it (NV12 -> BGRX with letterbox scaling) into the pool
             * slot. After this the slot holds a normal BGRX frame and the rest
             * of the pipeline (inference/display) is identical to bgrx_stream. */
            int read_status = read_stream_frame(STDIN_FILENO, nv12_input, nv12_size);
            if (read_status == 1) {
                cplus_frame_pool_release(&pool, slot);
                break;
            }
            if (read_status < 0) {
                cplus_frame_pool_release(&pool, slot);
                fprintf(stderr, "Unable to read streamed NV12 frame %d\n", frame_index);
                goto done;
            }
            const uint8_t *converted = cplus_rga_convert(rga, nv12_input, nv12_size);
            if (!converted) {
                cplus_frame_pool_release(&pool, slot);
                fprintf(stderr, "RGA conversion failed for frame %d\n", frame_index);
                goto done;
            }
            memcpy(frame, converted, frame_size);
        } else if ((options.input_bgrx_path && read_file_exact(options.input_bgrx_path, frame, frame_size) < 0) ||
            (options.input_frames_dir && read_file_exact(frame_names[frame_index], frame, frame_size) < 0) ||
            (!options.input_bgrx_path && !options.input_frames_dir && read_dma_frame(fd, frame, frame_size) < 0)) {
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
        have_result = cplus_async_refresh_result(&infer, &latest);
        if (cplus_async_submit_frame(&infer, slot, (uint64_t)frame_index) < 0 ||
            (options.display && cplus_display_async_submit(&display, slot,
                                                           latest.results, latest.count,
                                                           have_result, latest.mask,
                                                           have_result && latest.mask_valid &&
                                                               options.display_mask) < 0)) {
            cplus_frame_pool_release(&pool, slot);
            fprintf(stderr, "Unable to submit frame %d to asynchronous pipeline\n", frame_index);
            goto done;
        }
        if (options.input_frames_dir) {
            /* Offline batch mode: wait for this frame's result now, render the
             * annotated overlay, and release the slot. Models stay loaded. */
            if (cplus_async_wait_for_frame(&infer, (uint64_t)frame_index) < 0 ||
                !cplus_async_get_result(&infer, &latest)) {
                cplus_frame_pool_release(&pool, slot);
                fprintf(stderr, "Frame %d inference failed\n", frame_index);
                goto done;
            }
            if (options.output_frames_dir) {
                const uint8_t *source = cplus_frame_pool_data(&pool, slot);
                uint8_t *rendered = source ? malloc(frame_size) : NULL;
                char out_path[4096];
                if (!rendered) {
                    cplus_frame_pool_release(&pool, slot);
                    fprintf(stderr, "Unable to allocate output frame %d\n", frame_index);
                    goto done;
                }
                memcpy(rendered, source, frame_size);
                if (latest.mask_valid) {
                    cplus_overlay_mask_bgrx(rendered, width * 4, width, height,
                                            latest.mask, CPLUS_MODEL_WIDTH,
                                            CPLUS_MODEL_HEIGHT);
                }
                cplus_overlay_results(rendered, width * 4, width, height,
                                       latest.results, latest.count, latest.count > 0);
                snprintf(out_path, sizeof(out_path), "%s/frame%05d.bgrx",
                         options.output_frames_dir, frame_index);
                if (write_file_exact(out_path, rendered, frame_size) < 0) {
                    free(rendered);
                    cplus_frame_pool_release(&pool, slot);
                    fprintf(stderr, "Unable to write output frame %d\n", frame_index);
                    goto done;
                }
                free(rendered);
            }
            cplus_frame_pool_release(&pool, slot);
        } else {
            keep_final = frame_count > 0 && frame_index + 1 == frame_count;
            if (keep_final) {
                final_slot = slot;
            } else {
                cplus_frame_pool_release(&pool, slot);
            }
        }
        if (cplus_async_failed(&infer) ||
            (options.display && cplus_display_async_failed(&display))) {
            fprintf(stderr, "Asynchronous worker failed\n");
            goto done;
        }
        if (options.input_bgrx_stream || options.input_nv12_stream) {
            int64_t now_us = monotonic_us();
            int64_t elapsed_us = now_us - stream_stats_last_us;
            if (elapsed_us >= 2000000) {
                struct cplus_async_stats current_infer = {0};
                uint64_t current_displayed = 0;
                uint64_t ignored_replaced = 0;
                uint64_t current_input = (uint64_t)frame_index + 1U;
                double scale = 1000000.0 / (double)elapsed_us;
                cplus_async_get_stats(&infer, &current_infer);
                cplus_display_async_stats(&display, &current_displayed,
                                          &ignored_replaced);
                fprintf(stderr,
                        "[stream] input_fps=%.1f display_fps=%.1f infer_fps=%.1f "
                        "input=%llu displayed=%llu inferred=%llu\n",
                        (double)(current_input - stream_stats_last_input) * scale,
                        (double)(current_displayed - stream_stats_last_displayed) * scale,
                        (double)(current_infer.completed - stream_stats_last_inferred) * scale,
                        (unsigned long long)current_input,
                        (unsigned long long)current_displayed,
                        (unsigned long long)current_infer.completed);
                stream_stats_last_us = now_us;
                stream_stats_last_input = current_input;
                stream_stats_last_displayed = current_displayed;
                stream_stats_last_inferred = current_infer.completed;
            }
        }
    }
    if (!stop_requested && options.generate_fixed_mask_path) {
        uint8_t fixed_mask[CPLUS_MODEL_PIXELS];
        if (cplus_async_wait_for_frame(&infer, 0) < 0 ||
            !cplus_async_get_result(&infer, &latest) ||
            cplus_async_copy_fixed_mask(&infer, fixed_mask, sizeof(fixed_mask),
                                        NULL, NULL) < 0 ||
            write_file_exact(options.generate_fixed_mask_path, fixed_mask,
                             sizeof(fixed_mask)) < 0) {
            fprintf(stderr, "Unable to save fixed mask %s\n",
                    options.generate_fixed_mask_path);
            goto done;
        }
    }
    if (!stop_requested && !options.input_frames_dir && frame_count > 0 && frame_index > 0) {
        uint64_t final_frame = (uint64_t)(frame_index - 1);
        if (cplus_async_wait_for_frame(&infer, final_frame) < 0 ||
            !cplus_async_get_result(&infer, &latest)) {
            fprintf(stderr, "Final background inference failed\n");
            goto done;
        }
        if (options.display &&
            cplus_display_async_submit(&display, final_slot, latest.results,
                                       latest.count, true, latest.mask,
                                       latest.mask_valid && options.display_mask) < 0) {
            fprintf(stderr, "Unable to submit final result to HDMI display\n");
            goto done;
        }
        if (options.output_mask_bgrx_path) {
            const uint8_t *source = cplus_frame_pool_data(&pool, final_slot);
            uint8_t *rendered = source ? malloc(frame_size) : NULL;

            if (!rendered) {
                fprintf(stderr, "Unable to allocate mask output frame\n");
                goto done;
            }
            memcpy(rendered, source, frame_size);
            if (latest.mask_valid) {
                cplus_overlay_mask_bgrx(rendered, width * 4, width, height,
                                        latest.mask, CPLUS_MODEL_WIDTH,
                                        CPLUS_MODEL_HEIGHT);
            }
            /* Draw person boxes, foot regions and decision labels so the
             * saved JPEG/web preview shows which targets are suspected
             * violations, not just the ground mask. */
            cplus_overlay_results(rendered, width * 4, width, height,
                                  latest.results, latest.count, latest.count > 0);
            if (write_file_exact(options.output_mask_bgrx_path, rendered,
                                 frame_size) < 0) {
                free(rendered);
                fprintf(stderr, "Unable to save mask overlay BGRX frame to %s\n",
                        options.output_mask_bgrx_path);
                goto done;
            }
            free(rendered);
            fprintf(stderr, "[mask] wrote overlay frame %s valid=%d targets=%d\n",
                    options.output_mask_bgrx_path,
                    latest.mask_valid ? 1 : 0, latest.count);
        }
    }
    if (!stop_requested && options.hold_display) {
        fprintf(stderr, "[display] holding final frame until stopped\n");
        while (!stop_requested)
            sleep_until_us(monotonic_us() + 100000);
    }
    status = 0;

done:
    if (final_slot >= 0) {
        cplus_frame_pool_release(&pool, final_slot);
        final_slot = -1;
    }
    if (rga) {
        cplus_rga_destroy(rga);
        rga = NULL;
    }
    free(nv12_input);
    nv12_input = NULL;
    cplus_async_get_stats(&infer, &infer_stats);
    cplus_display_async_stats(&display, &display_presented, &display_replaced);
    display_presented = cplus_display_async_stop(&display);
    infer_stats.completed = cplus_async_infer_stop(&infer);
    if (infer_stats.submitted || infer_stats.completed) {
        fprintf(stderr, "[pipeline] inference submitted=%llu completed=%llu replaced=%llu "
                        "displayed=%llu display_replaced=%llu\n",
                (unsigned long long)infer_stats.submitted,
                (unsigned long long)infer_stats.completed,
                (unsigned long long)infer_stats.replaced,
                (unsigned long long)display_presented,
                (unsigned long long)display_replaced);
    }
    if (fd >= 0) close(fd);
    cplus_rknn_model_release(&detector);
    cplus_rknn_model_release(&segmenter);
    if (pool_initialized && cplus_frame_pool_destroy(&pool) < 0) {
        fprintf(stderr, "Shared frame pool still had outstanding references during shutdown\n");
        status = 1;
    }
    if (frame_names_count > 0) free_frame_names(frame_names, frame_names_count);
    return status;
}
