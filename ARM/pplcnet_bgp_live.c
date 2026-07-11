// SPDX-License-Identifier: GPL-2.0
/*
 * Standalone blue/green/police/embassy/yellow PPLCNet live validation driver.
 *
 * Pipeline:
 *   OV5640 FPGA DMA or phone WebRTC/RTSP frame -> YOLOv8n-pose plate quad ->
 *   quad warp crop -> blue / green / police / embassy / yellow PPLCNet CTC ->
 *   HDMI/KMS display overlay.
 *
 * Routing (see lpr_infer.c::pick_route + lpr_color.c):
 *   green body  -> green PPLCNet (8-char new-energy decode family)
 *   yellow body -> yellow PPLCNet if loaded, else police, else blue
 *   white body  -> police PPLCNet if loaded, else blue
 *   black body  -> embassy PPLCNet if loaded, else police, else blue
 *   blue body / unknown -> blue PPLCNet (default base case)
 *
 * Police, embassy and yellow OCR are optional; if omitted, the driver silently
 * falls back through the route chain above.
 *
 * Code organization (see lpr_live/):
 *   lpr_common  : shared types and utilities
 *   lpr_dma     : FPGA DMA capture and pixel format conversion
 *   lpr_detector: YOLOv8n-pose decode + NMS
 *   lpr_warp    : 4-point homography crop
 *   lpr_color   : RGB body color classifier fallback
 *   lpr_ptype   : optional RKNN plate-type classifier route override
 *   lpr_ocr     : PPLCNet CTC OCR (input prep, layout autodetect, decode)
 *   lpr_display : DRM/KMS RGB16 output + overlay drawing
 *   lpr_infer   : background inference thread, route selection
 *
 * This file (pplcnet_bgp_live.c) is the main entry point: option parsing,
 * model loading, DMA pump loop and shutdown.
 */

#include "lpr_live/lpr_common.h"
#include "lpr_live/lpr_color.h"
#include "lpr_live/lpr_control.h"
#include "lpr_live/lpr_detector.h"
#include "lpr_live/lpr_display.h"
#include "lpr_live/lpr_dma.h"
#include "lpr_live/lpr_fpga_source.h"
#include "lpr_live/lpr_frame.h"
#include "lpr_live/lpr_infer.h"
#include "lpr_live/lpr_ocr.h"
#include "lpr_live/lpr_phone_source.h"
#include "lpr_live/lpr_preview.h"
#include "lpr_live/lpr_ptype.h"
#include "lpr_live/lpr_source.h"
#include "lpr_live/lpr_warp.h"
#include "ocr_decode.h"
#include "pcie_fpga_dma.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <gst/gst.h>

#define DEFAULT_DEVICE   "/dev/" FPGA_DMA_DEV_NAME
#define DEFAULT_DRM_CARD "/dev/dri/card0"
#define DEFAULT_PHONE_RTSP "rtsp://127.0.0.1:8554/phone"
#define DEFAULT_CONTROL_SOCKET "/run/pplcnet-bgp-live/control.sock"
#define LIVE_FRAME_WIDTH 1280U
#define LIVE_FRAME_HEIGHT 720U

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static uint32_t lpr_camera_status_words(const struct fpga_frame_status *status)
{
    return status->camera_shape & 0x000fffffU;
}

static uint32_t lpr_camera_status_lines(const struct fpga_frame_status *status)
{
    return status->camera_shape >> 20;
}

struct lpr_interval_stats {
    int samples;
    int64_t min_us;
    int64_t max_us;
    int64_t sum_us;
    int over_40ms;
    int over_50ms;
};

static void lpr_interval_stats_update(struct lpr_interval_stats *stats, int64_t delta_us)
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

static void lpr_interval_stats_print(const char *name, const struct lpr_interval_stats *stats)
{
    if (!stats || stats->samples <= 0)
        return;
    fprintf(stderr,
            "[bgp-live] %s interval summary: samples=%d avg_ms=%.2f min_ms=%.2f max_ms=%.2f over40ms=%d over50ms=%d\n",
            name, stats->samples,
            (double)stats->sum_us / (double)stats->samples / 1000.0,
            (double)stats->min_us / 1000.0, (double)stats->max_us / 1000.0,
            stats->over_40ms, stats->over_50ms);
}

static uint64_t lpr_hash_mix64(uint64_t h, uint64_t v)
{
    h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h *= 1099511628211ULL;
    return h;
}

static uint64_t lpr_frame_hash64_full(const uint8_t *data, size_t size)
{
    uint64_t h = lpr_hash_mix64(1469598103934665603ULL, (uint64_t)size);
    size_t i = 0;

    while (i + sizeof(uint64_t) <= size) {
        uint64_t v;
        memcpy(&v, data + i, sizeof(v));
        h = lpr_hash_mix64(h, v);
        i += sizeof(uint64_t);
    }
    while (i < size) {
        h = lpr_hash_mix64(h, (uint64_t)data[i]);
        i++;
    }
    return h;
}

static uint64_t lpr_rotl64(uint64_t v, unsigned int r)
{
    return (v << r) | (v >> (64U - r));
}

#define LPR_XXH64_PRIME1 11400714785074694791ULL
#define LPR_XXH64_PRIME2 14029467366897019727ULL
#define LPR_XXH64_PRIME3 1609587929392839161ULL
#define LPR_XXH64_PRIME4 9650029242287828579ULL
#define LPR_XXH64_PRIME5 2870177450012600261ULL

static uint64_t lpr_read64_unaligned(const uint8_t *data)
{
    uint64_t v;
    memcpy(&v, data, sizeof(v));
    return v;
}

static uint32_t lpr_read32_unaligned(const uint8_t *data)
{
    uint32_t v;
    memcpy(&v, data, sizeof(v));
    return v;
}

static uint64_t lpr_xxh64_round(uint64_t acc, uint64_t input)
{
    acc += input * LPR_XXH64_PRIME2;
    acc = lpr_rotl64(acc, 31);
    acc *= LPR_XXH64_PRIME1;
    return acc;
}

static uint64_t lpr_xxh64_merge_round(uint64_t acc, uint64_t val)
{
    val = lpr_xxh64_round(0, val);
    acc ^= val;
    acc = acc * LPR_XXH64_PRIME1 + LPR_XXH64_PRIME4;
    return acc;
}

static uint64_t lpr_frame_fingerprint64_fast(const uint8_t *data, size_t size)
{
    const uint8_t *p = data;
    const uint8_t *end = data + size;
    uint64_t h;

    if (size >= 32U) {
        const uint8_t *limit = end - 32U;
        uint64_t v1 = LPR_XXH64_PRIME1 + LPR_XXH64_PRIME2;
        uint64_t v2 = LPR_XXH64_PRIME2;
        uint64_t v3 = 0;
        uint64_t v4 = 0 - LPR_XXH64_PRIME1;

        do {
            v1 = lpr_xxh64_round(v1, lpr_read64_unaligned(p)); p += 8U;
            v2 = lpr_xxh64_round(v2, lpr_read64_unaligned(p)); p += 8U;
            v3 = lpr_xxh64_round(v3, lpr_read64_unaligned(p)); p += 8U;
            v4 = lpr_xxh64_round(v4, lpr_read64_unaligned(p)); p += 8U;
        } while (p <= limit);

        h = lpr_rotl64(v1, 1) + lpr_rotl64(v2, 7) +
            lpr_rotl64(v3, 12) + lpr_rotl64(v4, 18);
        h = lpr_xxh64_merge_round(h, v1);
        h = lpr_xxh64_merge_round(h, v2);
        h = lpr_xxh64_merge_round(h, v3);
        h = lpr_xxh64_merge_round(h, v4);
    } else {
        h = LPR_XXH64_PRIME5;
    }

    h += (uint64_t)size;

    while ((size_t)(end - p) >= 8U) {
        uint64_t k1 = lpr_xxh64_round(0, lpr_read64_unaligned(p));
        h ^= k1;
        h = lpr_rotl64(h, 27) * LPR_XXH64_PRIME1 + LPR_XXH64_PRIME4;
        p += 8U;
    }
    if ((size_t)(end - p) >= 4U) {
        h ^= (uint64_t)lpr_read32_unaligned(p) * LPR_XXH64_PRIME1;
        h = lpr_rotl64(h, 23) * LPR_XXH64_PRIME2 + LPR_XXH64_PRIME3;
        p += 4U;
    }
    while (p < end) {
        h ^= (uint64_t)(*p) * LPR_XXH64_PRIME5;
        h = lpr_rotl64(h, 11) * LPR_XXH64_PRIME1;
        p++;
    }

    h ^= h >> 33;
    h *= LPR_XXH64_PRIME2;
    h ^= h >> 29;
    h *= LPR_XXH64_PRIME3;
    h ^= h >> 32;
    return h;
}

/* FPGA frame-identity stamp: wr_buf overwrites the first 8 pixels of the
 * first and last active lines with {X=0xA5, R=cnt, G=~cnt, B=0x5A} where cnt
 * increments once per camera frame. Returns 0 and the counter value if all 8
 * stamp pixels are well-formed and agree, -1 otherwise. */
#define FRAME_STAMP_PIXELS 8
static int frame_stamp_extract(const uint8_t *frame, uint32_t w, uint32_t line, uint8_t *cnt_out)
{
    const uint8_t *p = frame + (size_t)line * w * 4U;
    uint8_t cnt = p[2];
    int i;

    for (i = 0; i < FRAME_STAMP_PIXELS; i++, p += 4) {
        if (p[0] != 0x5A || p[3] != 0xA5 ||
            p[1] != (uint8_t)~p[2] || p[2] != cnt)
            return -1;
    }
    *cnt_out = cnt;
    return 0;
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s --plate-model yolov8n_pose.rknn --ocr-blue-model blue.rknn --ocr-green-model green.rknn [opts]\n"
            "\n"
            "Required:\n"
            "  --plate-model <path>          YOLOv8n pose RKNN plate detector\n"
            "  --ocr-blue-model <path>       Blue PPLCNet OCR RKNN\n"
            "  --ocr-green-model <path>      Green PPLCNet OCR RKNN\n"
            "  --ocr-keys <path>             Shared fallback OCR keys file for blue+green\n"
            "  --ocr-blue-keys <path>        Override blue OCR keys file\n"
            "  --ocr-green-keys <path>       Override green OCR keys file\n"
            "\n"
            "Optional route classifier:\n"
            "  --plate-type-classifier-model <path|off>  BGPEY classifier RKNN; off disables\n"
            "  --plate-type-classifier-min-conf <v>      Min confidence for route override (default: 0.80)\n"
            "  --plate-type-classifier-special-min-conf <v> Min police/embassy confidence (default: 0.70)\n"
            "\n"
            "Optional routes (silent fallback if omitted):\n"
            "  --ocr-police-model <path>     Police PPLCNet OCR RKNN\n"
            "  --ocr-police-keys <path>      Police keys file\n"
            "  --ocr-embassy-model <path>    Embassy PPLCNet OCR RKNN\n"
            "  --ocr-embassy-keys <path>     Embassy keys file\n"
            "  --ocr-yellow-model <path>     Yellow PPLCNet OCR RKNN\n"
            "  --ocr-yellow-keys <path>      Yellow keys file\n"
            "\n"
            "Display / capture options:\n"
            "  --device <path>               FPGA DMA device (default: /dev/fpga_dma0)\n"
            "  --input-bgrx <path>           Repeat one 1280x720 BGRx image instead of DMA\n"
            "  --source <fpga|phone>         Desired source at startup (default: fpga)\n"
            "  --phone-rtsp <uri>            MediaMTX phone RTSP URI\n"
            "  --control-socket <path|off>   Runtime JSON Unix socket\n"
            "  --drm-card <path>             DRM card (default: /dev/dri/card0)\n"
            "  --connector-id <id>           Optional KMS connector id\n"
            "  --no-display                  Disable HDMI/KMS display\n"
            "  --no-infer                    Disable RKNN inference for display-only diagnostics\n"
            "  --display-sync <0|1>          kmssink sync (default: 0)\n"
            "  --display-atomic-flip <0|1>   Force kmssink sync-mode=flip (default: 0)\n"
            "  --display-do-timestamp <0|1> appsrc do-timestamp (default: 1)\n"
            "  --frames <n>                  Frame budget; 0 = forever (default: 0)\n"
            "  --fps <n>                     Capture throttle FPS (default: 10)\n"
            "  --min-plate-conf <v>          Detector threshold (default: 0.50)\n"
            "  --det-score-scale <v>         Divide detector class scores by v (default: 1)\n"
            "  --plate-nms-iou <v>           NMS IoU (default: 0.45)\n"
            "  --plate-max-det <n>           Max dets per frame (default: 8)\n"
            "  --class-filter <id>           Filter detector class; -1 disables (default: -1)\n"
            "  --auto-green-filter <0|1>     Auto-set class 1 when pose_nc>=5 (default: 0)\n"
            "  --det-resize <stretch|letterbox>  Detector mapping (default: stretch)\n"
            "  --det-zerocopy                Zero-copy detector input (pass_through; default: off)\n"
            "  --ocr-preproc <none|gray|bin> OCR preprocess (default: gray)\n"
            "  --pixel-order <bgr565|rgb565> Raw 565 byte order (default: bgr565)\n"
            "  --swap16 <0|1>                Swap raw 565 byte halves (default: 0)\n"
            "  --dump-frames <n>             Dump first n raw BGRX frames to disk for diagnostics (default: 0)\n"
            "  --dump-path <dir>             Directory for dumped frames (default: ./dump)\n"
            "  --hash-frames <n>             Check first n raw frames for exact adjacent duplicates (default: 0)\n"
            "  --hash-full                   Use slower full-frame hash comparison instead of exact frame copy\n"
            "  --dma-pre-delay-us <n>       Sleep before each DMA read, for phase diagnostics (default: 0)\n"
            "  --display-every <n>          Display one of every n captured frames (default: 1)\n"
            "  --wait-new-frame <0|1>       Wait for FPGA frame counter before each DMA read (default: 0)\n"
            "  --frame-stamp-check <0|1>    Verify FPGA frame-identity stamp in DMA readbacks (default: 0)\n",
            prog);
}

static void defaults(struct live_options *o)
{
    memset(o, 0, sizeof(*o));
    o->device_path = DEFAULT_DEVICE;
    o->phone_rtsp_uri = DEFAULT_PHONE_RTSP;
    o->control_socket_path = DEFAULT_CONTROL_SOCKET;
    o->initial_phone_source = false;
    o->drm_card_path = DEFAULT_DRM_CARD;
    o->frames = 0;
    o->fps = 10;
    o->min_conf = 0.50f;
    o->det_score_scale = 1.0f;
    o->nms_iou = 0.45f;
    o->plate_type_classifier_min_conf = PLATE_TYPE_CLASSIFIER_DEFAULT_MIN_CONF;
    o->plate_type_classifier_special_min_conf = PLATE_TYPE_CLASSIFIER_DEFAULT_SPECIAL_MIN_CONF;
    o->max_det = 8;
    o->class_filter = -1;
    o->connector_id = -1;
    o->auto_green_filter = false;
    o->det_resize_mode = DET_RESIZE_STRETCH;
    o->ocr_preproc_mode = OCR_PREPROC_GRAY;
    o->pixel_order = PIXEL_ORDER_BGR565;
    o->swap16 = false;
    o->display = true;
    /* Favor steady 30fps motion by default. Atomic flip is available as an
     * explicit diagnostic mode for true scanout tearing, but forcing it made
     * moving objects visibly stutter on this path. */
    o->display_sync = false;
    o->display_atomic_flip = false;
    o->display_do_timestamp = true;
    o->det_zero_copy = false;
    o->no_infer = false;
    o->dump_frames = 0;
    o->dump_path = NULL;
    o->hash_frames = 0;
    o->hash_full = false;
    o->dma_pre_delay_us = 0;
    o->display_every = 1;
    o->wait_new_frame = false;
    o->frame_stamp_check = false;
}

static int parse_options(int argc, char **argv, struct live_options *o)
{
    /* Long-option ids; intentionally non-contiguous to make new options
     * easy to add without renumbering. */
    enum {
        OPT_DEVICE = 1,
        OPT_PLATE_MODEL,
        OPT_OCR_GREEN_MODEL,
        OPT_OCR_KEYS,
        OPT_FRAMES,
        OPT_FPS,
        OPT_MIN_CONF,
        OPT_NMS_IOU,
        OPT_MAX_DET,
        OPT_CLASS_FILTER,
        OPT_DET_RESIZE,
        OPT_DET_ZEROCOPY,
        OPT_OCR_PREPROC,
        OPT_PIXEL_ORDER,
        OPT_SWAP16,
        OPT_DRM_CARD,
        OPT_CONNECTOR_ID,
        OPT_NO_DISPLAY,
        OPT_DISPLAY_SYNC,
        OPT_DISPLAY_ATOMIC_FLIP,
        OPT_DISPLAY_DO_TIMESTAMP,
        OPT_AUTO_GREEN,
        OPT_OCR_BLUE_MODEL,
        OPT_OCR_POLICE_MODEL,
        OPT_OCR_POLICE_KEYS,
        OPT_OCR_BLUE_KEYS,
        OPT_OCR_GREEN_KEYS,
        OPT_OCR_EMBASSY_MODEL,
        OPT_OCR_EMBASSY_KEYS,
        OPT_OCR_YELLOW_MODEL,
        OPT_OCR_YELLOW_KEYS,
        OPT_PLATE_TYPE_CLASSIFIER_MODEL,
        OPT_PLATE_TYPE_CLASSIFIER_MIN_CONF,
        OPT_PLATE_TYPE_CLASSIFIER_SPECIAL_MIN_CONF,
        OPT_DET_SCORE_SCALE,
        OPT_DUMP_FRAMES,
        OPT_DUMP_PATH,
        OPT_HASH_FRAMES,
        OPT_HASH_FULL,
        OPT_NO_INFER,
        OPT_DMA_PRE_DELAY_US,
        OPT_DISPLAY_EVERY,
        OPT_WAIT_NEW_FRAME,
        OPT_FRAME_STAMP_CHECK,
        OPT_SOURCE,
        OPT_PHONE_RTSP,
        OPT_CONTROL_SOCKET,
        OPT_INPUT_BGRX,
    };
    static const struct option opts[] = {
        {"device",            required_argument, NULL, OPT_DEVICE},
        {"input-bgrx",        required_argument, NULL, OPT_INPUT_BGRX},
        {"source",            required_argument, NULL, OPT_SOURCE},
        {"phone-rtsp",        required_argument, NULL, OPT_PHONE_RTSP},
        {"control-socket",    required_argument, NULL, OPT_CONTROL_SOCKET},
        {"plate-model",       required_argument, NULL, OPT_PLATE_MODEL},
        {"ocr-green-model",   required_argument, NULL, OPT_OCR_GREEN_MODEL},
        {"ocr-keys",          required_argument, NULL, OPT_OCR_KEYS},
        {"frames",            required_argument, NULL, OPT_FRAMES},
        {"fps",               required_argument, NULL, OPT_FPS},
        {"min-plate-conf",    required_argument, NULL, OPT_MIN_CONF},
        {"det-score-scale",   required_argument, NULL, OPT_DET_SCORE_SCALE},
        {"plate-nms-iou",     required_argument, NULL, OPT_NMS_IOU},
        {"plate-max-det",     required_argument, NULL, OPT_MAX_DET},
        {"class-filter",      required_argument, NULL, OPT_CLASS_FILTER},
        {"det-resize",        required_argument, NULL, OPT_DET_RESIZE},
        {"det-zerocopy",      no_argument,       NULL, OPT_DET_ZEROCOPY},
        {"ocr-preproc",       required_argument, NULL, OPT_OCR_PREPROC},
        {"pixel-order",       required_argument, NULL, OPT_PIXEL_ORDER},
        {"swap16",            required_argument, NULL, OPT_SWAP16},
        {"drm-card",          required_argument, NULL, OPT_DRM_CARD},
        {"connector-id",      required_argument, NULL, OPT_CONNECTOR_ID},
        {"no-display",        no_argument,       NULL, OPT_NO_DISPLAY},
        {"no-infer",          no_argument,       NULL, OPT_NO_INFER},
        {"display-sync",      required_argument, NULL, OPT_DISPLAY_SYNC},
        {"display-atomic-flip", required_argument, NULL, OPT_DISPLAY_ATOMIC_FLIP},
        {"display-do-timestamp", required_argument, NULL, OPT_DISPLAY_DO_TIMESTAMP},
        {"auto-green-filter", required_argument, NULL, OPT_AUTO_GREEN},
        {"ocr-blue-model",    required_argument, NULL, OPT_OCR_BLUE_MODEL},
        {"ocr-police-model",  required_argument, NULL, OPT_OCR_POLICE_MODEL},
        {"ocr-police-keys",   required_argument, NULL, OPT_OCR_POLICE_KEYS},
        {"ocr-blue-keys",     required_argument, NULL, OPT_OCR_BLUE_KEYS},
        {"ocr-green-keys",    required_argument, NULL, OPT_OCR_GREEN_KEYS},
        {"ocr-embassy-model", required_argument, NULL, OPT_OCR_EMBASSY_MODEL},
        {"ocr-embassy-keys",  required_argument, NULL, OPT_OCR_EMBASSY_KEYS},
        {"ocr-yellow-model",  required_argument, NULL, OPT_OCR_YELLOW_MODEL},
        {"ocr-yellow-keys",   required_argument, NULL, OPT_OCR_YELLOW_KEYS},
        {"plate-type-classifier-model", required_argument, NULL, OPT_PLATE_TYPE_CLASSIFIER_MODEL},
        {"plate-type-classifier-min-conf", required_argument, NULL, OPT_PLATE_TYPE_CLASSIFIER_MIN_CONF},
        {"plate-type-classifier-special-min-conf", required_argument, NULL, OPT_PLATE_TYPE_CLASSIFIER_SPECIAL_MIN_CONF},
        {"dump-frames",      required_argument, NULL, OPT_DUMP_FRAMES},
        {"dump-path",        required_argument, NULL, OPT_DUMP_PATH},
        {"hash-frames",      required_argument, NULL, OPT_HASH_FRAMES},
        {"hash-full",        no_argument,       NULL, OPT_HASH_FULL},
        {"dma-pre-delay-us", required_argument, NULL, OPT_DMA_PRE_DELAY_US},
        {"display-every",    required_argument, NULL, OPT_DISPLAY_EVERY},
        {"wait-new-frame",   required_argument, NULL, OPT_WAIT_NEW_FRAME},
        {"frame-stamp-check", required_argument, NULL, OPT_FRAME_STAMP_CHECK},
        {"help",              no_argument,       NULL, 'h'},
        {0, 0, 0, 0},
    };
    int c;
    defaults(o);
    while ((c = getopt_long(argc, argv, "h", opts, NULL)) != -1) {
        switch (c) {
        case OPT_DEVICE:           o->device_path = optarg; break;
        case OPT_INPUT_BGRX:       o->input_bgrx_path = optarg; break;
        case OPT_SOURCE:
            if (strcmp(optarg, "fpga") == 0 || strcmp(optarg, "ov5640") == 0)
                o->initial_phone_source = false;
            else if (strcmp(optarg, "phone") == 0)
                o->initial_phone_source = true;
            else
                return -1;
            break;
        case OPT_PHONE_RTSP:
            o->phone_rtsp_uri = optarg;
            break;
        case OPT_CONTROL_SOCKET:
            if (strcmp(optarg, "off") == 0 ||
                strcmp(optarg, "none") == 0)
                o->control_socket_path = NULL;
            else
                o->control_socket_path = optarg;
            break;
        case OPT_PLATE_MODEL:      o->plate_model_path = optarg; break;
        case OPT_OCR_GREEN_MODEL:  o->ocr_green_model_path = optarg; break;
        case OPT_OCR_KEYS:         /* shared keys path used as default for blue+green */
            if (!o->keys_blue_path)  o->keys_blue_path  = optarg;
            if (!o->keys_green_path) o->keys_green_path = optarg;
            break;
        case OPT_FRAMES:           o->frames = atoi(optarg); break;
        case OPT_FPS:              o->fps = atoi(optarg); break;
        case OPT_MIN_CONF:         o->min_conf = strtof(optarg, NULL); break;
        case OPT_DET_SCORE_SCALE:  o->det_score_scale = strtof(optarg, NULL); break;
        case OPT_NMS_IOU:          o->nms_iou = strtof(optarg, NULL); break;
        case OPT_MAX_DET:          o->max_det = atoi(optarg); break;
        case OPT_CLASS_FILTER:
            o->class_filter = atoi(optarg);
            o->auto_green_filter = false;
            break;
        case OPT_DET_RESIZE:
            if (strcmp(optarg, "stretch") == 0) o->det_resize_mode = DET_RESIZE_STRETCH;
            else if (strcmp(optarg, "letterbox") == 0) o->det_resize_mode = DET_RESIZE_LETTERBOX;
            else return -1;
            break;
        case OPT_DET_ZEROCOPY:
            o->det_zero_copy = true;
            break;
        case OPT_OCR_PREPROC:
            if (strcmp(optarg, "none") == 0) o->ocr_preproc_mode = OCR_PREPROC_NONE;
            else if (strcmp(optarg, "gray") == 0 || strcmp(optarg, "gray3") == 0) o->ocr_preproc_mode = OCR_PREPROC_GRAY;
            else if (strcmp(optarg, "bin") == 0) o->ocr_preproc_mode = OCR_PREPROC_BIN;
            else return -1;
            break;
        case OPT_PIXEL_ORDER:
            if (strcmp(optarg, "bgr565") == 0) o->pixel_order = PIXEL_ORDER_BGR565;
            else if (strcmp(optarg, "rgb565") == 0) o->pixel_order = PIXEL_ORDER_RGB565;
            else return -1;
            break;
        case OPT_SWAP16:
            o->swap16 = (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0 || strcmp(optarg, "on") == 0);
            break;
        case OPT_DRM_CARD:        o->drm_card_path = optarg; break;
        case OPT_CONNECTOR_ID:    o->connector_id = atoi(optarg); break;
        case OPT_NO_DISPLAY:      o->display = false; break;
        case OPT_NO_INFER:        o->no_infer = true; break;
        case OPT_DISPLAY_SYNC:
            o->display_sync = (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0 || strcmp(optarg, "on") == 0);
            break;
        case OPT_DISPLAY_ATOMIC_FLIP:
            o->display_atomic_flip = (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0 || strcmp(optarg, "on") == 0);
            break;
        case OPT_DISPLAY_DO_TIMESTAMP:
            o->display_do_timestamp = (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0 || strcmp(optarg, "on") == 0);
            break;
        case OPT_AUTO_GREEN:
            o->auto_green_filter = (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0 || strcmp(optarg, "on") == 0);
            break;
        case OPT_OCR_BLUE_MODEL:   o->ocr_blue_model_path = optarg; break;
        case OPT_OCR_POLICE_MODEL:  o->ocr_police_model_path = optarg; break;
        case OPT_OCR_POLICE_KEYS:   o->keys_police_path = optarg; break;
        case OPT_OCR_BLUE_KEYS:     o->keys_blue_path = optarg; break;
        case OPT_OCR_GREEN_KEYS:    o->keys_green_path = optarg; break;
        case OPT_OCR_EMBASSY_MODEL: o->ocr_embassy_model_path = optarg; break;
        case OPT_OCR_EMBASSY_KEYS:  o->keys_embassy_path = optarg; break;
        case OPT_OCR_YELLOW_MODEL:  o->ocr_yellow_model_path = optarg; break;
        case OPT_OCR_YELLOW_KEYS:   o->keys_yellow_path = optarg; break;
        case OPT_PLATE_TYPE_CLASSIFIER_MODEL:
            if (strcmp(optarg, "off") == 0 || strcmp(optarg, "none") == 0 || strcmp(optarg, "disable") == 0)
                o->plate_type_classifier_model_path = NULL;
            else
                o->plate_type_classifier_model_path = optarg;
            break;
        case OPT_PLATE_TYPE_CLASSIFIER_MIN_CONF:
            o->plate_type_classifier_min_conf = strtof(optarg, NULL);
            break;
        case OPT_PLATE_TYPE_CLASSIFIER_SPECIAL_MIN_CONF:
            o->plate_type_classifier_special_min_conf = strtof(optarg, NULL);
            break;
        case OPT_DUMP_FRAMES:
            o->dump_frames = atoi(optarg);
            if (o->dump_frames < 0) return -1;
            break;
        case OPT_DUMP_PATH:
            o->dump_path = optarg;
            break;
        case OPT_HASH_FRAMES:
            o->hash_frames = atoi(optarg);
            if (o->hash_frames < 0) return -1;
            break;
        case OPT_HASH_FULL:
            o->hash_full = true;
            break;
        case OPT_DMA_PRE_DELAY_US:
            o->dma_pre_delay_us = atoi(optarg);
            if (o->dma_pre_delay_us < 0) return -1;
            break;
        case OPT_DISPLAY_EVERY:
            o->display_every = atoi(optarg);
            if (o->display_every <= 0) return -1;
            break;
        case OPT_WAIT_NEW_FRAME:
            o->wait_new_frame = (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0 || strcmp(optarg, "on") == 0);
            break;
        case OPT_FRAME_STAMP_CHECK:
            o->frame_stamp_check = (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0 || strcmp(optarg, "on") == 0);
            break;
        case 'h': return 1;
        default:  return -1;
        }
    }
    if (!o->no_infer) {
        if (!o->plate_model_path || !o->ocr_blue_model_path || !o->ocr_green_model_path)
            return -1;
        if (!o->keys_blue_path || !o->keys_green_path)
            return -1;
    }
    if (o->ocr_police_model_path && !o->keys_police_path) {
        fprintf(stderr, "[bgp-live] --ocr-police-model requires --ocr-police-keys\n");
        return -1;
    }
    if (o->ocr_embassy_model_path && !o->keys_embassy_path) {
        fprintf(stderr, "[bgp-live] --ocr-embassy-model requires --ocr-embassy-keys\n");
        return -1;
    }
    if (o->ocr_yellow_model_path && !o->keys_yellow_path) {
        fprintf(stderr, "[bgp-live] --ocr-yellow-model requires --ocr-yellow-keys\n");
        return -1;
    }
    if (o->plate_type_classifier_min_conf < 0.0f || o->plate_type_classifier_min_conf > 1.0f)
        return -1;
    if (o->plate_type_classifier_special_min_conf < 0.0f || o->plate_type_classifier_special_min_conf > 1.0f)
        return -1;
    if (o->det_score_scale <= 0.0f)
        return -1;
    if (o->fps <= 0 || o->fps > 120 || o->frames < 0 || o->max_det <= 0 || o->max_det > MAX_DETS)
        return -1;
    if (o->hash_frames < 0)
        return -1;
    if (o->dma_pre_delay_us < 0 || o->dma_pre_delay_us > 1000000)
        return -1;
    if (o->display_every <= 0 || o->display_every > 120)
        return -1;
    return 0;
}

int main(int argc, char **argv)
{
    struct live_options opt;
    struct lpr_fpga_source fpga_source;
    struct lpr_phone_source phone_source;
    struct lpr_frame_source *fpga;
    struct lpr_frame_source *phone;
    struct dma_state *dma;
    struct lpr_source_manager source_manager;
    struct lpr_source_health fpga_health;
    struct lpr_source_health phone_health;
    struct lpr_control control;
    struct lpr_preview preview;
    struct live_result current_result;
    struct rknn_model det_model;
    struct rknn_model ocr_blue_model;
    struct rknn_model ocr_green_model;
    struct rknn_model ocr_police_model;
    struct rknn_model ocr_embassy_model;
    struct rknn_model ocr_yellow_model;
    struct rknn_model ptype_model;
    struct ocr_keys keys_blue;
    struct ocr_keys keys_green;
    struct ocr_keys keys_police;
    struct ocr_keys keys_embassy;
    struct ocr_keys keys_yellow;
    struct display_state display;
    struct infer_state infer;
    int pose_nc;
    int class_filter;
    int ret = 1;
    int parsed;
    int64_t target_us;
    bool police_enabled;
    bool embassy_enabled;
    bool yellow_enabled;
    bool ptype_enabled;
    bool fpga_initialized = false;
    bool fpga_opened = false;
    bool phone_initialized = false;
    bool phone_opened = false;
    bool control_started = false;
    bool preview_started = false;
    bool pipeline_paused = false;
    bool current_result_available = false;
    bool dump_disabled = false;
    int hash_seen = 0;
    int hash_adjacent_dups = 0;
    int hash_current_run = 0;
    int hash_longest_run = 0;
    int64_t hash_first_us = 0;
    int64_t hash_last_us = 0;
    uint8_t *hash_prev_frame = NULL;
    uint32_t stamp_samples = 0;
    uint32_t stamp_malformed = 0;
    uint32_t stamp_torn = 0;
    uint32_t stamp_duplicates = 0;
    uint32_t stamp_backward = 0;
    uint32_t stamp_skips = 0;
    uint32_t stamp_skipped_frames = 0;
    uint8_t stamp_prev_cnt = 0;
    bool stamp_have_prev = false;
    bool camera_status_seen = false;
    uint32_t camera_frame_start = 0;
    uint32_t camera_sample_prev_counter = 0;
    uint32_t camera_sample_prev_hash = 0;
    uint32_t camera_hash_samples = 0;
    uint32_t camera_hash_adjacent_dups = 0;
    uint32_t camera_hash_current_run = 0;
    uint32_t camera_hash_longest_run = 0;
    uint32_t camera_counter_nonunit_steps = 0;
    int64_t camera_status_start_us = 0;
    struct lpr_interval_stats capture_start_intervals = {0};
    struct lpr_interval_stats dma_done_intervals = {0};
    struct lpr_interval_stats display_push_intervals = {0};
    int64_t prev_capture_start_us = 0;
    int64_t prev_dma_done_us = 0;
    int64_t prev_display_push_us = 0;
    uint64_t hash_prev = 0;
    uint64_t preview_sequence = 0;
    uint64_t last_phone_sequence = 0;
    uint64_t input_frames = 0;
    uint64_t previous_metric_input_frames = 0;
    uint64_t previous_metric_infer_frames = 0;
    int64_t metric_last_us = 0;
    double active_input_fps = 0.0;
    double active_infer_fps = 0.0;
    int64_t latest_frame_us = 0;

    parsed = parse_options(argc, argv, &opt);
    if (parsed != 0) {
        usage(argv[0]);
        return parsed > 0 ? 0 : 1;
    }

    memset(&fpga_source, 0, sizeof(fpga_source));
    memset(&phone_source, 0, sizeof(phone_source));
    memset(&source_manager, 0, sizeof(source_manager));
    memset(&fpga_health, 0, sizeof(fpga_health));
    memset(&phone_health, 0, sizeof(phone_health));
    memset(&control, 0, sizeof(control)); control.listen_fd = -1;
    memset(&preview, 0, sizeof(preview));
    memset(&current_result, 0, sizeof(current_result));
    dma = NULL;
    fpga = NULL;
    phone = NULL;
    memset(&display, 0, sizeof(display)); display.drm_fd = -1;
    memset(&infer, 0, sizeof(infer));
    memset(&det_model, 0, sizeof(det_model));
    memset(&ocr_blue_model, 0, sizeof(ocr_blue_model));
    memset(&ocr_green_model, 0, sizeof(ocr_green_model));
    memset(&ocr_police_model, 0, sizeof(ocr_police_model));
    memset(&ocr_embassy_model, 0, sizeof(ocr_embassy_model));
    memset(&ocr_yellow_model, 0, sizeof(ocr_yellow_model));
    memset(&ptype_model, 0, sizeof(ptype_model));
    memset(&keys_blue, 0, sizeof(keys_blue));
    memset(&keys_green, 0, sizeof(keys_green));
    memset(&keys_police, 0, sizeof(keys_police));
    memset(&keys_embassy, 0, sizeof(keys_embassy));
    memset(&keys_yellow, 0, sizeof(keys_yellow));

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    gst_init(NULL, NULL);

    police_enabled = (opt.ocr_police_model_path != NULL);
    embassy_enabled = (opt.ocr_embassy_model_path != NULL);
    yellow_enabled = (opt.ocr_yellow_model_path != NULL);
    ptype_enabled = (opt.plate_type_classifier_model_path != NULL);

    if (!opt.no_infer && lpr_load_keys(opt.keys_blue_path, &keys_blue) < 0) {
        fprintf(stderr, "[bgp-live] failed to load blue keys: %s\n", opt.keys_blue_path);
        goto out;
    }
    if (!opt.no_infer && lpr_load_keys(opt.keys_green_path, &keys_green) < 0) {
        fprintf(stderr, "[bgp-live] failed to load green keys: %s\n", opt.keys_green_path);
        goto out;
    }
    if (!opt.no_infer && police_enabled) {
        if (lpr_load_keys(opt.keys_police_path, &keys_police) < 0) {
            fprintf(stderr, "[bgp-live] failed to load police keys: %s\n", opt.keys_police_path);
            goto out;
        }
    }
    if (!opt.no_infer && embassy_enabled) {
        if (lpr_load_keys(opt.keys_embassy_path, &keys_embassy) < 0) {
            fprintf(stderr, "[bgp-live] failed to load embassy keys: %s\n", opt.keys_embassy_path);
            goto out;
        }
    }
    if (!opt.no_infer && yellow_enabled) {
        if (lpr_load_keys(opt.keys_yellow_path, &keys_yellow) < 0) {
            fprintf(stderr, "[bgp-live] failed to load yellow keys: %s\n", opt.keys_yellow_path);
            goto out;
        }
    }

    if (lpr_source_manager_init(
            &source_manager,
            opt.initial_phone_source ? LPR_SOURCE_PHONE : LPR_SOURCE_FPGA,
            LPR_SOURCE_FPGA, 1, lpr_mono_us()) < 0) {
        fprintf(stderr, "[bgp-live] failed to initialize source manager\n");
        goto out;
    }
    if (opt.initial_phone_source)
        source_manager.reason = LPR_SOURCE_REASON_PHONE_UNAVAILABLE;
    if (lpr_fpga_source_init(&fpga_source, &opt,
                             source_manager.source_generation) < 0) {
        fprintf(stderr, "[bgp-live] failed to initialize FPGA source\n");
        goto out;
    }
    fpga_initialized = true;
    fpga = lpr_fpga_source_as_frame_source(&fpga_source);
    if (lpr_frame_source_open(fpga) < 0) {
        fprintf(stderr, "[bgp-live] failed to open/start FPGA source: %s\n",
                opt.device_path);
        goto out;
    }
    fpga_opened = true;
    if (lpr_frame_source_start(fpga) < 0) {
        fprintf(stderr, "[bgp-live] failed to start FPGA source\n");
        goto out;
    }
    dma = lpr_fpga_source_dma(&fpga_source);
    if (!dma || dma->frame_w != LIVE_FRAME_WIDTH ||
        dma->frame_h != LIVE_FRAME_HEIGHT || dma->frame_bpp != 4U) {
        fprintf(stderr,
                "[bgp-live] FPGA source must be 1280x720 BGRx8888\n");
        goto out;
    }
    if (lpr_phone_source_init(&phone_source, opt.phone_rtsp_uri,
                              source_manager.source_generation) < 0) {
        fprintf(stderr, "[bgp-live] failed to initialize phone source\n");
        goto out;
    }
    phone_initialized = true;
    phone = lpr_phone_source_as_frame_source(&phone_source);
    if (lpr_frame_source_open(phone) < 0) {
        fprintf(stderr,
                "[bgp-live] failed to open/start phone GStreamer source\n");
        goto out;
    }
    phone_opened = true;
    if (lpr_frame_source_start(phone) < 0) {
        fprintf(stderr, "[bgp-live] failed to start phone source\n");
        goto out;
    }

    if (opt.hash_frames > 0 && !opt.hash_full) {
        hash_prev_frame = malloc(dma->frame_size);
        if (!hash_prev_frame) {
            fprintf(stderr, "[bgp-live] failed to allocate exact duplicate buffer (%zu bytes)\n", dma->frame_size);
            goto out;
        }
    }
    {
        struct fpga_frame_status status;

        if (lpr_dma_get_frame_status(dma, &status) == 0 &&
            status.camera_magic == FPGA_CAMERA_STATUS_MAGIC) {
            camera_status_seen = true;
            camera_frame_start = status.camera_frame_counter;
            camera_status_start_us = lpr_mono_us();
        }
    }
    if (lpr_display_start(&display, &opt, LIVE_FRAME_WIDTH,
                          LIVE_FRAME_HEIGHT,
                          source_manager.source_generation) < 0) {
        fprintf(stderr, "[bgp-live] failed to start display\n");
        goto out;
    }
    if (opt.control_socket_path) {
        if (lpr_control_start(&control, opt.control_socket_path) < 0)
            goto out;
        control_started = true;
    }
    if (lpr_preview_start(&preview, control_started) < 0) {
        fprintf(stderr, "[bgp-live] failed to start JPEG preview\n");
        goto out;
    }
    preview_started = true;
    if (opt.no_infer)
        goto infer_ready;
    det_model.input_zero_copy = opt.det_zero_copy;
    if (lpr_model_load(&det_model, "yolov8n_pose", opt.plate_model_path) < 0) {
        fprintf(stderr, "[bgp-live] failed to load detector: %s\n", opt.plate_model_path);
        goto out;
    }
    if (lpr_model_load(&ocr_blue_model, "pplcnet_blue", opt.ocr_blue_model_path) < 0) {
        fprintf(stderr, "[bgp-live] failed to load blue OCR: %s\n", opt.ocr_blue_model_path);
        goto out;
    }
    if (lpr_model_load(&ocr_green_model, "pplcnet_green", opt.ocr_green_model_path) < 0) {
        fprintf(stderr, "[bgp-live] failed to load green OCR: %s\n", opt.ocr_green_model_path);
        goto out;
    }
    if (!opt.no_infer && police_enabled) {
        if (lpr_model_load(&ocr_police_model, "pplcnet_police", opt.ocr_police_model_path) < 0) {
            fprintf(stderr, "[bgp-live] failed to load police OCR: %s\n", opt.ocr_police_model_path);
            goto out;
        }
    }
    if (!opt.no_infer && embassy_enabled) {
        if (lpr_model_load(&ocr_embassy_model, "pplcnet_embassy", opt.ocr_embassy_model_path) < 0) {
            fprintf(stderr, "[bgp-live] failed to load embassy OCR: %s\n", opt.ocr_embassy_model_path);
            goto out;
        }
    }
    if (!opt.no_infer && yellow_enabled) {
        if (lpr_model_load(&ocr_yellow_model, "pplcnet_yellow", opt.ocr_yellow_model_path) < 0) {
            fprintf(stderr, "[bgp-live] failed to load yellow OCR: %s\n", opt.ocr_yellow_model_path);
            goto out;
        }
    }
    if (!opt.no_infer && ptype_enabled) {
        if (lpr_model_load(&ptype_model, "plate_type_classifier", opt.plate_type_classifier_model_path) < 0) {
            fprintf(stderr, "[bgp-live] failed to load plate type classifier: %s\n",
                    opt.plate_type_classifier_model_path);
            goto out;
        }
    }

infer_ready:
    if (!opt.no_infer && (det_model.in_w != ALGO_STREAM_SIZE || det_model.in_h != ALGO_STREAM_SIZE || det_model.in_c != 3)) {
        fprintf(stderr, "[bgp-live] detector input must be 640x640x3, got %ux%ux%u\n",
                det_model.in_w, det_model.in_h, det_model.in_c);
        goto out;
    }
    if (!opt.no_infer && ptype_enabled && ptype_model.in_c != 3) {
        fprintf(stderr, "[bgp-live] plate type classifier input must have 3 channels, got %u\n", ptype_model.in_c);
        goto out;
    }
    if (!opt.no_infer && (ocr_blue_model.in_c != 3 || ocr_green_model.in_c != 3 ||
        (police_enabled && ocr_police_model.in_c != 3) ||
        (embassy_enabled && ocr_embassy_model.in_c != 3) ||
        (yellow_enabled && ocr_yellow_model.in_c != 3))) {
        fprintf(stderr,
                "[bgp-live] OCR input must have 3 channels, got blue=%u green=%u police=%u embassy=%u yellow=%u\n",
                ocr_blue_model.in_c, ocr_green_model.in_c,
                police_enabled ? ocr_police_model.in_c : 0,
                embassy_enabled ? ocr_embassy_model.in_c : 0,
                yellow_enabled ? ocr_yellow_model.in_c : 0);
        goto out;
    }

    if (!opt.no_infer) {
        lpr_ocr_log_contract("blue",  &ocr_blue_model,  &keys_blue);
    lpr_ocr_log_contract("green", &ocr_green_model, &keys_green);
    if (police_enabled)
        lpr_ocr_log_contract("police", &ocr_police_model, &keys_police);
    if (embassy_enabled)
        lpr_ocr_log_contract("embassy", &ocr_embassy_model, &keys_embassy);
    if (yellow_enabled)
        lpr_ocr_log_contract("yellow", &ocr_yellow_model, &keys_yellow);
    if (ptype_enabled) {
        fprintf(stderr,
                "[ptype] classifier enabled: model=%s min_conf=%.2f special_min_conf=%.2f input=RGB %ux%u\n",
                opt.plate_type_classifier_model_path,
                opt.plate_type_classifier_min_conf,
                opt.plate_type_classifier_special_min_conf,
                ptype_model.in_w, ptype_model.in_h);
    }
    }

    pose_nc = opt.no_infer ? 0 : lpr_detector_pose_nc(&det_model);
    class_filter = opt.class_filter;
    if (opt.auto_green_filter && pose_nc >= 5)
        class_filter = 1;

    fprintf(stderr,
            "[bgp-live] start frame=%ux%u src=%s frames=%d fps=%d pose_nc=%d class_filter=%d "
            "det_resize=%s det_score_scale=%.1f blue_ocr=%ux%u green_ocr=%ux%u police_ocr=%s embassy_ocr=%s yellow_ocr=%s "
            "ptype=%s preproc=%s display=%d display_sync=%d display_atomic_flip=%d display_do_timestamp=%d auto_green_filter=%d no_infer=%d async_infer=%d dma_pre_delay_us=%d display_every=%d wait_new_frame=%d hash_frames=%d hash_mode=%s\n",
            dma->frame_w, dma->frame_h,
            dma->src_is_bgrx ? "bgrx8888" : "bgr565",
            opt.frames, opt.fps, pose_nc, class_filter,
            opt.det_resize_mode == DET_RESIZE_LETTERBOX ? "letterbox" : "stretch",
            opt.det_score_scale,
            ocr_blue_model.in_w, ocr_blue_model.in_h,
            ocr_green_model.in_w, ocr_green_model.in_h,
            police_enabled ? "enabled" : "disabled",
            embassy_enabled ? "enabled" : "disabled",
            yellow_enabled ? "enabled" : "disabled",
            ptype_enabled ? "enabled" : "disabled",
            opt.ocr_preproc_mode == OCR_PREPROC_GRAY ? "gray" :
                (opt.ocr_preproc_mode == OCR_PREPROC_BIN ? "bin" : "none"),
            opt.display ? 1 : 0, opt.display_sync ? 1 : 0, opt.display_atomic_flip ? 1 : 0,
            opt.display_do_timestamp ? 1 : 0, opt.auto_green_filter ? 1 : 0, opt.no_infer ? 1 : 0, opt.no_infer ? 0 : 1,
            opt.dma_pre_delay_us, opt.display_every, opt.wait_new_frame ? 1 : 0, opt.hash_frames,
            opt.hash_full ? "strong-full" : "exact-adjacent");

    if (!opt.no_infer) {
    /* Build the per-route binding table for the inference thread. */
    struct lpr_route routes[LPR_ROUTE_COUNT];
    memset(routes, 0, sizeof(routes));

    routes[LPR_ROUTE_BLUE].model = &ocr_blue_model;
    routes[LPR_ROUTE_BLUE].keys = &keys_blue;
    routes[LPR_ROUTE_BLUE].decode_family = OCR_DECODE_FAMILY_NORMAL7;
    routes[LPR_ROUTE_BLUE].display_tag = 'B';
    snprintf(routes[LPR_ROUTE_BLUE].name, sizeof(routes[LPR_ROUTE_BLUE].name), "blue");

    routes[LPR_ROUTE_GREEN].model = &ocr_green_model;
    routes[LPR_ROUTE_GREEN].keys = &keys_green;
    routes[LPR_ROUTE_GREEN].decode_family = OCR_DECODE_FAMILY_GREEN8;
    routes[LPR_ROUTE_GREEN].display_tag = 'G';
    snprintf(routes[LPR_ROUTE_GREEN].name, sizeof(routes[LPR_ROUTE_GREEN].name), "green");

    if (!opt.no_infer && police_enabled) {
        routes[LPR_ROUTE_POLICE].model = &ocr_police_model;
        routes[LPR_ROUTE_POLICE].keys = &keys_police;
        routes[LPR_ROUTE_POLICE].decode_family = OCR_DECODE_FAMILY_POLICE7;
        routes[LPR_ROUTE_POLICE].display_tag = 'P';
        snprintf(routes[LPR_ROUTE_POLICE].name, sizeof(routes[LPR_ROUTE_POLICE].name), "police");
    } else {
        routes[LPR_ROUTE_POLICE].model = NULL;
    }

    if (!opt.no_infer && embassy_enabled) {
        routes[LPR_ROUTE_EMBASSY].model = &ocr_embassy_model;
        routes[LPR_ROUTE_EMBASSY].keys = &keys_embassy;
        routes[LPR_ROUTE_EMBASSY].decode_family = OCR_DECODE_FAMILY_EMBASSY7;
        routes[LPR_ROUTE_EMBASSY].display_tag = 'E';
        snprintf(routes[LPR_ROUTE_EMBASSY].name, sizeof(routes[LPR_ROUTE_EMBASSY].name), "embassy");
    } else {
        routes[LPR_ROUTE_EMBASSY].model = NULL;
    }

    if (!opt.no_infer && yellow_enabled) {
        routes[LPR_ROUTE_YELLOW].model = &ocr_yellow_model;
        routes[LPR_ROUTE_YELLOW].keys = &keys_yellow;
        routes[LPR_ROUTE_YELLOW].decode_family = OCR_DECODE_FAMILY_YELLOW7;
        routes[LPR_ROUTE_YELLOW].display_tag = 'Y';
        snprintf(routes[LPR_ROUTE_YELLOW].name, sizeof(routes[LPR_ROUTE_YELLOW].name), "yellow");
    } else {
        routes[LPR_ROUTE_YELLOW].model = NULL;
    }

    if (lpr_infer_start(&infer, &opt, &det_model,
                        ptype_enabled ? &ptype_model : NULL, routes,
                        pose_nc, class_filter,
                        (int)LIVE_FRAME_WIDTH, (int)LIVE_FRAME_HEIGHT,
                        source_manager.source_generation) < 0) {
        fprintf(stderr, "[bgp-live] failed to start infer thread\n");
        goto out;
    }

    }

    target_us = 1000000LL / opt.fps;
    int64_t next_frame_us = lpr_mono_us();
    int64_t stat_last_us = lpr_mono_us();
    int64_t stat_dma_us = 0, stat_overlay_us = 0, stat_push_us = 0, stat_sleep_us = 0;
    uint64_t stat_display_drop = display.dropped_frames;
    int stat_frames = 0;
    int frame = 0;
    metric_last_us = stat_last_us;
    while (!g_stop && (opt.frames == 0 || frame < opt.frames)) {
        struct lpr_frame_ref phone_frame = {0};
        struct lpr_frame_ref active_frame = {0};
        struct live_result latest;
        struct lpr_source_switch_event switch_event;
        enum lpr_source_id active_before;
        enum lpr_control_source requested_source;
        enum lpr_pipeline_command pipeline_command;
        const uint8_t *slot_frame = NULL;
        size_t frame_size = (size_t)LIVE_FRAME_WIDTH *
                            LIVE_FRAME_HEIGHT * 4U;
        bool clear_queues = false;
        uint64_t generation_before;
        uint64_t infer_total = 0;
        uint64_t infer_dropped = 0;
        int phone_read_result;
        int source_result;
        int64_t ts_a, ts_b, ts_c, ts_d;

        ts_a = lpr_mono_us();
        if (prev_capture_start_us > 0)
            lpr_interval_stats_update(&capture_start_intervals, ts_a - prev_capture_start_us);
        prev_capture_start_us = ts_a;

        phone_read_result = lpr_frame_source_read_latest(phone, &phone_frame);
        if (phone_read_result < 0 && phone_read_result != -EAGAIN &&
            phone_read_result != -ESHUTDOWN) {
            fprintf(stderr, "[bgp-live] phone latest-frame read failed: %s\n",
                    strerror(-phone_read_result));
        }
        lpr_frame_source_health(phone, ts_a, &phone_health);
        lpr_frame_source_health(fpga, ts_a, &fpga_health);

        generation_before = source_manager.source_generation;
        active_before = source_manager.active;
        requested_source = control_started ?
            lpr_control_take_source(&control) : LPR_CONTROL_SOURCE_NONE;
        if (requested_source != LPR_CONTROL_SOURCE_NONE) {
            enum lpr_source_id desired =
                requested_source == LPR_CONTROL_SOURCE_PHONE ?
                    LPR_SOURCE_PHONE : LPR_SOURCE_FPGA;

            source_result = lpr_source_manager_set_desired(
                &source_manager, desired, &phone_health, ts_a,
                &switch_event);
            if (source_result < 0) {
                fprintf(stderr, "[bgp-live] invalid source switch request\n");
                lpr_frame_ref_release(&phone_frame);
                goto out;
            }
        }

        pipeline_command = control_started ?
            lpr_control_take_pipeline_command(&control) :
            LPR_PIPELINE_COMMAND_NONE;
        if (pipeline_command == LPR_PIPELINE_COMMAND_PAUSE) {
            if (lpr_source_manager_advance_generation(&source_manager,
                                                      ts_a) < 0) {
                fprintf(stderr,
                        "[bgp-live] failed to advance generation for pause\n");
                lpr_frame_ref_release(&phone_frame);
                goto out;
            }
            pipeline_paused = true;
            clear_queues = true;
        } else if (pipeline_command == LPR_PIPELINE_COMMAND_RESUME) {
            pipeline_paused = false;
        } else if (pipeline_command == LPR_PIPELINE_COMMAND_RESTART) {
            lpr_frame_source_stop(phone);
            lpr_frame_source_stop(fpga);
            if (lpr_frame_source_start(fpga) < 0 ||
                lpr_frame_source_start(phone) < 0) {
                fprintf(stderr, "[bgp-live] source restart failed\n");
                lpr_frame_ref_release(&phone_frame);
                goto out;
            }
            if (lpr_source_manager_restart(&source_manager, ts_a) < 0) {
                fprintf(stderr,
                        "[bgp-live] failed to advance generation for restart\n");
                lpr_frame_ref_release(&phone_frame);
                goto out;
            }
            pipeline_paused = false;
            clear_queues = true;
        }

        source_result = lpr_source_manager_update(
            &source_manager, &phone_health, ts_a, &switch_event);
        if (source_result < 0) {
            fprintf(stderr, "[bgp-live] source state update failed\n");
            lpr_frame_ref_release(&phone_frame);
            goto out;
        }

        if (source_manager.source_generation != generation_before) {
            if (source_manager.active != active_before) {
                fprintf(stderr,
                        "[bgp-live] source switch %s -> %s reason=%s generation=%llu\n",
                        lpr_source_id_string(active_before),
                        lpr_source_id_string(source_manager.active),
                        lpr_source_reason_string(source_manager.reason),
                        (unsigned long long)source_manager.source_generation);
            } else {
                const char *action =
                    pipeline_command == LPR_PIPELINE_COMMAND_PAUSE ?
                        "pause" :
                    pipeline_command == LPR_PIPELINE_COMMAND_RESTART ?
                        "restart" : "epoch";

                fprintf(stderr,
                        "[bgp-live] pipeline %s active=%s reason=%s generation=%llu\n",
                        action,
                        lpr_source_id_string(source_manager.active),
                        lpr_source_reason_string(source_manager.reason),
                        (unsigned long long)source_manager.source_generation);
            }
            lpr_fpga_source_set_generation(
                &fpga_source, source_manager.source_generation);
            lpr_phone_source_set_generation(
                &phone_source, source_manager.source_generation);
            if (!opt.no_infer)
                lpr_infer_reset(&infer,
                                source_manager.source_generation);
            lpr_display_reset(&display,
                              source_manager.source_generation);
            memset(&current_result, 0, sizeof(current_result));
            current_result.source_generation =
                source_manager.source_generation;
            current_result_available = false;
            last_phone_sequence = 0;
            latest_frame_us = 0;
            if (preview_started) {
                if (lpr_preview_reset(&preview) < 0)
                    fprintf(stderr, "[bgp-live] preview reset failed\n");
                preview_sequence = 0;
            }
            if (control_started) {
                lpr_control_update_results(
                    &control, NULL,
                    source_manager.source_generation);
                lpr_control_clear_jpeg(&control);
            }
        } else if (clear_queues) {
            if (!opt.no_infer)
                lpr_infer_reset(&infer,
                                source_manager.source_generation);
            lpr_display_reset(&display,
                              source_manager.source_generation);
            memset(&current_result, 0, sizeof(current_result));
            current_result.source_generation =
                source_manager.source_generation;
            current_result_available = false;
            latest_frame_us = 0;
            if (preview_started) {
                if (lpr_preview_reset(&preview) < 0)
                    fprintf(stderr, "[bgp-live] preview reset failed\n");
                preview_sequence = 0;
            }
            if (control_started) {
                lpr_control_update_results(
                    &control, NULL,
                    source_manager.source_generation);
                lpr_control_clear_jpeg(&control);
            }
        }

        if (pipeline_paused)
            goto loop_status;

        if (source_manager.active == LPR_SOURCE_PHONE) {
            if (phone_read_result == 0 &&
                phone_frame.meta.source_generation ==
                    source_manager.source_generation &&
                phone_frame.meta.sequence != last_phone_sequence) {
                active_frame = phone_frame;
                memset(&phone_frame, 0, sizeof(phone_frame));
                last_phone_sequence = active_frame.meta.sequence;
            } else {
                goto loop_status;
            }
        } else {
            source_result =
                lpr_frame_source_read_latest(fpga, &active_frame);
            if (source_result < 0) {
                fprintf(stderr, "[bgp-live] FPGA frame read failed: %s\n",
                        strerror(-source_result));
                lpr_frame_ref_release(&phone_frame);
                lpr_frame_ref_release(&active_frame);
                goto out;
            }
        }

        slot_frame = lpr_frame_ref_data(&active_frame);
        if (!slot_frame ||
            active_frame.meta.format != LPR_FRAME_FORMAT_BGRX8888 ||
            active_frame.meta.width != LIVE_FRAME_WIDTH ||
            active_frame.meta.height != LIVE_FRAME_HEIGHT ||
            active_frame.meta.stride != LIVE_FRAME_WIDTH * 4U) {
            fprintf(stderr, "[bgp-live] active source returned invalid frame\n");
            lpr_frame_ref_release(&phone_frame);
            lpr_frame_ref_release(&active_frame);
            goto out;
        }
        ts_b = lpr_mono_us();
        input_frames++;
        latest_frame_us = active_frame.meta.monotonic_us;
        if (prev_dma_done_us > 0)
            lpr_interval_stats_update(&dma_done_intervals, ts_b - prev_dma_done_us);
        prev_dma_done_us = ts_b;

        if (opt.hash_frames > 0 && frame < opt.hash_frames) {
            uint64_t h = 0;
            bool duplicate = false;
            if (opt.hash_full)
                h = lpr_frame_hash64_full(slot_frame, frame_size);

            hash_seen++;
            if (hash_seen == 1) {
                hash_first_us = ts_b;
                hash_current_run = 1;
            } else {
                if (opt.hash_full)
                    duplicate = (h == hash_prev);
                else
                    duplicate = (memcmp(hash_prev_frame, slot_frame,
                                        frame_size) == 0);

                if (duplicate) {
                    hash_adjacent_dups++;
                    hash_current_run++;
                    if (!opt.hash_full)
                        h = lpr_frame_fingerprint64_fast(slot_frame,
                                                         frame_size);
                    fprintf(stderr,
                            "[bgp-live] frame-hash duplicate mode=%s prev=%d frame=%d hash=0x%016llx\n",
                            opt.hash_full ? "strong-full" : "exact-adjacent", frame - 1, frame, (unsigned long long)h);
                } else {
                    if (hash_current_run > hash_longest_run)
                        hash_longest_run = hash_current_run;
                    hash_current_run = 1;
                }
            }
            if (opt.hash_full)
                hash_prev = h;
            else
                memcpy(hash_prev_frame, slot_frame, frame_size);
            hash_last_us = ts_b;
        }

        if (opt.frame_stamp_check &&
            (active_frame.meta.fpga_caps &
             LPR_FRAME_FPGA_CAP_FRAME_STAMP)) {
            uint8_t head_cnt = 0, tail_cnt = 0;
            int head_ok = frame_stamp_extract(
                slot_frame, LIVE_FRAME_WIDTH, 0, &head_cnt);
            int tail_ok = frame_stamp_extract(
                slot_frame, LIVE_FRAME_WIDTH,
                LIVE_FRAME_HEIGHT - 1U, &tail_cnt);

            stamp_samples++;
            if (head_ok < 0 || tail_ok < 0) {
                stamp_malformed++;
                if (stamp_malformed <= 8)
                    fprintf(stderr, "[bgp-live] frame-stamp malformed frame=%d head_ok=%d tail_ok=%d "
                            "(is the stamp bitstream loaded?)\n", frame, head_ok >= 0, tail_ok >= 0);
            } else {
                if (head_cnt != tail_cnt) {
                    stamp_torn++;
                    if (stamp_torn <= 12)
                        fprintf(stderr, "[bgp-live] frame-stamp TORN frame=%d head=%u tail=%u\n",
                                frame, head_cnt, tail_cnt);
                }
                if (stamp_have_prev) {
                    uint8_t delta = (uint8_t)(head_cnt - stamp_prev_cnt);
                    if (delta == 0) {
                        stamp_duplicates++;
                        if (stamp_duplicates <= 12)
                            fprintf(stderr, "[bgp-live] frame-stamp DUPLICATE frame=%d cnt=%u\n",
                                    frame, head_cnt);
                    } else if (delta >= 128) {
                        stamp_backward++;
                        if (stamp_backward <= 12)
                            fprintf(stderr, "[bgp-live] frame-stamp BACKWARD frame=%d prev=%u now=%u\n",
                                    frame, stamp_prev_cnt, head_cnt);
                    } else if (delta > 1) {
                        stamp_skips++;
                        stamp_skipped_frames += delta - 1;
                    }
                }
                stamp_prev_cnt = head_cnt;
                stamp_have_prev = true;
            }
        }

        if (!dump_disabled && opt.dump_frames > 0 && frame < opt.dump_frames) {
            const char *dp = opt.dump_path ? opt.dump_path : "dump";
            char path[512];
            int dfd;
            mkdir(dp, 0755); /* best-effort; ignore EEXIST */
            snprintf(path, sizeof(path), "%s/frame_%04d.bgrx", dp, frame);
            dfd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
            if (dfd < 0) {
                fprintf(stderr, "[bgp-live] dump open failed: %s: %s; disabling dump\n",
                        path, strerror(errno));
                dump_disabled = true;
            } else {
                size_t off = 0;
                bool dump_ok = true;
                int dump_errno = 0;

                while (off < frame_size) {
                    ssize_t w = write(dfd, slot_frame + off,
                                      frame_size - off);
                    if (w < 0) {
                        dump_ok = false;
                        dump_errno = errno;
                        break;
                    }
                    if (w == 0) {
                        dump_ok = false;
                        dump_errno = ENOSPC;
                        break;
                    }
                    off += (size_t)w;
                }
                if (close(dfd) < 0 && dump_ok) {
                    dump_ok = false;
                    dump_errno = errno;
                }

                if (dump_ok && off == frame_size) {
                    fprintf(stderr, "[bgp-live] dumped %s (%zu bytes)\n", path, off);
                } else {
                    if (dump_errno == 0)
                        dump_errno = EIO;
                    fprintf(stderr,
                            "[bgp-live] dump short write: %s wrote=%zu expected=%zu error=%s; disabling dump\n",
                            path, off, frame_size, strerror(dump_errno));
                    unlink(path);
                    dump_disabled = true;
                }
            }
        }

        if (opt.wait_new_frame &&
            source_manager.active == LPR_SOURCE_FPGA &&
            camera_status_seen) {
            struct fpga_frame_status status;

            if (lpr_dma_get_frame_status(dma, &status) == 0 &&
                status.camera_magic == FPGA_CAMERA_STATUS_MAGIC) {
                if (camera_hash_samples == 0) {
                    camera_hash_current_run = 1;
                } else {
                    uint32_t counter_step =
                        status.camera_frame_counter -
                        camera_sample_prev_counter;

                    if (counter_step != 1U)
                        camera_counter_nonunit_steps++;
                    if (status.camera_hash ==
                        camera_sample_prev_hash) {
                        camera_hash_adjacent_dups++;
                        camera_hash_current_run++;
                    } else {
                        if (camera_hash_current_run >
                            camera_hash_longest_run)
                            camera_hash_longest_run =
                                camera_hash_current_run;
                        camera_hash_current_run = 1;
                    }
                }
                camera_sample_prev_counter =
                    status.camera_frame_counter;
                camera_sample_prev_hash = status.camera_hash;
                camera_hash_samples++;
            }
        }

        if (!opt.no_infer) {
            if (lpr_infer_submit_latest(&infer, &active_frame) < 0) {
                fprintf(stderr, "[bgp-live] inference submit rejected\n");
                lpr_frame_ref_release(&phone_frame);
                lpr_frame_ref_release(&active_frame);
                goto out;
            }
            memset(&latest, 0, sizeof(latest));
            lpr_infer_get_result(&infer, &latest);
            if (lpr_source_result_is_current(
                    &source_manager, latest.source_generation)) {
                current_result = latest;
                current_result_available = latest.valid;
            }
        }
        ts_c = lpr_mono_us();
        stat_dma_us += ts_b - ts_a;
        stat_overlay_us += ts_c - ts_b;

        if (preview_started &&
            lpr_preview_push(&preview, &active_frame) < 0) {
            fprintf(stderr, "[bgp-live] preview push failed\n");
            lpr_frame_ref_release(&phone_frame);
            lpr_frame_ref_release(&active_frame);
            goto out;
        }
        if (opt.display && (frame % opt.display_every) == 0) {
            if (lpr_display_push_frame(
                    &display, &active_frame,
                    current_result_available ? &current_result : NULL) < 0) {
                lpr_frame_ref_release(&phone_frame);
                lpr_frame_ref_release(&active_frame);
                goto out;
            }
            ts_d = lpr_mono_us();
            stat_push_us += ts_d - ts_c;
            if (prev_display_push_us > 0)
                lpr_interval_stats_update(&display_push_intervals, ts_d - prev_display_push_us);
            prev_display_push_us = ts_d;
        } else {
            ts_d = ts_c;
        }

        lpr_frame_ref_release(&active_frame);
        stat_frames++;
        frame++;

loop_status:
        lpr_frame_ref_release(&phone_frame);
        lpr_frame_ref_release(&active_frame);
        {
            uint8_t *jpeg = NULL;
            size_t jpeg_size = 0;
            uint64_t jpeg_sequence = 0;

            if (preview_started && control_started &&
                lpr_preview_snapshot(
                    &preview, preview_sequence, &jpeg, &jpeg_size,
                    &jpeg_sequence) == 0) {
                lpr_control_update_jpeg(
                    &control, jpeg, jpeg_size, jpeg_sequence,
                    source_manager.source_generation);
                preview_sequence = jpeg_sequence;
                free(jpeg);
            }
        }
        {
            int64_t now = lpr_mono_us();

            lpr_frame_source_health(phone, now, &phone_health);
            lpr_frame_source_health(fpga, now, &fpga_health);
            if (!opt.no_infer) {
                pthread_mutex_lock(&infer.lock);
                infer_total = infer.infer_count;
                infer_dropped = infer.overwrite_count;
                pthread_mutex_unlock(&infer.lock);
            }
            if (now - metric_last_us >= 1000000LL) {
                double elapsed =
                    (double)(now - metric_last_us) / 1000000.0;

                active_input_fps =
                    (double)(input_frames -
                             previous_metric_input_frames) / elapsed;
                active_infer_fps =
                    (double)(infer_total -
                             previous_metric_infer_frames) / elapsed;
                previous_metric_input_frames = input_frames;
                previous_metric_infer_frames = infer_total;
                metric_last_us = now;
            }
            if (control_started) {
                const struct lpr_source_health *active_health =
                    source_manager.active == LPR_SOURCE_PHONE ?
                        &phone_health : &fpga_health;
                struct lpr_runtime_status status;

                memset(&status, 0, sizeof(status));
                status.desired_source =
                    lpr_source_id_string(source_manager.desired);
                status.active_source =
                    lpr_source_id_string(source_manager.active);
                status.failover_reason =
                    lpr_source_reason_string(source_manager.reason);
                status.source_generation =
                    source_manager.source_generation;
                status.paused = pipeline_paused;
                status.fpga_healthy = fpga_health.healthy;
                status.phone_healthy =
                    lpr_source_phone_is_fresh(
                        &phone_health, now);
                status.width = LIVE_FRAME_WIDTH;
                status.height = LIVE_FRAME_HEIGHT;
                status.input_fps = active_input_fps;
                status.decode_fps =
                    source_manager.active == LPR_SOURCE_PHONE ?
                        phone_health.input_fps : 0.0;
                status.infer_fps = active_infer_fps;
                status.frame_age_ms =
                    latest_frame_us > 0 && now >= latest_frame_us ?
                        (double)(now - latest_frame_us) / 1000.0 :
                        -1.0;
                status.input_frames = input_frames;
                status.decoded_frames = phone_health.sequence;
                status.inferred_frames = infer_total;
                status.input_dropped =
                    active_health->dropped_frames;
                status.decode_dropped =
                    phone_health.dropped_frames;
                status.infer_dropped = infer_dropped;
                status.display_dropped =
                    display.dropped_frames;
                lpr_control_update_status(&control, &status);
                lpr_control_update_results(
                    &control,
                    current_result.source_generation ==
                            source_manager.source_generation ?
                        &current_result : NULL,
                    source_manager.source_generation);
            }

            next_frame_us += target_us;
            if (now > next_frame_us + target_us)
                next_frame_us = now + target_us;
            if (next_frame_us > now) {
                int64_t pre = lpr_mono_us();
                struct timespec sleep_until;

                sleep_until.tv_sec =
                    (time_t)(next_frame_us / 1000000LL);
                sleep_until.tv_nsec =
                    (long)((next_frame_us % 1000000LL) * 1000LL);
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                &sleep_until, NULL);
                stat_sleep_us += lpr_mono_us() - pre;
            }
        }
        int64_t now = lpr_mono_us();
        if (now - stat_last_us >= 1000000LL) {
            int64_t total = now - stat_last_us;
            if (stat_frames > 0) {
                fprintf(stderr,
                        "[bgp-live] cadence: source=%s frames=%d total=%.0fms capture=%.1fms submit=%.1fms "
                        "push=%.1fms sleep=%.1fms display_drop=%llu (per-frame avg)\n",
                        lpr_source_id_string(source_manager.active),
                        stat_frames, total / 1000.0,
                        (double)stat_dma_us / stat_frames / 1000.0,
                        (double)stat_overlay_us / stat_frames / 1000.0,
                        (double)stat_push_us / stat_frames / 1000.0,
                        (double)stat_sleep_us / stat_frames / 1000.0,
                        (unsigned long long)
                            (display.dropped_frames -
                             stat_display_drop));
            }
            stat_last_us = now;
            stat_display_drop = display.dropped_frames;
            stat_dma_us = stat_overlay_us = stat_push_us = stat_sleep_us = 0;
            stat_frames = 0;
        }
    }
    ret = 0;

out:
    if (camera_status_seen && dma && dma->fd >= 0) {
        struct fpga_frame_status status;
        if (lpr_dma_get_frame_status(dma, &status) == 0 &&
            status.camera_magic == FPGA_CAMERA_STATUS_MAGIC) {
            uint32_t camera_delta = status.camera_frame_counter - camera_frame_start;
            int64_t camera_elapsed_us = lpr_mono_us() - camera_status_start_us;
            double camera_elapsed_ms = (double)camera_elapsed_us / 1000.0;
            double camera_fps = 0.0;

            if (camera_elapsed_us > 0)
                camera_fps = (double)camera_delta * 1000000.0 / (double)camera_elapsed_us;
            fprintf(stderr,
                    "[bgp-live] camera-status summary: frames_delta=%u elapsed_ms=%.1f camera_fps=%.2f last_lines=%u last_words=%u last_hash=0x%08x\n",
                    camera_delta,
                    camera_elapsed_ms,
                    camera_fps,
                    lpr_camera_status_lines(&status),
                    lpr_camera_status_words(&status),
                    status.camera_hash);
        } else {
            fprintf(stderr, "[bgp-live] camera-status summary unavailable\n");
        }
        if (camera_hash_samples > 0) {
            double camera_hash_dup_pct = 0.0;

            if (camera_hash_current_run > camera_hash_longest_run)
                camera_hash_longest_run = camera_hash_current_run;
            if (camera_hash_samples > 1)
                camera_hash_dup_pct = (double)camera_hash_adjacent_dups * 100.0 /
                                      (double)(camera_hash_samples - 1U);
            fprintf(stderr,
                    "[bgp-live] camera-hash summary: samples=%u adjacent_duplicates=%u longest_run=%u counter_nonunit_steps=%u duplicate_ratio=%.1f%%\n",
                    camera_hash_samples,
                    camera_hash_adjacent_dups,
                    camera_hash_longest_run,
                    camera_counter_nonunit_steps,
                    camera_hash_dup_pct);
        }
    }
    lpr_interval_stats_print("capture-start", &capture_start_intervals);
    lpr_interval_stats_print("dma-done", &dma_done_intervals);
    lpr_interval_stats_print("display-push", &display_push_intervals);
    if (hash_seen > 0) {
        int hash_unique_min = hash_seen - hash_adjacent_dups;
        double hash_elapsed_ms = 0.0;
        double hash_read_fps = 0.0;
        double hash_unique_fps = 0.0;
        double hash_dup_pct = 0.0;
        if (hash_current_run > hash_longest_run)
            hash_longest_run = hash_current_run;
        if (hash_seen > 1 && hash_last_us > hash_first_us) {
            hash_elapsed_ms = (double)(hash_last_us - hash_first_us) / 1000.0;
            hash_read_fps = (double)(hash_seen - 1) * 1000.0 / hash_elapsed_ms;
            if (hash_unique_min > 1)
                hash_unique_fps = (double)(hash_unique_min - 1) * 1000.0 / hash_elapsed_ms;
            hash_dup_pct = (double)hash_adjacent_dups * 100.0 / (double)(hash_seen - 1);
        }
        fprintf(stderr,
                "[bgp-live] frame-hash summary: mode=%s frames=%d adjacent_duplicates=%d longest_run=%d effective_unique_min=%d elapsed_ms=%.1f read_fps=%.2f effective_unique_fps=%.2f duplicate_ratio=%.1f%%\n",
                opt.hash_full ? "strong-full" : "exact-adjacent", hash_seen, hash_adjacent_dups, hash_longest_run,
                hash_unique_min, hash_elapsed_ms, hash_read_fps, hash_unique_fps, hash_dup_pct);
    }
    if (opt.frame_stamp_check && stamp_samples > 0) {
        uint32_t stamp_valid = stamp_samples - stamp_malformed;
        double dup_pct = 0.0, torn_pct = 0.0;

        if (stamp_valid > 1) {
            dup_pct = (double)stamp_duplicates * 100.0 / (double)(stamp_valid - 1);
            torn_pct = (double)stamp_torn * 100.0 / (double)stamp_valid;
        }
        fprintf(stderr,
                "[bgp-live] frame-stamp summary: samples=%u malformed=%u torn=%u (%.1f%%) "
                "duplicates=%u (%.1f%%) backward=%u skips=%u skipped_frames=%u\n",
                stamp_samples, stamp_malformed, stamp_torn, torn_pct,
                stamp_duplicates, dup_pct, stamp_backward, stamp_skips, stamp_skipped_frames);
    }
    free(hash_prev_frame);
    if (control_started)
        lpr_control_stop(&control);
    if (preview_started)
        lpr_preview_stop(&preview);
    if (!opt.no_infer)
        lpr_infer_stop(&infer);
    lpr_display_stop(&display);
    if (phone_initialized) {
        if (phone_opened)
            lpr_frame_source_stop(phone);
        lpr_frame_source_close(phone);
    }
    if (fpga_initialized) {
        if (fpga_opened)
            lpr_frame_source_stop(fpga);
        lpr_frame_source_close(fpga);
    }
    if (!opt.no_infer) {
        lpr_model_release(&det_model);
        lpr_model_release(&ocr_blue_model);
        lpr_model_release(&ocr_green_model);
    }
    if (!opt.no_infer && police_enabled)
        lpr_model_release(&ocr_police_model);
    if (!opt.no_infer && embassy_enabled)
        lpr_model_release(&ocr_embassy_model);
    if (!opt.no_infer && yellow_enabled)
        lpr_model_release(&ocr_yellow_model);
    if (!opt.no_infer && ptype_enabled)
        lpr_model_release(&ptype_model);
    return ret;
}
