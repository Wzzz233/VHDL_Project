// SPDX-License-Identifier: GPL-2.0
/*
 * Standalone blue/green/police/embassy/yellow PPLCNet live validation driver.
 *
 * Pipeline:
 *   FPGA DMA frame -> YOLOv8n-pose plate quad -> color route ->
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
 *   lpr_color   : RGB body color classifier
 *   lpr_ocr     : PPLCNet CTC OCR (input prep, layout autodetect, decode)
 *   lpr_display : DRM/KMS RGB16 output + overlay drawing
 *   lpr_infer   : background inference thread, route selection
 *
 * This file (pplcnet_bgp_live.c) is the main entry point: option parsing,
 * model loading, DMA pump loop and shutdown.
 */

#include "lpr_live/lpr_common.h"
#include "lpr_live/lpr_color.h"
#include "lpr_live/lpr_detector.h"
#include "lpr_live/lpr_display.h"
#include "lpr_live/lpr_dma.h"
#include "lpr_live/lpr_infer.h"
#include "lpr_live/lpr_ocr.h"
#include "lpr_live/lpr_warp.h"
#include "ocr_decode.h"
#include "pcie_fpga_dma.h"

#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <gst/gst.h>

#define DEFAULT_DEVICE   "/dev/" FPGA_DMA_DEV_NAME
#define DEFAULT_DRM_CARD "/dev/dri/card0"

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
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
            "  --drm-card <path>             DRM card (default: /dev/dri/card0)\n"
            "  --connector-id <id>           Optional KMS connector id\n"
            "  --no-display                  Disable HDMI/KMS display\n"
            "  --display-sync <0|1>          kmssink sync (default: 0)\n"
            "  --frames <n>                  Frame budget; 0 = forever (default: 0)\n"
            "  --fps <n>                     Capture throttle FPS (default: 10)\n"
            "  --min-plate-conf <v>          Detector threshold (default: 0.50)\n"
            "  --plate-nms-iou <v>           NMS IoU (default: 0.45)\n"
            "  --plate-max-det <n>           Max dets per frame (default: 8)\n"
            "  --class-filter <id>           Filter detector class; -1 disables (default: -1)\n"
            "  --auto-green-filter <0|1>     Auto-set class 1 when pose_nc>=5 (default: 0)\n"
            "  --det-resize <stretch|letterbox>  Detector mapping (default: stretch)\n"
            "  --ocr-preproc <none|gray|bin> OCR preprocess (default: gray)\n"
            "  --pixel-order <bgr565|rgb565> Raw 565 byte order (default: bgr565)\n"
            "  --swap16 <0|1>                Swap raw 565 byte halves (default: 0)\n",
            prog);
}

static void defaults(struct live_options *o)
{
    memset(o, 0, sizeof(*o));
    o->device_path = DEFAULT_DEVICE;
    o->drm_card_path = DEFAULT_DRM_CARD;
    o->frames = 0;
    o->fps = 10;
    o->min_conf = 0.50f;
    o->nms_iou = 0.45f;
    o->max_det = 8;
    o->class_filter = -1;
    o->connector_id = -1;
    o->auto_green_filter = false;
    o->det_resize_mode = DET_RESIZE_STRETCH;
    o->ocr_preproc_mode = OCR_PREPROC_GRAY;
    o->pixel_order = PIXEL_ORDER_BGR565;
    o->swap16 = false;
    o->display = true;
    o->display_sync = false;
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
        OPT_OCR_PREPROC,
        OPT_PIXEL_ORDER,
        OPT_SWAP16,
        OPT_DRM_CARD,
        OPT_CONNECTOR_ID,
        OPT_NO_DISPLAY,
        OPT_DISPLAY_SYNC,
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
    };
    static const struct option opts[] = {
        {"device",            required_argument, NULL, OPT_DEVICE},
        {"plate-model",       required_argument, NULL, OPT_PLATE_MODEL},
        {"ocr-green-model",   required_argument, NULL, OPT_OCR_GREEN_MODEL},
        {"ocr-keys",          required_argument, NULL, OPT_OCR_KEYS},
        {"frames",            required_argument, NULL, OPT_FRAMES},
        {"fps",               required_argument, NULL, OPT_FPS},
        {"min-plate-conf",    required_argument, NULL, OPT_MIN_CONF},
        {"plate-nms-iou",     required_argument, NULL, OPT_NMS_IOU},
        {"plate-max-det",     required_argument, NULL, OPT_MAX_DET},
        {"class-filter",      required_argument, NULL, OPT_CLASS_FILTER},
        {"det-resize",        required_argument, NULL, OPT_DET_RESIZE},
        {"ocr-preproc",       required_argument, NULL, OPT_OCR_PREPROC},
        {"pixel-order",       required_argument, NULL, OPT_PIXEL_ORDER},
        {"swap16",            required_argument, NULL, OPT_SWAP16},
        {"drm-card",          required_argument, NULL, OPT_DRM_CARD},
        {"connector-id",      required_argument, NULL, OPT_CONNECTOR_ID},
        {"no-display",        no_argument,       NULL, OPT_NO_DISPLAY},
        {"display-sync",      required_argument, NULL, OPT_DISPLAY_SYNC},
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
        {"help",              no_argument,       NULL, 'h'},
        {0, 0, 0, 0},
    };
    int c;
    defaults(o);
    while ((c = getopt_long(argc, argv, "h", opts, NULL)) != -1) {
        switch (c) {
        case OPT_DEVICE:           o->device_path = optarg; break;
        case OPT_PLATE_MODEL:      o->plate_model_path = optarg; break;
        case OPT_OCR_GREEN_MODEL:  o->ocr_green_model_path = optarg; break;
        case OPT_OCR_KEYS:         /* shared keys path used as default for blue+green */
            if (!o->keys_blue_path)  o->keys_blue_path  = optarg;
            if (!o->keys_green_path) o->keys_green_path = optarg;
            break;
        case OPT_FRAMES:           o->frames = atoi(optarg); break;
        case OPT_FPS:              o->fps = atoi(optarg); break;
        case OPT_MIN_CONF:         o->min_conf = strtof(optarg, NULL); break;
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
        case OPT_DISPLAY_SYNC:
            o->display_sync = (strcmp(optarg, "1") == 0 || strcmp(optarg, "true") == 0 || strcmp(optarg, "on") == 0);
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
        case 'h': return 1;
        default:  return -1;
        }
    }
    if (!o->plate_model_path || !o->ocr_blue_model_path || !o->ocr_green_model_path)
        return -1;
    if (!o->keys_blue_path || !o->keys_green_path)
        return -1;
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
    if (o->fps <= 0 || o->fps > 120 || o->frames < 0 || o->max_det <= 0 || o->max_det > MAX_DETS)
        return -1;
    return 0;
}

int main(int argc, char **argv)
{
    struct live_options opt;
    struct dma_state dma;
    struct rknn_model det_model;
    struct rknn_model ocr_blue_model;
    struct rknn_model ocr_green_model;
    struct rknn_model ocr_police_model;
    struct rknn_model ocr_embassy_model;
    struct rknn_model ocr_yellow_model;
    struct ocr_keys keys_blue;
    struct ocr_keys keys_green;
    struct ocr_keys keys_police;
    struct ocr_keys keys_embassy;
    struct ocr_keys keys_yellow;
    struct display_state display;
    struct infer_state infer;
    uint8_t *rgb = NULL;
    uint16_t *display_frame = NULL;
    int pose_nc;
    int class_filter;
    int ret = 1;
    int parsed;
    int64_t target_us;
    bool police_enabled;
    bool embassy_enabled;
    bool yellow_enabled;

    parsed = parse_options(argc, argv, &opt);
    if (parsed != 0) {
        usage(argv[0]);
        return parsed > 0 ? 0 : 1;
    }

    memset(&dma, 0, sizeof(dma)); dma.fd = -1;
    memset(&display, 0, sizeof(display)); display.drm_fd = -1;
    memset(&infer, 0, sizeof(infer));
    memset(&det_model, 0, sizeof(det_model));
    memset(&ocr_blue_model, 0, sizeof(ocr_blue_model));
    memset(&ocr_green_model, 0, sizeof(ocr_green_model));
    memset(&ocr_police_model, 0, sizeof(ocr_police_model));
    memset(&ocr_embassy_model, 0, sizeof(ocr_embassy_model));
    memset(&ocr_yellow_model, 0, sizeof(ocr_yellow_model));
    memset(&keys_blue, 0, sizeof(keys_blue));
    memset(&keys_green, 0, sizeof(keys_green));
    memset(&keys_police, 0, sizeof(keys_police));
    memset(&keys_embassy, 0, sizeof(keys_embassy));
    memset(&keys_yellow, 0, sizeof(keys_yellow));

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    if (opt.display)
        gst_init(NULL, NULL);

    police_enabled = (opt.ocr_police_model_path != NULL);
    embassy_enabled = (opt.ocr_embassy_model_path != NULL);
    yellow_enabled = (opt.ocr_yellow_model_path != NULL);

    if (lpr_load_keys(opt.keys_blue_path, &keys_blue) < 0) {
        fprintf(stderr, "[bgp-live] failed to load blue keys: %s\n", opt.keys_blue_path);
        goto out;
    }
    if (lpr_load_keys(opt.keys_green_path, &keys_green) < 0) {
        fprintf(stderr, "[bgp-live] failed to load green keys: %s\n", opt.keys_green_path);
        goto out;
    }
    if (police_enabled) {
        if (lpr_load_keys(opt.keys_police_path, &keys_police) < 0) {
            fprintf(stderr, "[bgp-live] failed to load police keys: %s\n", opt.keys_police_path);
            goto out;
        }
    }
    if (embassy_enabled) {
        if (lpr_load_keys(opt.keys_embassy_path, &keys_embassy) < 0) {
            fprintf(stderr, "[bgp-live] failed to load embassy keys: %s\n", opt.keys_embassy_path);
            goto out;
        }
    }
    if (yellow_enabled) {
        if (lpr_load_keys(opt.keys_yellow_path, &keys_yellow) < 0) {
            fprintf(stderr, "[bgp-live] failed to load yellow keys: %s\n", opt.keys_yellow_path);
            goto out;
        }
    }

    if (lpr_dma_init(&dma, &opt) < 0) {
        fprintf(stderr, "[bgp-live] failed to init DMA: %s: %s\n",
                opt.device_path, strerror(errno));
        goto out;
    }
    if (lpr_display_start(&display, &opt, dma.frame_w, dma.frame_h) < 0) {
        fprintf(stderr, "[bgp-live] failed to start display\n");
        goto out;
    }
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
    if (police_enabled) {
        if (lpr_model_load(&ocr_police_model, "pplcnet_police", opt.ocr_police_model_path) < 0) {
            fprintf(stderr, "[bgp-live] failed to load police OCR: %s\n", opt.ocr_police_model_path);
            goto out;
        }
    }
    if (embassy_enabled) {
        if (lpr_model_load(&ocr_embassy_model, "pplcnet_embassy", opt.ocr_embassy_model_path) < 0) {
            fprintf(stderr, "[bgp-live] failed to load embassy OCR: %s\n", opt.ocr_embassy_model_path);
            goto out;
        }
    }
    if (yellow_enabled) {
        if (lpr_model_load(&ocr_yellow_model, "pplcnet_yellow", opt.ocr_yellow_model_path) < 0) {
            fprintf(stderr, "[bgp-live] failed to load yellow OCR: %s\n", opt.ocr_yellow_model_path);
            goto out;
        }
    }

    if (det_model.in_w != ALGO_STREAM_SIZE || det_model.in_h != ALGO_STREAM_SIZE || det_model.in_c != 3) {
        fprintf(stderr, "[bgp-live] detector input must be 640x640x3, got %ux%ux%u\n",
                det_model.in_w, det_model.in_h, det_model.in_c);
        goto out;
    }
    if (ocr_blue_model.in_c != 3 || ocr_green_model.in_c != 3 ||
        (police_enabled && ocr_police_model.in_c != 3) ||
        (embassy_enabled && ocr_embassy_model.in_c != 3) ||
        (yellow_enabled && ocr_yellow_model.in_c != 3)) {
        fprintf(stderr,
                "[bgp-live] OCR input must have 3 channels, got blue=%u green=%u police=%u embassy=%u yellow=%u\n",
                ocr_blue_model.in_c, ocr_green_model.in_c,
                police_enabled ? ocr_police_model.in_c : 0,
                embassy_enabled ? ocr_embassy_model.in_c : 0,
                yellow_enabled ? ocr_yellow_model.in_c : 0);
        goto out;
    }

    lpr_ocr_log_contract("blue",  &ocr_blue_model,  &keys_blue);
    lpr_ocr_log_contract("green", &ocr_green_model, &keys_green);
    if (police_enabled)
        lpr_ocr_log_contract("police", &ocr_police_model, &keys_police);
    if (embassy_enabled)
        lpr_ocr_log_contract("embassy", &ocr_embassy_model, &keys_embassy);
    if (yellow_enabled)
        lpr_ocr_log_contract("yellow", &ocr_yellow_model, &keys_yellow);

    pose_nc = lpr_detector_pose_nc(&det_model);
    class_filter = opt.class_filter;
    if (opt.auto_green_filter && pose_nc >= 5)
        class_filter = 1;

    rgb = malloc((size_t)dma.frame_w * dma.frame_h * 3U);
    if (opt.display)
        display_frame = malloc((size_t)dma.frame_w * dma.frame_h * 2U);
    if (!rgb || (opt.display && !display_frame))
        goto out;

    fprintf(stderr,
            "[bgp-live] start frame=%ux%u src=%s frames=%d fps=%d pose_nc=%d class_filter=%d "
            "det_resize=%s blue_ocr=%ux%u green_ocr=%ux%u police_ocr=%s embassy_ocr=%s yellow_ocr=%s "
            "preproc=%s display=%d auto_green_filter=%d async_infer=1\n",
            dma.frame_w, dma.frame_h, dma.src_is_bgrx ? "bgrx8888" : "bgr565",
            opt.frames, opt.fps, pose_nc, class_filter,
            opt.det_resize_mode == DET_RESIZE_LETTERBOX ? "letterbox" : "stretch",
            ocr_blue_model.in_w, ocr_blue_model.in_h,
            ocr_green_model.in_w, ocr_green_model.in_h,
            police_enabled ? "enabled" : "disabled",
            embassy_enabled ? "enabled" : "disabled",
            yellow_enabled ? "enabled" : "disabled",
            opt.ocr_preproc_mode == OCR_PREPROC_GRAY ? "gray" :
                (opt.ocr_preproc_mode == OCR_PREPROC_BIN ? "bin" : "none"),
            opt.display ? 1 : 0, opt.auto_green_filter ? 1 : 0);

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

    if (police_enabled) {
        routes[LPR_ROUTE_POLICE].model = &ocr_police_model;
        routes[LPR_ROUTE_POLICE].keys = &keys_police;
        routes[LPR_ROUTE_POLICE].decode_family = OCR_DECODE_FAMILY_NORMAL7;
        routes[LPR_ROUTE_POLICE].display_tag = 'P';
        snprintf(routes[LPR_ROUTE_POLICE].name, sizeof(routes[LPR_ROUTE_POLICE].name), "police");
    } else {
        routes[LPR_ROUTE_POLICE].model = NULL;
    }

    if (embassy_enabled) {
        routes[LPR_ROUTE_EMBASSY].model = &ocr_embassy_model;
        routes[LPR_ROUTE_EMBASSY].keys = &keys_embassy;
        routes[LPR_ROUTE_EMBASSY].decode_family = OCR_DECODE_FAMILY_NORMAL7;
        routes[LPR_ROUTE_EMBASSY].display_tag = 'E';
        snprintf(routes[LPR_ROUTE_EMBASSY].name, sizeof(routes[LPR_ROUTE_EMBASSY].name), "embassy");
    } else {
        routes[LPR_ROUTE_EMBASSY].model = NULL;
    }

    if (yellow_enabled) {
        routes[LPR_ROUTE_YELLOW].model = &ocr_yellow_model;
        routes[LPR_ROUTE_YELLOW].keys = &keys_yellow;
        routes[LPR_ROUTE_YELLOW].decode_family = OCR_DECODE_FAMILY_NORMAL7;
        routes[LPR_ROUTE_YELLOW].display_tag = 'Y';
        snprintf(routes[LPR_ROUTE_YELLOW].name, sizeof(routes[LPR_ROUTE_YELLOW].name), "yellow");
    } else {
        routes[LPR_ROUTE_YELLOW].model = NULL;
    }

    if (lpr_infer_start(&infer, &opt, &det_model, routes,
                        pose_nc, class_filter,
                        (int)dma.frame_w, (int)dma.frame_h) < 0) {
        fprintf(stderr, "[bgp-live] failed to start infer thread\n");
        goto out;
    }

    target_us = 1000000LL / opt.fps;
    for (int frame = 0; !g_stop && (opt.frames == 0 || frame < opt.frames); frame++) {
        struct live_result latest;
        int64_t t0 = lpr_mono_us();
        if (lpr_dma_read_frame(&dma) < 0) {
            fprintf(stderr, "[bgp-live] DMA frame read failed\n");
            goto out;
        }
        lpr_frame_to_rgb888(&dma, &opt, rgb);
        lpr_infer_submit_latest(&infer, rgb);

        if (display_frame) {
            lpr_rgb888_to_rgb565(rgb, display_frame, (int)dma.frame_w, (int)dma.frame_h);
            lpr_infer_get_result(&infer, &latest);
            if (latest.valid) {
                char ascii[32];
                char overlay[64];
                int ty = latest.box.y1 - (7 * OVERLAY_TEXT_SCALE + 3);
                char tag = 'B';
                if (latest.route_name[0] == 'g') tag = 'G';
                else if (latest.route_name[0] == 'p') tag = 'P';
                else if (latest.route_name[0] == 'e') tag = 'E';
                else if (latest.route_name[0] == 'y') tag = 'Y';
                if (ty < 0) ty = latest.box.y1 + 3;
                lpr_overlay_ascii_from_text(latest.text, ascii, sizeof(ascii));
                snprintf(overlay, sizeof(overlay), "%s %c %.2f",
                         ascii[0] ? ascii : "OCR", tag, latest.conf);
                lpr_draw_rect_565(display_frame, (int)dma.frame_w, (int)dma.frame_h,
                                  &latest.box, COLOR_CYAN_565);
                lpr_draw_text_565(display_frame, (int)dma.frame_w, (int)dma.frame_h,
                                  latest.box.x1, ty, overlay, COLOR_CYAN_565,
                                  OVERLAY_TEXT_SCALE);
            }
            if (lpr_display_push(&display, display_frame) < 0)
                goto out;
        }
        {
            int64_t used = lpr_mono_us() - t0;
            if (used < target_us)
                usleep((useconds_t)(target_us - used));
        }
    }
    ret = 0;

out:
    lpr_infer_stop(&infer);
    free(rgb);
    free(display_frame);
    lpr_display_stop(&display);
    lpr_model_release(&det_model);
    lpr_model_release(&ocr_blue_model);
    lpr_model_release(&ocr_green_model);
    if (police_enabled)
        lpr_model_release(&ocr_police_model);
    if (embassy_enabled)
        lpr_model_release(&ocr_embassy_model);
    if (yellow_enabled)
        lpr_model_release(&ocr_yellow_model);
    lpr_dma_release(&dma);
    return ret;
}
