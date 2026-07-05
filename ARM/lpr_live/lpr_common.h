// SPDX-License-Identifier: GPL-2.0
/*
 * Common types and configuration shared across the live LPR pipeline modules.
 *
 * Pipeline stages (each with its own .c/.h pair under lpr_live/):
 *   1. lpr_dma      : FPGA DMA frame capture, BGR565/BGRX8888 -> RGB888
 *   2. lpr_detector : YOLOv8n-pose plate detector with NMS and quad output
 *   3. lpr_warp     : 4-point homography warp from quad to plate crop
 *   4. lpr_color    : RGB-based blue/green/yellow/white/black plate color classification
 *   5. lpr_ptype    : optional RKNN plate-type classifier for route override
 *   6. lpr_ocr      : PPLCNet RKNN OCR with CTC decode and layout autodetect
 *   7. lpr_display  : DRM/KMS appsrc -> kmssink RGB16 display with overlay
 *   8. lpr_infer    : background inference thread that owns the latest frame
 *
 * The main() entry point in pplcnet_bgp_live.c only handles option parsing,
 * model loading, DMA pump loop, display push and shutdown. All pipeline logic
 * lives in the modules above for testability and reuse.
 *
 * Conventions:
 *   - All buffers are caller-owned; modules never realloc input pointers.
 *   - All return codes: 0 on success, negative on failure, with errno-style
 *     reporting via stderr at the layer where the syscall happens.
 *   - Image data is always RGB888 packed in module APIs unless explicitly
 *     stated otherwise (e.g., display takes RGB565).
 */

#ifndef LPR_LIVE_LPR_COMMON_H
#define LPR_LIVE_LPR_COMMON_H

/* Required for POSIX features used across the modules:
 *   _POSIX_C_SOURCE >= 200809  : strnlen, clock_gettime, useconds_t
 *   _DEFAULT_SOURCE / _GNU_SOURCE : O_CLOEXEC
 * The original pplcnet_bg_live.c relied on glibc default feature macros
 * (no -std=c11 -Wextra), so we re-enable them here for the modular build. */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include <rknn_api.h>

/* Detector input dimensions (YOLOv8n-pose). */
#define ALGO_STREAM_SIZE 640
#define OBB_POINT_COUNT  8400

/* Pose head layout: 4 box channels + N class channels + 4 keypoints * (x,y,vis). */
#define POSE_KPT_COUNT      4
#define POSE_KPT_DIMS       3
#define POSE_BOX_CHANNELS   4
#define POSE_KPT_CHANNELS   (POSE_KPT_COUNT * POSE_KPT_DIMS)
#define POSE_MIN_CHANNELS   (POSE_BOX_CHANNELS + 1 + POSE_KPT_CHANNELS)

/* Detection ring sizes. */
#define MAX_DETS         1024
#define MAX_LIVE_PLATES  8
#define MAX_OCR_KEYS     128
#define MAX_OCR_KEY_LEN  16

/* Overlay rendering. */
#define OVERLAY_TEXT_SCALE 2
#define COLOR_CYAN_565     0x07FF
#define COLOR_RED_565      0xF800
#define COLOR_WHITE_565    0xFFFF
#define COLOR_YELLOW_565   0xFFE0
#define COLOR_GREEN_565    0x07E0

#define PLATE_TYPE_CLASSIFIER_DEFAULT_MIN_CONF 0.80f
#define PLATE_TYPE_CLASSIFIER_DEFAULT_SPECIAL_MIN_CONF 0.70f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------------- Enumerations ---------------- */

enum pixel_order {
    PIXEL_ORDER_BGR565 = 0,
    PIXEL_ORDER_RGB565,
};

enum det_resize_mode {
    DET_RESIZE_STRETCH = 0,
    DET_RESIZE_LETTERBOX,
};

enum ocr_preproc_mode {
    OCR_PREPROC_NONE = 0,
    OCR_PREPROC_GRAY,
    OCR_PREPROC_BIN,
};

enum plate_color {
    PLATE_COLOR_UNKNOWN = 0,
    PLATE_COLOR_BLUE,
    PLATE_COLOR_GREEN,
    PLATE_COLOR_YELLOW,   /* yellow body: yellow plates (taxi/learner/heavy) or police candidate under low light */
    PLATE_COLOR_WHITE,    /* white body: police plates (and most embassy bodies) */
    PLATE_COLOR_BLACK,    /* black body: embassy plates */
};

/* ---------------- Live options (parsed from CLI) ---------------- */

struct live_options {
    const char *device_path;
    const char *plate_model_path;
    /* OCR model paths per plate type. blue + green are required;
     * police, embassy, yellow are optional and silently skipped if NULL. */
    const char *ocr_blue_model_path;
    const char *ocr_green_model_path;
    const char *ocr_police_model_path;
    const char *ocr_embassy_model_path;
    const char *ocr_yellow_model_path;
    /* Per-route keys files. blue + green can fall back to a shared
     * --ocr-keys file; police / embassy / yellow each need their own. */
    const char *keys_blue_path;
    const char *keys_green_path;
    const char *keys_police_path;
    const char *keys_embassy_path;
    const char *keys_yellow_path;
    const char *plate_type_classifier_model_path;
    const char *drm_card_path;
    int connector_id;
    int frames;
    int fps;
    float min_conf;
    float det_score_scale;
    float nms_iou;
    float plate_type_classifier_min_conf;
    float plate_type_classifier_special_min_conf;
    int max_det;
    int class_filter;
    bool auto_green_filter;
    enum det_resize_mode det_resize_mode;
    enum ocr_preproc_mode ocr_preproc_mode;
    enum pixel_order pixel_order;
    bool swap16;
    bool display;
    bool display_sync;
    bool display_atomic_flip;
    bool det_zero_copy;
    bool no_infer;
    /* Raw-frame dump for display-vs-capture diagnostics. When dump_frames > 0,
     * the first dump_frames BGRX frames captured from DMA are written verbatim
     * (frame_w*frame_h*4 bytes) to <dump_path>/frame_NNNN.bgrx, before any
     * overlay drawing or display push. Lets you inspect whether a visual
     * artifact (green lines, tearing) is already present in the captured
     * frame or introduced downstream by the display path. */
    int dump_frames;
    const char *dump_path;
    int dma_pre_delay_us;
    int display_every;
};

/* ---------------- Detector outputs ---------------- */

struct det_box {
    int x1;
    int y1;
    int x2;
    int y2;
    float conf;
    int cls;
    float quad[8];   /* TL.x, TL.y, TR.x, TR.y, BR.x, BR.y, BL.x, BL.y */
};

/* ---------------- RKNN model wrapper ---------------- */

struct rknn_model {
    const char *name;
    const char *path;
    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr input_attr;
    rknn_tensor_attr output_attrs[8];
    uint32_t in_w;
    uint32_t in_h;
    uint32_t in_c;
    /* Zero-copy input tensor memory (NULL when disabled or allocation failed).
     * When input_zero_copy is set and input_native_supported is true, the
     * detector writes its letterboxed input directly into input_mem->virt_addr
     * and binds it via rknn_set_io_mem with pass_through, skipping the costly
     * rknn_inputs_set UINT8->internal conversion. */
    rknn_tensor_mem *input_mem;
    bool input_zero_copy;
    bool input_native_supported;
};

/* ---------------- OCR keys table ---------------- */

struct ocr_keys {
    char keys[MAX_OCR_KEYS][MAX_OCR_KEY_LEN];
    int count;
};

/* Per-OCR-call timing breakdown for diagnostics. */
struct ocr_timing {
    double prep_ms;
    double input_ms;
    double run_ms;
    double output_ms;
    double decode_ms;
};

/* Per-detector-call timing breakdown for diagnostics. */
struct det_timing {
    double prep_ms;
    double input_ms;
    double run_ms;
    double output_ms;
    double decode_ms;
    double nms_ms;
};

/* ---------------- Live result published from infer thread ---------------- */

struct live_plate_result {
    struct det_box box;
    int crop_w;
    int crop_h;
    enum plate_color color;
    int ptype_cls;
    float ptype_conf;
    bool ptype_applied;
    char route_name[8];     /* "blue" / "green" / "police" / "embassy" / "yellow" */
    char text[64];
    float conf;
    float blank_ratio;
};

struct live_result {
    bool valid;
    uint64_t seq;
    int det_count;
    int result_count;
    int best;
    int frame_slot;
    uint64_t frame_generation;
    struct live_plate_result plates[MAX_LIVE_PLATES];
};

/* ---------------- Common helpers ---------------- */

int64_t lpr_mono_us(void);
float lpr_sigmoidf(float x);
int lpr_load_file(const char *path, void **data_out, uint32_t *size_out);
int lpr_load_keys(const char *path, struct ocr_keys *keys);
const char *lpr_plate_color_str(enum plate_color c);

static inline void lpr_bgrx_pixel_rgb(const uint8_t *bgrx, int w, int x, int y, uint8_t p[3])
{
    const uint8_t *src = bgrx + ((size_t)y * (size_t)w + (size_t)x) * 4U;
    p[0] = src[2];
    p[1] = src[1];
    p[2] = src[0];
}

static inline void lpr_bgrx_set_rgb(uint8_t *bgrx, int w, int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t *dst = bgrx + ((size_t)y * (size_t)w + (size_t)x) * 4U;
    dst[0] = b;
    dst[1] = g;
    dst[2] = r;
}

#endif /* LPR_LIVE_LPR_COMMON_H */
