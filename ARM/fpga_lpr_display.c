// SPDX-License-Identifier: GPL-2.0
/*
 * FPGA LPR Display Application (Phase 4)
 *
 * Display path:
 *   /dev/fpga_dma0 -> appsrc(BGR16) -> queue(leaky) -> kmssink
 *
 * Inference path:
 *   Dedicated worker thread converts BGR565 to RGB888 for RKNN only.
 */

#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <rknn_api.h>

#include "pcie_fpga_dma.h"
#include "ocr_decode.h"

#define DEFAULT_DEVICE "/dev/" FPGA_DMA_DEV_NAME
#define DEFAULT_DRM_CARD "/dev/dri/card0"
#define DEFAULT_FPS 15
#define DEFAULT_TIMEOUT_MS 5000
#define DEFAULT_STATS_INTERVAL 1
#define DEFAULT_COPY_BUFFERS 2
#define DEFAULT_QUEUE_DEPTH 1
#define DEFAULT_QUAD_REFINER_MODEL "stage1_r18_gt_best.rknn"
#define MIN_COPY_BUFFERS 2
#define MAX_COPY_BUFFERS 6

#define MAX_LABELS 256
#define MAX_LABEL_LEN 64
#define MAX_DETS 128
#define MAX_OCR_KEYS 128
#define MAX_OCR_KEY_LEN 16
#define ALGO_STREAM_SIZE 640
#define OCR_CROP_WIDTH 150
#define OCR_CROP_HEIGHT 50
#define OBB_POINT_COUNT 8400
#define POSE_KPT_COUNT 4
#define POSE_KPT_DIMS 3
#define POSE_OUTPUT_CHANNELS (4 + 1 + POSE_KPT_COUNT * POSE_KPT_DIMS)
#define OCR_TRACK_MAX 24
#define OCR_TRACK_HIST 8
#define MAX_UTF8_TOKEN_BYTES 8
#define MAX_PLATE_TOKENS 16

#define COLOR_YELLOW_565 0xFFE0
#define COLOR_CYAN_565 0x07FF
#define COLOR_RED_565 0xF800
#define COLOR_GREEN_565 0x07E0
#define OVERLAY_TEXT_SCALE 3

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum pixel_order {
    PIXEL_ORDER_BGR565 = 0,
    PIXEL_ORDER_RGB565,
};

enum plate_color {
    PLATE_COLOR_UNKNOWN = 0,
    PLATE_COLOR_BLUE,
    PLATE_COLOR_GREEN,
    PLATE_COLOR_YELLOW,
};

enum plate_type {
    PLATE_TYPE_COMMON_BLUE = 0,
    PLATE_TYPE_COMMON_GREEN,
    PLATE_TYPE_YELLOW,
    PLATE_TYPE_POLICE,
    PLATE_TYPE_TRAILER,
    PLATE_TYPE_EMBASSY_CONSULATE,
    PLATE_TYPE_UNKNOWN,
};

enum plate_decode_mode {
    PLATE_DECODE_NONE = 0,
    PLATE_DECODE_ROWS,
    PLATE_DECODE_HEADS,
    PLATE_DECODE_MERGED,
    PLATE_DECODE_OBB,
};

enum ocr_channel_order {
    OCR_CH_RGB = 0,
    OCR_CH_BGR,
};

enum ocr_crop_mode {
    OCR_CROP_FIXED = 0,
    OCR_CROP_BOX,
    OCR_CROP_TIGHT,
    OCR_CROP_BOX_PAD,
    OCR_CROP_MATCH,
    OCR_CROP_OBB_WARP,
    OCR_CROP_OBB_PIECEWISE,
};

enum detector_type {
    DETECTOR_YOLOV5 = 0,
    DETECTOR_YOLOV8_OBB_RKNN,
    DETECTOR_YOLOV8_POSE_RKNN,
};

enum ocr_resize_mode {
    OCR_RESIZE_STRETCH = 0,
    OCR_RESIZE_LETTERBOX,
};

enum ocr_resize_kernel {
    OCR_KERNEL_NN = 0,
    OCR_KERNEL_BILINEAR,
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

struct options {
    const char *device_path;
    const char *drm_card_path;
    const char *veh_model_path;
    const char *plate_model_path;
    const char *ocr_model_path;
    const char *ocr_keys_path;
    const char *quad_refiner_model_path;
    const char *labels_path;
    const char *pred_log_path;
    const char *offline_image_path;
    const char *offline_roi_arg;
    int connector_id;
    int fps;
    enum pixel_order pixel_order;
    int timeout_ms;
    int stats_interval;
    int copy_buffers;
    int queue_depth;
    float min_car_conf;
    float min_plate_conf;
    int plate_on_car_only;
    int plate_only;
    int sw_preproc;
    int fpga_a_mask;
    float a_proj_ratio;
    float a_roi_iou_min;
    int ped_event;
    int red_stable_frames;
    float red_ratio_thr;
    float stopline_ratio;
    int det_resize_mode;
    int plate_refine;
    int plate_detector_type;
    float plate_nms_iou;
    int plate_max_det;
    int plate_class_id;
    int ocr_channel_order;
    int ocr_crop_mode;
    int ocr_resize_mode;
    int ocr_resize_kernel;
    int ocr_preproc_mode;
    int show_crop_box;
    int ocr_min_plate_h;
    float ocr_min_sharpness;
    float ocr_min_occ_ratio;
    int ocr_ctc_diag;
    int ocr_crop_dump_max;
    const char *ocr_crop_dump_dir;
    int offline_detect_plate;
    bool swap16;
};

struct det_box {
    int x1;
    int y1;
    int x2;
    int y2;
    float conf;
    int cls;
    int has_obb;
    float cx;
    float cy;
    float w;
    float h;
    float angle;
    float quad[8];
};

struct plate_det {
    struct det_box box;
    struct det_box crop_box;
    enum plate_color color;
    enum plate_type type;
    int parent_car;
    char ocr_text[64];
    float ocr_conf;
    float ocr_blank_top1;
    float ocr_in_occ_ratio;
};

struct frame_slot {
    uint8_t *data;
    bool in_use;
    uint64_t generation;
};

struct lpr_results {
    struct det_box cars[MAX_DETS];
    int car_count;
    int car_raw_count;
    struct det_box persons[MAX_DETS];
    int person_count;
    int person_raw_count;
    struct plate_det plates[MAX_DETS];
    int plate_count;
    int plate_raw_count;
    int plate_rows_raw;
    int plate_heads_raw;
    int plate_rows_keep;
    int plate_heads_keep;
    int plate_decode_mode;
    int ocr_run_count;
    int ocr_skip_size;
    int ocr_skip_blur;
    int ocr_nonempty_count;
    int overlay_text_nonempty_count;
    struct det_box a_roi;
    int a_roi_valid;
    int light_red;
    uint64_t ped_event_total;
    uint64_t ped_event_last_frame;
    uint64_t frame_seq;
    double infer_ms_last;
    uint64_t infer_frames_total;
    double infer_ms_total;
};

struct detect_decode_diag {
    int rows_raw;
    int heads_raw;
    int rows_keep;
    int heads_keep;
    int mode;
};

struct ocr_diag {
    int t_size;
    int c_size;
    int blank_idx;
    float blank_top1_ratio;
    float in_occ_ratio;
};

struct letterbox_meta {
    float scale;
    int pad_x;
    int pad_y;
    int src_w;
    int src_h;
    int dst_w;
    int dst_h;
    bool valid;
};

struct ocr_track_sample {
    char text[64];
    float conf;
    uint64_t frame_seq;
};

struct ocr_track {
    bool used;
    int ttl;
    uint64_t last_seq;
    struct det_box box;
    struct ocr_track_sample hist[OCR_TRACK_HIST];
    int hist_count;
    int hist_next;
    char province_tok[MAX_UTF8_TOKEN_BYTES];
    float province_score;
};

struct yolo_model {
    const char *name;
    const char *path;
    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr input_attr;
    rknn_tensor_attr output_attrs[8];
    uint32_t in_w;
    uint32_t in_h;
    uint32_t in_c;
    int class_count;
    int detector_type;
    float nms_iou_thr;
    int max_det;
    int class_filter;
};

struct ocr_model {
    const char *name;
    const char *path;
    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr input_attr;
    rknn_tensor_attr output_attrs[4];
    uint32_t in_w;
    uint32_t in_h;
    uint32_t in_c;
};

struct quad_refiner_model {
    const char *name;
    const char *path;
    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr input_attr;
    rknn_tensor_attr output_attrs[4];
    uint32_t in_w;
    uint32_t in_h;
    uint32_t in_c;
};

struct app_ctx;

static bool run_quad_refiner(const struct app_ctx *ctx,
                              const uint8_t *rgb, int img_w, int img_h,
                              const float coarse_quad[8],
                              float refined_quad_out[8]);

static float quad_area8(const float q[8]);
static void quad_center8(const float q[8], float *cx, float *cy);
static float quad_edge_len8(const float q[8], int i);
static bool quad_is_convex8(const float q[8]);
static void rect_box_to_quad(const struct det_box *box, float quad[8]);
static void bbox_from_quad_float(const float q[8], int img_w, int img_h,
                                 int *x1, int *y1, int *x2, int *y2);
static bool warp_quad_to_rect_piecewise_rgb888(const uint8_t *rgb, int img_w, int img_h,
                                               const float quad_in[8], uint8_t *dst,
                                               int dst_cap_w, int dst_cap_h,
                                               int *out_w, int *out_h);
static bool decode_refiner_output_layout(const rknn_tensor_attr *a, int *h, int *w, int *c, bool *is_nchw);

struct app_ctx {
    struct options opt;
    int dev_fd;
    int drm_fd;
    void *dma_map;
    size_t dma_map_size;
    uint8_t *dma_copy;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t src_frame_bpp;
    size_t src_frame_size;
    bool src_is_bgrx;
    uint32_t frame_bpp;
    size_t frame_size;

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
    int64_t last_stats_us;
    uint64_t last_stats_cap;
    uint64_t last_stats_rel;
    uint64_t last_stats_infer;
    uint64_t slot_wait_timeout_count;
    uint64_t infer_overwrite_count;

    pthread_t infer_thread;
    pthread_mutex_t infer_lock;
    pthread_cond_t infer_cond;
    uint8_t *infer_latest_raw;
    bool infer_has_new;
    uint64_t infer_frame_seq;

    pthread_mutex_t result_lock;
    struct lpr_results results;

    struct yolo_model veh_model;
    struct yolo_model plate_model;
    struct ocr_model ocr_model;
    struct quad_refiner_model quad_refiner_model;
    char ocr_keys[MAX_OCR_KEYS][MAX_OCR_KEY_LEN];
    int ocr_key_count;
    int ocr_blank_index;
    bool ocr_keysize_warned;
    FILE *pred_log_fp;
    FILE *ocr_crop_index_fp;
    int ocr_crop_dumped;
    pthread_mutex_t pred_log_lock;
    char labels[MAX_LABELS][MAX_LABEL_LEN];
    int label_count;
    int car_class_id;
    int person_class_id;

    struct det_box plate_hist1[MAX_DETS];
    int plate_hist1_count;
    struct det_box plate_hist2[MAX_DETS];
    int plate_hist2_count;

    uint64_t pred_rows_total;
    uint64_t gate_plate_raw_positive_frames;
    uint64_t gate_plate_raw_positive_streak;

    struct det_box ped_track_box[MAX_DETS];
    int ped_track_id[MAX_DETS];
    int ped_track_ttl[MAX_DETS];
    int ped_track_count;
    int ped_next_track_id;
    int ped_red_streak;

    struct ocr_track ocr_tracks[OCR_TRACK_MAX];
    uint64_t ocr_track_age_seq;
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

static void resize_rgb888_nn(const uint8_t *src, int sw, int sh, uint8_t *dst, int dw, int dh);
static void resize_rgb888_bilinear(const uint8_t *src, int sw, int sh, uint8_t *dst, int dw, int dh);
static void resize_rgb888_with_kernel(const uint8_t *src, int sw, int sh,
                                      uint8_t *dst, int dw, int dh, int kernel);
static void __attribute__((unused)) resize_rgb888_nn_letterbox(const uint8_t *src, int sw, int sh,
                                                               uint8_t *dst, int dw, int dh, uint8_t pad);
static void resize_rgb888_letterbox_kernel(const uint8_t *src, int sw, int sh,
                                           uint8_t *dst, int dw, int dh, uint8_t pad,
                                           int kernel, struct letterbox_meta *meta);
static void ocr_preprocess_rgb888(uint8_t *rgb, int w, int h, int mode);
static float box_iou(const struct det_box *a, const struct det_box *b);
static float laplacian_variance_rgb888(const uint8_t *rgb, int w, int h);
static uint8_t *prepare_ocr_input_rgb888(const struct app_ctx *ctx,
                                         const uint8_t *crop_rgb, int crop_w, int crop_h,
                                         float *occ_ratio_out);
static bool append_utf8_token(char *dst, size_t dst_len, const char *token);
static int rknn_quad_refiner_model_load(struct quad_refiner_model *m, const char *name, const char *path);
static void rknn_quad_refiner_model_release(struct quad_refiner_model *m);
static void copy_cstr_trunc(char *dst, size_t dst_len, const char *src);
static int utf8_token_len(const char *s);
static uint32_t utf8_token_codepoint(const char *tok);
static int split_utf8_tokens(const char *s, char tokens[][MAX_UTF8_TOKEN_BYTES], int max_tokens);
static bool utf8_token_is_cjk(const char *tok);
static bool utf8_token_is_ascii_alnum(const char *tok);
static const char *province_token_ascii(const char *tok);
static void age_ocr_tracks(struct app_ctx *ctx, uint64_t frame_seq);
static int find_or_create_ocr_track(struct app_ctx *ctx, const struct det_box *box);
static void ocr_temporal_smooth(struct app_ctx *ctx, const struct det_box *box, uint64_t frame_seq,
                                char *text, size_t text_len, float *conf);
static int run_model_detect(struct yolo_model *m, const uint8_t *in_rgb, int src_w, int src_h,
                            float conf_thr, struct det_box *out, int *out_count,
                            struct detect_decode_diag *diag);
static void dump_ocr_pair(struct app_ctx *ctx, uint64_t frame_id, const struct plate_det *pd,
                          const uint8_t *crop_rgb, int crop_w, int crop_h,
                          const uint8_t *ocr_in, int ocr_w, int ocr_h);
static void dump_ocr_ab_variant(const struct app_ctx *ctx, uint64_t frame_id, int sample_id,
                                const char *tag, const float quad[8],
                                const uint8_t *crop_rgb, int crop_w, int crop_h);
static void log_prediction_row(struct app_ctx *ctx, uint64_t frame_id, int64_t ts_us,
                               const struct plate_det *pd);
static const char *detector_type_str(int mode);

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
            "  --device <path>         FPGA device (default: %s)\n"
            "  --drm-card <path>       DRM card (default: %s)\n"
            "  --veh-model <path>      Vehicle RKNN model path (required for live camera mode)\n"
            "  --plate-model <path>    Plate RKNN model path (required)\n"
            "  --ocr-model <path>      OCR RKNN model path (required)\n"
            "  --ocr-keys <path>       OCR keys file path (required)\n"
            "  --quad-refiner-model <path|off> Quad refiner RKNN path; default: " DEFAULT_QUAD_REFINER_MODEL " ; pass off to disable\n"
            "  --labels <path>         Labels file path (required for live camera mode)\n"
            "  --pred-log <path>       Prediction CSV output path (optional)\n"
            "  --offline-image <path>  One-shot offline infer on PPM(P6) image and exit\n"
            "  --offline-roi <x1,y1,x2,y2>  Optional plate ROI on offline image\n"
            "  --offline-detect-plate <0|1> Auto detect plate on offline image (default: 1)\n"
            "  --connector-id <id>     Optional KMS connector id\n"
            "  --fps <num>             Target FPS (default: %d)\n"
            "  --pixel-order <mode>    bgr565|rgb565 (default: bgr565)\n"
            "  --swap16 <0|1>          Swap bytes per 16-bit pixel (default: 1)\n"
            "  --timeout-ms <ms>       Frame timeout (default: %d)\n"
            "  --stats-interval <sec>  Stats print interval (default: %d)\n"
            "  --copy-buffers <num>    Copy ring size (default: %d)\n"
            "  --queue-depth <num>     appsrc max frame queue (default: %d)\n"
            "  --min-car-conf <v>      Car confidence threshold (default: 0.35)\n"
            "  --min-plate-conf <v>    Plate confidence threshold (default: 0.45)\n"
            "  --plate-on-car-only <0|1>  Reserve switch (default: 0)\n"
            "  --plate-only <0|1>      Disable vehicle dependency for plate output (default: 1)\n"
            "  --sw-preproc <0|1>      Enable software preproc A/B path (default: 0)\n"
            "  --fpga-a-mask <0|1>     Enable FPGA A-channel ROI fusion (default: 0)\n"
            "  --a-proj-ratio <v>      A-channel projection threshold ratio (default: 0.35)\n"
            "  --a-roi-iou-min <v>     Min IoU for A-ROI filtering (default: 0.05)\n"
            "  --ped-event <0|1>       Enable pedestrian red-light event (default: 0)\n"
            "  --red-stable-frames <n> Red light debounce frames (default: 5)\n"
            "  --red-ratio-thr <v>     A-channel red ratio threshold (default: 0.002)\n"
            "  --stopline-ratio <v>    Stopline Y ratio [0,1] (default: 0.55)\n"
            "  --det-resize-mode <m>   Detect resize: stretch|letterbox (default: letterbox)\n"
            "  --plate-refine <0|1>    Enable local high-res plate refine (default: 1)\n"
            "  --plate-detector-type <m> Plate detector: yolov5|yolov8_obb_rknn|yolov8_pose_rknn (default: yolov5)\n"
            "  --plate-nms-iou <v>     Plate NMS IoU threshold (default: 0.45)\n"
            "  --plate-max-det <n>     Plate max detections after NMS (default: 128)\n"
            "  --plate-class-id <n>    Optional class filter for plate model (-1: disabled)\n"
            "  --ocr-channel-order <m> OCR input order: rgb|bgr (default: rgb)\n"
            "  --ocr-crop-mode <m>     OCR crop mode: fixed|box|tight|box-pad|match|obb_warp|obb_piecewise (default: obb_warp)\n"
            "  --ocr-resize-mode <m>   OCR resize: stretch|letterbox (default: stretch)\n"
            "  --ocr-resize-kernel <m> OCR resize kernel: nn|bilinear (default: nn)\n"
            "  --ocr-preproc <m>       OCR crop preproc: none|gray|bin (default: none)\n"
            "  --show-crop-box <0|1>   Overlay OCR crop box in red (default: 0)\n"
            "  --ocr-min-plate-h <n>   Skip OCR if plate box h < n (default: 24)\n"
            "  --ocr-min-sharpness <v> Skip OCR if Laplacian var < v (default: 20)\n"
            "  --ocr-min-occ-ratio <v> Re-crop once if OCR width occupancy < v (default: 0)\n"
            "  --ocr-ctc-diag <0|1>    Print CTC decode diagnostics (default: 0)\n"
            "  --ocr-crop-dump-dir <p> Dump OCR crops+inputs to directory (default: off)\n"
            "  --ocr-crop-dump-max <n> Max dumped OCR samples (default: 20)\n"
            "  --help                  Show this help\n",
            prog, DEFAULT_DEVICE, DEFAULT_DRM_CARD, DEFAULT_FPS, DEFAULT_TIMEOUT_MS,
            DEFAULT_STATS_INTERVAL, DEFAULT_COPY_BUFFERS, DEFAULT_QUEUE_DEPTH);
}

static int parse_options(int argc, char **argv, struct options *opt)
{
    static const struct option long_opts[] = {
        {"device", required_argument, NULL, 1},
        {"drm-card", required_argument, NULL, 2},
        {"veh-model", required_argument, NULL, 3},
        {"plate-model", required_argument, NULL, 4},
        {"ocr-model", required_argument, NULL, 5},
        {"ocr-keys", required_argument, NULL, 6},
        {"quad-refiner-model", required_argument, NULL, 50},
        {"labels", required_argument, NULL, 7},
        {"pred-log", required_argument, NULL, 8},
        {"offline-image", required_argument, NULL, 36},
        {"offline-roi", required_argument, NULL, 37},
        {"offline-detect-plate", required_argument, NULL, 38},
        {"connector-id", required_argument, NULL, 9},
        {"fps", required_argument, NULL, 10},
        {"pixel-order", required_argument, NULL, 11},
        {"swap16", required_argument, NULL, 12},
        {"timeout-ms", required_argument, NULL, 13},
        {"stats-interval", required_argument, NULL, 14},
        {"copy-buffers", required_argument, NULL, 15},
        {"queue-depth", required_argument, NULL, 16},
        {"min-car-conf", required_argument, NULL, 17},
        {"min-plate-conf", required_argument, NULL, 18},
        {"plate-on-car-only", required_argument, NULL, 19},
        {"plate-only", required_argument, NULL, 20},
        {"sw-preproc", required_argument, NULL, 21},
        {"fpga-a-mask", required_argument, NULL, 22},
        {"a-proj-ratio", required_argument, NULL, 23},
        {"a-roi-iou-min", required_argument, NULL, 24},
        {"ped-event", required_argument, NULL, 25},
        {"red-stable-frames", required_argument, NULL, 26},
        {"red-ratio-thr", required_argument, NULL, 27},
        {"stopline-ratio", required_argument, NULL, 28},
        {"det-resize-mode", required_argument, NULL, 39},
        {"plate-refine", required_argument, NULL, 40},
        {"plate-detector-type", required_argument, NULL, 46},
        {"plate-nms-iou", required_argument, NULL, 47},
        {"plate-max-det", required_argument, NULL, 48},
        {"plate-class-id", required_argument, NULL, 49},
        {"ocr-channel-order", required_argument, NULL, 29},
        {"ocr-crop-mode", required_argument, NULL, 30},
        {"ocr-resize-mode", required_argument, NULL, 31},
        {"ocr-resize-kernel", required_argument, NULL, 44},
        {"ocr-preproc", required_argument, NULL, 32},
        {"show-crop-box", required_argument, NULL, 43},
        {"ocr-min-plate-h", required_argument, NULL, 41},
        {"ocr-min-sharpness", required_argument, NULL, 42},
        {"ocr-min-occ-ratio", required_argument, NULL, 45},
        {"ocr-ctc-diag", required_argument, NULL, 33},
        {"ocr-crop-dump-dir", required_argument, NULL, 34},
        {"ocr-crop-dump-max", required_argument, NULL, 35},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };
    int c;

    memset(opt, 0, sizeof(*opt));
    opt->device_path = DEFAULT_DEVICE;
    opt->drm_card_path = DEFAULT_DRM_CARD;
    opt->quad_refiner_model_path = DEFAULT_QUAD_REFINER_MODEL;
    opt->connector_id = -1;
    opt->fps = DEFAULT_FPS;
    opt->pixel_order = PIXEL_ORDER_BGR565;
    opt->swap16 = true;
    opt->timeout_ms = DEFAULT_TIMEOUT_MS;
    opt->stats_interval = DEFAULT_STATS_INTERVAL;
    opt->copy_buffers = DEFAULT_COPY_BUFFERS;
    opt->queue_depth = DEFAULT_QUEUE_DEPTH;
    opt->min_car_conf = 0.35f;
    opt->min_plate_conf = 0.45f;
    opt->plate_on_car_only = 0;
    opt->plate_only = 1;
    opt->sw_preproc = 0;
    opt->fpga_a_mask = 0;
    opt->a_proj_ratio = 0.35f;
    opt->a_roi_iou_min = 0.05f;
    opt->ped_event = 0;
    opt->red_stable_frames = 5;
    opt->red_ratio_thr = 0.002f;
    opt->stopline_ratio = 0.55f;
    opt->det_resize_mode = DET_RESIZE_LETTERBOX;
    opt->plate_refine = 1;
    opt->plate_detector_type = DETECTOR_YOLOV5;
    opt->plate_nms_iou = 0.45f;
    opt->plate_max_det = MAX_DETS;
    opt->plate_class_id = -1;
    opt->ocr_channel_order = OCR_CH_RGB;
    opt->ocr_crop_mode = OCR_CROP_OBB_WARP;
    opt->ocr_resize_mode = OCR_RESIZE_STRETCH;
    opt->ocr_resize_kernel = OCR_KERNEL_NN;
    opt->ocr_preproc_mode = OCR_PREPROC_NONE;
    opt->show_crop_box = 0;
    opt->ocr_min_plate_h = 24;
    opt->ocr_min_sharpness = 20.0f;
    opt->ocr_min_occ_ratio = 0.0f;
    opt->ocr_ctc_diag = 0;
    opt->ocr_crop_dump_max = 20;
    opt->ocr_crop_dump_dir = NULL;
    opt->offline_detect_plate = 1;

    while ((c = getopt_long(argc, argv, "h", long_opts, NULL)) != -1) {
        switch (c) {
        case 1: opt->device_path = optarg; break;
        case 2: opt->drm_card_path = optarg; break;
        case 3: opt->veh_model_path = optarg; break;
        case 4: opt->plate_model_path = optarg; break;
        case 5: opt->ocr_model_path = optarg; break;
        case 6: opt->ocr_keys_path = optarg; break;
        case 50:
            if (strcmp(optarg, "off") == 0 || strcmp(optarg, "none") == 0 || strcmp(optarg, "disable") == 0)
                opt->quad_refiner_model_path = NULL;
            else
                opt->quad_refiner_model_path = optarg;
            break;
        case 7: opt->labels_path = optarg; break;
        case 8: opt->pred_log_path = optarg; break;
        case 36: opt->offline_image_path = optarg; break;
        case 37: opt->offline_roi_arg = optarg; break;
        case 38: opt->offline_detect_plate = atoi(optarg) ? 1 : 0; break;
        case 9: opt->connector_id = atoi(optarg); break;
        case 10: opt->fps = atoi(optarg); break;
        case 11:
            if (strcmp(optarg, "bgr565") == 0)
                opt->pixel_order = PIXEL_ORDER_BGR565;
            else if (strcmp(optarg, "rgb565") == 0)
                opt->pixel_order = PIXEL_ORDER_RGB565;
            else
                return -1;
            break;
        case 12: opt->swap16 = atoi(optarg) ? true : false; break;
        case 13: opt->timeout_ms = atoi(optarg); break;
        case 14: opt->stats_interval = atoi(optarg); break;
        case 15: opt->copy_buffers = atoi(optarg); break;
        case 16: opt->queue_depth = atoi(optarg); break;
        case 17: opt->min_car_conf = (float)atof(optarg); break;
        case 18: opt->min_plate_conf = (float)atof(optarg); break;
        case 19: opt->plate_on_car_only = atoi(optarg) ? 1 : 0; break;
        case 20: opt->plate_only = atoi(optarg) ? 1 : 0; break;
        case 21: opt->sw_preproc = atoi(optarg) ? 1 : 0; break;
        case 22: opt->fpga_a_mask = atoi(optarg) ? 1 : 0; break;
        case 23: opt->a_proj_ratio = (float)atof(optarg); break;
        case 24: opt->a_roi_iou_min = (float)atof(optarg); break;
        case 25: opt->ped_event = atoi(optarg) ? 1 : 0; break;
        case 26: opt->red_stable_frames = atoi(optarg); break;
        case 27: opt->red_ratio_thr = (float)atof(optarg); break;
        case 28: opt->stopline_ratio = (float)atof(optarg); break;
        case 39:
            if (strcmp(optarg, "stretch") == 0)
                opt->det_resize_mode = DET_RESIZE_STRETCH;
            else if (strcmp(optarg, "letterbox") == 0)
                opt->det_resize_mode = DET_RESIZE_LETTERBOX;
            else
                return -1;
            break;
        case 40: opt->plate_refine = atoi(optarg) ? 1 : 0; break;
        case 46:
            if (strcmp(optarg, "yolov5") == 0)
                opt->plate_detector_type = DETECTOR_YOLOV5;
            else if (strcmp(optarg, "yolov8_obb_rknn") == 0)
                opt->plate_detector_type = DETECTOR_YOLOV8_OBB_RKNN;
            else if (strcmp(optarg, "yolov8_pose_rknn") == 0)
                opt->plate_detector_type = DETECTOR_YOLOV8_POSE_RKNN;
            else
                return -1;
            break;
        case 47: opt->plate_nms_iou = (float)atof(optarg); break;
        case 48: opt->plate_max_det = atoi(optarg); break;
        case 49: opt->plate_class_id = atoi(optarg); break;
        case 29:
            if (strcmp(optarg, "rgb") == 0)
                opt->ocr_channel_order = OCR_CH_RGB;
            else if (strcmp(optarg, "bgr") == 0)
                opt->ocr_channel_order = OCR_CH_BGR;
            else
                return -1;
            break;
        case 30:
            if (strcmp(optarg, "fixed") == 0)
                opt->ocr_crop_mode = OCR_CROP_FIXED;
            else if (strcmp(optarg, "box") == 0)
                opt->ocr_crop_mode = OCR_CROP_BOX;
            else if (strcmp(optarg, "tight") == 0)
                opt->ocr_crop_mode = OCR_CROP_TIGHT;
            else if (strcmp(optarg, "box-pad") == 0)
                opt->ocr_crop_mode = OCR_CROP_BOX_PAD;
            else if (strcmp(optarg, "match") == 0)
                opt->ocr_crop_mode = OCR_CROP_MATCH;
            else if (strcmp(optarg, "obb_warp") == 0 || strcmp(optarg, "obb-warp") == 0)
                opt->ocr_crop_mode = OCR_CROP_OBB_WARP;
            else if (strcmp(optarg, "obb_piecewise") == 0 || strcmp(optarg, "obb-piecewise") == 0)
                opt->ocr_crop_mode = OCR_CROP_OBB_PIECEWISE;
            else
                return -1;
            break;
        case 31:
            if (strcmp(optarg, "stretch") == 0)
                opt->ocr_resize_mode = OCR_RESIZE_STRETCH;
            else if (strcmp(optarg, "letterbox") == 0)
                opt->ocr_resize_mode = OCR_RESIZE_LETTERBOX;
            else
                return -1;
            break;
        case 44:
            if (strcmp(optarg, "nn") == 0)
                opt->ocr_resize_kernel = OCR_KERNEL_NN;
            else if (strcmp(optarg, "bilinear") == 0)
                opt->ocr_resize_kernel = OCR_KERNEL_BILINEAR;
            else
                return -1;
            break;
        case 32:
            if (strcmp(optarg, "none") == 0)
                opt->ocr_preproc_mode = OCR_PREPROC_NONE;
            else if (strcmp(optarg, "gray") == 0)
                opt->ocr_preproc_mode = OCR_PREPROC_GRAY;
            else if (strcmp(optarg, "bin") == 0)
                opt->ocr_preproc_mode = OCR_PREPROC_BIN;
            else
                return -1;
            break;
        case 43: opt->show_crop_box = atoi(optarg) ? 1 : 0; break;
        case 41: opt->ocr_min_plate_h = atoi(optarg); break;
        case 42: opt->ocr_min_sharpness = (float)atof(optarg); break;
        case 45: opt->ocr_min_occ_ratio = (float)atof(optarg); break;
        case 33: opt->ocr_ctc_diag = atoi(optarg) ? 1 : 0; break;
        case 34: opt->ocr_crop_dump_dir = optarg; break;
        case 35: opt->ocr_crop_dump_max = atoi(optarg); break;
        case 'h':
            print_usage(argv[0]);
            exit(0);
        default:
            return -1;
        }
    }

    if (opt->fps <= 0 || opt->timeout_ms <= 0 || opt->stats_interval <= 0)
        return -1;
    if (opt->copy_buffers < MIN_COPY_BUFFERS || opt->copy_buffers > MAX_COPY_BUFFERS)
        return -1;
    if (opt->queue_depth <= 0)
        return -1;
    if (opt->a_proj_ratio <= 0.0f || opt->a_proj_ratio >= 1.0f)
        return -1;
    if (opt->a_roi_iou_min < 0.0f || opt->a_roi_iou_min > 1.0f)
        return -1;
    if (opt->red_stable_frames <= 0 || opt->red_stable_frames > 120)
        return -1;
    if (opt->red_ratio_thr < 0.0f || opt->red_ratio_thr > 1.0f)
        return -1;
    if (opt->stopline_ratio <= 0.05f || opt->stopline_ratio >= 0.95f)
        return -1;
    if (opt->plate_nms_iou < 0.0f || opt->plate_nms_iou > 1.0f)
        return -1;
    if (opt->plate_max_det <= 0 || opt->plate_max_det > MAX_DETS)
        return -1;
    if (opt->plate_class_id < -1)
        return -1;
    if (opt->ocr_min_plate_h < 0 || opt->ocr_min_plate_h > 512)
        return -1;
    if (opt->ocr_min_sharpness < 0.0f || opt->ocr_min_sharpness > 100000.0f)
        return -1;
    if (opt->ocr_min_occ_ratio < 0.0f || opt->ocr_min_occ_ratio > 1.0f)
        return -1;
    if (opt->ocr_crop_dump_max < 0 || opt->ocr_crop_dump_max > 100000)
        return -1;
    if (opt->offline_image_path && opt->offline_image_path[0] != '\0') {
        if (!opt->plate_model_path || !opt->ocr_model_path || !opt->ocr_keys_path)
            return -1;
    } else {
        if (!opt->veh_model_path || !opt->plate_model_path ||
            !opt->ocr_model_path || !opt->ocr_keys_path || !opt->labels_path)
            return -1;
    }
    return 0;
}

static void decode_pixel565(const struct options *opt, uint8_t lo_in, uint8_t hi_in,
                            uint8_t *r8, uint8_t *g8, uint8_t *b8)
{
    uint8_t lo = lo_in;
    uint8_t hi = hi_in;
    uint16_t pix;
    uint8_t r5, g6, b5;
    if (opt->swap16) {
        uint8_t t = lo;
        lo = hi;
        hi = t;
    }
    pix = (uint16_t)lo | ((uint16_t)hi << 8);
    if (opt->pixel_order == PIXEL_ORDER_BGR565) {
        r5 = pix & 0x1F;
        g6 = (pix >> 5) & 0x3F;
        b5 = (pix >> 11) & 0x1F;
    } else {
        r5 = (pix >> 11) & 0x1F;
        g6 = (pix >> 5) & 0x3F;
        b5 = pix & 0x1F;
    }
    *r8 = (uint8_t)((r5 << 3) | (r5 >> 2));
    *g8 = (uint8_t)((g6 << 2) | (g6 >> 4));
    *b8 = (uint8_t)((b5 << 3) | (b5 >> 2));
}

static int load_labels(struct app_ctx *ctx, const char *path)
{
    FILE *fp = fopen(path, "r");
    char line[256];
    int idx = 0;
    if (!fp) {
        fprintf(stderr, "Open labels failed: %s\n", strerror(errno));
        return -1;
    }
    while (fgets(line, sizeof(line), fp) && idx < MAX_LABELS) {
        char *nl = strchr(line, '\n');
        size_t n;
        if (nl) *nl = '\0';
        if (line[0] == '\0')
            continue;
        n = strnlen(line, MAX_LABEL_LEN - 1);
        memcpy(ctx->labels[idx], line, n);
        ctx->labels[idx][n] = '\0';
        idx++;
    }
    fclose(fp);
    ctx->label_count = idx;
    ctx->car_class_id = 2;
    ctx->person_class_id = 0;
    for (idx = 0; idx < ctx->label_count; idx++) {
        if (strcmp(ctx->labels[idx], "car") == 0) {
            ctx->car_class_id = idx;
        } else if (strcmp(ctx->labels[idx], "person") == 0) {
            ctx->person_class_id = idx;
        }
    }
    return 0;
}

static int load_ocr_keys(struct app_ctx *ctx, const char *path)
{
    FILE *fp = fopen(path, "r");
    char line[256];
    int idx = 0;
    if (!fp) {
        fprintf(stderr, "Open OCR keys failed: %s\n", strerror(errno));
        return -1;
    }
    while (fgets(line, sizeof(line), fp) && idx < MAX_OCR_KEYS) {
        char *nl = strchr(line, '\n');
        char *src = line;
        size_t n;
        if (nl) *nl = '\0';
        nl = strchr(line, '\r');
        if (nl) *nl = '\0';
        if (src[0] == '\0' || src[0] == '#')
            continue;
        if (idx == 0 &&
            (unsigned char)src[0] == 0xEF &&
            (unsigned char)src[1] == 0xBB &&
            (unsigned char)src[2] == 0xBF) {
            src += 3;
        }
        n = strnlen(src, MAX_OCR_KEY_LEN - 1);
        memcpy(ctx->ocr_keys[idx], src, n);
        ctx->ocr_keys[idx][n] = '\0';
        idx++;
    }
    fclose(fp);
    if (idx <= 0)
        return -1;
    ctx->ocr_key_count = idx;
    ctx->ocr_blank_index = idx;
    fprintf(stderr, "[ocr] loaded %d keys from %s\n", ctx->ocr_key_count, path);
    return 0;
}

static int rknn_ocr_model_load(struct ocr_model *m, const char *name, const char *path)
{
    FILE *fp;
    long sz;
    void *data;
    uint32_t i;
    memset(m, 0, sizeof(*m));
    m->name = name;
    m->path = path;

    fp = fopen(path, "rb");
    if (!fp) return -1;
    fseek(fp, 0, SEEK_END);
    sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    data = malloc((size_t)sz);
    if (!data) { fclose(fp); return -1; }
    if (fread(data, 1, (size_t)sz, fp) != (size_t)sz) { fclose(fp); free(data); return -1; }
    fclose(fp);

    if (rknn_init(&m->ctx, data, (uint32_t)sz, 0, NULL) < 0) { free(data); return -1; }
    free(data);
    if (rknn_query(m->ctx, RKNN_QUERY_IN_OUT_NUM, &m->io_num, sizeof(m->io_num)) < 0)
        return -1;
    if (m->io_num.n_output == 0 || m->io_num.n_output > 4)
        return -1;

    memset(&m->input_attr, 0, sizeof(m->input_attr));
    m->input_attr.index = 0;
    if (rknn_query(m->ctx, RKNN_QUERY_INPUT_ATTR, &m->input_attr, sizeof(m->input_attr)) < 0)
        return -1;
    if (m->input_attr.fmt == RKNN_TENSOR_NCHW) {
        m->in_c = m->input_attr.dims[1];
        m->in_h = m->input_attr.dims[2];
        m->in_w = m->input_attr.dims[3];
    } else {
        m->in_h = m->input_attr.dims[1];
        m->in_w = m->input_attr.dims[2];
        m->in_c = m->input_attr.dims[3];
    }

    for (i = 0; i < m->io_num.n_output; i++) {
        memset(&m->output_attrs[i], 0, sizeof(m->output_attrs[i]));
        m->output_attrs[i].index = i;
        if (rknn_query(m->ctx, RKNN_QUERY_OUTPUT_ATTR, &m->output_attrs[i], sizeof(m->output_attrs[i])) < 0)
            return -1;
    }

    fprintf(stderr, "[%s] loaded input=%ux%ux%u outputs=%u\n",
            name, m->in_w, m->in_h, m->in_c, m->io_num.n_output);
    fprintf(stderr, "[%s] input_attr fmt=%d type=%d qnt=%d zp=%d scale=%.6f\n",
            name,
            m->input_attr.fmt,
            m->input_attr.type,
            m->input_attr.qnt_type,
            m->input_attr.zp,
            m->input_attr.scale);
    return 0;
}

static void rknn_ocr_model_release(struct ocr_model *m)
{
    if (m->ctx)
        rknn_destroy(m->ctx);
    memset(m, 0, sizeof(*m));
}

static int rknn_quad_refiner_model_load(struct quad_refiner_model *m, const char *name, const char *path)
{
    FILE *fp;
    long sz;
    void *data;
    uint32_t i;
    memset(m, 0, sizeof(*m));
    m->name = name;
    m->path = path;

    if (!path || path[0] == '\0')
        return 0;

    fp = fopen(path, "rb");
    if (!fp) return -1;
    fseek(fp, 0, SEEK_END);
    sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    data = malloc((size_t)sz);
    if (!data) { fclose(fp); return -1; }
    if (fread(data, 1, (size_t)sz, fp) != (size_t)sz) { fclose(fp); free(data); return -1; }
    fclose(fp);

    if (rknn_init(&m->ctx, data, (uint32_t)sz, 0, NULL) < 0) { free(data); return -1; }
    free(data);
    if (rknn_query(m->ctx, RKNN_QUERY_IN_OUT_NUM, &m->io_num, sizeof(m->io_num)) < 0)
        return -1;
    if (m->io_num.n_output == 0 || m->io_num.n_output > 4)
        return -1;

    memset(&m->input_attr, 0, sizeof(m->input_attr));
    m->input_attr.index = 0;
    if (rknn_query(m->ctx, RKNN_QUERY_INPUT_ATTR, &m->input_attr, sizeof(m->input_attr)) < 0)
        return -1;
    if (m->input_attr.fmt == RKNN_TENSOR_NCHW) {
        m->in_c = m->input_attr.dims[1];
        m->in_h = m->input_attr.dims[2];
        m->in_w = m->input_attr.dims[3];
    } else {
        m->in_h = m->input_attr.dims[1];
        m->in_w = m->input_attr.dims[2];
        m->in_c = m->input_attr.dims[3];
    }

    for (i = 0; i < m->io_num.n_output; i++) {
        memset(&m->output_attrs[i], 0, sizeof(m->output_attrs[i]));
        m->output_attrs[i].index = i;
        if (rknn_query(m->ctx, RKNN_QUERY_OUTPUT_ATTR, &m->output_attrs[i], sizeof(m->output_attrs[i])) < 0)
            return -1;
    }

    fprintf(stderr, "[%s] loaded input=%ux%ux%u outputs=%u\n",
            name, m->in_w, m->in_h, m->in_c, m->io_num.n_output);
    fprintf(stderr, "[%s] input_attr fmt=%d type=%d qnt=%d zp=%d scale=%.6f\n",
            name,
            m->input_attr.fmt,
            m->input_attr.type,
            m->input_attr.qnt_type,
            m->input_attr.zp,
            m->input_attr.scale);
    for (i = 0; i < m->io_num.n_output; i++) {
        const rknn_tensor_attr *a = &m->output_attrs[i];
        fprintf(stderr, "[%s] out[%u] fmt=%d type=%d qnt=%d dims=(%u,%u,%u,%u) n_dims=%u\n",
                name, i, a->fmt, a->type, a->qnt_type,
                a->dims[0], a->dims[1], a->dims[2], a->dims[3], a->n_dims);
    }
    return 0;
}

static void rknn_quad_refiner_model_release(struct quad_refiner_model *m)
{
    if (m->ctx)
        rknn_destroy(m->ctx);
    memset(m, 0, sizeof(*m));
}

static bool build_ocr_layout(const rknn_tensor_attr *a, int *t_size, int *c_size, int *t_stride, int *c_stride)
{
    if (a->n_dims == 2) {
        *t_size = (int)a->dims[0];
        *c_size = (int)a->dims[1];
        *t_stride = *c_size;
        *c_stride = 1;
        return (*t_size > 0 && *c_size > 1);
    }
    if (a->n_dims == 3) {
        int d1 = (int)a->dims[1];
        int d2 = (int)a->dims[2];
        if (d1 <= 0 || d2 <= 1)
            return false;
        if (d1 <= d2) {
            *t_size = d1;
            *c_size = d2;
            *t_stride = *c_size;
            *c_stride = 1;
        } else {
            *t_size = d2;
            *c_size = d1;
            *t_stride = 1;
            *c_stride = *t_size;
        }
        return true;
    }
    if (a->n_dims == 4) {
        if (a->fmt == RKNN_TENSOR_NCHW) {
            *c_size = (int)a->dims[1];
            *t_size = (int)a->dims[2] * (int)a->dims[3];
            *t_stride = 1;
            *c_stride = *t_size;
        } else {
            *t_size = (int)a->dims[1] * (int)a->dims[2];
            *c_size = (int)a->dims[3];
            *t_stride = *c_size;
            *c_stride = 1;
        }
        return (*t_size > 0 && *c_size > 1);
    }
    return false;
}

static int ctc_decode_logits(const float *buf, int t_size, int c_size, int t_stride, int c_stride,
                             struct app_ctx *ctx, enum ocr_decode_family family,
                             char *text, size_t text_len, float *conf_out,
                             struct ocr_diag *diag)
{
    const char *keys[MAX_OCR_KEYS];
    struct ocr_decode_diag decode_diag;
    int i;
    int ret;

    if (!ctx || !buf || !text || text_len == 0)
        return -1;
    for (i = 0; i < ctx->ocr_key_count && i < MAX_OCR_KEYS; i++)
        keys[i] = ctx->ocr_keys[i];
    ret = ocr_decode_logits(buf, t_size, c_size, t_stride, c_stride,
                            keys, i, ctx->ocr_blank_index,
                            family, text, text_len, conf_out, &decode_diag);
    if (ret < 0)
        return ret;
    if (diag) {
        diag->t_size = decode_diag.t_size;
        diag->c_size = decode_diag.c_size;
        diag->blank_idx = decode_diag.blank_idx;
        diag->blank_top1_ratio = decode_diag.blank_top1_ratio;
    }
    return 0;
}

static bool append_utf8_token(char *dst, size_t dst_len, const char *token)
{
    size_t cur;
    size_t tok_len;

    if (!dst || dst_len == 0 || !token)
        return false;
    cur = strnlen(dst, dst_len);
    if (cur >= dst_len - 1)
        return false;
    tok_len = strnlen(token, MAX_OCR_KEY_LEN - 1);
    if (tok_len == 0)
        return false;
    if (tok_len >= dst_len - cur)
        return false;
    memcpy(dst + cur, token, tok_len);
    dst[cur + tok_len] = '\0';
    return true;
}

static void copy_cstr_trunc(char *dst, size_t dst_len, const char *src)
{
    size_t n;
    if (!dst || dst_len == 0)
        return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    n = strnlen(src, dst_len - 1);
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static int utf8_token_len(const char *s)
{
    unsigned char c0, c1, c2, c3;
    if (!s || s[0] == '\0')
        return 0;
    c0 = (unsigned char)s[0];
    if (c0 < 0x80)
        return 1;
    if ((c0 & 0xE0U) == 0xC0U) {
        c1 = (unsigned char)s[1];
        return ((c1 & 0xC0U) == 0x80U) ? 2 : 1;
    }
    if ((c0 & 0xF0U) == 0xE0U) {
        c1 = (unsigned char)s[1];
        c2 = (unsigned char)s[2];
        if ((c1 & 0xC0U) == 0x80U && (c2 & 0xC0U) == 0x80U)
            return 3;
        return 1;
    }
    if ((c0 & 0xF8U) == 0xF0U) {
        c1 = (unsigned char)s[1];
        c2 = (unsigned char)s[2];
        c3 = (unsigned char)s[3];
        if ((c1 & 0xC0U) == 0x80U && (c2 & 0xC0U) == 0x80U && (c3 & 0xC0U) == 0x80U)
            return 4;
        return 1;
    }
    return 1;
}

static uint32_t utf8_token_codepoint(const char *tok)
{
    int len;
    const unsigned char *p = (const unsigned char *)tok;
    if (!tok || tok[0] == '\0')
        return 0;
    len = utf8_token_len(tok);
    if (len == 1)
        return (uint32_t)p[0];
    if (len == 2)
        return (uint32_t)(((p[0] & 0x1FU) << 6) | (p[1] & 0x3FU));
    if (len == 3)
        return (uint32_t)(((p[0] & 0x0FU) << 12) | ((p[1] & 0x3FU) << 6) | (p[2] & 0x3FU));
    if (len == 4)
        return (uint32_t)(((p[0] & 0x07U) << 18) | ((p[1] & 0x3FU) << 12) |
                          ((p[2] & 0x3FU) << 6) | (p[3] & 0x3FU));
    return 0;
}

static int split_utf8_tokens(const char *s, char tokens[][MAX_UTF8_TOKEN_BYTES], int max_tokens)
{
    int n = 0;
    const char *p = s;
    if (!s || !tokens || max_tokens <= 0)
        return 0;
    while (p[0] != '\0' && n < max_tokens) {
        int len = utf8_token_len(p);
        int cp_len = len;
        if (cp_len < 1)
            break;
        if (len >= MAX_UTF8_TOKEN_BYTES)
            len = MAX_UTF8_TOKEN_BYTES - 1;
        memcpy(tokens[n], p, (size_t)len);
        tokens[n][len] = '\0';
        n++;
        p += cp_len;
    }
    return n;
}

static bool utf8_token_is_cjk(const char *tok)
{
    uint32_t cp = utf8_token_codepoint(tok);
    return (cp >= 0x4E00U && cp <= 0x9FFFU);
}

static bool utf8_token_is_ascii_alnum(const char *tok)
{
    return tok && tok[0] != '\0' && tok[1] == '\0' && isalnum((unsigned char)tok[0]);
}

static const char *province_token_ascii(const char *tok)
{
    uint32_t cp = utf8_token_codepoint(tok);
    switch (cp) {
    case 0x4EACU: return "BJ";
    case 0x6D25U: return "TJ";
    case 0x6CAAU: return "SH";
    case 0x6E1DU: return "CQ";
    case 0x5180U: return "HE";
    case 0x664BU: return "SX";
    case 0x8499U: return "NM";
    case 0x8FBDU: return "LN";
    case 0x5409U: return "JL";
    case 0x9ED1U: return "HL";
    case 0x82CFU: return "JS";
    case 0x6D59U: return "ZJ";
    case 0x7696U: return "AH";
    case 0x95FDU: return "FJ";
    case 0x8D63U: return "JX";
    case 0x9C81U: return "SD";
    case 0x8C6BU: return "HA";
    case 0x9102U: return "HB";
    case 0x6E58U: return "HN";
    case 0x7CA4U: return "GD";
    case 0x6842U: return "GX";
    case 0x743CU: return "HI";
    case 0x5DDDU: return "SC";
    case 0x8D35U: return "GZ";
    case 0x4E91U: return "YN";
    case 0x85CFU: return "XZ";
    case 0x9655U: return "SN";
    case 0x7518U: return "GS";
    case 0x9752U: return "QH";
    case 0x5B81U: return "NX";
    case 0x65B0U: return "XJ";
    case 0x6E2FU: return "HK";
    case 0x6FB3U: return "MO";
    case 0x53F0U: return "TW";
    case 0x8B66U: return "J";
    case 0x6302U: return "G";
    case 0x9886U: return "L";
    case 0x4F7FU: return "S";
    default: return NULL;
    }
}

static void age_ocr_tracks(struct app_ctx *ctx, uint64_t frame_seq)
{
    int i;
    if (!ctx || ctx->ocr_track_age_seq == frame_seq)
        return;
    for (i = 0; i < OCR_TRACK_MAX; i++) {
        struct ocr_track *tr = &ctx->ocr_tracks[i];
        if (!tr->used)
            continue;
        tr->ttl--;
        tr->province_score *= 0.97f;
        if (tr->ttl <= 0)
            memset(tr, 0, sizeof(*tr));
    }
    ctx->ocr_track_age_seq = frame_seq;
}

static int find_or_create_ocr_track(struct app_ctx *ctx, const struct det_box *box)
{
    int i;
    int best_idx = -1;
    float best_iou = 0.0f;
    int free_idx = -1;
    int replace_idx = -1;
    int min_ttl = 1 << 30;

    if (!ctx || !box)
        return -1;

    for (i = 0; i < OCR_TRACK_MAX; i++) {
        struct ocr_track *tr = &ctx->ocr_tracks[i];
        float iou;
        if (!tr->used) {
            if (free_idx < 0)
                free_idx = i;
            continue;
        }
        iou = box_iou(box, &tr->box);
        if (iou > best_iou) {
            best_iou = iou;
            best_idx = i;
        }
        if (tr->ttl < min_ttl) {
            min_ttl = tr->ttl;
            replace_idx = i;
        }
    }
    if (best_idx >= 0 && best_iou >= 0.25f)
        return best_idx;

    if (free_idx >= 0)
        best_idx = free_idx;
    else
        best_idx = replace_idx;

    if (best_idx >= 0)
        memset(&ctx->ocr_tracks[best_idx], 0, sizeof(ctx->ocr_tracks[best_idx]));
    return best_idx;
}

static void ocr_temporal_smooth(struct app_ctx *ctx, const struct det_box *box, uint64_t frame_seq,
                                char *text, size_t text_len, float *conf)
{
    struct ocr_track *tr;
    char raw_text[64];
    char smooth[64];
    int tr_idx;
    int i, k;
    int sample_count = 0;
    float sample_w[OCR_TRACK_HIST];
    int sample_lens[OCR_TRACK_HIST];
    char sample_tokens[OCR_TRACK_HIST][MAX_PLATE_TOKENS][MAX_UTF8_TOKEN_BYTES];
    float len_vote[MAX_PLATE_TOKENS + 1];
    int target_len = 0;
    float target_w = -1.0f;
    float smooth_ratio_sum = 0.0f;
    int smooth_ratio_n = 0;
    float smooth_conf;
    char first_tok[MAX_UTF8_TOKEN_BYTES];
    char smooth_first[MAX_UTF8_TOKEN_BYTES];
    bool raw_first_cjk = false;
    bool smooth_first_cjk = false;

    if (!ctx || !box || !text || text_len == 0 || !conf)
        return;

    copy_cstr_trunc(raw_text, sizeof(raw_text), text);

    tr_idx = find_or_create_ocr_track(ctx, box);
    if (tr_idx < 0)
        return;
    tr = &ctx->ocr_tracks[tr_idx];

    tr->used = true;
    tr->ttl = 10;
    tr->last_seq = frame_seq;
    tr->box = *box;

    if (raw_text[0] != '\0') {
        int pos = tr->hist_next;
        char toks[MAX_PLATE_TOKENS][MAX_UTF8_TOKEN_BYTES];
        int tok_n;

        copy_cstr_trunc(tr->hist[pos].text, sizeof(tr->hist[pos].text), raw_text);
        tr->hist[pos].conf = fmaxf(0.0f, fminf(1.0f, *conf));
        tr->hist[pos].frame_seq = frame_seq;
        tr->hist_next = (tr->hist_next + 1) % OCR_TRACK_HIST;
        if (tr->hist_count < OCR_TRACK_HIST)
            tr->hist_count++;

        tok_n = split_utf8_tokens(raw_text, toks, MAX_PLATE_TOKENS);
        if (tok_n > 0 && utf8_token_is_cjk(toks[0])) {
            if (tr->province_tok[0] == '\0' || strcmp(tr->province_tok, toks[0]) == 0) {
                copy_cstr_trunc(tr->province_tok, sizeof(tr->province_tok), toks[0]);
                tr->province_score = fminf(4.0f, tr->province_score + tr->hist[pos].conf);
            } else if (tr->hist[pos].conf >= tr->province_score * 0.80f) {
                copy_cstr_trunc(tr->province_tok, sizeof(tr->province_tok), toks[0]);
                tr->province_score = tr->hist[pos].conf;
            }
        }
    }

    if (tr->hist_count < 2)
        return;

    memset(len_vote, 0, sizeof(len_vote));
    for (k = 0; k < tr->hist_count && sample_count < OCR_TRACK_HIST; k++) {
        int idx = (tr->hist_next - 1 - k + OCR_TRACK_HIST) % OCR_TRACK_HIST;
        float recency = 1.0f - 0.08f * (float)k;
        int n;
        if (tr->hist[idx].text[0] == '\0')
            continue;
        if (recency < 0.45f)
            recency = 0.45f;
        sample_w[sample_count] = tr->hist[idx].conf * recency;
        n = split_utf8_tokens(tr->hist[idx].text, sample_tokens[sample_count], MAX_PLATE_TOKENS);
        sample_lens[sample_count] = n;
        if (n > 0 && n <= MAX_PLATE_TOKENS)
            len_vote[n] += sample_w[sample_count];
        sample_count++;
    }
    if (sample_count == 0)
        return;

    for (i = 1; i <= MAX_PLATE_TOKENS; i++) {
        if (len_vote[i] > target_w) {
            target_w = len_vote[i];
            target_len = i;
        }
    }
    if (target_len <= 0)
        return;

    smooth[0] = '\0';
    for (i = 0; i < target_len; i++) {
        char cand_tok[OCR_TRACK_HIST][MAX_UTF8_TOKEN_BYTES];
        float cand_w[OCR_TRACK_HIST];
        int cand_n = 0;
        float pos_total_w = 0.0f;
        float pos_best_w = -1.0f;
        int best_j = -1;

        memset(cand_w, 0, sizeof(cand_w));
        for (k = 0; k < sample_count; k++) {
            const char *tok;
            float w;
            int j;
            if (sample_lens[k] <= i)
                continue;
            tok = sample_tokens[k][i];
            w = sample_w[k];
            if (i == 0) {
                if (utf8_token_is_cjk(tok))
                    w *= 1.25f;
                else if (utf8_token_is_ascii_alnum(tok))
                    w *= 0.75f;
            } else if (utf8_token_is_cjk(tok)) {
                w *= 0.90f;
            }
            pos_total_w += w;
            for (j = 0; j < cand_n; j++) {
                if (strcmp(cand_tok[j], tok) == 0) {
                    cand_w[j] += w;
                    break;
                }
            }
            if (j == cand_n && cand_n < OCR_TRACK_HIST) {
                copy_cstr_trunc(cand_tok[cand_n], MAX_UTF8_TOKEN_BYTES, tok);
                cand_w[cand_n] = w;
                cand_n++;
            }
        }
        if (cand_n == 0)
            break;
        for (k = 0; k < cand_n; k++) {
            if (cand_w[k] > pos_best_w) {
                pos_best_w = cand_w[k];
                best_j = k;
            }
        }
        if (best_j < 0 || !append_utf8_token(smooth, sizeof(smooth), cand_tok[best_j]))
            break;
        if (pos_total_w > 1e-6f) {
            smooth_ratio_sum += pos_best_w / pos_total_w;
            smooth_ratio_n++;
        }
    }
    if (smooth[0] == '\0')
        return;

    first_tok[0] = '\0';
    smooth_first[0] = '\0';
    if (split_utf8_tokens(raw_text, sample_tokens[0], MAX_PLATE_TOKENS) > 0) {
        copy_cstr_trunc(first_tok, sizeof(first_tok), sample_tokens[0][0]);
        raw_first_cjk = utf8_token_is_cjk(first_tok);
    }
    if (split_utf8_tokens(smooth, sample_tokens[1], MAX_PLATE_TOKENS) > 0) {
        copy_cstr_trunc(smooth_first, sizeof(smooth_first), sample_tokens[1][0]);
        smooth_first_cjk = utf8_token_is_cjk(smooth_first);
    }

    if (!smooth_first_cjk && tr->province_tok[0] != '\0' && tr->province_score >= 1.2f) {
        char toks[MAX_PLATE_TOKENS][MAX_UTF8_TOKEN_BYTES];
        int n = split_utf8_tokens(smooth, toks, MAX_PLATE_TOKENS);
        if (n > 0) {
            int p;
            smooth[0] = '\0';
            append_utf8_token(smooth, sizeof(smooth), tr->province_tok);
            for (p = 1; p < n; p++)
                append_utf8_token(smooth, sizeof(smooth), toks[p]);
            smooth_first_cjk = true;
        }
    }

    smooth_conf = (smooth_ratio_n > 0) ? (smooth_ratio_sum / (float)smooth_ratio_n) : *conf;
    smooth_conf = fmaxf(0.0f, fminf(1.0f, smooth_conf));

    if (strcmp(raw_text, smooth) != 0) {
        if (smooth_conf >= (*conf * 0.90f) || (!raw_first_cjk && smooth_first_cjk)) {
            float old_conf = *conf;
            copy_cstr_trunc(text, text_len, smooth);
            *conf = fmaxf(old_conf * 0.90f, smooth_conf);
            fprintf(stderr,
                    "[ocr-smooth] frame=%" PRIu64 " raw=(%.2f,%s) smooth=(%.2f,%s)\n",
                    frame_seq, old_conf, raw_text, *conf, text);
        }
    }
}

static uint8_t *prepare_ocr_input_rgb888(const struct app_ctx *ctx,
                                         const uint8_t *crop_rgb, int crop_w, int crop_h,
                                         float *occ_ratio_out)
{
    const struct ocr_model *m = &ctx->ocr_model;
    uint8_t *crop_work = NULL;
    uint8_t *ocr_in = NULL;
    struct letterbox_meta lb;
    float occ = 1.0f;

    if (occ_ratio_out)
        *occ_ratio_out = 0.0f;

    crop_work = malloc((size_t)crop_w * crop_h * 3U);
    ocr_in = malloc((size_t)m->in_w * m->in_h * 3U);
    if (!crop_work || !ocr_in) {
        free(crop_work);
        free(ocr_in);
        return NULL;
    }

    memcpy(crop_work, crop_rgb, (size_t)crop_w * crop_h * 3U);
    ocr_preprocess_rgb888(crop_work, crop_w, crop_h, ctx->opt.ocr_preproc_mode);

    if (ctx->opt.ocr_resize_mode == OCR_RESIZE_LETTERBOX) {
        memset(&lb, 0, sizeof(lb));
        resize_rgb888_letterbox_kernel(crop_work, crop_w, crop_h, ocr_in,
                                       (int)m->in_w, (int)m->in_h, 0U,
                                       ctx->opt.ocr_resize_kernel, &lb);
        if (lb.valid && m->in_w > 0) {
            int scaled_w = (int)((float)crop_w * lb.scale + 0.5f);
            if (scaled_w < 1) scaled_w = 1;
            if (scaled_w > (int)m->in_w) scaled_w = (int)m->in_w;
            occ = (float)scaled_w / (float)m->in_w;
        } else {
            occ = 0.0f;
        }
    } else {
        resize_rgb888_with_kernel(crop_work, crop_w, crop_h, ocr_in, (int)m->in_w, (int)m->in_h,
                                  ctx->opt.ocr_resize_kernel);
        occ = 1.0f;
    }
    if (occ_ratio_out)
        *occ_ratio_out = occ;

    if (ctx->opt.ocr_channel_order == OCR_CH_BGR) {
        size_t pix = (size_t)m->in_w * m->in_h;
        size_t p;
        for (p = 0; p < pix; p++) {
            uint8_t *q = ocr_in + p * 3U;
            uint8_t t = q[0];
            q[0] = q[2];
            q[2] = t;
        }
    }

    free(crop_work);
    return ocr_in;
}

static enum ocr_decode_family select_decode_family(uint32_t decode_output_idx,
                                                  enum plate_color plate_color)
{
    if (decode_output_idx == 1)
        return OCR_DECODE_FAMILY_GREEN8;
    if (plate_color == PLATE_COLOR_GREEN)
        return OCR_DECODE_FAMILY_GREEN8;
    return OCR_DECODE_FAMILY_NONE;
}

static int run_model_ocr(struct app_ctx *ctx, const uint8_t *crop_rgb, int crop_w, int crop_h,
                         enum plate_color plate_color,
                         char *text, size_t text_len, float *conf_out,
                         struct ocr_diag *diag, uint8_t **model_input_out)
{
    struct ocr_model *m = &ctx->ocr_model;
    rknn_input in;
    rknn_output outs[4];
    uint8_t *ocr_in = NULL;
    const rknn_tensor_attr *out_attr;
    uint32_t decode_output_idx = 0;
    int t_size, c_size, t_stride, c_stride;
    int ret = -1;
    uint32_t i;
    float occ_ratio = 0.0f;

    if (diag)
        memset(diag, 0, sizeof(*diag));

    ocr_in = prepare_ocr_input_rgb888(ctx, crop_rgb, crop_w, crop_h, &occ_ratio);
    if (!ocr_in) {
        return -1;
    }

    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf = (void *)ocr_in;
    in.size = m->in_w * m->in_h * 3;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    ret = rknn_inputs_set(m->ctx, 1, &in);
    if (ret < 0)
        goto out;
    ret = rknn_run(m->ctx, NULL);
    if (ret < 0)
        goto out;

    memset(outs, 0, sizeof(outs));
    for (i = 0; i < m->io_num.n_output; i++)
        outs[i].want_float = 1;
    ret = rknn_outputs_get(m->ctx, m->io_num.n_output, outs, NULL);
    if (ret < 0)
        goto out;

    if (m->io_num.n_output >= 2)
        decode_output_idx = 1; /* 临时 green8 测试模式：多头时优先取 green8 head */
    out_attr = &m->output_attrs[decode_output_idx];
    if (!build_ocr_layout(out_attr, &t_size, &c_size, &t_stride, &c_stride)) {
        ret = -1;
        goto out_release;
    }
    if (ctx->ocr_blank_index < 0 || ctx->ocr_blank_index >= c_size) {
        if (c_size == ctx->ocr_key_count + 1)
            ctx->ocr_blank_index = ctx->ocr_key_count;
        else
            ctx->ocr_blank_index = c_size - 1;
    }
    if (!ctx->ocr_keysize_warned) {
        if (!(c_size == ctx->ocr_key_count || c_size == (ctx->ocr_key_count + 1))) {
            fprintf(stderr,
                    "[ocr] WARN key/output mismatch: keys=%d c_size=%d (expected N or N+1)\n",
                    ctx->ocr_key_count, c_size);
        }
        fprintf(stderr,
                "[ocr] decode_output_idx=%u/%u%s\n",
                decode_output_idx,
                m->io_num.n_output,
                (m->io_num.n_output >= 2) ? " (temporary green8 head test mode)" : "");
        ctx->ocr_keysize_warned = true;
    }
    {
        enum ocr_decode_family family = select_decode_family(decode_output_idx, plate_color);
        ret = ctc_decode_logits((const float *)outs[decode_output_idx].buf, t_size, c_size, t_stride, c_stride,
                                ctx, family, text, text_len, conf_out, diag);
    }
    if (diag)
        diag->in_occ_ratio = occ_ratio;
    fprintf(stderr, "[ocrin] resize_mode=%s kernel=%s in_occ_ratio=%.3f\n",
            (ctx->opt.ocr_resize_mode == OCR_RESIZE_LETTERBOX) ? "letterbox" : "stretch",
            (ctx->opt.ocr_resize_kernel == OCR_KERNEL_BILINEAR) ? "bilinear" : "nn",
            occ_ratio);

out_release:
    rknn_outputs_release(m->ctx, m->io_num.n_output, outs);
out:
    if (model_input_out && ret == 0) {
        *model_input_out = malloc((size_t)m->in_w * m->in_h * 3U);
        if (*model_input_out)
            memcpy(*model_input_out, ocr_in, (size_t)m->in_w * m->in_h * 3U);
    }
    free(ocr_in);
    return ret;
}

static int init_fpga_dma(struct app_ctx *ctx)
{
    struct fpga_info info;
    struct buffer_map map;
    uint32_t inferred_format;

    ctx->dev_fd = open(ctx->opt.device_path, O_RDWR | O_CLOEXEC);
    if (ctx->dev_fd < 0)
        return -1;
    if (ioctl(ctx->dev_fd, FPGA_DMA_GET_INFO, &info) < 0)
        return -1;
    if (info.frame_width != 1280 || info.frame_height != 720)
        return -1;

    inferred_format = info.pixel_format;

    if (inferred_format != FPGA_PIXEL_FORMAT_BGR565 &&
        inferred_format != FPGA_PIXEL_FORMAT_BGRX8888) {
        if (info.frame_bpp == 4)
            inferred_format = FPGA_PIXEL_FORMAT_BGRX8888;
        else
            inferred_format = FPGA_PIXEL_FORMAT_BGR565;
    }
    if (inferred_format == FPGA_PIXEL_FORMAT_BGRX8888)
        info.frame_bpp = 4;
    else
        info.frame_bpp = 2;

    ctx->frame_width = info.frame_width;
    ctx->frame_height = info.frame_height;
    ctx->src_frame_bpp = info.frame_bpp;
    ctx->src_is_bgrx = (inferred_format == FPGA_PIXEL_FORMAT_BGRX8888) || (info.frame_bpp == 4);
    ctx->src_frame_size = (size_t)ctx->frame_width * ctx->frame_height * ctx->src_frame_bpp;
    /* Internal pipeline keeps BGR565 for overlay and drawing. */
    ctx->frame_bpp = 2;
    ctx->frame_size = (size_t)ctx->frame_width * ctx->frame_height * ctx->frame_bpp;

    if (ctx->src_is_bgrx)
        ctx->opt.swap16 = false;

    memset(&map, 0, sizeof(map));
    map.index = 0;
    if (ioctl(ctx->dev_fd, FPGA_DMA_MAP_BUFFER, &map) < 0)
        return -1;
    if (map.size < ctx->src_frame_size)
        return -1;
    ctx->dma_map_size = map.size;
    ctx->dma_map = mmap(NULL, ctx->dma_map_size, PROT_READ, MAP_SHARED, ctx->dev_fd, 0);
    if (ctx->dma_map == MAP_FAILED) {
        ctx->dma_map = NULL;
        return -1;
    }

    ctx->dma_copy = malloc(ctx->src_frame_size);
    if (!ctx->dma_copy)
        return -1;
    return 0;
}

static int trigger_frame_dma(struct app_ctx *ctx)
{
    struct dma_transfer t;
    memset(&t, 0, sizeof(t));
    t.size = (uint32_t)ctx->src_frame_size;
    t.user_buf = (uint64_t)(uintptr_t)ctx->dma_copy;
    if (ioctl(ctx->dev_fd, FPGA_DMA_READ_FRAME, &t) < 0)
        return -1;
    return t.result == 0 ? 0 : -1;
}

static int init_copy_slots(struct app_ctx *ctx)
{
    int i;
    ctx->slots = calloc((size_t)ctx->opt.copy_buffers, sizeof(*ctx->slots));
    if (!ctx->slots)
        return -1;
    ctx->slot_count = ctx->opt.copy_buffers;
    for (i = 0; i < ctx->slot_count; i++) {
        ctx->slots[i].data = malloc(ctx->frame_size);
        if (!ctx->slots[i].data)
            return -1;
    }
    return 0;
}

static void copy_frame_to_slot565(struct app_ctx *ctx, uint8_t *dst, const uint8_t *src)
{
    size_t i;
    size_t pixels = (size_t)ctx->frame_width * ctx->frame_height;

    if (ctx->src_is_bgrx) {
        for (i = 0; i < pixels; i++) {
            const uint8_t *p = src + i * 4U;
            uint8_t b = p[0];
            uint8_t g = p[1];
            uint8_t r = p[2];
            uint16_t pix565;

            if (ctx->opt.pixel_order == PIXEL_ORDER_BGR565)
                pix565 = (uint16_t)(((b >> 3) << 11) | ((g >> 2) << 5) | (r >> 3));
            else
                pix565 = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));

            dst[i * 2U + 0] = (uint8_t)(pix565 & 0xFF);
            dst[i * 2U + 1] = (uint8_t)(pix565 >> 8);
        }
        return;
    }

    if (!ctx->opt.swap16) {
        memcpy(dst, src, ctx->frame_size);
        return;
    }
    for (i = 0; i + 1 < ctx->frame_size; i += 2) {
        dst[i] = src[i + 1];
        dst[i + 1] = src[i];
    }
}

static void draw_hline_565(uint16_t *pix, int w, int h, int x1, int x2, int y, uint16_t c)
{
    int x;
    if (y < 0 || y >= h)
        return;
    if (x1 > x2) { int t = x1; x1 = x2; x2 = t; }
    if (x1 < 0) x1 = 0;
    if (x2 >= w) x2 = w - 1;
    for (x = x1; x <= x2; x++)
        pix[y * w + x] = c;
}

static void draw_vline_565(uint16_t *pix, int w, int h, int x, int y1, int y2, uint16_t c)
{
    int y;
    if (x < 0 || x >= w)
        return;
    if (y1 > y2) { int t = y1; y1 = y2; y2 = t; }
    if (y1 < 0) y1 = 0;
    if (y2 >= h) y2 = h - 1;
    for (y = y1; y <= y2; y++)
        pix[y * w + x] = c;
}

static void draw_rect_565(uint16_t *pix, int w, int h, const struct det_box *b, uint16_t c)
{
    int t;
    for (t = 0; t < 2; t++) {
        draw_hline_565(pix, w, h, b->x1, b->x2, b->y1 + t, c);
        draw_hline_565(pix, w, h, b->x1, b->x2, b->y2 - t, c);
        draw_vline_565(pix, w, h, b->x1 + t, b->y1, b->y2, c);
        draw_vline_565(pix, w, h, b->x2 - t, b->y1, b->y2, c);
    }
}

static uint8_t glyph5x7(char ch, int row)
{
    if (ch >= 'a' && ch <= 'z')
        ch = (char)(ch - 'a' + 'A');
    switch (ch) {
    case '0': { static const uint8_t g[7]={0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; return g[row]; }
    case '1': { static const uint8_t g[7]={0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
    case '2': { static const uint8_t g[7]={0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}; return g[row]; }
    case '3': { static const uint8_t g[7]={0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case '4': { static const uint8_t g[7]={0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}; return g[row]; }
    case '5': { static const uint8_t g[7]={0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}; return g[row]; }
    case '6': { static const uint8_t g[7]={0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}; return g[row]; }
    case '7': { static const uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x08,0x08}; return g[row]; }
    case '8': { static const uint8_t g[7]={0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}; return g[row]; }
    case '9': { static const uint8_t g[7]={0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}; return g[row]; }
    case 'A': { static const uint8_t g[7]={0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'B': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}; return g[row]; }
    case 'C': { static const uint8_t g[7]={0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}; return g[row]; }
    case 'D': { static const uint8_t g[7]={0x1C,0x12,0x11,0x11,0x11,0x12,0x1C}; return g[row]; }
    case 'E': { static const uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}; return g[row]; }
    case 'F': { static const uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'G': { static const uint8_t g[7]={0x0F,0x10,0x10,0x13,0x11,0x11,0x0F}; return g[row]; }
    case 'H': { static const uint8_t g[7]={0x11,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'I': { static const uint8_t g[7]={0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
    case 'J': { static const uint8_t g[7]={0x01,0x01,0x01,0x01,0x11,0x11,0x0E}; return g[row]; }
    case 'K': { static const uint8_t g[7]={0x11,0x12,0x14,0x18,0x14,0x12,0x11}; return g[row]; }
    case 'L': { static const uint8_t g[7]={0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return g[row]; }
    case 'M': { static const uint8_t g[7]={0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; return g[row]; }
    case 'N': { static const uint8_t g[7]={0x11,0x19,0x15,0x13,0x11,0x11,0x11}; return g[row]; }
    case 'O': { static const uint8_t g[7]={0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'P': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'Q': { static const uint8_t g[7]={0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}; return g[row]; }
    case 'R': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return g[row]; }
    case 'S': { static const uint8_t g[7]={0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case 'T': { static const uint8_t g[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'U': { static const uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'V': { static const uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x0A,0x04}; return g[row]; }
    case 'W': { static const uint8_t g[7]={0x11,0x11,0x11,0x15,0x15,0x1B,0x11}; return g[row]; }
    case 'X': { static const uint8_t g[7]={0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}; return g[row]; }
    case 'Y': { static const uint8_t g[7]={0x11,0x11,0x0A,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'Z': { static const uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}; return g[row]; }
    case '-': { static const uint8_t g[7]={0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; return g[row]; }
    case '_': { static const uint8_t g[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x1F}; return g[row]; }
    case ':': { static const uint8_t g[7]={0x00,0x04,0x04,0x00,0x04,0x04,0x00}; return g[row]; }
    case ' ': { static const uint8_t g[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00}; return g[row]; }
    default: return 0;
    }
}

static void draw_text_565(uint16_t *pix, int w, int h, int x, int y, const char *s, uint16_t c, int scale)
{
    int i;
    int advance;
    if (!s || scale < 1)
        return;
    advance = 6 * scale;
    for (i = 0; s[i] != '\0'; i++) {
        int row;
        int col;
        int ox = x + i * advance;
        for (row = 0; row < 7; row++) {
            uint8_t bits = glyph5x7(s[i], row);
            for (col = 0; col < 5; col++) {
                if (bits & (1U << (4 - col))) {
                    int sy;
                    int sx;
                    int px0 = ox + col * scale;
                    int py0 = y + row * scale;
                    for (sy = 0; sy < scale; sy++) {
                        int py = py0 + sy;
                        if (py < 0 || py >= h)
                            continue;
                        for (sx = 0; sx < scale; sx++) {
                            int px = px0 + sx;
                            if (px >= 0 && px < w)
                                pix[py * w + px] = c;
                        }
                    }
                }
            }
        }
    }
}

static void release_slot_ticket(struct app_ctx *ctx, const struct slot_ticket *ticket, bool count_release)
{
    g_mutex_lock(&ctx->slots_lock);
    if (ticket->idx >= 0 && ticket->idx < ctx->slot_count &&
        ctx->slots[ticket->idx].in_use &&
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
    ticket->idx = -1;
    ticket->generation = 0;
    for (;;) {
        int i;
        int64_t now;
        int64_t wake_us;
        g_mutex_lock(&ctx->slots_lock);
        for (i = 0; i < ctx->slot_count; i++) {
            if (!ctx->slots[i].in_use) {
                ctx->slots[i].in_use = true;
                ctx->slots[i].generation++;
                ticket->idx = i;
                ticket->generation = ctx->slots[i].generation;
                g_mutex_unlock(&ctx->slots_lock);
                return 0;
            }
        }
        now = mono_us();
        if (now >= deadline_us) {
            g_mutex_unlock(&ctx->slots_lock);
            ctx->slot_wait_timeout_count++;
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
    struct frame_cookie *cookie = g_new0(struct frame_cookie, 1);
    GstBuffer *buf;
    if (!cookie)
        return NULL;
    cookie->ctx = ctx;
    cookie->ticket = *ticket;
    buf = gst_buffer_new_wrapped_full((GstMemoryFlags)0,
                                      ctx->slots[ticket->idx].data, ctx->frame_size,
                                      0, ctx->frame_size, cookie, frame_release_notify);
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

static int handle_bus_messages(struct app_ctx *ctx)
{
    GstMessage *msg;
    while ((msg = gst_bus_pop(ctx->bus)) != NULL) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
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
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
            ctx->running = false;
            gst_message_unref(msg);
            return -1;
        }
        gst_message_unref(msg);
    }
    return 0;
}

static void print_pad_caps(const char *label, GstElement *elem, const char *pad_name)
{
    GstPad *pad = gst_element_get_static_pad(elem, pad_name);
    GstCaps *caps;
    gchar *s;
    if (!pad)
        return;
    caps = gst_pad_get_current_caps(pad);
    if (!caps)
        caps = gst_pad_query_caps(pad, NULL);
    if (caps) {
        s = gst_caps_to_string(caps);
        fprintf(stderr, "%s caps: %s\n", label, s ? s : "<null>");
        g_free(s);
        gst_caps_unref(caps);
    }
    gst_object_unref(pad);
}

static int build_pipeline(struct app_ctx *ctx)
{
    GstCaps *caps;
    GstStateChangeReturn sret;
    const char *fmt = (ctx->opt.pixel_order == PIXEL_ORDER_BGR565) ? "BGR16" : "RGB16";
    ctx->pipeline = gst_pipeline_new("fpga-lpr");
    ctx->appsrc = gst_element_factory_make("appsrc", "src");
    ctx->queue = gst_element_factory_make("queue", "latency_queue");
    ctx->sink = gst_element_factory_make("kmssink", "sink");
    if (!ctx->pipeline || !ctx->appsrc || !ctx->queue || !ctx->sink)
        return -1;

    gst_bin_add_many(GST_BIN(ctx->pipeline), ctx->appsrc, ctx->queue, ctx->sink, NULL);
    if (!gst_element_link_many(ctx->appsrc, ctx->queue, ctx->sink, NULL))
        return -1;

    caps = gst_caps_new_simple("video/x-raw",
                               "format", G_TYPE_STRING, fmt,
                               "width", G_TYPE_INT, (int)ctx->frame_width,
                               "height", G_TYPE_INT, (int)ctx->frame_height,
                               "framerate", GST_TYPE_FRACTION, ctx->opt.fps, 1,
                               NULL);
    if (!caps)
        return -1;
    g_object_set(ctx->appsrc,
                 "caps", caps,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "format", GST_FORMAT_TIME,
                 "block", FALSE,
                 "max-bytes", (guint64)ctx->frame_size * (guint64)ctx->opt.queue_depth,
                 NULL);
    gst_caps_unref(caps);

    g_object_set(ctx->queue,
                 "max-size-buffers", 1,
                 "max-size-bytes", 0,
                 "max-size-time", (guint64)0,
                 "leaky", 2,
                 NULL);
    g_object_set(ctx->sink, "sync", FALSE, NULL);
    if (ctx->opt.connector_id >= 0)
        g_object_set(ctx->sink, "connector-id", ctx->opt.connector_id, NULL);
    if (ctx->drm_fd >= 0)
        g_object_set(ctx->sink, "fd", ctx->drm_fd, NULL);

    ctx->bus = gst_element_get_bus(ctx->pipeline);
    sret = gst_element_set_state(ctx->pipeline, GST_STATE_PLAYING);
    if (sret == GST_STATE_CHANGE_FAILURE)
        return -1;
    sret = gst_element_get_state(ctx->pipeline, NULL, NULL, 5 * GST_SECOND);
    if (sret == GST_STATE_CHANGE_FAILURE)
        return -1;
    fprintf(stderr, "Pipeline started: appsrc(%s)->queue(leaky)->kmssink\n", fmt);
    print_pad_caps("appsrc:src", ctx->appsrc, "src");
    print_pad_caps("kmssink:sink", ctx->sink, "sink");
    return 0;
}

static float sigmoidf_local(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

struct point2f {
    float x;
    float y;
};

static float box_iou(const struct det_box *a, const struct det_box *b)
{
    int x1 = a->x1 > b->x1 ? a->x1 : b->x1;
    int y1 = a->y1 > b->y1 ? a->y1 : b->y1;
    int x2 = a->x2 < b->x2 ? a->x2 : b->x2;
    int y2 = a->y2 < b->y2 ? a->y2 : b->y2;
    int iw = x2 - x1 + 1;
    int ih = y2 - y1 + 1;
    int ia;
    int ua;
    if (iw <= 0 || ih <= 0)
        return 0.0f;
    ia = iw * ih;
    ua = (a->x2 - a->x1 + 1) * (a->y2 - a->y1 + 1) +
         (b->x2 - b->x1 + 1) * (b->y2 - b->y1 + 1) - ia;
    if (ua <= 0)
        return 0.0f;
    return (float)ia / (float)ua;
}

static int det_conf_cmp(const void *pa, const void *pb)
{
    const struct det_box *a = pa;
    const struct det_box *b = pb;
    if (a->conf < b->conf) return 1;
    if (a->conf > b->conf) return -1;
    return 0;
}

static void nms_inplace(struct det_box *dets, int *count, float iou_thr)
{
    bool removed[MAX_DETS * 4];
    int n = *count;
    int i;
    int j;
    int out = 0;
    if (n < 0)
        n = 0;
    if (n > (int)(sizeof(removed) / sizeof(removed[0])))
        n = (int)(sizeof(removed) / sizeof(removed[0]));
    memset(removed, 0, sizeof(removed));
    qsort(dets, (size_t)n, sizeof(dets[0]), det_conf_cmp);
    for (i = 0; i < n; i++) {
        if (removed[i]) continue;
        dets[out++] = dets[i];
        for (j = i + 1; j < n; j++) {
            if (removed[j] || dets[i].cls != dets[j].cls)
                continue;
            if (box_iou(&dets[i], &dets[j]) > iou_thr)
                removed[j] = true;
        }
    }
    *count = out;
}

static void det_quad_from_obb(struct det_box *d)
{
    float c = cosf(d->angle);
    float s = sinf(d->angle);
    float hw = d->w * 0.5f;
    float hh = d->h * 0.5f;
    static const float local[4][2] = {
        {-1.0f, -1.0f},
        { 1.0f, -1.0f},
        { 1.0f,  1.0f},
        {-1.0f,  1.0f},
    };
    int i;
    float min_x = 1e30f, min_y = 1e30f;
    float max_x = -1e30f, max_y = -1e30f;
    for (i = 0; i < 4; i++) {
        float lx = local[i][0] * hw;
        float ly = local[i][1] * hh;
        float x = d->cx + lx * c - ly * s;
        float y = d->cy + lx * s + ly * c;
        d->quad[i * 2 + 0] = x;
        d->quad[i * 2 + 1] = y;
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
    }
    d->x1 = (int)floorf(min_x);
    d->y1 = (int)floorf(min_y);
    d->x2 = (int)ceilf(max_x);
    d->y2 = (int)ceilf(max_y);
    d->has_obb = 1;
}

static float polygon_area_signed(const struct point2f *pts, int n)
{
    float acc = 0.0f;
    int i;
    for (i = 0; i < n; i++) {
        const struct point2f *a = &pts[i];
        const struct point2f *b = &pts[(i + 1) % n];
        acc += a->x * b->y - b->x * a->y;
    }
    return 0.5f * acc;
}

static float polygon_area_abs(const struct point2f *pts, int n)
{
    float a = polygon_area_signed(pts, n);
    return (a >= 0.0f) ? a : -a;
}

static bool clip_inside(const struct point2f *p, const struct point2f *a,
                        const struct point2f *b, float orient_sign)
{
    float cross = (b->x - a->x) * (p->y - a->y) - (b->y - a->y) * (p->x - a->x);
    if (orient_sign >= 0.0f)
        return cross >= -1e-6f;
    return cross <= 1e-6f;
}

static struct point2f line_intersection(const struct point2f *s, const struct point2f *e,
                                        const struct point2f *a, const struct point2f *b)
{
    struct point2f out = *e;
    float x1 = s->x, y1 = s->y;
    float x2 = e->x, y2 = e->y;
    float x3 = a->x, y3 = a->y;
    float x4 = b->x, y4 = b->y;
    float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (fabsf(den) < 1e-8f)
        return out;
    out.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den;
    out.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den;
    return out;
}

static int convex_clip_polygon(const struct point2f *subject, int subject_n,
                               const struct point2f *clip, int clip_n,
                               struct point2f *out, int out_cap)
{
    struct point2f in_buf[16];
    struct point2f out_buf[16];
    int in_n = subject_n;
    int i;
    if (subject_n <= 0 || clip_n <= 0 || out_cap <= 0)
        return 0;
    if (subject_n > 16 || clip_n > 16)
        return 0;
    memcpy(in_buf, subject, (size_t)subject_n * sizeof(subject[0]));
    for (i = 0; i < clip_n; i++) {
        struct point2f a = clip[i];
        struct point2f b = clip[(i + 1) % clip_n];
        float orient = polygon_area_signed(clip, clip_n);
        int j;
        int out_n = 0;
        if (in_n <= 0)
            return 0;
        for (j = 0; j < in_n; j++) {
            struct point2f cur = in_buf[j];
            struct point2f prev = in_buf[(j + in_n - 1) % in_n];
            bool cur_in = clip_inside(&cur, &a, &b, orient);
            bool prev_in = clip_inside(&prev, &a, &b, orient);
            if (cur_in) {
                if (!prev_in && out_n < 16)
                    out_buf[out_n++] = line_intersection(&prev, &cur, &a, &b);
                if (out_n < 16)
                    out_buf[out_n++] = cur;
            } else if (prev_in) {
                if (out_n < 16)
                    out_buf[out_n++] = line_intersection(&prev, &cur, &a, &b);
            }
        }
        memcpy(in_buf, out_buf, (size_t)out_n * sizeof(out_buf[0]));
        in_n = out_n;
    }
    if (in_n > out_cap)
        in_n = out_cap;
    memcpy(out, in_buf, (size_t)in_n * sizeof(out[0]));
    return in_n;
}

static void det_to_quad_points(const struct det_box *d, struct point2f q[4])
{
    if (d->has_obb) {
        int i;
        for (i = 0; i < 4; i++) {
            q[i].x = d->quad[i * 2 + 0];
            q[i].y = d->quad[i * 2 + 1];
        }
        return;
    }
    q[0].x = (float)d->x1; q[0].y = (float)d->y1;
    q[1].x = (float)d->x2; q[1].y = (float)d->y1;
    q[2].x = (float)d->x2; q[2].y = (float)d->y2;
    q[3].x = (float)d->x1; q[3].y = (float)d->y2;
}

static float rotated_iou(const struct det_box *a, const struct det_box *b)
{
    struct point2f qa[4];
    struct point2f qb[4];
    struct point2f inter[16];
    float area_a;
    float area_b;
    float inter_area;
    float denom;
    int inter_n;
    if (box_iou(a, b) <= 0.0f)
        return 0.0f;
    det_to_quad_points(a, qa);
    det_to_quad_points(b, qb);
    area_a = polygon_area_abs(qa, 4);
    area_b = polygon_area_abs(qb, 4);
    if (area_a <= 1e-6f || area_b <= 1e-6f)
        return 0.0f;
    inter_n = convex_clip_polygon(qa, 4, qb, 4, inter, 16);
    if (inter_n <= 2)
        return 0.0f;
    inter_area = polygon_area_abs(inter, inter_n);
    if (inter_area <= 1e-6f)
        return 0.0f;
    denom = area_a + area_b - inter_area;
    if (denom <= 1e-6f)
        return 0.0f;
    return inter_area / denom;
}

static void rotated_nms_inplace(struct det_box *dets, int *count, float iou_thr, int max_det)
{
    bool removed[MAX_DETS * 4];
    int n = *count;
    int i;
    int j;
    int out = 0;
    if (max_det <= 0 || max_det > MAX_DETS)
        max_det = MAX_DETS;
    if (n < 0)
        n = 0;
    if (n > (int)(sizeof(removed) / sizeof(removed[0])))
        n = (int)(sizeof(removed) / sizeof(removed[0]));
    memset(removed, 0, sizeof(removed));
    qsort(dets, (size_t)n, sizeof(dets[0]), det_conf_cmp);
    for (i = 0; i < n && out < max_det; i++) {
        if (removed[i]) continue;
        dets[out++] = dets[i];
        for (j = i + 1; j < n; j++) {
            if (removed[j] || dets[i].cls != dets[j].cls)
                continue;
            if (rotated_iou(&dets[i], &dets[j]) > iou_thr)
                removed[j] = true;
        }
    }
    *count = out;
}

static const char *tensor_fmt_name(rknn_tensor_format fmt)
{
    switch (fmt) {
    case RKNN_TENSOR_NCHW: return "NCHW";
    case RKNN_TENSOR_NHWC: return "NHWC";
    default: return "UNSPEC";
    }
}

static const char *tensor_type_name(rknn_tensor_type t)
{
    switch (t) {
#ifdef RKNN_TENSOR_FLOAT32
    case RKNN_TENSOR_FLOAT32: return "f32";
#endif
#ifdef RKNN_TENSOR_FLOAT16
    case RKNN_TENSOR_FLOAT16: return "f16";
#endif
#ifdef RKNN_TENSOR_INT8
    case RKNN_TENSOR_INT8: return "i8";
#endif
#ifdef RKNN_TENSOR_UINT8
    case RKNN_TENSOR_UINT8: return "u8";
#endif
#ifdef RKNN_TENSOR_INT16
    case RKNN_TENSOR_INT16: return "i16";
#endif
#ifdef RKNN_TENSOR_UINT16
    case RKNN_TENSOR_UINT16: return "u16";
#endif
#ifdef RKNN_TENSOR_INT32
    case RKNN_TENSOR_INT32: return "i32";
#endif
#ifdef RKNN_TENSOR_UINT32
    case RKNN_TENSOR_UINT32: return "u32";
#endif
    default: return "other";
    }
}

static int rknn_model_load(struct yolo_model *m, const char *name, const char *path,
                           int class_count, int detector_type)
{
    FILE *fp;
    long sz;
    void *data;
    uint32_t i;
    memset(m, 0, sizeof(*m));
    m->name = name;
    m->path = path;
    m->class_count = class_count;
    m->detector_type = detector_type;
    m->nms_iou_thr = 0.45f;
    m->max_det = MAX_DETS;
    m->class_filter = -1;

    fp = fopen(path, "rb");
    if (!fp) return -1;
    fseek(fp, 0, SEEK_END);
    sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    data = malloc((size_t)sz);
    if (!data) { fclose(fp); return -1; }
    if (fread(data, 1, (size_t)sz, fp) != (size_t)sz) { fclose(fp); free(data); return -1; }
    fclose(fp);

    if (rknn_init(&m->ctx, data, (uint32_t)sz, 0, NULL) < 0) { free(data); return -1; }
    free(data);
    if (rknn_query(m->ctx, RKNN_QUERY_IN_OUT_NUM, &m->io_num, sizeof(m->io_num)) < 0)
        return -1;
    memset(&m->input_attr, 0, sizeof(m->input_attr));
    m->input_attr.index = 0;
    if (rknn_query(m->ctx, RKNN_QUERY_INPUT_ATTR, &m->input_attr, sizeof(m->input_attr)) < 0)
        return -1;
    if (m->input_attr.fmt == RKNN_TENSOR_NCHW) {
        m->in_c = m->input_attr.dims[1];
        m->in_h = m->input_attr.dims[2];
        m->in_w = m->input_attr.dims[3];
    } else {
        m->in_h = m->input_attr.dims[1];
        m->in_w = m->input_attr.dims[2];
        m->in_c = m->input_attr.dims[3];
    }
    if (m->io_num.n_output > 8) return -1;
    for (i = 0; i < m->io_num.n_output; i++) {
        memset(&m->output_attrs[i], 0, sizeof(m->output_attrs[i]));
        m->output_attrs[i].index = i;
        if (rknn_query(m->ctx, RKNN_QUERY_OUTPUT_ATTR, &m->output_attrs[i], sizeof(m->output_attrs[i])) < 0)
            return -1;
    }
    fprintf(stderr, "[%s] loaded input=%ux%ux%u outputs=%u detector=%s\n",
            name, m->in_w, m->in_h, m->in_c, m->io_num.n_output,
            detector_type_str(m->detector_type));
    for (i = 0; i < m->io_num.n_output; i++) {
        const rknn_tensor_attr *a = &m->output_attrs[i];
        fprintf(stderr,
                "  out[%u]: dims=%u x %u x %u x %u n_dims=%u fmt=%s type=%s qnt=%d zp=%d scale=%.6f\n",
                i,
                a->dims[0], a->dims[1], a->dims[2], a->dims[3], a->n_dims,
                tensor_fmt_name(a->fmt), tensor_type_name(a->type), a->qnt_type, a->zp, a->scale);
    }
    return 0;
}

static void rknn_model_release(struct yolo_model *m)
{
    if (m->ctx)
        rknn_destroy(m->ctx);
    memset(m, 0, sizeof(*m));
}

static void resize_rgb888_nn(const uint8_t *src, int sw, int sh, uint8_t *dst, int dw, int dh)
{
    int x, y;
    for (y = 0; y < dh; y++) {
        int sy = (y * sh) / dh;
        for (x = 0; x < dw; x++) {
            int sx = (x * sw) / dw;
            const uint8_t *p = src + (sy * sw + sx) * 3;
            uint8_t *q = dst + (y * dw + x) * 3;
            q[0] = p[0];
            q[1] = p[1];
            q[2] = p[2];
        }
    }
}

static void resize_rgb888_bilinear(const uint8_t *src, int sw, int sh, uint8_t *dst, int dw, int dh)
{
    int x, y;
    if (!src || !dst || sw <= 0 || sh <= 0 || dw <= 0 || dh <= 0)
        return;
    if (sw == 1 || sh == 1) {
        resize_rgb888_nn(src, sw, sh, dst, dw, dh);
        return;
    }
    for (y = 0; y < dh; y++) {
        float fy = ((float)y + 0.5f) * (float)sh / (float)dh - 0.5f;
        int y0 = (int)floorf(fy);
        int y1;
        float wy;
        if (y0 < 0) y0 = 0;
        if (y0 > sh - 1) y0 = sh - 1;
        y1 = y0 + 1;
        if (y1 > sh - 1) y1 = sh - 1;
        wy = fy - (float)y0;
        if (wy < 0.0f) wy = 0.0f;
        if (wy > 1.0f) wy = 1.0f;
        for (x = 0; x < dw; x++) {
            float fx = ((float)x + 0.5f) * (float)sw / (float)dw - 0.5f;
            int x0 = (int)floorf(fx);
            int x1;
            float wx;
            const uint8_t *p00, *p01, *p10, *p11;
            uint8_t *q;
            int c;

            if (x0 < 0) x0 = 0;
            if (x0 > sw - 1) x0 = sw - 1;
            x1 = x0 + 1;
            if (x1 > sw - 1) x1 = sw - 1;
            wx = fx - (float)x0;
            if (wx < 0.0f) wx = 0.0f;
            if (wx > 1.0f) wx = 1.0f;

            p00 = src + ((size_t)y0 * (size_t)sw + (size_t)x0) * 3U;
            p01 = src + ((size_t)y0 * (size_t)sw + (size_t)x1) * 3U;
            p10 = src + ((size_t)y1 * (size_t)sw + (size_t)x0) * 3U;
            p11 = src + ((size_t)y1 * (size_t)sw + (size_t)x1) * 3U;
            q = dst + ((size_t)y * (size_t)dw + (size_t)x) * 3U;
            for (c = 0; c < 3; c++) {
                float v0 = p00[c] * (1.0f - wx) + p01[c] * wx;
                float v1 = p10[c] * (1.0f - wx) + p11[c] * wx;
                float vv = v0 * (1.0f - wy) + v1 * wy;
                int iv = (int)(vv + 0.5f);
                if (iv < 0) iv = 0;
                if (iv > 255) iv = 255;
                q[c] = (uint8_t)iv;
            }
        }
    }
}

static void resize_rgb888_with_kernel(const uint8_t *src, int sw, int sh,
                                      uint8_t *dst, int dw, int dh, int kernel)
{
    if (kernel == OCR_KERNEL_BILINEAR)
        resize_rgb888_bilinear(src, sw, sh, dst, dw, dh);
    else
        resize_rgb888_nn(src, sw, sh, dst, dw, dh);
}

static void __attribute__((unused)) resize_rgb888_nn_letterbox(const uint8_t *src, int sw, int sh,
                                                               uint8_t *dst, int dw, int dh, uint8_t pad)
{
    int scaled_w, scaled_h;
    int off_x, off_y;
    float sx, sy, scale;
    uint8_t *tmp = NULL;
    size_t dst_sz;

    if (sw <= 0 || sh <= 0 || dw <= 0 || dh <= 0)
        return;

    dst_sz = (size_t)dw * dh * 3U;
    memset(dst, pad, dst_sz);

    sx = (float)dw / (float)sw;
    sy = (float)dh / (float)sh;
    scale = (sx < sy) ? sx : sy;
    if (scale <= 0.0f)
        return;

    scaled_w = (int)((float)sw * scale + 0.5f);
    scaled_h = (int)((float)sh * scale + 0.5f);
    if (scaled_w < 1) scaled_w = 1;
    if (scaled_h < 1) scaled_h = 1;
    if (scaled_w > dw) scaled_w = dw;
    if (scaled_h > dh) scaled_h = dh;

    off_x = (dw - scaled_w) / 2;
    off_y = (dh - scaled_h) / 2;

    tmp = malloc((size_t)scaled_w * scaled_h * 3U);
    if (!tmp)
        return;

    resize_rgb888_nn(src, sw, sh, tmp, scaled_w, scaled_h);
    for (int y = 0; y < scaled_h; y++) {
        const uint8_t *src_row = tmp + (size_t)y * scaled_w * 3U;
        uint8_t *dst_row = dst + ((size_t)(off_y + y) * dw + (size_t)off_x) * 3U;
        memcpy(dst_row, src_row, (size_t)scaled_w * 3U);
    }
    free(tmp);
}

static void resize_rgb888_letterbox_kernel(const uint8_t *src, int sw, int sh,
                                           uint8_t *dst, int dw, int dh, uint8_t pad,
                                           int kernel, struct letterbox_meta *meta)
{
    int scaled_w, scaled_h;
    int off_x, off_y;
    float sx, sy, scale;
    uint8_t *tmp = NULL;
    size_t dst_sz;

    if (meta) {
        memset(meta, 0, sizeof(*meta));
        meta->src_w = sw;
        meta->src_h = sh;
        meta->dst_w = dw;
        meta->dst_h = dh;
    }

    if (sw <= 0 || sh <= 0 || dw <= 0 || dh <= 0)
        return;

    dst_sz = (size_t)dw * (size_t)dh * 3U;
    memset(dst, pad, dst_sz);

    sx = (float)dw / (float)sw;
    sy = (float)dh / (float)sh;
    scale = (sx < sy) ? sx : sy;
    if (scale <= 0.0f)
        return;

    scaled_w = (int)((float)sw * scale + 0.5f);
    scaled_h = (int)((float)sh * scale + 0.5f);
    if (scaled_w < 1) scaled_w = 1;
    if (scaled_h < 1) scaled_h = 1;
    if (scaled_w > dw) scaled_w = dw;
    if (scaled_h > dh) scaled_h = dh;

    off_x = (dw - scaled_w) / 2;
    off_y = (dh - scaled_h) / 2;

    tmp = malloc((size_t)scaled_w * (size_t)scaled_h * 3U);
    if (!tmp)
        return;

    resize_rgb888_with_kernel(src, sw, sh, tmp, scaled_w, scaled_h, kernel);
    for (int y = 0; y < scaled_h; y++) {
        const uint8_t *src_row = tmp + (size_t)y * (size_t)scaled_w * 3U;
        uint8_t *dst_row = dst + ((size_t)(off_y + y) * (size_t)dw + (size_t)off_x) * 3U;
        memcpy(dst_row, src_row, (size_t)scaled_w * 3U);
    }
    free(tmp);

    if (meta) {
        meta->scale = scale;
        meta->pad_x = off_x;
        meta->pad_y = off_y;
        meta->valid = true;
    }
}

static void resize_rgb888_nn_letterbox_meta(const uint8_t *src, int sw, int sh,
                                            uint8_t *dst, int dw, int dh, uint8_t pad,
                                            struct letterbox_meta *meta)
{
    int scaled_w, scaled_h;
    int off_x, off_y;
    float sx, sy, scale;
    uint8_t *tmp = NULL;
    size_t dst_sz;

    if (meta) {
        memset(meta, 0, sizeof(*meta));
        meta->src_w = sw;
        meta->src_h = sh;
        meta->dst_w = dw;
        meta->dst_h = dh;
    }

    if (sw <= 0 || sh <= 0 || dw <= 0 || dh <= 0)
        return;

    dst_sz = (size_t)dw * dh * 3U;
    memset(dst, pad, dst_sz);

    sx = (float)dw / (float)sw;
    sy = (float)dh / (float)sh;
    scale = (sx < sy) ? sx : sy;
    if (scale <= 0.0f)
        return;

    scaled_w = (int)((float)sw * scale + 0.5f);
    scaled_h = (int)((float)sh * scale + 0.5f);
    if (scaled_w < 1) scaled_w = 1;
    if (scaled_h < 1) scaled_h = 1;
    if (scaled_w > dw) scaled_w = dw;
    if (scaled_h > dh) scaled_h = dh;

    off_x = (dw - scaled_w) / 2;
    off_y = (dh - scaled_h) / 2;

    tmp = malloc((size_t)scaled_w * scaled_h * 3U);
    if (!tmp)
        return;

    resize_rgb888_nn(src, sw, sh, tmp, scaled_w, scaled_h);
    for (int y = 0; y < scaled_h; y++) {
        const uint8_t *src_row = tmp + (size_t)y * scaled_w * 3U;
        uint8_t *dst_row = dst + ((size_t)(off_y + y) * dw + (size_t)off_x) * 3U;
        memcpy(dst_row, src_row, (size_t)scaled_w * 3U);
    }
    free(tmp);

    if (meta) {
        meta->scale = scale;
        meta->pad_x = off_x;
        meta->pad_y = off_y;
        meta->valid = true;
    }
}

static void raw565_to_rgb888_full(struct app_ctx *ctx, const uint8_t *raw, uint8_t *rgb)
{
    size_t i;
    size_t pixels = (size_t)ctx->frame_width * ctx->frame_height;
    for (i = 0; i < pixels; i++) {
        uint8_t r, g, b;
        decode_pixel565(&ctx->opt, raw[i * 2], raw[i * 2 + 1], &r, &g, &b);
        rgb[i * 3 + 0] = r;
        rgb[i * 3 + 1] = g;
        rgb[i * 3 + 2] = b;
    }
}

static void bgrx8888_to_rgb888_and_a(const uint8_t *src_bgrx, int w, int h,
                                     uint8_t *dst_rgb, uint8_t *dst_a)
{
    size_t i;
    size_t pixels = (size_t)w * h;
    for (i = 0; i < pixels; i++) {
        const uint8_t *p = src_bgrx + i * 4U;
        uint8_t *q = dst_rgb + i * 3U;
        q[0] = p[2];
        q[1] = p[1];
        q[2] = p[0];
        if (dst_a)
            dst_a[i] = p[3];
    }
}

static inline uint8_t clip_u8(int v)
{
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

static float laplacian_variance_rgb888(const uint8_t *rgb, int w, int h)
{
    int x, y;
    double sum = 0.0;
    double sum2 = 0.0;
    int n = 0;

    if (!rgb || w < 3 || h < 3)
        return 0.0f;

    for (y = 1; y < h - 1; y++) {
        for (x = 1; x < w - 1; x++) {
            int c, p00, p01, p02, p10, p12, p20, p21, p22;
            int lap;
            const uint8_t *pc = rgb + ((size_t)y * (size_t)w + (size_t)x) * 3U;
            const uint8_t *q;

            c = (77 * pc[0] + 150 * pc[1] + 29 * pc[2]) >> 8;

            q = rgb + (((size_t)y - 1U) * (size_t)w + (size_t)(x - 1)) * 3U;
            p00 = (77 * q[0] + 150 * q[1] + 29 * q[2]) >> 8;
            q = rgb + (((size_t)y - 1U) * (size_t)w + (size_t)x) * 3U;
            p01 = (77 * q[0] + 150 * q[1] + 29 * q[2]) >> 8;
            q = rgb + (((size_t)y - 1U) * (size_t)w + (size_t)(x + 1)) * 3U;
            p02 = (77 * q[0] + 150 * q[1] + 29 * q[2]) >> 8;

            q = rgb + ((size_t)y * (size_t)w + (size_t)(x - 1)) * 3U;
            p10 = (77 * q[0] + 150 * q[1] + 29 * q[2]) >> 8;
            q = rgb + ((size_t)y * (size_t)w + (size_t)(x + 1)) * 3U;
            p12 = (77 * q[0] + 150 * q[1] + 29 * q[2]) >> 8;

            q = rgb + (((size_t)y + 1U) * (size_t)w + (size_t)(x - 1)) * 3U;
            p20 = (77 * q[0] + 150 * q[1] + 29 * q[2]) >> 8;
            q = rgb + (((size_t)y + 1U) * (size_t)w + (size_t)x) * 3U;
            p21 = (77 * q[0] + 150 * q[1] + 29 * q[2]) >> 8;
            q = rgb + (((size_t)y + 1U) * (size_t)w + (size_t)(x + 1)) * 3U;
            p22 = (77 * q[0] + 150 * q[1] + 29 * q[2]) >> 8;

            lap = (8 * c) - p00 - p01 - p02 - p10 - p12 - p20 - p21 - p22;
            sum += (double)lap;
            sum2 += (double)lap * (double)lap;
            n++;
        }
    }

    if (n <= 0)
        return 0.0f;
    {
        double mean = sum / (double)n;
        double var = (sum2 / (double)n) - mean * mean;
        if (var < 0.0)
            var = 0.0;
        return (float)var;
    }
}

static void ocr_preprocess_rgb888(uint8_t *rgb, int w, int h, int mode)
{
    size_t pixels = (size_t)w * h;
    uint8_t *gray = NULL;
    uint8_t *tmp = NULL;
    int x, y;

    if (mode == OCR_PREPROC_NONE || w <= 1 || h <= 1)
        return;

    gray = malloc(pixels);
    if (!gray)
        return;

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            const uint8_t *p = rgb + ((size_t)y * w + x) * 3U;
            gray[(size_t)y * w + x] = (uint8_t)((77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8);
        }
    }

    if (mode == OCR_PREPROC_GRAY) {
        for (y = 0; y < h; y++) {
            for (x = 0; x < w; x++) {
                uint8_t g = gray[(size_t)y * w + x];
                uint8_t *q = rgb + ((size_t)y * w + x) * 3U;
                q[0] = g;
                q[1] = g;
                q[2] = g;
            }
        }
        free(gray);
        return;
    }

    tmp = malloc(pixels);
    if (!tmp) {
        free(gray);
        return;
    }

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            int s = 0;
            int c = 0;
            for (int ky = -1; ky <= 1; ky++) {
                int yy = y + ky;
                if (yy < 0 || yy >= h)
                    continue;
                for (int kx = -1; kx <= 1; kx++) {
                    int xx = x + kx;
                    if (xx < 0 || xx >= w)
                        continue;
                    s += gray[(size_t)yy * w + xx];
                    c++;
                }
            }
            tmp[(size_t)y * w + x] = (uint8_t)(s / (c > 0 ? c : 1));
        }
    }

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            int s = 0;
            int c = 0;
            uint8_t outv;
            for (int ky = -2; ky <= 2; ky++) {
                int yy = y + ky;
                if (yy < 0 || yy >= h)
                    continue;
                for (int kx = -2; kx <= 2; kx++) {
                    int xx = x + kx;
                    if (xx < 0 || xx >= w)
                        continue;
                    s += tmp[(size_t)yy * w + xx];
                    c++;
                }
            }
            s = (c > 0) ? (s / c) : 0;
            outv = (tmp[(size_t)y * w + x] > (s - 8)) ? 255U : 0U;
            rgb[((size_t)y * w + x) * 3U + 0] = outv;
            rgb[((size_t)y * w + x) * 3U + 1] = outv;
            rgb[((size_t)y * w + x) * 3U + 2] = outv;
        }
    }

    free(tmp);
    free(gray);
}

static void sw_preprocess_rgb888(uint8_t *rgb, int w, int h)
{
    size_t pixels = (size_t)w * h;
    uint8_t *gray = malloc(pixels);
    uint8_t *filt = malloc(pixels);
    uint8_t *edge = malloc(pixels);
    int x, y;
    if (!gray || !filt || !edge) {
        free(gray);
        free(filt);
        free(edge);
        return;
    }

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            const uint8_t *p = rgb + ((size_t)y * w + x) * 3U;
            int g = (77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8;
            gray[(size_t)y * w + x] = (uint8_t)g;
        }
    }
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            int s = 0;
            int c = 0;
            int ky, kx;
            for (ky = -1; ky <= 1; ky++) {
                int yy = y + ky;
                if (yy < 0 || yy >= h)
                    continue;
                for (kx = -1; kx <= 1; kx++) {
                    int xx = x + kx;
                    if (xx < 0 || xx >= w)
                        continue;
                    s += gray[(size_t)yy * w + xx];
                    c++;
                }
            }
            filt[(size_t)y * w + x] = (uint8_t)(s / (c > 0 ? c : 1));
        }
    }
    for (y = 1; y < h - 1; y++) {
        for (x = 1; x < w - 1; x++) {
            int gx =
                -filt[(size_t)(y - 1) * w + (x - 1)] + filt[(size_t)(y - 1) * w + (x + 1)] +
                -2 * filt[(size_t)y * w + (x - 1)]     + 2 * filt[(size_t)y * w + (x + 1)] +
                -filt[(size_t)(y + 1) * w + (x - 1)] + filt[(size_t)(y + 1) * w + (x + 1)];
            int gy =
                -filt[(size_t)(y - 1) * w + (x - 1)] - 2 * filt[(size_t)(y - 1) * w + x] - filt[(size_t)(y - 1) * w + (x + 1)] +
                 filt[(size_t)(y + 1) * w + (x - 1)] + 2 * filt[(size_t)(y + 1) * w + x] + filt[(size_t)(y + 1) * w + (x + 1)];
            int mag = (abs(gx) + abs(gy)) >> 2;
            edge[(size_t)y * w + x] = clip_u8(mag);
        }
    }
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            uint8_t g = filt[(size_t)y * w + x];
            uint8_t e = edge[(size_t)y * w + x];
            uint8_t enh = clip_u8((int)g + ((int)e >> 1));
            uint8_t *q = rgb + ((size_t)y * w + x) * 3U;
            q[0] = enh;
            q[1] = enh;
            q[2] = enh;
        }
    }

    free(gray);
    free(filt);
    free(edge);
}

static int extract_a_channel_roi(const uint8_t *a_map, int w, int h, float proj_ratio,
                                 struct det_box *roi, float *red_ratio_out)
{
    int *hist_x = calloc((size_t)w, sizeof(int));
    int *hist_y = calloc((size_t)h, sizeof(int));
    int x, y;
    int edge_total = 0;
    int valid_total = 0;
    int red_total = 0;
    int max_x = 0, max_y = 0;
    int x1 = -1, x2 = -1, y1 = -1, y2 = -1;

    if (!hist_x || !hist_y) {
        free(hist_x);
        free(hist_y);
        return 0;
    }

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            size_t idx = (size_t)y * w + x;
            uint8_t a = a_map[idx];
            uint8_t preproc_valid = (a >> 7) & 0x1;
            uint8_t edge_flag = (a >> 6) & 0x1;
            uint8_t color_class = (a >> 2) & 0x3;
            if (x == 0 && y == 0)
                continue; /* Pixel[0,0] may be watermark */
            if (!preproc_valid)
                continue;
            valid_total++;
            if (color_class == 0x3)
                red_total++;
            if (edge_flag) {
                hist_x[x]++;
                hist_y[y]++;
                edge_total++;
            }
        }
    }
    if (red_ratio_out) {
        *red_ratio_out = (valid_total > 0) ? ((float)red_total / (float)valid_total) : 0.0f;
    }
    if (edge_total <= 0) {
        free(hist_x);
        free(hist_y);
        return 0;
    }

    for (x = 0; x < w; x++)
        if (hist_x[x] > max_x) max_x = hist_x[x];
    for (y = 0; y < h; y++)
        if (hist_y[y] > max_y) max_y = hist_y[y];

    if (max_x <= 0 || max_y <= 0) {
        free(hist_x);
        free(hist_y);
        return 0;
    }

    {
        int tx = (int)((float)max_x * proj_ratio);
        int ty = (int)((float)max_y * proj_ratio);
        for (x = 0; x < w; x++) {
            if (hist_x[x] >= tx) {
                if (x1 < 0) x1 = x;
                x2 = x;
            }
        }
        for (y = 0; y < h; y++) {
            if (hist_y[y] >= ty) {
                if (y1 < 0) y1 = y;
                y2 = y;
            }
        }
    }

    free(hist_x);
    free(hist_y);

    if (x1 < 0 || y1 < 0 || x2 <= x1 || y2 <= y1)
        return 0;

    roi->x1 = x1;
    roi->y1 = y1;
    roi->x2 = x2;
    roi->y2 = y2;
    roi->conf = 1.0f;
    roi->cls = 0;
    return 1;
}

static int filter_boxes_by_roi(const struct det_box *in, int in_count, const struct det_box *roi,
                               float iou_thr, struct det_box *out, int out_cap)
{
    int i;
    int n = 0;
    for (i = 0; i < in_count && n < out_cap; i++) {
        const struct det_box *b = &in[i];
        int cx = (b->x1 + b->x2) / 2;
        int cy = (b->y1 + b->y2) / 2;
        bool center_inside = (cx >= roi->x1 && cx <= roi->x2 && cy >= roi->y1 && cy <= roi->y2);
        if (center_inside || box_iou(b, roi) >= iou_thr)
            out[n++] = *b;
    }
    return n;
}

static int update_ped_tracks_nn(struct app_ctx *ctx,
                                const struct det_box *persons, int person_count,
                                bool light_red, uint64_t frame_seq,
                                struct det_box *out_persons, int *out_person_count)
{
    int i;
    int j;
    int events = 0;
    int stopline_y = (int)((float)ctx->frame_height * ctx->opt.stopline_ratio);
    bool used_det[MAX_DETS] = {0};
    (void)frame_seq;

    for (i = 0; i < ctx->ped_track_count; i++)
        ctx->ped_track_ttl[i]--;

    for (i = 0; i < ctx->ped_track_count; i++) {
        int best = -1;
        int best_d2 = 0x7fffffff;
        int tcx = (ctx->ped_track_box[i].x1 + ctx->ped_track_box[i].x2) / 2;
        int tcy = (ctx->ped_track_box[i].y1 + ctx->ped_track_box[i].y2) / 2;
        for (j = 0; j < person_count; j++) {
            int dcx, dcy, d2;
            if (used_det[j])
                continue;
            dcx = (persons[j].x1 + persons[j].x2) / 2 - tcx;
            dcy = (persons[j].y1 + persons[j].y2) / 2 - tcy;
            d2 = dcx * dcx + dcy * dcy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best = j;
            }
        }
        if (best >= 0 && best_d2 <= (96 * 96)) {
            int old_cy = (ctx->ped_track_box[i].y1 + ctx->ped_track_box[i].y2) / 2;
            int new_cy = (persons[best].y1 + persons[best].y2) / 2;
            bool crossed = (old_cy < stopline_y) && (new_cy >= stopline_y);
            ctx->ped_track_box[i] = persons[best];
            ctx->ped_track_ttl[i] = 8;
            used_det[best] = true;
            if (light_red && crossed) {
                events++;
            }
        }
    }

    for (j = 0; j < person_count && ctx->ped_track_count < MAX_DETS; j++) {
        if (used_det[j])
            continue;
        ctx->ped_track_box[ctx->ped_track_count] = persons[j];
        ctx->ped_track_id[ctx->ped_track_count] = ctx->ped_next_track_id++;
        ctx->ped_track_ttl[ctx->ped_track_count] = 8;
        ctx->ped_track_count++;
    }

    i = 0;
    while (i < ctx->ped_track_count) {
        if (ctx->ped_track_ttl[i] > 0) {
            i++;
            continue;
        }
        ctx->ped_track_box[i] = ctx->ped_track_box[ctx->ped_track_count - 1];
        ctx->ped_track_id[i] = ctx->ped_track_id[ctx->ped_track_count - 1];
        ctx->ped_track_ttl[i] = ctx->ped_track_ttl[ctx->ped_track_count - 1];
        ctx->ped_track_count--;
    }

    *out_person_count = 0;
    for (i = 0; i < person_count && *out_person_count < MAX_DETS; i++)
        out_persons[(*out_person_count)++] = persons[i];

    return events;
}

static void clamp_box(struct det_box *b, int w, int h)
{
    if (b->x1 < 0) b->x1 = 0;
    if (b->y1 < 0) b->y1 = 0;
    if (b->x2 >= w) b->x2 = w - 1;
    if (b->y2 >= h) b->y2 = h - 1;
    if (b->x2 < b->x1) b->x2 = b->x1;
    if (b->y2 < b->y1) b->y2 = b->y1;
}

static void map_box_between_spaces(struct det_box *b, int src_w, int src_h, int dst_w, int dst_h)
{
    b->x1 = (int)((int64_t)b->x1 * dst_w / src_w);
    b->x2 = (int)((int64_t)b->x2 * dst_w / src_w);
    b->y1 = (int)((int64_t)b->y1 * dst_h / src_h);
    b->y2 = (int)((int64_t)b->y2 * dst_h / src_h);
    clamp_box(b, dst_w, dst_h);
}

static void map_box_from_letterbox(struct det_box *b, const struct letterbox_meta *lb)
{
    float x1, y1, x2, y2;
    if (!lb || !lb->valid || lb->scale <= 0.0f)
        return;

    x1 = ((float)b->x1 - (float)lb->pad_x) / lb->scale;
    x2 = ((float)b->x2 - (float)lb->pad_x) / lb->scale;
    y1 = ((float)b->y1 - (float)lb->pad_y) / lb->scale;
    y2 = ((float)b->y2 - (float)lb->pad_y) / lb->scale;

    b->x1 = (int)lroundf(x1);
    b->x2 = (int)lroundf(x2);
    b->y1 = (int)lroundf(y1);
    b->y2 = (int)lroundf(y2);
    clamp_box(b, lb->src_w, lb->src_h);
}

static void map_point_from_letterbox(float *x, float *y, const struct letterbox_meta *lb)
{
    if (!x || !y || !lb || !lb->valid || lb->scale <= 0.0f)
        return;
    *x = (*x - (float)lb->pad_x) / lb->scale;
    *y = (*y - (float)lb->pad_y) / lb->scale;
}

static void map_point_between_spaces(float *x, float *y, int src_w, int src_h, int dst_w, int dst_h)
{
    if (!x || !y || src_w <= 0 || src_h <= 0)
        return;
    *x = *x * (float)dst_w / (float)src_w;
    *y = *y * (float)dst_h / (float)src_h;
}

static void map_obb_from_detect_space(struct det_box *b, int det_resize_mode,
                                      const struct letterbox_meta *lb,
                                      int src_w, int src_h, int det_w, int det_h)
{
    int i;
    float min_x = 1e30f, min_y = 1e30f;
    float max_x = -1e30f, max_y = -1e30f;
    for (i = 0; i < 4; i++) {
        float x = b->quad[i * 2 + 0];
        float y = b->quad[i * 2 + 1];
        if (det_resize_mode == DET_RESIZE_LETTERBOX)
            map_point_from_letterbox(&x, &y, lb);
        else
            map_point_between_spaces(&x, &y, det_w, det_h, src_w, src_h);
        if (x < 0.0f) x = 0.0f;
        if (y < 0.0f) y = 0.0f;
        if (x > (float)(src_w - 1)) x = (float)(src_w - 1);
        if (y > (float)(src_h - 1)) y = (float)(src_h - 1);
        b->quad[i * 2 + 0] = x;
        b->quad[i * 2 + 1] = y;
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
    }
    b->x1 = (int)floorf(min_x);
    b->y1 = (int)floorf(min_y);
    b->x2 = (int)ceilf(max_x);
    b->y2 = (int)ceilf(max_y);
    b->cx = 0.25f * (b->quad[0] + b->quad[2] + b->quad[4] + b->quad[6]);
    b->cy = 0.25f * (b->quad[1] + b->quad[3] + b->quad[5] + b->quad[7]);
    b->w = hypotf(b->quad[2] - b->quad[0], b->quad[3] - b->quad[1]);
    b->h = hypotf(b->quad[6] - b->quad[0], b->quad[7] - b->quad[1]);
    b->angle = atan2f(b->quad[3] - b->quad[1], b->quad[2] - b->quad[0]);
    clamp_box(b, src_w, src_h);
}

static void map_box_from_detect_space(struct det_box *b, int det_resize_mode,
                                      const struct letterbox_meta *lb,
                                      int src_w, int src_h, int det_w, int det_h)
{
    if (det_resize_mode == DET_RESIZE_LETTERBOX)
        map_box_from_letterbox(b, lb);
    else
        map_box_between_spaces(b, det_w, det_h, src_w, src_h);
}

static void prepare_detect_canvas(const struct app_ctx *ctx, const uint8_t *src_rgb, int src_w, int src_h,
                                  uint8_t *det_rgb, struct letterbox_meta *lb)
{
    if (ctx->opt.det_resize_mode == DET_RESIZE_LETTERBOX) {
        resize_rgb888_nn_letterbox_meta(src_rgb, src_w, src_h,
                                        det_rgb, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE, 0U, lb);
        return;
    }
    if (lb) {
        memset(lb, 0, sizeof(*lb));
        lb->src_w = src_w;
        lb->src_h = src_h;
        lb->dst_w = ALGO_STREAM_SIZE;
        lb->dst_h = ALGO_STREAM_SIZE;
    }
    resize_rgb888_nn(src_rgb, src_w, src_h, det_rgb, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE);
}

static int run_detect_on_rgb(struct app_ctx *ctx, struct yolo_model *m,
                             const uint8_t *src_rgb, int src_w, int src_h,
                             float conf_thr, uint8_t *det_rgb, uint8_t *model_in,
                             struct det_box *out, int *out_count,
                             struct detect_decode_diag *diag)
{
    struct letterbox_meta lb;
    int i;

    prepare_detect_canvas(ctx, src_rgb, src_w, src_h, det_rgb, &lb);
    if (m->in_w == ALGO_STREAM_SIZE && m->in_h == ALGO_STREAM_SIZE)
        memcpy(model_in, det_rgb, (size_t)ALGO_STREAM_SIZE * ALGO_STREAM_SIZE * 3U);
    else
        resize_rgb888_nn(det_rgb, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE,
                         model_in, (int)m->in_w, (int)m->in_h);

    if (run_model_detect(m, model_in, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE,
                         conf_thr, out, out_count, diag) < 0) {
        *out_count = 0;
        return -1;
    }
    for (i = 0; i < *out_count; i++) {
        if (out[i].has_obb) {
            map_obb_from_detect_space(&out[i], ctx->opt.det_resize_mode, &lb,
                                      src_w, src_h, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE);
        } else {
            map_box_from_detect_space(&out[i], ctx->opt.det_resize_mode, &lb,
                                      src_w, src_h, ALGO_STREAM_SIZE, ALGO_STREAM_SIZE);
        }
    }
    return 0;
}

static void compute_center_crop_box(const struct det_box *src, int img_w, int img_h,
                                    int crop_w, int crop_h, struct det_box *crop)
{
    int cx = (src->x1 + src->x2) / 2;
    int cy = (src->y1 + src->y2) / 2;
    int x1 = cx - crop_w / 2;
    int y1 = cy - crop_h / 2;

    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x1 + crop_w > img_w) x1 = img_w - crop_w;
    if (y1 + crop_h > img_h) y1 = img_h - crop_h;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;

    crop->x1 = x1;
    crop->y1 = y1;
    crop->x2 = x1 + crop_w - 1;
    crop->y2 = y1 + crop_h - 1;
    clamp_box(crop, img_w, img_h);
}

static void compute_expand_crop_box(const struct det_box *src, int img_w, int img_h,
                                    float pad_x, float pad_y, struct det_box *crop)
{
    int bw = src->x2 - src->x1 + 1;
    int bh = src->y2 - src->y1 + 1;
    int ex = (int)((float)bw * pad_x + 0.5f);
    int ey = (int)((float)bh * pad_y + 0.5f);
    crop->x1 = src->x1 - ex;
    crop->y1 = src->y1 - ey;
    crop->x2 = src->x2 + ex;
    crop->y2 = src->y2 + ey;
    clamp_box(crop, img_w, img_h);
}

static void compute_ocr_crop_box(const struct app_ctx *ctx, const struct det_box *src,
                                 struct det_box *crop)
{
    if (ctx->opt.ocr_crop_mode == OCR_CROP_MATCH ||
        ctx->opt.ocr_crop_mode == OCR_CROP_OBB_WARP ||
        ctx->opt.ocr_crop_mode == OCR_CROP_OBB_PIECEWISE) {
        *crop = *src;
        clamp_box(crop, (int)ctx->frame_width, (int)ctx->frame_height);
        return;
    }
    if (ctx->opt.ocr_crop_mode == OCR_CROP_BOX) {
        compute_expand_crop_box(src, (int)ctx->frame_width, (int)ctx->frame_height, 0.06f, 0.12f, crop);
        return;
    }
    if (ctx->opt.ocr_crop_mode == OCR_CROP_TIGHT) {
        compute_expand_crop_box(src, (int)ctx->frame_width, (int)ctx->frame_height, 0.08f, 0.16f, crop);
        return;
    }
    if (ctx->opt.ocr_crop_mode == OCR_CROP_BOX_PAD) {
        compute_expand_crop_box(src, (int)ctx->frame_width, (int)ctx->frame_height, 0.15f, 0.28f, crop);
        return;
    }
    compute_center_crop_box(src, (int)ctx->frame_width, (int)ctx->frame_height,
                            OCR_CROP_WIDTH, OCR_CROP_HEIGHT, crop);
}

static float estimate_ocr_occ_ratio(const struct app_ctx *ctx, int crop_w, int crop_h)
{
    int in_w = (int)ctx->ocr_model.in_w;
    int in_h = (int)ctx->ocr_model.in_h;
    float sx, sy, scale;
    int scaled_w;

    if (crop_w <= 0 || crop_h <= 0 || in_w <= 0 || in_h <= 0)
        return 0.0f;
    if (ctx->opt.ocr_resize_mode != OCR_RESIZE_LETTERBOX)
        return 1.0f;

    sx = (float)in_w / (float)crop_w;
    sy = (float)in_h / (float)crop_h;
    scale = (sx < sy) ? sx : sy;
    if (scale <= 0.0f)
        return 0.0f;

    scaled_w = (int)((float)crop_w * scale + 0.5f);
    if (scaled_w < 1) scaled_w = 1;
    if (scaled_w > in_w) scaled_w = in_w;
    return (float)scaled_w / (float)in_w;
}

static bool compute_match_ytrim_crop(const struct app_ctx *ctx, const struct det_box *src,
                                     float min_occ_ratio, struct det_box *out)
{
    int bw, bh, in_w, in_h, new_h, trim;
    float model_aspect, target_aspect, cur_aspect;

    if (!ctx || !src || !out)
        return false;
    if (min_occ_ratio <= 0.0f)
        return false;

    bw = src->x2 - src->x1 + 1;
    bh = src->y2 - src->y1 + 1;
    in_w = (int)ctx->ocr_model.in_w;
    in_h = (int)ctx->ocr_model.in_h;
    if (bw <= 0 || bh <= 0 || in_w <= 0 || in_h <= 0)
        return false;

    if (min_occ_ratio > 1.0f)
        min_occ_ratio = 1.0f;
    model_aspect = (float)in_w / (float)in_h;
    target_aspect = model_aspect * min_occ_ratio;
    if (target_aspect <= 0.0f)
        return false;

    cur_aspect = (float)bw / (float)bh;
    if (cur_aspect >= target_aspect)
        return false;

    new_h = (int)((float)bw / target_aspect + 0.5f);
    if (new_h < 1)
        new_h = 1;
    if (new_h >= bh)
        return false;

    trim = (bh - new_h) / 2;
    *out = *src;
    out->y1 = src->y1 + trim;
    out->y2 = out->y1 + new_h - 1;
    if (out->y2 > src->y2) {
        out->y2 = src->y2;
        out->y1 = out->y2 - new_h + 1;
    }
    if (out->y1 < src->y1)
        out->y1 = src->y1;
    if (out->y2 <= out->y1)
        return false;
    return true;
}

static void copy_crop_rgb888(const uint8_t *rgb, int img_w, const struct det_box *crop, uint8_t *dst)
{
    int y;
    int crop_w = crop->x2 - crop->x1 + 1;
    int crop_h = crop->y2 - crop->y1 + 1;
    for (y = 0; y < crop_h; y++) {
        const uint8_t *src_row = rgb + ((crop->y1 + y) * img_w + crop->x1) * 3;
        uint8_t *dst_row = dst + y * crop_w * 3;
        memcpy(dst_row, src_row, (size_t)crop_w * 3U);
    }
}

static void order_quad_points(const float in[8], float out[8])
{
    struct quad_pt {
        float x;
        float y;
        float a;
    } pts[4];
    int idx[4] = {0, 1, 2, 3};
    int i, j;
    float cx = 0.0f, cy = 0.0f;
    float area;

    if (!in || !out)
        return;

    for (i = 0; i < 4; i++) {
        pts[i].x = in[i * 2 + 0];
        pts[i].y = in[i * 2 + 1];
        cx += pts[i].x;
        cy += pts[i].y;
    }
    cx *= 0.25f;
    cy *= 0.25f;
    for (i = 0; i < 4; i++)
        pts[i].a = atan2f(pts[i].y - cy, pts[i].x - cx);

    for (i = 0; i < 3; i++) {
        for (j = i + 1; j < 4; j++) {
            if (pts[idx[j]].a < pts[idx[i]].a) {
                int t = idx[i]; idx[i] = idx[j]; idx[j] = t;
            }
        }
    }

    {
        int top_edge = 0;
        float best_y = 0.5f * (pts[idx[0]].y + pts[idx[1]].y);
        int pos;
        for (i = 1; i < 4; i++) {
            float edge_y = 0.5f * (pts[idx[i]].y + pts[idx[(i + 1) & 3]].y);
            if (edge_y < best_y) {
                best_y = edge_y;
                top_edge = i;
            }
        }
        if (pts[idx[top_edge]].x <= pts[idx[(top_edge + 1) & 3]].x) {
            for (i = 0; i < 4; i++) {
                pos = idx[(top_edge + i) & 3];
                out[i * 2 + 0] = pts[pos].x;
                out[i * 2 + 1] = pts[pos].y;
            }
        } else {
            for (i = 0; i < 4; i++) {
                pos = idx[(top_edge + 1 - i + 4) & 3];
                out[i * 2 + 0] = pts[pos].x;
                out[i * 2 + 1] = pts[pos].y;
            }
        }
    }

    area = quad_area8(out);
    if (area < 4.0f || !quad_is_convex8(out)) {
        /* Degenerate OBB/refiner output: preserve input order rather than risk
         * sum/diff corner duplication or a folded homography.
         */
        memcpy(out, in, sizeof(float) * 8U);
    }
}

static void bbox_from_quad(const float quad[8], int img_w, int img_h, struct det_box *box)
{
    int i;
    float min_x = 1e30f, min_y = 1e30f;
    float max_x = -1e30f, max_y = -1e30f;
    memset(box, 0, sizeof(*box));
    for (i = 0; i < 4; i++) {
        float x = quad[i * 2 + 0];
        float y = quad[i * 2 + 1];
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
    }
    box->x1 = (int)floorf(min_x);
    box->y1 = (int)floorf(min_y);
    box->x2 = (int)ceilf(max_x);
    box->y2 = (int)ceilf(max_y);
    clamp_box(box, img_w, img_h);
}

static bool solve_linear_8x8(float a[8][9], float x[8])
{
    int i;
    int j;
    int k;
    for (i = 0; i < 8; i++) {
        int piv = i;
        float max_v = fabsf(a[i][i]);
        for (j = i + 1; j < 8; j++) {
            float v = fabsf(a[j][i]);
            if (v > max_v) {
                max_v = v;
                piv = j;
            }
        }
        if (max_v < 1e-8f)
            return false;
        if (piv != i) {
            for (k = i; k < 9; k++) {
                float t = a[i][k];
                a[i][k] = a[piv][k];
                a[piv][k] = t;
            }
        }
        {
            float div = a[i][i];
            for (k = i; k < 9; k++)
                a[i][k] /= div;
        }
        for (j = 0; j < 8; j++) {
            float mul;
            if (j == i)
                continue;
            mul = a[j][i];
            if (fabsf(mul) < 1e-8f)
                continue;
            for (k = i; k < 9; k++)
                a[j][k] -= mul * a[i][k];
        }
    }
    for (i = 0; i < 8; i++)
        x[i] = a[i][8];
    return true;
}

static bool get_homography_4pt(const float src[8], const float dst[8], float h[9])
{
    float mat[8][9];
    float sol[8];
    int i;
    memset(mat, 0, sizeof(mat));
    for (i = 0; i < 4; i++) {
        float x = src[i * 2 + 0];
        float y = src[i * 2 + 1];
        float u = dst[i * 2 + 0];
        float v = dst[i * 2 + 1];
        int r0 = i * 2;
        int r1 = r0 + 1;
        mat[r0][0] = x;
        mat[r0][1] = y;
        mat[r0][2] = 1.0f;
        mat[r0][6] = -u * x;
        mat[r0][7] = -u * y;
        mat[r0][8] = u;

        mat[r1][3] = x;
        mat[r1][4] = y;
        mat[r1][5] = 1.0f;
        mat[r1][6] = -v * x;
        mat[r1][7] = -v * y;
        mat[r1][8] = v;
    }
    if (!solve_linear_8x8(mat, sol))
        return false;
    h[0] = sol[0]; h[1] = sol[1]; h[2] = sol[2];
    h[3] = sol[3]; h[4] = sol[4]; h[5] = sol[5];
    h[6] = sol[6]; h[7] = sol[7]; h[8] = 1.0f;
    return true;
}

static bool invert_homography(const float h[9], float inv[9])
{
    float det = h[0] * (h[4] * h[8] - h[5] * h[7]) -
                h[1] * (h[3] * h[8] - h[5] * h[6]) +
                h[2] * (h[3] * h[7] - h[4] * h[6]);
    if (fabsf(det) < 1e-8f)
        return false;
    inv[0] =  (h[4] * h[8] - h[5] * h[7]) / det;
    inv[1] = -(h[1] * h[8] - h[2] * h[7]) / det;
    inv[2] =  (h[1] * h[5] - h[2] * h[4]) / det;
    inv[3] = -(h[3] * h[8] - h[5] * h[6]) / det;
    inv[4] =  (h[0] * h[8] - h[2] * h[6]) / det;
    inv[5] = -(h[0] * h[5] - h[2] * h[3]) / det;
    inv[6] =  (h[3] * h[7] - h[4] * h[6]) / det;
    inv[7] = -(h[0] * h[7] - h[1] * h[6]) / det;
    inv[8] =  (h[0] * h[4] - h[1] * h[3]) / det;
    return true;
}

static void bilinear_sample_rgb888(const uint8_t *rgb, int img_w, int img_h,
                                   float x, float y, uint8_t out[3])
{
    int x0 = (int)floorf(x);
    int y0 = (int)floorf(y);
    int x1;
    int y1;
    float wx;
    float wy;
    int c;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x0 > img_w - 1) x0 = img_w - 1;
    if (y0 > img_h - 1) y0 = img_h - 1;
    x1 = x0 + 1; if (x1 > img_w - 1) x1 = img_w - 1;
    y1 = y0 + 1; if (y1 > img_h - 1) y1 = img_h - 1;
    wx = x - (float)x0;
    wy = y - (float)y0;
    if (wx < 0.0f) wx = 0.0f;
    if (wx > 1.0f) wx = 1.0f;
    if (wy < 0.0f) wy = 0.0f;
    if (wy > 1.0f) wy = 1.0f;
    for (c = 0; c < 3; c++) {
        const uint8_t *p00 = rgb + ((size_t)y0 * (size_t)img_w + (size_t)x0) * 3U + (size_t)c;
        const uint8_t *p01 = rgb + ((size_t)y0 * (size_t)img_w + (size_t)x1) * 3U + (size_t)c;
        const uint8_t *p10 = rgb + ((size_t)y1 * (size_t)img_w + (size_t)x0) * 3U + (size_t)c;
        const uint8_t *p11 = rgb + ((size_t)y1 * (size_t)img_w + (size_t)x1) * 3U + (size_t)c;
        float v0 = (1.0f - wx) * (float)(*p00) + wx * (float)(*p01);
        float v1 = (1.0f - wx) * (float)(*p10) + wx * (float)(*p11);
        float v = (1.0f - wy) * v0 + wy * v1;
        if (v < 0.0f) v = 0.0f;
        if (v > 255.0f) v = 255.0f;
        out[c] = (uint8_t)(v + 0.5f);
    }
}

static bool warp_quad_to_rect_rgb888(const uint8_t *rgb, int img_w, int img_h, const float quad_in[8],
                                     uint8_t *dst, int dst_cap_w, int dst_cap_h,
                                     int *out_w, int *out_h)
{
    float quad[8];
    float dst_quad[8];
    float h[9];
    float inv_h[9];
    float w_top;
    float w_bottom;
    float h_left;
    float h_right;
    int dw;
    int dh;
    int y;
    int x;
    if (!rgb || !quad_in || !dst || !out_w || !out_h)
        return false;
    order_quad_points(quad_in, quad);
    w_top = hypotf(quad[2] - quad[0], quad[3] - quad[1]);
    w_bottom = hypotf(quad[4] - quad[6], quad[5] - quad[7]);
    h_left = hypotf(quad[6] - quad[0], quad[7] - quad[1]);
    h_right = hypotf(quad[4] - quad[2], quad[5] - quad[3]);
    dw = (int)(fmaxf(w_top, w_bottom) + 0.5f);
    dh = (int)(fmaxf(h_left, h_right) + 0.5f);
    if (dw < 1) dw = 1;
    if (dh < 1) dh = 1;
    if (dw > dst_cap_w) dw = dst_cap_w;
    if (dh > dst_cap_h) dh = dst_cap_h;
    if (dw <= 0 || dh <= 0)
        return false;

    dst_quad[0] = 0.0f;              dst_quad[1] = 0.0f;
    dst_quad[2] = (float)dw - 1.0f;  dst_quad[3] = 0.0f;
    dst_quad[4] = (float)dw - 1.0f;  dst_quad[5] = (float)dh - 1.0f;
    dst_quad[6] = 0.0f;              dst_quad[7] = (float)dh - 1.0f;
    if (!get_homography_4pt(quad, dst_quad, h))
        return false;
    if (!invert_homography(h, inv_h))
        return false;

    for (y = 0; y < dh; y++) {
        for (x = 0; x < dw; x++) {
            float fx = (float)x;
            float fy = (float)y;
            float den = inv_h[6] * fx + inv_h[7] * fy + inv_h[8];
            float sx;
            float sy;
            uint8_t pix[3];
            uint8_t *q;
            if (fabsf(den) < 1e-8f)
                den = (den >= 0.0f) ? 1e-8f : -1e-8f;
            sx = (inv_h[0] * fx + inv_h[1] * fy + inv_h[2]) / den;
            sy = (inv_h[3] * fx + inv_h[4] * fy + inv_h[5]) / den;
            if (sx < 0.0f) sx = 0.0f;
            if (sy < 0.0f) sy = 0.0f;
            if (sx > (float)(img_w - 1)) sx = (float)(img_w - 1);
            if (sy > (float)(img_h - 1)) sy = (float)(img_h - 1);
            bilinear_sample_rgb888(rgb, img_w, img_h, sx, sy, pix);
            q = dst + ((size_t)y * (size_t)dw + (size_t)x) * 3U;
            q[0] = pix[0];
            q[1] = pix[1];
            q[2] = pix[2];
        }
    }
    *out_w = dw;
    *out_h = dh;
    return true;
}

static bool warp_quad_to_rect_piecewise_rgb888(const uint8_t *rgb, int img_w, int img_h,
                                               const float quad_in[8], uint8_t *dst,
                                               int dst_cap_w, int dst_cap_h,
                                               int *out_w, int *out_h)
{
    float quad[8];
    float top_w;
    float bottom_w;
    float left_h;
    float right_h;
    int dw;
    int dh;
    int x;
    int y;

    if (!rgb || !quad_in || !dst || !out_w || !out_h)
        return false;

    order_quad_points(quad_in, quad);
    top_w = hypotf(quad[2] - quad[0], quad[3] - quad[1]);
    bottom_w = hypotf(quad[4] - quad[6], quad[5] - quad[7]);
    left_h = hypotf(quad[6] - quad[0], quad[7] - quad[1]);
    right_h = hypotf(quad[4] - quad[2], quad[5] - quad[3]);
    dw = (int)(fmaxf(top_w, bottom_w) + 0.5f);
    dh = (int)(fmaxf(left_h, right_h) + 0.5f);
    if (dw < 1) dw = 1;
    if (dh < 1) dh = 1;
    if (dw > dst_cap_w) dw = dst_cap_w;
    if (dh > dst_cap_h) dh = dst_cap_h;
    if (dw <= 0 || dh <= 0)
        return false;

    for (y = 0; y < dh; y++) {
        float ty = (dh > 1) ? ((float)y / (float)(dh - 1)) : 0.0f;
        float lx = quad[0] + (quad[6] - quad[0]) * ty;
        float ly = quad[1] + (quad[7] - quad[1]) * ty;
        float rx = quad[2] + (quad[4] - quad[2]) * ty;
        float ry = quad[3] + (quad[5] - quad[3]) * ty;
        for (x = 0; x < dw; x++) {
            float tx = (dw > 1) ? ((float)x / (float)(dw - 1)) : 0.0f;
            float sx = lx + (rx - lx) * tx;
            float sy = ly + (ry - ly) * tx;
            uint8_t pix[3];
            uint8_t *q;
            if (sx < 0.0f) sx = 0.0f;
            if (sy < 0.0f) sy = 0.0f;
            if (sx > (float)(img_w - 1)) sx = (float)(img_w - 1);
            if (sy > (float)(img_h - 1)) sy = (float)(img_h - 1);
            bilinear_sample_rgb888(rgb, img_w, img_h, sx, sy, pix);
            q = dst + ((size_t)y * (size_t)dw + (size_t)x) * 3U;
            q[0] = pix[0];
            q[1] = pix[1];
            q[2] = pix[2];
        }
    }

    *out_w = dw;
    *out_h = dh;
    return true;
}

static float quad_area8(const float q[8])
{
    float s = 0.0f;
    int i;
    for (i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        s += q[i * 2 + 0] * q[j * 2 + 1] - q[j * 2 + 0] * q[i * 2 + 1];
    }
    return fabsf(s) * 0.5f;
}

static void quad_center8(const float q[8], float *cx, float *cy)
{
    int i;
    float sx = 0.0f, sy = 0.0f;
    for (i = 0; i < 4; i++) {
        sx += q[i * 2 + 0];
        sy += q[i * 2 + 1];
    }
    *cx = sx * 0.25f;
    *cy = sy * 0.25f;
}

static float quad_edge_len8(const float q[8], int i)
{
    int j = (i + 1) % 4;
    float dx = q[j * 2 + 0] - q[i * 2 + 0];
    float dy = q[j * 2 + 1] - q[i * 2 + 1];
    return hypotf(dx, dy);
}

static bool quad_is_convex8(const float q[8])
{
    int i;
    bool has_pos = false, has_neg = false;
    for (i = 0; i < 4; i++) {
        int i1 = (i + 1) % 4;
        int i2 = (i + 2) % 4;
        float ax = q[i1 * 2 + 0] - q[i * 2 + 0];
        float ay = q[i1 * 2 + 1] - q[i * 2 + 1];
        float bx = q[i2 * 2 + 0] - q[i1 * 2 + 0];
        float by = q[i2 * 2 + 1] - q[i1 * 2 + 1];
        float cross = ax * by - ay * bx;
        if (cross > 1e-5f) has_pos = true;
        if (cross < -1e-5f) has_neg = true;
    }
    return !(has_pos && has_neg);
}

static void rect_box_to_quad(const struct det_box *box, float quad[8])
{
    if (!box || !quad)
        return;
    quad[0] = (float)box->x1; quad[1] = (float)box->y1;
    quad[2] = (float)box->x2; quad[3] = (float)box->y1;
    quad[4] = (float)box->x2; quad[5] = (float)box->y2;
    quad[6] = (float)box->x1; quad[7] = (float)box->y2;
}

static void bbox_from_quad_float(const float q[8], int img_w, int img_h,
                                 int *x1, int *y1, int *x2, int *y2)
{
    float minx = q[0], maxx = q[0], miny = q[1], maxy = q[1];
    int i;
    for (i = 1; i < 4; i++) {
        float x = q[i * 2 + 0];
        float y = q[i * 2 + 1];
        if (x < minx) minx = x;
        if (x > maxx) maxx = x;
        if (y < miny) miny = y;
        if (y > maxy) maxy = y;
    }
    *x1 = (int)floorf(minx + 1e-6f);
    *y1 = (int)floorf(miny + 1e-6f);
    *x2 = (int)ceilf(maxx - 1e-6f);
    *y2 = (int)ceilf(maxy - 1e-6f);
    if (*x1 < 0) *x1 = 0;
    if (*y1 < 0) *y1 = 0;
    if (*x2 >= img_w) *x2 = img_w - 1;
    if (*y2 >= img_h) *y2 = img_h - 1;
}

static bool decode_refiner_output_layout(const rknn_tensor_attr *a, int *h, int *w, int *c, bool *is_nchw)
{
    if (!a || !h || !w || !c || !is_nchw)
        return false;
    if (a->n_dims != 4)
        return false;
    if (a->fmt == RKNN_TENSOR_NCHW) {
        *is_nchw = true;
        *c = (int)a->dims[1];
        *h = (int)a->dims[2];
        *w = (int)a->dims[3];
    } else {
        *is_nchw = false;
        *h = (int)a->dims[1];
        *w = (int)a->dims[2];
        *c = (int)a->dims[3];
    }
    return (*h > 0 && *w > 0 && *c > 0);
}

static bool run_quad_refiner(const struct app_ctx *ctx,
                              const uint8_t *rgb, int img_w, int img_h,
                              const float coarse_quad[8],
                              float refined_quad_out[8])
{
    const struct quad_refiner_model *m = &ctx->quad_refiner_model;
    rknn_input in;
    rknn_output outs[4];
    float ordered[8];
    float pred_ordered[8];
    int patch_x1, patch_y1, patch_x2, patch_y2;
    int patch_w, patch_h;
    float *input_buf = NULL;
    float *heatmaps = NULL;
    float *offsets_buf = NULL;
    float corner_conf[4] = {0};
    int ref_w, ref_h;
    int hm_h, hm_w, hm_c;
    int off_h = 0, off_w = 0, off_c = 0;
    bool hm_is_nchw = false;
    bool off_is_nchw = false;
    bool has_offset = false;
    float scale_x, scale_y;
    size_t input_count;
    size_t input_size;
    int i, c;
    int ret = -1;
    const char *reject_reason = "unknown";
    float reject_metric = 0.0f;
    float reject_limit = 0.0f;
    float gate_area_ratio = 0.0f;
    float gate_center_shift = 0.0f;
    float gate_center_shift_limit = 0.0f;
    float gate_max_corner_shift = 0.0f;
    float gate_corner_shift_limit = 0.0f;
    float gate_edge_ratio_sanity = 0.0f;
    float gate_edge_ratio_limit = 0.0f;
    const float pad_x_ratio = 0.20f;
    const float pad_y_ratio = 0.25f;
    const float min_corner_conf = 0.20f;
    const float min_area_ratio = 0.50f;
    const float max_area_ratio = 1.80f;
    const float max_center_shift_ratio = 0.30f;
    const float max_corner_shift_ratio = 0.28f;
    const float max_edge_ratio_ratio = 3.0f;

    if (!ctx || !rgb || !coarse_quad || !refined_quad_out || !m->ctx)
        return false;

    order_quad_points(coarse_quad, ordered);
    bbox_from_quad_float(ordered, img_w, img_h, &patch_x1, &patch_y1, &patch_x2, &patch_y2);
    {
        int ex = (int)lroundf((float)(patch_x2 - patch_x1 + 1) * pad_x_ratio);
        int ey = (int)lroundf((float)(patch_y2 - patch_y1 + 1) * pad_y_ratio);
        patch_x1 -= ex; patch_x2 += ex;
        patch_y1 -= ey; patch_y2 += ey;
        if (patch_x1 < 0) patch_x1 = 0;
        if (patch_y1 < 0) patch_y1 = 0;
        if (patch_x2 >= img_w) patch_x2 = img_w - 1;
        if (patch_y2 >= img_h) patch_y2 = img_h - 1;
    }
    patch_w = patch_x2 - patch_x1 + 1;
    patch_h = patch_y2 - patch_y1 + 1;
    if (patch_w < 4 || patch_h < 4)
        return false;

    ref_w = (int)m->in_w;
    ref_h = (int)m->in_h;
    if (ref_w <= 1 || ref_h <= 1)
        return false;

    input_count = (size_t)3 * (size_t)ref_h * (size_t)ref_w;
    input_size = input_count * sizeof(float);
    input_buf = (float *)malloc(input_size);
    if (!input_buf)
        return false;

    if (m->input_attr.fmt == RKNN_TENSOR_NCHW) {
        for (int dy = 0; dy < ref_h; dy++) {
            float sy = (ref_h > 1) ? ((float)dy * (float)(patch_h - 1) / (float)(ref_h - 1)) : 0.0f;
            for (int dx = 0; dx < ref_w; dx++) {
                float sx = (ref_w > 1) ? ((float)dx * (float)(patch_w - 1) / (float)(ref_w - 1)) : 0.0f;
                uint8_t pix[3];
                bilinear_sample_rgb888(rgb, img_w, img_h, (float)patch_x1 + sx, (float)patch_y1 + sy, pix);
                for (c = 0; c < 3; c++)
                    input_buf[((size_t)c * ref_h + (size_t)dy) * ref_w + (size_t)dx] = (float)pix[c] / 255.0f;
            }
        }
    } else {
        for (int dy = 0; dy < ref_h; dy++) {
            float sy = (ref_h > 1) ? ((float)dy * (float)(patch_h - 1) / (float)(ref_h - 1)) : 0.0f;
            for (int dx = 0; dx < ref_w; dx++) {
                float sx = (ref_w > 1) ? ((float)dx * (float)(patch_w - 1) / (float)(ref_w - 1)) : 0.0f;
                uint8_t pix[3];
                bilinear_sample_rgb888(rgb, img_w, img_h, (float)patch_x1 + sx, (float)patch_y1 + sy, pix);
                for (c = 0; c < 3; c++)
                    input_buf[((size_t)dy * ref_w + (size_t)dx) * 3 + (size_t)c] = (float)pix[c] / 255.0f;
            }
        }
    }

    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf = input_buf;
    in.size = input_size;
    in.type = RKNN_TENSOR_FLOAT32;
    in.fmt = m->input_attr.fmt;
    ret = rknn_inputs_set(m->ctx, 1, &in);
    if (ret < 0)
        goto out;
    ret = rknn_run(m->ctx, NULL);
    if (ret < 0)
        goto out;

    memset(outs, 0, sizeof(outs));
    for (i = 0; i < (int)m->io_num.n_output; i++)
        outs[i].want_float = 1;
    ret = rknn_outputs_get(m->ctx, m->io_num.n_output, outs, NULL);
    if (ret < 0)
        goto out;

    if (!decode_refiner_output_layout(&m->output_attrs[0], &hm_h, &hm_w, &hm_c, &hm_is_nchw))
        goto out_release;
    if (hm_c != 4)
        goto out_release;

    heatmaps = (float *)malloc((size_t)hm_h * (size_t)hm_w * 4U * sizeof(float));
    if (!heatmaps)
        goto out_release;

    {
        const float *src = (const float *)outs[0].buf;
        if (hm_is_nchw) {
            memcpy(heatmaps, src, (size_t)hm_h * (size_t)hm_w * 4U * sizeof(float));
        } else {
            for (int y = 0; y < hm_h; y++) {
                for (int x = 0; x < hm_w; x++) {
                    for (c = 0; c < 4; c++) {
                        heatmaps[((size_t)c * hm_h + (size_t)y) * hm_w + (size_t)x] =
                            src[((size_t)y * hm_w + (size_t)x) * 4U + (size_t)c];
                    }
                }
            }
        }
    }

    scale_x = (hm_w > 1 && ref_w > 1) ? (float)(ref_w - 1) / (float)(hm_w - 1) : 0.0f;
    scale_y = (hm_h > 1 && ref_h > 1) ? (float)(ref_h - 1) / (float)(hm_h - 1) : 0.0f;

    has_offset = (m->io_num.n_output >= 3);
    if (has_offset) {
        if (decode_refiner_output_layout(&m->output_attrs[2], &off_h, &off_w, &off_c, &off_is_nchw) && off_c == 8) {
            offsets_buf = (float *)malloc((size_t)off_h * (size_t)off_w * 8U * sizeof(float));
            if (offsets_buf) {
                const float *src = (const float *)outs[2].buf;
                if (off_is_nchw) {
                    memcpy(offsets_buf, src, (size_t)off_h * (size_t)off_w * 8U * sizeof(float));
                } else {
                    for (int y = 0; y < off_h; y++) {
                        for (int x = 0; x < off_w; x++) {
                            for (int ch = 0; ch < 8; ch++) {
                                offsets_buf[((size_t)ch * off_h + (size_t)y) * off_w + (size_t)x] =
                                    src[((size_t)y * off_w + (size_t)x) * 8U + (size_t)ch];
                            }
                        }
                    }
                }
            }
        } else {
            has_offset = false;
        }
    }

    for (c = 0; c < 4; c++) {
        float *ch = heatmaps + (size_t)c * hm_h * hm_w;
        float best = -1e30f;
        int best_idx = 0;
        int by, bx;
        float rx, ry;
        for (i = 0; i < hm_h * hm_w; i++) {
            float v = ch[i];
            if (v >= 0.0f) v = 1.0f / (1.0f + expf(-v));
            else {
                float ev = expf(v);
                v = ev / (1.0f + ev);
            }
            ch[i] = v;
            if (v > best) {
                best = v;
                best_idx = i;
            }
        }
        corner_conf[c] = best;
        by = best_idx / hm_w;
        bx = best_idx % hm_w;
        if (offsets_buf && bx < off_w && by < off_h) {
            size_t dx_idx = ((size_t)c * 2U * (size_t)off_h + (size_t)by) * (size_t)off_w + (size_t)bx;
            size_t dy_idx = (((size_t)c * 2U + 1U) * (size_t)off_h + (size_t)by) * (size_t)off_w + (size_t)bx;
            rx = (float)bx + offsets_buf[dx_idx];
            ry = (float)by + offsets_buf[dy_idx];
        } else {
            int x0 = (bx > 0) ? bx - 1 : 0;
            int x1 = (bx + 1 < hm_w) ? bx + 1 : hm_w - 1;
            int y0 = (by > 0) ? by - 1 : 0;
            int y1 = (by + 1 < hm_h) ? by + 1 : hm_h - 1;
            float total = 0.0f, wx = 0.0f, wy = 0.0f;
            int yy, xx;
            for (yy = y0; yy <= y1; yy++) {
                for (xx = x0; xx <= x1; xx++) {
                    float w = ch[yy * hm_w + xx];
                    total += w;
                    wx += w * (float)xx;
                    wy += w * (float)yy;
                }
            }
            if (total > 1e-6f) {
                rx = wx / total;
                ry = wy / total;
            } else {
                rx = (float)bx;
                ry = (float)by;
            }
        }
        pred_ordered[c * 2 + 0] = (rx * scale_x) * ((float)(patch_w - 1) / (float)(ref_w - 1 + 1e-6f)) + (float)patch_x1;
        pred_ordered[c * 2 + 1] = (ry * scale_y) * ((float)(patch_h - 1) / (float)(ref_h - 1 + 1e-6f)) + (float)patch_y1;
    }

    order_quad_points(pred_ordered, refined_quad_out);

    {
        float coarse_area = fmaxf(quad_area8(ordered), 1e-6f);
        float refined_area = quad_area8(refined_quad_out);
        float area_ratio = refined_area / coarse_area;
        float coarse_cx, coarse_cy, refined_cx, refined_cy;
        float center_shift;
        float max_corner_shift = 0.0f;
        float edge_ratio_sanity = 1.0f;
        float patch_diag = hypotf((float)ref_w, (float)ref_h);
        quad_center8(ordered, &coarse_cx, &coarse_cy);
        quad_center8(refined_quad_out, &refined_cx, &refined_cy);
        center_shift = hypotf(refined_cx - coarse_cx, refined_cy - coarse_cy);
        for (i = 0; i < 4; i++) {
            float shift = hypotf(refined_quad_out[i * 2 + 0] - ordered[i * 2 + 0],
                                 refined_quad_out[i * 2 + 1] - ordered[i * 2 + 1]);
            float ce = fmaxf(quad_edge_len8(ordered, i), 1e-6f);
            float re = fmaxf(quad_edge_len8(refined_quad_out, i), 1e-6f);
            float ratio = re / ce;
            if (shift > max_corner_shift) max_corner_shift = shift;
            if (ratio > edge_ratio_sanity) edge_ratio_sanity = ratio;
            if ((1.0f / ratio) > edge_ratio_sanity) edge_ratio_sanity = 1.0f / ratio;
        }
        gate_area_ratio = area_ratio;
        gate_center_shift = center_shift;
        gate_center_shift_limit = max_center_shift_ratio * patch_diag;
        gate_max_corner_shift = max_corner_shift;
        gate_corner_shift_limit = max_corner_shift_ratio * patch_diag;
        gate_edge_ratio_sanity = edge_ratio_sanity;
        gate_edge_ratio_limit = max_edge_ratio_ratio;
        if (corner_conf[0] < min_corner_conf || corner_conf[1] < min_corner_conf ||
            corner_conf[2] < min_corner_conf || corner_conf[3] < min_corner_conf) {
            float min_conf_seen = corner_conf[0];
            if (corner_conf[1] < min_conf_seen) min_conf_seen = corner_conf[1];
            if (corner_conf[2] < min_conf_seen) min_conf_seen = corner_conf[2];
            if (corner_conf[3] < min_conf_seen) min_conf_seen = corner_conf[3];
            reject_reason = "low_corner_conf";
            reject_metric = min_conf_seen;
            reject_limit = min_corner_conf;
            goto out_release;
        }
        if (!quad_is_convex8(refined_quad_out)) {
            reject_reason = "non_convex";
            goto out_release;
        }
        if (area_ratio < min_area_ratio) {
            reject_reason = "area_ratio_low";
            reject_metric = area_ratio;
            reject_limit = min_area_ratio;
            goto out_release;
        }
        if (area_ratio > max_area_ratio) {
            reject_reason = "area_ratio_high";
            reject_metric = area_ratio;
            reject_limit = max_area_ratio;
            goto out_release;
        }
        if (center_shift > max_center_shift_ratio * patch_diag) {
            reject_reason = "center_shift";
            reject_metric = center_shift;
            reject_limit = max_center_shift_ratio * patch_diag;
            goto out_release;
        }
        if (max_corner_shift > max_corner_shift_ratio * patch_diag) {
            reject_reason = "corner_shift";
            reject_metric = max_corner_shift;
            reject_limit = max_corner_shift_ratio * patch_diag;
            goto out_release;
        }
        if (edge_ratio_sanity > max_edge_ratio_ratio) {
            reject_reason = "edge_ratio";
            reject_metric = edge_ratio_sanity;
            reject_limit = max_edge_ratio_ratio;
            goto out_release;
        }
        if (refined_area < 4.0f) {
            reject_reason = "refined_area";
            reject_metric = refined_area;
            reject_limit = 4.0f;
            goto out_release;
        }
    }

    ret = 0;
out_release:
    if (m->ctx)
        rknn_outputs_release(m->ctx, m->io_num.n_output, outs);
out:
    if (offsets_buf)
        free(offsets_buf);
    free(input_buf);
    free(heatmaps);
    if (ret == 0) {
        fprintf(stderr,
                "[quad_refiner] gate ACCEPT conf=[%.3f %.3f %.3f %.3f] area_ratio=%.3f center_shift=%.3f/%.3f corner_shift=%.3f/%.3f edge_ratio=%.3f/%.3f\n",
                corner_conf[0], corner_conf[1], corner_conf[2], corner_conf[3],
                gate_area_ratio,
                gate_center_shift, gate_center_shift_limit,
                gate_max_corner_shift, gate_corner_shift_limit,
                gate_edge_ratio_sanity, gate_edge_ratio_limit);
        return true;
    }
    fprintf(stderr,
            "[quad_refiner] gate REJECT/FAIL reason=%s metric=%.3f limit=%.3f conf=[%.3f %.3f %.3f %.3f]\n",
            reject_reason, reject_metric, reject_limit,
            corner_conf[0], corner_conf[1], corner_conf[2], corner_conf[3]);
    return false;
}

static bool prepare_plate_crop_rgb888(const struct app_ctx *ctx,
                                      const uint8_t *rgb, int img_w, int img_h,
                                      const struct det_box *plate_box,
                                      uint8_t *crop_buf, int crop_cap_w, int crop_cap_h,
                                      struct det_box *crop_box, int *crop_w, int *crop_h,
                                      float *occ_ratio, bool *used_obb_warp,
                                      uint64_t frame_id)
{
    float ordered[8];
    float coarse_ordered[8];
    float refined_quad[8];
    bool have_refined = false;
    if (!ctx || !rgb || !plate_box || !crop_buf || !crop_box || !crop_w || !crop_h || !occ_ratio || !used_obb_warp)
        return false;
    *used_obb_warp = false;

    if (ctx->opt.ocr_crop_mode == OCR_CROP_OBB_WARP ||
        ctx->opt.ocr_crop_mode == OCR_CROP_OBB_PIECEWISE) {
        bool use_piecewise = (ctx->opt.ocr_crop_mode == OCR_CROP_OBB_PIECEWISE);
        bool warp_ok;
        if (plate_box->has_obb)
            order_quad_points(plate_box->quad, ordered);
        else
            rect_box_to_quad(plate_box, ordered);
        memcpy(coarse_ordered, ordered, sizeof(coarse_ordered));
        /* Run quad refiner before OBB warp if model is loaded */
        if (ctx->quad_refiner_model.ctx) {
            if (run_quad_refiner(ctx, rgb, img_w, img_h, ordered, refined_quad)) {
                /* Gate accepted: replace ordered quad with refined quad */
                memcpy(ordered, refined_quad, sizeof(float) * 8);
                have_refined = true;
            }
        }
        warp_ok = use_piecewise ?
            warp_quad_to_rect_piecewise_rgb888(rgb, img_w, img_h, ordered,
                                               crop_buf, crop_cap_w, crop_cap_h,
                                               crop_w, crop_h) :
            warp_quad_to_rect_rgb888(rgb, img_w, img_h, ordered,
                                     crop_buf, crop_cap_w, crop_cap_h,
                                     crop_w, crop_h);
        if (warp_ok) {
            bbox_from_quad(ordered, img_w, img_h, crop_box);
            *occ_ratio = estimate_ocr_occ_ratio(ctx, *crop_w, *crop_h);
            *used_obb_warp = true;
            if (ctx->ocr_crop_index_fp && ctx->ocr_crop_dumped < ctx->opt.ocr_crop_dump_max) {
                uint8_t *coarse_crop = malloc((size_t)crop_cap_w * (size_t)crop_cap_h * 3U);
                int coarse_w = 0, coarse_h = 0;
                int sample_id = ctx->ocr_crop_dumped;
                if (coarse_crop) {
                    bool coarse_ok = use_piecewise ?
                        warp_quad_to_rect_piecewise_rgb888(rgb, img_w, img_h, coarse_ordered,
                                                           coarse_crop, crop_cap_w, crop_cap_h,
                                                           &coarse_w, &coarse_h) :
                        warp_quad_to_rect_rgb888(rgb, img_w, img_h, coarse_ordered,
                                                 coarse_crop, crop_cap_w, crop_cap_h,
                                                 &coarse_w, &coarse_h);
                    if (coarse_ok) {
                        dump_ocr_ab_variant(ctx, frame_id, sample_id, "coarse", coarse_ordered,
                                            coarse_crop, coarse_w, coarse_h);
                    }
                    free(coarse_crop);
                }
                if (have_refined) {
                    dump_ocr_ab_variant(ctx, frame_id, sample_id, "refined", ordered,
                                        crop_buf, *crop_w, *crop_h);
                    fprintf(stderr,
                            "[ocr-ab] frame=%" PRIu64 " sample=%d mode=%s dump=coarse/refined refiner=accepted\n",
                            frame_id, sample_id, use_piecewise ? "piecewise" : "homography");
                } else {
                    fprintf(stderr,
                            "[ocr-ab] frame=%" PRIu64 " sample=%d mode=%s dump=coarse refiner=not_accepted\n",
                            frame_id, sample_id, use_piecewise ? "piecewise" : "homography");
                }
            }
            return true;
        }
    }

    compute_ocr_crop_box(ctx, plate_box, crop_box);
    *crop_w = crop_box->x2 - crop_box->x1 + 1;
    *crop_h = crop_box->y2 - crop_box->y1 + 1;
    if (*crop_w <= 0 || *crop_h <= 0 || *crop_w > crop_cap_w || *crop_h > crop_cap_h)
        return false;
    copy_crop_rgb888(rgb, img_w, crop_box, crop_buf);
    *occ_ratio = estimate_ocr_occ_ratio(ctx, *crop_w, *crop_h);
    return true;
}

static bool refine_plate_box_local(struct app_ctx *ctx, const uint8_t *rgb_full, int img_w, int img_h,
                                   const struct det_box *seed_box, uint8_t *det_rgb, uint8_t *plate_in,
                                   struct det_box *out_box)
{
    struct det_box roi;
    struct det_box cand[MAX_DETS];
    int roi_w, roi_h;
    int count = 0;
    uint8_t *roi_rgb = NULL;
    int seed_cx, seed_cy;
    float best_score = -1e9f;
    int best = -1;
    int i;
    float det_thr;

    if (!ctx || !rgb_full || !seed_box || !det_rgb || !plate_in || !out_box)
        return false;

    if (!ctx->opt.plate_refine)
        return false;

    compute_expand_crop_box(seed_box, img_w, img_h, 0.40f, 0.40f, &roi);
    roi_w = roi.x2 - roi.x1 + 1;
    roi_h = roi.y2 - roi.y1 + 1;
    if (roi_w < 8 || roi_h < 8)
        return false;

    roi_rgb = malloc((size_t)roi_w * roi_h * 3U);
    if (!roi_rgb)
        return false;
    copy_crop_rgb888(rgb_full, img_w, &roi, roi_rgb);

    det_thr = fmaxf(0.03f, ctx->opt.min_plate_conf * 0.8f);
    if (run_detect_on_rgb(ctx, &ctx->plate_model, roi_rgb, roi_w, roi_h,
                          det_thr, det_rgb, plate_in, cand, &count, NULL) < 0 || count <= 0) {
        free(roi_rgb);
        return false;
    }

    seed_cx = (seed_box->x1 + seed_box->x2) / 2;
    seed_cy = (seed_box->y1 + seed_box->y2) / 2;
    for (i = 0; i < count; i++) {
        int cx = (cand[i].x1 + cand[i].x2) / 2 + roi.x1;
        int cy = (cand[i].y1 + cand[i].y2) / 2 + roi.y1;
        float dx = fabsf((float)(cx - seed_cx)) / (float)(roi_w > 0 ? roi_w : 1);
        float dy = fabsf((float)(cy - seed_cy)) / (float)(roi_h > 0 ? roi_h : 1);
        float score = cand[i].conf - 0.25f * (dx + dy);
        if (score > best_score) {
            best_score = score;
            best = i;
        }
    }
    if (best < 0) {
        free(roi_rgb);
        return false;
    }

    *out_box = cand[best];
    out_box->x1 += roi.x1;
    out_box->x2 += roi.x1;
    out_box->y1 += roi.y1;
    out_box->y2 += roi.y1;
    if (out_box->has_obb) {
        int k;
        out_box->cx += (float)roi.x1;
        out_box->cy += (float)roi.y1;
        for (k = 0; k < 4; k++) {
            out_box->quad[k * 2 + 0] += (float)roi.x1;
            out_box->quad[k * 2 + 1] += (float)roi.y1;
        }
    }
    clamp_box(out_box, img_w, img_h);

    free(roi_rgb);
    return true;
}

static float rows_get_value(const float *buf, bool transposed, int n_rows, int n_cols, int r, int c)
{
    if (transposed)
        return buf[(size_t)c * (size_t)n_rows + (size_t)r];
    return buf[(size_t)r * (size_t)n_cols + (size_t)c];
}

static bool rows_coords_mostly_normalized(const float *buf, bool transposed, int n_rows, int n_cols)
{
    int i;
    int sample = (n_rows < 128) ? n_rows : 128;
    int total = 0;
    int in01 = 0;
    for (i = 0; i < sample; i++) {
        int k;
        for (k = 0; k < 4; k++) {
            float v = rows_get_value(buf, transposed, n_rows, n_cols, i, k);
            if (v >= 0.0f && v <= 1.0f)
                in01++;
            total++;
        }
    }
    if (total <= 0)
        return false;
    return ((float)in01 / (float)total) >= 0.75f;
}

static bool rows_classid_like(const float *buf, bool transposed, int n_rows, int n_cols, int class_count)
{
    int i;
    int sample = (n_rows < 128) ? n_rows : 128;
    int near_int = 0;
    int in_range = 0;
    if (n_cols < 6)
        return false;
    if (class_count <= 0)
        class_count = 1;
    for (i = 0; i < sample; i++) {
        float clsf = rows_get_value(buf, transposed, n_rows, n_cols, i, 5);
        int cls = (int)lroundf(clsf);
        if (fabsf(clsf - (float)cls) <= 0.15f)
            near_int++;
        if (cls >= 0 && cls < class_count)
            in_range++;
    }
    return (near_int >= (sample * 7) / 10) && (in_range >= (sample * 7) / 10);
}

static int decode_rows_mode_xywh(const float *buf, bool transposed, int n_rows, int n_cols, int class_count,
                                 float conf_thr, int src_w, int src_h, int in_w, int in_h,
                                 struct det_box *out, float *avg_conf_out)
{
    int i;
    int count = 0;
    float conf_sum = 0.0f;
    int cls_lim = class_count;

    if (n_cols < 6)
        return 0;
    if (cls_lim <= 0)
        cls_lim = 1;
    if (cls_lim > n_cols - 5)
        cls_lim = n_cols - 5;

    for (i = 0; i < n_rows && count < MAX_DETS; i++) {
        int c;
        float obj = rows_get_value(buf, transposed, n_rows, n_cols, i, 4);
        float best = (cls_lim > 0) ? 0.0f : 1.0f;
        int best_id = 0;
        float cx = rows_get_value(buf, transposed, n_rows, n_cols, i, 0);
        float cy = rows_get_value(buf, transposed, n_rows, n_cols, i, 1);
        float bw = rows_get_value(buf, transposed, n_rows, n_cols, i, 2);
        float bh = rows_get_value(buf, transposed, n_rows, n_cols, i, 3);
        struct det_box b;

        for (c = 0; c < cls_lim; c++) {
            float p = rows_get_value(buf, transposed, n_rows, n_cols, i, 5 + c);
            if (p > best) {
                best = p;
                best_id = c;
            }
        }
        if (obj <= 1.0f) obj = sigmoidf_local(obj);
        if (best <= 1.0f) best = sigmoidf_local(best);
        if (obj * best < conf_thr)
            continue;

        if (bw <= 2.0f && bh <= 2.0f) {
            cx *= (float)in_w;
            cy *= (float)in_h;
            bw *= (float)in_w;
            bh *= (float)in_h;
        }

        memset(&b, 0, sizeof(b));
        b.x1 = (int)((cx - bw * 0.5f) * ((float)src_w / (float)in_w));
        b.y1 = (int)((cy - bh * 0.5f) * ((float)src_h / (float)in_h));
        b.x2 = (int)((cx + bw * 0.5f) * ((float)src_w / (float)in_w));
        b.y2 = (int)((cy + bh * 0.5f) * ((float)src_h / (float)in_h));
        b.conf = obj * best;
        b.cls = best_id;
        clamp_box(&b, src_w, src_h);
        out[count++] = b;
        conf_sum += b.conf;
    }

    if (avg_conf_out)
        *avg_conf_out = (count > 0) ? (conf_sum / (float)count) : 0.0f;
    return count;
}

static int decode_rows_mode_xyxy_clsid(const float *buf, bool transposed, int n_rows, int n_cols, int class_count,
                                       float conf_thr, int src_w, int src_h, int in_w, int in_h,
                                       struct det_box *out, float *avg_conf_out)
{
    int i;
    int count = 0;
    float conf_sum = 0.0f;
    bool normalized = rows_coords_mostly_normalized(buf, transposed, n_rows, n_cols);
    bool clsid_hint = rows_classid_like(buf, transposed, n_rows, n_cols, class_count);

    if (n_cols < 6 || !clsid_hint)
        return 0;
    if (class_count <= 0)
        class_count = 1;

    for (i = 0; i < n_rows && count < MAX_DETS; i++) {
        float x1 = rows_get_value(buf, transposed, n_rows, n_cols, i, 0);
        float y1 = rows_get_value(buf, transposed, n_rows, n_cols, i, 1);
        float x2 = rows_get_value(buf, transposed, n_rows, n_cols, i, 2);
        float y2 = rows_get_value(buf, transposed, n_rows, n_cols, i, 3);
        float score = rows_get_value(buf, transposed, n_rows, n_cols, i, 4);
        int cls = (int)lroundf(rows_get_value(buf, transposed, n_rows, n_cols, i, 5));
        struct det_box b;
        float max_x;
        float max_y;

        if (score < 0.0f || score > 1.0f)
            score = sigmoidf_local(score);
        if (score < conf_thr)
            continue;
        if (cls < 0 || cls >= class_count)
            continue;

        if (normalized) {
            x1 *= (float)in_w;
            x2 *= (float)in_w;
            y1 *= (float)in_h;
            y2 *= (float)in_h;
        }

        if (x2 < x1) { float t = x1; x1 = x2; x2 = t; }
        if (y2 < y1) { float t = y1; y1 = y2; y2 = t; }
        max_x = fmaxf(fabsf(x1), fabsf(x2));
        max_y = fmaxf(fabsf(y1), fabsf(y2));

        memset(&b, 0, sizeof(b));
        if (max_x <= (float)in_w * 1.5f && max_y <= (float)in_h * 1.5f) {
            b.x1 = (int)(x1 * ((float)src_w / (float)in_w));
            b.y1 = (int)(y1 * ((float)src_h / (float)in_h));
            b.x2 = (int)(x2 * ((float)src_w / (float)in_w));
            b.y2 = (int)(y2 * ((float)src_h / (float)in_h));
        } else {
            b.x1 = (int)x1;
            b.y1 = (int)y1;
            b.x2 = (int)x2;
            b.y2 = (int)y2;
        }
        b.conf = score;
        b.cls = cls;
        clamp_box(&b, src_w, src_h);
        out[count++] = b;
        conf_sum += b.conf;
    }

    if (avg_conf_out)
        *avg_conf_out = (count > 0) ? (conf_sum / (float)count) : 0.0f;
    return count;
}

static int decode_rows_tensor_output(const rknn_tensor_attr *a, const float *buf, int class_count,
                                     float conf_thr, int src_w, int src_h, int in_w, int in_h,
                                     struct det_box *out, int *out_count)
{
    struct det_box xywh_out[MAX_DETS];
    struct det_box xyxy_out[MAX_DETS];
    int n_rows;
    int n_cols;
    bool transposed = false;
    int xywh_count = 0;
    int xyxy_count = 0;
    float xywh_avg = 0.0f;
    float xyxy_avg = 0.0f;

    *out_count = 0;
    if (a->n_dims != 3)
        return 0;

    n_rows = (int)a->dims[1];
    n_cols = (int)a->dims[2];
    if (n_cols < 6 && n_rows >= 6) {
        int t = n_rows;
        n_rows = n_cols;
        n_cols = t;
        transposed = true;
    }
    if (n_cols < 6 || n_rows <= 0)
        return 0;

    xywh_count = decode_rows_mode_xywh(buf, transposed, n_rows, n_cols, class_count,
                                       conf_thr, src_w, src_h, in_w, in_h,
                                       xywh_out, &xywh_avg);
    xyxy_count = decode_rows_mode_xyxy_clsid(buf, transposed, n_rows, n_cols, class_count,
                                             conf_thr, src_w, src_h, in_w, in_h,
                                             xyxy_out, &xyxy_avg);

    if (xywh_count <= 0 && xyxy_count <= 0)
        return 0;
    if (xyxy_count <= 0 || (xywh_count > 0 && ((float)xywh_count + xywh_avg * 0.5f >= (float)xyxy_count + xyxy_avg * 0.5f))) {
        memcpy(out, xywh_out, (size_t)xywh_count * sizeof(out[0]));
        *out_count = xywh_count;
        return 1; /* xywh */
    }

    memcpy(out, xyxy_out, (size_t)xyxy_count * sizeof(out[0]));
    *out_count = xyxy_count;
    return 2; /* xyxy+clsid */
}

enum yolo_head_layout {
    YOLO_HEAD_4D_NCHW = 0,
    YOLO_HEAD_4D_NHWC,
    YOLO_HEAD_5D_AHWA,
    YOLO_HEAD_5D_HWAA
};

struct yolo_head_view {
    uint32_t out_idx;
    int h;
    int w;
    int anchors;
    int attrs;
    int stride;
    int layout;
};

static bool parse_yolo_head_view(const struct yolo_model *m, uint32_t out_idx, struct yolo_head_view *hv)
{
    const rknn_tensor_attr *a = &m->output_attrs[out_idx];
    int h = 0;
    int w = 0;
    int anchors = 3;
    int attrs = 0;
    int layout = YOLO_HEAD_4D_NCHW;

    if (a->n_dims == 4) {
        int c = 0;
        if (a->fmt == RKNN_TENSOR_NCHW) {
            c = (int)a->dims[1];
            h = (int)a->dims[2];
            w = (int)a->dims[3];
            layout = YOLO_HEAD_4D_NCHW;
        } else if (a->fmt == RKNN_TENSOR_NHWC) {
            h = (int)a->dims[1];
            w = (int)a->dims[2];
            c = (int)a->dims[3];
            layout = YOLO_HEAD_4D_NHWC;
        } else {
            if (a->dims[2] == a->dims[3] && a->dims[1] >= 18) {
                c = (int)a->dims[1];
                h = (int)a->dims[2];
                w = (int)a->dims[3];
                layout = YOLO_HEAD_4D_NCHW;
            } else if (a->dims[1] == a->dims[2] && a->dims[3] >= 18) {
                h = (int)a->dims[1];
                w = (int)a->dims[2];
                c = (int)a->dims[3];
                layout = YOLO_HEAD_4D_NHWC;
            } else {
                return false;
            }
        }
        if (c <= 0 || c % 3 != 0)
            return false;
        anchors = 3;
        attrs = c / 3;
    } else if (a->n_dims == 5) {
        int d1 = (int)a->dims[1];
        int d2 = (int)a->dims[2];
        int d3 = (int)a->dims[3];
        int d4 = (int)a->dims[4];
        int attrs_guess = (m->class_count > 0) ? (m->class_count + 5) : 6;

        if (d1 == 3 && d2 > 0 && d3 > 0) {
            h = d2;
            w = d3;
            anchors = 3;
            attrs = (d4 >= 6) ? d4 : attrs_guess;
            layout = YOLO_HEAD_5D_AHWA;
        } else if (d3 == 3 && d1 > 0 && d2 > 0) {
            h = d1;
            w = d2;
            anchors = 3;
            attrs = (d4 >= 6) ? d4 : attrs_guess;
            layout = YOLO_HEAD_5D_HWAA;
        } else {
            return false;
        }
    } else {
        return false;
    }

    if (h <= 0 || w <= 0 || anchors != 3 || attrs < 6)
        return false;

    hv->out_idx = out_idx;
    hv->h = h;
    hv->w = w;
    hv->anchors = anchors;
    hv->attrs = attrs;
    hv->layout = layout;
    hv->stride = (h > 0) ? ((int)m->in_h / h) : 0;
    return hv->stride > 0;
}

static void sort_heads_by_stride(struct yolo_head_view *heads, int n)
{
    int i;
    int j;
    for (i = 0; i < n; i++) {
        for (j = i + 1; j < n; j++) {
            if (heads[i].stride > heads[j].stride) {
                struct yolo_head_view t = heads[i];
                heads[i] = heads[j];
                heads[j] = t;
            }
        }
    }
}

static float head_read(const float *buf, const struct yolo_head_view *hv, int a, int gy, int gx, int k)
{
    size_t idx = 0;
    if (hv->layout == YOLO_HEAD_4D_NCHW) {
        int ch = a * hv->attrs + k;
        idx = ((size_t)ch * (size_t)hv->h + (size_t)gy) * (size_t)hv->w + (size_t)gx;
    } else if (hv->layout == YOLO_HEAD_4D_NHWC) {
        idx = ((size_t)gy * (size_t)hv->w + (size_t)gx) * (size_t)(hv->anchors * hv->attrs) +
              (size_t)(a * hv->attrs + k);
    } else if (hv->layout == YOLO_HEAD_5D_AHWA) {
        idx = ((((size_t)a * (size_t)hv->h + (size_t)gy) * (size_t)hv->w + (size_t)gx) * (size_t)hv->attrs) + (size_t)k;
    } else {
        idx = ((((size_t)gy * (size_t)hv->w + (size_t)gx) * (size_t)hv->anchors + (size_t)a) * (size_t)hv->attrs) + (size_t)k;
    }
    return buf[idx];
}

static void decode_yolo_head_output(const float *buf, const struct yolo_head_view *hv,
                                    const float anchors[3][2], int class_count, float conf_thr,
                                    int src_w, int src_h, int in_w, int in_h,
                                    struct det_box *out, int *out_count)
{
    int gy;
    int gx;
    int a;
    int classes = hv->attrs - 5;
    int cls_lim = class_count;
    if (classes <= 0)
        return;
    if (cls_lim <= 0)
        cls_lim = 1;
    if (cls_lim > classes)
        cls_lim = classes;

    for (gy = 0; gy < hv->h && *out_count < MAX_DETS; gy++) {
        for (gx = 0; gx < hv->w && *out_count < MAX_DETS; gx++) {
            for (a = 0; a < 3 && *out_count < MAX_DETS; a++) {
                float tx = head_read(buf, hv, a, gy, gx, 0);
                float ty = head_read(buf, hv, a, gy, gx, 1);
                float tw = head_read(buf, hv, a, gy, gx, 2);
                float th = head_read(buf, hv, a, gy, gx, 3);
                float to = head_read(buf, hv, a, gy, gx, 4);
                float obj = sigmoidf_local(to);
                float best = (cls_lim > 0) ? 0.0f : 1.0f;
                int best_id = 0;
                int c;
                struct det_box b;
                float bx;
                float by;
                float bw;
                float bh;
                float conf;

                if (obj < conf_thr * 0.5f)
                    continue;

                for (c = 0; c < cls_lim; c++) {
                    float p = sigmoidf_local(head_read(buf, hv, a, gy, gx, 5 + c));
                    if (p > best) {
                        best = p;
                        best_id = c;
                    }
                }

                conf = obj * best;
                if (conf < conf_thr)
                    continue;

                bx = ((sigmoidf_local(tx) * 2.0f - 0.5f) + (float)gx) * (float)hv->stride;
                by = ((sigmoidf_local(ty) * 2.0f - 0.5f) + (float)gy) * (float)hv->stride;
                bw = powf(sigmoidf_local(tw) * 2.0f, 2.0f) * anchors[a][0];
                bh = powf(sigmoidf_local(th) * 2.0f, 2.0f) * anchors[a][1];

                memset(&b, 0, sizeof(b));
                b.x1 = (int)((bx - bw * 0.5f) * ((float)src_w / (float)in_w));
                b.y1 = (int)((by - bh * 0.5f) * ((float)src_h / (float)in_h));
                b.x2 = (int)((bx + bw * 0.5f) * ((float)src_w / (float)in_w));
                b.y2 = (int)((by + bh * 0.5f) * ((float)src_h / (float)in_h));
                b.conf = conf;
                b.cls = best_id;
                clamp_box(&b, src_w, src_h);
                out[(*out_count)++] = b;
            }
        }
    }
}

static void decode_yolo_heads_outputs(const struct yolo_model *m, const rknn_output *outs,
                                      float conf_thr, int src_w, int src_h,
                                      struct det_box *out, int *out_count)
{
    static const float anchors_p5[3][3][2] = {
        {{10.0f, 13.0f}, {16.0f, 30.0f}, {33.0f, 23.0f}},
        {{30.0f, 61.0f}, {62.0f, 45.0f}, {59.0f, 119.0f}},
        {{116.0f, 90.0f}, {156.0f, 198.0f}, {373.0f, 326.0f}},
    };
    static const float anchors_p6[4][3][2] = {
        {{19.0f, 27.0f}, {44.0f, 40.0f}, {38.0f, 94.0f}},
        {{96.0f, 68.0f}, {86.0f, 152.0f}, {180.0f, 137.0f}},
        {{140.0f, 301.0f}, {303.0f, 264.0f}, {238.0f, 542.0f}},
        {{436.0f, 615.0f}, {739.0f, 380.0f}, {925.0f, 792.0f}},
    };
    struct yolo_head_view heads[8];
    int head_count = 0;
    uint32_t i;

    *out_count = 0;
    for (i = 0; i < m->io_num.n_output && head_count < 8; i++) {
        if (parse_yolo_head_view(m, i, &heads[head_count]))
            head_count++;
    }
    if (head_count == 0)
        return;

    sort_heads_by_stride(heads, head_count);
    for (i = 0; i < (uint32_t)head_count && *out_count < MAX_DETS; i++) {
        const float (*anchors)[2] = NULL;
        if (head_count == 3 && i < 3)
            anchors = anchors_p5[i];
        else if (head_count == 4 && i < 4)
            anchors = anchors_p6[i];
        else if (i < 3)
            anchors = anchors_p5[i];
        if (!anchors)
            continue;

        decode_yolo_head_output((const float *)outs[heads[i].out_idx].buf, &heads[i], anchors,
                                m->class_count, conf_thr, src_w, src_h,
                                (int)m->in_w, (int)m->in_h, out, out_count);
    }
}

struct tensor_cn_view {
    const float *buf;
    int c;
    int n;
    bool c_major;
};

static bool build_tensor_cn_view(const rknn_tensor_attr *a, const float *buf, struct tensor_cn_view *tv)
{
    int dims[4];
    int k = 0;
    int i;
    if (!a || !buf || !tv)
        return false;
    memset(tv, 0, sizeof(*tv));
    if (a->n_dims == 3) {
        int d1 = (int)a->dims[1];
        int d2 = (int)a->dims[2];
        if (d1 <= 0 || d2 <= 0)
            return false;
        if (d2 == OBB_POINT_COUNT) {
            tv->buf = buf;
            tv->c = d1;
            tv->n = d2;
            tv->c_major = true;
            return tv->c > 0;
        }
        if (d1 == OBB_POINT_COUNT) {
            tv->buf = buf;
            tv->c = d2;
            tv->n = d1;
            tv->c_major = false;
            return tv->c > 0;
        }
        return false;
    }

    for (i = 1; i < (int)a->n_dims && i < 4; i++) {
        int d = (int)a->dims[i];
        if (d > 1)
            dims[k++] = d;
    }
    if (k == 1 && dims[0] == OBB_POINT_COUNT) {
        tv->buf = buf;
        tv->c = 1;
        tv->n = OBB_POINT_COUNT;
        tv->c_major = true;
        return true;
    }
    if (k != 2)
        return false;
    if (dims[1] == OBB_POINT_COUNT) {
        tv->buf = buf;
        tv->c = dims[0];
        tv->n = dims[1];
        tv->c_major = true;
        return tv->c > 0;
    }
    if (dims[0] == OBB_POINT_COUNT) {
        tv->buf = buf;
        tv->c = dims[1];
        tv->n = dims[0];
        tv->c_major = false;
        return tv->c > 0;
    }
    return false;
}

static float tensor_cn_read(const struct tensor_cn_view *tv, int c, int n)
{
    if (tv->c_major)
        return tv->buf[(size_t)c * (size_t)tv->n + (size_t)n];
    return tv->buf[(size_t)n * (size_t)tv->c + (size_t)c];
}

static void tensor_sample_minmax(const struct tensor_cn_view *tv, float *min_v, float *max_v)
{
    int sample_n;
    int i;
    float mn = 1e30f;
    float mx = -1e30f;
    if (!tv || !min_v || !max_v || tv->c <= 0 || tv->n <= 0) {
        if (min_v) *min_v = 0.0f;
        if (max_v) *max_v = 0.0f;
        return;
    }
    sample_n = tv->n < 256 ? tv->n : 256;
    for (i = 0; i < sample_n; i++) {
        float v = tensor_cn_read(tv, 0, i);
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    *min_v = mn;
    *max_v = mx;
}

struct obb_anchor_cache {
    bool init;
    float x[OBB_POINT_COUNT];
    float y[OBB_POINT_COUNT];
    float stride[OBB_POINT_COUNT];
};

static bool detector_type_uses_quad(int mode)
{
    return mode == DETECTOR_YOLOV8_OBB_RKNN ||
           mode == DETECTOR_YOLOV8_POSE_RKNN;
}

static bool build_obb_anchor_cache(struct obb_anchor_cache *cache)
{
    static const int strides[3] = {8, 16, 32};
    int idx = 0;
    int si;
    if (!cache)
        return false;
    if (cache->init)
        return true;
    for (si = 0; si < 3; si++) {
        int s = strides[si];
        int gh = ALGO_STREAM_SIZE / s;
        int gw = ALGO_STREAM_SIZE / s;
        int gy;
        int gx;
        for (gy = 0; gy < gh; gy++) {
            for (gx = 0; gx < gw; gx++) {
                if (idx >= OBB_POINT_COUNT)
                    return false;
                cache->x[idx] = (float)gx + 0.5f;
                cache->y[idx] = (float)gy + 0.5f;
                cache->stride[idx] = (float)s;
                idx++;
            }
        }
    }
    if (idx != OBB_POINT_COUNT)
        return false;
    cache->init = true;
    return true;
}

static bool infer_obb_output_views(const struct yolo_model *m, const rknn_output *outs,
                                   struct tensor_cn_view *dist_view,
                                   struct tensor_cn_view *cls_view,
                                   struct tensor_cn_view *angle_view)
{
    int dist_idx = -1;
    int cls_idx = -1;
    int angle_idx = -1;
    int ones[4];
    int ones_count = 0;
    uint32_t i;

    /*
     * Prefer the exported middecode contract directly:
     *   out[0] = decoded ltrb distances
     *   out[1] = class logits
     *   out[2] = decoded angle scalar
     *
     * This avoids misclassifying single-class logits as angle when both tails
     * are 1x8400 tensors.
     */
    if (m->io_num.n_output >= 3) {
        struct tensor_cn_view tv0;
        struct tensor_cn_view tv1;
        struct tensor_cn_view tv2;
        if (build_tensor_cn_view(&m->output_attrs[0], (const float *)outs[0].buf, &tv0) &&
            build_tensor_cn_view(&m->output_attrs[1], (const float *)outs[1].buf, &tv1) &&
            build_tensor_cn_view(&m->output_attrs[2], (const float *)outs[2].buf, &tv2) &&
            tv0.c == 4 && tv0.n == OBB_POINT_COUNT &&
            tv1.c >= 1 && tv1.n == OBB_POINT_COUNT &&
            tv2.c == 1 && tv2.n == OBB_POINT_COUNT) {
            *dist_view = tv0;
            *cls_view = tv1;
            *angle_view = tv2;
            return true;
        }
    }

    for (i = 0; i < m->io_num.n_output; i++) {
        struct tensor_cn_view tv;
        if (!build_tensor_cn_view(&m->output_attrs[i], (const float *)outs[i].buf, &tv))
            continue;
        if (tv.n != OBB_POINT_COUNT)
            continue;
        if (tv.c == 4 && dist_idx < 0) {
            dist_idx = (int)i;
            continue;
        }
        if (tv.c > 1 && cls_idx < 0) {
            cls_idx = (int)i;
            continue;
        }
        if (tv.c == 1 && ones_count < 4)
            ones[ones_count++] = (int)i;
    }

    if (dist_idx < 0)
        return false;
    if (cls_idx < 0 && ones_count == 2) {
        struct tensor_cn_view t0;
        struct tensor_cn_view t1;
        float mn0, mx0, mn1, mx1;
        build_tensor_cn_view(&m->output_attrs[ones[0]], (const float *)outs[ones[0]].buf, &t0);
        build_tensor_cn_view(&m->output_attrs[ones[1]], (const float *)outs[ones[1]].buf, &t1);
        tensor_sample_minmax(&t0, &mn0, &mx0);
        tensor_sample_minmax(&t1, &mn1, &mx1);
        if (mn0 >= -1.2f && mx0 <= 3.2f) {
            angle_idx = ones[0];
            cls_idx = ones[1];
        } else if (mn1 >= -1.2f && mx1 <= 3.2f) {
            angle_idx = ones[1];
            cls_idx = ones[0];
        } else {
            angle_idx = ones[0];
            cls_idx = ones[1];
        }
    }
    if (angle_idx < 0 && ones_count > 0) {
        angle_idx = ones[0];
        if (cls_idx < 0 && ones_count > 1)
            cls_idx = ones[1];
    }
    if (cls_idx < 0 || angle_idx < 0)
        return false;

    if (!build_tensor_cn_view(&m->output_attrs[dist_idx], (const float *)outs[dist_idx].buf, dist_view))
        return false;
    if (!build_tensor_cn_view(&m->output_attrs[cls_idx], (const float *)outs[cls_idx].buf, cls_view))
        return false;
    if (!build_tensor_cn_view(&m->output_attrs[angle_idx], (const float *)outs[angle_idx].buf, angle_view))
        return false;
    return dist_view->c == 4 && angle_view->c == 1 && cls_view->c >= 1 &&
           dist_view->n == OBB_POINT_COUNT && cls_view->n == OBB_POINT_COUNT &&
           angle_view->n == OBB_POINT_COUNT;
}

static bool infer_pose_output_view(const struct yolo_model *m, const rknn_output *outs,
                                   struct tensor_cn_view *pose_view)
{
    uint32_t i;

    if (!m || !outs || !pose_view)
        return false;

    for (i = 0; i < m->io_num.n_output; i++) {
        struct tensor_cn_view tv;
        if (!build_tensor_cn_view(&m->output_attrs[i], (const float *)outs[i].buf, &tv))
            continue;
        if (tv.n == OBB_POINT_COUNT && tv.c == POSE_OUTPUT_CHANNELS) {
            *pose_view = tv;
            return true;
        }
    }
    return false;
}

static int decode_yolov8_obb_outputs(const struct yolo_model *m, const rknn_output *outs,
                                     float conf_thr, int src_w, int src_h,
                                     struct det_box *out, int *out_count)
{
    static struct obb_anchor_cache anchor_cache = {0};
    struct det_box cand[MAX_DETS * 4];
    struct tensor_cn_view dist_view;
    struct tensor_cn_view cls_view;
    struct tensor_cn_view angle_view;
    const int pre_nms_cap = MAX_DETS * 4;
    int cls_count;
    int cls_start = 0;
    int cls_end;
    int i;
    int count = 0;

    *out_count = 0;
    if (m->in_w != ALGO_STREAM_SIZE || m->in_h != ALGO_STREAM_SIZE)
        return -1;
    if (!build_obb_anchor_cache(&anchor_cache))
        return -1;
    if (!infer_obb_output_views(m, outs, &dist_view, &cls_view, &angle_view))
        return -1;

    cls_count = cls_view.c;
    if (m->class_count > 0 && m->class_count < cls_count)
        cls_count = m->class_count;
    cls_end = cls_count;
    if (m->class_filter >= 0 && m->class_filter < cls_count) {
        cls_start = m->class_filter;
        cls_end = cls_start + 1;
    }

    for (i = 0; i < OBB_POINT_COUNT; i++) {
        int c;
        int best_id = cls_start;
        float best = -1.0f;
        float l;
        float t;
        float r;
        float b;
        float angle;
        float off_x;
        float off_y;
        float cos_a;
        float sin_a;
        float stride;
        struct det_box det;

        for (c = cls_start; c < cls_end; c++) {
            float p = sigmoidf_local(tensor_cn_read(&cls_view, c, i));
            if (p > best) {
                best = p;
                best_id = c;
            }
        }
        if (best < conf_thr)
            continue;

        l = tensor_cn_read(&dist_view, 0, i);
        t = tensor_cn_read(&dist_view, 1, i);
        r = tensor_cn_read(&dist_view, 2, i);
        b = tensor_cn_read(&dist_view, 3, i);
        angle = tensor_cn_read(&angle_view, 0, i);
        if (!(l >= 0.0f && t >= 0.0f && r >= 0.0f && b >= 0.0f))
            continue;

        off_x = (r - l) * 0.5f;
        off_y = (b - t) * 0.5f;
        cos_a = cosf(angle);
        sin_a = sinf(angle);
        stride = anchor_cache.stride[i];

        memset(&det, 0, sizeof(det));
        det.cx = (anchor_cache.x[i] + off_x * cos_a - off_y * sin_a) * stride;
        det.cy = (anchor_cache.y[i] + off_x * sin_a + off_y * cos_a) * stride;
        det.w = (l + r) * stride;
        det.h = (t + b) * stride;
        det.angle = angle;
        det.conf = best;
        det.cls = best_id;
        if (det.w < 2.0f || det.h < 2.0f)
            continue;
        det_quad_from_obb(&det);
        clamp_box(&det, src_w, src_h);
        if (count < pre_nms_cap) {
            cand[count++] = det;
        } else {
            int k;
            int min_i = 0;
            float min_conf = cand[0].conf;
            for (k = 1; k < pre_nms_cap; k++) {
                if (cand[k].conf < min_conf) {
                    min_conf = cand[k].conf;
                    min_i = k;
                }
            }
            if (det.conf > min_conf)
                cand[min_i] = det;
        }
    }

    rotated_nms_inplace(cand, &count, m->nms_iou_thr, m->max_det);
    if (count > MAX_DETS)
        count = MAX_DETS;
    memcpy(out, cand, (size_t)count * sizeof(out[0]));
    *out_count = count;
    return 0;
}

static int decode_yolov8_pose_outputs(const struct yolo_model *m, const rknn_output *outs,
                                      float conf_thr, int src_w, int src_h,
                                      struct det_box *out, int *out_count)
{
    static struct obb_anchor_cache anchor_cache = {0};
    struct tensor_cn_view pose_view;
    struct det_box cand[MAX_DETS * 4];
    const int pre_nms_cap = MAX_DETS * 4;
    int count = 0;
    int i;

    *out_count = 0;
    if (m->in_w != ALGO_STREAM_SIZE || m->in_h != ALGO_STREAM_SIZE)
        return -1;
    if (m->class_filter > 0)
        return 0;
    if (!build_obb_anchor_cache(&anchor_cache))
        return -1;
    if (!infer_pose_output_view(m, outs, &pose_view))
        return -1;

    for (i = 0; i < OBB_POINT_COUNT; i++) {
        float anchor_x = anchor_cache.x[i];
        float anchor_y = anchor_cache.y[i];
        float stride = anchor_cache.stride[i];
        float l = tensor_cn_read(&pose_view, 0, i);
        float t = tensor_cn_read(&pose_view, 1, i);
        float r = tensor_cn_read(&pose_view, 2, i);
        float b = tensor_cn_read(&pose_view, 3, i);
        float score = sigmoidf_local(tensor_cn_read(&pose_view, 4, i));
        float ordered[8];
        struct det_box det;
        int k;
        bool valid = true;

        if (!(l >= 0.0f && t >= 0.0f && r >= 0.0f && b >= 0.0f))
            continue;
        if (score < conf_thr)
            continue;

        memset(&det, 0, sizeof(det));
        det.cx = (anchor_x + 0.5f * (r - l)) * stride;
        det.cy = (anchor_y + 0.5f * (b - t)) * stride;
        det.w = (l + r) * stride;
        det.h = (t + b) * stride;
        det.conf = score;
        det.cls = 0;
        det.has_obb = 1;
        if (det.w < 2.0f || det.h < 2.0f)
            continue;

        for (k = 0; k < POSE_KPT_COUNT; k++) {
            float kx = tensor_cn_read(&pose_view, 5 + k * POSE_KPT_DIMS + 0, i);
            float ky = tensor_cn_read(&pose_view, 5 + k * POSE_KPT_DIMS + 1, i);
            float kv = tensor_cn_read(&pose_view, 5 + k * POSE_KPT_DIMS + 2, i);
            if (!isfinite(kx) || !isfinite(ky) || !isfinite(kv)) {
                valid = false;
                break;
            }
            det.quad[k * 2 + 0] = (anchor_x + kx) * stride;
            det.quad[k * 2 + 1] = (anchor_y + ky) * stride;
        }
        if (!valid)
            continue;

        order_quad_points(det.quad, ordered);
        memcpy(det.quad, ordered, sizeof(ordered));
        if (quad_area8(det.quad) < 4.0f || !quad_is_convex8(det.quad))
            continue;

        bbox_from_quad_float(det.quad, src_w, src_h, &det.x1, &det.y1, &det.x2, &det.y2);
        clamp_box(&det, src_w, src_h);

        if (count < pre_nms_cap) {
            cand[count++] = det;
        } else {
            int min_i = 0;
            float min_conf = cand[0].conf;
            for (k = 1; k < pre_nms_cap; k++) {
                if (cand[k].conf < min_conf) {
                    min_conf = cand[k].conf;
                    min_i = k;
                }
            }
            if (det.conf > min_conf)
                cand[min_i] = det;
        }
    }

    nms_inplace(cand, &count, m->nms_iou_thr);
    if (m->max_det > 0 && count > m->max_det)
        count = m->max_det;
    if (count > MAX_DETS)
        count = MAX_DETS;
    memcpy(out, cand, (size_t)count * sizeof(out[0]));
    *out_count = count;
    return 0;
}

static int run_model_detect(struct yolo_model *m, const uint8_t *in_rgb, int src_w, int src_h,
                            float conf_thr, struct det_box *out, int *out_count,
                            struct detect_decode_diag *diag)
{
    rknn_input in;
    rknn_output outs[8];
    struct det_box rows_out[MAX_DETS];
    struct det_box heads_out[MAX_DETS];
    int rows_count = 0;
    int heads_count = 0;
    uint32_t i;
    int ret;

    *out_count = 0;
    if (diag)
        memset(diag, 0, sizeof(*diag));

    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf = (void *)in_rgb;
    in.size = m->in_w * m->in_h * 3;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    ret = rknn_inputs_set(m->ctx, 1, &in);
    if (ret < 0) return ret;
    ret = rknn_run(m->ctx, NULL);
    if (ret < 0) return ret;

    memset(outs, 0, sizeof(outs));
    for (i = 0; i < m->io_num.n_output; i++)
        outs[i].want_float = 1;
    ret = rknn_outputs_get(m->ctx, m->io_num.n_output, outs, NULL);
    if (ret < 0) return ret;

    if (m->detector_type == DETECTOR_YOLOV8_OBB_RKNN) {
        ret = decode_yolov8_obb_outputs(m, outs, conf_thr, src_w, src_h, out, out_count);
        if (ret < 0) {
            *out_count = 0;
            rknn_outputs_release(m->ctx, m->io_num.n_output, outs);
            return ret;
        }
        if (diag) {
            diag->rows_raw = 0;
            diag->heads_raw = *out_count;
            diag->rows_keep = 0;
            diag->heads_keep = *out_count;
            diag->mode = (*out_count > 0) ? PLATE_DECODE_OBB : PLATE_DECODE_NONE;
        }
        rknn_outputs_release(m->ctx, m->io_num.n_output, outs);
        return 0;
    } else if (m->detector_type == DETECTOR_YOLOV8_POSE_RKNN) {
        ret = decode_yolov8_pose_outputs(m, outs, conf_thr, src_w, src_h, out, out_count);
        if (ret < 0) {
            *out_count = 0;
            rknn_outputs_release(m->ctx, m->io_num.n_output, outs);
            return ret;
        }
        if (diag) {
            diag->rows_raw = 0;
            diag->heads_raw = *out_count;
            diag->rows_keep = 0;
            diag->heads_keep = *out_count;
            diag->mode = (*out_count > 0) ? PLATE_DECODE_OBB : PLATE_DECODE_NONE;
        }
        rknn_outputs_release(m->ctx, m->io_num.n_output, outs);
        return 0;
    }

    for (i = 0; i < m->io_num.n_output; i++) {
        const rknn_tensor_attr *a = &m->output_attrs[i];
        if (a->n_dims == 3 && rows_count == 0)
            decode_rows_tensor_output(a, (const float *)outs[i].buf, m->class_count,
                                      conf_thr, src_w, src_h, (int)m->in_w, (int)m->in_h,
                                      rows_out, &rows_count);
    }
    decode_yolo_heads_outputs(m, outs, conf_thr, src_w, src_h, heads_out, &heads_count);

    if (diag) {
        diag->rows_raw = rows_count;
        diag->heads_raw = heads_count;
        diag->rows_keep = 0;
        diag->heads_keep = 0;
        diag->mode = PLATE_DECODE_NONE;
    }

    if (rows_count > 0 && heads_count > 0) {
        int i2;
        int n = 0;
        for (i2 = 0; i2 < rows_count && n < MAX_DETS; i2++)
            out[n++] = rows_out[i2];
        for (i2 = 0; i2 < heads_count && n < MAX_DETS; i2++)
            out[n++] = heads_out[i2];
        *out_count = n;
        nms_inplace(out, out_count, m->nms_iou_thr);
        if (diag) {
            diag->mode = PLATE_DECODE_MERGED;
            diag->rows_keep = rows_count;
            diag->heads_keep = heads_count;
        }
    } else if (rows_count > 0) {
        memcpy(out, rows_out, (size_t)rows_count * sizeof(out[0]));
        *out_count = rows_count;
        nms_inplace(out, out_count, m->nms_iou_thr);
        if (diag) {
            diag->mode = PLATE_DECODE_ROWS;
            diag->rows_keep = *out_count;
        }
    } else if (heads_count > 0) {
        memcpy(out, heads_out, (size_t)heads_count * sizeof(out[0]));
        *out_count = heads_count;
        nms_inplace(out, out_count, m->nms_iou_thr);
        if (diag) {
            diag->mode = PLATE_DECODE_HEADS;
            diag->heads_keep = *out_count;
        }
    } else {
        *out_count = 0;
        if (diag) {
            diag->mode = PLATE_DECODE_NONE;
        }
    }

    if (*out_count > m->max_det)
        *out_count = m->max_det;
    if (*out_count > MAX_DETS)
        *out_count = MAX_DETS;

    rknn_outputs_release(m->ctx, m->io_num.n_output, outs);
    return 0;
}

static enum plate_color classify_plate_color_rgb(const uint8_t *rgb, int w, int h, const struct det_box *b)
{
    int x1 = b->x1 + (b->x2 - b->x1) / 6;
    int x2 = b->x2 - (b->x2 - b->x1) / 6;
    int y1 = b->y1 + (b->y2 - b->y1) / 6;
    int y2 = b->y2 - (b->y2 - b->y1) / 6;
    int x, y;
    int total = 0, blue_cnt = 0, green_cnt = 0, yellow_cnt = 0;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 >= w) x2 = w - 1;
    if (y2 >= h) y2 = h - 1;
    for (y = y1; y <= y2; y++) {
        for (x = x1; x <= x2; x++) {
            const uint8_t *p = rgb + (y * w + x) * 3;
            float r = p[0] / 255.0f;
            float g = p[1] / 255.0f;
            float bch = p[2] / 255.0f;
            float mx = fmaxf(r, fmaxf(g, bch));
            float mn = fminf(r, fminf(g, bch));
            float d = mx - mn;
            float h_deg = 0.0f;
            float s = (mx == 0.0f) ? 0.0f : (d / mx);
            float v = mx;
            if (d > 1e-6f) {
                if (mx == r) h_deg = 60.0f * fmodf((g - bch) / d, 6.0f);
                else if (mx == g) h_deg = 60.0f * (((bch - r) / d) + 2.0f);
                else h_deg = 60.0f * (((r - g) / d) + 4.0f);
            }
            if (h_deg < 0.0f) h_deg += 360.0f;
            total++;
            if (h_deg >= 90.0f && h_deg <= 130.0f && s > 0.23f && v > 0.16f) blue_cnt++;
            else if (h_deg >= 35.0f && h_deg <= 90.0f && s > 0.20f && v > 0.16f) green_cnt++;
            else if (h_deg >= 15.0f && h_deg <= 55.0f && s > 0.20f && v > 0.16f) yellow_cnt++;
        }
    }
    if (total == 0) return PLATE_COLOR_UNKNOWN;
    if ((float)blue_cnt / (float)total >= 0.20f && blue_cnt > green_cnt + (int)(0.05f * total))
        return PLATE_COLOR_BLUE;
    if ((float)green_cnt / (float)total >= 0.20f && green_cnt > blue_cnt + (int)(0.05f * total))
        return PLATE_COLOR_GREEN;
    if ((float)yellow_cnt / (float)total >= 0.18f)
        return PLATE_COLOR_YELLOW;
    return PLATE_COLOR_UNKNOWN;
}

static int find_parent_car(const struct det_box *plate, const struct det_box *cars, int car_count)
{
    int i;
    int cx = (plate->x1 + plate->x2) / 2;
    int cy = (plate->y1 + plate->y2) / 2;
    int best = -1;
    float best_ratio = 0.0f;
    for (i = 0; i < car_count; i++) {
        const struct det_box *c = &cars[i];
        int x1 = plate->x1 > c->x1 ? plate->x1 : c->x1;
        int y1 = plate->y1 > c->y1 ? plate->y1 : c->y1;
        int x2 = plate->x2 < c->x2 ? plate->x2 : c->x2;
        int y2 = plate->y2 < c->y2 ? plate->y2 : c->y2;
        bool center_inside = (cx >= c->x1 && cx <= c->x2 && cy >= c->y1 && cy <= c->y2);
        float ratio = 0.0f;
        if (x2 >= x1 && y2 >= y1) {
            int ia = (x2 - x1 + 1) * (y2 - y1 + 1);
            int pa = (plate->x2 - plate->x1 + 1) * (plate->y2 - plate->y1 + 1);
            ratio = pa > 0 ? (float)ia / (float)pa : 0.0f;
        }
        if (center_inside)
            return i;
        if (ratio > 0.70f && ratio > best_ratio) {
            best_ratio = ratio;
            best = i;
        }
    }
    return best;
}

static const char *plate_color_str(enum plate_color c)
{
    if (c == PLATE_COLOR_BLUE) return "BLUE";
    if (c == PLATE_COLOR_GREEN) return "GREEN";
    if (c == PLATE_COLOR_YELLOW) return "YELLOW";
    return "UNK";
}

static enum plate_type classify_plate_type(enum plate_color color, const char *text)
{
    const char *utf8_police = "\xE8\xAD\xA6";
    const char *utf8_trailer = "\xE6\x8C\x82";
    const char *utf8_embassy = "\xE4\xBD\xBF";
    const char *utf8_consulate = "\xE9\xA2\x86";

    if (text && strstr(text, utf8_police))
        return PLATE_TYPE_POLICE;
    if (text && strstr(text, utf8_trailer))
        return PLATE_TYPE_TRAILER;
    if (text && (strstr(text, utf8_embassy) || strstr(text, utf8_consulate)))
        return PLATE_TYPE_EMBASSY_CONSULATE;
    if (text && strncmp(text, "WJ", 2) == 0)
        return PLATE_TYPE_POLICE;

    if (color == PLATE_COLOR_GREEN)
        return PLATE_TYPE_COMMON_GREEN;
    if (color == PLATE_COLOR_BLUE)
        return PLATE_TYPE_COMMON_BLUE;
    if (color == PLATE_COLOR_YELLOW)
        return PLATE_TYPE_YELLOW;
    return PLATE_TYPE_UNKNOWN;
}

static const char *plate_type_str(enum plate_type t)
{
    switch (t) {
    case PLATE_TYPE_COMMON_BLUE: return "common_blue";
    case PLATE_TYPE_COMMON_GREEN: return "common_green";
    case PLATE_TYPE_YELLOW: return "yellow";
    case PLATE_TYPE_POLICE: return "police";
    case PLATE_TYPE_TRAILER: return "trailer";
    case PLATE_TYPE_EMBASSY_CONSULATE: return "embassy_consulate";
    default: return "unknown";
    }
}

static const char *plate_decode_mode_str(int mode)
{
    if (mode == PLATE_DECODE_ROWS) return "rows";
    if (mode == PLATE_DECODE_HEADS) return "heads";
    if (mode == PLATE_DECODE_MERGED) return "merged";
    if (mode == PLATE_DECODE_OBB) return "obb";
    return "none";
}

static const char *detector_type_str(int mode)
{
    if (mode == DETECTOR_YOLOV8_OBB_RKNN)
        return "yolov8_obb_rknn";
    if (mode == DETECTOR_YOLOV8_POSE_RKNN)
        return "yolov8_pose_rknn";
    return "yolov5";
}

static const char *ocr_channel_order_str(int mode)
{
    return (mode == OCR_CH_BGR) ? "bgr" : "rgb";
}

static const char *ocr_crop_mode_str(int mode)
{
    if (mode == OCR_CROP_BOX) return "box";
    if (mode == OCR_CROP_TIGHT) return "tight";
    if (mode == OCR_CROP_BOX_PAD) return "box-pad";
    if (mode == OCR_CROP_MATCH) return "match";
    if (mode == OCR_CROP_OBB_WARP) return "obb_warp";
    if (mode == OCR_CROP_OBB_PIECEWISE) return "obb_piecewise";
    return "fixed";
}

static const char *ocr_resize_mode_str(int mode)
{
    return (mode == OCR_RESIZE_LETTERBOX) ? "letterbox" : "stretch";
}

static const char *ocr_resize_kernel_str(int mode)
{
    return (mode == OCR_KERNEL_BILINEAR) ? "bilinear" : "nn";
}

static const char *ocr_preproc_mode_str(int mode)
{
    if (mode == OCR_PREPROC_GRAY) return "gray";
    if (mode == OCR_PREPROC_BIN) return "bin";
    return "none";
}

static const char *det_resize_mode_str(int mode)
{
    return (mode == DET_RESIZE_LETTERBOX) ? "letterbox" : "stretch";
}

static bool plate_box_pass_rules(const struct det_box *b, int frame_w, int frame_h)
{
    int bw = b->x2 - b->x1 + 1;
    int bh = b->y2 - b->y1 + 1;
    int cy = (b->y1 + b->y2) / 2;
    float aspect;
    float area;
    float min_area = (float)frame_w * (float)frame_h * 0.0016f;

    if (bw <= 0 || bh <= 0)
        return false;
    if (bw < 56 || bh < 18)
        return false;
    aspect = (float)bw / (float)bh;
    if (aspect < 2.4f || aspect > 5.8f)
        return false;
    area = (float)bw * (float)bh;
    if (area < min_area)
        return false;
    if (cy < (int)(0.12f * (float)frame_h) || cy > (int)(0.95f * (float)frame_h))
        return false;
    return true;
}

static bool plate_box_pass_rules_relaxed(const struct det_box *b, int frame_w, int frame_h)
{
    int bw = b->x2 - b->x1 + 1;
    int bh = b->y2 - b->y1 + 1;
    int cy = (b->y1 + b->y2) / 2;
    float aspect;
    float area;
    float min_area = (float)frame_w * (float)frame_h * 0.00025f;

    if (bw <= 0 || bh <= 0)
        return false;
    if (bw < 20 || bh < 8)
        return false;
    aspect = (float)bw / (float)bh;
    if (aspect < 1.8f || aspect > 8.5f)
        return false;
    area = (float)bw * (float)bh;
    if (area < min_area)
        return false;
    if (cy < (int)(0.02f * (float)frame_h) || cy > (int)(0.98f * (float)frame_h))
        return false;
    return true;
}

static float plate_abs_tilt_deg(const struct det_box *b)
{
    float deg;
    if (!b || !b->has_obb)
        return 0.0f;
    deg = fabsf(b->angle) * (180.0f / (float)M_PI);
    while (deg > 180.0f)
        deg -= 180.0f;
    if (deg > 90.0f)
        deg = 180.0f - deg;
    return deg;
}

static bool plate_box_pass_rules_obb(const struct det_box *b, int frame_w, int frame_h)
{
    int bw;
    int bh;
    int cy;
    float aspect;
    float area;
    float min_area = (float)frame_w * (float)frame_h * 0.00018f;
    float tilt;

    if (!b)
        return false;
    bw = b->x2 - b->x1 + 1;
    bh = b->y2 - b->y1 + 1;
    cy = (b->y1 + b->y2) / 2;
    if (bw <= 0 || bh <= 0)
        return false;
    if (bw < 16 || bh < 6)
        return false;
    aspect = (float)bw / (float)bh;
    if (aspect < 1.2f || aspect > 13.5f)
        return false;
    area = (float)bw * (float)bh;
    if (area < min_area)
        return false;
    if (cy < (int)(0.01f * (float)frame_h) || cy > (int)(0.99f * (float)frame_h))
        return false;
    if (b->has_obb) {
        tilt = plate_abs_tilt_deg(b);
        if (tilt > 82.0f)
            return false;
    }
    return true;
}

static bool plate_box_pass_rules_for_detector(int detector_type, const struct det_box *b,
                                              int frame_w, int frame_h)
{
    if (plate_box_pass_rules(b, frame_w, frame_h))
        return true;
    if (detector_type_uses_quad(detector_type)) {
        if (plate_box_pass_rules_obb(b, frame_w, frame_h))
            return true;
        if (plate_box_pass_rules_relaxed(b, frame_w, frame_h))
            return true;
    }
    return false;
}

static bool has_iou_match(const struct det_box *cur, const struct det_box *hist, int hist_count, float iou_thr)
{
    int i;
    for (i = 0; i < hist_count; i++) {
        if (box_iou(cur, &hist[i]) >= iou_thr)
            return true;
    }
    return false;
}

static void temporal_confirm_and_update(struct app_ctx *ctx,
                                        const struct det_box *filtered, int filtered_count,
                                        struct det_box *confirmed, int *confirmed_count)
{
    int i;
    float iou_thr_hist1 = 0.35f;
    float iou_thr_hist2 = 0.30f;
    float direct_keep_thr = 0.55f;
    *confirmed_count = 0;
    if (detector_type_uses_quad(ctx->opt.plate_detector_type)) {
        iou_thr_hist1 = 0.30f;
        iou_thr_hist2 = 0.22f;
        direct_keep_thr = fmaxf(0.55f, ctx->opt.min_plate_conf + 0.08f);
    }
    if (ctx->plate_hist1_count > 0 && ctx->plate_hist2_count > 0) {
        for (i = 0; i < filtered_count && *confirmed_count < MAX_DETS; i++) {
            if (has_iou_match(&filtered[i], ctx->plate_hist1, ctx->plate_hist1_count, iou_thr_hist1) &&
                has_iou_match(&filtered[i], ctx->plate_hist2, ctx->plate_hist2_count, iou_thr_hist2)) {
                confirmed[(*confirmed_count)++] = filtered[i];
            }
        }
    }
    if (*confirmed_count == 0 &&
        detector_type_uses_quad(ctx->opt.plate_detector_type)) {
        for (i = 0; i < filtered_count && *confirmed_count < MAX_DETS; i++) {
            if (filtered[i].conf >= direct_keep_thr)
                confirmed[(*confirmed_count)++] = filtered[i];
        }
    }
    memcpy(ctx->plate_hist2, ctx->plate_hist1, sizeof(ctx->plate_hist1));
    ctx->plate_hist2_count = ctx->plate_hist1_count;
    memcpy(ctx->plate_hist1, filtered, (size_t)filtered_count * sizeof(filtered[0]));
    ctx->plate_hist1_count = filtered_count;
}

static void csv_safe_text(const char *in, char *out, size_t out_len)
{
    size_t i = 0;
    if (out_len == 0)
        return;
    if (!in) {
        out[0] = '\0';
        return;
    }
    while (*in && i + 1 < out_len) {
        char ch = *in++;
        if (ch == ',' || ch == '\n' || ch == '\r')
            ch = '_';
        out[i++] = ch;
    }
    out[i] = '\0';
}

static int mkdir_p_simple(const char *path)
{
    char buf[512];
    size_t n;
    size_t i;

    if (!path || !path[0])
        return -1;
    n = strnlen(path, sizeof(buf) - 1);
    if (n == 0 || n >= sizeof(buf))
        return -1;
    memcpy(buf, path, n);
    buf[n] = '\0';

    for (i = 1; i < n; i++) {
        if (buf[i] == '/' || buf[i] == '\\') {
            char ch = buf[i];
            buf[i] = '\0';
            if (buf[0] != '\0') {
                if (mkdir(buf, 0777) < 0 && errno != EEXIST)
                    return -1;
            }
            buf[i] = ch;
        }
    }
    if (mkdir(buf, 0777) < 0 && errno != EEXIST)
        return -1;
    return 0;
}

static int write_ppm_rgb888(const char *path, const uint8_t *rgb, int w, int h)
{
    FILE *fp;
    size_t need;
    size_t wr;
    if (!path || !rgb || w <= 0 || h <= 0)
        return -1;
    fp = fopen(path, "wb");
    if (!fp)
        return -1;
    fprintf(fp, "P6\n%d %d\n255\n", w, h);
    need = (size_t)w * h * 3U;
    wr = fwrite(rgb, 1, need, fp);
    fclose(fp);
    return (wr == need) ? 0 : -1;
}

static int ppm_next_token(FILE *fp, char *tok, size_t tok_sz)
{
    int ch;
    size_t n = 0;

    if (!fp || !tok || tok_sz == 0)
        return -1;
    tok[0] = '\0';

    while (1) {
        ch = fgetc(fp);
        if (ch == '#') {
            while (ch != '\n' && ch != EOF)
                ch = fgetc(fp);
            continue;
        }
        if (ch == EOF)
            return -1;
        if (!isspace((unsigned char)ch))
            break;
    }

    while (ch != EOF && !isspace((unsigned char)ch) && ch != '#') {
        if (n + 1 < tok_sz)
            tok[n++] = (char)ch;
        ch = fgetc(fp);
    }
    tok[n] = '\0';

    if (ch == '#') {
        while (ch != '\n' && ch != EOF)
            ch = fgetc(fp);
    }
    return (n > 0) ? 0 : -1;
}

static int read_ppm_rgb888(const char *path, uint8_t **rgb_out, int *w_out, int *h_out)
{
    FILE *fp = NULL;
    char tok[64];
    int w, h, maxv;
    int ch;
    uint8_t *buf = NULL;
    size_t need;
    size_t got;

    if (!path || !rgb_out || !w_out || !h_out)
        return -1;
    *rgb_out = NULL;
    *w_out = 0;
    *h_out = 0;

    fp = fopen(path, "rb");
    if (!fp)
        return -1;

    if (ppm_next_token(fp, tok, sizeof(tok)) < 0 || strcmp(tok, "P6") != 0)
        goto fail;
    if (ppm_next_token(fp, tok, sizeof(tok)) < 0)
        goto fail;
    w = atoi(tok);
    if (ppm_next_token(fp, tok, sizeof(tok)) < 0)
        goto fail;
    h = atoi(tok);
    if (ppm_next_token(fp, tok, sizeof(tok)) < 0)
        goto fail;
    maxv = atoi(tok);
    if (w <= 0 || h <= 0 || maxv != 255)
        goto fail;
    do {
        ch = fgetc(fp);
    } while (ch != EOF && isspace((unsigned char)ch));
    if (ch == EOF)
        goto fail;
    if (ungetc(ch, fp) == EOF)
        goto fail;

    need = (size_t)w * (size_t)h * 3U;
    buf = (uint8_t *)malloc(need);
    if (!buf)
        goto fail;
    got = fread(buf, 1, need, fp);
    if (got != need)
        goto fail;

    fclose(fp);
    *rgb_out = buf;
    *w_out = w;
    *h_out = h;
    return 0;

fail:
    if (fp)
        fclose(fp);
    free(buf);
    return -1;
}

static int parse_roi_arg(const char *arg, struct det_box *b)
{
    int x1, y1, x2, y2;
    if (!arg || !b)
        return -1;
    if (sscanf(arg, "%d,%d,%d,%d", &x1, &y1, &x2, &y2) != 4)
        return -1;
    if (x2 < x1 || y2 < y1)
        return -1;
    b->x1 = x1;
    b->y1 = y1;
    b->x2 = x2;
    b->y2 = y2;
    b->conf = 1.0f;
    b->cls = 0;
    b->has_obb = 0;
    b->cx = 0.0f;
    b->cy = 0.0f;
    b->w = 0.0f;
    b->h = 0.0f;
    b->angle = 0.0f;
    memset(b->quad, 0, sizeof(b->quad));
    return 0;
}

static int run_offline_once(struct app_ctx *ctx)
{
    uint8_t *rgb = NULL;
    uint8_t *rgb_detect = NULL;
    uint8_t *algo_rgb = NULL;
    uint8_t *plate_in = NULL;
    uint8_t *plate_crop = NULL;
    uint8_t *ocr_input_dump = NULL;
    struct det_box dets[MAX_DETS];
    struct detect_decode_diag plate_diag;
    struct plate_det pd;
    struct ocr_diag odiag;
    struct det_box box;
    struct det_box valid[MAX_DETS];
    int det_count = 0;
    int valid_count = 0;
    int i;
    int w = 0;
    int h = 0;
    int crop_w;
    int crop_h;
    int ret = -1;
    int64_t ts_us;
    const uint8_t *det_src_rgb = NULL;
    float occ_ratio = 0.0f;
    bool used_obb_warp = false;

    if (read_ppm_rgb888(ctx->opt.offline_image_path, &rgb, &w, &h) < 0) {
        fprintf(stderr, "Offline image load failed (need PPM P6): %s\n",
                ctx->opt.offline_image_path ? ctx->opt.offline_image_path : "<null>");
        return -1;
    }
    ctx->frame_width = (uint32_t)w;
    ctx->frame_height = (uint32_t)h;
    det_src_rgb = rgb;
    plate_crop = malloc((size_t)w * h * 3U);
    if (!plate_crop)
        goto out;
    if (ctx->opt.sw_preproc) {
        rgb_detect = malloc((size_t)w * h * 3U);
        if (!rgb_detect)
            goto out;
        memcpy(rgb_detect, rgb, (size_t)w * h * 3U);
        sw_preprocess_rgb888(rgb_detect, w, h);
        det_src_rgb = rgb_detect;
    }

    memset(&pd, 0, sizeof(pd));
    memset(&odiag, 0, sizeof(odiag));
    memset(&plate_diag, 0, sizeof(plate_diag));
    memset(&box, 0, sizeof(box));

    if (ctx->opt.offline_roi_arg && ctx->opt.offline_roi_arg[0] != '\0') {
        if (parse_roi_arg(ctx->opt.offline_roi_arg, &box) < 0) {
            fprintf(stderr, "Invalid --offline-roi, expect x1,y1,x2,y2\n");
            goto out;
        }
        clamp_box(&box, w, h);
    } else if (ctx->opt.offline_detect_plate) {
        algo_rgb = malloc((size_t)ALGO_STREAM_SIZE * ALGO_STREAM_SIZE * 3U);
        plate_in = malloc((size_t)ctx->plate_model.in_w * ctx->plate_model.in_h * 3U);
        if (!algo_rgb || !plate_in)
            goto out;

        if (run_detect_on_rgb(ctx, &ctx->plate_model, det_src_rgb, w, h,
                              ctx->opt.min_plate_conf, algo_rgb, plate_in,
                              dets, &det_count, &plate_diag) < 0) {
            fprintf(stderr, "Offline plate detect failed\n");
            goto out;
        }
        if (det_count <= 0) {
            fprintf(stderr, "Offline plate detect empty\n");
            goto out;
        }
        for (i = 0; i < det_count && valid_count < MAX_DETS; i++) {
            if (plate_box_pass_rules_for_detector(ctx->opt.plate_detector_type, &dets[i], w, h))
                valid[valid_count++] = dets[i];
        }
        if (valid_count <= 0) {
            int relaxed_count = 0;
            for (i = 0; i < det_count && relaxed_count < MAX_DETS; i++) {
                if (plate_box_pass_rules_relaxed(&dets[i], w, h))
                    valid[relaxed_count++] = dets[i];
            }
            valid_count = relaxed_count;
            if (valid_count > 0) {
                fprintf(stderr, "Offline plate detect fallback(relaxed=%d raw=%d)\n", valid_count, det_count);
            } else if (ctx->opt.det_resize_mode == DET_RESIZE_LETTERBOX) {
                int saved_mode = ctx->opt.det_resize_mode;
                fprintf(stderr, "Offline plate detect retry with stretch(raw=%d)\n", det_count);
                ctx->opt.det_resize_mode = DET_RESIZE_STRETCH;
                if (run_detect_on_rgb(ctx, &ctx->plate_model, det_src_rgb, w, h,
                                      ctx->opt.min_plate_conf, algo_rgb, plate_in,
                                      dets, &det_count, &plate_diag) < 0) {
                    det_count = 0;
                }
                ctx->opt.det_resize_mode = saved_mode;
                for (i = 0; i < det_count && valid_count < MAX_DETS; i++) {
                    if (plate_box_pass_rules_for_detector(ctx->opt.plate_detector_type, &dets[i], w, h))
                        valid[valid_count++] = dets[i];
                }
                if (valid_count > 0)
                    fprintf(stderr, "Offline plate detect fallback(stretch valid=%d raw=%d)\n", valid_count, det_count);
            }
        }
        if (valid_count <= 0) {
            fprintf(stderr, "Offline plate detect empty(valid=0 raw=%d)\n", det_count);
            goto out;
        }
        box = valid[0];
        for (i = 1; i < valid_count; i++) {
            if (valid[i].conf > box.conf)
                box = valid[i];
        }
        if (ctx->opt.plate_refine) {
            struct det_box refined = box;
            if (refine_plate_box_local(ctx, det_src_rgb, w, h, &box, algo_rgb, plate_in, &refined) &&
                plate_box_pass_rules_for_detector(ctx->opt.plate_detector_type, &refined, w, h))
                box = refined;
        }
    } else {
        fprintf(stderr, "Need --offline-roi when --offline-detect-plate=0\n");
        goto out;
    }

    pd.box = box;
    pd.color = classify_plate_color_rgb(rgb, w, h, &pd.box);
    if (!prepare_plate_crop_rgb888(ctx, rgb, w, h, &pd.box,
                                   plate_crop, w, h, &pd.crop_box, &crop_w, &crop_h,
                                   &occ_ratio, &used_obb_warp, 0))
        goto out;
    if (ctx->opt.ocr_min_occ_ratio > 0.0f &&
        occ_ratio < ctx->opt.ocr_min_occ_ratio &&
        ctx->opt.ocr_crop_mode != OCR_CROP_TIGHT &&
        !used_obb_warp) {
        struct det_box recrop_box;
        float old_occ = occ_ratio;
        int new_w, new_h;
        const char *mode_tag = "tight";
        bool have_recap = false;
        bool is_match_mode = (ctx->opt.ocr_crop_mode == OCR_CROP_MATCH);
        if (ctx->opt.ocr_crop_mode == OCR_CROP_MATCH) {
            have_recap = compute_match_ytrim_crop(ctx, &pd.crop_box, ctx->opt.ocr_min_occ_ratio, &recrop_box);
            mode_tag = "match-ytrim";
        }
        if (!have_recap && !is_match_mode) {
            compute_expand_crop_box(&pd.box, (int)ctx->frame_width, (int)ctx->frame_height, 0.08f, 0.16f, &recrop_box);
            have_recap = true;
        }
        if (!have_recap) {
            fprintf(stderr, "[ocr-recrop] trigger=0 old_occ=%.3f mode=match-ytrim reason=not-improvable\n", old_occ);
        } else {
            new_w = recrop_box.x2 - recrop_box.x1 + 1;
            new_h = recrop_box.y2 - recrop_box.y1 + 1;
            if (new_w > 0 && new_h > 0 && new_w <= w && new_h <= h) {
                copy_crop_rgb888(rgb, w, &recrop_box, plate_crop);
                pd.crop_box = recrop_box;
                crop_w = new_w;
                crop_h = new_h;
                occ_ratio = estimate_ocr_occ_ratio(ctx, crop_w, crop_h);
                fprintf(stderr, "[ocr-recrop] trigger=1 old_occ=%.3f new_mode=%s new_occ=%.3f\n",
                        old_occ, mode_tag, occ_ratio);
            }
        }
    }
    pd.ocr_in_occ_ratio = occ_ratio;
    fprintf(stderr, "[crop-geom] box=[%d,%d,%d,%d] crop=[%d,%d,%d,%d] iou=%.3f\n",
            pd.box.x1, pd.box.y1, pd.box.x2, pd.box.y2,
            pd.crop_box.x1, pd.crop_box.y1, pd.crop_box.x2, pd.crop_box.y2,
            box_iou(&pd.box, &pd.crop_box));

    if ((pd.box.y2 - pd.box.y1 + 1) < ctx->opt.ocr_min_plate_h) {
        pd.ocr_text[0] = '\0';
        pd.ocr_conf = 0.0f;
        pd.ocr_blank_top1 = 0.0f;
        fprintf(stderr, "[offline][skip] reason=size plate_h=%d min_h=%d\n",
                pd.box.y2 - pd.box.y1 + 1, ctx->opt.ocr_min_plate_h);
    } else {
        float sharp = laplacian_variance_rgb888(plate_crop, crop_w, crop_h);
        if (sharp < ctx->opt.ocr_min_sharpness) {
            pd.ocr_text[0] = '\0';
            pd.ocr_conf = 0.0f;
            pd.ocr_blank_top1 = 0.0f;
            fprintf(stderr, "[offline][skip] reason=blur sharp=%.2f min=%.2f\n",
                    sharp, ctx->opt.ocr_min_sharpness);
        } else {
            if (ctx->ocr_crop_index_fp && ctx->ocr_crop_dumped < ctx->opt.ocr_crop_dump_max)
                ret = run_model_ocr(ctx, plate_crop, crop_w, crop_h, pd.color,
                                    pd.ocr_text, sizeof(pd.ocr_text), &pd.ocr_conf,
                                    &odiag, &ocr_input_dump);
            else
                ret = run_model_ocr(ctx, plate_crop, crop_w, crop_h, pd.color,
                                    pd.ocr_text, sizeof(pd.ocr_text), &pd.ocr_conf,
                                    &odiag, NULL);
            if (ret < 0) {
                fprintf(stderr, "Offline OCR failed\n");
                goto out;
            }
            pd.ocr_blank_top1 = odiag.blank_top1_ratio;
            pd.ocr_in_occ_ratio = odiag.in_occ_ratio;
        }
    }
    pd.type = classify_plate_type(pd.color, pd.ocr_text);

    fprintf(stderr,
            "[offline] image=%s size=%dx%d box=[%d,%d,%d,%d] crop=[%d,%d,%d,%d]\n",
            ctx->opt.offline_image_path, w, h,
            pd.box.x1, pd.box.y1, pd.box.x2, pd.box.y2,
            pd.crop_box.x1, pd.crop_box.y1, pd.crop_box.x2, pd.crop_box.y2);
    if (ctx->opt.ocr_ctc_diag) {
        fprintf(stderr,
                "[offline][ctc] t=%d c=%d blank=%d blank_top1=%.3f\n",
                odiag.t_size, odiag.c_size, odiag.blank_idx, odiag.blank_top1_ratio);
    }
    fprintf(stderr,
            "[offline][pred] text=%s conf=%.4f type=%s color=%s\n",
            pd.ocr_text, pd.ocr_conf, plate_type_str(pd.type), plate_color_str(pd.color));

    ts_us = mono_us();
    log_prediction_row(ctx, 0, ts_us, &pd);
    if (!ocr_input_dump && ctx->ocr_crop_index_fp &&
        ctx->ocr_crop_dumped < ctx->opt.ocr_crop_dump_max) {
        ocr_input_dump = prepare_ocr_input_rgb888(ctx, plate_crop, crop_w, crop_h, NULL);
    }
    if (ocr_input_dump) {
        dump_ocr_pair(ctx, 0, &pd, plate_crop, crop_w, crop_h,
                      ocr_input_dump, (int)ctx->ocr_model.in_w, (int)ctx->ocr_model.in_h);
        free(ocr_input_dump);
        ocr_input_dump = NULL;
    }
    ret = 0;

out:
    free(ocr_input_dump);
    free(plate_crop);
    free(plate_in);
    free(algo_rgb);
    free(rgb_detect);
    free(rgb);
    return ret;
}

static void dump_ocr_pair(struct app_ctx *ctx, uint64_t frame_id, const struct plate_det *pd,
                          const uint8_t *crop_rgb, int crop_w, int crop_h,
                          const uint8_t *ocr_in, int ocr_w, int ocr_h)
{
    char crop_path[640];
    char in_path[640];
    char safe_text[64];
    int idx;
    int64_t ts;

    if (!ctx->opt.ocr_crop_dump_dir || ctx->opt.ocr_crop_dump_dir[0] == '\0')
        return;
    if (ctx->opt.ocr_crop_dump_max <= 0)
        return;
    if (ctx->ocr_crop_dumped >= ctx->opt.ocr_crop_dump_max)
        return;
    if (!ctx->ocr_crop_index_fp)
        return;

    idx = ctx->ocr_crop_dumped;
    snprintf(crop_path, sizeof(crop_path), "%s/crop_%04d_f%06" PRIu64 ".ppm",
             ctx->opt.ocr_crop_dump_dir, idx, frame_id);
    snprintf(in_path, sizeof(in_path), "%s/ocrin_%04d_f%06" PRIu64 ".ppm",
             ctx->opt.ocr_crop_dump_dir, idx, frame_id);

    if (write_ppm_rgb888(crop_path, crop_rgb, crop_w, crop_h) < 0)
        return;
    if (ocr_in && ocr_w > 0 && ocr_h > 0) {
        if (write_ppm_rgb888(in_path, ocr_in, ocr_w, ocr_h) < 0)
            return;
    } else {
        in_path[0] = '\0';
    }

    ts = mono_us();
    csv_safe_text(pd->ocr_text, safe_text, sizeof(safe_text));
    fprintf(ctx->ocr_crop_index_fp,
            "%d,%" PRIu64 ",%" PRId64 ",%d,%d,%d,%d,%d,%d,%d,%d,%s,%.4f,%.4f,%.4f,%s,%s\n",
            idx, frame_id, ts,
            pd->box.x1, pd->box.y1, pd->box.x2, pd->box.y2,
            pd->crop_box.x1, pd->crop_box.y1, pd->crop_box.x2, pd->crop_box.y2,
            safe_text, pd->ocr_conf, pd->ocr_blank_top1, pd->ocr_in_occ_ratio, crop_path, in_path);
    fflush(ctx->ocr_crop_index_fp);
    ctx->ocr_crop_dumped++;
}

static void dump_ocr_ab_variant(const struct app_ctx *ctx, uint64_t frame_id, int sample_id,
                                const char *tag, const float quad[8],
                                const uint8_t *crop_rgb, int crop_w, int crop_h)
{
    char crop_path[640];
    char meta_path[640];
    FILE *fp;
    const char *dir;
    int i;

    if (!ctx || !ctx->opt.ocr_crop_dump_dir || ctx->opt.ocr_crop_dump_dir[0] == '\0' ||
        !tag || !quad || !crop_rgb || crop_w <= 0 || crop_h <= 0)
        return;

    dir = ctx->opt.ocr_crop_dump_dir;
    snprintf(crop_path, sizeof(crop_path), "%s/%s_%04d_f%06" PRIu64 ".ppm",
             dir, tag, sample_id, frame_id);
    snprintf(meta_path, sizeof(meta_path), "%s/%s_%04d_f%06" PRIu64 ".quad.txt",
             dir, tag, sample_id, frame_id);

    if (write_ppm_rgb888(crop_path, crop_rgb, crop_w, crop_h) < 0)
        return;

    fp = fopen(meta_path, "w");
    if (!fp)
        return;
    fprintf(fp, "tag=%s\nframe_id=%" PRIu64 "\nsample_id=%d\ncrop_w=%d\ncrop_h=%d\nquad=",
            tag, frame_id, sample_id, crop_w, crop_h);
    for (i = 0; i < 4; i++) {
        fprintf(fp, "%s%.3f,%.3f", (i == 0) ? "" : ";", quad[i * 2 + 0], quad[i * 2 + 1]);
    }
    fprintf(fp, "\n");
    fclose(fp);
}

static void log_prediction_row(struct app_ctx *ctx, uint64_t frame_id, int64_t ts_us,
                               const struct plate_det *pd)
{
    char safe_text[64];
    if (!ctx->pred_log_fp)
        return;
    csv_safe_text(pd->ocr_text, safe_text, sizeof(safe_text));
    pthread_mutex_lock(&ctx->pred_log_lock);
    fprintf(ctx->pred_log_fp,
            "%" PRIu64 ",%s,%s,%.4f,%d,%d,%d,%d,%" PRId64 "\n",
            frame_id,
            safe_text,
            plate_type_str(pd->type),
            pd->ocr_conf,
            pd->box.x1, pd->box.y1, pd->box.x2, pd->box.y2,
            ts_us);
    fflush(ctx->pred_log_fp);
    pthread_mutex_unlock(&ctx->pred_log_lock);
}

static void build_overlay_ascii_text(const struct plate_det *pd, char *out, size_t out_len)
{
    size_t i = 0;
    char toks[MAX_PLATE_TOKENS][MAX_UTF8_TOKEN_BYTES];
    int tok_n = 0;
    int t;

    if (out_len == 0)
        return;
    out[0] = '\0';

    tok_n = split_utf8_tokens(pd->ocr_text, toks, MAX_PLATE_TOKENS);
    for (t = 0; t < tok_n && i + 1 < out_len; t++) {
        const char *tok = toks[t];
        const char *alias = province_token_ascii(tok);
        if (tok[0] == '\0')
            continue;

        if (tok[1] == '\0') {
            unsigned char ch = (unsigned char)tok[0];
            if (ch == '.' || ch == '-' || ch == '_') {
                out[i++] = '-';
                continue;
            }
            if (ch >= 'a' && ch <= 'z')
                ch = (unsigned char)(ch - 'a' + 'A');
            if ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z')) {
                out[i++] = (char)ch;
                continue;
            }
        }
        if (alias && alias[0] != '\0') {
            int j;
            for (j = 0; alias[j] != '\0' && i + 1 < out_len; j++)
                out[i++] = alias[j];
        }
    }

    if (i == 0) {
        const char *fb = "UNK";
        if (pd->type == PLATE_TYPE_COMMON_BLUE) fb = "BLUE";
        else if (pd->type == PLATE_TYPE_COMMON_GREEN) fb = "GREEN";
        else if (pd->type == PLATE_TYPE_YELLOW) fb = "YELLOW";
        else if (pd->type == PLATE_TYPE_POLICE) fb = "POLICE";
        else if (pd->type == PLATE_TYPE_TRAILER) fb = "TRAILER";
        else if (pd->type == PLATE_TYPE_EMBASSY_CONSULATE) fb = "EMB";
        while (*fb && i + 1 < out_len)
            out[i++] = *fb++;
        if (i == 0) {
            out[0] = 'U';
            if (out_len > 1) out[1] = 'N';
            if (out_len > 2) out[2] = 'K';
            i = (out_len > 2) ? 3 : out_len - 1;
        }
    }
    out[i] = '\0';
}

static void *infer_thread_main(void *arg)
{
    struct app_ctx *ctx = (struct app_ctx *)arg;
    uint8_t *raw_local = malloc(ctx->src_frame_size);
    uint8_t *rgb_full = malloc((size_t)ctx->frame_width * ctx->frame_height * 3U);
    uint8_t *rgb_detect = malloc((size_t)ctx->frame_width * ctx->frame_height * 3U);
    uint8_t *a_map = malloc((size_t)ctx->frame_width * ctx->frame_height);
    uint8_t *algo_rgb = malloc((size_t)ALGO_STREAM_SIZE * ALGO_STREAM_SIZE * 3U);
    uint8_t *veh_in = malloc((size_t)ctx->veh_model.in_w * ctx->veh_model.in_h * 3U);
    uint8_t *plate_in = malloc((size_t)ctx->plate_model.in_w * ctx->plate_model.in_h * 3U);
    uint8_t *plate_crop = malloc((size_t)ctx->frame_width * ctx->frame_height * 3U);
    if (!raw_local || !rgb_full || !rgb_detect || !a_map || !algo_rgb || !veh_in || !plate_in || !plate_crop) {
        free(raw_local); free(rgb_full); free(rgb_detect); free(a_map); free(algo_rgb);
        free(veh_in); free(plate_in); free(plate_crop);
        return NULL;
    }

    while (ctx->running) {
        struct det_box cars[MAX_DETS];
        struct det_box raw_plates[MAX_DETS];
        struct det_box filtered_plates[MAX_DETS];
        struct det_box roi_plates[MAX_DETS];
        struct det_box stable_plates[MAX_DETS];
        struct det_box persons[MAX_DETS];
        struct det_box tracked_persons[MAX_DETS];
        struct lpr_results r;
        int car_count = 0;
        int raw_plate_count = 0;
        int filtered_plate_count = 0;
        int stable_plate_count = 0;
        int person_count = 0;
        int tracked_person_count = 0;
        int ped_events = 0;
        int ocr_run_count = 0;
        int ocr_skip_size = 0;
        int ocr_skip_blur = 0;
        int ocr_nonempty_count = 0;
        int overlay_nonempty_count = 0;
        struct detect_decode_diag plate_diag;
        bool light_red = false;
        float red_ratio = 0.0f;
        struct det_box a_roi = {0};
        bool a_roi_valid = false;
        int i;
        int64_t t0, t1;
        uint64_t seq;
        const uint8_t *det_src_rgb = rgb_full;

        pthread_mutex_lock(&ctx->infer_lock);
        while (ctx->running && !ctx->infer_has_new)
            pthread_cond_wait(&ctx->infer_cond, &ctx->infer_lock);
        if (!ctx->running) {
            pthread_mutex_unlock(&ctx->infer_lock);
            break;
        }
        memcpy(raw_local, ctx->infer_latest_raw, ctx->src_frame_size);
        seq = ctx->infer_frame_seq;
        ctx->infer_has_new = false;
        pthread_mutex_unlock(&ctx->infer_lock);

        age_ocr_tracks(ctx, seq);

        t0 = mono_us();
        if (ctx->src_is_bgrx) {
            bgrx8888_to_rgb888_and_a(raw_local, (int)ctx->frame_width, (int)ctx->frame_height, rgb_full, a_map);
        } else {
            raw565_to_rgb888_full(ctx, raw_local, rgb_full);
            memset(a_map, 0, (size_t)ctx->frame_width * ctx->frame_height);
        }
        if (ctx->opt.sw_preproc) {
            memcpy(rgb_detect, rgb_full, (size_t)ctx->frame_width * ctx->frame_height * 3U);
            sw_preprocess_rgb888(rgb_detect, (int)ctx->frame_width, (int)ctx->frame_height);
            det_src_rgb = rgb_detect;
        }

        if (ctx->opt.fpga_a_mask && ctx->src_is_bgrx) {
            a_roi_valid = extract_a_channel_roi(a_map, (int)ctx->frame_width, (int)ctx->frame_height,
                                                ctx->opt.a_proj_ratio, &a_roi, &red_ratio) ? true : false;
            if (red_ratio >= ctx->opt.red_ratio_thr)
                ctx->ped_red_streak++;
            else
                ctx->ped_red_streak = 0;
            light_red = (ctx->ped_red_streak >= ctx->opt.red_stable_frames);
        } else {
            ctx->ped_red_streak = 0;
            light_red = false;
        }

        memset(&plate_diag, 0, sizeof(plate_diag));

        if (!ctx->opt.plate_only || ctx->opt.ped_event) {
            if (run_detect_on_rgb(ctx, &ctx->veh_model, det_src_rgb, (int)ctx->frame_width, (int)ctx->frame_height,
                                  ctx->opt.min_car_conf, algo_rgb, veh_in, cars, &car_count, NULL) < 0)
                car_count = 0;
        }
        {
            float plate_thr = ctx->opt.min_plate_conf;
            if (ctx->opt.fpga_a_mask && a_roi_valid)
                plate_thr = fmaxf(0.05f, plate_thr - 0.05f);
            if (run_detect_on_rgb(ctx, &ctx->plate_model, det_src_rgb, (int)ctx->frame_width, (int)ctx->frame_height,
                                  plate_thr, algo_rgb, plate_in, raw_plates, &raw_plate_count, &plate_diag) < 0)
                raw_plate_count = 0;
            if (raw_plate_count <= 0 &&
                detector_type_uses_quad(ctx->opt.plate_detector_type) &&
                ctx->opt.det_resize_mode == DET_RESIZE_LETTERBOX) {
                int saved_mode = ctx->opt.det_resize_mode;
                fprintf(stderr,
                        "[plate-fallback] frame=%" PRIu64 " retry=stretch reason=raw_empty\n",
                        seq);
                ctx->opt.det_resize_mode = DET_RESIZE_STRETCH;
                if (run_detect_on_rgb(ctx, &ctx->plate_model, det_src_rgb, (int)ctx->frame_width, (int)ctx->frame_height,
                                      plate_thr, algo_rgb, plate_in, raw_plates, &raw_plate_count, &plate_diag) < 0) {
                    raw_plate_count = 0;
                }
                ctx->opt.det_resize_mode = saved_mode;
            }
        }
        if (raw_plate_count > 0) {
            ctx->gate_plate_raw_positive_frames++;
            ctx->gate_plate_raw_positive_streak++;
        } else {
            ctx->gate_plate_raw_positive_streak = 0;
        }

        for (i = 0; i < car_count && person_count < MAX_DETS; i++) {
            if (cars[i].cls == ctx->person_class_id)
                persons[person_count++] = cars[i];
        }
        if (ctx->opt.ped_event)
            ped_events = update_ped_tracks_nn(ctx, persons, person_count, light_red, seq,
                                              tracked_persons, &tracked_person_count);

        for (i = 0; i < raw_plate_count && filtered_plate_count < MAX_DETS; i++) {
            bool keep = plate_box_pass_rules_for_detector(ctx->opt.plate_detector_type,
                                                          &raw_plates[i],
                                                          (int)ctx->frame_width,
                                                          (int)ctx->frame_height);
            if (keep)
                filtered_plates[filtered_plate_count++] = raw_plates[i];
        }
        if (filtered_plate_count == 0 &&
            raw_plate_count > 0 &&
            detector_type_uses_quad(ctx->opt.plate_detector_type)) {
            int best_i = 0;
            float best_conf = raw_plates[0].conf;
            for (i = 1; i < raw_plate_count; i++) {
                if (raw_plates[i].conf > best_conf) {
                    best_conf = raw_plates[i].conf;
                    best_i = i;
                }
            }
            if (best_conf >= fmaxf(0.50f, ctx->opt.min_plate_conf)) {
                filtered_plates[filtered_plate_count++] = raw_plates[best_i];
                fprintf(stderr,
                        "[plate-fallback] frame=%" PRIu64 " keep=top1 conf=%.3f reason=filtered_empty\n",
                        seq, best_conf);
            }
        }
        if (ctx->opt.fpga_a_mask && a_roi_valid) {
            int roi_count = filter_boxes_by_roi(filtered_plates, filtered_plate_count, &a_roi,
                                                ctx->opt.a_roi_iou_min, roi_plates, MAX_DETS);
            if (roi_count > 0) {
                memcpy(filtered_plates, roi_plates, (size_t)roi_count * sizeof(roi_plates[0]));
                filtered_plate_count = roi_count;
            }
        }

        temporal_confirm_and_update(ctx, filtered_plates, filtered_plate_count,
                                    stable_plates, &stable_plate_count);
        t1 = mono_us();

        memset(&r, 0, sizeof(r));
        r.car_raw_count = car_count;
        r.person_raw_count = person_count;
        r.plate_raw_count = raw_plate_count;
        r.plate_rows_raw = plate_diag.rows_raw;
        r.plate_heads_raw = plate_diag.heads_raw;
        r.plate_rows_keep = plate_diag.rows_keep;
        r.plate_heads_keep = plate_diag.heads_keep;
        r.plate_decode_mode = plate_diag.mode;
        r.ocr_run_count = 0;
        r.ocr_skip_size = 0;
        r.ocr_skip_blur = 0;
        r.ocr_nonempty_count = 0;
        r.overlay_text_nonempty_count = 0;
        r.a_roi_valid = a_roi_valid ? 1 : 0;
        r.a_roi = a_roi;
        r.light_red = light_red ? 1 : 0;
        for (i = 0; i < car_count && r.car_count < MAX_DETS; i++) {
            if (cars[i].cls == ctx->car_class_id)
                r.cars[r.car_count++] = cars[i];
        }
        if (!ctx->opt.ped_event) {
            for (i = 0; i < person_count && r.person_count < MAX_DETS; i++)
                r.persons[r.person_count++] = persons[i];
        } else {
            for (i = 0; i < tracked_person_count && r.person_count < MAX_DETS; i++)
                r.persons[r.person_count++] = tracked_persons[i];
        }

        for (i = 0; i < stable_plate_count && r.plate_count < MAX_DETS; i++) {
            struct plate_det pd;
            struct ocr_diag odiag;
            uint8_t *ocr_input_dump = NULL;
            uint8_t **ocr_input_out = NULL;
            int parent = -1;
            int crop_w;
            int crop_h;
            int plate_h;
            float sharpness = 0.0f;
            float occ_ratio = 0.0f;
            bool used_obb_warp = false;
            char overlay_txt[32];
            pd.box = stable_plates[i];
            if (ctx->opt.plate_refine) {
                struct det_box refined = pd.box;
                if (refine_plate_box_local(ctx, det_src_rgb, (int)ctx->frame_width, (int)ctx->frame_height,
                                           &pd.box, algo_rgb, plate_in, &refined))
                    pd.box = refined;
            }
            if (!ctx->opt.plate_only)
                parent = find_parent_car(&pd.box, r.cars, r.car_count);
            if (!ctx->opt.plate_only && ctx->opt.plate_on_car_only && parent < 0)
                continue;
            pd.parent_car = parent;
            pd.color = classify_plate_color_rgb(rgb_full, (int)ctx->frame_width, (int)ctx->frame_height, &pd.box);
            if (!prepare_plate_crop_rgb888(ctx, rgb_full, (int)ctx->frame_width, (int)ctx->frame_height,
                                           &pd.box, plate_crop, (int)ctx->frame_width, (int)ctx->frame_height,
                                           &pd.crop_box, &crop_w, &crop_h, &occ_ratio, &used_obb_warp, seq))
                continue;
            if (ctx->opt.ocr_min_occ_ratio > 0.0f &&
                occ_ratio < ctx->opt.ocr_min_occ_ratio &&
                ctx->opt.ocr_crop_mode != OCR_CROP_TIGHT &&
                !used_obb_warp) {
                struct det_box recrop_box;
                float old_occ = occ_ratio;
                int new_w, new_h;
                const char *mode_tag = "tight";
                bool have_recap = false;
                bool is_match_mode = (ctx->opt.ocr_crop_mode == OCR_CROP_MATCH);
                if (ctx->opt.ocr_crop_mode == OCR_CROP_MATCH) {
                    have_recap = compute_match_ytrim_crop(ctx, &pd.crop_box, ctx->opt.ocr_min_occ_ratio, &recrop_box);
                    mode_tag = "match-ytrim";
                }
                if (!have_recap && !is_match_mode) {
                    compute_expand_crop_box(&pd.box, (int)ctx->frame_width, (int)ctx->frame_height, 0.08f, 0.16f, &recrop_box);
                    have_recap = true;
                }
                if (!have_recap) {
                    fprintf(stderr,
                            "[ocr-recrop] frame=%" PRIu64 " trigger=0 old_occ=%.3f mode=match-ytrim reason=not-improvable\n",
                            seq, old_occ);
                } else {
                    new_w = recrop_box.x2 - recrop_box.x1 + 1;
                    new_h = recrop_box.y2 - recrop_box.y1 + 1;
                    if (new_w > 0 && new_h > 0 &&
                        new_w <= (int)ctx->frame_width && new_h <= (int)ctx->frame_height) {
                        copy_crop_rgb888(rgb_full, (int)ctx->frame_width, &recrop_box, plate_crop);
                        pd.crop_box = recrop_box;
                        crop_w = new_w;
                        crop_h = new_h;
                        occ_ratio = estimate_ocr_occ_ratio(ctx, crop_w, crop_h);
                        fprintf(stderr,
                                "[ocr-recrop] frame=%" PRIu64 " trigger=1 old_occ=%.3f new_mode=%s new_occ=%.3f\n",
                                seq, old_occ, mode_tag, occ_ratio);
                    }
                }
            }
            pd.ocr_in_occ_ratio = occ_ratio;
            fprintf(stderr,
                    "[crop-geom] frame=%" PRIu64 " box=[%d,%d,%d,%d] crop=[%d,%d,%d,%d] iou=%.3f\n",
                    seq,
                    pd.box.x1, pd.box.y1, pd.box.x2, pd.box.y2,
                    pd.crop_box.x1, pd.crop_box.y1, pd.crop_box.x2, pd.crop_box.y2,
                    box_iou(&pd.box, &pd.crop_box));
            plate_h = pd.box.y2 - pd.box.y1 + 1;
            if (ctx->ocr_crop_index_fp &&
                ctx->ocr_crop_dumped < ctx->opt.ocr_crop_dump_max)
                ocr_input_out = &ocr_input_dump;
            if (plate_h < ctx->opt.ocr_min_plate_h) {
                pd.ocr_text[0] = '\0';
                pd.ocr_conf = 0.0f;
                pd.ocr_blank_top1 = 0.0f;
                memset(&odiag, 0, sizeof(odiag));
                ocr_skip_size++;
                fprintf(stderr,
                        "[ocr-skip] frame=%" PRIu64 " reason=size plate_h=%d min_h=%d bbox=[%d,%d,%d,%d]\n",
                        seq, plate_h, ctx->opt.ocr_min_plate_h,
                        pd.box.x1, pd.box.y1, pd.box.x2, pd.box.y2);
            } else {
                sharpness = laplacian_variance_rgb888(plate_crop, crop_w, crop_h);
                if (sharpness < ctx->opt.ocr_min_sharpness) {
                    pd.ocr_text[0] = '\0';
                    pd.ocr_conf = 0.0f;
                    pd.ocr_blank_top1 = 0.0f;
                    memset(&odiag, 0, sizeof(odiag));
                    ocr_skip_blur++;
                    fprintf(stderr,
                            "[ocr-skip] frame=%" PRIu64 " reason=blur sharp=%.2f min=%.2f bbox=[%d,%d,%d,%d]\n",
                            seq, sharpness, ctx->opt.ocr_min_sharpness,
                            pd.box.x1, pd.box.y1, pd.box.x2, pd.box.y2);
                } else {
                    if (run_model_ocr(ctx, plate_crop, crop_w, crop_h, pd.color,
                                      pd.ocr_text, sizeof(pd.ocr_text), &pd.ocr_conf,
                                      &odiag, ocr_input_out) < 0) {
                        snprintf(pd.ocr_text, sizeof(pd.ocr_text), "UNK");
                        pd.ocr_conf = 0.0f;
                        pd.ocr_blank_top1 = 0.0f;
                        memset(&odiag, 0, sizeof(odiag));
                    } else {
                        pd.ocr_blank_top1 = odiag.blank_top1_ratio;
                        pd.ocr_in_occ_ratio = odiag.in_occ_ratio;
                        ocr_run_count++;
                    }
                }
            }
            if (ctx->opt.ocr_ctc_diag) {
                fprintf(stderr,
                        "[ctc] frame=%" PRIu64 " bbox=[%d,%d,%d,%d] t=%d c=%d blank=%d blank_top1=%.3f text=%s\n",
                        seq,
                        pd.box.x1, pd.box.y1, pd.box.x2, pd.box.y2,
                        odiag.t_size, odiag.c_size, odiag.blank_idx, odiag.blank_top1_ratio,
                        pd.ocr_text);
            }
            if (pd.ocr_text[0] != '\0') {
                ocr_temporal_smooth(ctx, &pd.box, seq, pd.ocr_text, sizeof(pd.ocr_text), &pd.ocr_conf);
            }
            if (pd.ocr_text[0] != '\0')
                ocr_nonempty_count++;
            pd.type = classify_plate_type(pd.color, pd.ocr_text);
            build_overlay_ascii_text(&pd, overlay_txt, sizeof(overlay_txt));
            if (overlay_txt[0] != '\0')
                overlay_nonempty_count++;
            if (!ocr_input_dump && ctx->ocr_crop_index_fp &&
                ctx->ocr_crop_dumped < ctx->opt.ocr_crop_dump_max) {
                ocr_input_dump = prepare_ocr_input_rgb888(ctx, plate_crop, crop_w, crop_h, NULL);
            }
            if (ocr_input_dump) {
                dump_ocr_pair(ctx, seq, &pd, plate_crop, crop_w, crop_h,
                              ocr_input_dump, (int)ctx->ocr_model.in_w, (int)ctx->ocr_model.in_h);
                free(ocr_input_dump);
                ocr_input_dump = NULL;
            }
            fprintf(stderr,
                    "[pred] frame=%" PRIu64 " ts_us=%" PRId64 " bbox=[%d,%d,%d,%d] text=%s conf=%.2f type=%s color=%s\n",
                    seq,
                    mono_us(),
                    pd.box.x1, pd.box.y1, pd.box.x2, pd.box.y2,
                    pd.ocr_text,
                    pd.ocr_conf,
                    plate_type_str(pd.type),
                    plate_color_str(pd.color));
            log_prediction_row(ctx, seq, mono_us(), &pd);
            ctx->pred_rows_total++;
            r.plates[r.plate_count++] = pd;
        }
        r.ocr_run_count = ocr_run_count;
        r.ocr_skip_size = ocr_skip_size;
        r.ocr_skip_blur = ocr_skip_blur;
        r.ocr_nonempty_count = ocr_nonempty_count;
        r.overlay_text_nonempty_count = overlay_nonempty_count;
        r.frame_seq = seq;
        r.infer_ms_last = (double)(t1 - t0) / 1000.0;
        pthread_mutex_lock(&ctx->result_lock);
        r.infer_frames_total = ctx->results.infer_frames_total + 1;
        r.infer_ms_total = ctx->results.infer_ms_total + r.infer_ms_last;
        r.ped_event_total = ctx->results.ped_event_total + (uint64_t)ped_events;
        r.ped_event_last_frame = (ped_events > 0) ? seq : ctx->results.ped_event_last_frame;
        ctx->results = r;
        pthread_mutex_unlock(&ctx->result_lock);
    }

    free(raw_local); free(rgb_full); free(rgb_detect); free(a_map); free(algo_rgb);
    free(veh_in); free(plate_in); free(plate_crop);
    return NULL;
}

static void overlay_results_on_slot(struct app_ctx *ctx, uint8_t *slot_data)
{
    struct lpr_results r;
    uint16_t *pix = (uint16_t *)slot_data;
    int i;
    int stopline_y = (int)((float)ctx->frame_height * ctx->opt.stopline_ratio);
    pthread_mutex_lock(&ctx->result_lock);
    r = ctx->results;
    pthread_mutex_unlock(&ctx->result_lock);

    for (i = 0; i < r.car_count; i++)
        draw_rect_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, &r.cars[i], COLOR_YELLOW_565);
    for (i = 0; i < r.person_count; i++)
        draw_rect_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, &r.persons[i], COLOR_GREEN_565);

    for (i = 0; i < r.plate_count; i++) {
        char txt[32];
        int tx = r.plates[i].box.x1;
        int text_scale = OVERLAY_TEXT_SCALE;
        int text_h = 7 * text_scale;
        int ty = r.plates[i].box.y1 - (text_h + 3);
        if (ty < 0)
            ty = r.plates[i].box.y1 + 3;
        if (ty + text_h >= (int)ctx->frame_height)
            ty = (int)ctx->frame_height - text_h - 1;
        if (ty < 0)
            ty = 0;
        build_overlay_ascii_text(&r.plates[i], txt, sizeof(txt));
        draw_rect_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, &r.plates[i].box, COLOR_CYAN_565);
        if (ctx->opt.show_crop_box)
            draw_rect_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, &r.plates[i].crop_box, COLOR_RED_565);
        draw_text_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, tx, ty, txt, COLOR_CYAN_565, text_scale);
    }

    if (ctx->opt.fpga_a_mask && r.a_roi_valid)
        draw_rect_565(pix, (int)ctx->frame_width, (int)ctx->frame_height, &r.a_roi, COLOR_GREEN_565);
    if (ctx->opt.ped_event) {
        draw_hline_565(pix, (int)ctx->frame_width, (int)ctx->frame_height,
                       0, (int)ctx->frame_width - 1, stopline_y,
                       r.light_red ? COLOR_RED_565 : COLOR_GREEN_565);
    }
}

static void push_latest_to_infer(struct app_ctx *ctx, const uint8_t *raw)
{
    pthread_mutex_lock(&ctx->infer_lock);
    if (ctx->infer_has_new)
        ctx->infer_overwrite_count++;
    memcpy(ctx->infer_latest_raw, raw, ctx->src_frame_size);
    ctx->infer_frame_seq++;
    ctx->infer_has_new = true;
    pthread_cond_signal(&ctx->infer_cond);
    pthread_mutex_unlock(&ctx->infer_lock);
}

static void print_stats(struct app_ctx *ctx)
{
    int64_t now = mono_us();
    int64_t dt = now - ctx->last_stats_us;
    struct lpr_results r;
    const char *decode_mode;
    if (dt < (int64_t)ctx->opt.stats_interval * 1000000LL)
        return;
    pthread_mutex_lock(&ctx->result_lock);
    r = ctx->results;
    pthread_mutex_unlock(&ctx->result_lock);
    decode_mode = plate_decode_mode_str(r.plate_decode_mode);
    fprintf(stderr,
            "[stats] cap=%" PRIu64 " push=%" PRIu64 " rel=%" PRIu64
            " infer=%" PRIu64 " infer_ms=%.2f cars=%d(raw=%d) persons=%d(raw=%d)"
            " plates=%d(raw=%d) rows=%d/%d heads=%d/%d mode=%s ocr=%d run=%d skip_sz=%d skip_blur=%d ovtxt=%d aroi=%d red=%d ped_evt=%" PRIu64
            " gate_raw_pos=%" PRIu64 " gate_streak=%" PRIu64 " pred_rows=%" PRIu64 " drop=%" PRIu64
            " cap_fps=%.2f disp_fps=%.2f infer_fps=%.2f\n",
            ctx->captured_frames, ctx->pushed_frames, ctx->released_frames,
            r.infer_frames_total, r.infer_ms_last,
            r.car_count, r.car_raw_count,
            r.person_count, r.person_raw_count,
            r.plate_count, r.plate_raw_count,
            r.plate_rows_raw, r.plate_rows_keep,
            r.plate_heads_raw, r.plate_heads_keep, decode_mode,
            r.ocr_nonempty_count, r.ocr_run_count, r.ocr_skip_size, r.ocr_skip_blur, r.overlay_text_nonempty_count,
            r.a_roi_valid, r.light_red, r.ped_event_total,
            ctx->gate_plate_raw_positive_frames, ctx->gate_plate_raw_positive_streak, ctx->pred_rows_total,
            ctx->infer_overwrite_count,
            (double)(ctx->captured_frames - ctx->last_stats_cap) * 1000000.0 / (double)dt,
            (double)(ctx->released_frames - ctx->last_stats_rel) * 1000000.0 / (double)dt,
            (double)(r.infer_frames_total - ctx->last_stats_infer) * 1000000.0 / (double)dt);
    ctx->last_stats_cap = ctx->captured_frames;
    ctx->last_stats_rel = ctx->released_frames;
    ctx->last_stats_infer = r.infer_frames_total;
    ctx->last_stats_us = now;
}

static void cleanup(struct app_ctx *ctx)
{
    int i;
    ctx->running = false;
    pthread_mutex_lock(&ctx->infer_lock);
    pthread_cond_broadcast(&ctx->infer_cond);
    pthread_mutex_unlock(&ctx->infer_lock);
    if (ctx->infer_thread)
        pthread_join(ctx->infer_thread, NULL);

    rknn_model_release(&ctx->veh_model);
    rknn_model_release(&ctx->plate_model);
    rknn_ocr_model_release(&ctx->ocr_model);
    rknn_quad_refiner_model_release(&ctx->quad_refiner_model);

    if (ctx->pred_log_fp) {
        fclose(ctx->pred_log_fp);
        ctx->pred_log_fp = NULL;
    }
    if (ctx->ocr_crop_index_fp) {
        fclose(ctx->ocr_crop_index_fp);
        ctx->ocr_crop_index_fp = NULL;
    }

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
    free(ctx->dma_copy);
    free(ctx->infer_latest_raw);

    if (ctx->slots) {
        for (i = 0; i < ctx->slot_count; i++)
            free(ctx->slots[i].data);
        free(ctx->slots);
    }

    if (ctx->dev_fd >= 0)
        close(ctx->dev_fd);
    if (ctx->drm_fd >= 0)
        close(ctx->drm_fd);

    pthread_mutex_destroy(&ctx->infer_lock);
    pthread_cond_destroy(&ctx->infer_cond);
    pthread_mutex_destroy(&ctx->result_lock);
    pthread_mutex_destroy(&ctx->pred_log_lock);
    g_cond_clear(&ctx->slots_cond);
    g_mutex_clear(&ctx->slots_lock);
}

int main(int argc, char **argv)
{
    struct app_ctx ctx;
    bool offline_mode;
    int ret = 1;
    memset(&ctx, 0, sizeof(ctx));
    ctx.dev_fd = -1;
    ctx.drm_fd = -1;
    ctx.running = true;

    g_mutex_init(&ctx.slots_lock);
    g_cond_init(&ctx.slots_cond);
    pthread_mutex_init(&ctx.infer_lock, NULL);
    pthread_cond_init(&ctx.infer_cond, NULL);
    pthread_mutex_init(&ctx.result_lock, NULL);
    pthread_mutex_init(&ctx.pred_log_lock, NULL);

    if (parse_options(argc, argv, &ctx.opt) < 0) {
        print_usage(argv[0]);
        goto out;
    }
    if (detector_type_uses_quad(ctx.opt.plate_detector_type) &&
        ctx.opt.ocr_crop_mode != OCR_CROP_OBB_WARP &&
        ctx.opt.ocr_crop_mode != OCR_CROP_OBB_PIECEWISE) {
        fprintf(stderr,
                "[cfg] detector=%s requires ocr-crop-mode=obb_warp|obb_piecewise, force switching\n",
                detector_type_str(ctx.opt.plate_detector_type));
        ctx.opt.ocr_crop_mode = OCR_CROP_OBB_WARP;
    }
    if (detector_type_uses_quad(ctx.opt.plate_detector_type) &&
        ctx.opt.ocr_channel_order != OCR_CH_BGR) {
        fprintf(stderr,
                "[cfg] detector=%s keeps OCR contract, force ocr-channel-order=bgr\n",
                detector_type_str(ctx.opt.plate_detector_type));
        ctx.opt.ocr_channel_order = OCR_CH_BGR;
    }
    if (detector_type_uses_quad(ctx.opt.plate_detector_type) &&
        ctx.opt.ocr_resize_mode != OCR_RESIZE_LETTERBOX) {
        fprintf(stderr,
                "[cfg] detector=%s keeps OCR contract, force ocr-resize-mode=letterbox\n",
                detector_type_str(ctx.opt.plate_detector_type));
        ctx.opt.ocr_resize_mode = OCR_RESIZE_LETTERBOX;
    }
    if (detector_type_uses_quad(ctx.opt.plate_detector_type) &&
        ctx.opt.ocr_resize_kernel != OCR_KERNEL_NN) {
        fprintf(stderr,
                "[cfg] detector=%s keeps OCR contract, force ocr-resize-kernel=nn\n",
                detector_type_str(ctx.opt.plate_detector_type));
        ctx.opt.ocr_resize_kernel = OCR_KERNEL_NN;
    }
    offline_mode = (ctx.opt.offline_image_path && ctx.opt.offline_image_path[0] != '\0');

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    if (!offline_mode)
        gst_init(&argc, &argv);

    if (!offline_mode) {
        ctx.drm_fd = open(ctx.opt.drm_card_path, O_RDWR | O_CLOEXEC);
        if (ctx.drm_fd < 0)
            goto out;
    }
    if (!offline_mode && load_labels(&ctx, ctx.opt.labels_path) < 0)
        goto out;
    if (load_ocr_keys(&ctx, ctx.opt.ocr_keys_path) < 0)
        goto out;
    if (!offline_mode && init_fpga_dma(&ctx) < 0)
        goto out;
    if (!offline_mode && init_copy_slots(&ctx) < 0)
        goto out;
    if (!offline_mode &&
        rknn_model_load(&ctx.veh_model, "vehicle", ctx.opt.veh_model_path,
                        ctx.label_count, DETECTOR_YOLOV5) < 0)
        goto out;
    if (rknn_model_load(&ctx.plate_model, "plate", ctx.opt.plate_model_path,
                        (ctx.opt.plate_detector_type == DETECTOR_YOLOV8_OBB_RKNN) ? 0 : 1,
                        ctx.opt.plate_detector_type) < 0)
        goto out;
    ctx.plate_model.nms_iou_thr = ctx.opt.plate_nms_iou;
    ctx.plate_model.max_det = ctx.opt.plate_max_det;
    ctx.plate_model.class_filter = ctx.opt.plate_class_id;
    if (rknn_ocr_model_load(&ctx.ocr_model, "ocr", ctx.opt.ocr_model_path) < 0)
        goto out;
    if (rknn_quad_refiner_model_load(&ctx.quad_refiner_model, "quad_refiner", ctx.opt.quad_refiner_model_path) < 0)
        goto out;

    if (ctx.opt.pred_log_path && ctx.opt.pred_log_path[0] != '\0') {
        ctx.pred_log_fp = fopen(ctx.opt.pred_log_path, "w");
        if (!ctx.pred_log_fp)
            goto out;
        fprintf(ctx.pred_log_fp, "frame_id,plate_text_pred,plate_type_pred,conf,x1,y1,x2,y2,ts_us\n");
        fflush(ctx.pred_log_fp);
    }
    if (ctx.opt.ocr_crop_dump_dir && ctx.opt.ocr_crop_dump_dir[0] != '\0') {
        char idx_path[640];
        if (mkdir_p_simple(ctx.opt.ocr_crop_dump_dir) < 0)
            goto out;
        snprintf(idx_path, sizeof(idx_path), "%s/index.csv", ctx.opt.ocr_crop_dump_dir);
        ctx.ocr_crop_index_fp = fopen(idx_path, "w");
        if (!ctx.ocr_crop_index_fp)
            goto out;
        fprintf(ctx.ocr_crop_index_fp,
                "sample_id,frame_id,ts_us,box_x1,box_y1,box_x2,box_y2,"
                "crop_x1,crop_y1,crop_x2,crop_y2,app_text,app_conf,app_blank_top1,app_occ_ratio,"
                "crop_path,ocr_input_path\n");
        fflush(ctx.ocr_crop_index_fp);
    }

    if (offline_mode) {
        fprintf(stderr,
                "Start OFFLINE OCR: image=%s roi=%s auto_det=%d min_plate=%.2f det_resize=%s plate_refine=%d "
                "plate_det=%s nms_iou=%.2f max_det=%d cls_filter=%d "
                "ocr_ch=%s ocr_crop=%s ocr_resize=%s ocr_kernel=%s ocr_pp=%s min_h=%d min_sharp=%.2f min_occ=%.2f "
                "crop_src=fullres_raw det_src=%s quad_refiner=%s\n",
                ctx.opt.offline_image_path,
                (ctx.opt.offline_roi_arg && ctx.opt.offline_roi_arg[0]) ? ctx.opt.offline_roi_arg : "<none>",
                ctx.opt.offline_detect_plate,
                ctx.opt.min_plate_conf,
                det_resize_mode_str(ctx.opt.det_resize_mode),
                ctx.opt.plate_refine,
                detector_type_str(ctx.opt.plate_detector_type),
                ctx.opt.plate_nms_iou,
                ctx.opt.plate_max_det,
                ctx.opt.plate_class_id,
                ocr_channel_order_str(ctx.opt.ocr_channel_order),
                ocr_crop_mode_str(ctx.opt.ocr_crop_mode),
                ocr_resize_mode_str(ctx.opt.ocr_resize_mode),
                ocr_resize_kernel_str(ctx.opt.ocr_resize_kernel),
                ocr_preproc_mode_str(ctx.opt.ocr_preproc_mode),
                ctx.opt.ocr_min_plate_h,
                ctx.opt.ocr_min_sharpness,
                ctx.opt.ocr_min_occ_ratio,
                ctx.opt.sw_preproc ? "preproc" : "raw",
                ctx.opt.quad_refiner_model_path ? ctx.opt.quad_refiner_model_path : "<off>");
        if (run_offline_once(&ctx) < 0)
            goto out;
        ret = 0;
        goto out;
    }

    ctx.infer_latest_raw = malloc(ctx.src_frame_size);
    if (!ctx.infer_latest_raw)
        goto out;
    if (build_pipeline(&ctx) < 0)
        goto out;
    if (pthread_create(&ctx.infer_thread, NULL, infer_thread_main, &ctx) != 0)
        goto out;

    fprintf(stderr,
            "Start LPR loop: fps=%d src=%s pixel=%s swap16=%s min_car=%.2f min_plate=%.2f plate_only=%d "
            "sw_preproc=%d fpga_a_mask=%d ped_event=%d det_resize=%s plate_refine=%d "
            "plate_det=%s nms_iou=%.2f max_det=%d cls_filter=%d "
            "ocr_ch=%s ocr_crop=%s ocr_resize=%s ocr_kernel=%s ocr_pp=%s min_h=%d min_sharp=%.2f min_occ=%.2f show_crop=%d "
            "crop_src=fullres_raw det_src=%s ctc_diag=%d ocr_dump=%s max=%d pred_log=%s quad_refiner=%s\n",
            ctx.opt.fps,
            ctx.src_is_bgrx ? "bgrx8888" : "bgr565",
            (ctx.opt.pixel_order == PIXEL_ORDER_BGR565) ? "bgr565" : "rgb565",
            ctx.opt.swap16 ? "on" : "off",
            ctx.opt.min_car_conf, ctx.opt.min_plate_conf,
            ctx.opt.plate_only,
            ctx.opt.sw_preproc,
            ctx.opt.fpga_a_mask,
            ctx.opt.ped_event,
            det_resize_mode_str(ctx.opt.det_resize_mode),
            ctx.opt.plate_refine,
            detector_type_str(ctx.opt.plate_detector_type),
            ctx.opt.plate_nms_iou,
            ctx.opt.plate_max_det,
            ctx.opt.plate_class_id,
            ocr_channel_order_str(ctx.opt.ocr_channel_order),
            ocr_crop_mode_str(ctx.opt.ocr_crop_mode),
            ocr_resize_mode_str(ctx.opt.ocr_resize_mode),
            ocr_resize_kernel_str(ctx.opt.ocr_resize_kernel),
            ocr_preproc_mode_str(ctx.opt.ocr_preproc_mode),
            ctx.opt.ocr_min_plate_h,
            ctx.opt.ocr_min_sharpness,
            ctx.opt.ocr_min_occ_ratio,
            ctx.opt.show_crop_box,
            ctx.opt.sw_preproc ? "preproc" : "raw",
            ctx.opt.ocr_ctc_diag,
            ctx.opt.ocr_crop_dump_dir ? ctx.opt.ocr_crop_dump_dir : "<off>",
            ctx.opt.ocr_crop_dump_max,
            ctx.opt.pred_log_path ? ctx.opt.pred_log_path : "<off>",
            ctx.opt.quad_refiner_model_path ? ctx.opt.quad_refiner_model_path : "<off>");

    ctx.last_stats_us = mono_us();

    while (ctx.running) {
        struct slot_ticket ticket;
        GstBuffer *buf;
        GstFlowReturn flow;
        int64_t t0;
        int64_t loop_us;
        int64_t target_us = 1000000LL / ctx.opt.fps;

        if (g_stop)
            ctx.running = false;
        if (!ctx.running)
            break;
        if (handle_bus_messages(&ctx) < 0)
            break;

        t0 = mono_us();
        if (trigger_frame_dma(&ctx) < 0)
            break;
        ctx.captured_frames++;

        if (acquire_free_slot(&ctx, &ticket) < 0)
            break;
        copy_frame_to_slot565(&ctx, ctx.slots[ticket.idx].data, ctx.dma_copy);
        push_latest_to_infer(&ctx, ctx.dma_copy);
        overlay_results_on_slot(&ctx, ctx.slots[ticket.idx].data);

        buf = build_frame_buffer(&ctx, &ticket);
        if (!buf)
            break;
        flow = gst_app_src_push_buffer(GST_APP_SRC(ctx.appsrc), buf);
        if (flow != GST_FLOW_OK) {
            release_slot_ticket(&ctx, &ticket, false);
            break;
        }
        ctx.pushed_frames++;
        print_stats(&ctx);

        loop_us = mono_us() - t0;
        if (loop_us < target_us)
            usleep((useconds_t)(target_us - loop_us));
    }

    fprintf(stderr, "Exit: cap=%" PRIu64 " push=%" PRIu64 " rel=%" PRIu64 "\n",
            ctx.captured_frames, ctx.pushed_frames, ctx.released_frames);
    ret = 0;
out:
    cleanup(&ctx);
    return ret;
}
