/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared types, enums, macros and structs for FPGA LPR display application.
 * Extracted from fpga_lpr_display.c to enable modular compilation.
 */

#ifndef FPGA_LPR_TYPES_H
#define FPGA_LPR_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <inttypes.h>

#include <gst/gst.h>
#include <rknn_api.h>

#include "pcie_fpga_dma.h"
#include "ocr_decode.h"

/* ─── Device & Display Constants ─── */

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

/* ─── Algorithm Limits ─── */

#define MAX_LABELS 256
#define MAX_LABEL_LEN 64
#define MAX_DETS 128
#define MAX_OCR_KEYS 128
#define MAX_OCR_KEY_LEN 16
#define ALGO_STREAM_SIZE 640
#define OCR_CROP_WIDTH 150
#define OCR_CROP_HEIGHT 50
#define FIRSTCHAR_WARP_WIDTH 224
#define FIRSTCHAR_WARP_HEIGHT 72
#define OBB_POINT_COUNT 8400
#define POSE_KPT_COUNT 4
#define POSE_KPT_DIMS 3
#define POSE_BOX_CHANNELS 4
#define POSE_KPT_CHANNELS (POSE_KPT_COUNT * POSE_KPT_DIMS) /* 12 */
#define POSE_MIN_CHANNELS (POSE_BOX_CHANNELS + 1 + POSE_KPT_CHANNELS) /* 17 */
/* POSE_OUTPUT_CHANNELS kept for backward compat references */
#define POSE_OUTPUT_CHANNELS POSE_MIN_CHANNELS
#define OCR_TRACK_MAX 24
#define OCR_TRACK_HIST 8
#define PLATE_TRACK_MAX 24
#define PLATE_TRACK_TTL 3
#define PLATE_TRACK_MIN_HITS 2
#define PLATE_TRACK_MATCH_IOU 0.20f
#define PLATE_TRACK_SMOOTH_ALPHA 0.40f
#define FIRSTCHAR_TRACK_HIST 8
#define GREEN_FIRSTCHAR_DEFAULT_MIN_VOTES 5
#define GREEN_FIRSTCHAR_DEFAULT_MIN_SHARE 0.60f
#define MAX_UTF8_TOKEN_BYTES 8
#define MAX_PLATE_TOKENS 16

/* ─── Rendering Colors (RGB565) ─── */

#define COLOR_YELLOW_565 0xFFE0
#define COLOR_CYAN_565 0x07FF
#define COLOR_RED_565 0xF800
#define COLOR_GREEN_565 0x07E0
#define OVERLAY_TEXT_SCALE 3

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ─── Enums ─── */

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
    DETECTOR_YOLOV8_OBB_RKNN = 0,
    DETECTOR_YOLOV8_POSE_RKNN,
    DETECTOR_YOLOV8_DET,
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

/* ─── Structs ─── */

struct options {
    const char *device_path;
    const char *drm_card_path;
    const char *ped_model_path;
    const char *plate_model_path;
    const char *ocr_model_path;
    const char *ocr_blue_model_path;
    const char *ocr_green_model_path;
    const char *ocr_yellow_model_path;
    const char *ocr_yellow_keys_path;
    const char *ocr_special_model_path;
    const char *ocr_special_keys_path;
    const char *ocr_police_model_path;
    const char *ocr_police_keys_path;
    const char *ocr_embassy_model_path;
    const char *ocr_embassy_keys_path;
    const char *ocr_keys_path;
    const char *green_firstchar_model_path;
    const char *police_firstchar_model_path;
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
    int green_firstchar_min_votes;
    float green_firstchar_min_share;
    int police_firstchar_min_votes;
    float police_firstchar_min_share;
    int clahe_enable;
    int clahe_compare;
    const char *clahe_dump_dir;
    int clahe_dump_max;
    int offline_detect_plate;
    int pose_nc;
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
    int det_cls;
    char route_name[24];
    char ocr_expert[24];
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
    char fc_tok[FIRSTCHAR_TRACK_HIST][MAX_UTF8_TOKEN_BYTES];
    float fc_conf[FIRSTCHAR_TRACK_HIST];
    int fc_count;
    int fc_next;
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
    char keys[MAX_OCR_KEYS][MAX_OCR_KEY_LEN];
    int key_count;
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

struct firstchar_model {
    const char *name;
    const char *path;
    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr input_attr;
    rknn_tensor_attr output_attr;
    uint32_t in_w;
    uint32_t in_h;
    uint32_t in_c;
};

struct tensor_cn_view {
    const float *buf;
    int c;
    int n;
    bool c_major;
};

struct plate_track {
    bool used;
    bool shown;
    int ttl;
    int hits;
    struct det_box box;
};

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

    struct yolo_model ped_model;
    struct yolo_model plate_model;
    struct ocr_model ocr_model;
    struct ocr_model ocr_green_model;
    struct ocr_model ocr_yellow_model;
    struct ocr_model ocr_special_model;
    struct ocr_model ocr_police_model;
    struct ocr_model ocr_embassy_model;
    struct firstchar_model green_firstchar_model;
    struct firstchar_model police_firstchar_model;
    struct quad_refiner_model quad_refiner_model;
    char ocr_keys[MAX_OCR_KEYS][MAX_OCR_KEY_LEN];
    int ocr_key_count;
    int ocr_blank_index;
    bool ocr_keysize_warned;
    FILE *pred_log_fp;
    FILE *ocr_crop_index_fp;
    int ocr_crop_dumped;
    int clahe_dump_count;
    pthread_mutex_t pred_log_lock;
    char labels[MAX_LABELS][MAX_LABEL_LEN];
    int label_count;
    int car_class_id;
    int person_class_id;

    struct det_box plate_hist1[MAX_DETS];
    int plate_hist1_count;
    struct det_box plate_hist2[MAX_DETS];
    int plate_hist2_count;
    struct plate_track plate_tracks[PLATE_TRACK_MAX];

    uint64_t pred_rows_total;
    uint64_t gate_plate_raw_positive_frames;
    uint64_t gate_plate_raw_positive_streak;

    struct det_box ped_track_box[MAX_DETS];
    int ped_track_count;
    struct det_box ped_track_prev[MAX_DETS];
    int ped_track_prev_count;

    struct ocr_track ocr_tracks[OCR_TRACK_MAX];
    uint64_t ocr_track_age_seq;

    bool quad_refiner_active;
    int pose_nc_detected;

    bool offline_mode;
    uint8_t *offline_rgb;
    int offline_w;
    int offline_h;

    int plate_rows_prev;
    int plate_heads_prev;
    int plate_rows_raw_prev;
    int plate_heads_raw_prev;
};

struct slot_ticket {
    int idx;
    uint64_t generation;
};

/* ─── Global externs (shared across modules) ─── */

extern volatile sig_atomic_t g_running;

#endif /* FPGA_LPR_TYPES_H */
