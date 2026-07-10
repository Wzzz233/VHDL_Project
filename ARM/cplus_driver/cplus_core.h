// SPDX-License-Identifier: MIT
/*
 * C+ road-crossing driver core.
 *
 * This module contains the model-independent part of the RK3568 handoff:
 * image mapping, COCO output decoding, rider suppression, Candidate C mask
 * filtering and the final pedestrian decision.  It deliberately has no
 * dependency on RKNN so it can be tested on the host.
 */

#ifndef CPLUS_DRIVER_CORE_H
#define CPLUS_DRIVER_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CPLUS_MODEL_WIDTH 640
#define CPLUS_MODEL_HEIGHT 640
#define CPLUS_YOLO_CHANNELS 84
#define CPLUS_YOLO_PREDICTIONS 8400
#define CPLUS_MAX_DETECTIONS 128

enum cplus_class_id {
    CPLUS_COCO_PERSON = 0,
    CPLUS_COCO_BICYCLE = 1,
    CPLUS_COCO_MOTORCYCLE = 3,
};

enum cplus_mask_class {
    CPLUS_MASK_OTHER = 0,
    CPLUS_MASK_ROAD = 1,
    CPLUS_MASK_SIDEWALK = 2,
    CPLUS_MASK_CROSSWALK_ZEBRA = 3,
};

enum cplus_target_type {
    CPLUS_TARGET_PEDESTRIAN,
    CPLUS_TARGET_BICYCLE,
    CPLUS_TARGET_MOTORCYCLE,
    CPLUS_TARGET_BICYCLE_RIDER,
    CPLUS_TARGET_MOTORCYCLE_RIDER,
};

enum cplus_decision {
    CPLUS_DECISION_NOT_SUSPECTED,
    CPLUS_DECISION_SUSPECTED,
};

enum cplus_reason {
    CPLUS_REASON_TWO_WHEEL_FILTERED,
    CPLUS_REASON_RIDER_FILTERED,
    CPLUS_REASON_ZEBRA_SUPPRESSED,
    CPLUS_REASON_SIDEWALK_SUPPRESSED,
    CPLUS_REASON_WEAK_GROUND_EVIDENCE,
    CPLUS_REASON_ROAD_DOMINANT_WITHOUT_ZEBRA,
    CPLUS_REASON_ROAD_DOMINANT_ZEBRA_NOISE_IGNORED,
    CPLUS_REASON_ROAD_NOT_DOMINANT,
};

struct cplus_box {
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    int class_id;
};

struct cplus_detection {
    struct cplus_box box;
    enum cplus_target_type target_type;
};

struct cplus_letterbox {
    float scale;
    int pad_x;
    int pad_y;
    int src_w;
    int src_h;
    int dst_w;
    int dst_h;
    bool valid;
};

struct cplus_rect {
    int x1;
    int y1;
    int x2;
    int y2;
};

struct cplus_ground_ratios {
    float road;
    float sidewalk;
    float zebra;
    float known_ground;
    int total_pixels;
    int zebra_pixels;
    int max_zebra_component_pixels;
};

struct cplus_person_result {
    struct cplus_detection detection;
    struct cplus_rect foot_box_source;
    struct cplus_rect foot_box_mask;
    struct cplus_ground_ratios ground;
    enum cplus_decision decision;
    enum cplus_reason reason;
};

struct cplus_runtime_config {
    float person_confidence;
    float vehicle_confidence;
    float class_aware_nms_iou;
    float person_dedup_iou;
    float rider_lower_overlap;
    float rider_person_overlap;
    float rider_min_vehicle_confidence;
    float rider_min_person_height_ratio;
};

struct cplus_mask_stats {
    int road_hole_components_filled;
    int road_hole_pixels_filled;
    int sidewalk_hole_components_filled;
    int sidewalk_hole_pixels_filled;
    int small_zebra_components_removed;
    int small_zebra_pixels_removed;
    int zebra_hole_components_filled;
    int zebra_hole_pixels_filled;
    int zebra_inner_island_components_absorbed;
    int zebra_inner_island_pixels_absorbed;
    int changed_pixels_total;
};

void cplus_default_runtime_config(struct cplus_runtime_config *config);

const char *cplus_target_type_name(enum cplus_target_type type);
const char *cplus_decision_name(enum cplus_decision decision);
const char *cplus_reason_name(enum cplus_reason reason);

void cplus_resize_rgb_nearest(const uint8_t *src, int src_w, int src_h,
                               uint8_t *dst, int dst_w, int dst_h);
void cplus_prepare_detector_rgb(const uint8_t *src, int src_w, int src_h,
                                 uint8_t *dst, struct cplus_letterbox *meta);
void cplus_prepare_segmenter_rgb(const uint8_t *src, int src_w, int src_h,
                                  uint8_t *dst);
void cplus_map_box_from_detector(struct cplus_box *box, const struct cplus_letterbox *meta);

float cplus_box_iou(const struct cplus_box *a, const struct cplus_box *b);
int cplus_decode_yolo(const float *output, size_t float_count,
                      const struct cplus_letterbox *meta,
                      const struct cplus_runtime_config *config,
                      struct cplus_detection *detections, int capacity);
void cplus_assign_riders(struct cplus_detection *detections, int count,
                         int image_w, int image_h,
                         const struct cplus_runtime_config *config);

int cplus_mask_argmax(const float *logits, size_t float_count, uint8_t *mask);
int cplus_postprocess_mask_candidate_c(uint8_t *mask, int width, int height,
                                        struct cplus_mask_stats *stats);
void cplus_make_foot_box(const struct cplus_box *box, int image_w, int image_h,
                         struct cplus_rect *foot);
void cplus_scale_rect(const struct cplus_rect *source, int source_w, int source_h,
                      int dest_w, int dest_h, struct cplus_rect *dest);
void cplus_compute_ground_ratios(const uint8_t *mask, int mask_w, int mask_h,
                                 const struct cplus_rect *foot,
                                 struct cplus_ground_ratios *ratios);
void cplus_apply_rule(const struct cplus_ground_ratios *ratios,
                      enum cplus_decision *decision, enum cplus_reason *reason);
int cplus_evaluate_detections(const struct cplus_detection *detections, int count,
                              const uint8_t *mask, int mask_w, int mask_h,
                              int image_w, int image_h,
                              struct cplus_person_result *results, int capacity);

#endif /* CPLUS_DRIVER_CORE_H */
