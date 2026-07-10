// SPDX-License-Identifier: MIT

#include "cplus_core.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return -1; \
    } \
} while (0)

static int nearly_equal(float left, float right)
{
    return fabsf(left - right) < 0.001f;
}

static int test_preprocessing_contracts(void)
{
    uint8_t source[4 * 2 * 3];
    uint8_t *detector = malloc((size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT * 3U);
    uint8_t *segmenter = malloc((size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT * 3U);
    struct cplus_letterbox meta;
    int index;
    CHECK(detector && segmenter);
    for (index = 0; index < (int)sizeof(source); ++index) source[index] = (uint8_t)(index + 1);
    cplus_prepare_detector_rgb(source, 4, 2, detector, &meta);
    cplus_prepare_segmenter_rgb(source, 4, 2, segmenter);
    CHECK(meta.valid);
    CHECK(nearly_equal(meta.scale, 160.0f));
    CHECK(meta.pad_x == 0 && meta.pad_y == 160);
    CHECK(detector[0] == 114 && detector[1] == 114 && detector[2] == 114);
    CHECK(memcmp(detector + ((size_t)160 * CPLUS_MODEL_WIDTH) * 3U, source, 3U) == 0);
    CHECK(memcmp(segmenter, source, 3U) == 0);
    free(detector);
    free(segmenter);
    return 0;
}

static int test_yolo_decode_and_nms(void)
{
    size_t floats = (size_t)CPLUS_YOLO_CHANNELS * CPLUS_YOLO_PREDICTIONS;
    float *output = calloc(floats, sizeof(*output));
    struct cplus_letterbox meta = { .scale = 1.0f, .src_w = 640, .src_h = 640, .dst_w = 640, .dst_h = 640, .valid = true };
    struct cplus_runtime_config config;
    struct cplus_detection detections[CPLUS_MAX_DETECTIONS];
    int count;
    CHECK(output);
    cplus_default_runtime_config(&config);
    output[0] = 320.0f; output[CPLUS_YOLO_PREDICTIONS] = 320.0f;
    output[(size_t)2 * CPLUS_YOLO_PREDICTIONS] = 100.0f;
    output[(size_t)3 * CPLUS_YOLO_PREDICTIONS] = 200.0f;
    output[(size_t)(4 + CPLUS_COCO_PERSON) * CPLUS_YOLO_PREDICTIONS] = 0.90f;
    output[1] = 322.0f; output[CPLUS_YOLO_PREDICTIONS + 1] = 320.0f;
    output[(size_t)2 * CPLUS_YOLO_PREDICTIONS + 1] = 100.0f;
    output[(size_t)3 * CPLUS_YOLO_PREDICTIONS + 1] = 200.0f;
    output[(size_t)(4 + CPLUS_COCO_PERSON) * CPLUS_YOLO_PREDICTIONS + 1] = 0.80f;
    output[2] = 320.0f; output[CPLUS_YOLO_PREDICTIONS + 2] = 350.0f;
    output[(size_t)2 * CPLUS_YOLO_PREDICTIONS + 2] = 110.0f;
    output[(size_t)3 * CPLUS_YOLO_PREDICTIONS + 2] = 70.0f;
    output[(size_t)(4 + CPLUS_COCO_BICYCLE) * CPLUS_YOLO_PREDICTIONS + 2] = 0.70f;
    count = cplus_decode_yolo(output, floats, &meta, &config, detections, CPLUS_MAX_DETECTIONS);
    CHECK(count == 2);
    CHECK(detections[0].box.class_id == CPLUS_COCO_PERSON);
    CHECK(detections[1].box.class_id == CPLUS_COCO_BICYCLE);
    CHECK(nearly_equal(detections[0].box.x1, 270.0f));
    CHECK(nearly_equal(detections[0].box.y1, 220.0f));
    free(output);
    return 0;
}

static int test_rider_filtering(void)
{
    struct cplus_runtime_config config;
    struct cplus_detection detections[3] = {
        { .box = {100.0f, 100.0f, 200.0f, 300.0f, 0.90f, CPLUS_COCO_PERSON}, .target_type = CPLUS_TARGET_PEDESTRIAN },
        { .box = {105.0f, 200.0f, 215.0f, 295.0f, 0.80f, CPLUS_COCO_BICYCLE}, .target_type = CPLUS_TARGET_BICYCLE },
        { .box = {400.0f, 120.0f, 480.0f, 290.0f, 0.95f, CPLUS_COCO_PERSON}, .target_type = CPLUS_TARGET_PEDESTRIAN },
    };
    cplus_default_runtime_config(&config);
    cplus_assign_riders(detections, 3, 640, 480, &config);
    CHECK(detections[0].target_type == CPLUS_TARGET_BICYCLE_RIDER);
    CHECK(detections[2].target_type == CPLUS_TARGET_PEDESTRIAN);
    return 0;
}

static int test_decision_rules(void)
{
    struct cplus_ground_ratios ratios = { .road = 0.70f, .sidewalk = 0.10f, .zebra = 0.0f, .known_ground = 0.80f };
    enum cplus_decision decision;
    enum cplus_reason reason;
    cplus_apply_rule(&ratios, &decision, &reason);
    CHECK(decision == CPLUS_DECISION_SUSPECTED);
    CHECK(reason == CPLUS_REASON_ROAD_DOMINANT_WITHOUT_ZEBRA);
    ratios.sidewalk = 0.30f;
    cplus_apply_rule(&ratios, &decision, &reason);
    CHECK(decision == CPLUS_DECISION_NOT_SUSPECTED);
    CHECK(reason == CPLUS_REASON_SIDEWALK_SUPPRESSED);
    ratios.road = 0.70f; ratios.sidewalk = 0.0f; ratios.zebra = 0.10f;
    ratios.known_ground = 0.80f; ratios.total_pixels = 1000; ratios.zebra_pixels = 100;
    ratios.max_zebra_component_pixels = 70;
    cplus_apply_rule(&ratios, &decision, &reason);
    CHECK(decision == CPLUS_DECISION_NOT_SUSPECTED);
    CHECK(reason == CPLUS_REASON_ZEBRA_SUPPRESSED);
    ratios.max_zebra_component_pixels = 5;
    cplus_apply_rule(&ratios, &decision, &reason);
    CHECK(decision == CPLUS_DECISION_SUSPECTED);
    CHECK(reason == CPLUS_REASON_ROAD_DOMINANT_ZEBRA_NOISE_IGNORED);
    return 0;
}

static int test_candidate_c_mask_cleanup(void)
{
    uint8_t *mask = malloc((size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT);
    struct cplus_mask_stats stats;
    int x, y;
    CHECK(mask);
    memset(mask, CPLUS_MASK_ROAD, (size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT);
    for (y = 300; y < 305; ++y) for (x = 300; x < 305; ++x)
        mask[(size_t)y * CPLUS_MODEL_WIDTH + x] = CPLUS_MASK_OTHER;
    for (y = 100; y < 110; ++y) for (x = 100; x < 110; ++x)
        mask[(size_t)y * CPLUS_MODEL_WIDTH + x] = CPLUS_MASK_CROSSWALK_ZEBRA;
    CHECK(cplus_postprocess_mask_candidate_c(mask, CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT, &stats) == 0);
    CHECK(mask[(size_t)302 * CPLUS_MODEL_WIDTH + 302] == CPLUS_MASK_ROAD);
    CHECK(mask[(size_t)105 * CPLUS_MODEL_WIDTH + 105] == CPLUS_MASK_ROAD);
    CHECK(stats.road_hole_pixels_filled >= 25);
    CHECK(stats.small_zebra_pixels_removed == 100);
    free(mask);
    return 0;
}

int main(void)
{
    CHECK(test_preprocessing_contracts() == 0);
    CHECK(test_yolo_decode_and_nms() == 0);
    CHECK(test_rider_filtering() == 0);
    CHECK(test_decision_rules() == 0);
    CHECK(test_candidate_c_mask_cleanup() == 0);
    puts("cplus_engine tests passed");
    return 0;
}
