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

static int test_bilinear_resize(void)
{
    const uint8_t source[2 * 3] = {0, 0, 0, 100, 200, 40};
    uint8_t output[4 * 3] = {0};
    cplus_resize_rgb_bilinear(source, 2, 1, output, 4, 1);
    CHECK(output[0] == 0 && output[1] == 0 && output[2] == 0);
    CHECK(output[3] == 25 && output[4] == 50 && output[5] == 10);
    CHECK(output[6] == 75 && output[7] == 150 && output[8] == 30);
    CHECK(output[9] == 100 && output[10] == 200 && output[11] == 40);
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

static int test_fp16_yolo_decode(void)
{
    size_t elements = (size_t)CPLUS_YOLO_CHANNELS * CPLUS_YOLO_PREDICTIONS;
    uint16_t *output = calloc(elements, sizeof(*output));
    struct cplus_letterbox meta = {
        .scale = 1.0f, .src_w = 640, .src_h = 640,
        .dst_w = 640, .dst_h = 640, .valid = true
    };
    struct cplus_runtime_config config;
    struct cplus_detection detections[CPLUS_MAX_DETECTIONS];
    int count;
    CHECK(output);
    cplus_default_runtime_config(&config);
    output[0] = 0x5d00U; /* 320 */
    output[CPLUS_YOLO_PREDICTIONS] = 0x5d00U;
    output[(size_t)2 * CPLUS_YOLO_PREDICTIONS] = 0x5640U; /* 100 */
    output[(size_t)3 * CPLUS_YOLO_PREDICTIONS] = 0x5a40U; /* 200 */
    output[(size_t)(4 + CPLUS_COCO_PERSON) * CPLUS_YOLO_PREDICTIONS] = 0x3b33U;
    count = cplus_decode_yolo_fp16(output, elements, &meta, &config,
                                   detections, CPLUS_MAX_DETECTIONS);
    CHECK(count == 1);
    CHECK(detections[0].box.class_id == CPLUS_COCO_PERSON);
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

static int test_fp16_mask_argmax(void)
{
    size_t plane = CPLUS_MODEL_PIXELS;
    uint16_t *logits = calloc(plane * 4U, sizeof(*logits));
    uint8_t *mask = malloc(plane);
    size_t pixel;
    CHECK(logits && mask);
    for (pixel = 0; pixel < plane; ++pixel) logits[pixel] = 0x3c00U;
    logits[11] = 0;
    logits[plane + 11] = 0x3c00U;
    logits[12] = 0;
    logits[2U * plane + 12] = 0x4000U;
    logits[13] = 0;
    logits[3U * plane + 13] = 0x4200U;
    logits[14] = 0xc000U;
    logits[plane + 14] = 0xbc00U;
    logits[2U * plane + 14] = 0xc200U;
    logits[3U * plane + 14] = 0xc400U;
    logits[15] = 0x7e00U;
    logits[2U * plane + 15] = 0x3c00U;
    CHECK(cplus_mask_argmax_fp16(logits, plane * 4U, mask) == 0);
    CHECK(mask[0] == CPLUS_MASK_OTHER);
    CHECK(mask[11] == CPLUS_MASK_ROAD);
    CHECK(mask[12] == CPLUS_MASK_SIDEWALK);
    CHECK(mask[13] == CPLUS_MASK_CROSSWALK_ZEBRA);
    CHECK(mask[14] == CPLUS_MASK_ROAD);
    CHECK(mask[15] == CPLUS_MASK_SIDEWALK);
    free(logits);
    free(mask);
    return 0;
}

static int test_mask_overlay_palette(void)
{
    uint8_t pixels[4 * 4] = {
        20, 40, 60, 0, 20, 40, 60, 0,
        20, 40, 60, 0, 20, 40, 60, 0,
    };
    const uint8_t mask[4] = {
        CPLUS_MASK_OTHER, CPLUS_MASK_ROAD,
        CPLUS_MASK_SIDEWALK, CPLUS_MASK_CROSSWALK_ZEBRA,
    };
    cplus_overlay_mask_bgrx(pixels, 16, 4, 1, mask, 4, 1);
    CHECK(pixels[0] == 20 && pixels[1] == 40 && pixels[2] == 60);
    CHECK(pixels[4] == 10 && pixels[5] == 110 && pixels[6] == 30);
    CHECK(pixels[8] == 137 && pixels[9] == 80 && pixels[10] == 30);
    CHECK(pixels[12] == 35 && pixels[13] == 45 && pixels[14] == 157);
    {
        uint8_t scaled[3 * 4] = {0};
        const uint8_t scaled_mask[2] = {CPLUS_MASK_OTHER, CPLUS_MASK_ROAD};
        cplus_overlay_mask_bgrx(scaled, 12, 3, 1, scaled_mask, 2, 1);
        CHECK(scaled[1] == 0);
        CHECK(scaled[5] == 90);
        CHECK(scaled[9] == 90);
    }
    return 0;
}

static int test_candidate_c_mask_cleanup(void)
{
    uint8_t *mask = malloc(CPLUS_MODEL_PIXELS);
    struct cplus_mask_stats stats;
    struct cplus_mask_workspace workspace;
    int pass, x, y;
    CHECK(mask);
    CHECK(cplus_mask_workspace_init(&workspace, CPLUS_MODEL_WIDTH,
                                    CPLUS_MODEL_HEIGHT) == 0);
    for (pass = 0; pass < 2; ++pass) {
        memset(mask, CPLUS_MASK_ROAD, CPLUS_MODEL_PIXELS);
        for (y = 300; y < 305; ++y) for (x = 300; x < 305; ++x)
            mask[(size_t)y * CPLUS_MODEL_WIDTH + x] = CPLUS_MASK_OTHER;
        for (y = 100; y < 110; ++y) for (x = 100; x < 110; ++x)
            mask[(size_t)y * CPLUS_MODEL_WIDTH + x] = CPLUS_MASK_CROSSWALK_ZEBRA;
        CHECK(cplus_postprocess_mask_candidate_c_workspace(
                  mask, CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT,
                  &stats, &workspace) == 0);
        CHECK(mask[(size_t)302 * CPLUS_MODEL_WIDTH + 302] == CPLUS_MASK_ROAD);
        CHECK(mask[(size_t)105 * CPLUS_MODEL_WIDTH + 105] == CPLUS_MASK_ROAD);
        CHECK(stats.road_hole_pixels_filled >= 25);
        CHECK(stats.small_zebra_pixels_removed == 100);
    }
    memset(mask, CPLUS_MASK_OTHER, CPLUS_MODEL_PIXELS);
    CHECK(cplus_postprocess_mask_candidate_c_workspace(
              mask, CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT,
              &stats, &workspace) == 0);
    CHECK(stats.changed_pixels_total == 0);
    cplus_mask_workspace_release(&workspace);
    free(mask);
    return 0;
}

int main(void)
{
    CHECK(test_preprocessing_contracts() == 0);
    CHECK(test_bilinear_resize() == 0);
    CHECK(test_yolo_decode_and_nms() == 0);
    CHECK(test_fp16_yolo_decode() == 0);
    CHECK(test_rider_filtering() == 0);
    CHECK(test_decision_rules() == 0);
    CHECK(test_fp16_mask_argmax() == 0);
    CHECK(test_mask_overlay_palette() == 0);
    CHECK(test_candidate_c_mask_cleanup() == 0);
    puts("cplus_engine tests passed");
    return 0;
}
