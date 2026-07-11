// SPDX-License-Identifier: MIT

#include "cplus_async.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static int64_t monotonic_us(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)now.tv_sec * 1000000LL + now.tv_nsec / 1000;
}

static double elapsed_ms(int64_t start, int64_t end)
{
    return (double)(end - start) / 1000.0;
}

static double npu_duration_ms(int64_t duration_us)
{
    return duration_us >= 0 ? (double)duration_us / 1000.0 : -1.0;
}

static void mask_class_counts(const uint8_t *mask, uint64_t counts[4])
{
    size_t index;
    memset(counts, 0, 4U * sizeof(counts[0]));
    for (index = 0; index < CPLUS_MODEL_PIXELS; ++index)
        if (mask[index] < 4) ++counts[mask[index]];
}

static int decode_detector_output(const struct cplus_rknn_output_view *output,
                                  const struct cplus_letterbox *letterbox,
                                  const struct cplus_runtime_config *config,
                                  struct cplus_detection *detections)
{
    if (output->type == RKNN_TENSOR_FLOAT16)
        return cplus_decode_yolo_fp16(output->data, output->element_count,
                                     letterbox, config, detections,
                                     CPLUS_MAX_DETECTIONS);
    if (output->type == RKNN_TENSOR_FLOAT32)
        return cplus_decode_yolo(output->data, output->element_count,
                                 letterbox, config, detections,
                                 CPLUS_MAX_DETECTIONS);
    fprintf(stderr, "[detector] unsupported native output type %d\n",
            (int)output->type);
    return -1;
}

static int decode_segmenter_output(const struct cplus_rknn_output_view *output,
                                   uint8_t *mask)
{
    if (output->type == RKNN_TENSOR_FLOAT16)
        return cplus_mask_argmax_fp16(output->data, output->element_count, mask);
    if (output->type == RKNN_TENSOR_FLOAT32)
        return cplus_mask_argmax(output->data, output->element_count, mask);
    fprintf(stderr, "[segmenter] unsupported native output type %d\n",
            (int)output->type);
    return -1;
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

static int ordinary_people_count(const struct cplus_detection *detections, int count)
{
    int index;
    int ordinary = 0;
    for (index = 0; index < count; ++index) {
        if (detections[index].box.class_id == CPLUS_COCO_PERSON &&
            detections[index].target_type == CPLUS_TARGET_PEDESTRIAN)
            ++ordinary;
    }
    return ordinary;
}

static void print_result_json(const struct cplus_async_result *result)
{
    int index;
    printf("{\"frame\":%llu,\"targets\":[", (unsigned long long)result->source_frame);
    for (index = 0; index < result->count; ++index) {
        const struct cplus_person_result *target = &result->results[index];
        if (index) putchar(',');
        printf("{\"type\":\"%s\",\"score\":%.3f,\"decision\":\"%s\",\"reason\":\"%s\","
               "\"box\":[%.1f,%.1f,%.1f,%.1f],\"ground\":{\"road\":%.4f,\"sidewalk\":%.4f,\"zebra\":%.4f}}",
               cplus_target_type_name(target->detection.target_type), target->detection.box.score,
               cplus_decision_name(target->decision), cplus_reason_name(target->reason),
               target->detection.box.x1, target->detection.box.y1,
               target->detection.box.x2, target->detection.box.y2,
               target->ground.road, target->ground.sidewalk, target->ground.zebra);
    }
    puts("]}");
    fflush(stdout);
}

static int infer_frame(struct cplus_async_infer *state, const uint8_t *bgrx,
                       uint64_t source_frame, struct cplus_async_result *result)
{
    struct cplus_detection detections[CPLUS_MAX_DETECTIONS];
    struct cplus_letterbox letterbox;
    struct cplus_mask_stats mask_stats;
    struct cplus_rknn_output_view output;
    uint64_t raw_counts[4] = {0};
    uint64_t post_counts[4] = {0};
    int64_t started;
    int64_t rgb_done;
    int64_t detector_prepared;
    int64_t detector_done;
    int64_t detector_post_done;
    int64_t detector_npu_us = -1;
    int64_t segmenter_npu_us = -1;
    int64_t segmenter_prepared;
    int64_t segmenter_done;
    int64_t argmax_done;
    int64_t mask_post_done;
    int64_t finished;
    int detection_count;
    int ordinary_count;
    bool segmented = false;

    memset(result, 0, sizeof(*result));
    memset(&mask_stats, 0, sizeof(mask_stats));
    result->source_frame = source_frame;
    started = monotonic_us();
    bgrx_to_rgb(bgrx, state->width, state->height, state->rgb);
    rgb_done = monotonic_us();
    cplus_prepare_detector_rgb(state->rgb, state->width, state->height,
                               state->detector_rgb, &letterbox);
    detector_prepared = monotonic_us();
    if (cplus_rknn_infer_rgb(state->detector, state->detector_rgb,
                             CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT,
                             &output) < 0)
        return -1;
    detector_npu_us = output.npu_duration_us;
    detector_done = monotonic_us();
    detection_count = decode_detector_output(&output, &letterbox,
                                               &state->config, detections);
    cplus_rknn_release_output(state->detector);
    if (detection_count < 0) return -1;
    cplus_assign_riders(detections, detection_count, state->width, state->height,
                        &state->config);
    ordinary_count = ordinary_people_count(detections, detection_count);
    detector_post_done = monotonic_us();
    segmenter_prepared = detector_post_done;
    segmenter_done = detector_post_done;
    argmax_done = detector_post_done;
    mask_post_done = detector_post_done;

    if (ordinary_count || state->always_segment) {
        segmented = true;
        cplus_prepare_segmenter_rgb(state->rgb, state->width, state->height,
                                    state->segmenter_rgb);
        segmenter_prepared = monotonic_us();
        if (cplus_rknn_infer_rgb(state->segmenter, state->segmenter_rgb,
                                 CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT,
                                 &output) < 0)
            return -1;
        segmenter_npu_us = output.npu_duration_us;
        segmenter_done = monotonic_us();
        if (decode_segmenter_output(&output, state->mask) < 0) {
            cplus_rknn_release_output(state->segmenter);
            return -1;
        }
        cplus_rknn_release_output(state->segmenter);
        argmax_done = monotonic_us();
        mask_class_counts(state->mask, raw_counts);
        if (cplus_postprocess_mask_candidate_c_workspace(
                state->mask, CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT,
                &mask_stats, &state->mask_workspace) < 0)
            return -1;
        mask_post_done = monotonic_us();
        mask_class_counts(state->mask, post_counts);
        memcpy(result->mask, state->mask, CPLUS_MODEL_PIXELS);
        result->mask_valid = true;
    } else {
        memset(state->mask, CPLUS_MASK_OTHER, CPLUS_MODEL_PIXELS);
    }

    result->count = cplus_evaluate_detections(
        detections, detection_count, state->mask,
        CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT,
        state->width, state->height,
        result->results, CPLUS_MAX_DETECTIONS);
    if (result->count < 0) return -1;
    result->valid = true;
    finished = monotonic_us();
    fprintf(stderr,
            "[infer] source_frame=%llu targets=%d ordinary_pedestrians=%d%s "
            "timing_ms={rgb:%.1f,det_prep:%.1f,det_run:%.1f,det_npu:%.1f,det_post:%.1f,"
            "seg_prep:%.1f,seg_run:%.1f,seg_npu:%.1f,argmax:%.1f,mask_post:%.1f,rules:%.1f,total:%.1f}\n",
            (unsigned long long)source_frame, result->count, ordinary_count,
            segmented ? " segmented" : "",
            elapsed_ms(started, rgb_done),
            elapsed_ms(rgb_done, detector_prepared),
            elapsed_ms(detector_prepared, detector_done),
            npu_duration_ms(detector_npu_us),
            elapsed_ms(detector_done, detector_post_done),
            elapsed_ms(detector_post_done, segmenter_prepared),
            elapsed_ms(segmenter_prepared, segmenter_done),
            npu_duration_ms(segmenter_npu_us),
            elapsed_ms(segmenter_done, argmax_done),
            elapsed_ms(argmax_done, mask_post_done),
            elapsed_ms(mask_post_done, finished),
            elapsed_ms(started, finished));
    if (segmented) {
        fprintf(stderr,
                "[mask] source_frame=%llu raw=[other:%llu road:%llu sidewalk:%llu zebra:%llu] "
                "post=[other:%llu road:%llu sidewalk:%llu zebra:%llu] changed=%d\n",
                (unsigned long long)source_frame,
                (unsigned long long)raw_counts[0],
                (unsigned long long)raw_counts[1],
                (unsigned long long)raw_counts[2],
                (unsigned long long)raw_counts[3],
                (unsigned long long)post_counts[0],
                (unsigned long long)post_counts[1],
                (unsigned long long)post_counts[2],
                (unsigned long long)post_counts[3],
                mask_stats.changed_pixels_total);
        if (!state->mask_all_other_reported &&
            raw_counts[1] + raw_counts[2] + raw_counts[3] == 0) {
            fprintf(stderr,
                    "[mask] WARNING: segmenter returned only class 0; verify model checksum "
                    "and RGB uint8/ImageNet RKNN preprocessing\n");
            state->mask_all_other_reported = true;
        }
    }
    return 0;
}

static void *infer_thread_main(void *argument)
{
    struct cplus_async_infer *state = argument;
    int slot;
    int take_status;

    while ((take_status = cplus_latest_queue_take(&state->queue, &slot)) > 0) {
        struct cplus_async_result result;
        uint64_t source_frame = state->slot_frame[slot];
        const uint8_t *frame = cplus_frame_pool_data(state->pool, slot);
        int inference_status = infer_frame(state, frame, source_frame, &result);
        cplus_frame_pool_release(state->pool, slot);
        pthread_mutex_lock(&state->result_lock);
        if (inference_status < 0) {
            state->failed = 1;
            pthread_cond_broadcast(&state->result_cond);
            pthread_mutex_unlock(&state->result_lock);
            fprintf(stderr, "[infer] source_frame=%llu failed\n",
                    (unsigned long long)source_frame);
            cplus_latest_queue_stop(&state->queue);
            break;
        }
        ++state->completed;
        result.sequence = state->completed;
        state->result = result;
        pthread_cond_broadcast(&state->result_cond);
        pthread_mutex_unlock(&state->result_lock);
        print_result_json(&result);
    }
    if (take_status < 0) {
        pthread_mutex_lock(&state->result_lock);
        state->failed = 1;
        pthread_cond_broadcast(&state->result_cond);
        pthread_mutex_unlock(&state->result_lock);
    }
    return NULL;
}

int cplus_async_infer_start(struct cplus_async_infer *state,
                            struct cplus_frame_pool *pool,
                            int width, int height,
                            struct cplus_rknn_model *detector,
                            struct cplus_rknn_model *segmenter,
                            const struct cplus_runtime_config *config,
                            bool always_segment)
{
    if (!state || !pool || !detector || !segmenter || !config || width <= 0 || height <= 0)
        return -1;
    memset(state, 0, sizeof(*state));
    state->width = width;
    state->height = height;
    state->pool = pool;
    state->detector = detector;
    state->segmenter = segmenter;
    state->config = *config;
    state->always_segment = always_segment;
    state->rgb = malloc((size_t)width * height * 3U);
    state->detector_rgb = malloc((size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT * 3U);
    state->segmenter_rgb = malloc((size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT * 3U);
    state->mask = malloc((size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT);
    if (!state->rgb || !state->detector_rgb || !state->segmenter_rgb || !state->mask)
        goto failed;
    if (cplus_mask_workspace_init(&state->mask_workspace, CPLUS_MODEL_WIDTH,
                                  CPLUS_MODEL_HEIGHT) < 0)
        goto failed;
    state->mask_workspace_initialized = 1;
    if (pthread_mutex_init(&state->result_lock, NULL) != 0) goto failed;
    if (pthread_cond_init(&state->result_cond, NULL) != 0) {
        pthread_mutex_destroy(&state->result_lock);
        goto failed;
    }
    state->result_sync_initialized = 1;
    if (cplus_latest_queue_init(&state->queue, pool) < 0) goto failed;
    state->queue_initialized = 1;
    if (pthread_create(&state->thread, NULL, infer_thread_main, state) != 0)
        goto failed;
    state->thread_started = 1;
    return 0;

failed:
    cplus_async_infer_stop(state);
    return -1;
}

uint64_t cplus_async_infer_stop(struct cplus_async_infer *state)
{
    uint64_t completed;
    if (!state) return 0;
    if (state->queue_initialized) cplus_latest_queue_stop(&state->queue);
    if (state->thread_started) {
        pthread_join(state->thread, NULL);
        state->thread_started = 0;
    }
    if (state->queue_initialized) {
        cplus_latest_queue_destroy(&state->queue);
        state->queue_initialized = 0;
    }
    if (state->result_sync_initialized) {
        pthread_cond_destroy(&state->result_cond);
        pthread_mutex_destroy(&state->result_lock);
        state->result_sync_initialized = 0;
    }
    if (state->mask_workspace_initialized) {
        cplus_mask_workspace_release(&state->mask_workspace);
        state->mask_workspace_initialized = 0;
    }
    free(state->rgb);
    free(state->detector_rgb);
    free(state->segmenter_rgb);
    free(state->mask);
    completed = state->completed;
    memset(state, 0, sizeof(*state));
    return completed;
}

int cplus_async_submit_frame(struct cplus_async_infer *state, int slot,
                             uint64_t source_frame)
{
    if (!state || !state->thread_started || slot < 0 || slot >= CPLUS_FRAME_POOL_SLOTS)
        return -1;
    state->slot_frame[slot] = source_frame;
    return cplus_latest_queue_submit(&state->queue, slot);
}

bool cplus_async_get_result(struct cplus_async_infer *state,
                            struct cplus_async_result *result)
{
    bool valid;
    if (!state || !result || !state->result_sync_initialized) return false;
    pthread_mutex_lock(&state->result_lock);
    *result = state->result;
    valid = result->valid;
    pthread_mutex_unlock(&state->result_lock);
    return valid;
}

bool cplus_async_refresh_result(struct cplus_async_infer *state,
                                struct cplus_async_result *result)
{
    bool valid;
    if (!state || !result || !state->result_sync_initialized) return false;
    pthread_mutex_lock(&state->result_lock);
    if (state->result.valid &&
        (!result->valid || result->sequence != state->result.sequence))
        *result = state->result;
    valid = result->valid;
    pthread_mutex_unlock(&state->result_lock);
    return valid;
}

int cplus_async_wait_for_frame(struct cplus_async_infer *state,
                               uint64_t source_frame)
{
    int failed;
    if (!state || !state->result_sync_initialized) return -1;
    pthread_mutex_lock(&state->result_lock);
    while (!state->failed && (!state->result.valid || state->result.source_frame < source_frame))
        pthread_cond_wait(&state->result_cond, &state->result_lock);
    failed = state->failed;
    pthread_mutex_unlock(&state->result_lock);
    return failed ? -1 : 0;
}

bool cplus_async_failed(struct cplus_async_infer *state)
{
    int failed;
    if (!state || !state->result_sync_initialized) return true;
    pthread_mutex_lock(&state->result_lock);
    failed = state->failed;
    pthread_mutex_unlock(&state->result_lock);
    return failed != 0;
}

void cplus_async_get_stats(struct cplus_async_infer *state,
                           struct cplus_async_stats *stats)
{
    if (!stats) return;
    memset(stats, 0, sizeof(*stats));
    if (!state) return;
    if (state->queue_initialized)
        cplus_latest_queue_stats(&state->queue, &stats->submitted, &stats->replaced);
    if (state->result_sync_initialized) {
        pthread_mutex_lock(&state->result_lock);
        stats->completed = state->completed;
        pthread_mutex_unlock(&state->result_lock);
    }
}
