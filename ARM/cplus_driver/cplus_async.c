// SPDX-License-Identifier: MIT

#include "cplus_async.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
    const float *output;
    size_t output_count;
    int detection_count;
    int ordinary_count;

    memset(result, 0, sizeof(*result));
    result->source_frame = source_frame;
    bgrx_to_rgb(bgrx, state->width, state->height, state->rgb);
    cplus_prepare_detector_rgb(state->rgb, state->width, state->height,
                               state->detector_rgb, &letterbox);
    if (cplus_rknn_infer_rgb(state->detector, state->detector_rgb,
                             CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT,
                             &output, &output_count) < 0)
        return -1;
    detection_count = cplus_decode_yolo(output, output_count, &letterbox,
                                        &state->config, detections, CPLUS_MAX_DETECTIONS);
    cplus_rknn_release_output(state->detector);
    if (detection_count < 0) return -1;
    cplus_assign_riders(detections, detection_count, state->width, state->height,
                        &state->config);
    ordinary_count = ordinary_people_count(detections, detection_count);
    if (ordinary_count || state->always_segment) {
        cplus_prepare_segmenter_rgb(state->rgb, state->width, state->height,
                                    state->segmenter_rgb);
        if (cplus_rknn_infer_rgb(state->segmenter, state->segmenter_rgb,
                                 CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT,
                                 &output, &output_count) < 0)
            return -1;
        if (cplus_mask_argmax(output, output_count, state->mask) < 0) {
            cplus_rknn_release_output(state->segmenter);
            return -1;
        }
        cplus_rknn_release_output(state->segmenter);
        if (cplus_postprocess_mask_candidate_c(state->mask, CPLUS_MODEL_WIDTH,
                                               CPLUS_MODEL_HEIGHT, &mask_stats) < 0)
            return -1;
    } else {
        memset(state->mask, CPLUS_MASK_OTHER,
               (size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT);
    }
    result->count = cplus_evaluate_detections(detections, detection_count, state->mask,
                                              CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT,
                                              state->width, state->height,
                                              result->results, CPLUS_MAX_DETECTIONS);
    if (result->count < 0) return -1;
    result->valid = true;
    fprintf(stderr, "[infer] source_frame=%llu targets=%d ordinary_pedestrians=%d%s\n",
            (unsigned long long)source_frame, result->count, ordinary_count,
            (ordinary_count || state->always_segment) ? " segmented" : "");
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

void cplus_async_infer_stop(struct cplus_async_infer *state)
{
    if (!state) return;
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
    free(state->rgb);
    free(state->detector_rgb);
    free(state->segmenter_rgb);
    free(state->mask);
    memset(state, 0, sizeof(*state));
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
