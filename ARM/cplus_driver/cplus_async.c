// SPDX-License-Identifier: MIT
/* The display loop never waits for RKNN; this worker accepts only the latest job. */

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
        printf("{\"type\":\"%s\",\"score\":%.3f,\"decision\":\"%s\",\"reason\":\"%s\",\"box\":[%.1f,%.1f,%.1f,%.1f],\"ground\":{\"road\":%.4f,\"sidewalk\":%.4f,\"zebra\":%.4f}}",
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
    if (detection_count < 0)
        return -1;
    cplus_assign_riders(detections, detection_count, state->width, state->height,
                        &state->config);
    ordinary_count = ordinary_people_count(detections, detection_count);
    if (ordinary_count || state->always_segment) {
        cplus_prepare_segmenter_rgb(state->rgb, state->width, state->height,
                                    state->segmenter_rgb);
        if (cplus_rknn_infer_rgb(state->segmenter, state->segmenter_rgb,
                                 CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT,
                                 &output, &output_count) < 0) {
            return -1;
        }
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
    if (result->count < 0)
        return -1;
    result->valid = true;
    fprintf(stderr, "[infer] source_frame=%llu targets=%d ordinary_pedestrians=%d%s\n",
            (unsigned long long)source_frame, result->count, ordinary_count,
            (ordinary_count || state->always_segment) ? " segmented" : "");
    return 0;
}

static void *infer_thread_main(void *argument)
{
    struct cplus_async_infer *state = argument;

    for (;;) {
        struct cplus_async_result result;
        int slot;
        uint64_t source_frame;
        int inference_status;

        pthread_mutex_lock(&state->lock);
        while (state->running && state->pending_slot < 0)
            pthread_cond_wait(&state->pending_cond, &state->lock);
        if (!state->running) {
            if (state->pending_slot >= 0)
                state->slot_state[state->pending_slot] = CPLUS_ASYNC_SLOT_FREE;
            state->pending_slot = -1;
            pthread_cond_broadcast(&state->idle_cond);
            pthread_mutex_unlock(&state->lock);
            break;
        }
        slot = state->pending_slot;
        source_frame = state->slot_frame[slot];
        state->pending_slot = -1;
        state->slot_state[slot] = CPLUS_ASYNC_SLOT_WORKING;
        state->busy = true;
        pthread_mutex_unlock(&state->lock);

        inference_status = infer_frame(state, state->slots[slot], source_frame, &result);

        if (inference_status < 0) {
            pthread_mutex_lock(&state->lock);
            state->slot_state[slot] = CPLUS_ASYNC_SLOT_FREE;
            state->busy = false;
            state->failed = true;
            state->running = false;
            pthread_cond_broadcast(&state->idle_cond);
            pthread_mutex_unlock(&state->lock);
            fprintf(stderr, "[infer] source_frame=%llu failed\n",
                    (unsigned long long)source_frame);
            break;
        }
        pthread_mutex_lock(&state->lock);
        ++state->completed;
        result.sequence = state->completed;
        pthread_mutex_unlock(&state->lock);
        pthread_mutex_lock(&state->result_lock);
        state->result = result;
        pthread_mutex_unlock(&state->result_lock);
        print_result_json(&result);
        pthread_mutex_lock(&state->lock);
        state->slot_state[slot] = CPLUS_ASYNC_SLOT_FREE;
        state->busy = false;
        pthread_cond_broadcast(&state->idle_cond);
        pthread_mutex_unlock(&state->lock);
    }
    return NULL;
}

int cplus_async_infer_start(struct cplus_async_infer *state, int width, int height,
                            struct cplus_rknn_model *detector,
                            struct cplus_rknn_model *segmenter,
                            const struct cplus_runtime_config *config,
                            bool always_segment)
{
    int index;
    if (!state || !detector || !segmenter || !config || width <= 0 || height <= 0)
        return -1;
    memset(state, 0, sizeof(*state));
    state->width = width;
    state->height = height;
    state->frame_size = (size_t)width * height * 4U;
    state->detector = detector;
    state->segmenter = segmenter;
    state->config = *config;
    state->always_segment = always_segment;
    state->pending_slot = -1;
    for (index = 0; index < CPLUS_ASYNC_SLOTS; ++index) {
        state->slots[index] = malloc(state->frame_size);
        if (!state->slots[index])
            goto failed;
    }
    state->rgb = malloc((size_t)width * height * 3U);
    state->detector_rgb = malloc((size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT * 3U);
    state->segmenter_rgb = malloc((size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT * 3U);
    state->mask = malloc((size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT);
    if (!state->rgb || !state->detector_rgb || !state->segmenter_rgb || !state->mask)
        goto failed;
    if (pthread_mutex_init(&state->lock, NULL) != 0 ||
        pthread_mutex_init(&state->result_lock, NULL) != 0 ||
        pthread_cond_init(&state->pending_cond, NULL) != 0 ||
        pthread_cond_init(&state->idle_cond, NULL) != 0)
        goto failed;
    state->running = true;
    if (pthread_create(&state->thread, NULL, infer_thread_main, state) != 0)
        goto failed_sync;
    state->thread_started = true;
    return 0;

failed_sync:
    state->running = false;
    pthread_cond_destroy(&state->idle_cond);
    pthread_cond_destroy(&state->pending_cond);
    pthread_mutex_destroy(&state->result_lock);
    pthread_mutex_destroy(&state->lock);
failed:
    for (index = 0; index < CPLUS_ASYNC_SLOTS; ++index)
        free(state->slots[index]);
    free(state->rgb);
    free(state->detector_rgb);
    free(state->segmenter_rgb);
    free(state->mask);
    memset(state, 0, sizeof(*state));
    state->pending_slot = -1;
    return -1;
}

void cplus_async_infer_stop(struct cplus_async_infer *state)
{
    int index;
    if (!state) return;
    if (state->thread_started) {
        pthread_mutex_lock(&state->lock);
        state->running = false;
        pthread_cond_broadcast(&state->pending_cond);
        pthread_mutex_unlock(&state->lock);
        pthread_join(state->thread, NULL);
        pthread_cond_destroy(&state->idle_cond);
        pthread_cond_destroy(&state->pending_cond);
        pthread_mutex_destroy(&state->result_lock);
        pthread_mutex_destroy(&state->lock);
    }
    for (index = 0; index < CPLUS_ASYNC_SLOTS; ++index)
        free(state->slots[index]);
    free(state->rgb);
    free(state->detector_rgb);
    free(state->segmenter_rgb);
    free(state->mask);
    memset(state, 0, sizeof(*state));
    state->pending_slot = -1;
}

uint8_t *cplus_async_acquire_frame(struct cplus_async_infer *state, int *slot)
{
    int index;
    if (!state || !slot) return NULL;
    pthread_mutex_lock(&state->lock);
    if (!state->running || state->failed) {
        pthread_mutex_unlock(&state->lock);
        return NULL;
    }
    for (index = 0; index < CPLUS_ASYNC_SLOTS; ++index) {
        if (state->slot_state[index] == CPLUS_ASYNC_SLOT_FREE) {
            state->slot_state[index] = CPLUS_ASYNC_SLOT_FILLING;
            *slot = index;
            pthread_mutex_unlock(&state->lock);
            return state->slots[index];
        }
    }
    pthread_mutex_unlock(&state->lock);
    return NULL;
}

void cplus_async_discard_frame(struct cplus_async_infer *state, int slot)
{
    if (!state || slot < 0 || slot >= CPLUS_ASYNC_SLOTS) return;
    pthread_mutex_lock(&state->lock);
    if (state->slot_state[slot] == CPLUS_ASYNC_SLOT_FILLING)
        state->slot_state[slot] = CPLUS_ASYNC_SLOT_FREE;
    pthread_mutex_unlock(&state->lock);
}

int cplus_async_submit_frame(struct cplus_async_infer *state, int slot, uint64_t source_frame)
{
    int result = 0;
    if (!state || slot < 0 || slot >= CPLUS_ASYNC_SLOTS) return -1;
    pthread_mutex_lock(&state->lock);
    if (!state->running || state->failed || state->slot_state[slot] != CPLUS_ASYNC_SLOT_FILLING) {
        pthread_mutex_unlock(&state->lock);
        return -1;
    }
    if (state->pending_slot >= 0) {
        state->slot_state[slot] = CPLUS_ASYNC_SLOT_FREE;
        ++state->dropped;
        result = 1;
    } else {
        state->slot_frame[slot] = source_frame;
        state->slot_state[slot] = CPLUS_ASYNC_SLOT_PENDING;
        state->pending_slot = slot;
        ++state->submitted;
        pthread_cond_signal(&state->pending_cond);
    }
    pthread_mutex_unlock(&state->lock);
    return result;
}

bool cplus_async_get_result(struct cplus_async_infer *state, struct cplus_async_result *result)
{
    bool valid;
    if (!state || !result) return false;
    pthread_mutex_lock(&state->result_lock);
    *result = state->result;
    valid = result->valid;
    pthread_mutex_unlock(&state->result_lock);
    return valid;
}

int cplus_async_wait_idle(struct cplus_async_infer *state)
{
    if (!state) return -1;
    pthread_mutex_lock(&state->lock);
    while (!state->failed && (state->busy || state->pending_slot >= 0))
        pthread_cond_wait(&state->idle_cond, &state->lock);
    pthread_mutex_unlock(&state->lock);
    return state->failed ? -1 : 0;
}

bool cplus_async_failed(struct cplus_async_infer *state)
{
    bool failed;
    if (!state || !state->thread_started) return true;
    pthread_mutex_lock(&state->lock);
    failed = state->failed;
    pthread_mutex_unlock(&state->lock);
    return failed;
}

void cplus_async_get_stats(struct cplus_async_infer *state, struct cplus_async_stats *stats)
{
    if (!stats) return;
    memset(stats, 0, sizeof(*stats));
    if (!state || !state->thread_started) return;
    pthread_mutex_lock(&state->lock);
    stats->submitted = state->submitted;
    stats->dropped = state->dropped;
    stats->completed = state->completed;
    pthread_mutex_unlock(&state->lock);
}
