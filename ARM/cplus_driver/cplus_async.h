// SPDX-License-Identifier: MIT
/* Latest-frame background RKNN inference for the C+ live driver. */

#ifndef CPLUS_DRIVER_ASYNC_H
#define CPLUS_DRIVER_ASYNC_H

#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cplus_core.h"
#include "cplus_frame_pool.h"
#include "cplus_latest_queue.h"
#include "cplus_rknn.h"

struct cplus_async_result {
    struct cplus_person_result results[CPLUS_MAX_DETECTIONS];
    int count;
    uint64_t sequence;
    uint64_t source_frame;
    uint64_t source_generation;
    bool mask_valid;
    bool mask_updated;
    bool fixed_mask_ready;
    bool valid;
    uint8_t mask[CPLUS_MODEL_PIXELS];
};

struct cplus_async_stats {
    uint64_t submitted;
    uint64_t replaced;
    uint64_t completed;
};

struct cplus_async_infer {
    pthread_t thread;
    pthread_mutex_t result_lock;
    pthread_cond_t result_cond;
    int result_sync_initialized;
    int queue_initialized;
    int thread_started;
    int failed;
    int width;
    int height;
    bool always_segment;
    bool fixed_mask_mode;
    bool mask_generate_requested;
    bool mask_generation_in_progress;
    bool fixed_mask_ready;
    uint64_t fixed_mask_sequence;
    uint64_t fixed_mask_source_generation;
    struct cplus_runtime_config config;
    struct cplus_rknn_model *detector;
    struct cplus_rknn_model *segmenter;
    struct cplus_frame_pool *pool;
    struct cplus_latest_queue queue;
    uint64_t slot_frame[CPLUS_FRAME_POOL_SLOTS];
    uint64_t slot_source_generation[CPLUS_FRAME_POOL_SLOTS];
    uint8_t *rgb;
    uint8_t *detector_rgb;
    uint8_t *segmenter_rgb;
    uint8_t *mask;
    uint8_t *fixed_mask;
    struct cplus_mask_workspace mask_workspace;
    int mask_workspace_initialized;
    bool mask_all_other_reported;
    uint64_t completed;
    struct cplus_async_result result;
};

int cplus_async_infer_start(struct cplus_async_infer *state,
                            struct cplus_frame_pool *pool,
                            int width, int height,
                            struct cplus_rknn_model *detector,
                            struct cplus_rknn_model *segmenter,
                            const struct cplus_runtime_config *config,
                            bool always_segment,
                            bool fixed_mask_mode);
uint64_t cplus_async_infer_stop(struct cplus_async_infer *state);
int cplus_async_submit_frame(struct cplus_async_infer *state, int slot,
                             uint64_t source_frame);
int cplus_async_submit_frame_epoch(struct cplus_async_infer *state, int slot,
                                   uint64_t source_frame,
                                   uint64_t source_generation);
int cplus_async_request_fixed_mask(struct cplus_async_infer *state);
int cplus_async_set_fixed_mask(struct cplus_async_infer *state,
                               const uint8_t *mask, size_t mask_size,
                               uint64_t source_generation);
void cplus_async_clear_fixed_mask(struct cplus_async_infer *state);
bool cplus_async_fixed_mask_status(struct cplus_async_infer *state,
                                   bool *generating,
                                   uint64_t *sequence,
                                   uint64_t *source_generation);
int cplus_async_copy_fixed_mask(struct cplus_async_infer *state,
                                uint8_t *mask, size_t mask_size,
                                uint64_t *sequence,
                                uint64_t *source_generation);
bool cplus_async_get_result(struct cplus_async_infer *state,
                            struct cplus_async_result *result);
bool cplus_async_refresh_result(struct cplus_async_infer *state,
                                struct cplus_async_result *result);
int cplus_async_wait_for_frame(struct cplus_async_infer *state,
                               uint64_t source_frame);
bool cplus_async_failed(struct cplus_async_infer *state);
void cplus_async_get_stats(struct cplus_async_infer *state,
                           struct cplus_async_stats *stats);

#endif /* CPLUS_DRIVER_ASYNC_H */
