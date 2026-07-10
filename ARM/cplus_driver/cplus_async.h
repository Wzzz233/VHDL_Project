// SPDX-License-Identifier: MIT
/* Latest-frame background RKNN inference for the C+ live driver. */

#ifndef CPLUS_DRIVER_ASYNC_H
#define CPLUS_DRIVER_ASYNC_H

#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cplus_core.h"
#include "cplus_rknn.h"

#define CPLUS_ASYNC_SLOTS 3

enum cplus_async_slot_state {
    CPLUS_ASYNC_SLOT_FREE = 0,
    CPLUS_ASYNC_SLOT_FILLING,
    CPLUS_ASYNC_SLOT_PENDING,
    CPLUS_ASYNC_SLOT_WORKING,
};

struct cplus_async_result {
    struct cplus_person_result results[CPLUS_MAX_DETECTIONS];
    int count;
    uint64_t sequence;
    uint64_t source_frame;
    bool valid;
};

struct cplus_async_stats {
    uint64_t submitted;
    uint64_t dropped;
    uint64_t completed;
};

struct cplus_async_infer {
    pthread_t thread;
    pthread_mutex_t lock;
    pthread_mutex_t result_lock;
    pthread_cond_t pending_cond;
    pthread_cond_t idle_cond;
    bool thread_started;
    bool running;
    bool busy;
    bool failed;
    int pending_slot;
    int width;
    int height;
    size_t frame_size;
    bool always_segment;
    struct cplus_runtime_config config;
    struct cplus_rknn_model *detector;
    struct cplus_rknn_model *segmenter;
    uint8_t *slots[CPLUS_ASYNC_SLOTS];
    enum cplus_async_slot_state slot_state[CPLUS_ASYNC_SLOTS];
    uint64_t slot_frame[CPLUS_ASYNC_SLOTS];
    uint8_t *rgb;
    uint8_t *detector_rgb;
    uint8_t *segmenter_rgb;
    uint8_t *mask;
    uint64_t submitted;
    uint64_t dropped;
    uint64_t completed;
    struct cplus_async_result result;
};

int cplus_async_infer_start(struct cplus_async_infer *state, int width, int height,
                            struct cplus_rknn_model *detector,
                            struct cplus_rknn_model *segmenter,
                            const struct cplus_runtime_config *config,
                            bool always_segment);
void cplus_async_infer_stop(struct cplus_async_infer *state);

/* The caller owns a filling slot until it either submits or discards it. */
uint8_t *cplus_async_acquire_frame(struct cplus_async_infer *state, int *slot);
void cplus_async_discard_frame(struct cplus_async_infer *state, int slot);

/* Returns 0 when queued, 1 when dropped because a newer job is already queued. */
int cplus_async_submit_frame(struct cplus_async_infer *state, int slot, uint64_t source_frame);
bool cplus_async_get_result(struct cplus_async_infer *state, struct cplus_async_result *result);
int cplus_async_wait_idle(struct cplus_async_infer *state);
bool cplus_async_failed(struct cplus_async_infer *state);
void cplus_async_get_stats(struct cplus_async_infer *state, struct cplus_async_stats *stats);

#endif /* CPLUS_DRIVER_ASYNC_H */
