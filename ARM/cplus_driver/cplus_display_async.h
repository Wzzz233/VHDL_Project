// SPDX-License-Identifier: MIT

#ifndef CPLUS_DRIVER_DISPLAY_ASYNC_H
#define CPLUS_DRIVER_DISPLAY_ASYNC_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

#include "cplus_display.h"
#include "cplus_frame_pool.h"
#include "cplus_latest_queue.h"

struct cplus_display_job {
    struct cplus_person_result results[CPLUS_MAX_DETECTIONS];
    int count;
    bool result_available;
};

struct cplus_display_async {
    struct cplus_display drm;
    struct cplus_frame_pool *pool;
    struct cplus_latest_queue queue;
    struct cplus_display_job jobs[CPLUS_FRAME_POOL_SLOTS];
    pthread_t thread;
    pthread_mutex_t status_lock;
    int status_lock_initialized;
    int queue_initialized;
    int thread_started;
    int failed;
    uint64_t presented;
};

int cplus_display_async_start(struct cplus_display_async *display,
                              struct cplus_frame_pool *pool,
                              const char *drm_card, int connector_id,
                              int width, int height);
int cplus_display_async_submit(struct cplus_display_async *display, int slot,
                               const struct cplus_person_result *results, int count,
                               bool result_available);
bool cplus_display_async_failed(struct cplus_display_async *display);
void cplus_display_async_stop(struct cplus_display_async *display);
void cplus_display_async_stats(struct cplus_display_async *display,
                               uint64_t *presented, uint64_t *replaced);

#endif /* CPLUS_DRIVER_DISPLAY_ASYNC_H */
