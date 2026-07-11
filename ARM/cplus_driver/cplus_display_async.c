// SPDX-License-Identifier: MIT

#include "cplus_display_async.h"

#include <stdio.h>
#include <string.h>

static void *display_thread_main(void *argument)
{
    struct cplus_display_async *display = argument;
    int slot;
    int take_status;

    while ((take_status = cplus_latest_queue_take(&display->queue, &slot)) > 0) {
        const struct cplus_display_job *job = &display->jobs[slot];
        const uint8_t *frame = cplus_frame_pool_data(display->pool, slot);
        int present_status = cplus_display_present(&display->drm, frame,
                                                   job->result_available ? job->results : NULL,
                                                   job->result_available ? job->count : 0,
                                                   job->result_available,
                                                   job->mask_available ? job->mask : NULL,
                                                   job->mask_available);
        cplus_frame_pool_release(display->pool, slot);
        pthread_mutex_lock(&display->status_lock);
        if (present_status < 0) {
            display->failed = 1;
        } else {
            ++display->presented;
        }
        pthread_mutex_unlock(&display->status_lock);
        if (present_status < 0) {
            fprintf(stderr, "[display] asynchronous DRM update failed\n");
            cplus_latest_queue_stop(&display->queue);
            break;
        }
    }
    if (take_status < 0) {
        pthread_mutex_lock(&display->status_lock);
        display->failed = 1;
        pthread_mutex_unlock(&display->status_lock);
    }
    return NULL;
}

int cplus_display_async_start(struct cplus_display_async *display,
                              struct cplus_frame_pool *pool,
                              const char *drm_card, int connector_id,
                              int width, int height)
{
    if (!display || !pool) return -1;
    memset(display, 0, sizeof(*display));
    display->drm.fd = -1;
    display->drm.active_fb = -1;
    display->pool = pool;
    if (cplus_display_start(&display->drm, drm_card, connector_id, width, height) < 0)
        goto failed;
    if (pthread_mutex_init(&display->status_lock, NULL) != 0)
        goto failed;
    display->status_lock_initialized = 1;
    if (cplus_latest_queue_init(&display->queue, pool) < 0)
        goto failed;
    display->queue_initialized = 1;
    if (pthread_create(&display->thread, NULL, display_thread_main, display) != 0)
        goto failed;
    display->thread_started = 1;
    fprintf(stderr, "[display] asynchronous latest-frame thread started\n");
    return 0;

failed:
    cplus_display_async_stop(display);
    return -1;
}

int cplus_display_async_submit(struct cplus_display_async *display, int slot,
                               const struct cplus_person_result *results, int count,
                               bool result_available,
                               const uint8_t *mask, bool mask_available)
{
    struct cplus_display_job *job;
    if (!display || !display->thread_started || slot < 0 ||
        slot >= CPLUS_FRAME_POOL_SLOTS || count < 0 || count > CPLUS_MAX_DETECTIONS ||
        (result_available && count > 0 && !results) ||
        (mask_available && (!result_available || !mask)))
        return -1;
    if (cplus_display_async_failed(display)) return -1;
    job = &display->jobs[slot];
    job->count = result_available ? count : 0;
    job->result_available = result_available;
    job->mask_available = result_available && mask_available;
    if (job->count > 0 && results)
        memcpy(job->results, results, (size_t)job->count * sizeof(job->results[0]));
    if (job->mask_available) memcpy(job->mask, mask, CPLUS_MODEL_PIXELS);
    return cplus_latest_queue_submit(&display->queue, slot);
}

bool cplus_display_async_failed(struct cplus_display_async *display)
{
    int failed;
    if (!display || !display->status_lock_initialized) return true;
    pthread_mutex_lock(&display->status_lock);
    failed = display->failed;
    pthread_mutex_unlock(&display->status_lock);
    return failed != 0;
}

uint64_t cplus_display_async_stop(struct cplus_display_async *display)
{
    uint64_t presented;
    if (!display) return 0;
    if (display->queue_initialized) cplus_latest_queue_stop(&display->queue);
    if (display->thread_started) {
        pthread_join(display->thread, NULL);
        display->thread_started = 0;
    }
    if (display->queue_initialized) {
        cplus_latest_queue_destroy(&display->queue);
        display->queue_initialized = 0;
    }
    if (display->status_lock_initialized) {
        pthread_mutex_destroy(&display->status_lock);
        display->status_lock_initialized = 0;
    }
    cplus_display_stop(&display->drm);
    presented = display->presented;
    memset(display, 0, sizeof(*display));
    display->drm.fd = -1;
    display->drm.active_fb = -1;
    return presented;
}

void cplus_display_async_stats(struct cplus_display_async *display,
                               uint64_t *presented, uint64_t *replaced)
{
    uint64_t ignored_submitted;
    if (presented) *presented = 0;
    if (replaced) *replaced = 0;
    if (!display) return;
    if (display->status_lock_initialized) {
        pthread_mutex_lock(&display->status_lock);
        if (presented) *presented = display->presented;
        pthread_mutex_unlock(&display->status_lock);
    }
    if (display->queue_initialized)
        cplus_latest_queue_stats(&display->queue, &ignored_submitted, replaced);
}
