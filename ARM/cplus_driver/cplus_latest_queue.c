// SPDX-License-Identifier: MIT

#include "cplus_latest_queue.h"

#include <string.h>

int cplus_latest_queue_init(struct cplus_latest_queue *queue,
                            struct cplus_frame_pool *pool)
{
    if (!queue || !pool || !pool->lock_initialized) return -1;
    memset(queue, 0, sizeof(*queue));
    queue->pool = pool;
    queue->pending_slot = -1;
    if (pthread_mutex_init(&queue->lock, NULL) != 0) return -1;
    if (pthread_cond_init(&queue->cond, NULL) != 0) {
        pthread_mutex_destroy(&queue->lock);
        return -1;
    }
    queue->initialized = 1;
    queue->running = 1;
    return 0;
}

void cplus_latest_queue_stop(struct cplus_latest_queue *queue)
{
    if (!queue || !queue->initialized) return;
    pthread_mutex_lock(&queue->lock);
    queue->running = 0;
    pthread_cond_broadcast(&queue->cond);
    pthread_mutex_unlock(&queue->lock);
}

void cplus_latest_queue_destroy(struct cplus_latest_queue *queue)
{
    int pending_slot;
    if (!queue || !queue->initialized) return;
    cplus_latest_queue_stop(queue);
    pthread_mutex_lock(&queue->lock);
    pending_slot = queue->pending_slot;
    queue->pending_slot = -1;
    pthread_mutex_unlock(&queue->lock);
    if (pending_slot >= 0) cplus_frame_pool_release(queue->pool, pending_slot);
    pthread_cond_destroy(&queue->cond);
    pthread_mutex_destroy(&queue->lock);
    memset(queue, 0, sizeof(*queue));
    queue->pending_slot = -1;
}

int cplus_latest_queue_submit(struct cplus_latest_queue *queue, int slot)
{
    int replaced_slot = -1;
    if (!queue || !queue->initialized ||
        cplus_frame_pool_addref(queue->pool, slot) < 0)
        return -1;
    pthread_mutex_lock(&queue->lock);
    if (!queue->running) {
        pthread_mutex_unlock(&queue->lock);
        cplus_frame_pool_release(queue->pool, slot);
        return -1;
    }
    if (queue->pending_slot >= 0) {
        replaced_slot = queue->pending_slot;
        ++queue->replaced;
    }
    queue->pending_slot = slot;
    ++queue->submitted;
    pthread_cond_signal(&queue->cond);
    pthread_mutex_unlock(&queue->lock);
    if (replaced_slot >= 0) cplus_frame_pool_release(queue->pool, replaced_slot);
    return 0;
}

int cplus_latest_queue_take(struct cplus_latest_queue *queue, int *slot)
{
    if (!queue || !queue->initialized || !slot) return -1;
    pthread_mutex_lock(&queue->lock);
    while (queue->running && queue->pending_slot < 0)
        pthread_cond_wait(&queue->cond, &queue->lock);
    if (queue->pending_slot < 0) {
        pthread_mutex_unlock(&queue->lock);
        return 0;
    }
    *slot = queue->pending_slot;
    queue->pending_slot = -1;
    pthread_mutex_unlock(&queue->lock);
    return 1;
}

void cplus_latest_queue_stats(struct cplus_latest_queue *queue,
                              uint64_t *submitted, uint64_t *replaced)
{
    if (submitted) *submitted = 0;
    if (replaced) *replaced = 0;
    if (!queue || !queue->initialized) return;
    pthread_mutex_lock(&queue->lock);
    if (submitted) *submitted = queue->submitted;
    if (replaced) *replaced = queue->replaced;
    pthread_mutex_unlock(&queue->lock);
}
