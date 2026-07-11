// SPDX-License-Identifier: MIT

#ifndef CPLUS_DRIVER_LATEST_QUEUE_H
#define CPLUS_DRIVER_LATEST_QUEUE_H

#include <pthread.h>
#include <stdint.h>

#include "cplus_frame_pool.h"

struct cplus_latest_queue {
    pthread_mutex_t lock;
    pthread_cond_t cond;
    int initialized;
    int running;
    int pending_slot;
    uint64_t submitted;
    uint64_t replaced;
    struct cplus_frame_pool *pool;
};

int cplus_latest_queue_init(struct cplus_latest_queue *queue,
                            struct cplus_frame_pool *pool);
void cplus_latest_queue_stop(struct cplus_latest_queue *queue);
void cplus_latest_queue_destroy(struct cplus_latest_queue *queue);
int cplus_latest_queue_submit(struct cplus_latest_queue *queue, int slot);
int cplus_latest_queue_take(struct cplus_latest_queue *queue, int *slot);
void cplus_latest_queue_stats(struct cplus_latest_queue *queue,
                              uint64_t *submitted, uint64_t *replaced);

#endif /* CPLUS_DRIVER_LATEST_QUEUE_H */
