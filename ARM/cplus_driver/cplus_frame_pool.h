// SPDX-License-Identifier: MIT

#ifndef CPLUS_DRIVER_FRAME_POOL_H
#define CPLUS_DRIVER_FRAME_POOL_H

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>

#define CPLUS_FRAME_POOL_SLOTS 6

struct cplus_frame_slot {
    uint8_t *data;
    unsigned int references;
    uint64_t generation;
};

struct cplus_frame_pool {
    pthread_mutex_t lock;
    int lock_initialized;
    size_t frame_size;
    struct cplus_frame_slot slots[CPLUS_FRAME_POOL_SLOTS];
};

int cplus_frame_pool_init(struct cplus_frame_pool *pool, size_t frame_size);
int cplus_frame_pool_destroy(struct cplus_frame_pool *pool);
int cplus_frame_pool_acquire(struct cplus_frame_pool *pool, int *slot, uint8_t **data);
int cplus_frame_pool_addref(struct cplus_frame_pool *pool, int slot);
int cplus_frame_pool_release(struct cplus_frame_pool *pool, int slot);
uint8_t *cplus_frame_pool_data(struct cplus_frame_pool *pool, int slot);
uint64_t cplus_frame_pool_generation(struct cplus_frame_pool *pool, int slot);
unsigned int cplus_frame_pool_references(struct cplus_frame_pool *pool, int slot);

#endif /* CPLUS_DRIVER_FRAME_POOL_H */
