// SPDX-License-Identifier: MIT

#include "cplus_frame_pool.h"

#include <stdlib.h>
#include <string.h>

static int valid_slot(int slot)
{
    return slot >= 0 && slot < CPLUS_FRAME_POOL_SLOTS;
}

int cplus_frame_pool_init(struct cplus_frame_pool *pool, size_t frame_size)
{
    int index;
    if (!pool || frame_size == 0) return -1;
    memset(pool, 0, sizeof(*pool));
    pool->frame_size = frame_size;
    for (index = 0; index < CPLUS_FRAME_POOL_SLOTS; ++index) {
        pool->slots[index].data = malloc(frame_size);
        if (!pool->slots[index].data) goto failed;
    }
    if (pthread_mutex_init(&pool->lock, NULL) != 0) goto failed;
    pool->lock_initialized = 1;
    return 0;

failed:
    for (index = 0; index < CPLUS_FRAME_POOL_SLOTS; ++index) {
        free(pool->slots[index].data);
        pool->slots[index].data = NULL;
    }
    memset(pool, 0, sizeof(*pool));
    return -1;
}

int cplus_frame_pool_destroy(struct cplus_frame_pool *pool)
{
    int index;
    int busy = 0;
    if (!pool) return -1;
    if (pool->lock_initialized) {
        pthread_mutex_lock(&pool->lock);
        for (index = 0; index < CPLUS_FRAME_POOL_SLOTS; ++index) {
            if (pool->slots[index].references != 0) busy = 1;
        }
        pthread_mutex_unlock(&pool->lock);
        if (busy) return -1;
        pthread_mutex_destroy(&pool->lock);
    }
    for (index = 0; index < CPLUS_FRAME_POOL_SLOTS; ++index)
        free(pool->slots[index].data);
    memset(pool, 0, sizeof(*pool));
    return 0;
}

int cplus_frame_pool_acquire(struct cplus_frame_pool *pool, int *slot, uint8_t **data)
{
    int index;
    if (!pool || !pool->lock_initialized || !slot || !data) return -1;
    pthread_mutex_lock(&pool->lock);
    for (index = 0; index < CPLUS_FRAME_POOL_SLOTS; ++index) {
        if (pool->slots[index].references == 0) {
            pool->slots[index].references = 1;
            ++pool->slots[index].generation;
            *slot = index;
            *data = pool->slots[index].data;
            pthread_mutex_unlock(&pool->lock);
            return 0;
        }
    }
    pthread_mutex_unlock(&pool->lock);
    return 1;
}

int cplus_frame_pool_addref(struct cplus_frame_pool *pool, int slot)
{
    if (!pool || !pool->lock_initialized || !valid_slot(slot)) return -1;
    pthread_mutex_lock(&pool->lock);
    if (pool->slots[slot].references == 0) {
        pthread_mutex_unlock(&pool->lock);
        return -1;
    }
    ++pool->slots[slot].references;
    pthread_mutex_unlock(&pool->lock);
    return 0;
}

int cplus_frame_pool_release(struct cplus_frame_pool *pool, int slot)
{
    if (!pool || !pool->lock_initialized || !valid_slot(slot)) return -1;
    pthread_mutex_lock(&pool->lock);
    if (pool->slots[slot].references == 0) {
        pthread_mutex_unlock(&pool->lock);
        return -1;
    }
    --pool->slots[slot].references;
    pthread_mutex_unlock(&pool->lock);
    return 0;
}

uint8_t *cplus_frame_pool_data(struct cplus_frame_pool *pool, int slot)
{
    if (!pool || !valid_slot(slot)) return NULL;
    return pool->slots[slot].data;
}

uint64_t cplus_frame_pool_generation(struct cplus_frame_pool *pool, int slot)
{
    uint64_t generation = 0;
    if (!pool || !pool->lock_initialized || !valid_slot(slot)) return 0;
    pthread_mutex_lock(&pool->lock);
    generation = pool->slots[slot].generation;
    pthread_mutex_unlock(&pool->lock);
    return generation;
}

unsigned int cplus_frame_pool_references(struct cplus_frame_pool *pool, int slot)
{
    unsigned int references = 0;
    if (!pool || !pool->lock_initialized || !valid_slot(slot)) return 0;
    pthread_mutex_lock(&pool->lock);
    references = pool->slots[slot].references;
    pthread_mutex_unlock(&pool->lock);
    return references;
}
