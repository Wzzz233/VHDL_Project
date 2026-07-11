// SPDX-License-Identifier: MIT

#include "cplus_frame_pool.h"
#include "cplus_latest_queue.h"

#include <assert.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static void test_reference_lifetime(void)
{
    struct cplus_frame_pool pool;
    uint8_t *frame;
    int slot;

    assert(cplus_frame_pool_init(&pool, 64) == 0);
    assert(cplus_frame_pool_acquire(&pool, &slot, &frame) == 0);
    memset(frame, 0x5a, 64);
    assert(cplus_frame_pool_references(&pool, slot) == 1);
    assert(cplus_frame_pool_addref(&pool, slot) == 0);
    assert(cplus_frame_pool_references(&pool, slot) == 2);
    assert(cplus_frame_pool_release(&pool, slot) == 0);
    assert(cplus_frame_pool_release(&pool, slot) == 0);
    assert(cplus_frame_pool_release(&pool, slot) < 0);
    assert(cplus_frame_pool_destroy(&pool) == 0);
}

static void test_latest_replaces_pending(void)
{
    struct cplus_frame_pool pool;
    struct cplus_latest_queue queue;
    uint8_t *first;
    uint8_t *latest;
    uint64_t submitted;
    uint64_t replaced;
    int first_slot;
    int latest_slot;
    int taken_slot = -1;

    assert(cplus_frame_pool_init(&pool, 64) == 0);
    assert(cplus_latest_queue_init(&queue, &pool) == 0);
    assert(cplus_frame_pool_acquire(&pool, &first_slot, &first) == 0);
    assert(cplus_frame_pool_acquire(&pool, &latest_slot, &latest) == 0);
    first[0] = 1;
    latest[0] = 2;
    assert(cplus_latest_queue_submit(&queue, first_slot) == 0);
    assert(cplus_frame_pool_release(&pool, first_slot) == 0);
    assert(cplus_latest_queue_submit(&queue, latest_slot) == 0);
    assert(cplus_frame_pool_release(&pool, latest_slot) == 0);
    assert(cplus_frame_pool_references(&pool, first_slot) == 0);
    assert(cplus_latest_queue_take(&queue, &taken_slot) == 1);
    assert(taken_slot == latest_slot);
    assert(cplus_frame_pool_data(&pool, taken_slot)[0] == 2);
    cplus_latest_queue_stats(&queue, &submitted, &replaced);
    assert(submitted == 2);
    assert(replaced == 1);
    assert(cplus_frame_pool_release(&pool, taken_slot) == 0);
    cplus_latest_queue_stop(&queue);
    assert(cplus_latest_queue_take(&queue, &taken_slot) == 0);
    cplus_latest_queue_destroy(&queue);
    assert(cplus_frame_pool_destroy(&pool) == 0);
}

static void test_stop_drains_latest(void)
{
    struct cplus_frame_pool pool;
    struct cplus_latest_queue queue;
    uint8_t *frame;
    int slot;
    int taken_slot = -1;

    assert(cplus_frame_pool_init(&pool, 64) == 0);
    assert(cplus_latest_queue_init(&queue, &pool) == 0);
    assert(cplus_frame_pool_acquire(&pool, &slot, &frame) == 0);
    frame[0] = 7;
    assert(cplus_latest_queue_submit(&queue, slot) == 0);
    assert(cplus_frame_pool_release(&pool, slot) == 0);
    cplus_latest_queue_stop(&queue);
    assert(cplus_latest_queue_take(&queue, &taken_slot) == 1);
    assert(taken_slot == slot);
    assert(cplus_frame_pool_release(&pool, taken_slot) == 0);
    assert(cplus_latest_queue_take(&queue, &taken_slot) == 0);
    cplus_latest_queue_destroy(&queue);
    assert(cplus_frame_pool_destroy(&pool) == 0);
}

struct concurrent_context {
    struct cplus_frame_pool *pool;
    struct cplus_latest_queue *queue;
    uint32_t last_value;
    unsigned int consumed;
};

static void *consumer_main(void *argument)
{
    struct concurrent_context *context = argument;
    struct timespec delay = { .tv_sec = 0, .tv_nsec = 500000L };
    int slot;
    int status;

    while ((status = cplus_latest_queue_take(context->queue, &slot)) > 0) {
        memcpy(&context->last_value, cplus_frame_pool_data(context->pool, slot),
               sizeof(context->last_value));
        ++context->consumed;
        nanosleep(&delay, NULL);
        assert(cplus_frame_pool_release(context->pool, slot) == 0);
    }
    assert(status == 0);
    return NULL;
}

static void test_concurrent_latest_delivery(void)
{
    struct cplus_frame_pool pool;
    struct cplus_latest_queue queue;
    struct concurrent_context context;
    pthread_t consumer;
    uint64_t submitted;
    uint64_t replaced;
    uint32_t value;
    int index;

    assert(cplus_frame_pool_init(&pool, 64) == 0);
    assert(cplus_latest_queue_init(&queue, &pool) == 0);
    memset(&context, 0, sizeof(context));
    context.pool = &pool;
    context.queue = &queue;
    assert(pthread_create(&consumer, NULL, consumer_main, &context) == 0);
    for (value = 1; value <= 1000; ++value) {
        uint8_t *frame = NULL;
        int slot = -1;
        while (cplus_frame_pool_acquire(&pool, &slot, &frame) == 1)
            sched_yield();
        assert(frame != NULL);
        memcpy(frame, &value, sizeof(value));
        assert(cplus_latest_queue_submit(&queue, slot) == 0);
        assert(cplus_frame_pool_release(&pool, slot) == 0);
    }
    cplus_latest_queue_stop(&queue);
    assert(pthread_join(consumer, NULL) == 0);
    cplus_latest_queue_stats(&queue, &submitted, &replaced);
    assert(submitted == 1000);
    assert(replaced > 0);
    assert(context.consumed > 0);
    assert(context.last_value == 1000);
    for (index = 0; index < CPLUS_FRAME_POOL_SLOTS; ++index)
        assert(cplus_frame_pool_references(&pool, index) == 0);
    cplus_latest_queue_destroy(&queue);
    assert(cplus_frame_pool_destroy(&pool) == 0);
}

int main(void)
{
    test_reference_lifetime();
    test_latest_replaces_pending();
    test_stop_drains_latest();
    test_concurrent_latest_delivery();
    puts("cplus async primitive tests passed");
    return 0;
}
