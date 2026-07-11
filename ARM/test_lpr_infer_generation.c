// SPDX-License-Identifier: GPL-2.0
/* Host race test for inference generation reset and result publication. */

#include "lpr_live/lpr_infer.h"

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define CHECK(expr) do {                                                     \
    if (!(expr)) {                                                           \
        fprintf(stderr, "[FAIL] %s:%d: %s\n", __func__, __LINE__, #expr); \
        return -1;                                                           \
    }                                                                        \
} while (0)

struct delayed_publish {
    struct infer_state *infer;
    struct live_result result;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    bool ready;
    bool release;
    bool accepted;
};

static void *publish_after_release(void *opaque)
{
    struct delayed_publish *publish = opaque;

    pthread_mutex_lock(&publish->lock);
    publish->ready = true;
    pthread_cond_signal(&publish->cond);
    while (!publish->release)
        pthread_cond_wait(&publish->cond, &publish->lock);
    pthread_mutex_unlock(&publish->lock);
    publish->accepted =
        lpr_infer_publish_result(publish->infer, &publish->result);
    return NULL;
}

static int init_infer_gate(struct infer_state *infer, uint64_t generation)
{
    int rc;

    memset(infer, 0, sizeof(*infer));
    rc = pthread_mutex_init(&infer->lock, NULL);
    if (rc != 0)
        return -1;
    rc = pthread_mutex_init(&infer->result_lock, NULL);
    if (rc != 0) {
        pthread_mutex_destroy(&infer->lock);
        return -1;
    }
    rc = pthread_mutex_init(&infer->epoch_lock, NULL);
    if (rc != 0) {
        pthread_mutex_destroy(&infer->result_lock);
        pthread_mutex_destroy(&infer->lock);
        return -1;
    }
    atomic_init(&infer->accepted_source_generation, generation);
    infer->result.frame_slot = -1;
    infer->result.source_generation = generation;
    return 0;
}

static void destroy_infer_gate(struct infer_state *infer)
{
    pthread_mutex_destroy(&infer->epoch_lock);
    pthread_mutex_destroy(&infer->result_lock);
    pthread_mutex_destroy(&infer->lock);
}

static int test_reset_rejects_delayed_old_result(void)
{
    struct infer_state infer;
    struct delayed_publish publish;
    struct live_result snapshot;
    struct live_result current;
    pthread_t thread;

    CHECK(init_infer_gate(&infer, 7) == 0);
    memset(&publish, 0, sizeof(publish));
    publish.infer = &infer;
    publish.result.valid = true;
    publish.result.seq = 17;
    publish.result.source_generation = 7;
    publish.result.frame_slot = -1;
    CHECK(pthread_mutex_init(&publish.lock, NULL) == 0);
    CHECK(pthread_cond_init(&publish.cond, NULL) == 0);
    CHECK(pthread_create(&thread, NULL, publish_after_release, &publish) == 0);

    pthread_mutex_lock(&publish.lock);
    while (!publish.ready)
        pthread_cond_wait(&publish.cond, &publish.lock);
    pthread_mutex_unlock(&publish.lock);

    lpr_infer_reset(&infer, 8);
    memset(&snapshot, 0, sizeof(snapshot));
    CHECK(!lpr_infer_get_result(&infer, &snapshot));
    CHECK(!snapshot.valid && snapshot.source_generation == 8);

    pthread_mutex_lock(&publish.lock);
    publish.release = true;
    pthread_cond_signal(&publish.cond);
    pthread_mutex_unlock(&publish.lock);
    CHECK(pthread_join(thread, NULL) == 0);
    CHECK(!publish.accepted);
    CHECK(!lpr_infer_get_result(&infer, &snapshot));
    CHECK(!snapshot.valid && snapshot.source_generation == 8);

    memset(&current, 0, sizeof(current));
    current.valid = true;
    current.seq = 18;
    current.source_generation = 8;
    current.frame_slot = -1;
    CHECK(lpr_infer_publish_result(&infer, &current));
    CHECK(lpr_infer_get_result(&infer, &snapshot));
    CHECK(snapshot.valid && snapshot.seq == 18 &&
          snapshot.source_generation == 8);
    CHECK(!lpr_infer_publish_result(NULL, &current));

    pthread_cond_destroy(&publish.cond);
    pthread_mutex_destroy(&publish.lock);
    destroy_infer_gate(&infer);
    return 0;
}

int main(void)
{
    if (test_reset_rejects_delayed_old_result() < 0)
        return 1;
    puts("[PASS] infer reset rejects delayed stale publication");
    return 0;
}
