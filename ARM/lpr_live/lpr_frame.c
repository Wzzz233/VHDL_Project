// SPDX-License-Identifier: GPL-2.0
/* Generic fixed-slot frame ownership and BGRx image helpers. */

#include "lpr_frame.h"

#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

struct lpr_frame_pool_slot {
    uint8_t *data;
    size_t capacity;
    uint64_t generation;
    unsigned int refs;
    bool filling;
    bool owned;
    struct lpr_frame_meta meta;
};

static void clear_writer(struct lpr_frame_writer *writer)
{
    if (writer)
        memset(writer, 0, sizeof(*writer));
}

static void clear_ref(struct lpr_frame_ref *ref)
{
    if (ref)
        memset(ref, 0, sizeof(*ref));
}

static int init_pool(struct lpr_frame_pool *pool, size_t slot_count)
{
    int rc;

    if (!pool || slot_count == 0 || slot_count > UINT32_MAX)
        return -EINVAL;
    memset(pool, 0, sizeof(*pool));
    pool->slots = calloc(slot_count, sizeof(*pool->slots));
    if (!pool->slots)
        return -ENOMEM;
    rc = pthread_mutex_init(&pool->lock, NULL);
    if (rc != 0) {
        free(pool->slots);
        memset(pool, 0, sizeof(*pool));
        return -rc;
    }
    rc = pthread_cond_init(&pool->cond, NULL);
    if (rc != 0) {
        pthread_mutex_destroy(&pool->lock);
        free(pool->slots);
        memset(pool, 0, sizeof(*pool));
        return -rc;
    }
    pool->slot_count = slot_count;
    pool->initialized = true;
    return 0;
}

int lpr_frame_pool_init_heap(struct lpr_frame_pool *pool,
                             size_t slot_count, size_t slot_capacity)
{
    int rc;

    if (slot_capacity == 0)
        return -EINVAL;
    rc = init_pool(pool, slot_count);
    if (rc < 0)
        return rc;
    for (size_t i = 0; i < slot_count; i++) {
        pool->slots[i].data = malloc(slot_capacity);
        if (!pool->slots[i].data) {
            for (size_t j = 0; j < i; j++)
                free(pool->slots[j].data);
            pthread_cond_destroy(&pool->cond);
            pthread_mutex_destroy(&pool->lock);
            free(pool->slots);
            memset(pool, 0, sizeof(*pool));
            return -ENOMEM;
        }
        pool->slots[i].capacity = slot_capacity;
        pool->slots[i].owned = true;
    }
    return 0;
}

int lpr_frame_pool_init_external(struct lpr_frame_pool *pool,
                                 uint8_t *const *buffers,
                                 const size_t *capacities,
                                 size_t slot_count)
{
    int rc;

    if (!buffers || !capacities || slot_count == 0)
        return -EINVAL;
    for (size_t i = 0; i < slot_count; i++) {
        if (!buffers[i] || capacities[i] == 0)
            return -EINVAL;
    }
    rc = init_pool(pool, slot_count);
    if (rc < 0)
        return rc;
    for (size_t i = 0; i < slot_count; i++) {
        pool->slots[i].data = buffers[i];
        pool->slots[i].capacity = capacities[i];
        pool->slots[i].owned = false;
    }
    return 0;
}

void lpr_frame_pool_shutdown(struct lpr_frame_pool *pool)
{
    if (!pool || !pool->initialized)
        return;
    pthread_mutex_lock(&pool->lock);
    pool->stopping = true;
    pthread_cond_broadcast(&pool->cond);
    pthread_mutex_unlock(&pool->lock);
}

int lpr_frame_pool_destroy(struct lpr_frame_pool *pool)
{
    if (!pool || !pool->initialized)
        return -EINVAL;

    pthread_mutex_lock(&pool->lock);
    for (size_t i = 0; i < pool->slot_count; i++) {
        if (pool->slots[i].refs != 0 || pool->slots[i].filling) {
            pthread_mutex_unlock(&pool->lock);
            return -EBUSY;
        }
    }
    pthread_mutex_unlock(&pool->lock);

    for (size_t i = 0; i < pool->slot_count; i++) {
        if (pool->slots[i].owned)
            free(pool->slots[i].data);
    }
    pthread_cond_destroy(&pool->cond);
    pthread_mutex_destroy(&pool->lock);
    free(pool->slots);
    memset(pool, 0, sizeof(*pool));
    return 0;
}

static bool writer_matches_locked(const struct lpr_frame_writer *writer,
                                  struct lpr_frame_pool_slot **slot_out)
{
    struct lpr_frame_pool_slot *slot;

    if (!writer || !writer->pool || !writer->pool->initialized ||
        writer->slot >= writer->pool->slot_count)
        return false;
    slot = &writer->pool->slots[writer->slot];
    if (!slot->filling || slot->refs != 1 ||
        slot->generation != writer->slot_generation)
        return false;
    if (slot_out)
        *slot_out = slot;
    return true;
}

static bool ref_matches_locked(const struct lpr_frame_ref *ref,
                               struct lpr_frame_pool_slot **slot_out)
{
    struct lpr_frame_pool_slot *slot;

    if (!ref || !ref->pool || !ref->pool->initialized ||
        ref->slot >= ref->pool->slot_count)
        return false;
    slot = &ref->pool->slots[ref->slot];
    if (slot->filling || slot->refs == 0 ||
        slot->generation != ref->slot_generation)
        return false;
    if (slot_out)
        *slot_out = slot;
    return true;
}

int lpr_frame_pool_acquire(struct lpr_frame_pool *pool,
                           struct lpr_frame_writer *writer, bool block)
{
    size_t selected;

    if (!pool || !writer || !pool->initialized)
        return -EINVAL;
    clear_writer(writer);
    pthread_mutex_lock(&pool->lock);
    for (;;) {
        selected = SIZE_MAX;
        if (pool->stopping) {
            pthread_mutex_unlock(&pool->lock);
            return -ESHUTDOWN;
        }
        for (size_t i = 0; i < pool->slot_count; i++) {
            if (pool->slots[i].refs == 0 && !pool->slots[i].filling) {
                selected = i;
                break;
            }
        }
        if (selected != SIZE_MAX)
            break;
        if (!block) {
            pthread_mutex_unlock(&pool->lock);
            return -EAGAIN;
        }
        pthread_cond_wait(&pool->cond, &pool->lock);
    }

    pool->slots[selected].generation++;
    if (pool->slots[selected].generation == 0)
        pool->slots[selected].generation = 1;
    pool->slots[selected].refs = 1;
    pool->slots[selected].filling = true;
    memset(&pool->slots[selected].meta, 0,
           sizeof(pool->slots[selected].meta));
    writer->pool = pool;
    writer->slot = (uint32_t)selected;
    writer->slot_generation = pool->slots[selected].generation;
    pthread_mutex_unlock(&pool->lock);
    return 0;
}

uint8_t *lpr_frame_writer_data(const struct lpr_frame_writer *writer)
{
    struct lpr_frame_pool_slot *slot = NULL;
    uint8_t *data = NULL;

    if (!writer || !writer->pool || !writer->pool->initialized)
        return NULL;
    pthread_mutex_lock(&writer->pool->lock);
    if (writer_matches_locked(writer, &slot))
        data = slot->data;
    pthread_mutex_unlock(&writer->pool->lock);
    return data;
}

size_t lpr_frame_writer_capacity(const struct lpr_frame_writer *writer)
{
    struct lpr_frame_pool_slot *slot = NULL;
    size_t capacity = 0;

    if (!writer || !writer->pool || !writer->pool->initialized)
        return 0;
    pthread_mutex_lock(&writer->pool->lock);
    if (writer_matches_locked(writer, &slot))
        capacity = slot->capacity;
    pthread_mutex_unlock(&writer->pool->lock);
    return capacity;
}

static bool meta_fits(const struct lpr_frame_meta *meta, size_t capacity)
{
    uint64_t row_bytes;
    uint64_t required;

    if (!meta || meta->format != LPR_FRAME_FORMAT_BGRX8888 ||
        meta->width == 0 || meta->height == 0)
        return false;
    row_bytes = (uint64_t)meta->width * 4U;
    if ((uint64_t)meta->stride < row_bytes)
        return false;
    required = (uint64_t)(meta->height - 1U) * meta->stride + row_bytes;
    return required <= capacity;
}

int lpr_frame_writer_publish(struct lpr_frame_writer *writer,
                             const struct lpr_frame_meta *meta,
                             struct lpr_frame_ref *out)
{
    struct lpr_frame_pool *pool;
    struct lpr_frame_pool_slot *slot = NULL;

    if (!writer || !writer->pool || !out)
        return -EINVAL;
    pool = writer->pool;
    pthread_mutex_lock(&pool->lock);
    if (!writer_matches_locked(writer, &slot)) {
        pthread_mutex_unlock(&pool->lock);
        return -ESTALE;
    }
    if (!meta_fits(meta, slot->capacity)) {
        pthread_mutex_unlock(&pool->lock);
        return -EINVAL;
    }
    slot->meta = *meta;
    slot->filling = false;
    clear_ref(out);
    out->pool = pool;
    out->slot = writer->slot;
    out->slot_generation = writer->slot_generation;
    out->meta = *meta;
    clear_writer(writer);
    pthread_mutex_unlock(&pool->lock);
    return 0;
}

int lpr_frame_writer_abort(struct lpr_frame_writer *writer)
{
    struct lpr_frame_pool *pool;
    struct lpr_frame_pool_slot *slot = NULL;

    if (!writer || !writer->pool)
        return -EINVAL;
    pool = writer->pool;
    pthread_mutex_lock(&pool->lock);
    if (!writer_matches_locked(writer, &slot)) {
        pthread_mutex_unlock(&pool->lock);
        clear_writer(writer);
        return -ESTALE;
    }
    slot->filling = false;
    slot->refs = 0;
    memset(&slot->meta, 0, sizeof(slot->meta));
    pthread_cond_signal(&pool->cond);
    pthread_mutex_unlock(&pool->lock);
    clear_writer(writer);
    return 0;
}

int lpr_frame_ref_clone(const struct lpr_frame_ref *src,
                        struct lpr_frame_ref *out)
{
    struct lpr_frame_pool_slot *slot = NULL;

    if (!src || !src->pool || !out || src == out)
        return -EINVAL;
    pthread_mutex_lock(&src->pool->lock);
    if (!ref_matches_locked(src, &slot)) {
        pthread_mutex_unlock(&src->pool->lock);
        clear_ref(out);
        return -ESTALE;
    }
    if (slot->refs == UINT_MAX) {
        pthread_mutex_unlock(&src->pool->lock);
        clear_ref(out);
        return -EOVERFLOW;
    }
    slot->refs++;
    clear_ref(out);
    out->pool = src->pool;
    out->slot = src->slot;
    out->slot_generation = src->slot_generation;
    out->meta = src->meta;
    pthread_mutex_unlock(&src->pool->lock);
    return 0;
}

int lpr_frame_ref_clone_rebind_source_generation(
    const struct lpr_frame_ref *src, uint64_t source_generation,
    struct lpr_frame_ref *out)
{
    int rc;

    if (source_generation == 0)
        return -EINVAL;
    rc = lpr_frame_ref_clone(src, out);

    if (rc == 0)
        out->meta.source_generation = source_generation;
    return rc;
}

int lpr_frame_ref_release(struct lpr_frame_ref *ref)
{
    struct lpr_frame_pool *pool;
    struct lpr_frame_pool_slot *slot = NULL;

    if (!ref || !ref->pool)
        return -EINVAL;
    pool = ref->pool;
    pthread_mutex_lock(&pool->lock);
    if (!ref_matches_locked(ref, &slot)) {
        pthread_mutex_unlock(&pool->lock);
        clear_ref(ref);
        return -ESTALE;
    }
    slot->refs--;
    if (slot->refs == 0)
        pthread_cond_signal(&pool->cond);
    pthread_mutex_unlock(&pool->lock);
    clear_ref(ref);
    return 0;
}

const uint8_t *lpr_frame_ref_data(const struct lpr_frame_ref *ref)
{
    struct lpr_frame_pool_slot *slot = NULL;
    const uint8_t *data = NULL;

    if (!ref || !ref->pool || !ref->pool->initialized)
        return NULL;
    pthread_mutex_lock(&ref->pool->lock);
    if (ref_matches_locked(ref, &slot))
        data = slot->data;
    pthread_mutex_unlock(&ref->pool->lock);
    return data;
}

size_t lpr_frame_ref_capacity(const struct lpr_frame_ref *ref)
{
    struct lpr_frame_pool_slot *slot = NULL;
    size_t capacity = 0;

    if (!ref || !ref->pool || !ref->pool->initialized)
        return 0;
    pthread_mutex_lock(&ref->pool->lock);
    if (ref_matches_locked(ref, &slot))
        capacity = slot->capacity;
    pthread_mutex_unlock(&ref->pool->lock);
    return capacity;
}

bool lpr_frame_ref_is_valid(const struct lpr_frame_ref *ref)
{
    bool valid;

    if (!ref || !ref->pool || !ref->pool->initialized)
        return false;
    pthread_mutex_lock(&ref->pool->lock);
    valid = ref_matches_locked(ref, NULL);
    pthread_mutex_unlock(&ref->pool->lock);
    return valid;
}

bool lpr_frame_ref_matches_source_generation(const struct lpr_frame_ref *ref,
                                             uint64_t source_generation)
{
    return source_generation != 0 && lpr_frame_ref_is_valid(ref) &&
           ref->meta.source_generation == source_generation;
}

int lpr_bgrx_letterbox_copy(const uint8_t *src,
                            uint32_t src_width, uint32_t src_height,
                            uint32_t src_stride,
                            uint8_t *dst,
                            uint32_t dst_width, uint32_t dst_height,
                            uint32_t dst_stride,
                            struct lpr_letterbox_layout *layout)
{
    uint32_t scaled_width;
    uint32_t scaled_height;
    uint32_t pad_left;
    uint32_t pad_top;
    uint64_t src_row_bytes = (uint64_t)src_width * 4U;
    uint64_t dst_row_bytes = (uint64_t)dst_width * 4U;

    if (!src || !dst || src_width == 0 || src_height == 0 ||
        dst_width == 0 || dst_height == 0 ||
        src_row_bytes > UINT32_MAX || dst_row_bytes > UINT32_MAX ||
        src_stride < src_row_bytes || dst_stride < dst_row_bytes)
        return -EINVAL;

    if ((uint64_t)dst_width * src_height <=
        (uint64_t)dst_height * src_width) {
        scaled_width = dst_width;
        scaled_height = (uint32_t)(((uint64_t)src_height * dst_width +
                                    src_width / 2U) / src_width);
        if (scaled_height == 0)
            scaled_height = 1;
        if (scaled_height > dst_height)
            scaled_height = dst_height;
    } else {
        scaled_height = dst_height;
        scaled_width = (uint32_t)(((uint64_t)src_width * dst_height +
                                   src_height / 2U) / src_height);
        if (scaled_width == 0)
            scaled_width = 1;
        if (scaled_width > dst_width)
            scaled_width = dst_width;
    }
    pad_left = (dst_width - scaled_width) / 2U;
    pad_top = (dst_height - scaled_height) / 2U;

    if (src == dst && src_width == dst_width && src_height == dst_height &&
        src_stride == dst_stride) {
        if (layout) {
            layout->scaled_width = scaled_width;
            layout->scaled_height = scaled_height;
            layout->pad_left = pad_left;
            layout->pad_top = pad_top;
        }
        return 0;
    }

    for (uint32_t y = 0; y < dst_height; y++) {
        uint8_t *row = dst + (size_t)y * dst_stride;
        memset(row, 0, dst_stride);
        for (uint32_t x = 0; x < dst_width; x++)
            row[(size_t)x * 4U + 3U] = 0xff;
    }
    for (uint32_t y = 0; y < scaled_height; y++) {
        uint32_t src_y = (uint32_t)((uint64_t)y * src_height / scaled_height);
        const uint8_t *src_row = src + (size_t)src_y * src_stride;
        uint8_t *dst_row = dst + (size_t)(y + pad_top) * dst_stride +
                           (size_t)pad_left * 4U;

        for (uint32_t x = 0; x < scaled_width; x++) {
            uint32_t src_x = (uint32_t)((uint64_t)x * src_width / scaled_width);
            memcpy(dst_row + (size_t)x * 4U,
                   src_row + (size_t)src_x * 4U, 4U);
        }
    }

    if (layout) {
        layout->scaled_width = scaled_width;
        layout->scaled_height = scaled_height;
        layout->pad_left = pad_left;
        layout->pad_top = pad_top;
    }
    return 0;
}
