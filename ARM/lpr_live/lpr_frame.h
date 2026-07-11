// SPDX-License-Identifier: GPL-2.0
/* Generic fixed-slot frame ownership and BGRx image helpers. */

#ifndef LPR_LIVE_LPR_FRAME_H
#define LPR_LIVE_LPR_FRAME_H

#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum lpr_frame_format {
    LPR_FRAME_FORMAT_BGRX8888 = 1,
};

enum lpr_frame_fpga_capability {
    LPR_FRAME_FPGA_CAP_DMA           = 1U << 0,
    LPR_FRAME_FPGA_CAP_FRAME_STATUS  = 1U << 1,
    LPR_FRAME_FPGA_CAP_CAMERA_STATUS = 1U << 2,
    LPR_FRAME_FPGA_CAP_FRAME_STAMP   = 1U << 3,
};

struct lpr_frame_meta {
    enum lpr_frame_format format;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    int64_t monotonic_us;
    uint64_t sequence;
    uint64_t source_generation;
    uint32_t fpga_caps;
};

struct lpr_frame_pool_slot;

struct lpr_frame_pool {
    struct lpr_frame_pool_slot *slots;
    size_t slot_count;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    bool initialized;
    bool stopping;
};

/* A writer is mutable and producer-only. Publishing consumes it and returns
 * an immutable frame reference for all downstream users. */
struct lpr_frame_writer {
    struct lpr_frame_pool *pool;
    uint32_t slot;
    uint64_t slot_generation;
};

struct lpr_frame_ref {
    struct lpr_frame_pool *pool;
    uint32_t slot;
    uint64_t slot_generation;
    struct lpr_frame_meta meta;
};

/* A frame_ref is an owning handle: do not create ownership with a struct
 * assignment. Use clone/release. Metadata is copied by value on clone, while
 * pixel storage remains shared and read-only. */

struct lpr_letterbox_layout {
    uint32_t scaled_width;
    uint32_t scaled_height;
    uint32_t pad_left;
    uint32_t pad_top;
};

int lpr_frame_pool_init_heap(struct lpr_frame_pool *pool,
                             size_t slot_count, size_t slot_capacity);
int lpr_frame_pool_init_external(struct lpr_frame_pool *pool,
                                 uint8_t *const *buffers,
                                 const size_t *capacities,
                                 size_t slot_count);
void lpr_frame_pool_shutdown(struct lpr_frame_pool *pool);
int lpr_frame_pool_destroy(struct lpr_frame_pool *pool);

/* Returns -EAGAIN when block is false and every slot is referenced. */
int lpr_frame_pool_acquire(struct lpr_frame_pool *pool,
                           struct lpr_frame_writer *writer, bool block);
uint8_t *lpr_frame_writer_data(const struct lpr_frame_writer *writer);
size_t lpr_frame_writer_capacity(const struct lpr_frame_writer *writer);
int lpr_frame_writer_publish(struct lpr_frame_writer *writer,
                             const struct lpr_frame_meta *meta,
                             struct lpr_frame_ref *out);
int lpr_frame_writer_abort(struct lpr_frame_writer *writer);

int lpr_frame_ref_clone(const struct lpr_frame_ref *src,
                        struct lpr_frame_ref *out);
int lpr_frame_ref_clone_rebind_source_generation(
    const struct lpr_frame_ref *src, uint64_t source_generation,
    struct lpr_frame_ref *out);
int lpr_frame_ref_release(struct lpr_frame_ref *ref);
const uint8_t *lpr_frame_ref_data(const struct lpr_frame_ref *ref);
size_t lpr_frame_ref_capacity(const struct lpr_frame_ref *ref);
bool lpr_frame_ref_is_valid(const struct lpr_frame_ref *ref);
bool lpr_frame_ref_matches_source_generation(const struct lpr_frame_ref *ref,
                                             uint64_t source_generation);

int lpr_bgrx_letterbox_copy(const uint8_t *src,
                            uint32_t src_width, uint32_t src_height,
                            uint32_t src_stride,
                            uint8_t *dst,
                            uint32_t dst_width, uint32_t dst_height,
                            uint32_t dst_stride,
                            struct lpr_letterbox_layout *layout);

#endif /* LPR_LIVE_LPR_FRAME_H */
