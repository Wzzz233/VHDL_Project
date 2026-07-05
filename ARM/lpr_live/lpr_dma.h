// SPDX-License-Identifier: GPL-2.0
/* FPGA DMA frame capture and pixel format conversion. */

#ifndef LPR_LIVE_LPR_DMA_H
#define LPR_LIVE_LPR_DMA_H

#include "lpr_common.h"

#include <pthread.h>
#include <stdint.h>

#include "../pcie_fpga_dma.h"

#define LPR_DMA_MAX_SLOTS 8
#define LPR_DMA_DEFAULT_SLOTS 4

struct dma_slot {
    uint8_t *data;
    size_t size;
    uint32_t index;
    uint64_t generation;
    int refs;
};

struct dma_state {
    int fd;
    struct dma_slot slots[LPR_DMA_MAX_SLOTS];
    int slot_count;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    bool lock_init;
    uint32_t frame_w;
    uint32_t frame_h;
    uint32_t frame_bpp;
    size_t frame_size;
    bool src_is_bgrx;
    bool zero_copy;
};

int lpr_dma_init(struct dma_state *d, const struct live_options *opt);
void lpr_dma_release(struct dma_state *d);
int lpr_dma_acquire_slot(struct dma_state *d);
void lpr_dma_slot_addref(struct dma_state *d, int slot);
void lpr_dma_slot_release(struct dma_state *d, int slot);
uint8_t *lpr_dma_slot_data(struct dma_state *d, int slot);
uint64_t lpr_dma_slot_generation(struct dma_state *d, int slot);
int lpr_dma_read_frame_slot(struct dma_state *d, int slot);
int lpr_dma_get_frame_status(struct dma_state *d, struct fpga_frame_status *status);
int lpr_dma_wait_new_frame(struct dma_state *d, uint32_t *last_change_count, int timeout_ms);

/* Decode a single BGR565/RGB565 word. Helper exposed for testability. */
void lpr_decode_pixel565(enum pixel_order order, bool swap16,
                         uint8_t lo_in, uint8_t hi_in,
                         uint8_t *r, uint8_t *g, uint8_t *b);

#endif /* LPR_LIVE_LPR_DMA_H */
