// SPDX-License-Identifier: GPL-2.0
/* FPGA DMA frame capture and pixel format conversion. */

#ifndef LPR_LIVE_LPR_DMA_H
#define LPR_LIVE_LPR_DMA_H

#include "lpr_common.h"

#include <stdint.h>

struct dma_state {
    int fd;
    void *map;
    size_t map_size;
    uint8_t *copy;
    uint32_t frame_w;
    uint32_t frame_h;
    uint32_t frame_bpp;
    size_t frame_size;
    bool src_is_bgrx;
};

int lpr_dma_init(struct dma_state *d, const struct live_options *opt);
void lpr_dma_release(struct dma_state *d);
int lpr_dma_read_frame(struct dma_state *d);

/* Decode a single BGR565/RGB565 word. Helper exposed for testability. */
void lpr_decode_pixel565(enum pixel_order order, bool swap16,
                         uint8_t lo_in, uint8_t hi_in,
                         uint8_t *r, uint8_t *g, uint8_t *b);

/* Convert one captured frame to a packed RGB888 buffer (caller-owned). */
void lpr_frame_to_rgb888(const struct dma_state *d,
                         const struct live_options *opt,
                         uint8_t *rgb);

/* Convert RGB888 buffer to RGB565 inplace into a caller-owned dst. */
void lpr_rgb888_to_rgb565(const uint8_t *rgb, uint16_t *dst, int w, int h);

#endif /* LPR_LIVE_LPR_DMA_H */
