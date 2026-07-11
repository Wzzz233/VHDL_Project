// SPDX-License-Identifier: GPL-2.0
/* OV5640/FPGA DMA implementation of the generic frame-source interface. */

#ifndef LPR_LIVE_LPR_FPGA_SOURCE_H
#define LPR_LIVE_LPR_FPGA_SOURCE_H

#include "lpr_dma.h"
#include "lpr_source.h"

#include <pthread.h>

#define LPR_FPGA_FRAME_WIDTH 1280U
#define LPR_FPGA_FRAME_HEIGHT 720U

struct lpr_fpga_source {
    struct lpr_frame_source source;
    struct dma_state dma;
    struct lpr_frame_pool file_pool;
    uint8_t *file_pixels;
    size_t file_size;
    const struct live_options *options;
    pthread_mutex_t lock;
    bool lock_init;
    bool opened;
    bool file_mode;
    bool file_pool_init;
    bool running;
    bool healthy;
    bool has_frame;
    bool frame_status_available;
    bool camera_status_available;
    uint32_t last_frame_change_count;
    int64_t last_frame_us;
    int64_t previous_frame_us;
    uint64_t source_generation;
    uint64_t sequence;
    uint64_t dropped_frames;
    uint64_t error_count;
    double input_fps;
};

int lpr_fpga_source_init(struct lpr_fpga_source *source,
                         const struct live_options *options,
                         uint64_t source_generation);
struct lpr_frame_source *lpr_fpga_source_as_frame_source(
    struct lpr_fpga_source *source);
struct dma_state *lpr_fpga_source_dma(struct lpr_fpga_source *source);
void lpr_fpga_source_set_generation(struct lpr_fpga_source *source,
                                    uint64_t source_generation);

#endif /* LPR_LIVE_LPR_FPGA_SOURCE_H */
