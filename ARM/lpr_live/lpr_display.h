// SPDX-License-Identifier: GPL-2.0
/* Display module: DRM/KMS appsrc -> kmssink BGRx + drawing primitives. */

#ifndef LPR_LIVE_LPR_DISPLAY_H
#define LPR_LIVE_LPR_DISPLAY_H

#include "lpr_common.h"
#include "lpr_dma.h"

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <pthread.h>

#define LPR_DISPLAY_COPY_SLOTS 6
#define LPR_DISPLAY_RELEASE_DELAY_MS 0

struct display_copy_slot {
    uint8_t *data;
    bool in_use;
    bool release_pending;
    int64_t release_at_us;
    uint64_t generation;
};

struct display_state {
    bool enabled;
    int drm_fd;
    uint32_t w;
    uint32_t h;
    int fps;
    int connector_id;
    bool sync;
    bool atomic_flip;
    size_t frame_size;
    uint64_t next_pts_ns;
    GstElement *pipeline;
    GstElement *appsrc;
    GstElement *queue;
    GstElement *sink;
    GstBus *bus;
    pthread_t thread;
    bool thread_started;
    bool running;
    bool has_new;
    bool display_error;
    struct dma_state *dma;
    int pending_slot;
    uint64_t pending_generation;
    struct display_copy_slot copy_slots[LPR_DISPLAY_COPY_SLOTS];
    pthread_mutex_t slots_lock;
    pthread_cond_t slots_cond;
    bool slots_lock_init;
    uint64_t dropped_frames;
};

int lpr_display_start(struct display_state *d, const struct live_options *opt,
                      uint32_t w, uint32_t h);
void lpr_display_stop(struct display_state *d);
int lpr_display_push_bgrx_slot(struct display_state *d, struct dma_state *dma, int slot);

/* Drawing primitives operate directly on BGRX8888 or RGB565 frames. */
void lpr_draw_rect_bgrx(uint8_t *pix, int w, int h, const struct det_box *b, uint8_t r, uint8_t g, uint8_t bl);
/* Draw the 4-point perspective quadrilateral (quad[0..7] = TL.x,TL.y, TR.x,TR.y,
 * BR.x,BR.y, BL.x,BL.y) plus 3x3 corner markers. Replaces the axis-aligned rect
 * for plates so adjacent tilted plates no longer overlap as bounding boxes. */
void lpr_draw_quad_bgrx(uint8_t *pix, int w, int h, const float quad[8], uint8_t r, uint8_t g, uint8_t bl);
void lpr_draw_text_bgrx(uint8_t *pix, int w, int h, int x, int y, const char *s,
                        uint8_t r, uint8_t g, uint8_t bl, int scale);
void lpr_draw_rect_565(uint16_t *pix, int w, int h, const struct det_box *b, uint16_t c);
void lpr_draw_text_565(uint16_t *pix, int w, int h, int x, int y, const char *s,
                       uint16_t c, int scale);
void lpr_overlay_ascii_from_text(const char *text, char *out, size_t out_len);

#endif /* LPR_LIVE_LPR_DISPLAY_H */
