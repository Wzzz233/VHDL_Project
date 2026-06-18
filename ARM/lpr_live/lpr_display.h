// SPDX-License-Identifier: GPL-2.0
/* Display module: DRM/KMS appsrc -> kmssink RGB16 + drawing primitives. */

#ifndef LPR_LIVE_LPR_DISPLAY_H
#define LPR_LIVE_LPR_DISPLAY_H

#include "lpr_common.h"

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

struct display_state {
    bool enabled;
    int drm_fd;
    uint32_t w;
    uint32_t h;
    int fps;
    int connector_id;
    bool sync;
    size_t frame_size;
    uint64_t next_pts_ns;
    GstElement *pipeline;
    GstElement *appsrc;
    GstElement *queue;
    GstElement *sink;
    GstBus *bus;
};

int lpr_display_start(struct display_state *d, const struct live_options *opt,
                      uint32_t w, uint32_t h);
void lpr_display_stop(struct display_state *d);
int lpr_display_push(struct display_state *d, const uint16_t *frame);

/* Drawing primitives operate directly on RGB565 frames. */
void lpr_draw_rect_565(uint16_t *pix, int w, int h, const struct det_box *b, uint16_t c);
void lpr_draw_text_565(uint16_t *pix, int w, int h, int x, int y, const char *s,
                       uint16_t c, int scale);
void lpr_overlay_ascii_from_text(const char *text, char *out, size_t out_len);

#endif /* LPR_LIVE_LPR_DISPLAY_H */
