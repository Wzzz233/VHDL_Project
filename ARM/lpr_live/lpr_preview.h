// SPDX-License-Identifier: GPL-2.0
/* Low-rate in-memory JPEG preview for the LAN control panel. */

#ifndef LPR_LIVE_LPR_PREVIEW_H
#define LPR_LIVE_LPR_PREVIEW_H

#include "lpr_frame.h"

#include <gst/gst.h>
#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LPR_PREVIEW_WIDTH 640U
#define LPR_PREVIEW_HEIGHT 360U
#define LPR_PREVIEW_MAX_FPS 5
#define LPR_PREVIEW_JPEG_QUALITY 75

struct lpr_preview {
    GstElement *pipeline;
    GstElement *appsrc;
    GstElement *appsink;
    GstBus *bus;
    pthread_mutex_t lock;
    bool lock_initialized;
    bool enabled;
    int64_t last_push_us;
    uint8_t *jpeg;
    size_t jpeg_size;
    uint64_t jpeg_sequence;
    uint64_t dropped_frames;
};

int lpr_preview_start(struct lpr_preview *preview, bool enabled);
void lpr_preview_stop(struct lpr_preview *preview);
int lpr_preview_reset(struct lpr_preview *preview);
int lpr_preview_push(struct lpr_preview *preview,
                     const struct lpr_frame_ref *frame);

/* Returns 1 when no newer JPEG exists, 0 with a caller-owned snapshot. */
int lpr_preview_snapshot(struct lpr_preview *preview, uint64_t since_sequence,
                         uint8_t **jpeg_out, size_t *size_out,
                         uint64_t *sequence_out);

#endif /* LPR_LIVE_LPR_PREVIEW_H */
