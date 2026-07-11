// SPDX-License-Identifier: GPL-2.0
/* MediaMTX RTSP/GStreamer implementation of the phone frame source. */

#ifndef LPR_LIVE_LPR_PHONE_SOURCE_H
#define LPR_LIVE_LPR_PHONE_SOURCE_H

#include "lpr_source.h"

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <pthread.h>

#define LPR_PHONE_FRAME_WIDTH 1280U
#define LPR_PHONE_FRAME_HEIGHT 720U
#define LPR_PHONE_FRAME_STRIDE (LPR_PHONE_FRAME_WIDTH * 4U)
#define LPR_PHONE_POOL_SLOTS 4U
#define LPR_PHONE_RTSP_URL "rtsp://127.0.0.1:8554/phone"
#define LPR_PHONE_RTSP_URL_MAX 512U

struct lpr_phone_source {
    struct lpr_frame_source source;
    struct lpr_frame_pool pool;
    struct lpr_frame_ref latest;
    pthread_mutex_t lock;
    pthread_t thread;
    bool lock_init;
    bool pool_init;
    bool opened;
    bool running;
    bool thread_started;
    bool has_frame;
    char rtsp_url[LPR_PHONE_RTSP_URL_MAX];
    char *pipeline_description;
    GstElement *pipeline;
    GstAppSink *appsink;
    GstBus *bus;
    int64_t last_frame_us;
    int64_t previous_frame_us;
    uint64_t source_generation;
    uint64_t sequence;
    uint64_t dropped_frames;
    uint64_t error_count;
    uint64_t reconnect_count;
    double input_fps;
};

int lpr_phone_source_init(struct lpr_phone_source *source,
                          const char *rtsp_url,
                          uint64_t source_generation);
struct lpr_frame_source *lpr_phone_source_as_frame_source(
    struct lpr_phone_source *source);
void lpr_phone_source_set_generation(struct lpr_phone_source *source,
                                     uint64_t source_generation);
const char *lpr_phone_source_pipeline_description(
    const struct lpr_phone_source *source);

#endif /* LPR_LIVE_LPR_PHONE_SOURCE_H */
