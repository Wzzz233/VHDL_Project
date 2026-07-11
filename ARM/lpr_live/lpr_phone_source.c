// SPDX-License-Identifier: GPL-2.0
/* MediaMTX RTSP/GStreamer phone frame source. */

#include "lpr_phone_source.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gst/video/video.h>

#define PHONE_PULL_TIMEOUT_NS (100U * GST_MSECOND)
#define PHONE_RECONNECT_DELAY_US 250000U

static int phone_open(void *ctx);
static int phone_start(void *ctx);
static int phone_read_latest(void *ctx, struct lpr_frame_ref *out);
static int phone_health(void *ctx, int64_t now_us,
                        struct lpr_source_health *out);
static void phone_stop(void *ctx);
static void phone_close(void *ctx);
static void *phone_thread_main(void *ctx);

static const struct lpr_frame_source_ops phone_ops = {
    .open = phone_open,
    .start = phone_start,
    .read_latest = phone_read_latest,
    .health = phone_health,
    .stop = phone_stop,
    .close = phone_close,
};

static bool phone_is_running(struct lpr_phone_source *source)
{
    bool running;

    pthread_mutex_lock(&source->lock);
    running = source->running;
    pthread_mutex_unlock(&source->lock);
    return running;
}

static void phone_note_drop(struct lpr_phone_source *source, bool error)
{
    pthread_mutex_lock(&source->lock);
    source->dropped_frames++;
    if (error)
        source->error_count++;
    pthread_mutex_unlock(&source->lock);
}

static void phone_release_latest(struct lpr_phone_source *source)
{
    struct lpr_frame_ref old;

    memset(&old, 0, sizeof(old));
    pthread_mutex_lock(&source->lock);
    if (lpr_frame_ref_is_valid(&source->latest)) {
        old = source->latest;
        memset(&source->latest, 0, sizeof(source->latest));
    }
    source->has_frame = false;
    pthread_mutex_unlock(&source->lock);
    if (old.pool)
        lpr_frame_ref_release(&old);
}

int lpr_phone_source_init(struct lpr_phone_source *source,
                          const char *rtsp_url,
                          uint64_t source_generation)
{
    const char *url = rtsp_url ? rtsp_url : LPR_PHONE_RTSP_URL;
    size_t url_len;
    int rc;

    if (!source)
        return -EINVAL;
    url_len = strlen(url);
    if (url_len == 0 || url_len >= LPR_PHONE_RTSP_URL_MAX)
        return -EINVAL;
    memset(source, 0, sizeof(*source));
    memcpy(source->rtsp_url, url, url_len + 1U);
    source->source_generation = source_generation;
    source->source.id = LPR_SOURCE_PHONE;
    source->source.name = "phone";
    source->source.ops = &phone_ops;
    source->source.ctx = source;
    rc = pthread_mutex_init(&source->lock, NULL);
    if (rc != 0)
        return -rc;
    source->lock_init = true;
    return 0;
}

struct lpr_frame_source *lpr_phone_source_as_frame_source(
    struct lpr_phone_source *source)
{
    return source ? &source->source : NULL;
}

const char *lpr_phone_source_pipeline_description(
    const struct lpr_phone_source *source)
{
    return source ? source->pipeline_description : NULL;
}

void lpr_phone_source_set_generation(struct lpr_phone_source *source,
                                     uint64_t source_generation)
{
    struct lpr_frame_ref old;

    if (!source || !source->lock_init)
        return;
    memset(&old, 0, sizeof(old));
    pthread_mutex_lock(&source->lock);
    source->source_generation = source_generation;
    if (lpr_frame_ref_is_valid(&source->latest)) {
        old = source->latest;
        memset(&source->latest, 0, sizeof(source->latest));
    }
    source->has_frame = false;
    source->last_frame_us = 0;
    source->previous_frame_us = 0;
    source->input_fps = 0.0;
    pthread_mutex_unlock(&source->lock);
    if (old.pool)
        lpr_frame_ref_release(&old);
}

static void phone_release_pipeline(struct lpr_phone_source *source)
{
    if (source->pipeline)
        gst_element_set_state(source->pipeline, GST_STATE_NULL);
    if (source->bus)
        gst_object_unref(source->bus);
    if (source->appsink)
        gst_object_unref(source->appsink);
    if (source->pipeline)
        gst_object_unref(source->pipeline);
    source->bus = NULL;
    source->appsink = NULL;
    source->pipeline = NULL;
    g_free(source->pipeline_description);
    source->pipeline_description = NULL;
}

static int phone_build_pipeline(struct lpr_phone_source *source)
{
    char *escaped_url;
    GstElement *sink_element;
    GError *error = NULL;

    escaped_url = g_strescape(source->rtsp_url, NULL);
    if (!escaped_url)
        return -ENOMEM;
    source->pipeline_description = g_strdup_printf(
        "rtspsrc name=phone_rtsp location=\"%s\" protocols=tcp latency=100 "
        "drop-on-latency=true tcp-timeout=2000000 ! "
        "rtph264depay ! h264parse ! mppvideodec ! videoconvert ! "
        "videoscale add-borders=true ! "
        "video/x-raw,format=BGRx,width=%u,height=%u,pixel-aspect-ratio=1/1 ! "
        "appsink name=phone_sink max-buffers=1 drop=true sync=false",
        escaped_url, LPR_PHONE_FRAME_WIDTH, LPR_PHONE_FRAME_HEIGHT);
    g_free(escaped_url);
    if (!source->pipeline_description)
        return -ENOMEM;

    source->pipeline = gst_parse_launch(source->pipeline_description, &error);
    if (!source->pipeline) {
        fprintf(stderr, "[phone-source] failed to build pipeline: %s\n",
                error ? error->message : "unknown error");
        if (error)
            g_error_free(error);
        return -EINVAL;
    }
    if (error) {
        fprintf(stderr, "[phone-source] incomplete pipeline: %s\n",
                error->message);
        g_error_free(error);
        return -EINVAL;
    }
    sink_element = gst_bin_get_by_name(GST_BIN(source->pipeline),
                                       "phone_sink");
    if (!sink_element || !GST_IS_APP_SINK(sink_element)) {
        if (sink_element)
            gst_object_unref(sink_element);
        fprintf(stderr, "[phone-source] appsink was not created\n");
        return -EINVAL;
    }
    source->appsink = GST_APP_SINK(sink_element);
    gst_app_sink_set_max_buffers(source->appsink, 1);
    gst_app_sink_set_drop(source->appsink, TRUE);
    g_object_set(source->appsink, "sync", FALSE, "emit-signals", FALSE,
                 NULL);
    source->bus = gst_element_get_bus(source->pipeline);
    if (!source->bus)
        return -EIO;
    return 0;
}

static int phone_open(void *ctx)
{
    struct lpr_phone_source *source = ctx;
    size_t frame_size = (size_t)LPR_PHONE_FRAME_STRIDE *
                        LPR_PHONE_FRAME_HEIGHT;
    int rc;

    if (!source || !source->lock_init)
        return -EINVAL;
    pthread_mutex_lock(&source->lock);
    if (source->opened) {
        pthread_mutex_unlock(&source->lock);
        return 0;
    }
    pthread_mutex_unlock(&source->lock);

    gst_init(NULL, NULL);
    rc = lpr_frame_pool_init_heap(&source->pool, LPR_PHONE_POOL_SLOTS,
                                  frame_size);
    if (rc < 0)
        return rc;
    source->pool_init = true;
    rc = phone_build_pipeline(source);
    if (rc < 0) {
        phone_release_pipeline(source);
        lpr_frame_pool_destroy(&source->pool);
        source->pool_init = false;
        return rc;
    }
    pthread_mutex_lock(&source->lock);
    source->opened = true;
    source->has_frame = false;
    source->last_frame_us = 0;
    source->previous_frame_us = 0;
    source->input_fps = 0.0;
    pthread_mutex_unlock(&source->lock);
    fprintf(stderr,
            "[phone-source] opened %s -> %ux%u BGRx appsink max-buffers=1 drop=true sync=false\n",
            source->rtsp_url, LPR_PHONE_FRAME_WIDTH,
            LPR_PHONE_FRAME_HEIGHT);
    return 0;
}

static int phone_start(void *ctx)
{
    struct lpr_phone_source *source = ctx;
    int rc;

    if (!source || !source->lock_init)
        return -EINVAL;
    pthread_mutex_lock(&source->lock);
    if (!source->opened) {
        pthread_mutex_unlock(&source->lock);
        return -ENODEV;
    }
    if (source->running) {
        pthread_mutex_unlock(&source->lock);
        return 0;
    }
    source->running = true;
    pthread_mutex_unlock(&source->lock);
    rc = pthread_create(&source->thread, NULL, phone_thread_main, source);
    if (rc != 0) {
        pthread_mutex_lock(&source->lock);
        source->running = false;
        source->error_count++;
        pthread_mutex_unlock(&source->lock);
        return -rc;
    }
    source->thread_started = true;
    return 0;
}

static int phone_read_latest(void *ctx, struct lpr_frame_ref *out)
{
    struct lpr_phone_source *source = ctx;
    int rc;

    if (!source || !out || !source->lock_init)
        return -EINVAL;
    memset(out, 0, sizeof(*out));
    pthread_mutex_lock(&source->lock);
    if (!source->opened || !source->running) {
        pthread_mutex_unlock(&source->lock);
        return -ESHUTDOWN;
    }
    if (!source->has_frame) {
        pthread_mutex_unlock(&source->lock);
        return -EAGAIN;
    }
    rc = lpr_frame_ref_clone(&source->latest, out);
    pthread_mutex_unlock(&source->lock);
    return rc;
}

static int phone_health(void *ctx, int64_t now_us,
                        struct lpr_source_health *out)
{
    struct lpr_phone_source *source = ctx;

    if (!source || !out || !source->lock_init)
        return -EINVAL;
    memset(out, 0, sizeof(*out));
    pthread_mutex_lock(&source->lock);
    out->opened = source->opened;
    out->running = source->running;
    out->has_frame = source->has_frame;
    out->last_frame_us = source->last_frame_us;
    if (source->has_frame && now_us >= source->last_frame_us)
        out->frame_age_us = now_us - source->last_frame_us;
    else
        out->frame_age_us = -1;
    out->healthy = source->opened && source->running &&
                   source->has_frame && out->frame_age_us >= 0 &&
                   out->frame_age_us <= LPR_PHONE_STALE_US;
    out->sequence = source->sequence;
    out->input_fps = source->input_fps;
    out->dropped_frames = source->dropped_frames;
    out->error_count = source->error_count;
    out->reconnect_count = source->reconnect_count;
    pthread_mutex_unlock(&source->lock);
    return 0;
}

static void phone_stop(void *ctx)
{
    struct lpr_phone_source *source = ctx;

    if (!source || !source->lock_init)
        return;
    pthread_mutex_lock(&source->lock);
    source->running = false;
    pthread_mutex_unlock(&source->lock);
    if (source->pipeline)
        gst_element_set_state(source->pipeline, GST_STATE_NULL);
    if (source->thread_started) {
        pthread_join(source->thread, NULL);
        source->thread_started = false;
    }
}

static void phone_close(void *ctx)
{
    struct lpr_phone_source *source = ctx;
    int rc;

    if (!source || !source->lock_init)
        return;
    phone_stop(source);
    pthread_mutex_lock(&source->lock);
    source->opened = false;
    pthread_mutex_unlock(&source->lock);
    phone_release_latest(source);
    phone_release_pipeline(source);
    if (source->pool_init) {
        lpr_frame_pool_shutdown(&source->pool);
        rc = lpr_frame_pool_destroy(&source->pool);
        if (rc < 0) {
            fprintf(stderr,
                    "[phone-source] frame pool still has live references during close: %s\n",
                    strerror(-rc));
            return;
        }
        source->pool_init = false;
    }
    pthread_mutex_destroy(&source->lock);
    source->lock_init = false;
}

static int phone_publish_sample(struct lpr_phone_source *source,
                                GstSample *sample)
{
    GstCaps *caps = gst_sample_get_caps(sample);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstVideoInfo info;
    GstVideoFrame frame;
    struct lpr_frame_writer writer;
    struct lpr_frame_ref fresh;
    struct lpr_frame_ref old;
    struct lpr_frame_meta meta;
    uint8_t *dst;
    const uint8_t *src;
    int src_stride;
    uint64_t sequence;
    uint64_t source_generation;
    int64_t now_us;
    int rc;
    bool keep;

    memset(&frame, 0, sizeof(frame));
    memset(&writer, 0, sizeof(writer));
    memset(&fresh, 0, sizeof(fresh));
    memset(&old, 0, sizeof(old));
    if (!caps || !buffer || !gst_video_info_from_caps(&info, caps) ||
        GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_BGRx ||
        GST_VIDEO_INFO_WIDTH(&info) != LPR_PHONE_FRAME_WIDTH ||
        GST_VIDEO_INFO_HEIGHT(&info) != LPR_PHONE_FRAME_HEIGHT ||
        !gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) {
        phone_note_drop(source, true);
        return -EINVAL;
    }
    src = GST_VIDEO_FRAME_PLANE_DATA(&frame, 0);
    src_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    /* GstVideoFrame row iteration is only unambiguous here for top-down
     * packed output. videoconvert/videoscale normally provides positive
     * BGRx stride; reject bottom-up frames instead of risking an underflow. */
    if (!src || src_stride < (int)LPR_PHONE_FRAME_STRIDE) {
        gst_video_frame_unmap(&frame);
        phone_note_drop(source, true);
        return -EINVAL;
    }

    now_us = g_get_monotonic_time();
    pthread_mutex_lock(&source->lock);
    sequence = ++source->sequence;
    source_generation = source->source_generation;
    pthread_mutex_unlock(&source->lock);

    rc = lpr_frame_pool_acquire(&source->pool, &writer, false);
    if (rc < 0) {
        gst_video_frame_unmap(&frame);
        phone_note_drop(source, rc != -EAGAIN);
        return rc == -EAGAIN ? 0 : rc;
    }
    if (lpr_frame_writer_capacity(&writer) <
        (size_t)LPR_PHONE_FRAME_STRIDE * LPR_PHONE_FRAME_HEIGHT) {
        lpr_frame_writer_abort(&writer);
        gst_video_frame_unmap(&frame);
        phone_note_drop(source, true);
        return -ENOSPC;
    }
    dst = lpr_frame_writer_data(&writer);
    if (!dst) {
        lpr_frame_writer_abort(&writer);
        gst_video_frame_unmap(&frame);
        phone_note_drop(source, true);
        return -EIO;
    }
    for (uint32_t y = 0; y < LPR_PHONE_FRAME_HEIGHT; y++) {
        memcpy(dst + (size_t)y * LPR_PHONE_FRAME_STRIDE,
               src + (ptrdiff_t)y * src_stride,
               LPR_PHONE_FRAME_STRIDE);
    }
    gst_video_frame_unmap(&frame);

    memset(&meta, 0, sizeof(meta));
    meta.format = LPR_FRAME_FORMAT_BGRX8888;
    meta.width = LPR_PHONE_FRAME_WIDTH;
    meta.height = LPR_PHONE_FRAME_HEIGHT;
    meta.stride = LPR_PHONE_FRAME_STRIDE;
    meta.monotonic_us = now_us;
    meta.sequence = sequence;
    meta.source_generation = source_generation;
    rc = lpr_frame_writer_publish(&writer, &meta, &fresh);
    if (rc < 0) {
        phone_note_drop(source, true);
        return rc;
    }

    pthread_mutex_lock(&source->lock);
    keep = source->running &&
           source_generation == source->source_generation;
    if (keep) {
        if (lpr_frame_ref_is_valid(&source->latest))
            old = source->latest;
        source->latest = fresh;
        source->has_frame = true;
        if (source->previous_frame_us > 0 &&
            now_us > source->previous_frame_us) {
            double instantaneous = 1000000.0 /
                (double)(now_us - source->previous_frame_us);
            source->input_fps = source->input_fps > 0.0
                ? source->input_fps * 0.9 + instantaneous * 0.1
                : instantaneous;
        }
        source->previous_frame_us = now_us;
        source->last_frame_us = now_us;
    }
    pthread_mutex_unlock(&source->lock);
    if (old.pool)
        lpr_frame_ref_release(&old);
    if (!keep)
        lpr_frame_ref_release(&fresh);
    return 0;
}

static void phone_log_pipeline_message(struct lpr_phone_source *source,
                                       GstMessage *message)
{
    uint64_t error_count;

    pthread_mutex_lock(&source->lock);
    source->error_count++;
    error_count = source->error_count;
    pthread_mutex_unlock(&source->lock);
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
        GError *error = NULL;
        char *debug = NULL;

        gst_message_parse_error(message, &error, &debug);
        if (error_count <= 5 || error_count % 20 == 0)
            fprintf(stderr, "[phone-source] pipeline error: %s\n",
                    error ? error->message : "unknown error");
        if (error)
            g_error_free(error);
        g_free(debug);
    } else if (error_count <= 5 || error_count % 20 == 0) {
        fprintf(stderr, "[phone-source] pipeline reached end of stream\n");
    }
}

static void phone_drain_bus(struct lpr_phone_source *source)
{
    GstMessage *message;

    while ((message = gst_bus_pop(source->bus)) != NULL)
        gst_message_unref(message);
}

static void *phone_thread_main(void *ctx)
{
    struct lpr_phone_source *source = ctx;
    bool first_attempt = true;

    while (phone_is_running(source)) {
        GstStateChangeReturn state_result;
        int64_t attempt_us;

        gst_element_set_state(source->pipeline, GST_STATE_NULL);
        phone_drain_bus(source);
        if (!first_attempt) {
            pthread_mutex_lock(&source->lock);
            source->reconnect_count++;
            pthread_mutex_unlock(&source->lock);
            g_usleep(PHONE_RECONNECT_DELAY_US);
            if (!phone_is_running(source))
                break;
        }
        first_attempt = false;
        attempt_us = g_get_monotonic_time();
        pthread_mutex_lock(&source->lock);
        source->previous_frame_us = 0;
        source->input_fps = 0.0;
        pthread_mutex_unlock(&source->lock);
        state_result = gst_element_set_state(source->pipeline,
                                             GST_STATE_PLAYING);
        if (state_result == GST_STATE_CHANGE_FAILURE) {
            pthread_mutex_lock(&source->lock);
            source->error_count++;
            pthread_mutex_unlock(&source->lock);
            continue;
        }

        while (phone_is_running(source)) {
            GstSample *sample = gst_app_sink_try_pull_sample(
                source->appsink, PHONE_PULL_TIMEOUT_NS);
            GstMessage *message;
            bool restart = false;
            int64_t now_us;
            int64_t last_frame_us;

            if (sample) {
                if (phone_publish_sample(source, sample) < 0)
                    restart = true;
                gst_sample_unref(sample);
            }
            message = gst_bus_timed_pop_filtered(
                source->bus, 0, GST_MESSAGE_ERROR | GST_MESSAGE_EOS);
            if (message) {
                phone_log_pipeline_message(source, message);
                gst_message_unref(message);
                restart = true;
            }
            now_us = g_get_monotonic_time();
            pthread_mutex_lock(&source->lock);
            last_frame_us = source->last_frame_us;
            pthread_mutex_unlock(&source->lock);
            if (now_us - (last_frame_us > attempt_us
                              ? last_frame_us : attempt_us) >
                LPR_PHONE_STALE_US) {
                pthread_mutex_lock(&source->lock);
                source->error_count++;
                pthread_mutex_unlock(&source->lock);
                restart = true;
            }
            if (restart)
                break;
        }
    }
    gst_element_set_state(source->pipeline, GST_STATE_NULL);
    return NULL;
}
