// SPDX-License-Identifier: GPL-2.0
/* Low-rate in-memory JPEG preview for the LAN control panel. */

#include "lpr_preview.h"

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static GstFlowReturn preview_new_sample(GstAppSink *sink, gpointer user_data)
{
    struct lpr_preview *preview = (struct lpr_preview *)user_data;
    GstSample *sample;
    GstBuffer *buffer;
    GstMapInfo map;
    uint8_t *copy;

    sample = gst_app_sink_pull_sample(sink);
    if (!sample)
        return GST_FLOW_ERROR;
    buffer = gst_sample_get_buffer(sample);
    if (!buffer || !gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }
    copy = malloc(map.size);
    if (!copy) {
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }
    memcpy(copy, map.data, map.size);

    pthread_mutex_lock(&preview->lock);
    free(preview->jpeg);
    preview->jpeg = copy;
    preview->jpeg_size = map.size;
    preview->jpeg_sequence++;
    pthread_mutex_unlock(&preview->lock);

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

static int preview_check_bus(struct lpr_preview *preview)
{
    GstMessage *message;

    while ((message = gst_bus_pop(preview->bus)) != NULL) {
        if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
            GError *error = NULL;
            gchar *debug = NULL;

            gst_message_parse_error(message, &error, &debug);
            fprintf(stderr, "[preview] pipeline error: %s\n",
                    error ? error->message : "unknown");
            if (error)
                g_error_free(error);
            g_free(debug);
            gst_message_unref(message);
            return -1;
        }
        gst_message_unref(message);
    }
    return 0;
}

int lpr_preview_start(struct lpr_preview *preview, bool enabled)
{
    static const char pipeline_description[] =
        "appsrc name=preview_src is-live=true block=false format=time "
        "do-timestamp=true ! "
        "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        "leaky=downstream ! "
        "videoconvert ! videoscale add-borders=true ! "
        "video/x-raw,format=I420,width=640,height=360,pixel-aspect-ratio=1/1 ! "
        "jpegenc quality=75 ! "
        "appsink name=preview_sink max-buffers=1 drop=true sync=false "
        "emit-signals=true";
    GstCaps *caps;
    GstStateChangeReturn state_result;
    GError *error = NULL;

    if (!preview)
        return -1;
    memset(preview, 0, sizeof(*preview));
    preview->enabled = enabled;
    if (!enabled)
        return 0;

    pthread_mutex_init(&preview->lock, NULL);
    preview->lock_initialized = true;
    preview->pipeline = gst_parse_launch(pipeline_description, &error);
    if (!preview->pipeline || error) {
        fprintf(stderr, "[preview] failed to create pipeline: %s\n",
                 error ? error->message : "unknown");
        if (error)
            g_error_free(error);
        goto fail;
    }
    preview->appsrc = gst_bin_get_by_name(GST_BIN(preview->pipeline),
                                           "preview_src");
    preview->appsink = gst_bin_get_by_name(GST_BIN(preview->pipeline),
                                            "preview_sink");
    if (!preview->appsrc || !preview->appsink)
        goto fail;

    caps = gst_caps_new_simple("video/x-raw",
                              "format", G_TYPE_STRING, "BGRx",
                              "width", G_TYPE_INT, 1280,
                              "height", G_TYPE_INT, 720,
                              "framerate", GST_TYPE_FRACTION,
                              LPR_PREVIEW_MAX_FPS, 1, NULL);
    if (!caps)
        goto fail;
    gst_app_src_set_caps(GST_APP_SRC(preview->appsrc), caps);
    gst_caps_unref(caps);
    g_signal_connect(preview->appsink, "new-sample",
                     G_CALLBACK(preview_new_sample), preview);

    preview->bus = gst_element_get_bus(preview->pipeline);
    state_result = gst_element_set_state(preview->pipeline, GST_STATE_PLAYING);
    if (state_result == GST_STATE_CHANGE_FAILURE)
        goto fail;
    state_result = gst_element_get_state(preview->pipeline, NULL, NULL,
                                         5 * GST_SECOND);
    if (state_result == GST_STATE_CHANGE_FAILURE)
        goto fail;
    fprintf(stderr, "[preview] started 640x360 JPEG quality=75 max_fps=5\n");
    return 0;

fail:
    lpr_preview_stop(preview);
    return -1;
}

void lpr_preview_stop(struct lpr_preview *preview)
{
    if (!preview)
        return;
    if (preview->appsink)
        g_signal_handlers_disconnect_by_data(preview->appsink, preview);
    if (preview->appsrc)
        gst_app_src_end_of_stream(GST_APP_SRC(preview->appsrc));
    if (preview->pipeline)
        gst_element_set_state(preview->pipeline, GST_STATE_NULL);
    if (preview->bus)
        gst_object_unref(preview->bus);
    if (preview->appsrc)
        gst_object_unref(preview->appsrc);
    if (preview->appsink)
        gst_object_unref(preview->appsink);
    if (preview->pipeline)
        gst_object_unref(preview->pipeline);
    if (preview->lock_initialized) {
        pthread_mutex_lock(&preview->lock);
        free(preview->jpeg);
        preview->jpeg = NULL;
        preview->jpeg_size = 0;
        pthread_mutex_unlock(&preview->lock);
        pthread_mutex_destroy(&preview->lock);
    }
    memset(preview, 0, sizeof(*preview));
}

int lpr_preview_reset(struct lpr_preview *preview)
{
    GstStateChangeReturn state_result;

    if (!preview)
        return -1;
    if (!preview->enabled)
        return 0;
    if (!preview->pipeline || !preview->lock_initialized)
        return -1;

    /* READY releases queued raw/JPEG buffers and synchronizes with the
     * appsink callback, so no old callback can repopulate the cleared cache. */
    state_result = gst_element_set_state(preview->pipeline, GST_STATE_READY);
    if (state_result != GST_STATE_CHANGE_FAILURE)
        state_result = gst_element_get_state(preview->pipeline, NULL, NULL,
                                             5 * GST_SECOND);
    if (state_result == GST_STATE_CHANGE_FAILURE ||
        state_result == GST_STATE_CHANGE_ASYNC) {
        state_result = gst_element_set_state(preview->pipeline,
                                             GST_STATE_NULL);
        if (state_result == GST_STATE_CHANGE_FAILURE)
            return -1;
        state_result = gst_element_get_state(preview->pipeline, NULL, NULL,
                                             5 * GST_SECOND);
        if (state_result == GST_STATE_CHANGE_FAILURE ||
            state_result == GST_STATE_CHANGE_ASYNC)
            return -1;
    }

    pthread_mutex_lock(&preview->lock);
    free(preview->jpeg);
    preview->jpeg = NULL;
    preview->jpeg_size = 0;
    preview->jpeg_sequence = 0;
    preview->last_push_us = 0;
    pthread_mutex_unlock(&preview->lock);

    state_result = gst_element_set_state(preview->pipeline,
                                         GST_STATE_PLAYING);
    if (state_result == GST_STATE_CHANGE_FAILURE)
        return -1;
    state_result = gst_element_get_state(preview->pipeline, NULL, NULL,
                                         5 * GST_SECOND);
    if (state_result == GST_STATE_CHANGE_FAILURE ||
        state_result == GST_STATE_CHANGE_ASYNC)
        return -1;
    return 0;
}

int lpr_preview_push(struct lpr_preview *preview,
                     const struct lpr_frame_ref *frame)
{
    const uint8_t *source;
    GstBuffer *buffer;
    GstMapInfo map;
    GstFlowReturn flow;
    int64_t now_us;
    size_t row_size;
    size_t frame_size;

    if (!preview || !preview->enabled)
        return 0;
    if (!frame || !lpr_frame_ref_is_valid(frame) ||
        frame->meta.format != LPR_FRAME_FORMAT_BGRX8888 ||
        frame->meta.width != 1280U || frame->meta.height != 720U ||
        frame->meta.stride < 1280U * 4U)
        return -1;
    if (preview_check_bus(preview) < 0)
        return -1;

    now_us = g_get_monotonic_time();
    if (preview->last_push_us > 0 &&
        now_us - preview->last_push_us <
            1000000LL / LPR_PREVIEW_MAX_FPS) {
        preview->dropped_frames++;
        return 0;
    }
    preview->last_push_us = now_us;

    row_size = 1280U * 4U;
    frame_size = row_size * 720U;
    buffer = gst_buffer_new_allocate(NULL, frame_size, NULL);
    if (!buffer || !gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        if (buffer)
            gst_buffer_unref(buffer);
        return -1;
    }
    source = lpr_frame_ref_data(frame);
    if (!source) {
        gst_buffer_unmap(buffer, &map);
        gst_buffer_unref(buffer);
        return -1;
    }
    for (uint32_t y = 0; y < 720U; y++)
        memcpy(map.data + (size_t)y * row_size,
               source + (size_t)y * frame->meta.stride, row_size);
    gst_buffer_unmap(buffer, &map);

    flow = gst_app_src_push_buffer(GST_APP_SRC(preview->appsrc), buffer);
    return flow == GST_FLOW_OK ? 0 : -1;
}

int lpr_preview_snapshot(struct lpr_preview *preview, uint64_t since_sequence,
                         uint8_t **jpeg_out, size_t *size_out,
                         uint64_t *sequence_out)
{
    uint8_t *copy;

    if (!preview || !jpeg_out || !size_out || !sequence_out ||
        !preview->lock_initialized)
        return -1;
    *jpeg_out = NULL;
    *size_out = 0;
    *sequence_out = 0;
    pthread_mutex_lock(&preview->lock);
    if (!preview->jpeg || preview->jpeg_sequence <= since_sequence) {
        pthread_mutex_unlock(&preview->lock);
        return 1;
    }
    copy = malloc(preview->jpeg_size);
    if (!copy) {
        pthread_mutex_unlock(&preview->lock);
        return -1;
    }
    memcpy(copy, preview->jpeg, preview->jpeg_size);
    *jpeg_out = copy;
    *size_out = preview->jpeg_size;
    *sequence_out = preview->jpeg_sequence;
    pthread_mutex_unlock(&preview->lock);
    return 0;
}
