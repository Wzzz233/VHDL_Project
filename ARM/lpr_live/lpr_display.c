// SPDX-License-Identifier: GPL-2.0
/* Display module: HDMI/KMS RGB16 output and overlay drawing. */

#include "lpr_display.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ---------------- Drawing primitives ---------------- */

static void draw_hline_565(uint16_t *pix, int w, int h, int x1, int x2, int y, uint16_t c)
{
    if (y < 0 || y >= h) return;
    if (x1 > x2) { int t = x1; x1 = x2; x2 = t; }
    if (x1 < 0) x1 = 0;
    if (x2 >= w) x2 = w - 1;
    for (int x = x1; x <= x2; x++) pix[y * w + x] = c;
}

static void draw_vline_565(uint16_t *pix, int w, int h, int x, int y1, int y2, uint16_t c)
{
    if (x < 0 || x >= w) return;
    if (y1 > y2) { int t = y1; y1 = y2; y2 = t; }
    if (y1 < 0) y1 = 0;
    if (y2 >= h) y2 = h - 1;
    for (int y = y1; y <= y2; y++) pix[y * w + x] = c;
}

void lpr_draw_rect_565(uint16_t *pix, int w, int h, const struct det_box *b, uint16_t c)
{
    for (int t = 0; t < 2; t++) {
        draw_hline_565(pix, w, h, b->x1, b->x2, b->y1 + t, c);
        draw_hline_565(pix, w, h, b->x1, b->x2, b->y2 - t, c);
        draw_vline_565(pix, w, h, b->x1 + t, b->y1, b->y2, c);
        draw_vline_565(pix, w, h, b->x2 - t, b->y1, b->y2, c);
    }
}

/* 5x7 ASCII glyph (subset of printable chars). Compact LUT, no malloc. */
static uint8_t glyph5x7(char ch, int row)
{
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
    switch (ch) {
    case '0': { static const uint8_t g[7]={0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; return g[row]; }
    case '1': { static const uint8_t g[7]={0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
    case '2': { static const uint8_t g[7]={0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}; return g[row]; }
    case '3': { static const uint8_t g[7]={0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case '4': { static const uint8_t g[7]={0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}; return g[row]; }
    case '5': { static const uint8_t g[7]={0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}; return g[row]; }
    case '6': { static const uint8_t g[7]={0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}; return g[row]; }
    case '7': { static const uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x08,0x08}; return g[row]; }
    case '8': { static const uint8_t g[7]={0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}; return g[row]; }
    case '9': { static const uint8_t g[7]={0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}; return g[row]; }
    case 'A': { static const uint8_t g[7]={0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'B': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}; return g[row]; }
    case 'C': { static const uint8_t g[7]={0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}; return g[row]; }
    case 'D': { static const uint8_t g[7]={0x1C,0x12,0x11,0x11,0x11,0x12,0x1C}; return g[row]; }
    case 'E': { static const uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}; return g[row]; }
    case 'F': { static const uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'G': { static const uint8_t g[7]={0x0F,0x10,0x10,0x13,0x11,0x11,0x0F}; return g[row]; }
    case 'H': { static const uint8_t g[7]={0x11,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'I': { static const uint8_t g[7]={0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
    case 'J': { static const uint8_t g[7]={0x01,0x01,0x01,0x01,0x11,0x11,0x0E}; return g[row]; }
    case 'K': { static const uint8_t g[7]={0x11,0x12,0x14,0x18,0x14,0x12,0x11}; return g[row]; }
    case 'L': { static const uint8_t g[7]={0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return g[row]; }
    case 'M': { static const uint8_t g[7]={0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; return g[row]; }
    case 'N': { static const uint8_t g[7]={0x11,0x19,0x15,0x13,0x11,0x11,0x11}; return g[row]; }
    case 'O': { static const uint8_t g[7]={0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'P': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'Q': { static const uint8_t g[7]={0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}; return g[row]; }
    case 'R': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return g[row]; }
    case 'S': { static const uint8_t g[7]={0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case 'T': { static const uint8_t g[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'U': { static const uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'V': { static const uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x0A,0x04}; return g[row]; }
    case 'W': { static const uint8_t g[7]={0x11,0x11,0x11,0x15,0x15,0x1B,0x11}; return g[row]; }
    case 'X': { static const uint8_t g[7]={0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}; return g[row]; }
    case 'Y': { static const uint8_t g[7]={0x11,0x11,0x0A,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'Z': { static const uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}; return g[row]; }
    case '-': { static const uint8_t g[7]={0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; return g[row]; }
    case '.': { static const uint8_t g[7]={0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}; return g[row]; }
    case ':': { static const uint8_t g[7]={0x00,0x04,0x04,0x00,0x04,0x04,0x00}; return g[row]; }
    case ' ': { static const uint8_t g[7]={0,0,0,0,0,0,0}; return g[row]; }
    default: return 0;
    }
}

void lpr_draw_text_565(uint16_t *pix, int w, int h, int x, int y, const char *s,
                       uint16_t c, int scale)
{
    if (!s || scale < 1) return;
    for (int i = 0; s[i] != '\0'; i++) {
        int ox = x + i * 6 * scale;
        for (int row = 0; row < 7; row++) {
            uint8_t bits = glyph5x7(s[i], row);
            for (int col = 0; col < 5; col++) {
                if (!(bits & (1U << (4 - col)))) continue;
                for (int sy = 0; sy < scale; sy++) {
                    int py = y + row * scale + sy;
                    if (py < 0 || py >= h) continue;
                    for (int sx = 0; sx < scale; sx++) {
                        int px = ox + col * scale + sx;
                        if (px >= 0 && px < w) pix[py * w + px] = c;
                    }
                }
            }
        }
    }
}

void lpr_overlay_ascii_from_text(const char *text, char *out, size_t out_len)
{
    size_t j = 0;
    if (!out || out_len == 0) return;
    if (!text) text = "";
    for (size_t i = 0; text[i] && j + 1 < out_len; i++) {
        unsigned char ch = (unsigned char)text[i];
        if ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z'))
            out[j++] = (char)ch;
    }
    out[j] = '\0';
}

/* ---------------- KMS pipeline lifecycle ---------------- */

int lpr_display_start(struct display_state *d, const struct live_options *opt,
                      uint32_t w, uint32_t h)
{
    GstCaps *caps;
    GstStateChangeReturn sret;
    memset(d, 0, sizeof(*d));
    d->enabled = opt->display;
    d->drm_fd = -1;
    if (!d->enabled) return 0;
    d->w = w; d->h = h; d->fps = opt->fps; d->connector_id = opt->connector_id; d->sync = opt->display_sync;
    d->frame_size = (size_t)w * (size_t)h * 2U;
    if (opt->drm_card_path && opt->drm_card_path[0]) {
        d->drm_fd = open(opt->drm_card_path, O_RDWR | O_CLOEXEC);
        if (d->drm_fd < 0)
            fprintf(stderr, "[display] warning: failed to open %s: %s\n", opt->drm_card_path, strerror(errno));
    }
    d->pipeline = gst_pipeline_new("pplcnet-bgp-live");
    d->appsrc = gst_element_factory_make("appsrc", "src");
    d->queue = gst_element_factory_make("queue", "queue");
    d->sink = gst_element_factory_make("kmssink", "sink");
    if (!d->pipeline || !d->appsrc || !d->queue || !d->sink) {
        fprintf(stderr, "[display] failed to create appsrc/queue/kmssink\n");
        return -1;
    }
    gst_bin_add_many(GST_BIN(d->pipeline), d->appsrc, d->queue, d->sink, NULL);
    if (!gst_element_link_many(d->appsrc, d->queue, d->sink, NULL)) {
        fprintf(stderr, "[display] failed to link appsrc -> queue -> kmssink\n");
        return -1;
    }
    caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "RGB16",
                               "width", G_TYPE_INT, (int)w, "height", G_TYPE_INT, (int)h,
                               "framerate", GST_TYPE_FRACTION, opt->fps, 1, NULL);
    if (!caps) return -1;
    g_object_set(d->appsrc, "caps", caps, "is-live", TRUE, "do-timestamp", TRUE,
                 "format", GST_FORMAT_TIME, "block", FALSE,
                 "max-bytes", (guint64)d->frame_size * 2U, NULL);
    gst_caps_unref(caps);
    g_object_set(d->queue, "max-size-buffers", 1, "max-size-bytes", 0,
                 "max-size-time", (guint64)0, "leaky", 2, NULL);
    g_object_set(d->sink, "sync", d->sync ? TRUE : FALSE, NULL);
    if (d->connector_id >= 0) g_object_set(d->sink, "connector-id", d->connector_id, NULL);
    if (d->drm_fd >= 0) g_object_set(d->sink, "fd", d->drm_fd, NULL);
    d->bus = gst_element_get_bus(d->pipeline);
    sret = gst_element_set_state(d->pipeline, GST_STATE_PLAYING);
    if (sret == GST_STATE_CHANGE_FAILURE) return -1;
    sret = gst_element_get_state(d->pipeline, NULL, NULL, 5 * GST_SECOND);
    if (sret == GST_STATE_CHANGE_FAILURE) return -1;
    fprintf(stderr, "[display] started appsrc RGB16 %ux%u -> kmssink sync=%d connector=%d\n",
            w, h, d->sync ? 1 : 0, d->connector_id);
    return 0;
}

void lpr_display_stop(struct display_state *d)
{
    if (!d || !d->enabled) return;
    if (d->appsrc) gst_app_src_end_of_stream(GST_APP_SRC(d->appsrc));
    if (d->pipeline) gst_element_set_state(d->pipeline, GST_STATE_NULL);
    if (d->bus) gst_object_unref(d->bus);
    if (d->pipeline) gst_object_unref(d->pipeline);
    if (d->drm_fd >= 0) close(d->drm_fd);
    memset(d, 0, sizeof(*d));
    d->drm_fd = -1;
}

static int handle_bus(struct display_state *d)
{
    GstMessage *msg;
    if (!d || !d->enabled || !d->bus) return 0;
    while ((msg = gst_bus_pop(d->bus)) != NULL) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError *err = NULL; gchar *dbg = NULL;
            gst_message_parse_error(msg, &err, &dbg);
            fprintf(stderr, "[display] error: %s\n", err ? err->message : "unknown");
            if (err) g_error_free(err);
            g_free(dbg);
            gst_message_unref(msg);
            return -1;
        }
        gst_message_unref(msg);
    }
    return 0;
}

int lpr_display_push(struct display_state *d, const uint16_t *frame)
{
    uint8_t *copy;
    GstBuffer *buf;
    GstFlowReturn flow;
    if (!d || !d->enabled) return 0;
    if (handle_bus(d) < 0) return -1;
    copy = g_malloc(d->frame_size);
    if (!copy) return -1;
    memcpy(copy, frame, d->frame_size);
    buf = gst_buffer_new_wrapped_full((GstMemoryFlags)0, copy, d->frame_size, 0, d->frame_size, copy, g_free);
    if (!buf) { g_free(copy); return -1; }
    GST_BUFFER_PTS(buf) = d->next_pts_ns;
    GST_BUFFER_DURATION(buf) = (guint64)(GST_SECOND / d->fps);
    d->next_pts_ns += GST_BUFFER_DURATION(buf);
    flow = gst_app_src_push_buffer(GST_APP_SRC(d->appsrc), buf);
    return flow == GST_FLOW_OK ? 0 : -1;
}
