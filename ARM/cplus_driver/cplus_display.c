// SPDX-License-Identifier: MIT
#define _GNU_SOURCE
/* Direct DRM/KMS display and BGRX overlay, derived from the existing ARM path. */

#include "cplus_display.h"

#include <drm_fourcc.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

static void set_pixel(uint8_t *pixels, int stride, int width, int height, int x, int y,
                      uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t *pixel;
    if (x < 0 || x >= width || y < 0 || y >= height) return;
    pixel = pixels + (size_t)y * (size_t)stride + (size_t)x * 4U;
    pixel[0] = blue;
    pixel[1] = green;
    pixel[2] = red;
}

static void draw_rect(uint8_t *pixels, int stride, int width, int height, const struct cplus_box *box,
                      uint8_t red, uint8_t green, uint8_t blue)
{
    int x1 = (int)lroundf(box->x1);
    int y1 = (int)lroundf(box->y1);
    int x2 = (int)lroundf(box->x2);
    int y2 = (int)lroundf(box->y2);
    int thickness;
    int x;
    int y;
    for (thickness = 0; thickness < 3; ++thickness) {
        for (x = x1; x <= x2; ++x) {
            set_pixel(pixels, stride, width, height, x, y1 + thickness, red, green, blue);
            set_pixel(pixels, stride, width, height, x, y2 - thickness, red, green, blue);
        }
        for (y = y1; y <= y2; ++y) {
            set_pixel(pixels, stride, width, height, x1 + thickness, y, red, green, blue);
            set_pixel(pixels, stride, width, height, x2 - thickness, y, red, green, blue);
        }
    }
}

static void draw_source_rect(uint8_t *pixels, int stride, int width, int height,
                             const struct cplus_rect *rect,
                             uint8_t red, uint8_t green, uint8_t blue)
{
    struct cplus_box box;
    if (!rect) return;
    memset(&box, 0, sizeof(box));
    box.x1 = (float)rect->x1;
    box.y1 = (float)rect->y1;
    box.x2 = (float)rect->x2;
    box.y2 = (float)rect->y2;
    draw_rect(pixels, stride, width, height, &box, red, green, blue);
}

static uint8_t glyph5x7(char character, int row)
{
    if (character >= 'a' && character <= 'z') character = (char)(character - 'a' + 'A');
    switch (character) {
    case '0': { static const uint8_t g[7] = {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; return g[row]; }
    case '1': { static const uint8_t g[7] = {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
    case '2': { static const uint8_t g[7] = {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}; return g[row]; }
    case '3': { static const uint8_t g[7] = {0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case '4': { static const uint8_t g[7] = {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}; return g[row]; }
    case '5': { static const uint8_t g[7] = {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}; return g[row]; }
    case '6': { static const uint8_t g[7] = {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}; return g[row]; }
    case '7': { static const uint8_t g[7] = {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}; return g[row]; }
    case '8': { static const uint8_t g[7] = {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}; return g[row]; }
    case '9': { static const uint8_t g[7] = {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}; return g[row]; }
    case 'A': { static const uint8_t g[7] = {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'B': { static const uint8_t g[7] = {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}; return g[row]; }
    case 'C': { static const uint8_t g[7] = {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}; return g[row]; }
    case 'D': { static const uint8_t g[7] = {0x1C,0x12,0x11,0x11,0x11,0x12,0x1C}; return g[row]; }
    case 'E': { static const uint8_t g[7] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}; return g[row]; }
    case 'F': { static const uint8_t g[7] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'G': { static const uint8_t g[7] = {0x0F,0x10,0x10,0x13,0x11,0x11,0x0F}; return g[row]; }
    case 'H': { static const uint8_t g[7] = {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'I': { static const uint8_t g[7] = {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
    case 'J': { static const uint8_t g[7] = {0x01,0x01,0x01,0x01,0x11,0x11,0x0E}; return g[row]; }
    case 'K': { static const uint8_t g[7] = {0x11,0x12,0x14,0x18,0x14,0x12,0x11}; return g[row]; }
    case 'L': { static const uint8_t g[7] = {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return g[row]; }
    case 'M': { static const uint8_t g[7] = {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; return g[row]; }
    case 'N': { static const uint8_t g[7] = {0x11,0x19,0x15,0x13,0x11,0x11,0x11}; return g[row]; }
    case 'O': { static const uint8_t g[7] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'P': { static const uint8_t g[7] = {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'R': { static const uint8_t g[7] = {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return g[row]; }
    case 'S': { static const uint8_t g[7] = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case 'T': { static const uint8_t g[7] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'U': { static const uint8_t g[7] = {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'V': { static const uint8_t g[7] = {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}; return g[row]; }
    case 'W': { static const uint8_t g[7] = {0x11,0x11,0x11,0x15,0x15,0x1B,0x11}; return g[row]; }
    case 'X': { static const uint8_t g[7] = {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}; return g[row]; }
    case 'Y': { static const uint8_t g[7] = {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'Z': { static const uint8_t g[7] = {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}; return g[row]; }
    case '-': { static const uint8_t g[7] = {0,0,0,0x1F,0,0,0}; return g[row]; }
    case ':': { static const uint8_t g[7] = {0,0x04,0x04,0,0x04,0x04,0}; return g[row]; }
    case ' ': { static const uint8_t g[7] = {0,0,0,0,0,0,0}; return g[row]; }
    default: return 0;
    }
}

static void draw_text(uint8_t *pixels, int stride, int width, int height, int x, int y, const char *text,
                      uint8_t red, uint8_t green, uint8_t blue)
{
    int cursor = x;
    size_t index;
    if (!text) return;
    for (index = 0; text[index] != '\0'; ++index) {
        int row;
        for (row = 0; row < 7; ++row) {
            uint8_t bits = glyph5x7(text[index], row);
            int column;
            for (column = 0; column < 5; ++column) {
                if (bits & (1U << (4 - column))) {
                    set_pixel(pixels, stride, width, height, cursor + column, y + row, red, green, blue);
                    set_pixel(pixels, stride, width, height, cursor + column + 1, y + row, red, green, blue);
                }
            }
        }
        cursor += 7;
    }
}

static const char *overlay_label(const struct cplus_person_result *result)
{
    if (result->decision == CPLUS_DECISION_SUSPECTED) return "SUSPECT";
    if (result->detection.box.class_id == CPLUS_COCO_BICYCLE ||
        result->detection.box.class_id == CPLUS_COCO_MOTORCYCLE) return "VEHICLE";
    if (result->detection.target_type != CPLUS_TARGET_PEDESTRIAN) return "RIDER";
    switch (result->reason) {
    case CPLUS_REASON_ZEBRA_SUPPRESSED: return "ZEBRA";
    case CPLUS_REASON_SIDEWALK_SUPPRESSED: return "SIDEWALK";
    case CPLUS_REASON_WEAK_GROUND_EVIDENCE: return "NO-GROUND";
    default: return "CLEAR";
    }
}

void cplus_overlay_results(uint8_t *bgrx, int stride, int width, int height,
                           const struct cplus_person_result *results, int count,
                           bool result_available)
{
    int index;
    for (index = 0; index < count; ++index) {
        const struct cplus_person_result *result = &results[index];
        uint8_t red = 40;
        uint8_t green = 220;
        uint8_t blue = 40;
        int text_y;
        if (result->decision == CPLUS_DECISION_SUSPECTED) {
            red = 240; green = 50; blue = 40;
        } else if (result->detection.target_type != CPLUS_TARGET_PEDESTRIAN) {
            red = 190; green = 70; blue = 230;
        } else if (result->detection.box.class_id != CPLUS_COCO_PERSON) {
            red = 250; green = 180; blue = 30;
        }
        draw_rect(bgrx, stride, width, height, &result->detection.box, red, green, blue);
        if (result->detection.box.class_id == CPLUS_COCO_PERSON &&
            result->detection.target_type == CPLUS_TARGET_PEDESTRIAN)
            draw_source_rect(bgrx, stride, width, height, &result->foot_box_source,
                             255, 220, 40);
        text_y = (int)lroundf(result->detection.box.y1) - 11;
        if (text_y < 1) text_y = (int)lroundf(result->detection.box.y2) + 3;
        draw_text(bgrx, stride, width, height, (int)lroundf(result->detection.box.x1), text_y,
                  overlay_label(result), red, green, blue);
    }
    draw_text(bgrx, stride, width, height, 12, 12, result_available ?
              (count ? "CPLUS" : "CPLUS NO TARGET") : "CPLUS STARTING", 255, 255, 255);
}

static drmModeConnector *find_connector(int fd, drmModeRes *resources, int requested_id)
{
    int index;
    for (index = 0; index < resources->count_connectors; ++index) {
        drmModeConnector *connector = drmModeGetConnector(fd, resources->connectors[index]);
        if (connector && (requested_id < 0 || (int)connector->connector_id == requested_id) &&
            connector->connection == DRM_MODE_CONNECTED && connector->count_modes > 0)
            return connector;
        if (connector) drmModeFreeConnector(connector);
    }
    return NULL;
}

static uint32_t find_crtc(int fd, drmModeRes *resources, drmModeConnector *connector)
{
    drmModeEncoder *encoder;
    int index;
    if (connector->encoder_id) {
        encoder = drmModeGetEncoder(fd, connector->encoder_id);
        if (encoder) {
            uint32_t crtc = encoder->crtc_id;
            drmModeFreeEncoder(encoder);
            if (crtc) return crtc;
        }
    }
    for (index = 0; index < connector->count_encoders; ++index) {
        int crtc_index;
        encoder = drmModeGetEncoder(fd, connector->encoders[index]);
        if (!encoder) continue;
        for (crtc_index = 0; crtc_index < resources->count_crtcs; ++crtc_index) {
            if (encoder->possible_crtcs & (1U << crtc_index)) {
                uint32_t crtc = resources->crtcs[crtc_index];
                drmModeFreeEncoder(encoder);
                return crtc;
            }
        }
        drmModeFreeEncoder(encoder);
    }
    return 0;
}

static int create_framebuffer(int fd, int width, int height, struct cplus_drm_fb *framebuffer)
{
    struct drm_mode_create_dumb create;
    struct drm_mode_map_dumb map;
    uint32_t handles[4] = {0};
    uint32_t pitches[4] = {0};
    uint32_t offsets[4] = {0};
    memset(framebuffer, 0, sizeof(*framebuffer));
    memset(&create, 0, sizeof(create));
    create.width = (uint32_t)width;
    create.height = (uint32_t)height;
    create.bpp = 32;
    if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) return -1;
    framebuffer->handle = create.handle;
    framebuffer->pitch = create.pitch;
    framebuffer->size = create.size;
    handles[0] = framebuffer->handle;
    pitches[0] = framebuffer->pitch;
    if (drmModeAddFB2(fd, (uint32_t)width, (uint32_t)height, DRM_FORMAT_XRGB8888,
                      handles, pitches, offsets, &framebuffer->fb_id, 0) < 0) return -1;
    memset(&map, 0, sizeof(map));
    map.handle = framebuffer->handle;
    if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &map) < 0) return -1;
    framebuffer->map = mmap(NULL, framebuffer->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)map.offset);
    if (framebuffer->map == MAP_FAILED) {
        framebuffer->map = NULL;
        return -1;
    }
    memset(framebuffer->map, 0, framebuffer->size);
    return 0;
}

static void destroy_framebuffer(int fd, struct cplus_drm_fb *framebuffer)
{
    struct drm_mode_destroy_dumb destroy;
    if (!framebuffer) return;
    if (framebuffer->map) munmap(framebuffer->map, framebuffer->size);
    if (framebuffer->fb_id) drmModeRmFB(fd, framebuffer->fb_id);
    if (framebuffer->handle) {
        memset(&destroy, 0, sizeof(destroy));
        destroy.handle = framebuffer->handle;
        ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    }
    memset(framebuffer, 0, sizeof(*framebuffer));
}

static void page_flip_handler(int fd, unsigned int frame, unsigned int seconds,
                              unsigned int microseconds, void *data)
{
    bool *waiting = data;
    (void)fd; (void)frame; (void)seconds; (void)microseconds;
    *waiting = false;
}

static int wait_for_page_flip(int fd, bool *waiting)
{
    drmEventContext events;
    memset(&events, 0, sizeof(events));
    events.version = DRM_EVENT_CONTEXT_VERSION;
    events.page_flip_handler = page_flip_handler;
    while (*waiting) {
        fd_set descriptors;
        struct timeval timeout = { .tv_sec = 1, .tv_usec = 0 };
        int result;
        FD_ZERO(&descriptors);
        FD_SET(fd, &descriptors);
        result = select(fd + 1, &descriptors, NULL, NULL, &timeout);
        if (result == 0) { errno = ETIMEDOUT; return -1; }
        if (result < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (drmHandleEvent(fd, &events) < 0) return -1;
    }
    return 0;
}

int cplus_display_start(struct cplus_display *display, const char *drm_card,
                        int requested_connector, int width, int height)
{
    drmModeRes *resources = NULL;
    drmModeConnector *connector = NULL;
    int index;
    if (!display || !drm_card || width <= 0 || height <= 0) return -1;
    memset(display, 0, sizeof(*display));
    display->fd = -1;
    display->active_fb = -1;
    display->fd = open(drm_card, O_RDWR | O_CLOEXEC);
    if (display->fd < 0) return -1;
    resources = drmModeGetResources(display->fd);
    connector = resources ? find_connector(display->fd, resources, requested_connector) : NULL;
    if (!connector) goto failed;
    for (index = 0; index < connector->count_modes; ++index) {
        if (connector->modes[index].hdisplay == width && connector->modes[index].vdisplay == height) {
            display->mode = connector->modes[index];
            break;
        }
    }
    if (index == connector->count_modes) { errno = EINVAL; goto failed; }
    display->crtc_id = find_crtc(display->fd, resources, connector);
    display->connector_id = connector->connector_id;
    if (!display->crtc_id) { errno = ENODEV; goto failed; }
    display->saved_crtc = drmModeGetCrtc(display->fd, display->crtc_id);
    display->width = width;
    display->height = height;
    display->render_size = (size_t)width * (size_t)height * 4U;
    display->render_buffer = malloc(display->render_size);
    if (!display->render_buffer) goto failed;
    if (create_framebuffer(display->fd, width, height, &display->fb[0]) < 0 ||
        create_framebuffer(display->fd, width, height, &display->fb[1]) < 0) goto failed;
    display->started = true;
    fprintf(stderr, "[display] DRM connector=%u crtc=%u mode=%dx%d\n",
            display->connector_id, display->crtc_id, width, height);
    drmModeFreeConnector(connector);
    drmModeFreeResources(resources);
    return 0;
failed:
    if (connector) drmModeFreeConnector(connector);
    if (resources) drmModeFreeResources(resources);
    cplus_display_stop(display);
    return -1;
}

int cplus_display_present(struct cplus_display *display, const uint8_t *bgrx,
                          const struct cplus_person_result *results, int count,
                          bool result_available,
                          const uint8_t *mask, bool mask_available)
{
    int next;
    int row;
    bool waiting = true;
    uint8_t *rendered;
    struct cplus_drm_fb *framebuffer;
    if (!display || !display->started || !bgrx || count < 0 ||
        count > CPLUS_MAX_DETECTIONS || (count > 0 && !results) ||
        (mask_available && !mask)) return -1;
    next = display->active_fb < 0 ? 0 : 1 - display->active_fb;
    framebuffer = &display->fb[next];
    rendered = display->render_buffer;
    for (row = 0; row < display->height; ++row)
        memcpy(rendered + (size_t)row * display->width * 4U,
               bgrx + (size_t)row * display->width * 4U,
               (size_t)display->width * 4U);
    if (mask_available)
        cplus_overlay_mask_bgrx(rendered, display->width * 4,
                                display->width, display->height, mask,
                                CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT);
    cplus_overlay_results(rendered, display->width * 4, display->width, display->height,
                          results, count, result_available);
    for (row = 0; row < display->height; ++row)
        memcpy(framebuffer->map + (size_t)row * framebuffer->pitch,
               rendered + (size_t)row * display->width * 4U,
               (size_t)display->width * 4U);
    if (display->active_fb < 0) {
        if (drmModeSetCrtc(display->fd, display->crtc_id, framebuffer->fb_id, 0, 0,
                           &display->connector_id, 1, &display->mode) < 0) return -1;
    } else {
        if (drmModePageFlip(display->fd, display->crtc_id, framebuffer->fb_id,
                            DRM_MODE_PAGE_FLIP_EVENT, &waiting) < 0 ||
            wait_for_page_flip(display->fd, &waiting) < 0) return -1;
    }
    display->active_fb = next;
    return 0;
}

void cplus_display_stop(struct cplus_display *display)
{
    drmModeCrtc *saved;
    if (!display) return;
    saved = display->saved_crtc;
    if (display->fd >= 0 && saved) {
        drmModeSetCrtc(display->fd, saved->crtc_id, saved->buffer_id, saved->x, saved->y,
                       &display->connector_id, 1, &saved->mode);
        drmModeFreeCrtc(saved);
    }
    if (display->fd >= 0) {
        destroy_framebuffer(display->fd, &display->fb[0]);
        destroy_framebuffer(display->fd, &display->fb[1]);
        close(display->fd);
    }
    free(display->render_buffer);
    memset(display, 0, sizeof(*display));
    display->fd = -1;
    display->active_fb = -1;
}
