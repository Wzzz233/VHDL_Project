// SPDX-License-Identifier: MIT

#ifndef CPLUS_DRIVER_DISPLAY_H
#define CPLUS_DRIVER_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#include <xf86drmMode.h>

#include "cplus_core.h"

struct cplus_drm_fb {
    uint32_t handle;
    uint32_t fb_id;
    uint32_t pitch;
    uint64_t size;
    uint8_t *map;
};

struct cplus_display {
    int fd;
    uint32_t connector_id;
    uint32_t crtc_id;
    int width;
    int height;
    int active_fb;
    bool started;
    drmModeCrtc *saved_crtc;
    drmModeModeInfo mode;
    struct cplus_drm_fb fb[2];
};

int cplus_display_start(struct cplus_display *display, const char *drm_card,
                        int connector_id, int width, int height);
int cplus_display_present(struct cplus_display *display, const uint8_t *bgrx,
                          const struct cplus_person_result *results, int count,
                          bool result_available);
void cplus_display_stop(struct cplus_display *display);
void cplus_overlay_results(uint8_t *bgrx, int width, int height,
                           const struct cplus_person_result *results, int count,
                           bool result_available);

#endif /* CPLUS_DRIVER_DISPLAY_H */
