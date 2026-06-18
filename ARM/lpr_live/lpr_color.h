// SPDX-License-Identifier: GPL-2.0
/* RGB-based plate body color classification (blue/green/yellow/white). */

#ifndef LPR_LIVE_LPR_COLOR_H
#define LPR_LIVE_LPR_COLOR_H

#include "lpr_common.h"

/* Classify the dominant body color of the plate region inside box `b`,
 * sampled directly from the source RGB888 frame. The function uses HSV
 * thresholds robust to mild lighting variation; returns
 * PLATE_COLOR_UNKNOWN if no clear vote is cast. */
enum plate_color lpr_classify_plate_color(const uint8_t *rgb, int w, int h,
                                          const struct det_box *b);

#endif /* LPR_LIVE_LPR_COLOR_H */
