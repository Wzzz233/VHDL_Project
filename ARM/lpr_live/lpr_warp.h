// SPDX-License-Identifier: GPL-2.0
/* 4-point homography warp from detected quad to a flat plate crop. */

#ifndef LPR_LIVE_LPR_WARP_H
#define LPR_LIVE_LPR_WARP_H

#include "lpr_common.h"

/* Warp quad-bounded plate region from RGB888 source to a flat RGB888 crop.
 *
 * `quad_in` is 4 (x, y) corners in any winding order; the function reorders
 * them TL/TR/BR/BL internally and rejects degenerate quads.
 *
 * `dst` must be at least cap_w * cap_h * 3 bytes; the actual output size
 * `(*out_w, *out_h)` is derived from the quad's bilateral max edge length
 * and clipped to (cap_w, cap_h).
 *
 * Returns true on success. */
bool lpr_warp_quad_homography(const uint8_t *rgb, int img_w, int img_h,
                              const float quad_in[8],
                              uint8_t *dst, int cap_w, int cap_h,
                              int *out_w, int *out_h);
bool lpr_warp_quad_homography_bgrx(const uint8_t *bgrx, int img_w, int img_h,
                                   const float quad_in[8],
                                   uint8_t *dst, int cap_w, int cap_h,
                                   int *out_w, int *out_h);

#endif /* LPR_LIVE_LPR_WARP_H */
