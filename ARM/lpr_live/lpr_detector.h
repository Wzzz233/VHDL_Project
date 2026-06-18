// SPDX-License-Identifier: GPL-2.0
/* YOLOv8n-pose plate detector with NMS and quad output. */

#ifndef LPR_LIVE_LPR_DETECTOR_H
#define LPR_LIVE_LPR_DETECTOR_H

#include "lpr_common.h"

/* Inspect detector output to determine pose_nc (number of class channels).
 * Returns 0 if no compatible output is found. */
int lpr_detector_pose_nc(const struct rknn_model *m);

/* Run the detector on `rgb` (img_w x img_h, RGB888 packed).
 *
 * `input` is a caller-owned scratch buffer of size m->in_w*m->in_h*3 used to
 * stage the resized/letterboxed detector input. It is never returned to the
 * caller and may contain stale bytes after this call.
 *
 * `dets[0..MAX_DETS-1]` will be filled with up to `max_det` boxes after NMS.
 * `det_count` will be set to the number of boxes written. */
int lpr_detector_run(struct rknn_model *m, const uint8_t *rgb,
                     int img_w, int img_h, uint8_t *input,
                     enum det_resize_mode resize_mode,
                     int pose_nc, int class_filter,
                     float conf_thr, float nms_iou, int max_det,
                     struct det_box *dets, int *det_count);

/* Pick highest-confidence box index, or -1 if no boxes. */
int lpr_detector_pick_best(const struct det_box *dets, int count);

#endif /* LPR_LIVE_LPR_DETECTOR_H */
