// SPDX-License-Identifier: GPL-2.0
/* Plate type classifier RKNN: BGPEY route override helper. */

#ifndef LPR_LIVE_LPR_PTYPE_H
#define LPR_LIVE_LPR_PTYPE_H

#include "lpr_common.h"

#define LPR_PTYPE_CLASS_COUNT 6

enum lpr_ptype_class {
    LPR_PTYPE_BLUE = 0,
    LPR_PTYPE_GREEN,
    LPR_PTYPE_YELLOW,
    LPR_PTYPE_POLICE,
    LPR_PTYPE_EMBASSY,
    LPR_PTYPE_OTHER,
    LPR_PTYPE_UNKNOWN = -1,
};

const char *lpr_ptype_class_str(int cls);

/* Run the 6-class plate-type model on a warped RGB plate crop. The model is
 * expected to take uint8 NHWC RGB and output logits for:
 * blue, green, yellow, police, embassy, other. */
int lpr_ptype_run(struct rknn_model *m,
                  const uint8_t *plate_rgb, int plate_w, int plate_h,
                  int *cls_out, float *conf_out, double *timing_ms);

#endif /* LPR_LIVE_LPR_PTYPE_H */
