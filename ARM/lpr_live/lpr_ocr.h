// SPDX-License-Identifier: GPL-2.0
/* PPLCNet RKNN OCR: input prep, model run, CTC decode with layout autodetect. */

#ifndef LPR_LIVE_LPR_OCR_H
#define LPR_LIVE_LPR_OCR_H

#include "lpr_common.h"
#include "../ocr_decode.h"

/* Load an RKNN model file and populate the rknn_model wrapper, including
 * input/output tensor attribute introspection used by detector and OCR
 * layout selection. */
int lpr_model_load(struct rknn_model *m, const char *name, const char *path);
void lpr_model_release(struct rknn_model *m);

/* Run an OCR forward pass on `crop_rgb` (crop_w x crop_h, RGB888 packed),
 * decode logits with the keys table, and return the recognized text and
 * confidence. `family` selects the CTC postprocess variant
 * (NORMAL7 / GREEN8). `timing` is optional; if non-NULL it is filled with
 * per-stage breakdown. Returns 0 on success. */
int lpr_ocr_run(struct rknn_model *m, const struct ocr_keys *keys,
                enum ocr_preproc_mode preproc, enum ocr_decode_family family,
                const uint8_t *crop_rgb, int crop_w, int crop_h,
                char *text, size_t text_len, float *conf,
                struct ocr_decode_diag *diag, struct ocr_timing *timing);

/* Log the keys/route/layout binding for diagnostics. */
void lpr_ocr_log_contract(const char *route_name,
                          const struct rknn_model *m,
                          const struct ocr_keys *keys);

#endif /* LPR_LIVE_LPR_OCR_H */
