#ifndef OCR_DECODE_H
#define OCR_DECODE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ocr_decode_diag {
    int t_size;
    int c_size;
    int blank_idx;
    float blank_top1_ratio;
};

enum ocr_decode_family {
    OCR_DECODE_FAMILY_NONE = 0,
    OCR_DECODE_FAMILY_GREEN8,
    OCR_DECODE_FAMILY_NORMAL7,
};

int ocr_decode_logits(const float *buf, int t_size, int c_size, int t_stride, int c_stride,
                      const char *const *keys, int key_count, int blank_idx,
                      enum ocr_decode_family family,
                      char *text, size_t text_len, float *conf_out,
                      struct ocr_decode_diag *diag);

#ifdef __cplusplus
}
#endif

#endif
