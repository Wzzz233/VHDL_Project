// SPDX-License-Identifier: MIT

#ifndef CPLUS_DRIVER_RKNN_H
#define CPLUS_DRIVER_RKNN_H

#include <stddef.h>
#include <stdint.h>

#include <rknn_api.h>

struct cplus_rknn_output_view {
    const void *data;
    size_t element_count;
    rknn_tensor_type type;
    int64_t npu_duration_us;
};

struct cplus_rknn_model {
    const char *name;
    rknn_context context;
    rknn_input_output_num io_num;
    rknn_tensor_attr input_attr;
    rknn_tensor_attr output_attr;
    uint32_t input_width;
    uint32_t input_height;
    uint32_t input_channels;
    rknn_output output;
    int output_active;
};

int cplus_rknn_model_load(struct cplus_rknn_model *model, const char *name, const char *path);
void cplus_rknn_model_release(struct cplus_rknn_model *model);
int cplus_rknn_infer_rgb(struct cplus_rknn_model *model, const uint8_t *rgb,
                         uint32_t width, uint32_t height,
                         struct cplus_rknn_output_view *output);
void cplus_rknn_release_output(struct cplus_rknn_model *model);

#endif /* CPLUS_DRIVER_RKNN_H */
