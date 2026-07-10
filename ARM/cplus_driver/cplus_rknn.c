// SPDX-License-Identifier: MIT

#include "cplus_rknn.h"

#include <stdio.h>
#include <string.h>

static const char *tensor_format_name(rknn_tensor_format format)
{
    if (format == RKNN_TENSOR_NCHW) return "NCHW";
    if (format == RKNN_TENSOR_NHWC) return "NHWC";
    return "other";
}

static const char *tensor_type_name(rknn_tensor_type type)
{
    switch (type) {
    case RKNN_TENSOR_UINT8: return "uint8";
    case RKNN_TENSOR_INT8: return "int8";
    case RKNN_TENSOR_FLOAT16: return "float16";
    case RKNN_TENSOR_FLOAT32: return "float32";
    default: return "other";
    }
}

int cplus_rknn_model_load(struct cplus_rknn_model *model, const char *name, const char *path)
{
    int ret;
    if (!model || !name || !path || path[0] == '\0') return -1;
    memset(model, 0, sizeof(*model));
    model->name = name;
    ret = rknn_init(&model->context, (void *)path, 0, 0, NULL);
    if (ret < 0) {
        fprintf(stderr, "[%s] rknn_init(%s) failed: %d\n", name, path, ret);
        return -1;
    }
    ret = rknn_query(model->context, RKNN_QUERY_IN_OUT_NUM, &model->io_num, sizeof(model->io_num));
    if (ret < 0 || model->io_num.n_input != 1 || model->io_num.n_output != 1) {
        fprintf(stderr, "[%s] expected one input and one output, got %u/%u\n", name,
                model->io_num.n_input, model->io_num.n_output);
        cplus_rknn_model_release(model);
        return -1;
    }
    model->input_attr.index = 0;
    model->output_attr.index = 0;
    if (rknn_query(model->context, RKNN_QUERY_INPUT_ATTR, &model->input_attr, sizeof(model->input_attr)) < 0 ||
        rknn_query(model->context, RKNN_QUERY_OUTPUT_ATTR, &model->output_attr, sizeof(model->output_attr)) < 0) {
        fprintf(stderr, "[%s] failed to query tensor attributes\n", name);
        cplus_rknn_model_release(model);
        return -1;
    }
    if (model->input_attr.n_dims != 4) {
        fprintf(stderr, "[%s] expected a four-dimensional input, got %u dimensions\n", name,
                model->input_attr.n_dims);
        cplus_rknn_model_release(model);
        return -1;
    }
    if (model->input_attr.fmt == RKNN_TENSOR_NCHW) {
        model->input_channels = model->input_attr.dims[1];
        model->input_height = model->input_attr.dims[2];
        model->input_width = model->input_attr.dims[3];
    } else {
        model->input_height = model->input_attr.dims[1];
        model->input_width = model->input_attr.dims[2];
        model->input_channels = model->input_attr.dims[3];
    }
    fprintf(stderr,
            "[%s] input=%ux%ux%u format=%s type=%s output_dims=%u [%u,%u,%u,%u]\n",
            name, model->input_width, model->input_height, model->input_channels,
            tensor_format_name(model->input_attr.fmt), tensor_type_name(model->input_attr.type),
            model->output_attr.n_dims, model->output_attr.dims[0], model->output_attr.dims[1],
            model->output_attr.dims[2], model->output_attr.dims[3]);
    return 0;
}

void cplus_rknn_release_output(struct cplus_rknn_model *model)
{
    if (!model || !model->output_active) return;
    rknn_outputs_release(model->context, 1, &model->output);
    memset(&model->output, 0, sizeof(model->output));
    model->output_active = 0;
}

void cplus_rknn_model_release(struct cplus_rknn_model *model)
{
    if (!model) return;
    cplus_rknn_release_output(model);
    if (model->context) rknn_destroy(model->context);
    memset(model, 0, sizeof(*model));
}

int cplus_rknn_infer_rgb(struct cplus_rknn_model *model, const uint8_t *rgb,
                          uint32_t width, uint32_t height,
                          const float **output, size_t *float_count)
{
    rknn_input input;
    int ret;
    if (!model || !model->context || !rgb || !output || !float_count ||
        width != model->input_width || height != model->input_height || model->input_channels != 3) return -1;
    cplus_rknn_release_output(model);
    memset(&input, 0, sizeof(input));
    input.index = 0;
    input.buf = (void *)rgb;
    input.size = width * height * 3U;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;
    input.pass_through = 0;
    ret = rknn_inputs_set(model->context, 1, &input);
    if (ret < 0) return -1;
    ret = rknn_run(model->context, NULL);
    if (ret < 0) return -1;
    memset(&model->output, 0, sizeof(model->output));
    model->output.index = 0;
    model->output.want_float = 1;
    ret = rknn_outputs_get(model->context, 1, &model->output, NULL);
    if (ret < 0 || !model->output.buf || model->output.size < sizeof(float)) return -1;
    model->output_active = 1;
    *output = model->output.buf;
    *float_count = model->output.size / sizeof(float);
    return 0;
}
