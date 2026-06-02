#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define RKNN_TENSOR_NCHW 0
#define RKNN_TENSOR_NHWC 1

struct tensor_attr_like {
    int n_dims;
    int dims[4];
    int fmt;
};

static bool dim_matches_ocr_class_count(int dim, int key_count)
{
    return dim > 1 && key_count > 0 && (dim == key_count || dim == key_count + 1);
}

static void set_ocr_layout_ct(int c, int t, int *t_size, int *c_size, int *t_stride, int *c_stride)
{
    *c_size = c;
    *t_size = t;
    *t_stride = 1;
    *c_stride = *t_size;
}

static void set_ocr_layout_tc(int t, int c, int *t_size, int *c_size, int *t_stride, int *c_stride)
{
    *t_size = t;
    *c_size = c;
    *t_stride = *c_size;
    *c_stride = 1;
}

static bool build_ocr_layout_like(const struct tensor_attr_like *a, int key_count,
                                  int *t_size, int *c_size, int *t_stride, int *c_stride)
{
    if (a->n_dims == 2) {
        int d0 = a->dims[0];
        int d1 = a->dims[1];
        bool d0_class;
        bool d1_class;
        if (d0 <= 0 || d1 <= 1)
            return false;
        d0_class = dim_matches_ocr_class_count(d0, key_count);
        d1_class = dim_matches_ocr_class_count(d1, key_count);
        if (d0_class && !d1_class)
            set_ocr_layout_ct(d0, d1, t_size, c_size, t_stride, c_stride);
        else if (d1_class && !d0_class)
            set_ocr_layout_tc(d0, d1, t_size, c_size, t_stride, c_stride);
        else if (d1 == 18 && d0 != 18)
            set_ocr_layout_ct(d0, d1, t_size, c_size, t_stride, c_stride);
        else if (d0 == 18 && d1 != 18)
            set_ocr_layout_tc(d0, d1, t_size, c_size, t_stride, c_stride);
        else if (d0 > d1)
            set_ocr_layout_ct(d0, d1, t_size, c_size, t_stride, c_stride);
        else
            set_ocr_layout_tc(d0, d1, t_size, c_size, t_stride, c_stride);
        return (*t_size > 0 && *c_size > 1);
    }
    if (a->n_dims == 3) {
        int d1 = a->dims[1];
        int d2 = a->dims[2];
        bool d1_class;
        bool d2_class;
        if (d1 <= 0 || d2 <= 1)
            return false;
        d1_class = dim_matches_ocr_class_count(d1, key_count);
        d2_class = dim_matches_ocr_class_count(d2, key_count);
        if (d1_class && !d2_class)
            set_ocr_layout_ct(d1, d2, t_size, c_size, t_stride, c_stride);
        else if (d2_class && !d1_class)
            set_ocr_layout_tc(d1, d2, t_size, c_size, t_stride, c_stride);
        else if (d2 == 18 && d1 != 18)
            set_ocr_layout_ct(d1, d2, t_size, c_size, t_stride, c_stride);
        else if (d1 == 18 && d2 != 18)
            set_ocr_layout_tc(d1, d2, t_size, c_size, t_stride, c_stride);
        else if (a->fmt == RKNN_TENSOR_NCHW)
            set_ocr_layout_ct(d1, d2, t_size, c_size, t_stride, c_stride);
        else if (a->fmt == RKNN_TENSOR_NHWC)
            set_ocr_layout_tc(d1, d2, t_size, c_size, t_stride, c_stride);
        else if (d1 > d2)
            set_ocr_layout_ct(d1, d2, t_size, c_size, t_stride, c_stride);
        else
            set_ocr_layout_tc(d1, d2, t_size, c_size, t_stride, c_stride);
        return true;
    }
    return false;
}

static void expect_layout(const char *name, struct tensor_attr_like a, int key_count,
                          int want_t, int want_c, int want_t_stride, int want_c_stride)
{
    int t = 0, c = 0, ts = 0, cs = 0;
    if (!build_ocr_layout_like(&a, key_count, &t, &c, &ts, &cs)) {
        fprintf(stderr, "[FAIL] %s layout rejected\n", name);
        exit(1);
    }
    if (t != want_t || c != want_c || ts != want_t_stride || cs != want_c_stride) {
        fprintf(stderr, "[FAIL] %s got t=%d c=%d ts=%d cs=%d want t=%d c=%d ts=%d cs=%d\n",
                name, t, c, ts, cs, want_t, want_c, want_t_stride, want_c_stride);
        exit(1);
    }
}

int main(void)
{
    expect_layout("embassy_2d_ct", (struct tensor_attr_like){2, {12, 18, 0, 0}, RKNN_TENSOR_NCHW},
                  11, 18, 12, 1, 18);
    expect_layout("police_2d_ct", (struct tensor_attr_like){2, {67, 18, 0, 0}, RKNN_TENSOR_NCHW},
                  66, 18, 67, 1, 18);
    expect_layout("yellow_2d_ct", (struct tensor_attr_like){2, {70, 18, 0, 0}, RKNN_TENSOR_NCHW},
                  69, 18, 70, 1, 18);
    expect_layout("embassy_3d_nchw", (struct tensor_attr_like){3, {1, 12, 18, 0}, RKNN_TENSOR_NCHW},
                  11, 18, 12, 1, 18);
    expect_layout("green_3d_nhwc", (struct tensor_attr_like){3, {1, 18, 70, 0}, RKNN_TENSOR_NHWC},
                  69, 18, 70, 70, 1);
    printf("[PASS] test_ocr_layout\n");
    return 0;
}
