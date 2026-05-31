/*
 * test_pose_decoder.c — Unit tests for YOLOv8-pose multi-class decoding
 *
 * Compile (host): gcc -Wall -Wextra -O2 -std=c11 -o test_pose_decoder \
 *                     test_pose_decoder.c -lm
 * Run: ./test_pose_decoder
 *
 * Tests decode logic extracted from fpga_lpr_display.c:
 *   - Old model [1,17,8400] → det.cls == 0
 *   - New 5-class [1,21,8400] → det.cls == best class
 *   - Class 4 max → det.cls == 4
 *   - Class 3 max → det.cls == 3
 *   - Score sigmoid fallback
 *   - kpt offset: base=5 for nc==1, base=9 for nc==5
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ──────────── extracted constants ──────────── */
#define OBB_POINT_COUNT 8400
#define POSE_KPT_COUNT 4
#define POSE_KPT_DIMS 3
#define POSE_BOX_CHANNELS 4
#define POSE_KPT_CHANNELS (POSE_KPT_COUNT * POSE_KPT_DIMS)
#define POSE_MIN_CHANNELS (POSE_BOX_CHANNELS + 1 + POSE_KPT_CHANNELS)

/* ──────────── extracted structs & helpers ──────────── */
struct tensor_cn_view {
    const float *buf;
    int c;
    int n;
    bool c_major;
};

static float tensor_cn_read(const struct tensor_cn_view *tv, int c, int n)
{
    if (tv->c_major)
        return tv->buf[(size_t)c * (size_t)tv->n + (size_t)n];
    return tv->buf[(size_t)n * (size_t)tv->c + (size_t)c];
}

static float sigmoidf_local(float x)
{
    if (x < -80.0f) return 0.0f;
    if (x > 80.0f) return 1.0f;
    return 1.0f / (1.0f + expf(-x));
}

struct det_box {
    int x1, y1, x2, y2;
    float conf;
    int cls;
    int has_obb;
    float cx, cy, w, h, angle;
    float quad[8];
};

/* ──────────── inline decoder logic (parameterized) ──────────── */
static int decode_pose_inline(const float *data, int c_stride, int n_stride,
                              int total_c, int n_points,
                              float conf_thr,
                              struct det_box *out, int max_out,
                              int pose_nc, int nms_class_sep)
{
    struct tensor_cn_view view;
    int count = 0;
    int i;

    view.buf = data;
    view.c = total_c;
    view.n = n_points;
    view.c_major = (c_stride > 1); /* if c_stride != 1 then c-major */

    for (i = 0; i < n_points && count < max_out; i++) {
        float cx = tensor_cn_read(&view, 0, i);
        float cy = tensor_cn_read(&view, 1, i);
        float bw = tensor_cn_read(&view, 2, i);
        float bh = tensor_cn_read(&view, 3, i);
        float score;
        int best_id;
        int kpt_base;
        int k;
        bool valid = true;

        if (!isfinite(cx) || !isfinite(cy) || !isfinite(bw) || !isfinite(bh))
            continue;

        if (pose_nc == 1) {
            score = tensor_cn_read(&view, 4, i);
            best_id = 0;
        } else {
            int c;
            int best_c = 0;
            float best_s = -1.0f;
            for (c = 0; c < pose_nc; c++) {
                float s = tensor_cn_read(&view, 4 + c, i);
                if (s > best_s) {
                    best_s = s;
                    best_c = c;
                }
            }
            score = best_s;
            best_id = best_c;
        }

        if (!isfinite(score))
            continue;
        if (score < 0.0f || score > 1.0f)
            score = sigmoidf_local(score);
        if (score < conf_thr)
            continue;

        out[count].cx = cx;
        out[count].cy = cy;
        out[count].w = bw;
        out[count].h = bh;
        out[count].conf = score;
        out[count].cls = (best_id >= 0 && best_id < pose_nc) ? best_id : 0;
        out[count].has_obb = 1;

        kpt_base = POSE_BOX_CHANNELS + pose_nc;

        for (k = 0; k < POSE_KPT_COUNT; k++) {
            float kx = tensor_cn_read(&view, kpt_base + k * POSE_KPT_DIMS + 0, i);
            float ky = tensor_cn_read(&view, kpt_base + k * POSE_KPT_DIMS + 1, i);
            float kv = tensor_cn_read(&view, kpt_base + k * POSE_KPT_DIMS + 2, i);
            if (!isfinite(kx) || !isfinite(ky) || !isfinite(kv)) {
                valid = false;
                break;
            }
            out[count].quad[k * 2 + 0] = kx;
            out[count].quad[k * 2 + 1] = ky;
        }
        if (!valid)
            continue;

        /* Minimal validation: skip zero-area */
        if (out[count].w < 2.0f || out[count].h < 2.0f)
            continue;

        count++;
    }
    return count;
}

/* Helper: fill c-major buffer */
static void fill_cmaj(float *buf, int c, int n, int c_idx, int n_idx, float val)
{
    buf[(size_t)c_idx * n + (size_t)n_idx] = val;
}

static float read_cmaj(const float *buf, int c, int n, int c_idx, int n_idx)
{
    return buf[(size_t)c_idx * n + (size_t)n_idx];
}

/* ──────────── tests ──────────── */
static int g_tests = 0;
static int g_fails = 0;

#define TEST_ASSERT(cond, msg) do { \
    g_tests++; \
    if (!(cond)) { \
        fprintf(stderr, "[FAIL] %s:%d: %s\n", __FILE__, __LINE__, msg); \
        g_fails++; \
    } else { \
        printf("[PASS] %s\n", msg); \
    } \
} while (0)

static void test_old_17ch_model(void)
{
    /* Simulate old [1,17,8400] c-major output */
    int nc = 1;
    int total_c = 17;
    int n = 100; /* small slice */
    float *buf = calloc((size_t)total_c * n, sizeof(float));
    struct det_box dets[10];

    /* Fill bbox valid for point 5 */
    fill_cmaj(buf, total_c, n, 0, 5, 50.0f); /* cx */
    fill_cmaj(buf, total_c, n, 1, 5, 60.0f); /* cy */
    fill_cmaj(buf, total_c, n, 2, 5, 40.0f); /* w */
    fill_cmaj(buf, total_c, n, 3, 5, 30.0f); /* h */
    fill_cmaj(buf, total_c, n, 4, 5, 0.85f); /* score (single cls) */

    /* Keypoints at base=5 */
    fill_cmaj(buf, total_c, n, 5, 5, 30.0f); fill_cmaj(buf, total_c, n, 6, 5, 40.0f); fill_cmaj(buf, total_c, n, 7, 5, 1.0f);
    fill_cmaj(buf, total_c, n, 8, 5, 70.0f); fill_cmaj(buf, total_c, n, 9, 5, 40.0f); fill_cmaj(buf, total_c, n, 10, 5, 1.0f);
    fill_cmaj(buf, total_c, n, 11, 5, 70.0f); fill_cmaj(buf, total_c, n, 12, 5, 60.0f); fill_cmaj(buf, total_c, n, 13, 5, 1.0f);
    fill_cmaj(buf, total_c, n, 14, 5, 30.0f); fill_cmaj(buf, total_c, n, 15, 5, 60.0f); fill_cmaj(buf, total_c, n, 16, 5, 1.0f);

    int cnt = decode_pose_inline(buf, total_c, 1, total_c, n, 0.5f, dets, 10, nc, 1, true);
    TEST_ASSERT(cnt == 1, "old 17ch: one detection");
    TEST_ASSERT(dets[0].cls == 0, "old 17ch: cls == 0");
    TEST_ASSERT(dets[0].conf > 0.8f, "old 17ch: conf > 0.8");
    TEST_ASSERT(dets[0].w == 40.0f && dets[0].h == 30.0f, "old 17ch: wh match");
    /* Check first keypoint */
    TEST_ASSERT(fabsf(dets[0].quad[0] - 30.0f) < 0.01f && fabsf(dets[0].quad[1] - 40.0f) < 0.01f,
                "old 17ch: kpt0 correct");

    free(buf);
}

static void test_new_21ch_cls4(void)
{
    int nc = 5;
    int total_c = 21;
    int n = 100;
    float *buf = calloc((size_t)total_c * n, sizeof(float));
    struct det_box dets[10];

    /* Point 3: cls 4 (embassy) has highest score */
    fill_cmaj(buf, total_c, n, 0, 3, 55.0f);
    fill_cmaj(buf, total_c, n, 1, 3, 65.0f);
    fill_cmaj(buf, total_c, n, 2, 3, 45.0f);
    fill_cmaj(buf, total_c, n, 3, 3, 35.0f);
    /* class scores: ch 4..8 */
    fill_cmaj(buf, total_c, n, 4, 3, 0.1f);  /* blue */
    fill_cmaj(buf, total_c, n, 5, 3, 0.2f);  /* green */
    fill_cmaj(buf, total_c, n, 6, 3, 0.3f);  /* yellow */
    fill_cmaj(buf, total_c, n, 7, 3, 0.4f);  /* police */
    fill_cmaj(buf, total_c, n, 8, 3, 0.95f); /* embassy ← max */
    /* kpt base = 4+5 = 9 */
    fill_cmaj(buf, total_c, n, 9, 3, 35.0f); fill_cmaj(buf, total_c, n, 10, 3, 45.0f); fill_cmaj(buf, total_c, n, 11, 3, 1.0f);
    fill_cmaj(buf, total_c, n, 12, 3, 75.0f); fill_cmaj(buf, total_c, n, 13, 3, 45.0f); fill_cmaj(buf, total_c, n, 14, 3, 1.0f);
    fill_cmaj(buf, total_c, n, 15, 3, 75.0f); fill_cmaj(buf, total_c, n, 16, 3, 65.0f); fill_cmaj(buf, total_c, n, 17, 3, 1.0f);
    fill_cmaj(buf, total_c, n, 18, 3, 35.0f); fill_cmaj(buf, total_c, n, 19, 3, 65.0f); fill_cmaj(buf, total_c, n, 20, 3, 1.0f);

    int cnt = decode_pose_inline(buf, total_c, 1, total_c, n, 0.5f, dets, 10, nc, 1, true);
    TEST_ASSERT(cnt >= 1, "new 21ch cls4: at least one detection");
    /* The embassy detection (cls=4) should be present */
    bool found_embassy = false;
    int j;
    for (j = 0; j < cnt; j++) {
        if (dets[j].cls == 4) {
            found_embassy = true;
            TEST_ASSERT(dets[j].conf > 0.9f, "new 21ch cls4: embassy conf > 0.9");
            TEST_ASSERT(fabsf(dets[j].quad[0] - 35.0f) < 0.01f,
                        "new 21ch cls4: kpt0.x correct offset");
            break;
        }
    }
    TEST_ASSERT(found_embassy, "new 21ch cls4: embassy class detected");

    free(buf);
}

static void test_new_21ch_cls3(void)
{
    int nc = 5;
    int total_c = 21;
    int n = 100;
    float *buf = calloc((size_t)total_c * n, sizeof(float));
    struct det_box dets[10];

    /* Point 7: cls 3 (police) has highest */
    fill_cmaj(buf, total_c, n, 0, 7, 52.0f);
    fill_cmaj(buf, total_c, n, 1, 7, 62.0f);
    fill_cmaj(buf, total_c, n, 2, 7, 42.0f);
    fill_cmaj(buf, total_c, n, 3, 7, 32.0f);
    fill_cmaj(buf, total_c, n, 4, 7, 0.05f);
    fill_cmaj(buf, total_c, n, 5, 7, 0.10f);
    fill_cmaj(buf, total_c, n, 6, 7, 0.15f);
    fill_cmaj(buf, total_c, n, 7, 7, 0.97f); /* police ← max */
    fill_cmaj(buf, total_c, n, 8, 7, 0.20f);
    /* kpt base = 9 */
    fill_cmaj(buf, total_c, n, 9, 7, 32.0f); fill_cmaj(buf, total_c, n, 10, 7, 42.0f); fill_cmaj(buf, total_c, n, 11, 7, 1.0f);
    fill_cmaj(buf, total_c, n, 12, 7, 72.0f); fill_cmaj(buf, total_c, n, 13, 7, 42.0f); fill_cmaj(buf, total_c, n, 14, 7, 1.0f);
    fill_cmaj(buf, total_c, n, 15, 7, 72.0f); fill_cmaj(buf, total_c, n, 16, 7, 62.0f); fill_cmaj(buf, total_c, n, 17, 7, 1.0f);
    fill_cmaj(buf, total_c, n, 18, 7, 32.0f); fill_cmaj(buf, total_c, n, 19, 7, 62.0f); fill_cmaj(buf, total_c, n, 20, 7, 1.0f);

    int cnt = decode_pose_inline(buf, total_c, 1, total_c, n, 0.5f, dets, 10, nc, 1, true);
    TEST_ASSERT(cnt >= 1, "new 21ch cls3: at least one detection");
    bool found_police = false;
    int j;
    for (j = 0; j < cnt; j++) {
        if (dets[j].cls == 3) {
            found_police = true;
            break;
        }
    }
    TEST_ASSERT(found_police, "new 21ch cls3: police class detected");

    free(buf);
}

static void test_sigmoid_fallback(void)
{
    int nc = 1;
    int total_c = 17;
    int n = 100;
    float *buf = calloc((size_t)total_c * n, sizeof(float));
    struct det_box dets[10];

    /* Use a raw logit (-2.0) which after sigmoid = 0.119, below thr */
    fill_cmaj(buf, total_c, n, 0, 2, 50.0f);
    fill_cmaj(buf, total_c, n, 1, 2, 60.0f);
    fill_cmaj(buf, total_c, n, 2, 2, 30.0f);
    fill_cmaj(buf, total_c, n, 3, 2, 20.0f);
    fill_cmaj(buf, total_c, n, 4, 2, -2.0f); /* logit, not sigmoided */
    /* kpts */
    int kp_base = 5;
    fill_cmaj(buf, total_c, n, kp_base+0, 2, 30.0f); fill_cmaj(buf, total_c, n, kp_base+1, 2, 40.0f); fill_cmaj(buf, total_c, n, kp_base+2, 2, 1.0f);
    fill_cmaj(buf, total_c, n, kp_base+3, 2, 70.0f); fill_cmaj(buf, total_c, n, kp_base+4, 2, 40.0f); fill_cmaj(buf, total_c, n, kp_base+5, 2, 1.0f);
    fill_cmaj(buf, total_c, n, kp_base+6, 2, 70.0f); fill_cmaj(buf, total_c, n, kp_base+7, 2, 60.0f); fill_cmaj(buf, total_c, n, kp_base+8, 2, 1.0f);
    fill_cmaj(buf, total_c, n, kp_base+9, 2, 30.0f); fill_cmaj(buf, total_c, n, kp_base+10, 2, 60.0f); fill_cmaj(buf, total_c, n, kp_base+11, 2, 1.0f);

    int cnt = decode_pose_inline(buf, total_c, 1, total_c, n, 0.05f, dets, 10, nc, 1, true);
    TEST_ASSERT(cnt == 1, "sigmoid fallback: detection with logit");
    /* sigmoid(-2.0) = 0.119... */
    TEST_ASSERT(dets[0].conf > 0.1f && dets[0].conf < 0.13f,
                "sigmoid fallback: conf~0.119 after sigmoid");

    free(buf);
}

static void test_nc2_unknown_warning(void)
{
    int nc = 2; /* not 1 or 5 */
    int total_c = 18; /* 4 + 2 + 12 */
    int n = 100;
    float *buf = calloc((size_t)total_c * n, sizeof(float));
    struct det_box dets[10];

    fill_cmaj(buf, total_c, n, 0, 0, 50.0f);
    fill_cmaj(buf, total_c, n, 1, 0, 60.0f);
    fill_cmaj(buf, total_c, n, 2, 0, 30.0f);
    fill_cmaj(buf, total_c, n, 3, 0, 20.0f);
    fill_cmaj(buf, total_c, n, 4, 0, 0.3f); /* cls0 */
    fill_cmaj(buf, total_c, n, 5, 0, 0.95f); /* cls1 ← max */
    /* kpt base=6 */
    int kp_base = 6;
    fill_cmaj(buf, total_c, n, kp_base+0, 0, 30.0f); fill_cmaj(buf, total_c, n, kp_base+1, 0, 40.0f); fill_cmaj(buf, total_c, n, kp_base+2, 0, 1.0f);
    fill_cmaj(buf, total_c, n, kp_base+3, 0, 70.0f); fill_cmaj(buf, total_c, n, kp_base+4, 0, 40.0f); fill_cmaj(buf, total_c, n, kp_base+5, 0, 1.0f);
    fill_cmaj(buf, total_c, n, kp_base+6, 0, 70.0f); fill_cmaj(buf, total_c, n, kp_base+7, 0, 60.0f); fill_cmaj(buf, total_c, n, kp_base+8, 0, 1.0f);
    fill_cmaj(buf, total_c, n, kp_base+9, 0, 30.0f); fill_cmaj(buf, total_c, n, kp_base+10, 0, 60.0f); fill_cmaj(buf, total_c, n, kp_base+11, 0, 1.0f);

    int cnt = decode_pose_inline(buf, total_c, 1, total_c, n, 0.5f, dets, 10, nc, 1, true);
    TEST_ASSERT(cnt == 1, "nc=2 unknown: detection works");
    TEST_ASSERT(dets[0].cls == 1, "nc=2 unknown: cls=1 (argmax)");

    free(buf);
}

int main(void)
{
    printf("=== test_pose_decoder ===\n");
    test_old_17ch_model();
    test_new_21ch_cls4();
    test_new_21ch_cls3();
    test_sigmoid_fallback();
    test_nc2_unknown_warning();

    printf("\n%d tests, %d failures\n", g_tests, g_fails);
    return g_fails > 0 ? 1 : 0;
}
