// SPDX-License-Identifier: GPL-2.0
/* 4-point homography warp from detected quad to a flat plate crop. */

#include "lpr_warp.h"

#include <math.h>
#include <string.h>

static float quad_area8(const float q[8])
{
    float area = 0.0f;
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) & 3;
        area += q[i * 2] * q[j * 2 + 1] - q[j * 2] * q[i * 2 + 1];
    }
    return 0.5f * fabsf(area);
}

static bool quad_is_convex8(const float q[8])
{
    float prev = 0.0f;
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) & 3;
        int k = (i + 2) & 3;
        float ax = q[j * 2] - q[i * 2];
        float ay = q[j * 2 + 1] - q[i * 2 + 1];
        float bx = q[k * 2] - q[j * 2];
        float by = q[k * 2 + 1] - q[j * 2 + 1];
        float cross = ax * by - ay * bx;
        if (fabsf(cross) < 1e-5f)
            continue;
        if (prev != 0.0f && cross * prev < 0.0f)
            return false;
        prev = cross;
    }
    return prev != 0.0f;
}

/* Order 4 corners as TL, TR, BR, BL. Falls back to original order if the
 * resulting quad is degenerate. */
static void order_quad(const float in[8], float out[8])
{
    struct quad_pt { float x; float y; float a; } pts[4];
    int idx[4] = {0, 1, 2, 3};
    float cx = 0.0f, cy = 0.0f;
    if (!in || !out) return;
    for (int i = 0; i < 4; i++) {
        pts[i].x = in[i * 2];
        pts[i].y = in[i * 2 + 1];
        cx += pts[i].x;
        cy += pts[i].y;
    }
    cx *= 0.25f;
    cy *= 0.25f;
    for (int i = 0; i < 4; i++)
        pts[i].a = atan2f(pts[i].y - cy, pts[i].x - cx);
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
            if (pts[idx[j]].a < pts[idx[i]].a) {
                int t = idx[i]; idx[i] = idx[j]; idx[j] = t;
            }
        }
    }
    int top_edge = 0;
    float best_y = 0.5f * (pts[idx[0]].y + pts[idx[1]].y);
    for (int i = 1; i < 4; i++) {
        float edge_y = 0.5f * (pts[idx[i]].y + pts[idx[(i + 1) & 3]].y);
        if (edge_y < best_y) { best_y = edge_y; top_edge = i; }
    }
    if (pts[idx[top_edge]].x <= pts[idx[(top_edge + 1) & 3]].x) {
        for (int i = 0; i < 4; i++) {
            int pos = idx[(top_edge + i) & 3];
            out[i * 2] = pts[pos].x;
            out[i * 2 + 1] = pts[pos].y;
        }
    } else {
        for (int i = 0; i < 4; i++) {
            int pos = idx[(top_edge + 1 - i + 4) & 3];
            out[i * 2] = pts[pos].x;
            out[i * 2 + 1] = pts[pos].y;
        }
    }
    if (quad_area8(out) < 4.0f || !quad_is_convex8(out))
        memcpy(out, in, sizeof(float) * 8U);
}

static void sample_bilinear(const uint8_t *rgb, int w, int h, float x, float y, uint8_t p[3])
{
    int x0, y0, x1, y1;
    float wx, wy;
    if (x < 0.0f) x = 0.0f;
    if (y < 0.0f) y = 0.0f;
    if (x > (float)(w - 1)) x = (float)(w - 1);
    if (y > (float)(h - 1)) y = (float)(h - 1);
    x0 = (int)floorf(x); y0 = (int)floorf(y);
    x1 = x0 + 1; y1 = y0 + 1;
    if (x1 >= w) x1 = w - 1;
    if (y1 >= h) y1 = h - 1;
    wx = x - (float)x0; wy = y - (float)y0;
    for (int c = 0; c < 3; c++) {
        float p00 = rgb[((size_t)y0 * w + x0) * 3U + c];
        float p01 = rgb[((size_t)y0 * w + x1) * 3U + c];
        float p10 = rgb[((size_t)y1 * w + x0) * 3U + c];
        float p11 = rgb[((size_t)y1 * w + x1) * 3U + c];
        float v0 = p00 * (1.0f - wx) + p01 * wx;
        float v1 = p10 * (1.0f - wx) + p11 * wx;
        int iv = (int)(v0 * (1.0f - wy) + v1 * wy + 0.5f);
        if (iv < 0) iv = 0;
        if (iv > 255) iv = 255;
        p[c] = (uint8_t)iv;
    }
}

/* Solve a 8x9 augmented system in place via Gauss-Jordan. */
static bool solve_linear_8x8(float a[8][9], float x[8])
{
    for (int i = 0; i < 8; i++) {
        int piv = i;
        float max_v = fabsf(a[i][i]);
        for (int j = i + 1; j < 8; j++) {
            float v = fabsf(a[j][i]);
            if (v > max_v) { max_v = v; piv = j; }
        }
        if (max_v < 1e-8f) return false;
        if (piv != i) {
            for (int k = i; k < 9; k++) { float t = a[i][k]; a[i][k] = a[piv][k]; a[piv][k] = t; }
        }
        float div = a[i][i];
        for (int k = i; k < 9; k++) a[i][k] /= div;
        for (int j = 0; j < 8; j++) {
            if (j == i) continue;
            float mul = a[j][i];
            if (fabsf(mul) < 1e-8f) continue;
            for (int k = i; k < 9; k++) a[j][k] -= mul * a[i][k];
        }
    }
    for (int i = 0; i < 8; i++) x[i] = a[i][8];
    return true;
}

static bool get_homography_4pt(const float src[8], const float dst[8], float h[9])
{
    float mat[8][9];
    float sol[8];
    memset(mat, 0, sizeof(mat));
    for (int i = 0; i < 4; i++) {
        float x = src[i * 2], y = src[i * 2 + 1];
        float u = dst[i * 2], v = dst[i * 2 + 1];
        int r0 = i * 2, r1 = r0 + 1;
        mat[r0][0] = x; mat[r0][1] = y; mat[r0][2] = 1.0f;
        mat[r0][6] = -u * x; mat[r0][7] = -u * y; mat[r0][8] = u;
        mat[r1][3] = x; mat[r1][4] = y; mat[r1][5] = 1.0f;
        mat[r1][6] = -v * x; mat[r1][7] = -v * y; mat[r1][8] = v;
    }
    if (!solve_linear_8x8(mat, sol)) return false;
    h[0] = sol[0]; h[1] = sol[1]; h[2] = sol[2];
    h[3] = sol[3]; h[4] = sol[4]; h[5] = sol[5];
    h[6] = sol[6]; h[7] = sol[7]; h[8] = 1.0f;
    return true;
}

static bool invert_homography(const float h[9], float inv[9])
{
    float det = h[0] * (h[4] * h[8] - h[5] * h[7]) -
                h[1] * (h[3] * h[8] - h[5] * h[6]) +
                h[2] * (h[3] * h[7] - h[4] * h[6]);
    if (fabsf(det) < 1e-8f) return false;
    inv[0] =  (h[4] * h[8] - h[5] * h[7]) / det;
    inv[1] = -(h[1] * h[8] - h[2] * h[7]) / det;
    inv[2] =  (h[1] * h[5] - h[2] * h[4]) / det;
    inv[3] = -(h[3] * h[8] - h[5] * h[6]) / det;
    inv[4] =  (h[0] * h[8] - h[2] * h[6]) / det;
    inv[5] = -(h[0] * h[5] - h[2] * h[3]) / det;
    inv[6] =  (h[3] * h[7] - h[4] * h[6]) / det;
    inv[7] = -(h[0] * h[7] - h[1] * h[6]) / det;
    inv[8] =  (h[0] * h[4] - h[1] * h[3]) / det;
    return true;
}

bool lpr_warp_quad_homography(const uint8_t *rgb, int img_w, int img_h,
                              const float quad_in[8],
                              uint8_t *dst, int cap_w, int cap_h,
                              int *out_w, int *out_h)
{
    float q[8], dst_quad[8], h[9], inv_h[9];
    float top_w, bot_w, left_h, right_h;
    int dw, dh;
    order_quad(quad_in, q);
    top_w = hypotf(q[2] - q[0], q[3] - q[1]);
    bot_w = hypotf(q[4] - q[6], q[5] - q[7]);
    left_h = hypotf(q[6] - q[0], q[7] - q[1]);
    right_h = hypotf(q[4] - q[2], q[5] - q[3]);
    dw = (int)(fmaxf(top_w, bot_w) + 0.5f);
    dh = (int)(fmaxf(left_h, right_h) + 0.5f);
    if (dw < 1) dw = 1;
    if (dh < 1) dh = 1;
    if (dw > cap_w) dw = cap_w;
    if (dh > cap_h) dh = cap_h;
    if (dw <= 0 || dh <= 0) return false;
    dst_quad[0] = 0.0f;             dst_quad[1] = 0.0f;
    dst_quad[2] = (float)dw - 1.0f; dst_quad[3] = 0.0f;
    dst_quad[4] = (float)dw - 1.0f; dst_quad[5] = (float)dh - 1.0f;
    dst_quad[6] = 0.0f;             dst_quad[7] = (float)dh - 1.0f;
    if (!get_homography_4pt(q, dst_quad, h)) return false;
    if (!invert_homography(h, inv_h)) return false;
    for (int y = 0; y < dh; y++) {
        for (int x = 0; x < dw; x++) {
            float fx = (float)x, fy = (float)y;
            float den = inv_h[6] * fx + inv_h[7] * fy + inv_h[8];
            float sx, sy;
            uint8_t pix[3];
            if (fabsf(den) < 1e-8f) den = den >= 0.0f ? 1e-8f : -1e-8f;
            sx = (inv_h[0] * fx + inv_h[1] * fy + inv_h[2]) / den;
            sy = (inv_h[3] * fx + inv_h[4] * fy + inv_h[5]) / den;
            sample_bilinear(rgb, img_w, img_h, sx, sy, pix);
            memcpy(dst + ((size_t)y * dw + x) * 3U, pix, 3);
        }
    }
    *out_w = dw;
    *out_h = dh;
    return true;
}
