// SPDX-License-Identifier: GPL-2.0
/* RGB-based plate body color classification.
 *
 * Routing rules used by the live driver (see lpr_infer.c::pick_route):
 *   PLATE_COLOR_BLUE   -> blue PPLCNet OCR
 *   PLATE_COLOR_GREEN  -> green PPLCNet OCR (8-char new-energy decode)
 *   PLATE_COLOR_YELLOW -> yellow PPLCNet OCR (taxi/learner/heavy);
 *                         falls back to police if yellow is disabled
 *   PLATE_COLOR_WHITE  -> police PPLCNet OCR (most police plates have a
 *                         white body with red 警 suffix)
 *   PLATE_COLOR_BLACK  -> embassy PPLCNet OCR (black body, white text)
 *   PLATE_COLOR_UNKNOWN-> blue OCR (most common base case)
 *
 * The classifier reports the raw color label; the actual route mapping
 * (and the disabled-route fallback chain) is handled in lpr_infer.c.
 */

#include "lpr_color.h"

#include <math.h>

enum plate_color lpr_classify_plate_color(const uint8_t *rgb, int w, int h,
                                          const struct det_box *b)
{
    /* Sample the inner 2/3 of the box to avoid the colored frame border. */
    int x1 = b->x1 + (b->x2 - b->x1) / 6;
    int x2 = b->x2 - (b->x2 - b->x1) / 6;
    int y1 = b->y1 + (b->y2 - b->y1) / 6;
    int y2 = b->y2 - (b->y2 - b->y1) / 6;
    int total = 0;
    int blue_cnt = 0, green_cnt = 0, yellow_cnt = 0, white_cnt = 0, black_cnt = 0;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 >= w) x2 = w - 1;
    if (y2 >= h) y2 = h - 1;
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            const uint8_t *p = rgb + (y * w + x) * 3;
            float r = p[0] / 255.0f;
            float g = p[1] / 255.0f;
            float bch = p[2] / 255.0f;
            float mx = fmaxf(r, fmaxf(g, bch));
            float mn = fminf(r, fminf(g, bch));
            float d = mx - mn;
            float h_deg = 0.0f;
            float s = (mx == 0.0f) ? 0.0f : (d / mx);
            float v = mx;
            if (d > 1e-6f) {
                if (mx == r) h_deg = 60.0f * fmodf((g - bch) / d, 6.0f);
                else if (mx == g) h_deg = 60.0f * (((bch - r) / d) + 2.0f);
                else h_deg = 60.0f * (((r - g) / d) + 4.0f);
            }
            if (h_deg < 0.0f) h_deg += 360.0f;
            total++;
            /* Black: very low value across all channels (embassy body). */
            if (v < 0.15f) {
                black_cnt++;
                continue;
            }
            /* Blue: hue 190-260, saturated. */
            if (h_deg >= 190.0f && h_deg <= 260.0f && s > 0.23f && v > 0.16f)
                blue_cnt++;
            /* Green: hue 75-155. */
            else if (h_deg >= 75.0f && h_deg <= 155.0f && s > 0.20f && v > 0.16f)
                green_cnt++;
            /* Yellow: hue 15-55 (taxis, learner, heavy, some police variants). */
            else if (h_deg >= 15.0f && h_deg <= 55.0f && s > 0.15f && v > 0.16f)
                yellow_cnt++;
            /* White: low saturation, high brightness (police/embassy white body). */
            else if (s < 0.15f && v > 0.55f)
                white_cnt++;
        }
    }
    if (total == 0) return PLATE_COLOR_UNKNOWN;
    /* Vote in priority order: black wins if dominant, then chromatic colors,
     * then white. Embassy plates have a strong black body so we check that
     * first; otherwise yellow/police/white never trip on a black plate. */
    if ((float)black_cnt / (float)total >= 0.45f)
        return PLATE_COLOR_BLACK;
    if ((float)blue_cnt / (float)total >= 0.20f && blue_cnt > green_cnt + (int)(0.05f * total))
        return PLATE_COLOR_BLUE;
    if ((float)green_cnt / (float)total >= 0.20f && green_cnt > blue_cnt + (int)(0.05f * total))
        return PLATE_COLOR_GREEN;
    if ((float)yellow_cnt / (float)total >= 0.18f)
        return PLATE_COLOR_YELLOW;
    if ((float)white_cnt / (float)total >= 0.30f)
        return PLATE_COLOR_WHITE;
    return PLATE_COLOR_UNKNOWN;
}
