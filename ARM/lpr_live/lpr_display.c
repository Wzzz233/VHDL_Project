// SPDX-License-Identifier: GPL-2.0
/* Display module: HDMI/KMS RGB16 output and overlay drawing. */

#include "lpr_display.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ---------------- Drawing primitives ---------------- */

static void draw_hline_565(uint16_t *pix, int w, int h, int x1, int x2, int y, uint16_t c)
{
    if (y < 0 || y >= h) return;
    if (x1 > x2) { int t = x1; x1 = x2; x2 = t; }
    if (x1 < 0) x1 = 0;
    if (x2 >= w) x2 = w - 1;
    for (int x = x1; x <= x2; x++) pix[y * w + x] = c;
}

static void draw_vline_565(uint16_t *pix, int w, int h, int x, int y1, int y2, uint16_t c)
{
    if (x < 0 || x >= w) return;
    if (y1 > y2) { int t = y1; y1 = y2; y2 = t; }
    if (y1 < 0) y1 = 0;
    if (y2 >= h) y2 = h - 1;
    for (int y = y1; y <= y2; y++) pix[y * w + x] = c;
}

void lpr_draw_rect_565(uint16_t *pix, int w, int h, const struct det_box *b, uint16_t c)
{
    for (int t = 0; t < 2; t++) {
        draw_hline_565(pix, w, h, b->x1, b->x2, b->y1 + t, c);
        draw_hline_565(pix, w, h, b->x1, b->x2, b->y2 - t, c);
        draw_vline_565(pix, w, h, b->x1 + t, b->y1, b->y2, c);
        draw_vline_565(pix, w, h, b->x2 - t, b->y1, b->y2, c);
    }
}

/* 5x7 ASCII glyph (subset of printable chars). Compact LUT, no malloc. */
static uint8_t glyph5x7(char ch, int row)
{
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
    switch (ch) {
    case '0': { static const uint8_t g[7]={0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; return g[row]; }
    case '1': { static const uint8_t g[7]={0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
    case '2': { static const uint8_t g[7]={0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}; return g[row]; }
    case '3': { static const uint8_t g[7]={0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case '4': { static const uint8_t g[7]={0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}; return g[row]; }
    case '5': { static const uint8_t g[7]={0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}; return g[row]; }
    case '6': { static const uint8_t g[7]={0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}; return g[row]; }
    case '7': { static const uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x08,0x08}; return g[row]; }
    case '8': { static const uint8_t g[7]={0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}; return g[row]; }
    case '9': { static const uint8_t g[7]={0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}; return g[row]; }
    case 'A': { static const uint8_t g[7]={0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'B': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}; return g[row]; }
    case 'C': { static const uint8_t g[7]={0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}; return g[row]; }
    case 'D': { static const uint8_t g[7]={0x1C,0x12,0x11,0x11,0x11,0x12,0x1C}; return g[row]; }
    case 'E': { static const uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}; return g[row]; }
    case 'F': { static const uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'G': { static const uint8_t g[7]={0x0F,0x10,0x10,0x13,0x11,0x11,0x0F}; return g[row]; }
    case 'H': { static const uint8_t g[7]={0x11,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'I': { static const uint8_t g[7]={0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
    case 'J': { static const uint8_t g[7]={0x01,0x01,0x01,0x01,0x11,0x11,0x0E}; return g[row]; }
    case 'K': { static const uint8_t g[7]={0x11,0x12,0x14,0x18,0x14,0x12,0x11}; return g[row]; }
    case 'L': { static const uint8_t g[7]={0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return g[row]; }
    case 'M': { static const uint8_t g[7]={0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; return g[row]; }
    case 'N': { static const uint8_t g[7]={0x11,0x19,0x15,0x13,0x11,0x11,0x11}; return g[row]; }
    case 'O': { static const uint8_t g[7]={0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'P': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'Q': { static const uint8_t g[7]={0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}; return g[row]; }
    case 'R': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return g[row]; }
    case 'S': { static const uint8_t g[7]={0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case 'T': { static const uint8_t g[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'U': { static const uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'V': { static const uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x0A,0x04}; return g[row]; }
    case 'W': { static const uint8_t g[7]={0x11,0x11,0x11,0x15,0x15,0x1B,0x11}; return g[row]; }
    case 'X': { static const uint8_t g[7]={0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}; return g[row]; }
    case 'Y': { static const uint8_t g[7]={0x11,0x11,0x0A,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'Z': { static const uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}; return g[row]; }
    case '-': { static const uint8_t g[7]={0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; return g[row]; }
    case '.': { static const uint8_t g[7]={0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}; return g[row]; }
    case ':': { static const uint8_t g[7]={0x00,0x04,0x04,0x00,0x04,0x04,0x00}; return g[row]; }
    case '?': { static const uint8_t g[7]={0x0E,0x11,0x01,0x02,0x04,0x00,0x04}; return g[row]; }
    case ' ': { static const uint8_t g[7]={0,0,0,0,0,0,0}; return g[row]; }
    default: return 0;
    }
}

struct hanzi16_glyph {
    const char *utf8;
    uint16_t rows[16];
};

static const struct hanzi16_glyph hanzi16_glyphs[] = {
    {"京", {0x0400, 0x0400, 0x0200, 0xFFFC, 0x0000, 0x0000, 0x3FE0, 0x2020, 0x2020, 0x3FE0, 0x0200, 0x1240, 0x3260, 0x6230, 0x8218, 0x0C00}},
    {"沪", {0xC0C0, 0x6040, 0x03F8, 0x8208, 0xC208, 0x23F8, 0x2208, 0x2208, 0x2200, 0x4200, 0x4200, 0xC400, 0x8C00, 0x9800, 0x0000, 0x0000}},
    {"津", {0x6040, 0x33FC, 0x1044, 0x0044, 0x47FF, 0x6044, 0x33FC, 0x1040, 0x13FE, 0x1040, 0x2040, 0x27FF, 0x2040, 0x6040, 0x4040, 0x0000}},
    {"渝", {0x60C0, 0x30E0, 0x0130, 0x0318, 0x4FF6, 0x3000, 0x1784, 0x24A4, 0x24A4, 0x27A4, 0x24A4, 0x27A4, 0x44A4, 0x4484, 0x459C, 0x0000}},
    {"冀", {0x7CF2, 0x0484, 0x7C7C, 0x1FF0, 0x1110, 0x1FF0, 0x1110, 0x1FF0, 0x0440, 0x3FF8, 0xFFFC, 0x0860, 0x1818, 0x6008, 0x0000, 0x0000}},
    {"晋", {0x7FFE, 0x0240, 0x2248, 0x3258, 0x1250, 0x1240, 0xFFFF, 0x0000, 0x1FF8, 0x1008, 0x1008, 0x1FF8, 0x1008, 0x1FF8, 0x1008, 0x0000}},
    {"蒙", {0x0440, 0xFFFF, 0x0440, 0x7FFE, 0x4002, 0x4FF2, 0x0000, 0x7FFE, 0x0708, 0x1DB8, 0x66E0, 0x19A0, 0x6690, 0x1C8E, 0x7102, 0x0200}},
    {"辽", {0x13FC, 0x1008, 0x0030, 0x0020, 0x0020, 0x7020, 0x1020, 0x1020, 0x1020, 0x1020, 0x1020, 0x1020, 0x3840, 0x6C00, 0x43FE, 0x0000}},
    {"吉", {0x0200, 0x0200, 0x0200, 0xFFFC, 0x0200, 0x0200, 0xFFF8, 0x0000, 0x0000, 0x7FF0, 0x4010, 0x4010, 0x4010, 0x7FF0, 0x4010, 0x0000}},
    {"黑", {0x3FF8, 0x2928, 0x2928, 0x2D48, 0x2108, 0x3FF8, 0x0100, 0x0100, 0x7FF8, 0x0100, 0xFFFE, 0x0000, 0x24C8, 0x4444, 0xC226, 0x0000}},
    {"苏", {0x0820, 0x0820, 0xFFFE, 0x0820, 0x0000, 0x0200, 0x7FE0, 0x0220, 0x3228, 0x242C, 0x2424, 0x4C26, 0x1820, 0x3040, 0x43C0, 0x0000}},
    {"浙", {0x0408, 0xC438, 0x44C0, 0x0480, 0x1E80, 0x84FC, 0x8490, 0x2690, 0x5C90, 0x5490, 0x4490, 0x4490, 0x8510, 0x8510, 0x9810, 0x0000}},
    {"皖", {0x0020, 0x1030, 0x13FE, 0x2202, 0x7A02, 0x48FC, 0x4800, 0x4BFE, 0x7850, 0x4850, 0x4850, 0x4892, 0x4912, 0x7A12, 0x4C1E, 0x0000}},
    {"闽", {0x13FC, 0x4904, 0x4104, 0x4FE4, 0x4924, 0x4924, 0x4FE4, 0x4924, 0x4164, 0x4124, 0x5FD4, 0x4014, 0x4004, 0x4018, 0x0000, 0x0000}},
    {"赣", {0x1040, 0x7EBC, 0x25D0, 0x2878, 0x55FE, 0x7C20, 0x45FE, 0x7CFC, 0x7C84, 0x10A4, 0xFEA4, 0x1050, 0x10DC, 0x1386, 0x0000, 0x0000}},
    {"鲁", {0x0FE0, 0x18C0, 0x3FF8, 0x7108, 0x5FF0, 0x1108, 0x0EF0, 0x7FFE, 0x0000, 0x1FF0, 0x1810, 0x1FF0, 0x1FF0, 0x1810, 0x0000, 0x0000}},
    {"豫", {0x0040, 0x00C0, 0x3CF0, 0x3BFC, 0x1324, 0x1924, 0x7DFC, 0x1C40, 0x11E8, 0x12F0, 0x1368, 0x14EC, 0x1326, 0x3460, 0x0000, 0x0000}},
    {"鄂", {0x7700, 0x553E, 0x5526, 0x5524, 0x772C, 0x0028, 0x3F28, 0x0024, 0xFFA2, 0x1022, 0x3E22, 0x0222, 0x0226, 0x022C, 0x1C20, 0x0020}},
    {"湘", {0x213E, 0x1122, 0x0122, 0xC3A2, 0x613E, 0x33A2, 0x13A2, 0x1762, 0x253E, 0x2D22, 0x2922, 0x213E, 0x4122, 0x4122, 0x0000, 0x0000}},
    {"粤", {0x0600, 0x3FF0, 0x2950, 0x25D0, 0x2FF0, 0x27D0, 0x2D70, 0x3FF0, 0x2010, 0xFFFE, 0x0C00, 0x0FF0, 0x0010, 0x0010, 0x0070, 0x0000}},
    {"桂", {0x2040, 0x2040, 0xFBF8, 0x2040, 0x6040, 0x7040, 0x77FC, 0xE840, 0xA040, 0xA3F8, 0x2040, 0x2040, 0x2040, 0x27FC, 0x2000, 0x0000}},
    {"琼", {0x0180, 0xF840, 0x27FE, 0x2000, 0x23F8, 0x2208, 0xF208, 0x23F8, 0x2248, 0x2040, 0x2258, 0x3E4C, 0xCC44, 0x1842, 0x1180, 0x0000}},
    {"川", {0x0888, 0x0888, 0x0888, 0x0888, 0x0888, 0x0888, 0x0888, 0x0888, 0x0888, 0x0888, 0x0888, 0x1008, 0x3008, 0x6008, 0x0000, 0x0000}},
    {"贵", {0x1FF0, 0x1110, 0x1FF0, 0x0100, 0xFFFE, 0x0000, 0x1FF0, 0x1010, 0x1110, 0x1110, 0x1310, 0x12C0, 0x0C30, 0x780C, 0x0000, 0x0000}},
    {"云", {0x3FF8, 0x0000, 0x0000, 0x0000, 0x0000, 0x7FFE, 0x0200, 0x0600, 0x0460, 0x0830, 0x1810, 0x1018, 0x3FE8, 0x0004, 0x0000, 0x0000}},
    {"藏", {0x7FFF, 0x0428, 0x2014, 0x2FFA, 0x2BD0, 0x1A94, 0x0BD4, 0x7A54, 0x2BDC, 0x2A98, 0x2A9A, 0x2BFB, 0x5067, 0x30C2, 0x0000, 0x0000}},
    {"陕", {0x0020, 0x7C20, 0x4820, 0x4820, 0x5BFE, 0x5020, 0x532C, 0x49A8, 0x4C20, 0x47FF, 0x4430, 0x4450, 0x5C48, 0x418C, 0x4307, 0x4400}},
    {"甘", {0x0820, 0x0820, 0x0820, 0xFFFE, 0x0820, 0x0820, 0x0820, 0x0820, 0x0FE0, 0x0820, 0x0820, 0x0820, 0x0820, 0x0FE0, 0x0820, 0x0000}},
    {"青", {0x0200, 0xFFF0, 0x0200, 0x7FF0, 0x0200, 0xFFFC, 0x0000, 0x3FE0, 0x2020, 0x3FE0, 0x2020, 0x3FE0, 0x2020, 0x2020, 0x20C0, 0x0000}},
    {"宁", {0x0000, 0x0300, 0x0180, 0x7FFE, 0x4002, 0x4002, 0x0000, 0x7FFE, 0x0080, 0x0080, 0x0080, 0x0080, 0x0080, 0x0080, 0x0080, 0x0300}},
    {"新", {0x0000, 0x0C02, 0x3FBE, 0x1120, 0x1220, 0x0A20, 0x3FBF, 0x0424, 0x0424, 0x3FA4, 0x0424, 0x1724, 0x35A4, 0x2444, 0x44C4, 0x0C84}},
    {"使", {0x0820, 0x0820, 0x0BFE, 0x1820, 0x1020, 0x33FC, 0x7224, 0x5224, 0x13FC, 0x1020, 0x1320, 0x11C0, 0x10C0, 0x11B0, 0x120E, 0x0000}},
    {"警", {0x2440, 0xFF80, 0xA4F8, 0xFF70, 0xFA60, 0x7A70, 0x0F88, 0x0200, 0xFFF8, 0x3FE0, 0x3FE0, 0x0000, 0x3FE0, 0x3FE0, 0x2020, 0x0000}},
};

static const uint16_t *find_hanzi16_glyph(const char *s, size_t len)
{
    size_t n = sizeof(hanzi16_glyphs) / sizeof(hanzi16_glyphs[0]);
    for (size_t i = 0; i < n; i++) {
        if (strlen(hanzi16_glyphs[i].utf8) == len &&
            memcmp(s, hanzi16_glyphs[i].utf8, len) == 0)
            return hanzi16_glyphs[i].rows;
    }
    return NULL;
}

static size_t utf8_codepoint_len(const char *s)
{
    unsigned char ch = (unsigned char)s[0];
    size_t len;
    if (ch < 0x80) return 1;
    if ((ch & 0xE0) == 0xC0) len = 2;
    else if ((ch & 0xF0) == 0xE0) len = 3;
    else if ((ch & 0xF8) == 0xF0) len = 4;
    else return 1;
    for (size_t i = 1; i < len; i++) {
        if ((unsigned char)s[i] == 0 || (((unsigned char)s[i] & 0xC0) != 0x80))
            return 1;
    }
    return len;
}

static void draw_ascii_char_565(uint16_t *pix, int w, int h, int x, int y, char ch,
                                uint16_t c, int scale)
{
    for (int row = 0; row < 7; row++) {
        uint8_t bits = glyph5x7(ch, row);
        for (int col = 0; col < 5; col++) {
            if (!(bits & (1U << (4 - col)))) continue;
            for (int sy = 0; sy < scale; sy++) {
                int py = y + row * scale + sy;
                if (py < 0 || py >= h) continue;
                for (int sx = 0; sx < scale; sx++) {
                    int px = x + col * scale + sx;
                    if (px >= 0 && px < w) pix[py * w + px] = c;
                }
            }
        }
    }
}

static void draw_hanzi16_565(uint16_t *pix, int w, int h, int x, int y,
                             const uint16_t rows[16], uint16_t c, int scale)
{
    for (int row = 0; row < 16; row++) {
        uint16_t bits = rows[row];
        for (int col = 0; col < 16; col++) {
            if (!(bits & (uint16_t)(1U << (15 - col)))) continue;
            for (int sy = 0; sy < scale; sy++) {
                int py = y + row * scale + sy;
                if (py < 0 || py >= h) continue;
                for (int sx = 0; sx < scale; sx++) {
                    int px = x + col * scale + sx;
                    if (px >= 0 && px < w) pix[py * w + px] = c;
                }
            }
        }
    }
}

void lpr_draw_text_565(uint16_t *pix, int w, int h, int x, int y, const char *s,
                       uint16_t c, int scale)
{
    int pen_x = x;
    if (!s || scale < 1) return;
    for (size_t i = 0; s[i] != '\0'; ) {
        unsigned char ch = (unsigned char)s[i];
        if (ch < 0x80) {
            draw_ascii_char_565(pix, w, h, pen_x, y, s[i], c, scale);
            pen_x += 6 * scale;
            i++;
        } else {
            size_t len = utf8_codepoint_len(s + i);
            const uint16_t *glyph = find_hanzi16_glyph(s + i, len);
            if (glyph) {
                draw_hanzi16_565(pix, w, h, pen_x, y, glyph, c, scale);
                pen_x += 17 * scale;
            } else {
                draw_ascii_char_565(pix, w, h, pen_x, y, '?', c, scale);
                pen_x += 6 * scale;
            }
            i += len;
        }
    }
}

void lpr_overlay_ascii_from_text(const char *text, char *out, size_t out_len)
{
    size_t j = 0;
    if (!out || out_len == 0) return;
    if (!text) text = "";
    for (size_t i = 0; text[i] && j + 1 < out_len; i++) {
        unsigned char ch = (unsigned char)text[i];
        if ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z'))
            out[j++] = (char)ch;
    }
    out[j] = '\0';
}

/* ---------------- KMS pipeline lifecycle ---------------- */

int lpr_display_start(struct display_state *d, const struct live_options *opt,
                      uint32_t w, uint32_t h)
{
    GstCaps *caps;
    GstStateChangeReturn sret;
    memset(d, 0, sizeof(*d));
    d->enabled = opt->display;
    d->drm_fd = -1;
    if (!d->enabled) return 0;
    d->w = w; d->h = h; d->fps = opt->fps; d->connector_id = opt->connector_id; d->sync = opt->display_sync;
    d->frame_size = (size_t)w * (size_t)h * 2U;
    if (opt->drm_card_path && opt->drm_card_path[0]) {
        d->drm_fd = open(opt->drm_card_path, O_RDWR | O_CLOEXEC);
        if (d->drm_fd < 0)
            fprintf(stderr, "[display] warning: failed to open %s: %s\n", opt->drm_card_path, strerror(errno));
    }
    d->pipeline = gst_pipeline_new("pplcnet-bgp-live");
    d->appsrc = gst_element_factory_make("appsrc", "src");
    d->queue = gst_element_factory_make("queue", "queue");
    d->sink = gst_element_factory_make("kmssink", "sink");
    if (!d->pipeline || !d->appsrc || !d->queue || !d->sink) {
        fprintf(stderr, "[display] failed to create appsrc/queue/kmssink\n");
        return -1;
    }
    gst_bin_add_many(GST_BIN(d->pipeline), d->appsrc, d->queue, d->sink, NULL);
    if (!gst_element_link_many(d->appsrc, d->queue, d->sink, NULL)) {
        fprintf(stderr, "[display] failed to link appsrc -> queue -> kmssink\n");
        return -1;
    }
    caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "RGB16",
                               "width", G_TYPE_INT, (int)w, "height", G_TYPE_INT, (int)h,
                               "framerate", GST_TYPE_FRACTION, opt->fps, 1, NULL);
    if (!caps) return -1;
    g_object_set(d->appsrc, "caps", caps, "is-live", TRUE, "do-timestamp", TRUE,
                 "format", GST_FORMAT_TIME, "block", FALSE,
                 "max-bytes", (guint64)d->frame_size * 2U, NULL);
    gst_caps_unref(caps);
    g_object_set(d->queue, "max-size-buffers", 1, "max-size-bytes", 0,
                 "max-size-time", (guint64)0, "leaky", 2, NULL);
    g_object_set(d->sink, "sync", d->sync ? TRUE : FALSE, NULL);
    if (d->connector_id >= 0) g_object_set(d->sink, "connector-id", d->connector_id, NULL);
    if (d->drm_fd >= 0) g_object_set(d->sink, "fd", d->drm_fd, NULL);
    d->bus = gst_element_get_bus(d->pipeline);
    sret = gst_element_set_state(d->pipeline, GST_STATE_PLAYING);
    if (sret == GST_STATE_CHANGE_FAILURE) return -1;
    sret = gst_element_get_state(d->pipeline, NULL, NULL, 5 * GST_SECOND);
    if (sret == GST_STATE_CHANGE_FAILURE) return -1;
    fprintf(stderr, "[display] started appsrc RGB16 %ux%u -> kmssink sync=%d connector=%d\n",
            w, h, d->sync ? 1 : 0, d->connector_id);
    return 0;
}

void lpr_display_stop(struct display_state *d)
{
    if (!d || !d->enabled) return;
    if (d->appsrc) gst_app_src_end_of_stream(GST_APP_SRC(d->appsrc));
    if (d->pipeline) gst_element_set_state(d->pipeline, GST_STATE_NULL);
    if (d->bus) gst_object_unref(d->bus);
    if (d->pipeline) gst_object_unref(d->pipeline);
    if (d->drm_fd >= 0) close(d->drm_fd);
    memset(d, 0, sizeof(*d));
    d->drm_fd = -1;
}

static int handle_bus(struct display_state *d)
{
    GstMessage *msg;
    if (!d || !d->enabled || !d->bus) return 0;
    while ((msg = gst_bus_pop(d->bus)) != NULL) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError *err = NULL; gchar *dbg = NULL;
            gst_message_parse_error(msg, &err, &dbg);
            fprintf(stderr, "[display] error: %s\n", err ? err->message : "unknown");
            if (err) g_error_free(err);
            g_free(dbg);
            gst_message_unref(msg);
            return -1;
        }
        gst_message_unref(msg);
    }
    return 0;
}

int lpr_display_push(struct display_state *d, const uint16_t *frame)
{
    uint8_t *copy;
    GstBuffer *buf;
    GstFlowReturn flow;
    if (!d || !d->enabled) return 0;
    if (handle_bus(d) < 0) return -1;
    copy = g_malloc(d->frame_size);
    if (!copy) return -1;
    memcpy(copy, frame, d->frame_size);
    buf = gst_buffer_new_wrapped_full((GstMemoryFlags)0, copy, d->frame_size, 0, d->frame_size, copy, g_free);
    if (!buf) { g_free(copy); return -1; }
    GST_BUFFER_PTS(buf) = d->next_pts_ns;
    GST_BUFFER_DURATION(buf) = (guint64)(GST_SECOND / d->fps);
    d->next_pts_ns += GST_BUFFER_DURATION(buf);
    flow = gst_app_src_push_buffer(GST_APP_SRC(d->appsrc), buf);
    return flow == GST_FLOW_OK ? 0 : -1;
}
