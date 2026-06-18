// SPDX-License-Identifier: GPL-2.0
/* FPGA DMA frame capture and pixel format conversion. */

#include "lpr_dma.h"
#include "../pcie_fpga_dma.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

int lpr_dma_init(struct dma_state *d, const struct live_options *opt)
{
    struct fpga_info info;
    struct buffer_map map;
    uint32_t fmt;
    memset(d, 0, sizeof(*d));
    d->fd = -1;
    d->fd = open(opt->device_path, O_RDWR | O_CLOEXEC);
    if (d->fd < 0)
        return -1;
    if (ioctl(d->fd, FPGA_DMA_GET_INFO, &info) < 0)
        return -1;
    fmt = info.pixel_format;
    if (fmt != FPGA_PIXEL_FORMAT_BGR565 && fmt != FPGA_PIXEL_FORMAT_BGRX8888)
        fmt = (info.frame_bpp == 4) ? FPGA_PIXEL_FORMAT_BGRX8888 : FPGA_PIXEL_FORMAT_BGR565;
    info.frame_bpp = (fmt == FPGA_PIXEL_FORMAT_BGRX8888) ? 4 : 2;
    d->frame_w = info.frame_width;
    d->frame_h = info.frame_height;
    d->frame_bpp = info.frame_bpp;
    d->src_is_bgrx = fmt == FPGA_PIXEL_FORMAT_BGRX8888;
    d->frame_size = (size_t)d->frame_w * d->frame_h * d->frame_bpp;
    memset(&map, 0, sizeof(map));
    map.index = 0;
    if (ioctl(d->fd, FPGA_DMA_MAP_BUFFER, &map) < 0)
        return -1;
    if (map.size < d->frame_size)
        return -1;
    d->map_size = map.size;
    d->map = mmap(NULL, d->map_size, PROT_READ, MAP_SHARED, d->fd, 0);
    if (d->map == MAP_FAILED) {
        d->map = NULL;
        return -1;
    }
    d->copy = malloc(d->frame_size);
    if (!d->copy)
        return -1;
    return 0;
}

void lpr_dma_release(struct dma_state *d)
{
    if (!d) return;
    if (d->map)
        munmap(d->map, d->map_size);
    free(d->copy);
    if (d->fd >= 0)
        close(d->fd);
    memset(d, 0, sizeof(*d));
    d->fd = -1;
}

int lpr_dma_read_frame(struct dma_state *d)
{
    struct dma_transfer t;
    memset(&t, 0, sizeof(t));
    t.size = (uint32_t)d->frame_size;
    t.user_buf = (uint64_t)(uintptr_t)d->copy;
    if (ioctl(d->fd, FPGA_DMA_READ_FRAME, &t) < 0)
        return -1;
    return t.result == 0 ? 0 : -1;
}

void lpr_decode_pixel565(enum pixel_order order, bool swap16,
                         uint8_t lo_in, uint8_t hi_in,
                         uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t lo = swap16 ? hi_in : lo_in;
    uint8_t hi = swap16 ? lo_in : hi_in;
    uint16_t v = (uint16_t)lo | ((uint16_t)hi << 8);
    uint8_t c0 = (uint8_t)((v >> 11) & 0x1F);
    uint8_t c1 = (uint8_t)((v >> 5) & 0x3F);
    uint8_t c2 = (uint8_t)(v & 0x1F);
    if (order == PIXEL_ORDER_BGR565) {
        *b = (uint8_t)((c0 << 3) | (c0 >> 2));
        *g = (uint8_t)((c1 << 2) | (c1 >> 4));
        *r = (uint8_t)((c2 << 3) | (c2 >> 2));
    } else {
        *r = (uint8_t)((c0 << 3) | (c0 >> 2));
        *g = (uint8_t)((c1 << 2) | (c1 >> 4));
        *b = (uint8_t)((c2 << 3) | (c2 >> 2));
    }
}

void lpr_frame_to_rgb888(const struct dma_state *d,
                         const struct live_options *opt,
                         uint8_t *rgb)
{
    size_t pixels = (size_t)d->frame_w * d->frame_h;
    size_t i;
    if (d->src_is_bgrx) {
        for (i = 0; i < pixels; i++) {
            const uint8_t *p = d->copy + i * 4U;
            rgb[i * 3U + 0] = p[2];
            rgb[i * 3U + 1] = p[1];
            rgb[i * 3U + 2] = p[0];
        }
        return;
    }
    for (i = 0; i < pixels; i++) {
        uint8_t r, g, b;
        lpr_decode_pixel565(opt->pixel_order, opt->swap16,
                            d->copy[i * 2U], d->copy[i * 2U + 1], &r, &g, &b);
        rgb[i * 3U + 0] = r;
        rgb[i * 3U + 1] = g;
        rgb[i * 3U + 2] = b;
    }
}

void lpr_rgb888_to_rgb565(const uint8_t *rgb, uint16_t *dst, int w, int h)
{
    size_t n = (size_t)w * (size_t)h;
    for (size_t i = 0; i < n; i++) {
        uint8_t r = rgb[i * 3U + 0];
        uint8_t g = rgb[i * 3U + 1];
        uint8_t b = rgb[i * 3U + 2];
        dst[i] = (uint16_t)(((uint16_t)(r >> 3) << 11) |
                            ((uint16_t)(g >> 2) << 5) |
                            ((uint16_t)(b >> 3)));
    }
}
