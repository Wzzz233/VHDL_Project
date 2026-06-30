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
    uint32_t fmt;
    int requested = LPR_DMA_DEFAULT_SLOTS;
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
    d->zero_copy = d->src_is_bgrx;
    if (!d->zero_copy) {
        fprintf(stderr, "[dma] BGRX8888 source is required for live zero-copy path\n");
        return -1;
    }
    pthread_mutex_init(&d->lock, NULL);
    pthread_cond_init(&d->cond, NULL);
    d->lock_init = true;
    if (requested > LPR_DMA_MAX_SLOTS)
        requested = LPR_DMA_MAX_SLOTS;
    for (int i = 0; i < requested; i++) {
        struct buffer_map map;
        memset(&map, 0, sizeof(map));
        map.index = (uint32_t)i;
        if (ioctl(d->fd, FPGA_DMA_MAP_BUFFER, &map) < 0) {
            if (i >= 3)
                break;
            fprintf(stderr, "[dma] failed to map DMA slot %d: %s\n", i, strerror(errno));
            return -1;
        }
        if (map.size < d->frame_size) {
            fprintf(stderr, "[dma] mapped slot %d too small: %u < %zu\n", i, map.size, d->frame_size);
            return -1;
        }
        d->slots[i].data = mmap(NULL, map.size, PROT_READ | PROT_WRITE, MAP_SHARED, d->fd, (off_t)map.offset);
        if (d->slots[i].data == MAP_FAILED) {
            d->slots[i].data = NULL;
            fprintf(stderr, "[dma] mmap DMA slot %d failed: %s\n", i, strerror(errno));
            return -1;
        }
        d->slots[i].size = map.size;
        d->slots[i].index = (uint32_t)i;
        d->slots[i].generation = 0;
        d->slots[i].refs = 0;
        d->slot_count++;
    }
    if (d->slot_count < 3) {
        fprintf(stderr, "[dma] zero-copy live path needs at least 3 DMA slots, got %d\n", d->slot_count);
        return -1;
    }
    fprintf(stderr, "[dma] BGRX zero-copy slots=%d frame=%ux%u bpp=%u size=%zu\n",
            d->slot_count, d->frame_w, d->frame_h, d->frame_bpp, d->frame_size);
    return 0;
}

void lpr_dma_release(struct dma_state *d)
{
    if (!d) return;
    for (int i = 0; i < d->slot_count; i++) {
        if (d->slots[i].data)
            munmap(d->slots[i].data, d->slots[i].size);
        d->slots[i].data = NULL;
    }
    if (d->fd >= 0)
        close(d->fd);
    if (d->lock_init) {
        pthread_mutex_destroy(&d->lock);
        pthread_cond_destroy(&d->cond);
    }
    memset(d, 0, sizeof(*d));
    d->fd = -1;
}

int lpr_dma_acquire_slot(struct dma_state *d)
{
    int best = -1;
    pthread_mutex_lock(&d->lock);
    while (best < 0) {
        for (int i = 0; i < d->slot_count; i++) {
            if (d->slots[i].refs == 0) {
                best = i;
                d->slots[i].refs = 1;
                d->slots[i].generation++;
                break;
            }
        }
        if (best < 0)
            pthread_cond_wait(&d->cond, &d->lock);
    }
    pthread_mutex_unlock(&d->lock);
    return best;
}

void lpr_dma_slot_addref(struct dma_state *d, int slot)
{
    if (!d || slot < 0 || slot >= d->slot_count)
        return;
    pthread_mutex_lock(&d->lock);
    d->slots[slot].refs++;
    pthread_mutex_unlock(&d->lock);
}

void lpr_dma_slot_release(struct dma_state *d, int slot)
{
    if (!d || slot < 0 || slot >= d->slot_count)
        return;
    pthread_mutex_lock(&d->lock);
    if (d->slots[slot].refs > 0)
        d->slots[slot].refs--;
    if (d->slots[slot].refs == 0)
        pthread_cond_signal(&d->cond);
    pthread_mutex_unlock(&d->lock);
}

uint8_t *lpr_dma_slot_data(struct dma_state *d, int slot)
{
    if (!d || slot < 0 || slot >= d->slot_count)
        return NULL;
    return d->slots[slot].data;
}

uint64_t lpr_dma_slot_generation(struct dma_state *d, int slot)
{
    if (!d || slot < 0 || slot >= d->slot_count)
        return 0;
    return d->slots[slot].generation;
}

int lpr_dma_read_frame_slot(struct dma_state *d, int slot)
{
    struct dma_transfer t;
    if (!d || slot < 0 || slot >= d->slot_count)
        return -1;
    memset(&t, 0, sizeof(t));
    t.size = (uint32_t)d->frame_size;
    t.offset = d->slots[slot].index;
    t.user_buf = 0;
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
