// SPDX-License-Identifier: GPL-2.0
/* FPGA DMA frame capture and pixel format conversion. */

#include "lpr_dma.h"
#include "../pcie_fpga_dma.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

int lpr_dma_init(struct dma_state *d, const struct live_options *opt)
{
    struct fpga_info info;
    uint8_t *buffers[LPR_DMA_MAX_SLOTS];
    size_t capacities[LPR_DMA_MAX_SLOTS];
    uint32_t fmt;
    int requested = LPR_DMA_DEFAULT_SLOTS;
    int rc;

    memset(d, 0, sizeof(*d));
    d->fd = -1;
    atomic_init(&d->source_generation, 1);
    atomic_init(&d->fpga_caps, 0);
    if (opt->frame_stamp_check)
        atomic_fetch_or_explicit(&d->fpga_caps,
                                 LPR_FRAME_FPGA_CAP_FRAME_STAMP,
                                 memory_order_relaxed);
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
        buffers[i] = d->slots[i].data;
        capacities[i] = d->slots[i].size;
        d->slot_count++;
    }
    if (d->slot_count < 3) {
        fprintf(stderr, "[dma] zero-copy live path needs at least 3 DMA slots, got %d\n", d->slot_count);
        return -1;
    }
    rc = lpr_frame_pool_init_external(&d->frame_pool, buffers, capacities,
                                      (size_t)d->slot_count);
    if (rc < 0) {
        errno = -rc;
        fprintf(stderr, "[dma] failed to initialize frame pool: %s\n",
                strerror(errno));
        return -1;
    }
    d->frame_pool_init = true;
    atomic_fetch_or_explicit(&d->fpga_caps, LPR_FRAME_FPGA_CAP_DMA,
                             memory_order_relaxed);
    fprintf(stderr, "[dma] BGRX zero-copy slots=%d frame=%ux%u bpp=%u size=%zu\n",
            d->slot_count, d->frame_w, d->frame_h, d->frame_bpp, d->frame_size);
    return 0;
}

void lpr_dma_release(struct dma_state *d)
{
    int rc = 0;

    if (!d) return;
    if (d->frame_pool_init) {
        lpr_frame_pool_shutdown(&d->frame_pool);
        for (int i = 0; i < d->slot_count; i++) {
            if (d->writer_active[i]) {
                lpr_frame_writer_abort(&d->writers[i]);
                d->writer_active[i] = false;
            }
            if (d->ref_owned[i]) {
                struct lpr_frame_ref owned = d->refs[i];

                memset(&d->refs[i], 0, sizeof(d->refs[i]));
                d->ref_owned[i] = false;
                lpr_frame_ref_release(&owned);
            }
        }
        rc = lpr_frame_pool_destroy(&d->frame_pool);
        if (rc < 0) {
            fprintf(stderr,
                    "[dma] frame pool still has live references during shutdown: %s\n",
                    strerror(-rc));
            return;
        }
        d->frame_pool_init = false;
    }
    for (int i = 0; i < d->slot_count; i++) {
        if (d->slots[i].data)
            munmap(d->slots[i].data, d->slots[i].size);
        d->slots[i].data = NULL;
    }
    if (d->fd >= 0)
        close(d->fd);
    memset(d, 0, sizeof(*d));
    d->fd = -1;
}

int lpr_dma_acquire_slot(struct dma_state *d)
{
    struct lpr_frame_writer writer;
    int rc;
    int slot;

    if (!d || !d->frame_pool_init)
        return -1;
    rc = lpr_frame_pool_acquire(&d->frame_pool, &writer, true);
    if (rc < 0) {
        errno = -rc;
        return -1;
    }
    slot = (int)writer.slot;
    if (slot < 0 || slot >= d->slot_count) {
        lpr_frame_writer_abort(&writer);
        errno = EIO;
        return -1;
    }
    d->writers[slot] = writer;
    d->writer_active[slot] = true;
    d->ref_owned[slot] = false;
    memset(&d->refs[slot], 0, sizeof(d->refs[slot]));
    return slot;
}

void lpr_dma_slot_release(struct dma_state *d, int slot)
{
    struct lpr_frame_ref anonymous_ref;
    uint64_t generation;

    if (!d || slot < 0 || slot >= d->slot_count)
        return;
    if (d->writer_active[slot]) {
        lpr_frame_writer_abort(&d->writers[slot]);
        d->writer_active[slot] = false;
        return;
    }
    if (!d->ref_owned[slot] ||
        !lpr_frame_ref_is_valid(&d->refs[slot]))
        return;
    anonymous_ref = d->refs[slot];
    generation = anonymous_ref.slot_generation;
    memset(&d->refs[slot], 0, sizeof(d->refs[slot]));
    d->ref_owned[slot] = false;
    if (lpr_frame_ref_release(&anonymous_ref) < 0)
        fprintf(stderr, "[dma] failed to release slot %d generation=%" PRIu64 "\n",
                slot, generation);
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
    if (d->writer_active[slot])
        return d->writers[slot].slot_generation;
    return d->refs[slot].slot_generation;
}

int lpr_dma_slot_ref_clone(struct dma_state *d, int slot,
                           struct lpr_frame_ref *out)
{
    if (!d || !out || slot < 0 || slot >= d->slot_count ||
        d->writer_active[slot] || !d->ref_owned[slot])
        return -EINVAL;
    return lpr_frame_ref_clone(&d->refs[slot], out);
}

void lpr_dma_set_source_generation(struct dma_state *d,
                                   uint64_t source_generation)
{
    if (d)
        atomic_store_explicit(&d->source_generation, source_generation,
                              memory_order_relaxed);
}

int lpr_dma_get_frame_status(struct dma_state *d, struct fpga_frame_status *status)
{
    if (!d || d->fd < 0 || !status)
        return -1;
    memset(status, 0, sizeof(*status));
    if (ioctl(d->fd, FPGA_DMA_GET_FRAME_STATUS, status) < 0)
        return -1;
    if (status->magic != FPGA_FRAME_STATUS_MAGIC)
        return -1;
    atomic_fetch_or_explicit(&d->fpga_caps,
                             LPR_FRAME_FPGA_CAP_FRAME_STATUS,
                             memory_order_relaxed);
    if (status->camera_magic == FPGA_CAMERA_STATUS_MAGIC)
        atomic_fetch_or_explicit(&d->fpga_caps,
                                 LPR_FRAME_FPGA_CAP_CAMERA_STATUS,
                                 memory_order_relaxed);
    return 0;
}

int lpr_dma_wait_new_frame(struct dma_state *d, uint32_t *last_change_count, int timeout_ms)
{
    struct fpga_frame_status status;
    int waited_ms = 0;

    if (!last_change_count)
        return -1;
    while (timeout_ms <= 0 || waited_ms < timeout_ms) {
        if (lpr_dma_get_frame_status(d, &status) < 0)
            return -1;
        if (status.frame_change_count != *last_change_count) {
            *last_change_count = status.frame_change_count;
            return 0;
        }
        usleep(1000);
        waited_ms++;
    }
    return -1;
}

int lpr_dma_read_frame_slot(struct dma_state *d, int slot)
{
    struct dma_transfer t;
    struct lpr_frame_meta meta;
    int rc;

    if (!d || slot < 0 || slot >= d->slot_count ||
        !d->writer_active[slot])
        return -1;
    memset(&t, 0, sizeof(t));
    t.size = (uint32_t)d->frame_size;
    t.offset = d->slots[slot].index;
    t.user_buf = 0;
    if (ioctl(d->fd, FPGA_DMA_READ_FRAME, &t) < 0)
        return -1;
    if (t.result != 0) {
        errno = EIO;
        return -1;
    }
    memset(&meta, 0, sizeof(meta));
    meta.format = LPR_FRAME_FORMAT_BGRX8888;
    meta.width = d->frame_w;
    meta.height = d->frame_h;
    meta.stride = d->frame_w * 4U;
    meta.monotonic_us = lpr_mono_us();
    meta.sequence = ++d->capture_sequence;
    meta.source_generation = atomic_load_explicit(&d->source_generation,
                                                   memory_order_relaxed);
    meta.fpga_caps = atomic_load_explicit(&d->fpga_caps,
                                           memory_order_relaxed);
    rc = lpr_frame_writer_publish(&d->writers[slot], &meta, &d->refs[slot]);
    if (rc < 0) {
        errno = -rc;
        return -1;
    }
    d->writer_active[slot] = false;
    d->ref_owned[slot] = true;
    return 0;
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
