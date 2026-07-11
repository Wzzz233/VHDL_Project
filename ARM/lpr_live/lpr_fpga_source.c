// SPDX-License-Identifier: GPL-2.0
/* OV5640/FPGA DMA frame source. */

#include "lpr_fpga_source.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static int fpga_open(void *ctx);
static int fpga_start(void *ctx);
static int fpga_read_latest(void *ctx, struct lpr_frame_ref *out);
static int fpga_health(void *ctx, int64_t now_us,
                       struct lpr_source_health *out);
static void fpga_stop(void *ctx);
static void fpga_close(void *ctx);

static const struct lpr_frame_source_ops fpga_ops = {
    .open = fpga_open,
    .start = fpga_start,
    .read_latest = fpga_read_latest,
    .health = fpga_health,
    .stop = fpga_stop,
    .close = fpga_close,
};

static int read_exact_file(const char *path, uint8_t *data, size_t size)
{
    size_t offset = 0;
    int fd = open(path, O_RDONLY | O_CLOEXEC);

    if (fd < 0)
        return -errno;
    while (offset < size) {
        ssize_t count = read(fd, data + offset, size - offset);

        if (count < 0) {
            int rc = errno == EINTR ? 0 : -errno;
            if (rc == 0)
                continue;
            close(fd);
            return rc;
        }
        if (count == 0) {
            close(fd);
            return -EINVAL;
        }
        offset += (size_t)count;
    }
    {
        uint8_t extra;
        ssize_t count = read(fd, &extra, 1);
        close(fd);
        return count == 0 ? 0 : -EFBIG;
    }
}

static int file_open(struct lpr_fpga_source *source)
{
    const size_t frame_size = (size_t)LPR_FPGA_FRAME_WIDTH *
                              LPR_FPGA_FRAME_HEIGHT * 4U;
    int rc;

    source->file_pixels = malloc(frame_size);
    if (!source->file_pixels)
        return -ENOMEM;
    rc = read_exact_file(source->options->input_bgrx_path,
                         source->file_pixels, frame_size);
    if (rc < 0)
        goto fail;
    rc = lpr_frame_pool_init_heap(&source->file_pool, 3, frame_size);
    if (rc < 0)
        goto fail;
    source->file_pool_init = true;
    source->file_size = frame_size;
    source->file_mode = true;
    source->dma.frame_w = LPR_FPGA_FRAME_WIDTH;
    source->dma.frame_h = LPR_FPGA_FRAME_HEIGHT;
    source->dma.frame_bpp = 4U;
    source->dma.frame_size = frame_size;
    source->dma.src_is_bgrx = true;
    fprintf(stderr, "[image-source] loaded %s (%zu bytes BGRx)\n",
            source->options->input_bgrx_path, frame_size);
    return 0;

fail:
    free(source->file_pixels);
    source->file_pixels = NULL;
    return rc;
}

static uint32_t fpga_camera_status_words(
    const struct fpga_frame_status *status)
{
    return status->camera_shape & 0x000fffffU;
}

static uint32_t fpga_camera_status_lines(
    const struct fpga_frame_status *status)
{
    return (status->camera_shape >> 20) & 0x00000fffU;
}

int lpr_fpga_source_init(struct lpr_fpga_source *source,
                         const struct live_options *options,
                         uint64_t source_generation)
{
    int rc;

    if (!source || !options || !options->device_path)
        return -EINVAL;
    memset(source, 0, sizeof(*source));
    source->dma.fd = -1;
    source->options = options;
    source->source_generation = source_generation;
    source->source.id = LPR_SOURCE_FPGA;
    source->source.name = "ov5640";
    source->source.ops = &fpga_ops;
    source->source.ctx = source;
    rc = pthread_mutex_init(&source->lock, NULL);
    if (rc != 0)
        return -rc;
    source->lock_init = true;
    return 0;
}

struct lpr_frame_source *lpr_fpga_source_as_frame_source(
    struct lpr_fpga_source *source)
{
    return source ? &source->source : NULL;
}

struct dma_state *lpr_fpga_source_dma(struct lpr_fpga_source *source)
{
    return source ? &source->dma : NULL;
}

void lpr_fpga_source_set_generation(struct lpr_fpga_source *source,
                                    uint64_t source_generation)
{
    if (!source || !source->lock_init)
        return;
    pthread_mutex_lock(&source->lock);
    source->source_generation = source_generation;
    source->has_frame = false;
    source->last_frame_us = 0;
    source->previous_frame_us = 0;
    source->input_fps = 0.0;
    if (!source->file_mode)
        lpr_dma_set_source_generation(&source->dma, source_generation);
    pthread_mutex_unlock(&source->lock);
}

static int fpga_open(void *ctx)
{
    struct lpr_fpga_source *source = ctx;
    struct fpga_frame_status status;
    int saved_errno;

    if (!source || !source->lock_init)
        return -EINVAL;
    pthread_mutex_lock(&source->lock);
    if (source->opened) {
        pthread_mutex_unlock(&source->lock);
        return 0;
    }
    pthread_mutex_unlock(&source->lock);

    if (source->options->input_bgrx_path) {
        int rc = file_open(source);
        if (rc < 0)
            return rc;
    } else if (lpr_dma_init(&source->dma, source->options) < 0) {
        int rc = errno ? -errno : -EIO;
        lpr_dma_release(&source->dma);
        return rc;
    }
    if (source->dma.frame_w != LPR_FPGA_FRAME_WIDTH ||
        source->dma.frame_h != LPR_FPGA_FRAME_HEIGHT ||
        source->dma.frame_bpp != 4U || !source->dma.src_is_bgrx) {
        fprintf(stderr,
                "[fpga-source] expected 1280x720 BGRx, got %ux%u bpp=%u\n",
                source->dma.frame_w, source->dma.frame_h,
                source->dma.frame_bpp);
        lpr_dma_release(&source->dma);
        return -EINVAL;
    }
    if (!source->file_mode)
        lpr_dma_set_source_generation(&source->dma,
                                      source->source_generation);

    if (source->file_mode)
        goto ready;

    memset(&status, 0, sizeof(status));
    errno = 0;
    if (lpr_dma_get_frame_status(&source->dma, &status) == 0) {
        source->frame_status_available = true;
        source->last_frame_change_count = status.frame_change_count;
        fprintf(stderr,
                "[fpga-source] frame-status counter=%u changes=%u flags=0x%08x magic=0x%08x\n",
                status.frame_counter, status.frame_change_count, status.flags,
                status.magic);
        if (status.camera_magic == FPGA_CAMERA_STATUS_MAGIC) {
            source->camera_status_available = true;
            fprintf(stderr,
                    "[fpga-source] camera-status frames=%u lines=%u words=%u hash=0x%08x magic=0x%08x\n",
                    status.camera_frame_counter,
                    fpga_camera_status_lines(&status),
                    fpga_camera_status_words(&status), status.camera_hash,
                    status.camera_magic);
        } else {
            fprintf(stderr,
                    "[fpga-source] camera-status unavailable raw magic=0x%08x expected=0x%08x\n",
                    status.camera_magic, FPGA_CAMERA_STATUS_MAGIC);
        }
    } else {
        saved_errno = errno;
        fprintf(stderr,
                "[fpga-source] frame-status unavailable counter=%u changes=%u flags=0x%08x magic=0x%08x expected=0x%08x ioctl_errno=%d (%s)\n",
                status.frame_counter, status.frame_change_count, status.flags,
                status.magic, FPGA_FRAME_STATUS_MAGIC, saved_errno,
                saved_errno ? strerror(saved_errno) : "none");
        if (source->options->wait_new_frame) {
            lpr_dma_release(&source->dma);
            return saved_errno ? -saved_errno : -ENOTSUP;
        }
    }

ready:
    pthread_mutex_lock(&source->lock);
    source->opened = true;
    source->healthy = true;
    source->has_frame = false;
    source->last_frame_us = 0;
    source->previous_frame_us = 0;
    source->input_fps = 0.0;
    pthread_mutex_unlock(&source->lock);
    return 0;
}

static int fpga_start(void *ctx)
{
    struct lpr_fpga_source *source = ctx;

    if (!source || !source->lock_init)
        return -EINVAL;
    pthread_mutex_lock(&source->lock);
    if (!source->opened) {
        pthread_mutex_unlock(&source->lock);
        return -ENODEV;
    }
    source->running = true;
    source->healthy = true;
    pthread_mutex_unlock(&source->lock);
    return 0;
}

static int fpga_read_latest(void *ctx, struct lpr_frame_ref *out)
{
    struct lpr_fpga_source *source = ctx;
    struct lpr_frame_ref captured;
    uint64_t source_generation;
    int slot;
    int rc;
    int64_t now_us;

    if (!source || !out || !source->lock_init)
        return -EINVAL;
    memset(out, 0, sizeof(*out));
    pthread_mutex_lock(&source->lock);
    if (!source->opened || !source->running) {
        pthread_mutex_unlock(&source->lock);
        return -ESHUTDOWN;
    }
    pthread_mutex_unlock(&source->lock);

    if (source->file_mode) {
        struct lpr_frame_writer writer = {0};
        struct lpr_frame_meta meta = {0};
        uint8_t *destination;

        rc = lpr_frame_pool_acquire(&source->file_pool, &writer, true);
        if (rc < 0)
            goto fail;
        destination = lpr_frame_writer_data(&writer);
        if (!destination ||
            lpr_frame_writer_capacity(&writer) < source->file_size) {
            lpr_frame_writer_abort(&writer);
            rc = -EIO;
            goto fail;
        }
        memcpy(destination, source->file_pixels, source->file_size);
        pthread_mutex_lock(&source->lock);
        source_generation = source->source_generation;
        meta.sequence = ++source->sequence;
        pthread_mutex_unlock(&source->lock);
        meta.format = LPR_FRAME_FORMAT_BGRX8888;
        meta.width = LPR_FPGA_FRAME_WIDTH;
        meta.height = LPR_FPGA_FRAME_HEIGHT;
        meta.stride = LPR_FPGA_FRAME_WIDTH * 4U;
        meta.monotonic_us = lpr_mono_us();
        meta.source_generation = source_generation;
        rc = lpr_frame_writer_publish(&writer, &meta, out);
        if (rc < 0) {
            lpr_frame_writer_abort(&writer);
            goto fail;
        }
        now_us = meta.monotonic_us;
        goto record_health;
    }

    if (source->options->wait_new_frame &&
        lpr_dma_wait_new_frame(&source->dma,
                               &source->last_frame_change_count, 1000) < 0) {
        rc = errno ? -errno : -ETIMEDOUT;
        goto fail;
    }
    slot = lpr_dma_acquire_slot(&source->dma);
    if (slot < 0) {
        rc = errno ? -errno : -EIO;
        goto fail;
    }
    if (source->options->dma_pre_delay_us > 0)
        usleep((useconds_t)source->options->dma_pre_delay_us);
    if (lpr_dma_read_frame_slot(&source->dma, slot) < 0) {
        rc = errno ? -errno : -EIO;
        lpr_dma_slot_release(&source->dma, slot);
        goto fail;
    }
    memset(&captured, 0, sizeof(captured));
    rc = lpr_dma_slot_ref_clone(&source->dma, slot, &captured);
    lpr_dma_slot_release(&source->dma, slot);
    if (rc < 0)
        goto fail;

    pthread_mutex_lock(&source->lock);
    source_generation = source->source_generation;
    pthread_mutex_unlock(&source->lock);
    if (captured.meta.source_generation != source_generation) {
        rc = lpr_frame_ref_clone_rebind_source_generation(
            &captured, source_generation, out);
        lpr_frame_ref_release(&captured);
        if (rc < 0)
            goto fail;
    } else {
        *out = captured;
    }

    now_us = out->meta.monotonic_us;
record_health:
    pthread_mutex_lock(&source->lock);
    if (source->previous_frame_us > 0 && now_us > source->previous_frame_us) {
        double instantaneous = 1000000.0 /
            (double)(now_us - source->previous_frame_us);
        source->input_fps = source->input_fps > 0.0
            ? source->input_fps * 0.9 + instantaneous * 0.1
            : instantaneous;
    }
    source->previous_frame_us = now_us;
    source->last_frame_us = now_us;
    source->sequence = out->meta.sequence;
    source->has_frame = true;
    source->healthy = true;
    pthread_mutex_unlock(&source->lock);
    return 0;

fail:
    pthread_mutex_lock(&source->lock);
    source->dropped_frames++;
    source->error_count++;
    source->healthy = false;
    pthread_mutex_unlock(&source->lock);
    return rc;
}

static int fpga_health(void *ctx, int64_t now_us,
                       struct lpr_source_health *out)
{
    struct lpr_fpga_source *source = ctx;

    if (!source || !out || !source->lock_init)
        return -EINVAL;
    memset(out, 0, sizeof(*out));
    pthread_mutex_lock(&source->lock);
    out->opened = source->opened;
    out->running = source->running;
    out->healthy = source->healthy && source->opened && source->running;
    out->has_frame = source->has_frame;
    out->last_frame_us = source->last_frame_us;
    if (source->has_frame && now_us >= source->last_frame_us)
        out->frame_age_us = now_us - source->last_frame_us;
    else
        out->frame_age_us = -1;
    out->sequence = source->sequence;
    out->input_fps = source->input_fps;
    out->dropped_frames = source->dropped_frames;
    out->error_count = source->error_count;
    pthread_mutex_unlock(&source->lock);
    return 0;
}

static void fpga_stop(void *ctx)
{
    struct lpr_fpga_source *source = ctx;

    if (!source || !source->lock_init)
        return;
    pthread_mutex_lock(&source->lock);
    source->running = false;
    pthread_mutex_unlock(&source->lock);
}

static void fpga_close(void *ctx)
{
    struct lpr_fpga_source *source = ctx;
    bool was_opened;

    if (!source || !source->lock_init)
        return;
    fpga_stop(source);
    pthread_mutex_lock(&source->lock);
    was_opened = source->opened;
    source->opened = false;
    source->healthy = false;
    pthread_mutex_unlock(&source->lock);
    if (source->file_mode) {
        if (source->file_pool_init) {
            lpr_frame_pool_shutdown(&source->file_pool);
            lpr_frame_pool_destroy(&source->file_pool);
            source->file_pool_init = false;
        }
        free(source->file_pixels);
        source->file_pixels = NULL;
    } else if (was_opened || source->dma.fd >= 0 ||
               source->dma.frame_pool_init) {
        lpr_dma_release(&source->dma);
    }
    pthread_mutex_destroy(&source->lock);
    source->lock_init = false;
}
