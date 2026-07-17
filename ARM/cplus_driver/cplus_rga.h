// SPDX-License-Identifier: MIT
#ifndef CPLUS_DRIVER_RGA_H
#define CPLUS_DRIVER_RGA_H

#include <stddef.h>
#include <stdint.h>

/* RGA hardware color conversion: NV12 source -> BGRx destination with
 * aspect-ratio-preserving scaling and centered black borders.
 *
 * librga 1.10.x (im2d v2) on RK3568 requires explicit buffer import via
 * importbuffer_virtualaddr before blitting, otherwise the rga_buffer_t.handle
 * stays 0 and the kernel RGA driver triggers a translation-fault Oops while
 * building the DMA mapping. See cplus_rga.c for the exact im2d calls.
 *
 * The im2d types are kept out of this header so translation units that do not
 * use RGA do not need /usr/include/rga on their include path. */

#define CPLUS_RGA_DST_SIZE(w, h) ((size_t)(w) * (h) * 4U)

struct cplus_rga;

/* Initialize RGA conversion. src_* is the source video resolution (e.g.
 * 1920x1080), dst_* is the output (1280x720). Returns 0 on success. */
int cplus_rga_init(struct cplus_rga **rga_out, int src_width, int src_height,
                   int dst_width, int dst_height);

/* Convert one NV12 frame (src, src_size = w*h*3/2) into the internal BGRX
 * destination buffer. Returns a pointer to the BGRX frame (dst_w*dst_h*4) or
 * NULL on failure. The pointer stays valid until the next convert / destroy. */
const uint8_t *cplus_rga_convert(struct cplus_rga *rga, const uint8_t *nv12,
                                 size_t src_size);

/* Size of the BGRX destination buffer in bytes. */
size_t cplus_rga_dst_size(const struct cplus_rga *rga);

void cplus_rga_destroy(struct cplus_rga *rga);

#endif /* CPLUS_DRIVER_RGA_H */
