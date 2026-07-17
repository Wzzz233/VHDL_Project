// SPDX-License-Identifier: MIT
/* RGA hardware NV12->BGRx conversion with aspect-ratio letterboxing.
 *
 * Verified path on librga 1.10.1 (im2d v2):
 *   importbuffer_virtualaddr(va, im_handle_param_t*) -> rga_buffer_handle_t
 *   wrapbuffer_handle(handle, w, h, format) -> rga_buffer_t (handle filled)
 *   improcess(src, dst, pat, srect, drect, prect, usage)   (all by value)
 * imresize()/imresize_t() fill rect {0,0,0,0} for handle-based buffers and the
 * driver returns "Device or resource busy", so improcess with explicit rects
 * is mandatory. im_rect only has {x,y,width,height} (no stride/format fields). */

#define _GNU_SOURCE
#include "cplus_rga.h"

#include <im2d.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct cplus_rga {
	int src_width;
	int src_height;
	int dst_width;
	int dst_height;
	uint8_t *dst_buffer;          /* BGRX, dst_w*dst_h*4 */
	size_t dst_size;
	rga_buffer_handle_t dst_handle;
	rga_buffer_t src_buf;
	rga_buffer_t dst_buf;
	im_rect src_rect;             /* full source frame {0,0,w,h} */
	im_rect dst_rect;             /* centered scaled rect */
};

int cplus_rga_init(struct cplus_rga **rga_out, int src_width, int src_height,
                   int dst_width, int dst_height)
{
	struct cplus_rga *rga;
	im_handle_param_t src_param = {0};
	im_handle_param_t dst_param = {0};
	rga_buffer_handle_t src_probe;
	double scale, scale_x, scale_y;
	int scaled_w, scaled_h;

	if (!rga_out || src_width <= 0 || src_height <= 0 ||
	    dst_width <= 0 || dst_height <= 0)
		return -1;

	rga = calloc(1, sizeof(*rga));
	if (!rga)
		return -1;
	rga->src_width = src_width;
	rga->src_height = src_height;
	rga->dst_width = dst_width;
	rga->dst_height = dst_height;
	rga->dst_size = (size_t)dst_width * dst_height * 4U;
	rga->dst_handle = 0;

	if (posix_memalign((void **)&rga->dst_buffer, 64, rga->dst_size) != 0 ||
	    !rga->dst_buffer) {
		free(rga);
		return -1;
	}
	memset(rga->dst_buffer, 0, rga->dst_size);  /* black letterbox border */

	/* Import the fixed destination BGRX buffer once. The source is imported
	 * per-convert because the frame pointer changes every call. */
	dst_param.width = dst_width;
	dst_param.height = dst_height;
	dst_param.format = RK_FORMAT_BGRA_8888;
	rga->dst_handle = importbuffer_virtualaddr(rga->dst_buffer, &dst_param);
	if (!rga->dst_handle) {
		fprintf(stderr, "[rga] importbuffer_virtualaddr(dst) failed\n");
		goto failed;
	}
	rga->dst_buf = wrapbuffer_handle(rga->dst_handle, dst_width, dst_height,
	                                 RK_FORMAT_BGRA_8888);

	/* Probe the source import path once with a throwaway handle so we fail
	 * early on an unsupported source format, then release it. */
	src_param.width = src_width;
	src_param.height = src_height;
	src_param.format = RK_FORMAT_YCbCr_420_SP;
	src_probe = importbuffer_virtualaddr(rga->dst_buffer, &src_param);
	if (!src_probe) {
		fprintf(stderr, "[rga] importbuffer_virtualaddr(src probe) failed\n");
		goto failed;
	}
	releasebuffer_handle(src_probe);

	/* Source wrapbuffer template; handle is re-bound each convert. */
	rga->src_buf = wrapbuffer_handle(0, src_width, src_height,
	                                 RK_FORMAT_YCbCr_420_SP);

	/* Aspect-ratio letterbox: scale to the largest rect that fits, centered.
	 * 1920x1080 -> 1280x720 is exactly 16:9, fills dst with no border. */
	scale_x = (double)dst_width / (double)src_width;
	scale_y = (double)dst_height / (double)src_height;
	scale = scale_x < scale_y ? scale_x : scale_y;
	scaled_w = (int)((double)src_width * scale + 0.5);
	scaled_h = (int)((double)src_height * scale + 0.5);
	if (scaled_w < 1) scaled_w = 1;
	if (scaled_h < 1) scaled_h = 1;

	rga->src_rect.x = 0;
	rga->src_rect.y = 0;
	rga->src_rect.width = src_width;
	rga->src_rect.height = src_height;

	rga->dst_rect.x = (dst_width - scaled_w) / 2;
	rga->dst_rect.y = (dst_height - scaled_h) / 2;
	rga->dst_rect.width = scaled_w;
	rga->dst_rect.height = scaled_h;

	fprintf(stderr, "[rga] init src=%dx%d dst=%dx%d scaled=%dx%d offset=%d,%d\n",
	        src_width, src_height, dst_width, dst_height,
	        scaled_w, scaled_h, rga->dst_rect.x, rga->dst_rect.y);

	*rga_out = rga;
	return 0;

failed:
	cplus_rga_destroy(rga);
	return -1;
}

const uint8_t *cplus_rga_convert(struct cplus_rga *rga, const uint8_t *nv12,
                                 size_t src_size)
{
	rga_buffer_handle_t src_handle;
	rga_buffer_t src_buf;
	im_handle_param_t src_param = {0};
	rga_buffer_t empty_pat;
	IM_STATUS status;

	if (!rga || !nv12)
		return NULL;
	if (src_size != (size_t)rga->src_width * rga->src_height * 3U / 2U) {
		fprintf(stderr, "[rga] convert src_size mismatch: %zu != %zu\n",
		        src_size, (size_t)rga->src_width * rga->src_height * 3U / 2U);
		return NULL;
	}

	/* Re-import the source NV12 frame (pointer changes every call). */
	src_param.width = rga->src_width;
	src_param.height = rga->src_height;
	src_param.format = RK_FORMAT_YCbCr_420_SP;
	src_handle = importbuffer_virtualaddr((void *)nv12, &src_param);
	if (!src_handle) {
		fprintf(stderr, "[rga] importbuffer_virtualaddr(src) failed\n");
		return NULL;
	}
	src_buf = wrapbuffer_handle(src_handle, rga->src_width, rga->src_height,
	                            RK_FORMAT_YCbCr_420_SP);

	memset(&empty_pat, 0, sizeof(empty_pat));   /* no pattern blit */
	im_rect empty_rect = {0, 0, 0, 0};
	status = improcess(src_buf, rga->dst_buf, empty_pat,
	                   rga->src_rect, rga->dst_rect, empty_rect,
	                   IM_SYNC);
	releasebuffer_handle(src_handle);
	if (status != IM_STATUS_SUCCESS && status != IM_STATUS_NOERROR) {
		fprintf(stderr, "[rga] improcess failed status=%d\n", (int)status);
		return NULL;
	}
	return rga->dst_buffer;
}

size_t cplus_rga_dst_size(const struct cplus_rga *rga)
{
	return rga ? rga->dst_size : 0;
}

void cplus_rga_destroy(struct cplus_rga *rga)
{
	if (!rga)
		return;
	if (rga->dst_handle)
		releasebuffer_handle(rga->dst_handle);
	free(rga->dst_buffer);
	free(rga);
}
