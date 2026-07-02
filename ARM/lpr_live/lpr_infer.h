// SPDX-License-Identifier: GPL-2.0
/* Background inference thread that owns the latest captured frame and
 * publishes the most recent live_result. */

#ifndef LPR_LIVE_LPR_INFER_H
#define LPR_LIVE_LPR_INFER_H

#include "lpr_common.h"
#include "lpr_dma.h"
#include "../ocr_decode.h"   /* for enum ocr_decode_family */

#include <pthread.h>

/* Per-route binding: model + per-route keys + decode family + display label.
 * Routes can share model/keys (e.g. blue and unknown both use blue), or be
 * disabled by setting `model = NULL`.
 *
 * The route table is small (5 entries) so we just embed it in the infer
 * state instead of heap-allocating. */
enum lpr_route_id {
    LPR_ROUTE_BLUE = 0,
    LPR_ROUTE_GREEN,
    LPR_ROUTE_POLICE,
    LPR_ROUTE_EMBASSY,
    LPR_ROUTE_YELLOW,
    LPR_ROUTE_COUNT,
};

struct lpr_route {
    struct rknn_model *model;        /* may be NULL if disabled */
    const struct ocr_keys *keys;
    enum ocr_decode_family decode_family;
    char display_tag;                /* one-char tag drawn in overlay (e.g. 'B','G','P','E','Y') */
    char name[8];                    /* human label "blue" etc */
};

struct infer_state {
    pthread_t thread;
    bool thread_started;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    pthread_mutex_t result_lock;
    bool running;
    bool has_new;
    uint8_t *pending_bgrx;
    uint8_t *processing_bgrx;
    size_t bgrx_size;
    uint64_t latest_generation;
    double latest_copy_ms;
    uint64_t seq;
    uint64_t overwrite_count;
    uint64_t infer_count;
    struct dma_state *dma;
    /* Pending slot awaiting async copy by the infer thread. The main loop
     * addrefs this slot and hands it off without copying; the infer thread
     * memcpy's it into processing_bgrx and releases it. -1 = none pending. */
    int pending_slot;
    int frame_w;
    int frame_h;
    struct live_result result;

    const struct live_options *opt;
    struct rknn_model *det_model;
    struct rknn_model *ptype_model;
    struct lpr_route routes[LPR_ROUTE_COUNT];
    int pose_nc;
    int class_filter;
};

int lpr_infer_start(struct infer_state *st, const struct live_options *opt,
                    struct dma_state *dma,
                    struct rknn_model *det_model,
                    struct rknn_model *ptype_model,
                    const struct lpr_route *routes_in,  /* array of LPR_ROUTE_COUNT */
                    int pose_nc, int class_filter,
                    int frame_w, int frame_h);

void lpr_infer_stop(struct infer_state *st);

/* Submit a freshly DMA-filled BGRX slot for background inference. Does NOT
 * copy: the slot is addref'd and handed to the infer thread, which copies it
 * asynchronously. This keeps the 3.6MB memcpy off the display critical path.
 * The caller must NOT release the slot on behalf of the infer thread. */
void lpr_infer_submit_latest(struct infer_state *st, int slot, uint64_t generation);

/* Snapshot the most recent published result. Returns true if a valid OCR result
 * is available for overlay. The result does not own a DMA slot. */
bool lpr_infer_get_result(struct infer_state *st, struct live_result *res);

#endif /* LPR_LIVE_LPR_INFER_H */
