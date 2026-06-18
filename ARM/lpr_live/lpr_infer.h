// SPDX-License-Identifier: GPL-2.0
/* Background inference thread that owns the latest captured frame and
 * publishes the most recent live_result. */

#ifndef LPR_LIVE_LPR_INFER_H
#define LPR_LIVE_LPR_INFER_H

#include "lpr_common.h"
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
    uint64_t seq;
    uint64_t overwrite_count;
    uint64_t infer_count;
    uint8_t *latest_rgb;
    size_t rgb_size;
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
                    struct rknn_model *det_model,
                    struct rknn_model *ptype_model,
                    const struct lpr_route *routes_in,  /* array of LPR_ROUTE_COUNT */
                    int pose_nc, int class_filter,
                    int frame_w, int frame_h);

void lpr_infer_stop(struct infer_state *st);

/* Push a fresh RGB888 frame; if a previous unconsumed frame is still pending
 * it will be silently overwritten and `overwrite_count` is incremented. */
void lpr_infer_submit_latest(struct infer_state *st, const uint8_t *rgb);

/* Read the most recent published result (thread-safe snapshot copy). */
void lpr_infer_get_result(struct infer_state *st, struct live_result *res);

#endif /* LPR_LIVE_LPR_INFER_H */
