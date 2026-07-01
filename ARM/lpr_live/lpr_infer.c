// SPDX-License-Identifier: GPL-2.0
/* Background inference thread orchestration. */

#include "lpr_infer.h"
#include "lpr_color.h"
#include "lpr_detector.h"
#include "lpr_ocr.h"
#include "lpr_ptype.h"
#include "lpr_warp.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool route_enabled(const struct lpr_route *routes, enum lpr_route_id id)
{
    return id >= 0 && id < LPR_ROUTE_COUNT && routes[id].model != NULL;
}

/* Choose a route from the optional 6-class plate-type classifier output. */
static enum lpr_route_id pick_ptype_route(const struct lpr_route *routes, int cls)
{
    switch (cls) {
    case LPR_PTYPE_GREEN:
        if (route_enabled(routes, LPR_ROUTE_GREEN))
            return LPR_ROUTE_GREEN;
        return LPR_ROUTE_BLUE;
    case LPR_PTYPE_YELLOW:
        if (route_enabled(routes, LPR_ROUTE_YELLOW))
            return LPR_ROUTE_YELLOW;
        if (route_enabled(routes, LPR_ROUTE_POLICE))
            return LPR_ROUTE_POLICE;
        return LPR_ROUTE_BLUE;
    case LPR_PTYPE_POLICE:
        if (route_enabled(routes, LPR_ROUTE_POLICE))
            return LPR_ROUTE_POLICE;
        return LPR_ROUTE_BLUE;
    case LPR_PTYPE_EMBASSY:
        if (route_enabled(routes, LPR_ROUTE_EMBASSY))
            return LPR_ROUTE_EMBASSY;
        if (route_enabled(routes, LPR_ROUTE_POLICE))
            return LPR_ROUTE_POLICE;
        return LPR_ROUTE_BLUE;
    case LPR_PTYPE_BLUE:
    default:
        return LPR_ROUTE_BLUE;
    }
}

static bool ptype_should_apply(const struct live_options *opt, int cls, float conf)
{
    if (cls < LPR_PTYPE_BLUE || cls > LPR_PTYPE_EMBASSY)
        return false;
    if ((cls == LPR_PTYPE_POLICE || cls == LPR_PTYPE_EMBASSY) &&
        conf >= opt->plate_type_classifier_special_min_conf)
        return true;
    return conf >= opt->plate_type_classifier_min_conf;
}

/* Choose a fallback route based on classified plate body color. */
static enum lpr_route_id pick_route(const struct lpr_route *routes, enum plate_color color)
{
    switch (color) {
    case PLATE_COLOR_GREEN:
        if (route_enabled(routes, LPR_ROUTE_GREEN))
            return LPR_ROUTE_GREEN;
        return LPR_ROUTE_BLUE;
    case PLATE_COLOR_YELLOW:
        if (route_enabled(routes, LPR_ROUTE_YELLOW))
            return LPR_ROUTE_YELLOW;
        if (route_enabled(routes, LPR_ROUTE_POLICE))
            return LPR_ROUTE_POLICE;
        return LPR_ROUTE_BLUE;
    case PLATE_COLOR_WHITE:
        if (route_enabled(routes, LPR_ROUTE_POLICE))
            return LPR_ROUTE_POLICE;
        return LPR_ROUTE_BLUE;
    case PLATE_COLOR_BLACK:
        if (route_enabled(routes, LPR_ROUTE_EMBASSY))
            return LPR_ROUTE_EMBASSY;
        if (route_enabled(routes, LPR_ROUTE_POLICE))
            return LPR_ROUTE_POLICE;
        return LPR_ROUTE_BLUE;
    case PLATE_COLOR_BLUE:
    case PLATE_COLOR_UNKNOWN:
    default:
        return LPR_ROUTE_BLUE;
    }
}

static void publish_result(struct infer_state *st, const struct live_result *res)
{
    pthread_mutex_lock(&st->result_lock);
    if (st->result_owned)
        lpr_dma_slot_release(st->dma, st->result.frame_slot);
    st->result = *res;
    st->result_owned = res->frame_slot >= 0;
    pthread_mutex_unlock(&st->result_lock);
}

bool lpr_infer_take_result(struct infer_state *st, struct live_result *res)
{
    bool owned;
    pthread_mutex_lock(&st->result_lock);
    *res = st->result;
    owned = st->result_owned;
    st->result_owned = false;
    st->result.valid = false;
    st->result.frame_slot = -1;
    pthread_mutex_unlock(&st->result_lock);
    return owned;
}

void lpr_infer_release_result_slot(struct infer_state *st, const struct live_result *res)
{
    if (res && res->frame_slot >= 0)
        lpr_dma_slot_release(st->dma, res->frame_slot);
}

void lpr_infer_submit_latest(struct infer_state *st, int slot, uint64_t generation)
{
    pthread_mutex_lock(&st->lock);
    if (st->has_new) {
        lpr_dma_slot_release(st->dma, st->latest_slot);
        st->overwrite_count++;
    }
    lpr_dma_slot_addref(st->dma, slot);
    st->latest_slot = slot;
    st->latest_generation = generation;
    st->seq++;
    st->has_new = true;
    pthread_cond_signal(&st->cond);
    pthread_mutex_unlock(&st->lock);
}

static void *thread_main(void *arg)
{
    struct infer_state *st = (struct infer_state *)arg;
    size_t bgrx_size = (size_t)st->frame_w * (size_t)st->frame_h * 4U;
    uint8_t *cached_bgrx = malloc(bgrx_size);
    uint8_t *det_input = malloc((size_t)st->det_model->in_w * st->det_model->in_h * 3U);
    uint8_t *crop = malloc((size_t)st->frame_w * (size_t)st->frame_h * 3U);
    if (!cached_bgrx || !det_input || !crop) {
        fprintf(stderr, "[bgp-live] infer thread alloc failed\n");
        free(cached_bgrx); free(det_input); free(crop);
        return NULL;
    }

    while (1) {
        uint64_t seq;
        uint64_t generation;
        int slot;
        uint8_t *slot_bgrx;
        const uint8_t *bgrx;
        struct det_box dets[MAX_DETS];
        int det_count = 0;
        int best = -1;
        int crop_w = 0, crop_h = 0;
        char text[64] = "";
        float conf = 0.0f;
        struct ocr_decode_diag diag;
        struct ocr_timing ocr_timing;
        enum plate_color color = PLATE_COLOR_UNKNOWN;
        enum lpr_route_id route_id = LPR_ROUTE_BLUE;
        const char *route_name = "blue";
        int ptype_cls = LPR_PTYPE_UNKNOWN;
        float ptype_conf = 0.0f;
        bool ptype_applied = false;
        double copy_ms = 0.0, warp_ms = 0.0, color_ms = 0.0, ptype_ms = 0.0;
        int64_t t0, t1, t2;
        struct live_result res;

        pthread_mutex_lock(&st->lock);
        while (st->running && !st->has_new)
            pthread_cond_wait(&st->cond, &st->lock);
        if (!st->running && !st->has_new) {
            pthread_mutex_unlock(&st->lock);
            break;
        }
        slot = st->latest_slot;
        generation = st->latest_generation;
        seq = st->seq;
        st->has_new = false;
        pthread_mutex_unlock(&st->lock);
        slot_bgrx = lpr_dma_slot_data(st->dma, slot);
        if (!slot_bgrx) {
            lpr_dma_slot_release(st->dma, slot);
            continue;
        }

        memset(&res, 0, sizeof(res));
        res.frame_slot = slot;
        res.frame_generation = generation;
        res.seq = seq;
        t0 = lpr_mono_us();
        memcpy(cached_bgrx, slot_bgrx, bgrx_size);
        t1 = lpr_mono_us();
        copy_ms = (double)(t1 - t0) / 1000.0;
        bgrx = cached_bgrx;
        t0 = t1;
        if (lpr_detector_run_bgrx(st->det_model, bgrx, st->frame_w, st->frame_h, det_input,
                                  st->opt->det_resize_mode, st->pose_nc, st->class_filter,
                                  st->opt->min_conf, st->opt->nms_iou, st->opt->max_det,
                                  dets, &det_count) < 0) {
            fprintf(stderr, "[bgp-live] infer seq=%" PRIu64 " detector failed\n", seq);
            lpr_dma_slot_release(st->dma, slot);
            continue;
        }
        t1 = lpr_mono_us();
        best = lpr_detector_pick_best(dets, det_count);
        res.det_count = det_count;
        res.best = best;
        if (best >= 0) {
            int64_t tw0 = lpr_mono_us();
            bool warp_ok = lpr_warp_quad_homography_bgrx(bgrx, st->frame_w, st->frame_h, dets[best].quad,
                                                         crop, st->frame_w, st->frame_h, &crop_w, &crop_h);
            int64_t tw1 = lpr_mono_us();
            warp_ms = (double)(tw1 - tw0) / 1000.0;
            if (warp_ok) {
                int64_t tc0 = lpr_mono_us();
                color = lpr_classify_plate_color_bgrx(bgrx, st->frame_w, st->frame_h, &dets[best]);
                int64_t tc1 = lpr_mono_us();
                color_ms = (double)(tc1 - tc0) / 1000.0;

                if (st->ptype_model && st->ptype_model->ctx &&
                    lpr_ptype_run(st->ptype_model, crop, crop_w, crop_h,
                                  &ptype_cls, &ptype_conf, &ptype_ms) == 0 &&
                    ptype_should_apply(st->opt, ptype_cls, ptype_conf)) {
                    route_id = pick_ptype_route(st->routes, ptype_cls);
                    ptype_applied = true;
                } else {
                    route_id = pick_route(st->routes, color);
                }
                const struct lpr_route *route = &st->routes[route_id];
                route_name = route->name;

                memset(&diag, 0, sizeof(diag));
                memset(&ocr_timing, 0, sizeof(ocr_timing));
                if (lpr_ocr_run(route->model, route->keys, st->opt->ocr_preproc_mode,
                                route->decode_family, crop, crop_w, crop_h,
                                text, sizeof(text), &conf, &diag, &ocr_timing) < 0) {
                    snprintf(text, sizeof(text), "UNK");
                    conf = 0.0f;
                }
                res.valid = true;
                res.box = dets[best];
                res.crop_w = crop_w;
                res.crop_h = crop_h;
                res.color = color;
                res.ptype_cls = ptype_cls;
                res.ptype_conf = ptype_conf;
                res.ptype_applied = ptype_applied;
                snprintf(res.route_name, sizeof(res.route_name), "%s", route_name);
                snprintf(res.text, sizeof(res.text), "%s", text);
                res.conf = conf;
                res.blank_ratio = diag.blank_top1_ratio;
            }
        }
        t2 = lpr_mono_us();
        st->infer_count++;
        lpr_dma_slot_addref(st->dma, slot);
        publish_result(st, &res);
        lpr_dma_slot_release(st->dma, slot);
        if (res.valid) {
            printf("[bgp-live] infer_seq=%" PRIu64 " det=%d best=%d cls=%d color=%s "
                   "ptype=%s ptype_conf=%.3f ptype_apply=%d route=%s box=[%d,%d,%d,%d] crop=%dx%d "
                   "text=%s conf=%.3f blank=%.3f copy_ms=%.1f detocr_ms=%.1f det_ms=%.1f ocr_ms=%.1f "
                   "warp_ms=%.1f color_ms=%.1f ptype_ms=%.1f prep_ms=%.1f in_ms=%.1f run_ms=%.1f out_ms=%.1f dec_ms=%.1f overwritten=%" PRIu64 "\n",
                   seq, det_count, best, dets[best].cls,
                   lpr_plate_color_str(color), lpr_ptype_class_str(ptype_cls), ptype_conf,
                   ptype_applied ? 1 : 0, route_name,
                   dets[best].x1, dets[best].y1, dets[best].x2, dets[best].y2,
                   crop_w, crop_h, text, conf, diag.blank_top1_ratio, copy_ms,
                   (double)(t2 - t0) / 1000.0,
                   (double)(t1 - t0) / 1000.0,
                   (double)(t2 - t1) / 1000.0,
                   warp_ms, color_ms, ptype_ms, ocr_timing.prep_ms, ocr_timing.input_ms,
                   ocr_timing.run_ms, ocr_timing.output_ms, ocr_timing.decode_ms,
                   st->overwrite_count);
        } else {
            printf("[bgp-live] infer_seq=%" PRIu64 " det=%d best=%d copy_ms=%.1f det_ms=%.1f overwritten=%" PRIu64 "\n",
                   seq, det_count, best, copy_ms, (double)(t1 - t0) / 1000.0, st->overwrite_count);
        }
        fflush(stdout);
    }

    free(cached_bgrx); free(det_input); free(crop);
    return NULL;
}

int lpr_infer_start(struct infer_state *st, const struct live_options *opt,
                    struct dma_state *dma,
                    struct rknn_model *det_model,
                    struct rknn_model *ptype_model,
                    const struct lpr_route *routes_in,
                    int pose_nc, int class_filter,
                    int frame_w, int frame_h)
{
    memset(st, 0, sizeof(*st));
    st->opt = opt;
    st->dma = dma;
    st->latest_slot = -1;
    st->result.frame_slot = -1;
    st->det_model = det_model;
    st->ptype_model = ptype_model;
    for (int i = 0; i < LPR_ROUTE_COUNT; i++)
        st->routes[i] = routes_in[i];
    st->pose_nc = pose_nc;
    st->class_filter = class_filter;
    st->frame_w = frame_w;
    st->frame_h = frame_h;
    pthread_mutex_init(&st->lock, NULL);
    pthread_cond_init(&st->cond, NULL);
    pthread_mutex_init(&st->result_lock, NULL);
    st->running = true;
    if (pthread_create(&st->thread, NULL, thread_main, st) != 0)
        return -1;
    st->thread_started = true;
    return 0;
}

void lpr_infer_stop(struct infer_state *st)
{
    if (!st)
        return;
    pthread_mutex_lock(&st->lock);
    st->running = false;
    pthread_cond_broadcast(&st->cond);
    pthread_mutex_unlock(&st->lock);
    if (st->thread_started)
        pthread_join(st->thread, NULL);
    if (st->result_owned)
        lpr_dma_slot_release(st->dma, st->result.frame_slot);
    pthread_mutex_destroy(&st->lock);
    pthread_cond_destroy(&st->cond);
    pthread_mutex_destroy(&st->result_lock);
    memset(st, 0, sizeof(*st));
}
