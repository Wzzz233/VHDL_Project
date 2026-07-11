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

static bool publish_result_locked(struct infer_state *st,
                                  const struct live_result *res)
{
    if (res->source_generation !=
        atomic_load(&st->accepted_source_generation))
        return false;
    pthread_mutex_lock(&st->result_lock);
    if (res->source_generation ==
        atomic_load(&st->accepted_source_generation))
        st->result = *res;
    else {
        pthread_mutex_unlock(&st->result_lock);
        return false;
    }
    pthread_mutex_unlock(&st->result_lock);
    return true;
}

bool lpr_infer_publish_result(struct infer_state *st,
                              const struct live_result *res)
{
    bool published;

    if (!st || !res)
        return false;
    pthread_mutex_lock(&st->epoch_lock);
    published = publish_result_locked(st, res);
    pthread_mutex_unlock(&st->epoch_lock);
    return published;
}

struct infer_plate_log {
    bool warp_ok;
    double warp_ms;
    double color_ms;
    double ptype_ms;
    struct ocr_timing ocr;
};

bool lpr_infer_get_result(struct infer_state *st, struct live_result *res)
{
    bool valid;

    if (!st || !res)
        return false;
    pthread_mutex_lock(&st->result_lock);
    *res = st->result;
    valid = st->result.valid && st->result.source_generation ==
            atomic_load(&st->accepted_source_generation);
    if (!valid)
        res->valid = false;
    pthread_mutex_unlock(&st->result_lock);
    return valid;
}

int lpr_infer_submit_latest(struct infer_state *st,
                            const struct lpr_frame_ref *frame)
{
    struct lpr_frame_ref replacement = {0};
    struct lpr_frame_ref displaced = {0};

    if (!st || !frame || !lpr_frame_ref_is_valid(frame) ||
        frame->meta.format != LPR_FRAME_FORMAT_BGRX8888 ||
        frame->meta.width != (uint32_t)st->frame_w ||
        frame->meta.height != (uint32_t)st->frame_h ||
        frame->meta.stride < (uint32_t)st->frame_w * 4U ||
        frame->meta.source_generation !=
            atomic_load(&st->accepted_source_generation))
        return -1;
    if (lpr_frame_ref_clone(frame, &replacement) < 0)
        return -1;

    pthread_mutex_lock(&st->lock);
    if (!st->running ||
        frame->meta.source_generation !=
            atomic_load(&st->accepted_source_generation)) {
        pthread_mutex_unlock(&st->lock);
        lpr_frame_ref_release(&replacement);
        return -1;
    }
    if (st->has_new) {
        displaced = st->pending_frame;
        memset(&st->pending_frame, 0, sizeof(st->pending_frame));
        st->overwrite_count++;
    }
    st->pending_frame = replacement;
    st->latest_copy_ms = 0.0;
    st->seq++;
    st->submit_count++;
    st->has_new = true;
    pthread_cond_signal(&st->cond);
    pthread_mutex_unlock(&st->lock);
    lpr_frame_ref_release(&displaced);
    return 0;
}

void lpr_infer_reset(struct infer_state *st, uint64_t source_generation)
{
    struct lpr_frame_ref pending = {0};

    if (!st)
        return;
    pthread_mutex_lock(&st->epoch_lock);
    pthread_mutex_lock(&st->lock);
    atomic_store(&st->accepted_source_generation, source_generation);
    if (st->has_new) {
        pending = st->pending_frame;
        memset(&st->pending_frame, 0, sizeof(st->pending_frame));
        st->has_new = false;
    }
    st->latest_copy_ms = 0.0;
    st->latest_infer_ms = 0.0;
    pthread_mutex_unlock(&st->lock);

    pthread_mutex_lock(&st->result_lock);
    memset(&st->result, 0, sizeof(st->result));
    st->result.frame_slot = -1;
    st->result.source_generation = source_generation;
    pthread_mutex_unlock(&st->result_lock);
    pthread_mutex_unlock(&st->epoch_lock);
    lpr_frame_ref_release(&pending);
}

static void *thread_main(void *arg)
{
    struct infer_state *st = (struct infer_state *)arg;
    uint8_t *det_input = malloc((size_t)st->det_model->in_w * st->det_model->in_h * 3U);
    uint8_t *crop = malloc((size_t)st->frame_w * (size_t)st->frame_h * 3U);
    if (!det_input || !crop) {
        fprintf(stderr, "[bgp-live] infer thread alloc failed\n");
        free(det_input); free(crop);
        pthread_mutex_lock(&st->lock);
        st->running = false;
        pthread_cond_broadcast(&st->cond);
        pthread_mutex_unlock(&st->lock);
        return NULL;
    }

    while (1) {
        uint64_t seq;
        uint64_t slot_generation;
        uint64_t overwritten;
        const uint8_t *bgrx;
        struct lpr_frame_ref pending_frame = {0};
        struct lpr_frame_meta frame_meta;
        struct det_box dets[MAX_DETS];
        int det_count = 0;
        int best = -1;
        int processed = 0;
        struct det_timing det_timing;
        double copy_ms = 0.0;
        double total_ocr_ms = 0.0, total_warp_ms = 0.0, total_color_ms = 0.0, total_ptype_ms = 0.0;
        int64_t t0, t1, t2;
        struct live_result res;
        struct infer_plate_log plate_logs[MAX_LIVE_PLATES];

        pthread_mutex_lock(&st->lock);
        while (st->running && !st->has_new)
            pthread_cond_wait(&st->cond, &st->lock);
        if (!st->running && !st->has_new) {
            pthread_mutex_unlock(&st->lock);
            break;
        }
        seq = st->seq;
        pending_frame = st->pending_frame;
        memset(&st->pending_frame, 0, sizeof(st->pending_frame));
        st->has_new = false;
        overwritten = st->overwrite_count;
        pthread_mutex_unlock(&st->lock);

        frame_meta = pending_frame.meta;
        slot_generation = pending_frame.slot_generation;
        const uint8_t *src = lpr_frame_ref_data(&pending_frame);
        int64_t tc0 = lpr_mono_us();
        if (src) {
            size_t row_size = (size_t)st->frame_w * 4U;

            for (int y = 0; y < st->frame_h; y++)
                memcpy(st->processing_bgrx + (size_t)y * row_size,
                       src + (size_t)y * frame_meta.stride, row_size);
        }
        int64_t tc1 = lpr_mono_us();
        copy_ms = (double)(tc1 - tc0) / 1000.0;
        lpr_frame_ref_release(&pending_frame);
        if (!src || frame_meta.source_generation !=
                    atomic_load(&st->accepted_source_generation))
            continue;
        bgrx = st->processing_bgrx;

        memset(&res, 0, sizeof(res));
        res.frame_slot = -1;
        res.frame_generation = slot_generation;
        res.input_sequence = frame_meta.sequence;
        res.source_generation = frame_meta.source_generation;
        res.frame_monotonic_us = frame_meta.monotonic_us;
        res.seq = seq;
        t0 = lpr_mono_us();
        memset(&det_timing, 0, sizeof(det_timing));
        if (lpr_detector_run_bgrx_timed(st->det_model, bgrx, st->frame_w, st->frame_h, det_input,
                                        st->opt->det_resize_mode, st->pose_nc, st->class_filter,
                                        st->opt->min_conf, st->opt->det_score_scale,
                                        st->opt->nms_iou, st->opt->max_det,
                                        dets, &det_count, &det_timing) < 0) {
            pthread_mutex_lock(&st->epoch_lock);
            if (res.source_generation ==
                atomic_load(&st->accepted_source_generation))
                fprintf(stderr,
                        "[bgp-live] infer seq=%" PRIu64 " detector failed\n",
                        seq);
            pthread_mutex_unlock(&st->epoch_lock);
            continue;
        }
        t1 = lpr_mono_us();
        best = lpr_detector_pick_best(dets, det_count);
        res.det_count = det_count;
        res.best = best;

        for (int i = 0; i < det_count && processed < MAX_LIVE_PLATES; i++) {
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
            double warp_ms = 0.0, color_ms = 0.0, ptype_ms = 0.0;
            memset(&diag, 0, sizeof(diag));
            memset(&ocr_timing, 0, sizeof(ocr_timing));

            int plate_index = processed++;
            struct live_plate_result *plate = &res.plates[plate_index];
            plate->box = dets[i];

            int64_t tw0 = lpr_mono_us();
            bool warp_ok = lpr_warp_quad_homography_bgrx(bgrx, st->frame_w, st->frame_h, dets[i].quad,
                                                         crop, st->frame_w, st->frame_h, &crop_w, &crop_h);
            int64_t tw1 = lpr_mono_us();
            warp_ms = (double)(tw1 - tw0) / 1000.0;
            total_warp_ms += warp_ms;

            if (warp_ok) {
                int64_t tc0 = lpr_mono_us();
                color = lpr_classify_plate_color_bgrx(bgrx, st->frame_w, st->frame_h, &dets[i]);
                int64_t tc1 = lpr_mono_us();
                color_ms = (double)(tc1 - tc0) / 1000.0;
                total_color_ms += color_ms;

                if (st->ptype_model && st->ptype_model->ctx &&
                    lpr_ptype_run(st->ptype_model, crop, crop_w, crop_h,
                                  &ptype_cls, &ptype_conf, &ptype_ms) == 0 &&
                    ptype_should_apply(st->opt, ptype_cls, ptype_conf)) {
                    route_id = pick_ptype_route(st->routes, ptype_cls);
                    ptype_applied = true;
                } else {
                    route_id = pick_route(st->routes, color);
                }
                total_ptype_ms += ptype_ms;
                const struct lpr_route *route = &st->routes[route_id];
                route_name = route->name;

                if (lpr_ocr_run(route->model, route->keys, st->opt->ocr_preproc_mode,
                                route->decode_family, crop, crop_w, crop_h,
                                text, sizeof(text), &conf, &diag, &ocr_timing) < 0) {
                    snprintf(text, sizeof(text), "UNK");
                    conf = 0.0f;
                }
            } else {
                route_name = "det";
                snprintf(text, sizeof(text), "DET");
                conf = dets[i].conf;
            }
            total_ocr_ms += ocr_timing.prep_ms + ocr_timing.input_ms + ocr_timing.run_ms +
                            ocr_timing.output_ms + ocr_timing.decode_ms;
            plate->crop_w = crop_w;
            plate->crop_h = crop_h;
            plate->color = color;
            plate->ptype_cls = ptype_cls;
            plate->ptype_conf = ptype_conf;
            plate->ptype_applied = ptype_applied;
            snprintf(plate->route_name, sizeof(plate->route_name), "%s", route_name);
            snprintf(plate->text, sizeof(plate->text), "%s", text);
            plate->conf = conf;
            plate->blank_ratio = diag.blank_top1_ratio;
            plate_logs[plate_index].warp_ok = warp_ok;
            plate_logs[plate_index].warp_ms = warp_ms;
            plate_logs[plate_index].color_ms = color_ms;
            plate_logs[plate_index].ptype_ms = ptype_ms;
            plate_logs[plate_index].ocr = ocr_timing;
        }

        res.result_count = processed;
        res.valid = processed > 0;
        t2 = lpr_mono_us();
        res.infer_ms = (double)(t2 - t0) / 1000.0;
        pthread_mutex_lock(&st->epoch_lock);
        if (res.source_generation !=
            atomic_load(&st->accepted_source_generation)) {
            pthread_mutex_unlock(&st->epoch_lock);
            continue;
        }
        if (!publish_result_locked(st, &res)) {
            pthread_mutex_unlock(&st->epoch_lock);
            continue;
        }
        pthread_mutex_lock(&st->lock);
        if (res.source_generation !=
            atomic_load(&st->accepted_source_generation)) {
            pthread_mutex_unlock(&st->lock);
            pthread_mutex_unlock(&st->epoch_lock);
            continue;
        }
        st->infer_count++;
        st->latest_copy_ms = copy_ms;
        st->latest_infer_ms = res.infer_ms;
        pthread_mutex_unlock(&st->lock);
        for (int i = 0; i < processed; i++) {
            const struct live_plate_result *plate = &res.plates[i];
            const struct infer_plate_log *log = &plate_logs[i];

            printf("[bgp-live] plate_seq=%" PRIu64 " idx=%d cls=%d det_conf=%.3f color=%s "
                   "ptype=%s ptype_conf=%.3f ptype_apply=%d route=%s box=[%d,%d,%d,%d] crop=%dx%d warp_ok=%d "
                   "text=%s conf=%.3f blank=%.3f warp_ms=%.1f color_ms=%.1f ptype_ms=%.1f "
                   "prep_ms=%.1f in_ms=%.1f run_ms=%.1f out_ms=%.1f dec_ms=%.1f\n",
                   seq, i, plate->box.cls, plate->box.conf,
                   lpr_plate_color_str(plate->color),
                   lpr_ptype_class_str(plate->ptype_cls),
                   plate->ptype_conf, plate->ptype_applied ? 1 : 0,
                   plate->route_name, plate->box.x1, plate->box.y1,
                   plate->box.x2, plate->box.y2, plate->crop_w, plate->crop_h,
                   log->warp_ok ? 1 : 0, plate->text, plate->conf,
                   plate->blank_ratio, log->warp_ms, log->color_ms,
                   log->ptype_ms, log->ocr.prep_ms, log->ocr.input_ms,
                   log->ocr.run_ms, log->ocr.output_ms, log->ocr.decode_ms);
        }
        if (res.valid) {
            printf("[bgp-live] infer_seq=%" PRIu64 " det=%d results=%d best=%d copy_ms=%.1f detocr_ms=%.1f det_ms=%.1f "
                   "det_prep_ms=%.1f det_in_ms=%.1f det_run_ms=%.1f det_out_ms=%.1f det_dec_ms=%.1f det_nms_ms=%.1f "
                   "ocr_total_ms=%.1f warp_total_ms=%.1f color_total_ms=%.1f ptype_total_ms=%.1f overwritten=%" PRIu64 "\n",
                   seq, det_count, processed, best, copy_ms,
                   (double)(t2 - t0) / 1000.0,
                   (double)(t1 - t0) / 1000.0,
                   det_timing.prep_ms, det_timing.input_ms, det_timing.run_ms,
                   det_timing.output_ms, det_timing.decode_ms, det_timing.nms_ms,
                   total_ocr_ms, total_warp_ms, total_color_ms, total_ptype_ms,
                   overwritten);
        } else {
            printf("[bgp-live] infer_seq=%" PRIu64 " det=%d results=%d best=%d copy_ms=%.1f det_ms=%.1f "
                   "det_prep_ms=%.1f det_in_ms=%.1f det_run_ms=%.1f det_out_ms=%.1f det_dec_ms=%.1f det_nms_ms=%.1f overwritten=%" PRIu64 "\n",
                   seq, det_count, processed, best, copy_ms, (double)(t1 - t0) / 1000.0,
                   det_timing.prep_ms, det_timing.input_ms, det_timing.run_ms,
                   det_timing.output_ms, det_timing.decode_ms, det_timing.nms_ms,
                   overwritten);
        }
        fflush(stdout);
        pthread_mutex_unlock(&st->epoch_lock);
    }

    free(det_input); free(crop);
    return NULL;
}

int lpr_infer_start(struct infer_state *st, const struct live_options *opt,
                    struct rknn_model *det_model,
                    struct rknn_model *ptype_model,
                    const struct lpr_route *routes_in,
                    int pose_nc, int class_filter,
                    int frame_w, int frame_h,
                    uint64_t source_generation)
{
    if (!st || !opt || !det_model || !det_model->ctx || !routes_in ||
        frame_w <= 0 || frame_h <= 0 || source_generation == 0)
        return -1;
    memset(st, 0, sizeof(*st));
    st->opt = opt;
    st->bgrx_size = (size_t)frame_w * (size_t)frame_h * 4U;
    st->processing_bgrx = malloc(st->bgrx_size);
    st->result.frame_slot = -1;
    if (!st->processing_bgrx) {
        free(st->processing_bgrx);
        memset(st, 0, sizeof(*st));
        return -1;
    }
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
    pthread_mutex_init(&st->epoch_lock, NULL);
    atomic_init(&st->accepted_source_generation, source_generation);
    st->result.source_generation = source_generation;
    st->running = true;
    if (pthread_create(&st->thread, NULL, thread_main, st) != 0) {
        st->running = false;
        free(st->processing_bgrx);
        pthread_mutex_destroy(&st->lock);
        pthread_cond_destroy(&st->cond);
        pthread_mutex_destroy(&st->result_lock);
        pthread_mutex_destroy(&st->epoch_lock);
        memset(st, 0, sizeof(*st));
        return -1;
    }
    st->thread_started = true;
    return 0;
}

void lpr_infer_stop(struct infer_state *st)
{
    struct lpr_frame_ref pending = {0};

    if (!st || !st->thread_started)
        return;
    pthread_mutex_lock(&st->lock);
    st->running = false;
    if (st->has_new) {
        pending = st->pending_frame;
        memset(&st->pending_frame, 0, sizeof(st->pending_frame));
        st->has_new = false;
    }
    pthread_cond_broadcast(&st->cond);
    pthread_mutex_unlock(&st->lock);
    lpr_frame_ref_release(&pending);
    if (st->thread_started)
        pthread_join(st->thread, NULL);
    free(st->processing_bgrx);
    pthread_mutex_destroy(&st->lock);
    pthread_cond_destroy(&st->cond);
    pthread_mutex_destroy(&st->result_lock);
    pthread_mutex_destroy(&st->epoch_lock);
    memset(st, 0, sizeof(*st));
}
