// SPDX-License-Identifier: MIT
/* Portable implementation of the C+ RK3568 FP16 handoff contract. */

#include "cplus_core.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CPLUS_MAX_RAW_CANDIDATES 4096

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static int clampi(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float box_area(const struct cplus_box *box)
{
    return fmaxf(0.0f, box->x2 - box->x1) * fmaxf(0.0f, box->y2 - box->y1);
}

static float intersection_area(const struct cplus_box *a, const struct cplus_box *b)
{
    struct cplus_box i;
    i.x1 = fmaxf(a->x1, b->x1);
    i.y1 = fmaxf(a->y1, b->y1);
    i.x2 = fminf(a->x2, b->x2);
    i.y2 = fminf(a->y2, b->y2);
    return box_area(&i);
}

void cplus_default_runtime_config(struct cplus_runtime_config *config)
{
    if (!config) return;
    config->person_confidence = 0.25f;
    config->vehicle_confidence = 0.15f;
    config->class_aware_nms_iou = 0.70f;
    config->person_dedup_iou = 0.60f;
    config->rider_lower_overlap = 0.32f;
    config->rider_person_overlap = 0.16f;
    config->rider_min_vehicle_confidence = 0.15f;
    config->rider_min_person_height_ratio = 0.055f;
}

const char *cplus_target_type_name(enum cplus_target_type type)
{
    switch (type) {
    case CPLUS_TARGET_PEDESTRIAN: return "pedestrian";
    case CPLUS_TARGET_BICYCLE: return "bicycle";
    case CPLUS_TARGET_MOTORCYCLE: return "motorcycle";
    case CPLUS_TARGET_BICYCLE_RIDER: return "bicycle_rider";
    case CPLUS_TARGET_MOTORCYCLE_RIDER: return "scooter_or_motorcycle_rider";
    default: return "unknown";
    }
}

const char *cplus_decision_name(enum cplus_decision decision)
{
    return decision == CPLUS_DECISION_SUSPECTED ?
        "suspected_crossing_road_outside_zebra" : "not_suspected_crossing_road";
}

const char *cplus_reason_name(enum cplus_reason reason)
{
    switch (reason) {
    case CPLUS_REASON_TWO_WHEEL_FILTERED: return "two_wheel_filtered";
    case CPLUS_REASON_RIDER_FILTERED: return "rider_filtered";
    case CPLUS_REASON_ZEBRA_SUPPRESSED: return "zebra_suppressed";
    case CPLUS_REASON_SIDEWALK_SUPPRESSED: return "sidewalk_suppressed";
    case CPLUS_REASON_WEAK_GROUND_EVIDENCE: return "weak_ground_evidence";
    case CPLUS_REASON_ROAD_DOMINANT_WITHOUT_ZEBRA: return "road_dominant_without_zebra";
    case CPLUS_REASON_ROAD_DOMINANT_ZEBRA_NOISE_IGNORED: return "road_dominant_zebra_noise_ignored";
    case CPLUS_REASON_ROAD_NOT_DOMINANT: return "road_not_dominant";
    default: return "unknown";
    }
}

void cplus_resize_rgb_nearest(const uint8_t *src, int src_w, int src_h,
                               uint8_t *dst, int dst_w, int dst_h)
{
    int x, y;
    if (!src || !dst || src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0) return;
    for (y = 0; y < dst_h; ++y) {
        int sy = y * src_h / dst_h;
        for (x = 0; x < dst_w; ++x) {
            int sx = x * src_w / dst_w;
            memcpy(dst + ((size_t)y * dst_w + x) * 3U,
                   src + ((size_t)sy * src_w + sx) * 3U, 3U);
        }
    }
}

void cplus_prepare_detector_rgb(const uint8_t *src, int src_w, int src_h,
                                 uint8_t *dst, struct cplus_letterbox *meta)
{
    float scale;
    int resized_w, resized_h, offset_x, offset_y, y;
    uint8_t *resized;
    if (!meta || !dst) return;
    memset(meta, 0, sizeof(*meta));
    meta->src_w = src_w; meta->src_h = src_h;
    meta->dst_w = CPLUS_MODEL_WIDTH; meta->dst_h = CPLUS_MODEL_HEIGHT;
    if (!src || src_w <= 0 || src_h <= 0) return;
    memset(dst, 114, (size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT * 3U);
    scale = fminf((float)CPLUS_MODEL_WIDTH / src_w, (float)CPLUS_MODEL_HEIGHT / src_h);
    resized_w = clampi((int)lroundf(src_w * scale), 1, CPLUS_MODEL_WIDTH);
    resized_h = clampi((int)lroundf(src_h * scale), 1, CPLUS_MODEL_HEIGHT);
    offset_x = (CPLUS_MODEL_WIDTH - resized_w) / 2;
    offset_y = (CPLUS_MODEL_HEIGHT - resized_h) / 2;
    resized = malloc((size_t)resized_w * resized_h * 3U);
    if (!resized) return;
    cplus_resize_rgb_nearest(src, src_w, src_h, resized, resized_w, resized_h);
    for (y = 0; y < resized_h; ++y)
        memcpy(dst + ((size_t)(offset_y + y) * CPLUS_MODEL_WIDTH + offset_x) * 3U,
               resized + (size_t)y * resized_w * 3U, (size_t)resized_w * 3U);
    free(resized);
    meta->scale = scale; meta->pad_x = offset_x; meta->pad_y = offset_y; meta->valid = true;
}

void cplus_prepare_segmenter_rgb(const uint8_t *src, int src_w, int src_h, uint8_t *dst)
{
    cplus_resize_rgb_nearest(src, src_w, src_h, dst, CPLUS_MODEL_WIDTH, CPLUS_MODEL_HEIGHT);
}

void cplus_map_box_from_detector(struct cplus_box *box, const struct cplus_letterbox *meta)
{
    if (!box || !meta || !meta->valid || meta->scale <= 0.0f) return;
    box->x1 = clampf((box->x1 - meta->pad_x) / meta->scale, 0.0f, meta->src_w);
    box->y1 = clampf((box->y1 - meta->pad_y) / meta->scale, 0.0f, meta->src_h);
    box->x2 = clampf((box->x2 - meta->pad_x) / meta->scale, 0.0f, meta->src_w);
    box->y2 = clampf((box->y2 - meta->pad_y) / meta->scale, 0.0f, meta->src_h);
}

float cplus_box_iou(const struct cplus_box *a, const struct cplus_box *b)
{
    float intersection, union_area;
    if (!a || !b) return 0.0f;
    intersection = intersection_area(a, b);
    union_area = box_area(a) + box_area(b) - intersection;
    return union_area > 0.0f ? intersection / union_area : 0.0f;
}

static int compare_detection_desc(const void *left, const void *right)
{
    const struct cplus_detection *a = left, *b = right;
    return (a->box.score < b->box.score) - (a->box.score > b->box.score);
}

static bool retained_class(int class_id)
{
    return class_id == CPLUS_COCO_PERSON || class_id == CPLUS_COCO_BICYCLE ||
           class_id == CPLUS_COCO_MOTORCYCLE;
}

static enum cplus_target_type target_for_class(int class_id)
{
    if (class_id == CPLUS_COCO_BICYCLE) return CPLUS_TARGET_BICYCLE;
    if (class_id == CPLUS_COCO_MOTORCYCLE) return CPLUS_TARGET_MOTORCYCLE;
    return CPLUS_TARGET_PEDESTRIAN;
}

static int suppress_detections(struct cplus_detection *detections, int count,
                               float generic_iou, float person_iou)
{
    bool removed[CPLUS_MAX_RAW_CANDIDATES] = {false};
    int i, j, kept = 0;
    qsort(detections, (size_t)count, sizeof(*detections), compare_detection_desc);
    for (i = 0; i < count; ++i) {
        float threshold;
        if (removed[i]) continue;
        threshold = detections[i].box.class_id == CPLUS_COCO_PERSON ?
            fminf(generic_iou, person_iou) : generic_iou;
        for (j = i + 1; j < count; ++j) {
            if (!removed[j] && detections[j].box.class_id == detections[i].box.class_id &&
                cplus_box_iou(&detections[i].box, &detections[j].box) >= threshold)
                removed[j] = true;
        }
    }
    for (i = 0; i < count; ++i) if (!removed[i]) detections[kept++] = detections[i];
    return kept;
}

int cplus_decode_yolo(const float *output, size_t float_count,
                      const struct cplus_letterbox *meta,
                      const struct cplus_runtime_config *config,
                      struct cplus_detection *detections, int capacity)
{
    struct cplus_runtime_config defaults;
    struct cplus_detection candidates[CPLUS_MAX_RAW_CANDIDATES];
    int prediction, count = 0, output_count;
    if (!output || !meta || !detections || capacity <= 0 ||
        float_count < (size_t)CPLUS_YOLO_CHANNELS * CPLUS_YOLO_PREDICTIONS) return -1;
    if (!config) { cplus_default_runtime_config(&defaults); config = &defaults; }
    if (capacity > CPLUS_MAX_DETECTIONS) capacity = CPLUS_MAX_DETECTIONS;
    for (prediction = 0; prediction < CPLUS_YOLO_PREDICTIONS; ++prediction) {
        int channel, class_id = -1;
        float score = 0.0f, threshold;
        struct cplus_detection candidate;
        for (channel = 4; channel < CPLUS_YOLO_CHANNELS; ++channel) {
            int coco_id = channel - 4;
            float value;
            if (!retained_class(coco_id)) continue;
            value = output[(size_t)channel * CPLUS_YOLO_PREDICTIONS + prediction];
            if (value > score) { score = value; class_id = coco_id; }
        }
        threshold = class_id == CPLUS_COCO_PERSON ? config->person_confidence : config->vehicle_confidence;
        if (class_id < 0 || score < threshold || count == CPLUS_MAX_RAW_CANDIDATES) continue;
        memset(&candidate, 0, sizeof(candidate));
        candidate.box.x1 = output[prediction] - output[(size_t)2 * CPLUS_YOLO_PREDICTIONS + prediction] * 0.5f;
        candidate.box.y1 = output[CPLUS_YOLO_PREDICTIONS + prediction] -
                           output[(size_t)3 * CPLUS_YOLO_PREDICTIONS + prediction] * 0.5f;
        candidate.box.x2 = output[prediction] + output[(size_t)2 * CPLUS_YOLO_PREDICTIONS + prediction] * 0.5f;
        candidate.box.y2 = output[CPLUS_YOLO_PREDICTIONS + prediction] +
                           output[(size_t)3 * CPLUS_YOLO_PREDICTIONS + prediction] * 0.5f;
        candidate.box.score = score; candidate.box.class_id = class_id;
        candidate.target_type = target_for_class(class_id);
        cplus_map_box_from_detector(&candidate.box, meta);
        if (box_area(&candidate.box) > 0.0f) candidates[count++] = candidate;
    }
    output_count = suppress_detections(candidates, count, config->class_aware_nms_iou,
                                       config->person_dedup_iou);
    if (output_count > capacity) output_count = capacity;
    memcpy(detections, candidates, (size_t)output_count * sizeof(*detections));
    return output_count;
}

static struct cplus_box expand_box(const struct cplus_box *box, float x_pad, float y_pad,
                                   int width, int height)
{
    float box_w = box->x2 - box->x1, box_h = box->y2 - box->y1;
    struct cplus_box expanded = *box;
    expanded.x1 = clampf(box->x1 - x_pad * box_w, 0.0f, width);
    expanded.x2 = clampf(box->x2 + x_pad * box_w, 0.0f, width);
    expanded.y1 = clampf(box->y1 - y_pad * box_h, 0.0f, height);
    expanded.y2 = clampf(box->y2 + y_pad * box_h, 0.0f, height);
    return expanded;
}

struct rider_candidate { int person, vehicle; float score; enum cplus_target_type type; };

static int compare_rider_desc(const void *left, const void *right)
{
    const struct rider_candidate *a = left, *b = right;
    return (a->score < b->score) - (a->score > b->score);
}

void cplus_assign_riders(struct cplus_detection *detections, int count, int image_w, int image_h,
                         const struct cplus_runtime_config *config)
{
    struct cplus_runtime_config defaults;
    struct rider_candidate candidates[CPLUS_MAX_DETECTIONS * CPLUS_MAX_DETECTIONS];
    bool people_used[CPLUS_MAX_DETECTIONS] = {false}, vehicles_used[CPLUS_MAX_DETECTIONS] = {false};
    int candidate_count = 0, person_index, vehicle_index, index;
    if (!detections || count <= 0) return;
    if (!config) { cplus_default_runtime_config(&defaults); config = &defaults; }
    if (count > CPLUS_MAX_DETECTIONS) count = CPLUS_MAX_DETECTIONS;
    for (person_index = 0; person_index < count; ++person_index) {
        struct cplus_box person, lower;
        float person_height, person_width, lower_area, person_area;
        if (detections[person_index].box.class_id != CPLUS_COCO_PERSON) continue;
        person = detections[person_index].box;
        person_height = fmaxf(1.0f, person.y2 - person.y1);
        if (person_height / fmaxf(1, image_h) < config->rider_min_person_height_ratio) continue;
        person_width = fmaxf(1.0f, person.x2 - person.x1);
        lower = person; lower.y1 = person.y1 + 0.48f * person_height;
        lower_area = fmaxf(1.0f, box_area(&lower)); person_area = fmaxf(1.0f, box_area(&person));
        for (vehicle_index = 0; vehicle_index < count; ++vehicle_index) {
            struct cplus_box vehicle, expanded;
            float vehicle_width, vehicle_height, lower_overlap, person_overlap;
            float vehicle_center_x, vehicle_center_y;
            bool horizontal_close, lower_body;
            if (detections[vehicle_index].box.class_id != CPLUS_COCO_BICYCLE &&
                detections[vehicle_index].box.class_id != CPLUS_COCO_MOTORCYCLE) continue;
            vehicle = detections[vehicle_index].box;
            if (vehicle.score < config->rider_min_vehicle_confidence) continue;
            vehicle_width = fmaxf(1.0f, vehicle.x2 - vehicle.x1);
            vehicle_height = fmaxf(1.0f, vehicle.y2 - vehicle.y1);
            if (vehicle_height / fmaxf(1, image_h) < 0.025f && vehicle_width / fmaxf(1, image_w) < 0.035f) continue;
            expanded = expand_box(&vehicle, 0.10f, 0.12f, image_w, image_h);
            lower_overlap = intersection_area(&lower, &expanded) / lower_area;
            person_overlap = intersection_area(&person, &expanded) / person_area;
            vehicle_center_x = (vehicle.x1 + vehicle.x2) * 0.5f;
            vehicle_center_y = (vehicle.y1 + vehicle.y2) * 0.5f;
            horizontal_close = fabsf(vehicle_center_x - (person.x1 + person.x2) * 0.5f) <=
                               0.95f * fmaxf(person_width, vehicle_width);
            lower_body = vehicle_center_y >= person.y1 + 0.45f * person_height;
            if (lower_overlap < config->rider_lower_overlap || person_overlap < config->rider_person_overlap ||
                !horizontal_close || !lower_body) continue;
            candidates[candidate_count].person = person_index;
            candidates[candidate_count].vehicle = vehicle_index;
            candidates[candidate_count].score = lower_overlap + 0.05f * vehicle.score;
            candidates[candidate_count].type = vehicle.class_id == CPLUS_COCO_BICYCLE ?
                CPLUS_TARGET_BICYCLE_RIDER : CPLUS_TARGET_MOTORCYCLE_RIDER;
            ++candidate_count;
        }
    }
    qsort(candidates, (size_t)candidate_count, sizeof(candidates[0]), compare_rider_desc);
    for (index = 0; index < candidate_count; ++index) {
        struct rider_candidate *candidate = &candidates[index];
        if (!people_used[candidate->person] && !vehicles_used[candidate->vehicle]) {
            detections[candidate->person].target_type = candidate->type;
            people_used[candidate->person] = true; vehicles_used[candidate->vehicle] = true;
        }
    }
}

int cplus_mask_argmax(const float *logits, size_t float_count, uint8_t *mask)
{
    size_t pixel, plane = (size_t)CPLUS_MODEL_WIDTH * CPLUS_MODEL_HEIGHT;
    if (!logits || !mask || float_count < plane * 4U) return -1;
    for (pixel = 0; pixel < plane; ++pixel) {
        int class_id; float best = logits[pixel]; uint8_t best_class = 0;
        for (class_id = 1; class_id < 4; ++class_id) {
            float value = logits[(size_t)class_id * plane + pixel];
            if (value > best) { best = value; best_class = (uint8_t)class_id; }
        }
        mask[pixel] = best_class;
    }
    return 0;
}

/* Candidate C uses an elliptical close.  The sliding horizontal pass keeps
 * its cost linear in pixels times kernel height instead of kernel area. */
static void ellipse_dilate(const uint8_t *source, uint8_t *dest, int width, int height, int kernel)
{
    int radius = kernel / 2, dy, y;
    size_t total = (size_t)width * height;
    memset(dest, 0, total);
    for (dy = -radius; dy <= radius; ++dy) {
        float normalized = (float)dy / fmaxf(1.0f, (float)radius);
        int x_radius = (int)floorf((float)radius * sqrtf(fmaxf(0.0f, 1.0f - normalized * normalized)));
        for (y = 0; y < height; ++y) {
            int sy = y + dy, x, left = 0, right = -1, count = 0;
            const uint8_t *row;
            uint8_t *out;
            if (sy < 0 || sy >= height) continue;
            row = source + (size_t)sy * width;
            out = dest + (size_t)y * width;
            for (x = 0; x < width; ++x) {
                int wanted_right = x + x_radius;
                int wanted_left = x - x_radius;
                if (wanted_right >= width) wanted_right = width - 1;
                if (wanted_left < 0) wanted_left = 0;
                while (right < wanted_right) count += row[++right] != 0;
                while (left < wanted_left) count -= row[left++] != 0;
                if (count > 0) out[x] = 1;
            }
        }
    }
}

static void ellipse_close_class(const uint8_t *mask, int class_id, int width, int height, int kernel,
                                uint8_t *class_mask, uint8_t *tmp, uint8_t *closed)
{
    size_t total = (size_t)width * height, index;
    for (index = 0; index < total; ++index) class_mask[index] = mask[index] == class_id;
    ellipse_dilate(class_mask, tmp, width, height, kernel);
    for (index = 0; index < total; ++index) class_mask[index] = tmp[index] == 0;
    ellipse_dilate(class_mask, closed, width, height, kernel);
    for (index = 0; index < total; ++index) closed[index] = closed[index] == 0;
}

static int component_bfs(const uint8_t *candidate, uint8_t *visited, int start,
                         int width, int height, int *queue)
{
    int head = 0, tail = 1;
    queue[0] = start; visited[start] = 1;
    while (head < tail) {
        int current = queue[head++], x = current % width, y = current / width, dx, dy;
        for (dy = -1; dy <= 1; ++dy) for (dx = -1; dx <= 1; ++dx) {
            int nx, ny, next;
            if (dx == 0 && dy == 0) continue;
            nx = x + dx; ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            next = ny * width + nx;
            if (candidate[next] && !visited[next]) { visited[next] = 1; queue[tail++] = next; }
        }
    }
    return tail;
}

static void mark_component(const int *queue, int count, uint32_t *component_marks, uint32_t stamp)
{
    int index;
    for (index = 0; index < count; ++index) component_marks[queue[index]] = stamp;
}

static void border_counts(const uint8_t *mask, int width, int height, const int *queue, int count,
                          uint32_t *component_marks, uint32_t *border_marks, uint32_t stamp,
                          int radius, int counts[4])
{
    int index;
    memset(counts, 0, 4U * sizeof(counts[0]));
    mark_component(queue, count, component_marks, stamp);
    for (index = 0; index < count; ++index) {
        int center = queue[index], x = center % width, y = center / width, dx, dy;
        for (dy = -radius; dy <= radius; ++dy) for (dx = -radius; dx <= radius; ++dx) {
            int nx = x + dx, ny = y + dy, next;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            next = ny * width + nx;
            if (component_marks[next] == stamp || border_marks[next] == stamp) continue;
            border_marks[next] = stamp;
            if (mask[next] < 4) ++counts[mask[next]];
        }
    }
}

static int next_stamp(uint32_t *stamp, uint32_t *component_marks, uint32_t *border_marks, size_t total)
{
    ++*stamp;
    if (*stamp == 0) {
        memset(component_marks, 0, total * sizeof(*component_marks));
        memset(border_marks, 0, total * sizeof(*border_marks));
        *stamp = 1;
    }
    return 0;
}

static void guarded_fill_class(uint8_t *mask, int class_id, int width, int height, int kernel,
                               int max_area, uint8_t *candidate, uint8_t *closed, uint8_t *tmp,
                               uint8_t *visited, int *queue, uint32_t *component_marks,
                               uint32_t *border_marks, uint32_t *stamp, int *changed_components,
                               int *changed_pixels)
{
    size_t total = (size_t)width * height, index;
    int conflict_class = class_id == CPLUS_MASK_ROAD ? CPLUS_MASK_SIDEWALK : CPLUS_MASK_ROAD;
    ellipse_close_class(mask, class_id, width, height, kernel, candidate, tmp, closed);
    for (index = 0; index < total; ++index) candidate[index] = closed[index] && mask[index] == CPLUS_MASK_OTHER;
    memset(visited, 0, total);
    for (index = 0; index < total; ++index) {
        int area, counts[4], border_total, i;
        if (!candidate[index] || visited[index]) continue;
        area = component_bfs(candidate, visited, (int)index, width, height, queue);
        if (area > max_area) continue;
        next_stamp(stamp, component_marks, border_marks, total);
        border_counts(mask, width, height, queue, area, component_marks, border_marks, *stamp, 6, counts);
        border_total = counts[1] + counts[2] + counts[3];
        if (border_total == 0 || (float)counts[class_id] / border_total < 0.62f ||
            (float)counts[conflict_class] / border_total > 0.22f) continue;
        for (i = 0; i < area; ++i) mask[queue[i]] = (uint8_t)class_id;
        ++*changed_components; *changed_pixels += area;
    }
}

static int replacement_label(const uint8_t *mask, int width, int height, const int *queue, int count,
                             uint32_t *component_marks, uint32_t *border_marks, uint32_t stamp, int fallback)
{
    int counts[4], index, best = fallback, best_count = -1;
    border_counts(mask, width, height, queue, count, component_marks, border_marks, stamp, 4, counts);
    for (index = 0; index < 4; ++index) if (index != fallback && counts[index] > best_count) {
        best = index; best_count = counts[index];
    }
    return best_count > 0 ? best : fallback;
}

static void remove_small_raw_zebra(const uint8_t *raw, uint8_t *mask, int width, int height,
                                   uint8_t *candidate, uint8_t *visited, int *queue,
                                   uint32_t *component_marks, uint32_t *border_marks, uint32_t *stamp,
                                   int *changed_components, int *changed_pixels)
{
    size_t total = (size_t)width * height, index;
    memset(visited, 0, total);
    for (index = 0; index < total; ++index) candidate[index] = raw[index] == CPLUS_MASK_CROSSWALK_ZEBRA;
    for (index = 0; index < total; ++index) {
        int area, replacement, i, remaining = 0;
        if (!candidate[index] || visited[index]) continue;
        area = component_bfs(candidate, visited, (int)index, width, height, queue);
        if (area >= 450) continue;
        next_stamp(stamp, component_marks, border_marks, total);
        replacement = replacement_label(mask, width, height, queue, area, component_marks, border_marks,
                                        *stamp, CPLUS_MASK_CROSSWALK_ZEBRA);
        if (replacement == CPLUS_MASK_CROSSWALK_ZEBRA) replacement = CPLUS_MASK_OTHER;
        for (i = 0; i < area; ++i) if (mask[queue[i]] == CPLUS_MASK_CROSSWALK_ZEBRA) {
            mask[queue[i]] = (uint8_t)replacement; ++remaining;
        }
        if (remaining > 0) { ++*changed_components; *changed_pixels += remaining; }
    }
}

static void fill_zebra_holes(uint8_t *mask, int width, int height, uint8_t *candidate, uint8_t *closed,
                             uint8_t *tmp, uint8_t *visited, int *queue,
                             int *changed_components, int *changed_pixels)
{
    size_t total = (size_t)width * height, index;
    ellipse_close_class(mask, CPLUS_MASK_CROSSWALK_ZEBRA, width, height, 55, candidate, tmp, closed);
    for (index = 0; index < total; ++index) candidate[index] = closed[index] && mask[index] == CPLUS_MASK_OTHER;
    memset(visited, 0, total);
    for (index = 0; index < total; ++index) {
        int area, i;
        if (!candidate[index] || visited[index]) continue;
        area = component_bfs(candidate, visited, (int)index, width, height, queue);
        if (area > 5734) continue;
        for (i = 0; i < area; ++i) mask[queue[i]] = CPLUS_MASK_CROSSWALK_ZEBRA;
        ++*changed_components; *changed_pixels += area;
    }
}

static void absorb_zebra_islands(uint8_t *mask, int width, int height, uint8_t *candidate,
                                 uint8_t *visited, int *queue, uint32_t *component_marks,
                                 uint32_t *border_marks, uint32_t *stamp,
                                 int *changed_components, int *changed_pixels)
{
    size_t total = (size_t)width * height, index;
    for (index = 0; index < total; ++index) candidate[index] = mask[index] != CPLUS_MASK_CROSSWALK_ZEBRA;
    memset(visited, 0, total);
    for (index = 0; index < total; ++index) {
        int area, counts[4], total_border, i;
        if (!candidate[index] || visited[index]) continue;
        area = component_bfs(candidate, visited, (int)index, width, height, queue);
        if (area > 4915) continue;
        next_stamp(stamp, component_marks, border_marks, total);
        border_counts(mask, width, height, queue, area, component_marks, border_marks, *stamp, 6, counts);
        total_border = counts[1] + counts[2] + counts[3];
        if (total_border == 0 || (float)counts[CPLUS_MASK_CROSSWALK_ZEBRA] / total_border < 0.55f) continue;
        for (i = 0; i < area; ++i) mask[queue[i]] = CPLUS_MASK_CROSSWALK_ZEBRA;
        ++*changed_components; *changed_pixels += area;
    }
}

int cplus_postprocess_mask_candidate_c(uint8_t *mask, int width, int height, struct cplus_mask_stats *stats)
{
    size_t total;
    uint8_t *raw, *candidate, *closed, *tmp, *visited;
    int *queue;
    uint32_t *component_marks, *border_marks, stamp = 0;
    if (!mask || width <= 0 || height <= 0 || !stats) return -1;
    total = (size_t)width * height;
    raw = malloc(total); candidate = malloc(total); closed = malloc(total); tmp = malloc(total); visited = malloc(total);
    queue = malloc(total * sizeof(*queue));
    component_marks = calloc(total, sizeof(*component_marks)); border_marks = calloc(total, sizeof(*border_marks));
    if (!raw || !candidate || !closed || !tmp || !visited || !queue || !component_marks || !border_marks) {
        free(raw); free(candidate); free(closed); free(tmp); free(visited); free(queue); free(component_marks); free(border_marks);
        return -1;
    }
    memcpy(raw, mask, total); memset(stats, 0, sizeof(*stats));
    guarded_fill_class(mask, CPLUS_MASK_ROAD, width, height, 91, 32768, candidate, closed, tmp, visited, queue,
                       component_marks, border_marks, &stamp, &stats->road_hole_components_filled,
                       &stats->road_hole_pixels_filled);
    guarded_fill_class(mask, CPLUS_MASK_SIDEWALK, width, height, 91, 32768, candidate, closed, tmp, visited, queue,
                       component_marks, border_marks, &stamp, &stats->sidewalk_hole_components_filled,
                       &stats->sidewalk_hole_pixels_filled);
    remove_small_raw_zebra(raw, mask, width, height, candidate, visited, queue, component_marks, border_marks, &stamp,
                           &stats->small_zebra_components_removed, &stats->small_zebra_pixels_removed);
    fill_zebra_holes(mask, width, height, candidate, closed, tmp, visited, queue,
                     &stats->zebra_hole_components_filled, &stats->zebra_hole_pixels_filled);
    absorb_zebra_islands(mask, width, height, candidate, visited, queue, component_marks, border_marks, &stamp,
                         &stats->zebra_inner_island_components_absorbed,
                         &stats->zebra_inner_island_pixels_absorbed);
    for (size_t index = 0; index < total; ++index) if (mask[index] != raw[index]) ++stats->changed_pixels_total;
    free(raw); free(candidate); free(closed); free(tmp); free(visited); free(queue); free(component_marks); free(border_marks);
    return 0;
}

void cplus_make_foot_box(const struct cplus_box *box, int image_w, int image_h, struct cplus_rect *foot)
{
    float width, height, center_x, foot_width;
    if (!box || !foot) return;
    width = box->x2 - box->x1; height = box->y2 - box->y1; center_x = (box->x1 + box->x2) * 0.5f;
    foot_width = width * 1.45f;
    foot->x1 = clampi((int)lroundf(center_x - foot_width * 0.5f), 0, image_w);
    foot->x2 = clampi((int)lroundf(center_x + foot_width * 0.5f), 0, image_w);
    foot->y1 = clampi((int)lroundf(box->y2 - 0.22f * height), 0, image_h);
    foot->y2 = clampi((int)lroundf(box->y2 + 0.12f * height), 0, image_h);
    if (foot->x2 <= foot->x1) foot->x2 = clampi(foot->x1 + 1, 0, image_w);
    if (foot->y2 <= foot->y1) foot->y2 = clampi(foot->y1 + 1, 0, image_h);
}

void cplus_scale_rect(const struct cplus_rect *source, int source_w, int source_h,
                      int dest_w, int dest_h, struct cplus_rect *dest)
{
    float sx, sy;
    if (!source || !dest || source_w <= 0 || source_h <= 0 || dest_w <= 0 || dest_h <= 0) return;
    sx = (float)dest_w / source_w; sy = (float)dest_h / source_h;
    dest->x1 = clampi((int)lroundf(source->x1 * sx), 0, dest_w);
    dest->x2 = clampi((int)lroundf(source->x2 * sx), 0, dest_w);
    dest->y1 = clampi((int)lroundf(source->y1 * sy), 0, dest_h);
    dest->y2 = clampi((int)lroundf(source->y2 * sy), 0, dest_h);
    if (dest->x2 <= dest->x1) dest->x2 = clampi(dest->x1 + 1, 0, dest_w);
    if (dest->y2 <= dest->y1) dest->y2 = clampi(dest->y1 + 1, 0, dest_h);
}

static int max_zebra_component(const uint8_t *mask, int width, const struct cplus_rect *foot)
{
    int local_width = foot->x2 - foot->x1, local_height = foot->y2 - foot->y1, x, y, best = 0;
    size_t total = (size_t)local_width * local_height;
    uint8_t *visited = calloc(total, 1U); int *queue = malloc(total * sizeof(*queue));
    if (!visited || !queue) { free(visited); free(queue); return 0; }
    for (y = 0; y < local_height; ++y) for (x = 0; x < local_width; ++x) {
        int start = y * local_width + x, head, tail, area;
        if (visited[start] || mask[(size_t)(foot->y1 + y) * width + foot->x1 + x] != CPLUS_MASK_CROSSWALK_ZEBRA) continue;
        visited[start] = 1; queue[0] = start; head = 0; tail = 1; area = 0;
        while (head < tail) {
            int current = queue[head++], cx = current % local_width, cy = current / local_width, dx, dy;
            ++area;
            for (dy = -1; dy <= 1; ++dy) for (dx = -1; dx <= 1; ++dx) {
                int nx = cx + dx, ny = cy + dy, next;
                if ((dx == 0 && dy == 0) || nx < 0 || nx >= local_width || ny < 0 || ny >= local_height) continue;
                next = ny * local_width + nx;
                if (!visited[next] && mask[(size_t)(foot->y1 + ny) * width + foot->x1 + nx] == CPLUS_MASK_CROSSWALK_ZEBRA) {
                    visited[next] = 1; queue[tail++] = next;
                }
            }
        }
        if (area > best) best = area;
    }
    free(visited); free(queue); return best;
}

void cplus_compute_ground_ratios(const uint8_t *mask, int mask_w, int mask_h,
                                 const struct cplus_rect *foot, struct cplus_ground_ratios *ratios)
{
    int x, y, road = 0, sidewalk = 0, zebra = 0, total;
    if (!ratios) return;
    memset(ratios, 0, sizeof(*ratios));
    if (!mask || !foot || foot->x1 < 0 || foot->y1 < 0 || foot->x2 > mask_w || foot->y2 > mask_h ||
        foot->x2 <= foot->x1 || foot->y2 <= foot->y1) return;
    total = (foot->x2 - foot->x1) * (foot->y2 - foot->y1);
    for (y = foot->y1; y < foot->y2; ++y) for (x = foot->x1; x < foot->x2; ++x) {
        uint8_t value = mask[(size_t)y * mask_w + x];
        if (value == CPLUS_MASK_ROAD) ++road;
        else if (value == CPLUS_MASK_SIDEWALK) ++sidewalk;
        else if (value == CPLUS_MASK_CROSSWALK_ZEBRA) ++zebra;
    }
    ratios->total_pixels = total; ratios->road = (float)road / total; ratios->sidewalk = (float)sidewalk / total;
    ratios->zebra = (float)zebra / total; ratios->known_ground = (float)(road + sidewalk + zebra) / total;
    ratios->zebra_pixels = zebra; ratios->max_zebra_component_pixels = max_zebra_component(mask, mask_w, foot);
}

static bool strong_zebra(const struct cplus_ground_ratios *ratios)
{
    int min_zebra = ratios->total_pixels * 25 / 1000, min_component = ratios->total_pixels * 15 / 1000;
    if (ratios->zebra < 0.06f) return false;
    if (min_zebra < 80) min_zebra = 80;
    if (min_component < 40) min_component = 40;
    return ratios->zebra_pixels >= min_zebra && ratios->max_zebra_component_pixels >= min_component;
}

void cplus_apply_rule(const struct cplus_ground_ratios *ratios,
                      enum cplus_decision *decision, enum cplus_reason *reason)
{
    if (!ratios || !decision || !reason) return;
    if (ratios->zebra >= 0.06f && strong_zebra(ratios)) {
        *decision = CPLUS_DECISION_NOT_SUSPECTED; *reason = CPLUS_REASON_ZEBRA_SUPPRESSED;
    } else if (ratios->sidewalk >= 0.25f) {
        *decision = CPLUS_DECISION_NOT_SUSPECTED; *reason = CPLUS_REASON_SIDEWALK_SUPPRESSED;
    } else if (ratios->known_ground < 0.35f) {
        *decision = CPLUS_DECISION_NOT_SUSPECTED; *reason = CPLUS_REASON_WEAK_GROUND_EVIDENCE;
    } else if (ratios->road >= 0.45f) {
        *decision = CPLUS_DECISION_SUSPECTED;
        *reason = ratios->zebra >= 0.06f ? CPLUS_REASON_ROAD_DOMINANT_ZEBRA_NOISE_IGNORED :
                  CPLUS_REASON_ROAD_DOMINANT_WITHOUT_ZEBRA;
    } else {
        *decision = CPLUS_DECISION_NOT_SUSPECTED; *reason = CPLUS_REASON_ROAD_NOT_DOMINANT;
    }
}

int cplus_evaluate_detections(const struct cplus_detection *detections, int count,
                              const uint8_t *mask, int mask_w, int mask_h, int image_w, int image_h,
                              struct cplus_person_result *results, int capacity)
{
    int index, result_count = 0;
    if (!detections || !results || capacity <= 0) return -1;
    for (index = 0; index < count && result_count < capacity; ++index) {
        struct cplus_person_result *result = &results[result_count++];
        memset(result, 0, sizeof(*result)); result->detection = detections[index];
        if (detections[index].box.class_id != CPLUS_COCO_PERSON) {
            result->decision = CPLUS_DECISION_NOT_SUSPECTED; result->reason = CPLUS_REASON_TWO_WHEEL_FILTERED; continue;
        }
        cplus_make_foot_box(&detections[index].box, image_w, image_h, &result->foot_box_source);
        cplus_scale_rect(&result->foot_box_source, image_w, image_h, mask_w, mask_h, &result->foot_box_mask);
        cplus_compute_ground_ratios(mask, mask_w, mask_h, &result->foot_box_mask, &result->ground);
        if (detections[index].target_type != CPLUS_TARGET_PEDESTRIAN) {
            result->decision = CPLUS_DECISION_NOT_SUSPECTED; result->reason = CPLUS_REASON_RIDER_FILTERED;
        } else cplus_apply_rule(&result->ground, &result->decision, &result->reason);
    }
    return result_count;
}
