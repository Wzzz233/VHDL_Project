// SPDX-License-Identifier: GPL-2.0
/* Frame-source interface and deterministic dual-source selection policy. */

#include "lpr_source.h"

#include <errno.h>
#include <string.h>

static bool source_id_valid(enum lpr_source_id id)
{
    return id == LPR_SOURCE_FPGA || id == LPR_SOURCE_PHONE;
}

static uint64_t next_source_generation(uint64_t generation)
{
    generation++;
    return generation ? generation : 1U;
}

int lpr_frame_source_open(struct lpr_frame_source *source)
{
    if (!source || !source_id_valid(source->id) || !source->ops)
        return -EINVAL;
    if (!source->ops->open)
        return -ENOSYS;
    return source->ops->open(source->ctx);
}

int lpr_frame_source_start(struct lpr_frame_source *source)
{
    if (!source || !source_id_valid(source->id) || !source->ops)
        return -EINVAL;
    if (!source->ops->start)
        return -ENOSYS;
    return source->ops->start(source->ctx);
}

int lpr_frame_source_read_latest(struct lpr_frame_source *source,
                                 struct lpr_frame_ref *out)
{
    if (!source || !source_id_valid(source->id) || !source->ops || !out)
        return -EINVAL;
    if (!source->ops->read_latest)
        return -ENOSYS;
    return source->ops->read_latest(source->ctx, out);
}

int lpr_frame_source_health(struct lpr_frame_source *source, int64_t now_us,
                            struct lpr_source_health *out)
{
    int rc;

    if (!source || !source_id_valid(source->id) || !source->ops || !out ||
        now_us < 0)
        return -EINVAL;
    memset(out, 0, sizeof(*out));
    if (!source->ops->health)
        return -ENOSYS;
    rc = source->ops->health(source->ctx, now_us, out);
    if (rc < 0)
        return rc;
    if (out->has_frame) {
        if (out->last_frame_us <= now_us)
            out->frame_age_us = now_us - out->last_frame_us;
        else
            out->frame_age_us = 0;
    } else {
        out->frame_age_us = -1;
    }
    return 0;
}

void lpr_frame_source_stop(struct lpr_frame_source *source)
{
    if (source && source->ops && source->ops->stop)
        source->ops->stop(source->ctx);
}

void lpr_frame_source_close(struct lpr_frame_source *source)
{
    if (source && source->ops && source->ops->close)
        source->ops->close(source->ctx);
}

bool lpr_source_phone_is_fresh(const struct lpr_source_health *health,
                               int64_t now_us)
{
    int64_t age;

    if (!health || now_us < 0 || !health->opened || !health->running ||
        !health->healthy || !health->has_frame || health->last_frame_us < 0)
        return false;
    age = health->last_frame_us <= now_us ?
          now_us - health->last_frame_us : 0;
    return age < LPR_PHONE_STALE_US;
}

static void fill_event(const struct lpr_source_manager *manager,
                       struct lpr_source_switch_event *event)
{
    if (!event)
        return;
    memset(event, 0, sizeof(*event));
    event->previous = manager->active;
    event->active = manager->active;
    event->source_generation = manager->source_generation;
    event->reason = manager->reason;
}

static int switch_active(struct lpr_source_manager *manager,
                         enum lpr_source_id active,
                         enum lpr_source_switch_reason reason,
                         int64_t now_us,
                         struct lpr_source_switch_event *event)
{
    enum lpr_source_id previous = manager->active;

    if (previous == active) {
        manager->reason = reason;
        fill_event(manager, event);
        return 0;
    }
    manager->active = active;
    manager->active_since_us = now_us;
    manager->source_generation =
        next_source_generation(manager->source_generation);
    manager->reason = reason;
    if (event) {
        event->switched = true;
        event->previous = previous;
        event->active = active;
        event->source_generation = manager->source_generation;
        event->reason = reason;
    }
    return 1;
}

static int validate_update(struct lpr_source_manager *manager, int64_t now_us,
                           struct lpr_source_switch_event *event)
{
    if (!manager || !source_id_valid(manager->desired) ||
        !source_id_valid(manager->active) || now_us < 0)
        return -EINVAL;
    if (now_us < manager->last_update_us)
        return -ERANGE;
    fill_event(manager, event);
    manager->last_update_us = now_us;
    return 0;
}

int lpr_source_manager_advance_generation(
    struct lpr_source_manager *manager, int64_t now_us)
{
    if (!manager || !source_id_valid(manager->desired) ||
        !source_id_valid(manager->active) || now_us < 0)
        return -EINVAL;
    if (now_us < manager->last_update_us)
        return -ERANGE;
    manager->source_generation =
        next_source_generation(manager->source_generation);
    manager->phone_recovery_since_us = -1;
    manager->active_since_us = now_us;
    manager->last_update_us = now_us;
    return 0;
}

int lpr_source_manager_restart(struct lpr_source_manager *manager,
                               int64_t now_us)
{
    enum lpr_source_switch_reason reason;
    enum lpr_source_id desired;
    enum lpr_source_id active;
    uint64_t generation;
    int rc;

    if (!manager || !source_id_valid(manager->desired) ||
        !source_id_valid(manager->active) || now_us < 0)
        return -EINVAL;
    if (now_us < manager->last_update_us)
        return -ERANGE;
    desired = manager->desired;
    active = manager->active;
    reason = manager->reason;
    generation = next_source_generation(manager->source_generation);
    rc = lpr_source_manager_init(manager, desired, active, generation, now_us);
    if (rc == 0)
        manager->reason = reason;
    return rc;
}

int lpr_source_manager_init(struct lpr_source_manager *manager,
                            enum lpr_source_id desired,
                            enum lpr_source_id active,
                            uint64_t initial_generation,
                            int64_t now_us)
{
    if (!manager || !source_id_valid(desired) || !source_id_valid(active) ||
        now_us < 0)
        return -EINVAL;
    memset(manager, 0, sizeof(*manager));
    manager->desired = desired;
    manager->active = active;
    manager->source_generation = initial_generation ? initial_generation : 1;
    manager->reason = LPR_SOURCE_REASON_STARTUP;
    manager->phone_failover_latched =
        desired == LPR_SOURCE_PHONE && active == LPR_SOURCE_FPGA;
    manager->phone_recovery_since_us = -1;
    manager->active_since_us = now_us;
    manager->last_update_us = now_us;
    return 0;
}

int lpr_source_manager_set_desired(
    struct lpr_source_manager *manager, enum lpr_source_id desired,
    const struct lpr_source_health *phone_health, int64_t now_us,
    struct lpr_source_switch_event *event)
{
    int rc;

    if (!source_id_valid(desired))
        return -EINVAL;
    if (manager && desired == manager->desired)
        return lpr_source_manager_update(manager, phone_health, now_us, event);
    rc = validate_update(manager, now_us, event);
    if (rc < 0)
        return rc;
    manager->desired = desired;
    manager->phone_recovery_since_us = -1;

    if (desired == LPR_SOURCE_FPGA) {
        manager->phone_failover_latched = false;
        return switch_active(manager, LPR_SOURCE_FPGA,
                             LPR_SOURCE_REASON_DESIRED_FPGA, now_us, event);
    }
    if (lpr_source_phone_is_fresh(phone_health, now_us)) {
        manager->phone_failover_latched = false;
        return switch_active(manager, LPR_SOURCE_PHONE,
                             LPR_SOURCE_REASON_DESIRED_PHONE, now_us, event);
    }

    manager->phone_failover_latched = true;
    if (manager->active == LPR_SOURCE_PHONE)
        return switch_active(manager, LPR_SOURCE_FPGA,
                             LPR_SOURCE_REASON_PHONE_UNAVAILABLE, now_us,
                             event);
    manager->reason = LPR_SOURCE_REASON_PHONE_UNAVAILABLE;
    fill_event(manager, event);
    return 0;
}

int lpr_source_manager_update(
    struct lpr_source_manager *manager,
    const struct lpr_source_health *phone_health, int64_t now_us,
    struct lpr_source_switch_event *event)
{
    bool fresh;
    int rc = validate_update(manager, now_us, event);

    if (rc < 0)
        return rc;
    if (manager->desired == LPR_SOURCE_FPGA) {
        manager->phone_failover_latched = false;
        manager->phone_recovery_since_us = -1;
        return switch_active(manager, LPR_SOURCE_FPGA,
                             LPR_SOURCE_REASON_DESIRED_FPGA, now_us, event);
    }

    fresh = lpr_source_phone_is_fresh(phone_health, now_us);
    if (manager->active == LPR_SOURCE_PHONE) {
        bool timed_out;

        manager->phone_recovery_since_us = -1;
        if (phone_health && phone_health->has_frame &&
            phone_health->last_frame_us >= 0) {
            int64_t age = phone_health->last_frame_us <= now_us ?
                          now_us - phone_health->last_frame_us : 0;
            timed_out = age >= LPR_PHONE_STALE_US;
        } else {
            timed_out = now_us - manager->active_since_us >=
                        LPR_PHONE_STALE_US;
        }
        if (!timed_out)
            return 0;
        manager->phone_failover_latched = true;
        return switch_active(
            manager, LPR_SOURCE_FPGA,
            phone_health && phone_health->has_frame ?
                LPR_SOURCE_REASON_PHONE_STALE :
                LPR_SOURCE_REASON_PHONE_UNAVAILABLE,
            now_us, event);
    }

    if (!fresh) {
        manager->phone_recovery_since_us = -1;
        return 0;
    }
    if (!manager->phone_failover_latched) {
        return switch_active(manager, LPR_SOURCE_PHONE,
                             LPR_SOURCE_REASON_DESIRED_PHONE, now_us, event);
    }
    if (manager->phone_recovery_since_us < 0) {
        manager->phone_recovery_since_us = now_us;
        return 0;
    }
    if (now_us - manager->phone_recovery_since_us < LPR_PHONE_RECOVERY_US)
        return 0;

    manager->phone_failover_latched = false;
    manager->phone_recovery_since_us = -1;
    return switch_active(manager, LPR_SOURCE_PHONE,
                         LPR_SOURCE_REASON_PHONE_RECOVERED, now_us, event);
}

bool lpr_source_result_is_current(const struct lpr_source_manager *manager,
                                  uint64_t result_source_generation)
{
    return manager && result_source_generation != 0 &&
           result_source_generation == manager->source_generation;
}

const char *lpr_source_id_string(enum lpr_source_id id)
{
    switch (id) {
    case LPR_SOURCE_FPGA: return "fpga";
    case LPR_SOURCE_PHONE: return "phone";
    default: return "unknown";
    }
}

const char *lpr_source_reason_string(enum lpr_source_switch_reason reason)
{
    switch (reason) {
    case LPR_SOURCE_REASON_STARTUP: return "startup";
    case LPR_SOURCE_REASON_DESIRED_FPGA: return "desired_fpga";
    case LPR_SOURCE_REASON_DESIRED_PHONE: return "desired_phone";
    case LPR_SOURCE_REASON_PHONE_UNAVAILABLE: return "phone_unavailable";
    case LPR_SOURCE_REASON_PHONE_STALE: return "phone_stale";
    case LPR_SOURCE_REASON_PHONE_RECOVERED: return "phone_recovered";
    default: return "unknown";
    }
}
