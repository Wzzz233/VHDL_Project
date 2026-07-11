// SPDX-License-Identifier: GPL-2.0
/* Frame-source interface and deterministic dual-source selection policy. */

#ifndef LPR_LIVE_LPR_SOURCE_H
#define LPR_LIVE_LPR_SOURCE_H

#include "lpr_frame.h"

#include <stdbool.h>
#include <stdint.h>

#define LPR_PHONE_STALE_US    2000000LL
#define LPR_PHONE_RECOVERY_US 3000000LL

enum lpr_source_id {
    LPR_SOURCE_FPGA = 0,
    LPR_SOURCE_PHONE = 1,
};

struct lpr_source_health {
    bool opened;
    bool running;
    bool healthy;
    bool has_frame;
    int64_t last_frame_us;
    int64_t frame_age_us;
    uint64_t sequence;
    double input_fps;
    uint64_t dropped_frames;
    uint64_t error_count;
    uint64_t reconnect_count;
};

struct lpr_frame_source_ops {
    int (*open)(void *ctx);
    int (*start)(void *ctx);
    int (*read_latest)(void *ctx, struct lpr_frame_ref *out);
    int (*health)(void *ctx, int64_t now_us,
                  struct lpr_source_health *out);
    void (*stop)(void *ctx);
    void (*close)(void *ctx);
};

struct lpr_frame_source {
    enum lpr_source_id id;
    const char *name;
    const struct lpr_frame_source_ops *ops;
    void *ctx;
};

int lpr_frame_source_open(struct lpr_frame_source *source);
int lpr_frame_source_start(struct lpr_frame_source *source);
int lpr_frame_source_read_latest(struct lpr_frame_source *source,
                                 struct lpr_frame_ref *out);
int lpr_frame_source_health(struct lpr_frame_source *source, int64_t now_us,
                            struct lpr_source_health *out);
void lpr_frame_source_stop(struct lpr_frame_source *source);
void lpr_frame_source_close(struct lpr_frame_source *source);

enum lpr_source_switch_reason {
    LPR_SOURCE_REASON_STARTUP = 0,
    LPR_SOURCE_REASON_DESIRED_FPGA,
    LPR_SOURCE_REASON_DESIRED_PHONE,
    LPR_SOURCE_REASON_PHONE_UNAVAILABLE,
    LPR_SOURCE_REASON_PHONE_STALE,
    LPR_SOURCE_REASON_PHONE_RECOVERED,
};

struct lpr_source_manager {
    enum lpr_source_id desired;
    enum lpr_source_id active;
    uint64_t source_generation;
    enum lpr_source_switch_reason reason;
    bool phone_failover_latched;
    int64_t phone_recovery_since_us;
    int64_t active_since_us;
    int64_t last_update_us;
};

struct lpr_source_switch_event {
    bool switched;
    enum lpr_source_id previous;
    enum lpr_source_id active;
    uint64_t source_generation;
    enum lpr_source_switch_reason reason;
};

int lpr_source_manager_init(struct lpr_source_manager *manager,
                            enum lpr_source_id desired,
                            enum lpr_source_id active,
                            uint64_t initial_generation,
                            int64_t now_us);
/* Advance the frame/result epoch without changing source-selection state. */
int lpr_source_manager_advance_generation(
    struct lpr_source_manager *manager, int64_t now_us);
/* Begin a restarted capture epoch while preserving the current status reason. */
int lpr_source_manager_restart(struct lpr_source_manager *manager,
                               int64_t now_us);
/* Manager transitions return 1 when active source switched, 0 when it did
 * not, and a negative errno value for invalid or non-monotonic input. */
int lpr_source_manager_set_desired(
    struct lpr_source_manager *manager, enum lpr_source_id desired,
    const struct lpr_source_health *phone_health, int64_t now_us,
    struct lpr_source_switch_event *event);
int lpr_source_manager_update(
    struct lpr_source_manager *manager,
    const struct lpr_source_health *phone_health, int64_t now_us,
    struct lpr_source_switch_event *event);

bool lpr_source_phone_is_fresh(const struct lpr_source_health *health,
                               int64_t now_us);
bool lpr_source_result_is_current(const struct lpr_source_manager *manager,
                                  uint64_t result_source_generation);
const char *lpr_source_id_string(enum lpr_source_id id);
const char *lpr_source_reason_string(enum lpr_source_switch_reason reason);

#endif /* LPR_LIVE_LPR_SOURCE_H */
