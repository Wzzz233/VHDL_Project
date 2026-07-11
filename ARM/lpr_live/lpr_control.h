// SPDX-License-Identifier: GPL-2.0
/* Unix-domain control and snapshot service for the LAN web process. */

#ifndef LPR_LIVE_LPR_CONTROL_H
#define LPR_LIVE_LPR_CONTROL_H

#include "lpr_common.h"

#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum lpr_control_source {
    LPR_CONTROL_SOURCE_NONE = -1,
    LPR_CONTROL_SOURCE_FPGA = 0,
    LPR_CONTROL_SOURCE_PHONE = 1,
};

enum lpr_pipeline_command {
    LPR_PIPELINE_COMMAND_NONE = 0,
    LPR_PIPELINE_COMMAND_PAUSE,
    LPR_PIPELINE_COMMAND_RESUME,
    LPR_PIPELINE_COMMAND_RESTART,
};

struct lpr_runtime_status {
    const char *desired_source;
    const char *active_source;
    const char *failover_reason;
    uint64_t source_generation;
    bool paused;
    bool fpga_healthy;
    bool phone_healthy;
    uint32_t width;
    uint32_t height;
    double input_fps;
    double decode_fps;
    double infer_fps;
    double frame_age_ms;
    uint64_t input_frames;
    uint64_t decoded_frames;
    uint64_t inferred_frames;
    uint64_t input_dropped;
    uint64_t decode_dropped;
    uint64_t infer_dropped;
    uint64_t display_dropped;
};

struct lpr_control {
    int listen_fd;
    int client_fd;
    pthread_t thread;
    bool thread_started;
    bool running;
    pthread_mutex_t lock;
    bool lock_initialized;
    char socket_path[108];
    char status_json[4096];
    char results_json[16384];
    uint8_t *jpeg;
    size_t jpeg_size;
    uint64_t jpeg_sequence;
    uint64_t jpeg_source_generation;
    enum lpr_control_source pending_source;
    enum lpr_pipeline_command pending_pipeline_command;
};

int lpr_control_start(struct lpr_control *control, const char *socket_path);
void lpr_control_stop(struct lpr_control *control);
void lpr_control_update_status(struct lpr_control *control,
                               const struct lpr_runtime_status *status);
void lpr_control_update_results(struct lpr_control *control,
                                const struct live_result *result,
                                uint64_t accepted_source_generation);
int lpr_control_update_jpeg(struct lpr_control *control,
                            const uint8_t *jpeg, size_t jpeg_size,
                            uint64_t jpeg_sequence,
                            uint64_t source_generation);
void lpr_control_clear_jpeg(struct lpr_control *control);
enum lpr_control_source lpr_control_take_source(struct lpr_control *control);
enum lpr_pipeline_command
lpr_control_take_pipeline_command(struct lpr_control *control);

#endif /* LPR_LIVE_LPR_CONTROL_H */
