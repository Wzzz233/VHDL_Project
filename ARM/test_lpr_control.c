// SPDX-License-Identifier: GPL-2.0
/* Host checks for control JSON, JPEG snapshots and shutdown behavior. */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "lpr_live/lpr_control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define CHECK(expr) do {                                                     \
    if (!(expr)) {                                                           \
        fprintf(stderr, "[FAIL] line %d: %s\n", __LINE__, #expr);          \
        return 1;                                                            \
    }                                                                        \
} while (0)

static void copy_json(struct lpr_control *control, bool results,
                      char *out, size_t out_size)
{
    pthread_mutex_lock(&control->lock);
    snprintf(out, out_size, "%s",
             results ? control->results_json : control->status_json);
    pthread_mutex_unlock(&control->lock);
}

static int connect_control(const char *socket_path)
{
    struct sockaddr_un address;
    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);

    if (fd < 0)
        return -1;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    snprintf(address.sun_path, sizeof(address.sun_path), "%s", socket_path);
    if (connect(fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static int request_frame(const char *socket_path, char *header,
                         size_t header_size, uint8_t *body,
                         size_t body_size)
{
    static const char request[] = "{\"op\":\"frame\"}\n";
    size_t used = 0;
    size_t received = 0;
    int fd = connect_control(socket_path);

    if (fd < 0 || write(fd, request, sizeof(request) - 1U) !=
                      (ssize_t)(sizeof(request) - 1U))
        goto fail;
    while (used + 1U < header_size) {
        ssize_t count = read(fd, header + used, 1);

        if (count != 1)
            goto fail;
        if (header[used++] == '\n')
            break;
    }
    if (used == 0 || header[used - 1U] != '\n')
        goto fail;
    header[used] = '\0';
    while (received < body_size) {
        ssize_t count = read(fd, body + received, body_size - received);

        if (count <= 0)
            goto fail;
        received += (size_t)count;
    }
    close(fd);
    return 0;

fail:
    if (fd >= 0)
        close(fd);
    return -1;
}

int main(void)
{
    struct lpr_control control;
    struct lpr_runtime_status status;
    struct live_result result;
    struct timespec pause = { .tv_sec = 0, .tv_nsec = 1000000 };
    char socket_path[108];
    char json[16384];
    char frame_header[512];
    char oversized[5000];
    const uint8_t jpeg[] = { 0xff, 0xd8, 0x11, 0x22, 0xff, 0xd9 };
    uint8_t jpeg_body[sizeof(jpeg)];
    int client_fd;
    bool accepted = false;
    bool jpeg_metadata_ok;
    bool jpeg_cleared;

    snprintf(socket_path, sizeof(socket_path),
             "/tmp/lpr-control-test-%ld.sock", (long)getpid());
    CHECK(lpr_control_start(&control, socket_path) == 0);

    memset(&status, 0, sizeof(status));
    status.desired_source = "fpga\"selected";
    status.active_source = "phone";
    status.failover_reason = "line\\break\nreason";
    status.source_generation = 3;
    status.width = 1280;
    status.height = 720;
    status.frame_age_ms = NAN;
    status.input_fps = INFINITY;
    status.decode_fps = -INFINITY;
    status.infer_fps = 4.5;
    lpr_control_update_status(&control, &status);
    copy_json(&control, false, json, sizeof(json));
    CHECK(strstr(json, "fpga\\\"selected") != NULL);
    CHECK(strstr(json, "line\\\\break\\u000areason") != NULL);
    CHECK(strstr(json, ":nan") == NULL && strstr(json, ":inf") == NULL &&
          strstr(json, ":-inf") == NULL);

    memset(oversized, 'x', sizeof(oversized) - 1U);
    oversized[sizeof(oversized) - 1U] = '\0';
    status.desired_source = oversized;
    lpr_control_update_status(&control, &status);
    copy_json(&control, false, json, sizeof(json));
    CHECK(strcmp(json,
                 "{\"ok\":false,\"error\":\"serialization_failed\"}") == 0);

    memset(&result, 0, sizeof(result));
    result.valid = true;
    result.source_generation = 3;
    result.result_count = 1;
    result.infer_ms = INFINITY;
    result.plates[0].box.conf = NAN;
    result.plates[0].conf = INFINITY;
    snprintf(result.plates[0].route_name,
             sizeof(result.plates[0].route_name), "x\"\\");
    snprintf(result.plates[0].text, sizeof(result.plates[0].text),
             "line\nplate");
    lpr_control_update_results(&control, &result, 3);
    copy_json(&control, true, json, sizeof(json));
    CHECK(strstr(json, "\\u000a") != NULL);
    CHECK(strstr(json, ":nan") == NULL && strstr(json, ":inf") == NULL &&
          strstr(json, ":-inf") == NULL);
    CHECK(strstr(json, "\"valid\":true") != NULL);

    result.result_count = MAX_LIVE_PLATES;
    for (int i = 0; i < MAX_LIVE_PLATES; i++) {
        memset(result.plates[i].route_name, 'R',
               sizeof(result.plates[i].route_name));
        memset(result.plates[i].text, 'T', sizeof(result.plates[i].text));
    }
    lpr_control_update_results(&control, &result, 3);
    copy_json(&control, true, json, sizeof(json));
    CHECK(json[0] == '{' && json[strlen(json) - 1U] == '}');
    CHECK(strstr(json, "serialization_failed") == NULL);

    CHECK(lpr_control_update_jpeg(&control, jpeg, sizeof(jpeg), 9, 0) < 0);
    CHECK(lpr_control_update_jpeg(&control, jpeg, sizeof(jpeg), 9, 3) == 0);
    pthread_mutex_lock(&control.lock);
    jpeg_metadata_ok = control.jpeg_source_generation == 3 &&
                       control.jpeg_sequence == 9;
    pthread_mutex_unlock(&control.lock);
    CHECK(jpeg_metadata_ok);
    memset(frame_header, 0, sizeof(frame_header));
    memset(jpeg_body, 0, sizeof(jpeg_body));
    CHECK(request_frame(socket_path, frame_header, sizeof(frame_header),
                        jpeg_body, sizeof(jpeg_body)) == 0);
    CHECK(strstr(frame_header, "\"content_length\":6") != NULL);
    CHECK(strstr(frame_header, "\"sequence\":9") != NULL);
    CHECK(strstr(frame_header, "\"source_generation\":3") != NULL);
    CHECK(memcmp(jpeg_body, jpeg, sizeof(jpeg)) == 0);
    lpr_control_clear_jpeg(&control);
    pthread_mutex_lock(&control.lock);
    jpeg_cleared = control.jpeg == NULL && control.jpeg_size == 0 &&
                   control.jpeg_source_generation == 0;
    pthread_mutex_unlock(&control.lock);
    CHECK(jpeg_cleared);

    client_fd = connect_control(socket_path);
    CHECK(client_fd >= 0);
    for (int i = 0; i < 1000; i++) {
        pthread_mutex_lock(&control.lock);
        accepted = control.client_fd >= 0;
        pthread_mutex_unlock(&control.lock);
        if (accepted)
            break;
        nanosleep(&pause, NULL);
    }
    CHECK(accepted);

    /* The server thread is blocked reading this silent client. stop() must
     * shut that socket down before joining the thread. */
    lpr_control_stop(&control);
    close(client_fd);
    CHECK(access(socket_path, F_OK) < 0);
    puts("[PASS] control JSON escaping and finite numbers");
    puts("[PASS] control frame source generation and cache clear");
    puts("[PASS] control stop unblocks a silent client");
    return 0;
}
