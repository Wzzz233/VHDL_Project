// SPDX-License-Identifier: GPL-2.0
/* Unix-domain control and snapshot service for the LAN web process. */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "lpr_control.h"

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

static int write_all(int fd, const void *data, size_t size)
{
    const uint8_t *bytes = (const uint8_t *)data;
    size_t written = 0;

    while (written < size) {
        ssize_t ret = send(fd, bytes + written, size - written,
                           MSG_NOSIGNAL);

        if (ret < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (ret == 0)
            return -1;
        written += (size_t)ret;
    }
    return 0;
}

static int json_get_string(const char *json, const char *key,
                           char *out, size_t out_size)
{
    char needle[64];
    const char *cursor;
    size_t length = 0;

    if (!json || !key || !out || out_size == 0)
        return -1;
    if (snprintf(needle, sizeof(needle), "\"%s\"", key) >=
        (int)sizeof(needle))
        return -1;
    cursor = strstr(json, needle);
    if (!cursor)
        return -1;
    cursor += strlen(needle);
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' ||
           *cursor == '\n')
        cursor++;
    if (*cursor++ != ':')
        return -1;
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' ||
           *cursor == '\n')
        cursor++;
    if (*cursor++ != '"')
        return -1;
    while (*cursor && *cursor != '"') {
        if (*cursor == '\\')
            return -1;
        if (length + 1 >= out_size)
            return -1;
        out[length++] = *cursor++;
    }
    if (*cursor != '"')
        return -1;
    out[length] = '\0';
    return 0;
}

static int appendf(char *buffer, size_t capacity, size_t *used,
                   const char *format, ...)
{
    va_list args;
    int count;

    if (*used >= capacity)
        return -1;
    va_start(args, format);
    count = vsnprintf(buffer + *used, capacity - *used, format, args);
    va_end(args);
    if (count < 0 || (size_t)count >= capacity - *used)
        return -1;
    *used += (size_t)count;
    return 0;
}

static int append_json_string_n(char *buffer, size_t capacity, size_t *used,
                                const char *value, size_t max_length)
{
    const unsigned char *cursor = (const unsigned char *)(value ? value : "");
    size_t length = 0;

    if (appendf(buffer, capacity, used, "\"") < 0)
        return -1;
    while (*cursor && length < max_length) {
        if (*cursor == '"' || *cursor == '\\') {
            if (appendf(buffer, capacity, used, "\\%c", *cursor) < 0)
                return -1;
        } else if (*cursor < 0x20) {
            if (appendf(buffer, capacity, used, "\\u%04x", *cursor) < 0)
                return -1;
        } else {
            if (*used + 1 >= capacity)
                return -1;
            buffer[(*used)++] = (char)*cursor;
            buffer[*used] = '\0';
        }
        cursor++;
        length++;
    }
    return appendf(buffer, capacity, used, "\"");
}

static int append_json_string(char *buffer, size_t capacity, size_t *used,
                              const char *value)
{
    return append_json_string_n(buffer, capacity, used, value, SIZE_MAX);
}

static double json_finite_or_zero(double value)
{
    return isfinite(value) ? value : 0.0;
}

static int ensure_parent_directory(const char *socket_path)
{
    char directory[108];
    char *slash;
    size_t length;

    length = strnlen(socket_path, sizeof(directory));
    if (length == 0 || length >= sizeof(directory))
        return -1;
    memcpy(directory, socket_path, length + 1);
    slash = strrchr(directory, '/');
    if (!slash || slash == directory)
        return 0;
    *slash = '\0';
    if (mkdir(directory, 0755) < 0 && errno != EEXIST)
        return -1;
    return 0;
}

static int send_json_line(int fd, const char *json)
{
    if (write_all(fd, json, strlen(json)) < 0)
        return -1;
    return write_all(fd, "\n", 1);
}

static void handle_client(struct lpr_control *control, int client_fd)
{
    char request[1024];
    char op[32];
    size_t used = 0;

    while (used + 1 < sizeof(request)) {
        ssize_t count = read(client_fd, request + used,
                             sizeof(request) - used - 1);

        if (count < 0) {
            if (errno == EINTR)
                continue;
            return;
        }
        if (count == 0)
            break;
        used += (size_t)count;
        if (memchr(request, '\n', used))
            break;
    }
    request[used] = '\0';
    if (json_get_string(request, "op", op, sizeof(op)) < 0) {
        send_json_line(client_fd,
                       "{\"ok\":false,\"error\":\"invalid_request\"}");
        return;
    }

    if (strcmp(op, "status") == 0 || strcmp(op, "results") == 0) {
        char response[16384];
        const char *source;

        pthread_mutex_lock(&control->lock);
        source = strcmp(op, "status") == 0 ? control->status_json :
                                             control->results_json;
        snprintf(response, sizeof(response), "%s", source);
        pthread_mutex_unlock(&control->lock);
        send_json_line(client_fd, response);
        return;
    }

    if (strcmp(op, "frame") == 0) {
        uint8_t *jpeg = NULL;
        size_t jpeg_size = 0;
        uint64_t jpeg_sequence = 0;
        uint64_t jpeg_source_generation = 0;
        char header[256];

        pthread_mutex_lock(&control->lock);
        if (control->jpeg && control->jpeg_size > 0 &&
            control->jpeg_source_generation != 0) {
            jpeg = malloc(control->jpeg_size);
            if (jpeg) {
                memcpy(jpeg, control->jpeg, control->jpeg_size);
                jpeg_size = control->jpeg_size;
                jpeg_sequence = control->jpeg_sequence;
                jpeg_source_generation =
                    control->jpeg_source_generation;
            }
        }
        pthread_mutex_unlock(&control->lock);
        if (!jpeg) {
            send_json_line(client_fd,
                           "{\"ok\":false,\"error\":\"frame_unavailable\"}");
            return;
        }
        snprintf(header, sizeof(header),
                 "{\"ok\":true,\"content_type\":\"image/jpeg\","
                 "\"content_length\":%zu,\"sequence\":%llu,"
                 "\"source_generation\":%llu}\n",
                 jpeg_size, (unsigned long long)jpeg_sequence,
                 (unsigned long long)jpeg_source_generation);
        if (write_all(client_fd, header, strlen(header)) == 0)
            write_all(client_fd, jpeg, jpeg_size);
        free(jpeg);
        return;
    }

    if (strcmp(op, "source") == 0) {
        char source[16];
        enum lpr_control_source requested;

        if (json_get_string(request, "source", source, sizeof(source)) < 0) {
            send_json_line(client_fd,
                           "{\"ok\":false,\"error\":\"source_required\"}");
            return;
        }
        if (strcmp(source, "fpga") == 0)
            requested = LPR_CONTROL_SOURCE_FPGA;
        else if (strcmp(source, "phone") == 0)
            requested = LPR_CONTROL_SOURCE_PHONE;
        else {
            send_json_line(client_fd,
                           "{\"ok\":false,\"error\":\"invalid_source\"}");
            return;
        }
        pthread_mutex_lock(&control->lock);
        control->pending_source = requested;
        pthread_mutex_unlock(&control->lock);
        send_json_line(client_fd, "{\"ok\":true}");
        return;
    }

    if (strcmp(op, "pipeline") == 0) {
        char action[16];
        enum lpr_pipeline_command command;

        if (json_get_string(request, "action", action, sizeof(action)) < 0) {
            send_json_line(client_fd,
                           "{\"ok\":false,\"error\":\"action_required\"}");
            return;
        }
        if (strcmp(action, "pause") == 0)
            command = LPR_PIPELINE_COMMAND_PAUSE;
        else if (strcmp(action, "resume") == 0)
            command = LPR_PIPELINE_COMMAND_RESUME;
        else if (strcmp(action, "restart") == 0)
            command = LPR_PIPELINE_COMMAND_RESTART;
        else {
            send_json_line(client_fd,
                           "{\"ok\":false,\"error\":\"invalid_action\"}");
            return;
        }
        pthread_mutex_lock(&control->lock);
        control->pending_pipeline_command = command;
        pthread_mutex_unlock(&control->lock);
        send_json_line(client_fd, "{\"ok\":true}");
        return;
    }

    send_json_line(client_fd,
                   "{\"ok\":false,\"error\":\"unknown_operation\"}");
}

static void *control_thread_main(void *argument)
{
    struct lpr_control *control = (struct lpr_control *)argument;

    for (;;) {
        int client_fd = accept4(control->listen_fd, NULL, NULL, SOCK_CLOEXEC);

        if (client_fd < 0) {
            bool running;

            if (errno == EINTR)
                continue;
            pthread_mutex_lock(&control->lock);
            running = control->running;
            pthread_mutex_unlock(&control->lock);
            if (!running)
                break;
            continue;
        }
        pthread_mutex_lock(&control->lock);
        if (!control->running) {
            pthread_mutex_unlock(&control->lock);
            close(client_fd);
            break;
        }
        control->client_fd = client_fd;
        pthread_mutex_unlock(&control->lock);
        handle_client(control, client_fd);
        pthread_mutex_lock(&control->lock);
        if (control->client_fd == client_fd)
            control->client_fd = -1;
        pthread_mutex_unlock(&control->lock);
        close(client_fd);
    }
    return NULL;
}

int lpr_control_start(struct lpr_control *control, const char *socket_path)
{
    struct sockaddr_un address;

    if (!control || !socket_path ||
        strnlen(socket_path, sizeof(control->socket_path)) >=
            sizeof(control->socket_path))
        return -1;
    memset(control, 0, sizeof(*control));
    control->listen_fd = -1;
    control->client_fd = -1;
    control->pending_source = LPR_CONTROL_SOURCE_NONE;
    snprintf(control->status_json, sizeof(control->status_json),
             "{\"ok\":true,\"pipeline\":\"starting\"}");
    snprintf(control->results_json, sizeof(control->results_json),
             "{\"valid\":false,\"detections\":[]}");
    snprintf(control->socket_path, sizeof(control->socket_path), "%s",
             socket_path);
    pthread_mutex_init(&control->lock, NULL);
    control->lock_initialized = true;
    if (ensure_parent_directory(socket_path) < 0)
        goto fail;

    control->listen_fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (control->listen_fd < 0)
        goto fail;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    snprintf(address.sun_path, sizeof(address.sun_path), "%s", socket_path);
    unlink(socket_path);
    if (bind(control->listen_fd, (struct sockaddr *)&address,
             sizeof(address)) < 0)
        goto fail;
    if (chmod(socket_path, 0660) < 0)
        goto fail;
    if (listen(control->listen_fd, 8) < 0)
        goto fail;

    control->running = true;
    if (pthread_create(&control->thread, NULL, control_thread_main,
                       control) != 0)
        goto fail;
    control->thread_started = true;
    fprintf(stderr, "[control] listening on %s\n", socket_path);
    return 0;

fail:
    fprintf(stderr, "[control] failed to listen on %s: %s\n",
            socket_path, strerror(errno));
    lpr_control_stop(control);
    return -1;
}

void lpr_control_stop(struct lpr_control *control)
{
    int listen_fd = -1;

    if (!control)
        return;
    if (control->lock_initialized) {
        pthread_mutex_lock(&control->lock);
        control->running = false;
        listen_fd = control->listen_fd;
        if (control->client_fd >= 0)
            shutdown(control->client_fd, SHUT_RDWR);
        pthread_mutex_unlock(&control->lock);
    } else {
        listen_fd = control->listen_fd;
    }
    if (listen_fd >= 0) {
        shutdown(listen_fd, SHUT_RDWR);
        close(listen_fd);
    }
    if (control->thread_started)
        pthread_join(control->thread, NULL);
    control->listen_fd = -1;
    if (control->socket_path[0])
        unlink(control->socket_path);
    free(control->jpeg);
    control->jpeg = NULL;
    if (control->lock_initialized)
        pthread_mutex_destroy(&control->lock);
    memset(control, 0, sizeof(*control));
    control->listen_fd = -1;
    control->client_fd = -1;
}

void lpr_control_update_status(struct lpr_control *control,
                               const struct lpr_runtime_status *status)
{
    char json[4096];
    size_t used = 0;

    if (!control || !status || !control->lock_initialized)
        return;
    json[0] = '\0';
    if (appendf(json, sizeof(json), &used,
                "{\"ok\":true,\"desired_source\":") < 0 ||
        append_json_string(json, sizeof(json), &used,
                           status->desired_source ?
                               status->desired_source : "fpga") < 0 ||
        appendf(json, sizeof(json), &used, ",\"active_source\":") < 0 ||
        append_json_string(json, sizeof(json), &used,
                           status->active_source ?
                               status->active_source : "fpga") < 0 ||
        appendf(json, sizeof(json), &used, ",\"failover_reason\":") < 0 ||
        append_json_string(json, sizeof(json), &used,
                           status->failover_reason ?
                               status->failover_reason : "none") < 0 ||
        appendf(
            json, sizeof(json), &used,
            ",\"source_generation\":%llu,\"pipeline\":\"%s\","
            "\"fpga_healthy\":%s,\"phone_healthy\":%s,"
            "\"frame\":{\"width\":%u,\"height\":%u,\"age_ms\":%.1f},"
            "\"fps\":{\"input\":%.2f,\"decode\":%.2f,\"infer\":%.2f},"
            "\"frames\":{\"input\":%llu,\"decoded\":%llu,"
            "\"inferred\":%llu},"
            "\"dropped\":{\"input\":%llu,\"decode\":%llu,"
            "\"infer\":%llu,\"display\":%llu}}",
            (unsigned long long)status->source_generation,
            status->paused ? "paused" : "running",
            status->fpga_healthy ? "true" : "false",
            status->phone_healthy ? "true" : "false",
            status->width, status->height,
            json_finite_or_zero(status->frame_age_ms),
            json_finite_or_zero(status->input_fps),
            json_finite_or_zero(status->decode_fps),
            json_finite_or_zero(status->infer_fps),
            (unsigned long long)status->input_frames,
            (unsigned long long)status->decoded_frames,
            (unsigned long long)status->inferred_frames,
            (unsigned long long)status->input_dropped,
            (unsigned long long)status->decode_dropped,
            (unsigned long long)status->infer_dropped,
            (unsigned long long)status->display_dropped) < 0)
        snprintf(json, sizeof(json),
                 "{\"ok\":false,\"error\":\"serialization_failed\"}");
    pthread_mutex_lock(&control->lock);
    snprintf(control->status_json, sizeof(control->status_json), "%s", json);
    pthread_mutex_unlock(&control->lock);
}

void lpr_control_update_results(struct lpr_control *control,
                                const struct live_result *result,
                                uint64_t accepted_source_generation)
{
    char json[16384];
    size_t used = 0;
    bool accepted;

    if (!control || !control->lock_initialized)
        return;
    accepted = result && accepted_source_generation != 0 &&
               result->source_generation == accepted_source_generation;
    if (!accepted) {
        snprintf(json, sizeof(json),
                 "{\"valid\":false,\"source_generation\":%llu,"
                 "\"detections\":[]}",
                 (unsigned long long)accepted_source_generation);
    } else {
        json[0] = '\0';
        if (appendf(json, sizeof(json), &used,
                    "{\"valid\":%s,\"source_generation\":%llu,"
                    "\"sequence\":%llu,\"input_sequence\":%llu,"
                    "\"frame_timestamp_us\":%lld,\"infer_ms\":%.2f,"
                    "\"frame\":{\"width\":1280,\"height\":720},"
                    "\"detections\":[",
                    result->valid ? "true" : "false",
                    (unsigned long long)result->source_generation,
                    (unsigned long long)result->seq,
                    (unsigned long long)result->input_sequence,
                    (long long)result->frame_monotonic_us,
                    json_finite_or_zero(result->infer_ms)) < 0)
            goto serialization_failed;
        for (int i = 0;
             i < result->result_count && i < MAX_LIVE_PLATES; i++) {
            const struct live_plate_result *plate = &result->plates[i];

            if ((i > 0 && appendf(json, sizeof(json), &used, ",") < 0) ||
                appendf(json, sizeof(json), &used,
                        "{\"x1\":%d,\"y1\":%d,\"x2\":%d,\"y2\":%d,"
                        "\"det_conf\":%.4f,\"ocr_conf\":%.4f,"
                        "\"route\":",
                        plate->box.x1, plate->box.y1,
                        plate->box.x2, plate->box.y2,
                        json_finite_or_zero(plate->box.conf),
                        json_finite_or_zero(plate->conf)) < 0 ||
                append_json_string_n(json, sizeof(json), &used,
                                     plate->route_name,
                                     sizeof(plate->route_name)) < 0 ||
                appendf(json, sizeof(json), &used, ",\"text\":") < 0 ||
                append_json_string_n(json, sizeof(json), &used,
                                     plate->text,
                                     sizeof(plate->text)) < 0 ||
                appendf(json, sizeof(json), &used, "}") < 0)
                goto serialization_failed;
        }
        if (appendf(json, sizeof(json), &used, "]}") < 0)
            goto serialization_failed;
    }
    goto publish;

serialization_failed:
    snprintf(json, sizeof(json),
             "{\"valid\":false,\"source_generation\":%llu,"
             "\"detections\":[],\"error\":\"serialization_failed\"}",
             (unsigned long long)accepted_source_generation);
publish:
    pthread_mutex_lock(&control->lock);
    snprintf(control->results_json, sizeof(control->results_json), "%s", json);
    pthread_mutex_unlock(&control->lock);
}

int lpr_control_update_jpeg(struct lpr_control *control,
                            const uint8_t *jpeg, size_t jpeg_size,
                            uint64_t jpeg_sequence,
                            uint64_t source_generation)
{
    uint8_t *copy;

    if (!control || !jpeg || jpeg_size == 0 || source_generation == 0 ||
        !control->lock_initialized)
        return -1;
    copy = malloc(jpeg_size);
    if (!copy)
        return -1;
    memcpy(copy, jpeg, jpeg_size);
    pthread_mutex_lock(&control->lock);
    free(control->jpeg);
    control->jpeg = copy;
    control->jpeg_size = jpeg_size;
    control->jpeg_sequence = jpeg_sequence;
    control->jpeg_source_generation = source_generation;
    pthread_mutex_unlock(&control->lock);
    return 0;
}

void lpr_control_clear_jpeg(struct lpr_control *control)
{
    if (!control || !control->lock_initialized)
        return;
    pthread_mutex_lock(&control->lock);
    free(control->jpeg);
    control->jpeg = NULL;
    control->jpeg_size = 0;
    control->jpeg_source_generation = 0;
    control->jpeg_sequence++;
    if (control->jpeg_sequence == 0)
        control->jpeg_sequence = 1;
    pthread_mutex_unlock(&control->lock);
}

enum lpr_control_source lpr_control_take_source(struct lpr_control *control)
{
    enum lpr_control_source source = LPR_CONTROL_SOURCE_NONE;

    if (!control || !control->lock_initialized)
        return source;
    pthread_mutex_lock(&control->lock);
    source = control->pending_source;
    control->pending_source = LPR_CONTROL_SOURCE_NONE;
    pthread_mutex_unlock(&control->lock);
    return source;
}

enum lpr_pipeline_command
lpr_control_take_pipeline_command(struct lpr_control *control)
{
    enum lpr_pipeline_command command = LPR_PIPELINE_COMMAND_NONE;

    if (!control || !control->lock_initialized)
        return command;
    pthread_mutex_lock(&control->lock);
    command = control->pending_pipeline_command;
    control->pending_pipeline_command = LPR_PIPELINE_COMMAND_NONE;
    pthread_mutex_unlock(&control->lock);
    return command;
}
