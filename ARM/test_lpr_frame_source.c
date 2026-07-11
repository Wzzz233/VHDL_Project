// SPDX-License-Identifier: GPL-2.0
/* Host tests for generic frame ownership, source policy and letterboxing. */

#include "lpr_live/lpr_frame.h"
#include "lpr_live/lpr_source.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#define CHECK(expr) do {                                                     \
    if (!(expr)) {                                                           \
        fprintf(stderr, "[FAIL] %s:%d: %s\n", __func__, __LINE__, #expr);  \
        return -1;                                                           \
    }                                                                        \
} while (0)

static struct lpr_frame_meta make_meta(uint32_t width, uint32_t height,
                                       uint32_t stride, uint64_t sequence,
                                       uint64_t source_generation)
{
    struct lpr_frame_meta meta;

    memset(&meta, 0, sizeof(meta));
    meta.format = LPR_FRAME_FORMAT_BGRX8888;
    meta.width = width;
    meta.height = height;
    meta.stride = stride;
    meta.monotonic_us = 123456;
    meta.sequence = sequence;
    meta.source_generation = source_generation;
    meta.fpga_caps = LPR_FRAME_FPGA_CAP_DMA |
                     LPR_FRAME_FPGA_CAP_FRAME_STATUS;
    return meta;
}

static int test_heap_pool_ref_generation(void)
{
    struct lpr_frame_pool pool;
    struct lpr_frame_writer writer;
    struct lpr_frame_ref ref;
    struct lpr_frame_ref clone;
    struct lpr_frame_ref rebound;
    struct lpr_frame_ref rebound_clone;
    struct lpr_frame_ref stale;
    struct lpr_frame_ref current;
    struct lpr_frame_ref rejected;
    struct lpr_frame_meta meta = make_meta(4, 4, 16, 7, 3);
    uint64_t first_slot_generation;

    memset(&pool, 0, sizeof(pool));
    memset(&writer, 0, sizeof(writer));
    memset(&ref, 0, sizeof(ref));
    memset(&clone, 0, sizeof(clone));
    memset(&rebound, 0, sizeof(rebound));
    memset(&rebound_clone, 0, sizeof(rebound_clone));
    memset(&current, 0, sizeof(current));
    memset(&rejected, 0, sizeof(rejected));

    CHECK(lpr_frame_pool_init_heap(&pool, 1, 64) == 0);
    CHECK(lpr_frame_pool_acquire(&pool, &writer, false) == 0);
    CHECK(lpr_frame_writer_capacity(&writer) == 64);
    CHECK(lpr_frame_writer_data(&writer) != NULL);
    memset(lpr_frame_writer_data(&writer), 0x5a, 64);
    CHECK(lpr_frame_writer_publish(&writer, &meta, &ref) == 0);
    CHECK(writer.pool == NULL);
    CHECK(lpr_frame_ref_is_valid(&ref));
    CHECK(lpr_frame_ref_capacity(&ref) == 64);
    CHECK(lpr_frame_ref_data(&ref)[17] == 0x5a);
    CHECK(ref.meta.width == 4 && ref.meta.height == 4);
    CHECK(ref.meta.fpga_caps == (LPR_FRAME_FPGA_CAP_DMA |
                                 LPR_FRAME_FPGA_CAP_FRAME_STATUS));

    stale = ref;
    first_slot_generation = ref.slot_generation;
    CHECK(lpr_frame_ref_clone(&ref, &clone) == 0);
    CHECK(lpr_frame_ref_clone_rebind_source_generation(&ref, 0,
                                                       &rejected) == -EINVAL);
    CHECK(lpr_frame_ref_clone_rebind_source_generation(&ref, 99,
                                                       &rebound) == 0);
    CHECK(rebound.meta.source_generation == 99);
    CHECK(ref.meta.source_generation == 3);
    CHECK(lpr_frame_ref_clone(&rebound, &rebound_clone) == 0);
    CHECK(rebound_clone.meta.source_generation == 99);
    CHECK(lpr_frame_ref_matches_source_generation(&rebound_clone, 99));

    CHECK(lpr_frame_ref_release(&ref) == 0);
    CHECK(lpr_frame_pool_acquire(&pool, &writer, false) == -EAGAIN);
    CHECK(lpr_frame_ref_release(&clone) == 0);
    CHECK(lpr_frame_ref_release(&rebound) == 0);
    CHECK(lpr_frame_ref_release(&rebound_clone) == 0);

    CHECK(lpr_frame_pool_acquire(&pool, &writer, false) == 0);
    CHECK(writer.slot_generation != first_slot_generation);
    meta.sequence = 8;
    meta.source_generation = 4;
    CHECK(lpr_frame_writer_publish(&writer, &meta, &current) == 0);
    CHECK(lpr_frame_ref_clone(&stale, &rejected) == -ESTALE);
    CHECK(rejected.pool == NULL);
    CHECK(lpr_frame_ref_release(&stale) == -ESTALE);
    CHECK(lpr_frame_ref_is_valid(&current));
    CHECK(lpr_frame_pool_destroy(&pool) == -EBUSY);
    CHECK(lpr_frame_ref_release(&current) == 0);
    CHECK(lpr_frame_pool_destroy(&pool) == 0);
    return 0;
}

static int test_external_pool_and_shutdown(void)
{
    uint8_t external[16];
    uint8_t *buffers[] = { external };
    const size_t capacities[] = { sizeof(external) };
    struct lpr_frame_pool pool;
    struct lpr_frame_writer writer;

    memset(external, 0, sizeof(external));
    memset(&pool, 0, sizeof(pool));
    memset(&writer, 0, sizeof(writer));
    CHECK(lpr_frame_pool_init_external(&pool, buffers, capacities, 1) == 0);
    CHECK(lpr_frame_pool_acquire(&pool, &writer, false) == 0);
    CHECK(lpr_frame_writer_data(&writer) == external);
    lpr_frame_writer_data(&writer)[0] = 0xa5;
    CHECK(lpr_frame_writer_abort(&writer) == 0);
    lpr_frame_pool_shutdown(&pool);
    CHECK(lpr_frame_pool_acquire(&pool, &writer, false) == -ESHUTDOWN);
    CHECK(lpr_frame_pool_destroy(&pool) == 0);
    CHECK(external[0] == 0xa5);
    return 0;
}

static bool black_pixel(const uint8_t *pixel)
{
    return pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0 &&
           pixel[3] == 0xff;
}

static int test_letterbox_copy(void)
{
    uint8_t wide[2][16];
    uint8_t wide_out[4][20];
    uint8_t tall[4][8];
    uint8_t tall_out[4][16];
    struct lpr_letterbox_layout layout;

    memset(wide, 0, sizeof(wide));
    for (uint32_t y = 0; y < 2; y++) {
        for (uint32_t x = 0; x < 4; x++) {
            wide[y][x * 4U] = (uint8_t)(x + 1U);
            wide[y][x * 4U + 1U] = (uint8_t)(y + 10U);
            wide[y][x * 4U + 2U] = 42;
            wide[y][x * 4U + 3U] = 0xaa;
        }
    }
    memset(wide_out, 0xcc, sizeof(wide_out));
    CHECK(lpr_bgrx_letterbox_copy(&wide[0][0], 4, 2, 16,
                                  &wide_out[0][0], 4, 4, 20, &layout) == 0);
    CHECK(layout.scaled_width == 4 && layout.scaled_height == 2);
    CHECK(layout.pad_left == 0 && layout.pad_top == 1);
    CHECK(black_pixel(&wide_out[0][0]));
    CHECK(black_pixel(&wide_out[3][12]));
    CHECK(wide_out[0][16] == 0 && wide_out[0][19] == 0);
    CHECK(memcmp(&wide_out[1][0], &wide[0][0], 16) == 0);
    CHECK(memcmp(&wide_out[2][0], &wide[1][0], 16) == 0);

    memset(tall, 0, sizeof(tall));
    for (uint32_t y = 0; y < 4; y++) {
        for (uint32_t x = 0; x < 2; x++) {
            tall[y][x * 4U] = (uint8_t)(20U + x);
            tall[y][x * 4U + 1U] = (uint8_t)(30U + y);
            tall[y][x * 4U + 2U] = 40;
            tall[y][x * 4U + 3U] = 0xff;
        }
    }
    CHECK(lpr_bgrx_letterbox_copy(&tall[0][0], 2, 4, 8,
                                  &tall_out[0][0], 4, 4, 16,
                                  &layout) == 0);
    CHECK(layout.scaled_width == 2 && layout.scaled_height == 4);
    CHECK(layout.pad_left == 1 && layout.pad_top == 0);
    for (uint32_t y = 0; y < 4; y++) {
        CHECK(black_pixel(&tall_out[y][0]));
        CHECK(black_pixel(&tall_out[y][12]));
        CHECK(memcmp(&tall_out[y][4], &tall[y][0], 8) == 0);
    }
    CHECK(lpr_bgrx_letterbox_copy(&tall[0][0], 2, 4, 7,
                                  &tall_out[0][0], 4, 4, 16,
                                  NULL) == -EINVAL);
    return 0;
}

static void set_phone_health(struct lpr_source_health *health,
                             int64_t last_frame_us, uint64_t sequence)
{
    memset(health, 0, sizeof(*health));
    health->opened = true;
    health->running = true;
    health->healthy = true;
    health->has_frame = true;
    health->last_frame_us = last_frame_us;
    health->sequence = sequence;
}

static int test_source_manager_failover(void)
{
    const int64_t start = 1000000;
    struct lpr_source_manager manager;
    struct lpr_source_switch_event event;
    struct lpr_source_health phone;
    uint64_t phone_generation;

    CHECK(lpr_source_manager_init(&manager, LPR_SOURCE_FPGA,
                                  LPR_SOURCE_FPGA, 7, start) == 0);
    set_phone_health(&phone, start, 1);
    CHECK(lpr_source_manager_set_desired(&manager, LPR_SOURCE_PHONE,
                                         &phone, start, &event) == 1);
    CHECK(event.switched && event.previous == LPR_SOURCE_FPGA &&
          event.active == LPR_SOURCE_PHONE);
    CHECK(manager.active == LPR_SOURCE_PHONE &&
          manager.source_generation == 8 &&
          manager.reason == LPR_SOURCE_REASON_DESIRED_PHONE);
    phone_generation = manager.source_generation;
    CHECK(lpr_source_result_is_current(&manager, phone_generation));

    phone.running = false;
    phone.healthy = false;
    CHECK(lpr_source_manager_update(&manager, &phone, start + 500000,
                                    &event) == 0);
    CHECK(manager.active == LPR_SOURCE_PHONE);
    CHECK(lpr_source_manager_update(&manager, &phone,
                                     start + LPR_PHONE_STALE_US - 1,
                                     &event) == 0);
    CHECK(manager.active == LPR_SOURCE_PHONE);
    CHECK(lpr_source_manager_update(&manager, &phone,
                                     start + LPR_PHONE_STALE_US,
                                     &event) == 1);
    CHECK(manager.active == LPR_SOURCE_FPGA &&
          manager.source_generation == 9 &&
          manager.reason == LPR_SOURCE_REASON_PHONE_STALE);
    CHECK(!lpr_source_result_is_current(&manager, phone_generation));

    set_phone_health(&phone, start + 2100000, 2);
    CHECK(lpr_source_manager_update(&manager, &phone, start + 2100000,
                                    &event) == 0);
    set_phone_health(&phone, start + 4000000, 3);
    CHECK(lpr_source_manager_update(&manager, &phone, start + 4000000,
                                    &event) == 0);
    phone.healthy = false;
    CHECK(lpr_source_manager_update(&manager, &phone, start + 4500000,
                                    &event) == 0);
    CHECK(manager.phone_recovery_since_us == -1);

    set_phone_health(&phone, start + 5000000, 4);
    CHECK(lpr_source_manager_update(&manager, &phone, start + 5000000,
                                    &event) == 0);
    set_phone_health(&phone, start + 7999999, 5);
    CHECK(lpr_source_manager_update(&manager, &phone, start + 7999999,
                                    &event) == 0);
    CHECK(manager.active == LPR_SOURCE_FPGA);
    set_phone_health(&phone, start + 8000000, 6);
    CHECK(lpr_source_manager_update(&manager, &phone, start + 8000000,
                                    &event) == 1);
    CHECK(manager.active == LPR_SOURCE_PHONE &&
          manager.source_generation == 10 &&
          manager.reason == LPR_SOURCE_REASON_PHONE_RECOVERED);

    CHECK(lpr_source_manager_set_desired(&manager, LPR_SOURCE_FPGA, NULL,
                                         start + 8000001, &event) == 1);
    CHECK(manager.active == LPR_SOURCE_FPGA &&
          manager.source_generation == 11 &&
          manager.reason == LPR_SOURCE_REASON_DESIRED_FPGA);
    CHECK(lpr_source_manager_update(&manager, NULL, start + 8000000,
                                    &event) == -ERANGE);
    CHECK(strcmp(lpr_source_reason_string(LPR_SOURCE_REASON_PHONE_STALE),
                 "phone_stale") == 0);
    CHECK(strcmp(lpr_source_id_string(LPR_SOURCE_PHONE), "phone") == 0);
    return 0;
}

static int test_source_manager_generation_epochs(void)
{
    const int64_t start = 2000000;
    struct lpr_source_manager manager;
    struct lpr_source_health phone;

    CHECK(lpr_source_manager_init(&manager, LPR_SOURCE_PHONE,
                                  LPR_SOURCE_FPGA, UINT64_MAX, start) == 0);
    manager.reason = LPR_SOURCE_REASON_PHONE_STALE;
    manager.phone_failover_latched = true;
    manager.phone_recovery_since_us = start + 10;
    CHECK(lpr_source_manager_advance_generation(&manager,
                                                start + 50) == 0);
    CHECK(manager.source_generation == 1);
    CHECK(manager.desired == LPR_SOURCE_PHONE &&
          manager.active == LPR_SOURCE_FPGA);
    CHECK(manager.reason == LPR_SOURCE_REASON_PHONE_STALE &&
          manager.phone_failover_latched &&
          manager.phone_recovery_since_us == -1);
    CHECK(manager.active_since_us == start + 50 &&
          manager.last_update_us == start + 50);
    CHECK(!lpr_source_result_is_current(&manager, UINT64_MAX));
    CHECK(lpr_source_manager_advance_generation(&manager, start + 49) ==
          -ERANGE);
    CHECK(lpr_source_manager_advance_generation(NULL, start) == -EINVAL);

    CHECK(lpr_source_manager_init(&manager, LPR_SOURCE_PHONE,
                                  LPR_SOURCE_PHONE, 41, start) == 0);
    manager.reason = LPR_SOURCE_REASON_PHONE_RECOVERED;
    CHECK(lpr_source_manager_restart(&manager, start + 100) == 0);
    CHECK(manager.source_generation == 42 &&
          manager.desired == LPR_SOURCE_PHONE &&
          manager.active == LPR_SOURCE_PHONE);
    CHECK(manager.reason == LPR_SOURCE_REASON_PHONE_RECOVERED);
    CHECK(manager.active_since_us == start + 100 &&
          manager.last_update_us == start + 100);
    set_phone_health(&phone, start + 100, 1);
    CHECK(lpr_source_manager_update(&manager, &phone, start + 100,
                                    NULL) == 0);
    CHECK(manager.reason == LPR_SOURCE_REASON_PHONE_RECOVERED);
    CHECK(lpr_source_manager_restart(&manager, start + 99) == -ERANGE);
    return 0;
}

static int test_repeated_phone_failover_recovery(void)
{
    const int64_t start = 3000000;
    struct lpr_source_manager manager;
    struct lpr_source_switch_event event;
    struct lpr_source_health phone;
    uint64_t sequence = 1;
    int64_t cycle_start = start;

    CHECK(lpr_source_manager_init(&manager, LPR_SOURCE_PHONE,
                                  LPR_SOURCE_PHONE, 100, start) == 0);
    manager.reason = LPR_SOURCE_REASON_DESIRED_PHONE;
    for (int cycle = 0; cycle < 3; cycle++) {
        int64_t stale_at = cycle_start + LPR_PHONE_STALE_US;
        int64_t recovery_start = stale_at + 1;
        int64_t recovered_at = recovery_start + LPR_PHONE_RECOVERY_US;

        set_phone_health(&phone, cycle_start, sequence++);
        CHECK(lpr_source_manager_update(&manager, &phone, cycle_start,
                                        &event) == 0);
        CHECK(lpr_source_manager_update(&manager, &phone, stale_at - 1,
                                        &event) == 0);
        CHECK(manager.active == LPR_SOURCE_PHONE);
        CHECK(lpr_source_manager_update(&manager, &phone, stale_at,
                                        &event) == 1);
        CHECK(manager.active == LPR_SOURCE_FPGA && event.switched &&
              manager.reason == LPR_SOURCE_REASON_PHONE_STALE);

        set_phone_health(&phone, recovery_start, sequence++);
        CHECK(lpr_source_manager_update(&manager, &phone, recovery_start,
                                        &event) == 0);
        CHECK(manager.phone_recovery_since_us == recovery_start);
        set_phone_health(&phone, recovered_at - 1, sequence++);
        CHECK(lpr_source_manager_update(&manager, &phone, recovered_at - 1,
                                        &event) == 0);
        CHECK(manager.active == LPR_SOURCE_FPGA);
        set_phone_health(&phone, recovered_at, sequence++);
        CHECK(lpr_source_manager_update(&manager, &phone, recovered_at,
                                        &event) == 1);
        CHECK(manager.active == LPR_SOURCE_PHONE && event.switched &&
              manager.reason == LPR_SOURCE_REASON_PHONE_RECOVERED);
        CHECK(manager.desired == LPR_SOURCE_PHONE &&
              manager.source_generation == 100U + (uint64_t)(cycle + 1) * 2U);
        cycle_start = recovered_at + 1;
    }
    return 0;
}

struct fake_source {
    int open_calls;
    int start_calls;
    int read_calls;
    int health_calls;
    int stop_calls;
    int close_calls;
    struct lpr_frame_ref cached;
    struct lpr_source_health health;
};

static int fake_open(void *opaque)
{
    struct fake_source *fake = opaque;
    fake->open_calls++;
    return 0;
}

static int fake_start(void *opaque)
{
    struct fake_source *fake = opaque;
    fake->start_calls++;
    return 0;
}

static int fake_read_latest(void *opaque, struct lpr_frame_ref *out)
{
    struct fake_source *fake = opaque;
    fake->read_calls++;
    return lpr_frame_ref_clone(&fake->cached, out);
}

static int fake_health(void *opaque, int64_t now_us,
                       struct lpr_source_health *out)
{
    struct fake_source *fake = opaque;
    (void)now_us;
    fake->health_calls++;
    *out = fake->health;
    return 0;
}

static void fake_stop(void *opaque)
{
    struct fake_source *fake = opaque;
    fake->stop_calls++;
}

static void fake_close(void *opaque)
{
    struct fake_source *fake = opaque;
    fake->close_calls++;
}

static int test_frame_source_interface(void)
{
    static const struct lpr_frame_source_ops ops = {
        .open = fake_open,
        .start = fake_start,
        .read_latest = fake_read_latest,
        .health = fake_health,
        .stop = fake_stop,
        .close = fake_close,
    };
    struct lpr_frame_pool pool;
    struct lpr_frame_writer writer;
    struct lpr_frame_ref read_ref;
    struct lpr_frame_meta meta = make_meta(2, 2, 8, 1, 1);
    struct lpr_source_health health;
    struct fake_source fake;
    struct lpr_frame_source source;

    memset(&pool, 0, sizeof(pool));
    memset(&writer, 0, sizeof(writer));
    memset(&read_ref, 0, sizeof(read_ref));
    memset(&fake, 0, sizeof(fake));
    memset(&source, 0, sizeof(source));
    CHECK(lpr_frame_pool_init_heap(&pool, 1, 16) == 0);
    CHECK(lpr_frame_pool_acquire(&pool, &writer, false) == 0);
    CHECK(lpr_frame_writer_publish(&writer, &meta, &fake.cached) == 0);
    set_phone_health(&fake.health, 450, 1);
    fake.health.input_fps = 14.5;
    fake.health.dropped_frames = 2;
    source.id = LPR_SOURCE_PHONE;
    source.name = "fake-phone";
    source.ops = &ops;
    source.ctx = &fake;

    CHECK(lpr_frame_source_open(&source) == 0);
    CHECK(lpr_frame_source_start(&source) == 0);
    CHECK(lpr_frame_source_read_latest(&source, &read_ref) == 0);
    CHECK(lpr_frame_ref_data(&read_ref) == lpr_frame_ref_data(&fake.cached));
    CHECK(lpr_frame_source_health(&source, 500, &health) == 0);
    CHECK(health.frame_age_us == 50 && health.input_fps == 14.5 &&
          health.dropped_frames == 2);
    lpr_frame_source_stop(&source);
    lpr_frame_source_close(&source);
    CHECK(fake.open_calls == 1 && fake.start_calls == 1 &&
          fake.read_calls == 1 && fake.health_calls == 1 &&
          fake.stop_calls == 1 && fake.close_calls == 1);
    CHECK(lpr_frame_ref_release(&read_ref) == 0);
    CHECK(lpr_frame_ref_release(&fake.cached) == 0);
    CHECK(lpr_frame_pool_destroy(&pool) == 0);
    return 0;
}

int main(void)
{
    if (test_heap_pool_ref_generation() < 0)
        return 1;
    puts("[PASS] heap pool refcount and generation");
    if (test_external_pool_and_shutdown() < 0)
        return 1;
    puts("[PASS] external pool and shutdown");
    if (test_letterbox_copy() < 0)
        return 1;
    puts("[PASS] BGRx aspect-preserving letterbox");
    if (test_source_manager_failover() < 0)
        return 1;
    puts("[PASS] source manager 2s fallback and 3s recovery");
    if (test_source_manager_generation_epochs() < 0)
        return 1;
    puts("[PASS] source manager pause/restart generation epochs");
    if (test_repeated_phone_failover_recovery() < 0)
        return 1;
    puts("[PASS] repeated phone failover/recovery cycles");
    if (test_frame_source_interface() < 0)
        return 1;
    puts("[PASS] frame source interface");
    puts("[PASS] all frame/source tests");
    return 0;
}
