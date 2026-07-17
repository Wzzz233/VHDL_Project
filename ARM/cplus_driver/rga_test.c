// SPDX-License-Identifier: MIT
// librga 硬件转换验证（im2d v2, librga 1.10.1）：
// 从 stdin 读 1080p NV12，用 RGA 硬件缩放+转格式成 1280x720 BGRx，输出 stdout。
// 关键：必须先用 importbuffer_virtualaddr 把用户态内存导入 RGA 驱动拿到 handle，
// 再用 wrapbuffer_handle 构造 rga_buffer_t，否则 handle=0 会触发内核 Oops。
#define _GNU_SOURCE
#include <im2d.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define SRC_W 1920
#define SRC_H 1080
#define DST_W 1280
#define DST_H 720
#define NV12_SIZE  (SRC_W * SRC_H * 3 / 2)   /* 3110400 */
#define BGRX_SIZE  (DST_W * DST_H * 4)        /* 3686400 */

static int64_t now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000LL + t.tv_nsec / 1000;
}

int main(void)
{
    uint8_t *src_buf = NULL, *dst_buf = NULL;
    rga_buffer_handle_t src_handle_id = 0, dst_handle_id = 0;
    rga_buffer_t src = {0}, dst = {0};
    im_handle_param_t src_param = {0}, dst_param = {0};
    int64_t t0, t1;
    int frames = 0, rga_calls = 0;
    IM_STATUS st;

    posix_memalign((void **)&src_buf, 64, NV12_SIZE);
    posix_memalign((void **)&dst_buf, 64, BGRX_SIZE);
    if (!src_buf || !dst_buf) {
        fprintf(stderr, "alloc failed\n");
        return 1;
    }
    memset(dst_buf, 0, BGRX_SIZE);

    /* 1) 把用户态 malloc 内存导入 RGA 驱动，拿到 buffer handle。
     *    im_handle_param_t { width, height, format } 在 im2d_buffer.h 里。 */
    src_param.width = SRC_W;
    src_param.height = SRC_H;
    src_param.format = RK_FORMAT_YCbCr_420_SP;   /* NV12 */
    src_handle_id = importbuffer_virtualaddr(src_buf, &src_param);
    if (!src_handle_id) {
        fprintf(stderr, "importbuffer_virtualaddr(src) failed\n");
        return 1;
    }

    dst_param.width = DST_W;
    dst_param.height = DST_H;
    dst_param.format = RK_FORMAT_BGRA_8888;       /* BGRx */
    dst_handle_id = importbuffer_virtualaddr(dst_buf, &dst_param);
    if (!dst_handle_id) {
        fprintf(stderr, "importbuffer_virtualaddr(dst) failed\n");
        releasebuffer_handle(src_handle_id);
        return 1;
    }

    /* 2) 用 handle 构造 rga_buffer_t（这次 handle 字段会被正确填充）。
     *    wrapbuffer_handle(handle, width, height, format)。 */
    src = wrapbuffer_handle(src_handle_id, SRC_W, SRC_H, RK_FORMAT_YCbCr_420_SP);
    dst = wrapbuffer_handle(dst_handle_id, DST_W, DST_H, RK_FORMAT_BGRA_8888);

    fprintf(stderr, "[rga] handles src=%u dst=%u  src.w=%d dst.w=%d\n",
            (unsigned)src_handle_id, (unsigned)dst_handle_id,
            src.width, dst.width);

    t0 = now_us();
    while (frames < 300) {
        ssize_t need = NV12_SIZE, got = 0;
        while (got < need) {
            ssize_t n = read(STDIN_FILENO, src_buf + got, need - got);
            if (n == 0) goto done;
            if (n < 0) { perror("read"); goto done; }
            got += n;
        }

        /* 3) 硬件：NV12 1920x1080 -> BGRx 1280x720，缩放+格式转换一步完成。 */
        st = imresize(src, dst);
        if (st != IM_STATUS_SUCCESS && st != IM_STATUS_NOERROR) {
            if (rga_calls == 0)
                fprintf(stderr, "imresize failed status=%d\n", (int)st);
        } else {
            ++rga_calls;
        }

        if (write(STDOUT_FILENO, dst_buf, BGRX_SIZE) != BGRX_SIZE) {
            perror("write");
            goto done;
        }
        ++frames;
    }
done:
    t1 = now_us();
    {
        double secs = (double)(t1 - t0) / 1000000.0;
        if (secs > 0.0)
            fprintf(stderr, "[rga] frames=%d rga_ok=%d  %.1f fps  (%.0f MB/s out)\n",
                    frames, rga_calls, frames / secs,
                    (double)frames * BGRX_SIZE / secs / (1024.0 * 1024.0));
        else
            fprintf(stderr, "[rga] frames=%d rga_ok=%d (no timing)\n", frames, rga_calls);
    }
    if (src_handle_id) releasebuffer_handle(src_handle_id);
    if (dst_handle_id) releasebuffer_handle(dst_handle_id);
    free(src_buf);
    free(dst_buf);
    return 0;
}
