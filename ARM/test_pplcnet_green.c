/*
 * test_pplcnet_green.c — PPLCNet backbone 绿牌 dark OCR 独立上板测试驱动。
 *
 * 功能: 加载 pplcnet_green_v1_rk3568_fp16.rknn, 逐帧读 PPM 图,
 *       resize 到模型输入尺寸, uint8 NHWC 送 RKNN, CTC greedy decode, 打印结果。
 *
 * 设计: 完全独立, 不依赖 FPGA/GStreamer/pthread。复用 ocr_decode.c 的 CTC decode。
 *       不改动 fpga_lpr_display.c 任何代码。
 *
 * 编译: make pplcnet-test  (aarch64-linux-gnu-gcc 交叉编译, 链 -lrknnrt -lm)
 * 运行: ./test_pplcnet_green <model.rknn> <keys.txt> <image.ppm> [image2.ppm ...]
 *       或 ./test_pplcnet_green <model.rknn> <keys.txt> @list.txt  (每行一个 ppm 路径)
 *
 * 通道顺序说明: 训练用 cv2.imread (BGR) 直接喂模型, 故模型按 BGR 训练。
 *              本程序读 PPM (RGB) 后转 BGR 再送, 匹配训练分布。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <time.h>

#include "rknn_api.h"
#include "ocr_decode.h"

/* ---------------- PPM 读取 (RGB888, P6) ---------------- */

static int ppm_next_token(FILE *fp, char *tok, size_t tok_sz)
{
    int ch;
    size_t n = 0;
    if (!fp || !tok || tok_sz == 0) return -1;
    tok[0] = '\0';
    while (1) {
        ch = fgetc(fp);
        if (ch == '#') { while (ch != '\n' && ch != EOF) ch = fgetc(fp); continue; }
        if (ch == EOF) return -1;
        if (!isspace((unsigned char)ch)) break;
    }
    while (ch != EOF && !isspace((unsigned char)ch) && ch != '#') {
        if (n + 1 < tok_sz) tok[n++] = (char)ch;
        ch = fgetc(fp);
    }
    tok[n] = '\0';
    if (ch == '#') { while (ch != '\n' && ch != EOF) ch = fgetc(fp); }
    return (n > 0) ? 0 : -1;
}

static int read_ppm_rgb888(const char *path, uint8_t **rgb_out, int *w_out, int *h_out)
{
    FILE *fp; char tok[64]; int w, h, maxv, ch; uint8_t *buf; size_t need, got;
    *rgb_out = NULL; *w_out = 0; *h_out = 0;
    fp = fopen(path, "rb");
    if (!fp) return -1;
    if (ppm_next_token(fp, tok, sizeof(tok)) < 0 || strcmp(tok, "P6") != 0) goto fail;
    if (ppm_next_token(fp, tok, sizeof(tok)) < 0) goto fail; w = atoi(tok);
    if (ppm_next_token(fp, tok, sizeof(tok)) < 0) goto fail; h = atoi(tok);
    if (ppm_next_token(fp, tok, sizeof(tok)) < 0) goto fail; maxv = atoi(tok);
    if (w <= 0 || h <= 0 || maxv != 255) goto fail;
    do { ch = fgetc(fp); } while (ch != EOF && isspace((unsigned char)ch));
    if (ch == EOF) goto fail;
    if (ungetc(ch, fp) == EOF) goto fail;
    need = (size_t)w * (size_t)h * 3U;
    buf = (uint8_t *)malloc(need);
    if (!buf) goto fail;
    got = fread(buf, 1, need, fp);
    if (got != need) { free(buf); goto fail; }
    fclose(fp);
    *rgb_out = buf; *w_out = w; *h_out = h;
    return 0;
fail:
    if (fp) fclose(fp);
    return -1;
}

/* ---------------- bilinear resize (RGB888, 与训练 cv2 INTER_LINEAR 对齐) ---------------- */

static void resize_rgb888_bilinear(const uint8_t *src, int sw, int sh,
                                   uint8_t *dst, int dw, int dh)
{
    int x, y;
    if (!src || !dst || sw <= 0 || sh <= 0 || dw <= 0 || dh <= 0) return;
    for (y = 0; y < dh; y++) {
        float fy = ((float)y + 0.5f) * (float)sh / (float)dh - 0.5f;
        int y0 = (int)floorf(fy); int y1; float wy;
        if (y0 < 0) y0 = 0; if (y0 > sh - 1) y0 = sh - 1;
        y1 = y0 + 1; if (y1 > sh - 1) y1 = sh - 1;
        wy = fy - (float)y0; if (wy < 0) wy = 0; if (wy > 1) wy = 1;
        for (x = 0; x < dw; x++) {
            float fx = ((float)x + 0.5f) * (float)sw / (float)dw - 0.5f;
            int x0 = (int)floorf(fx); int x1; float wx;
            const uint8_t *p00, *p01, *p10, *p11; uint8_t *q; int c;
            if (x0 < 0) x0 = 0; if (x0 > sw - 1) x0 = sw - 1;
            x1 = x0 + 1; if (x1 > sw - 1) x1 = sw - 1;
            wx = fx - (float)x0; if (wx < 0) wx = 0; if (wx > 1) wx = 1;
            p00 = src + ((size_t)y0 * sw + x0) * 3U;
            p01 = src + ((size_t)y0 * sw + x1) * 3U;
            p10 = src + ((size_t)y1 * sw + x0) * 3U;
            p11 = src + ((size_t)y1 * sw + x1) * 3U;
            q = dst + ((size_t)y * dw + x) * 3U;
            for (c = 0; c < 3; c++) {
                float v0 = p00[c] * (1 - wx) + p01[c] * wx;
                float v1 = p10[c] * (1 - wx) + p11[c] * wx;
                int iv = (int)(v0 * (1 - wy) + v1 * wy + 0.5f);
                if (iv < 0) iv = 0; if (iv > 255) iv = 255;
                q[c] = (uint8_t)iv;
            }
        }
    }
}

/* RGB -> BGR 原地交换 (匹配 cv2.imread BGR 训练分布) */
static void rgb_to_bgr_inplace(uint8_t *buf, int w, int h)
{
    size_t n = (size_t)w * h;
    for (size_t i = 0; i < n; i++) {
        uint8_t t = buf[i * 3 + 0];
        buf[i * 3 + 0] = buf[i * 3 + 2];
        buf[i * 3 + 2] = t;
    }
}

/* gray3: BGR -> 灰度 -> 复制到 3 通道 (B1 训练用 preproc, opencv 灰度公式) */
static void apply_gray3_inplace(uint8_t *buf, int w, int h)
{
    /* opencv: gray = 0.114*B + 0.587*G + 0.299*R, 用整数近似 */
    size_t n = (size_t)w * h;
    for (size_t i = 0; i < n; i++) {
        uint8_t b = buf[i * 3 + 0];
        uint8_t g = buf[i * 3 + 1];
        uint8_t r = buf[i * 3 + 2];
        /* (114*B + 587*G + 299*R) / 1000, 与 cv2.cvtColor BGR2GRAY 等价 */
        uint8_t gray = (uint8_t)((114 * b + 587 * g + 299 * r) / 1000);
        buf[i * 3 + 0] = gray;
        buf[i * 3 + 1] = gray;
        buf[i * 3 + 2] = gray;
    }
}

/* ---------------- keys 加载 (每行一个 UTF-8 字符, # 注释, 空行跳过) ---------------- */

static int load_keys(const char *path, char ***keys_out, int *count_out)
{
    FILE *fp; char line[256]; char **keys = NULL; int n = 0, cap = 0;
    fp = fopen(path, "rb");
    if (!fp) return -1;
    while (fgets(line, sizeof(line), fp)) {
        char *s = line; size_t len;
        while (*s && isspace((unsigned char)*s)) s++;
        if (*s == '#' || *s == '\0') continue;
        len = strlen(s);
        while (len > 0 && (s[len-1] == '\n' || s[len-1] == '\r')) s[--len] = '\0';
        if (len == 0) continue;
        if (n == cap) { cap = cap ? cap * 2 : 64; keys = realloc(keys, cap * sizeof(char*)); }
        keys[n] = strdup(s);
        n++;
    }
    fclose(fp);
    *keys_out = keys; *count_out = n;
    return 0;
}

/* ---------------- main ---------------- */

int main(int argc, char **argv)
{
    const char *model_path, *keys_path;
    FILE *fp; long fsize; uint8_t *model_buf = NULL;
    rknn_context ctx; rknn_input_output_num io_num;
    rknn_tensor_attr in_attr, out_attr;
    int in_w, in_h, in_c, ret, i;
    char **keys = NULL; int key_count = 0;
    int blank_idx;
    uint8_t *input_buf = NULL;

    if (argc < 4) {
        fprintf(stderr, "用法: %s <model.rknn> <keys.txt> <img.ppm> [...] | @list.txt\n", argv[0]);
        return 1;
    }
    model_path = argv[1];
    keys_path = argv[2];

    /* keys */
    if (load_keys(keys_path, &keys, &key_count) < 0 || key_count <= 0) {
        fprintf(stderr, "[ERR] 加载 keys 失败: %s\n", keys_path); return 1;
    }
    printf("[keys] %d 个字符, blank=最后位\n", key_count);

    /* model */
    fp = fopen(model_path, "rb");
    if (!fp) { fprintf(stderr, "[ERR] 打不开模型 %s\n", model_path); return 1; }
    fseek(fp, 0, SEEK_END); fsize = ftell(fp); fseek(fp, 0, SEEK_SET);
    model_buf = (uint8_t*)malloc(fsize);
    if (fread(model_buf, 1, fsize, fp) != (size_t)fsize) { fprintf(stderr, "[ERR] 读模型失败\n"); return 1; }
    fclose(fp);

    ret = rknn_init(&ctx, model_buf, fsize, 0, NULL);
    if (ret < 0) { fprintf(stderr, "[ERR] rknn_init 失败: %d\n", ret); return 1; }
    printf("[model] rknn_init OK (%ld bytes)\n", fsize);

    rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    printf("[io] n_input=%d n_output=%d\n", io_num.n_input, io_num.n_output);

    memset(&in_attr, 0, sizeof(in_attr)); in_attr.index = 0;
    rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &in_attr, sizeof(in_attr));
    /* dims: NCHW 或 NHWC, 取后三维 */
    {
        int *d = in_attr.dims;
        if (in_attr.fmt == RKNN_TENSOR_NHWC) { in_h = d[1]; in_w = d[2]; in_c = d[3]; }
        else { in_c = d[1]; in_h = d[2]; in_w = d[3]; }
    }
    printf("[input] %dx%dx%d (HxWxC), fmt=%d, type=%d\n", in_h, in_w, in_c, in_attr.fmt, in_attr.type);

    memset(&out_attr, 0, sizeof(out_attr)); out_attr.index = 0;
    rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &out_attr, sizeof(out_attr));
    printf("[output] dims=[%d,%d,%d,%d] type=%d\n",
           out_attr.dims[0], out_attr.dims[1], out_attr.dims[2], out_attr.dims[3], out_attr.type);

    input_buf = (uint8_t*)malloc((size_t)in_w * in_h * in_c);
    blank_idx = key_count;  /* c_size 应为 key_count+1=68, blank=67=key_count */

    /* 收集图像路径 */
    char **paths = NULL; int npath = 0, pcap = 0;
    for (i = 3; i < argc; i++) {
        if (argv[i][0] == '@') {
            FILE *lf = fopen(argv[i] + 1, "r"); char lb[1024];
            if (!lf) { fprintf(stderr, "[WARN] list 打不开: %s\n", argv[i]+1); continue; }
            while (fgets(lb, sizeof(lb), lf)) {
                size_t L = strlen(lb);
                while (L && (lb[L-1]=='\n'||lb[L-1]=='\r')) lb[--L]='\0';
                if (L == 0) continue;
                if (npath == pcap) { pcap = pcap?pcap*2:64; paths = realloc(paths, pcap*sizeof(char*)); }
                paths[npath++] = strdup(lb);
            }
            fclose(lf);
        } else {
            if (npath == pcap) { pcap = pcap?pcap*2:64; paths = realloc(paths, pcap*sizeof(char*)); }
            paths[npath++] = strdup(argv[i]);
        }
    }
    printf("[run] %d 张图\n\n", npath);

    int exact = 0, prov = 0, wan = 0, tot = 0;
    double tsum = 0.0; int tn = 0;

    for (i = 0; i < npath; i++) {
        uint8_t *src = NULL; int sw, sh;
        char *gt = NULL;
        char path[1024];

        /* 路径可能带 GT: "path|GT" */
        strncpy(path, paths[i], sizeof(path)-1); path[sizeof(path)-1]='\0';
        char *bar = strchr(path, '|');
        if (bar) { *bar = '\0'; gt = bar + 1; }

        if (read_ppm_rgb888(path, &src, &sw, &sh) < 0) {
            fprintf(stderr, "[WARN] 读图失败: %s\n", path); continue;
        }

        /* resize 到模型尺寸 (bilinear), RGB->BGR */
        resize_rgb888_bilinear(src, sw, sh, input_buf, in_w, in_h);
        rgb_to_bgr_inplace(input_buf, in_w, in_h);
        /* 可选 gray3 preproc (PREPROC=gray3 启用, B1 训练用) */
        {
            const char *pp = getenv("PREPROC");
            if (pp && strcmp(pp, "gray3") == 0) {
                apply_gray3_inplace(input_buf, in_w, in_h);
            }
        }
        free(src);

        /* 推理 */
        rknn_input in; memset(&in, 0, sizeof(in));
        in.index = 0;
        in.buf = input_buf;
        in.size = (size_t)in_w * in_h * in_c;
        in.type = RKNN_TENSOR_UINT8;
        in.fmt = RKNN_TENSOR_NHWC;

        rknn_inputs_set(ctx, 1, &in);

        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        ret = rknn_run(ctx, NULL);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double ms = (t1.tv_sec - t0.tv_sec)*1000.0 + (t1.tv_nsec - t0.tv_nsec)/1e6;
        tsum += ms; tn++;
        (void)ret;

        rknn_output out; memset(&out, 0, sizeof(out));
        out.want_float = 1;
        out.is_prealloc = 0;
        rknn_outputs_get(ctx, 1, &out, NULL);

        /* 解析 output shape -> t_size, c_size
         * 可能形态: [1,68,24](NCT) / [1,24,68](NTC) / [1,68,1,24](NCHW去H) / [1,1,68,24]
         * 策略: 在维度里找等于 key_count+1(或key_count) 的为 c_size, 另一个>1的为 t_size */
        int t_size = 0, c_size = 0;
        int *d = out_attr.n_dims >= 4 ? &out_attr.dims[out_attr.n_dims - 3] : &out_attr.dims[1];
        int nd = out_attr.n_dims >= 4 ? 3 : out_attr.n_dims - 1;
        for (int k = 0; k < nd; k++) {
            if (d[k] == key_count + 1 || d[k] == key_count) c_size = d[k];
            else if (d[k] > 1) t_size = d[k];
        }
        if (c_size == 0) c_size = key_count + 1;
        if (t_size == 0) t_size = out_attr.n_elems / c_size;
        if (c_size == key_count + 1) blank_idx = key_count;
        else blank_idx = c_size - 1;

        char text[64] = ""; float conf = 0; struct ocr_decode_diag diag;
        memset(&diag, 0, sizeof(diag));

        /* 第一帧 dump logits 诊断 (对比 simulator) */
        if (i == 0 && getenv("DUMP_LOGITS")) {
            const float *p = (const float*)out.buf;
            float mn = 1e30, mx = -1e30, sum = 0;
            int n = t_size * c_size;
            for (int k = 0; k < n; k++) { if (p[k]<mn) mn=p[k]; if (p[k]>mx) mx=p[k]; sum+=p[k]; }
            printf("[DUMP] t=%d c=%d n=%d min=%.3f max=%.3f mean=%.3f\n", t_size, c_size, n, mn, mx, sum/n);
            /* 板端 buffer 为 C外T内 [c][t]: 元素(c,t) = p[c*t_size + t] */
            printf("[DUMP] argmax per t: ");
            for (int t = 0; t < t_size && t < 24; t++) {
                int am = 0; float mv = -1e30;
                for (int c = 0; c < c_size; c++) { float v = p[c * t_size + t]; if (v > mv) { mv = v; am = c; } }
                printf("%d ", am);
            }
            printf("\n");
        }

        /* 板端 RKNN output buffer 为 C外T内 [c][t]: t_stride=1, c_stride=t_size */
        ocr_decode_logits((const float*)out.buf, t_size, c_size, 1, t_size,
                          (const char *const *)keys, key_count, blank_idx,
                          OCR_DECODE_FAMILY_NONE, text, sizeof(text), &conf, &diag);

        printf("[%3d] %s -> %-12s ", i, gt ? gt : "?", text);
        if (gt) {
            int ok = (strcmp(text, gt) == 0);
            /* 首字比较: UTF-8 汉字 3 字节 */
            int pok = (text[0] && strncmp(text, gt, 3) == 0);
            /* 皖首字检测: "皖" UTF-8 = E7 9A 97 */
            int iswan = (text[0] && (unsigned char)text[0] == 0xE7 &&
                         (unsigned char)text[1] == 0x9A && (unsigned char)text[2] == 0x97);
            printf("exact=%s prov=%s %s", ok?"Y":"N", pok?"Y":"N", iswan?"[皖]":"");
            if (ok) exact++; if (pok) prov++; tot++; if (iswan) wan++;
        }
        printf("  (%.1fms)\n", ms);

        rknn_outputs_release(ctx, 1, &out);
    }

    printf("\n=== 汇总 ===\n");
    if (tot > 0)
        printf("exact=%d/%d (%.1f%%)  prov=%d/%d  皖首字=%d/%d\n",
               exact, tot, 100.0*exact/tot, prov, tot, wan, tot);
    if (tn > 0)
        printf("rknn_run 平均: %.1f ms/帧 (%d 帧)\n", tsum/tn, tn);

    rknn_destroy(ctx);
    free(model_buf); free(input_buf);
    for (i = 0; i < key_count; i++) free(keys[i]); free(keys);
    for (i = 0; i < npath; i++) free(paths[i]); free(paths);
    return 0;
}
