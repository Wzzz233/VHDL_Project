#define _POSIX_C_SOURCE 200809L
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <rknn_api.h>

#define MAX_OCR_KEYS 128
#define MAX_OCR_KEY_LEN 16
#define MAX_TEXT_LEN 64
#define MAX_PATH_LEN 1024
#define MAX_LINE_LEN 8192
#define MAX_FIELDS 64

struct ocr_diag {
    int t_size;
    int c_size;
    int blank_idx;
    float blank_top1_ratio;
};

struct ocr_model {
    const char *name;
    const char *path;
    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr input_attr;
    rknn_tensor_attr output_attrs[4];
    uint32_t in_w;
    uint32_t in_h;
    uint32_t in_c;
};

struct batch_ctx {
    struct ocr_model ocr_model;
    char ocr_keys[MAX_OCR_KEYS][MAX_OCR_KEY_LEN];
    int ocr_key_count;
    int ocr_blank_index;
    bool ocr_keysize_warned;
};

struct csv_header {
    int sample_id_idx;
    int preproc_mode_idx;
    int plate_text_idx;
    int preproc_ms_idx;
    int image_path_idx;
    int scenario_idx;
    int ocrin_path_idx;
};

struct sample_row {
    char sample_id[128];
    char preproc_mode[64];
    char plate_text_gt[64];
    double preproc_ms;
    char image_path[MAX_PATH_LEN];
    char scenario[128];
    char ocrin_path[MAX_PATH_LEN];
};

static void print_usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s --index <index.csv> --model <ocr.rknn> --ocr-keys <keys.txt> --out-csv <pred.csv>\n",
            prog);
}

static int64_t mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + (int64_t)(ts.tv_nsec / 1000L);
}

static void csv_safe_text(const char *in, char *out, size_t out_len)
{
    size_t i = 0;
    if (out_len == 0)
        return;
    if (!in) {
        out[0] = '\0';
        return;
    }
    while (*in && i + 1 < out_len) {
        char ch = *in++;
        if (ch == ',' || ch == '\n' || ch == '\r')
            ch = '_';
        out[i++] = ch;
    }
    out[i] = '\0';
}

static void copy_field(char *dst, size_t dst_sz, const char *src)
{
    size_t n;
    if (!dst || dst_sz == 0)
        return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    while (*src == ' ' || *src == '\t')
        src++;
    n = strcspn(src, "\r\n");
    if (n >= dst_sz)
        n = dst_sz - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static int split_csv_simple(char *line, char **fields, int max_fields)
{
    int count = 0;
    char *p;
    if (!line || !fields || max_fields <= 0)
        return 0;
    p = line;
    fields[count++] = p;
    while (*p && count < max_fields) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
        p++;
    }
    return count;
}

static int find_col(char **fields, int n, const char *name)
{
    int i;
    for (i = 0; i < n; i++) {
        char *s;
        if (!fields[i])
            continue;
        s = fields[i];
        s[strcspn(s, "\r\n")] = '\0';
        while (*s == ' ' || *s == '\t')
            s++;
        if (strcmp(s, name) == 0)
            return i;
    }
    return -1;
}

static void init_csv_header(struct csv_header *hdr)
{
    memset(hdr, 0xff, sizeof(*hdr));
}

static int parse_csv_header(char *line, struct csv_header *hdr)
{
    char *fields[MAX_FIELDS];
    int n;
    init_csv_header(hdr);
    n = split_csv_simple(line, fields, MAX_FIELDS);
    hdr->sample_id_idx = find_col(fields, n, "sample_id");
    hdr->preproc_mode_idx = find_col(fields, n, "preproc_mode");
    hdr->plate_text_idx = find_col(fields, n, "plate_text");
    hdr->preproc_ms_idx = find_col(fields, n, "preproc_ms");
    hdr->image_path_idx = find_col(fields, n, "image_path");
    hdr->scenario_idx = find_col(fields, n, "scenario");
    hdr->ocrin_path_idx = find_col(fields, n, "ocrin_path");

    if (hdr->sample_id_idx < 0 || hdr->preproc_mode_idx < 0 || hdr->plate_text_idx < 0 ||
        hdr->preproc_ms_idx < 0 || hdr->image_path_idx < 0 || hdr->scenario_idx < 0 ||
        hdr->ocrin_path_idx < 0) {
        fprintf(stderr,
                "index.csv missing required columns: sample_id/preproc_mode/plate_text/preproc_ms/image_path/scenario/ocrin_path\n");
        return -1;
    }
    return 0;
}

static int read_next_sample(FILE *fp, const struct csv_header *hdr, struct sample_row *row)
{
    char line[MAX_LINE_LEN];
    while (fgets(line, sizeof(line), fp)) {
        char *fields[MAX_FIELDS];
        int n = split_csv_simple(line, fields, MAX_FIELDS);
        if (n <= 1)
            continue;
        memset(row, 0, sizeof(*row));
        copy_field(row->sample_id, sizeof(row->sample_id), (hdr->sample_id_idx < n) ? fields[hdr->sample_id_idx] : "");
        copy_field(row->preproc_mode, sizeof(row->preproc_mode), (hdr->preproc_mode_idx < n) ? fields[hdr->preproc_mode_idx] : "");
        copy_field(row->plate_text_gt, sizeof(row->plate_text_gt), (hdr->plate_text_idx < n) ? fields[hdr->plate_text_idx] : "");
        copy_field(row->image_path, sizeof(row->image_path), (hdr->image_path_idx < n) ? fields[hdr->image_path_idx] : "");
        copy_field(row->scenario, sizeof(row->scenario), (hdr->scenario_idx < n) ? fields[hdr->scenario_idx] : "");
        copy_field(row->ocrin_path, sizeof(row->ocrin_path), (hdr->ocrin_path_idx < n) ? fields[hdr->ocrin_path_idx] : "");
        if (hdr->preproc_ms_idx < n && fields[hdr->preproc_ms_idx] && fields[hdr->preproc_ms_idx][0] != '\0')
            row->preproc_ms = atof(fields[hdr->preproc_ms_idx]);
        else
            row->preproc_ms = 0.0;
        return 1;
    }
    return 0;
}

static int load_ocr_keys(struct batch_ctx *ctx, const char *path)
{
    FILE *fp = fopen(path, "r");
    char line[256];
    int idx = 0;
    if (!fp) {
        fprintf(stderr, "Open OCR keys failed: %s\n", strerror(errno));
        return -1;
    }
    while (fgets(line, sizeof(line), fp) && idx < MAX_OCR_KEYS) {
        char *nl = strchr(line, '\n');
        size_t n;
        if (nl)
            *nl = '\0';
        nl = strchr(line, '\r');
        if (nl)
            *nl = '\0';
        if (line[0] == '\0' || line[0] == '#')
            continue;
        n = strnlen(line, MAX_OCR_KEY_LEN - 1);
        memcpy(ctx->ocr_keys[idx], line, n);
        ctx->ocr_keys[idx][n] = '\0';
        idx++;
    }
    fclose(fp);
    if (idx <= 0)
        return -1;
    ctx->ocr_key_count = idx;
    ctx->ocr_blank_index = idx;
    fprintf(stderr, "[ocr-batch] loaded %d keys from %s\n", ctx->ocr_key_count, path);
    return 0;
}

static int rknn_ocr_model_load(struct ocr_model *m, const char *name, const char *path)
{
    FILE *fp;
    long sz;
    void *data;
    uint32_t i;

    memset(m, 0, sizeof(*m));
    m->name = name;
    m->path = path;

    fp = fopen(path, "rb");
    if (!fp)
        return -1;
    fseek(fp, 0, SEEK_END);
    sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    data = malloc((size_t)sz);
    if (!data) {
        fclose(fp);
        return -1;
    }
    if (fread(data, 1, (size_t)sz, fp) != (size_t)sz) {
        fclose(fp);
        free(data);
        return -1;
    }
    fclose(fp);

    if (rknn_init(&m->ctx, data, (uint32_t)sz, 0, NULL) < 0) {
        free(data);
        return -1;
    }
    free(data);
    if (rknn_query(m->ctx, RKNN_QUERY_IN_OUT_NUM, &m->io_num, sizeof(m->io_num)) < 0)
        return -1;
    if (m->io_num.n_output == 0 || m->io_num.n_output > 4)
        return -1;

    memset(&m->input_attr, 0, sizeof(m->input_attr));
    m->input_attr.index = 0;
    if (rknn_query(m->ctx, RKNN_QUERY_INPUT_ATTR, &m->input_attr, sizeof(m->input_attr)) < 0)
        return -1;
    if (m->input_attr.fmt == RKNN_TENSOR_NCHW) {
        m->in_c = m->input_attr.dims[1];
        m->in_h = m->input_attr.dims[2];
        m->in_w = m->input_attr.dims[3];
    } else {
        m->in_h = m->input_attr.dims[1];
        m->in_w = m->input_attr.dims[2];
        m->in_c = m->input_attr.dims[3];
    }

    for (i = 0; i < m->io_num.n_output; i++) {
        memset(&m->output_attrs[i], 0, sizeof(m->output_attrs[i]));
        m->output_attrs[i].index = i;
        if (rknn_query(m->ctx, RKNN_QUERY_OUTPUT_ATTR, &m->output_attrs[i], sizeof(m->output_attrs[i])) < 0)
            return -1;
    }

    fprintf(stderr, "[ocr-batch] model=%s input=%ux%ux%u outputs=%u\n",
            path, m->in_w, m->in_h, m->in_c, m->io_num.n_output);
    return 0;
}

static void rknn_ocr_model_release(struct ocr_model *m)
{
    if (m->ctx)
        rknn_destroy(m->ctx);
    memset(m, 0, sizeof(*m));
}

static bool build_ocr_layout(const rknn_tensor_attr *a, int *t_size, int *c_size, int *t_stride, int *c_stride)
{
    if (a->n_dims == 2) {
        *t_size = (int)a->dims[0];
        *c_size = (int)a->dims[1];
        *t_stride = *c_size;
        *c_stride = 1;
        return (*t_size > 0 && *c_size > 1);
    }
    if (a->n_dims == 3) {
        int d1 = (int)a->dims[1];
        int d2 = (int)a->dims[2];
        if (d1 <= 0 || d2 <= 1)
            return false;
        if (d1 <= d2) {
            *t_size = d1;
            *c_size = d2;
            *t_stride = *c_size;
            *c_stride = 1;
        } else {
            *t_size = d2;
            *c_size = d1;
            *t_stride = 1;
            *c_stride = *t_size;
        }
        return true;
    }
    if (a->n_dims == 4) {
        if (a->fmt == RKNN_TENSOR_NCHW) {
            *c_size = (int)a->dims[1];
            *t_size = (int)a->dims[2] * (int)a->dims[3];
            *t_stride = 1;
            *c_stride = *t_size;
        } else {
            *t_size = (int)a->dims[1] * (int)a->dims[2];
            *c_size = (int)a->dims[3];
            *t_stride = *c_size;
            *c_stride = 1;
        }
        return (*t_size > 0 && *c_size > 1);
    }
    return false;
}

static int ctc_decode_logits(const float *buf, int t_size, int c_size, int t_stride, int c_stride,
                             struct batch_ctx *ctx, char *text, size_t text_len, float *conf_out,
                             struct ocr_diag *diag)
{
    int t;
    int prev = -1;
    int emitted = 0;
    float conf_sum = 0.0f;
    int blank_idx = ctx->ocr_blank_index;
    int blank_top1_count = 0;

    if (text_len == 0)
        return -1;
    text[0] = '\0';

    if (blank_idx < 0 || blank_idx >= c_size) {
        if (c_size == ctx->ocr_key_count + 1)
            blank_idx = ctx->ocr_key_count;
        else
            blank_idx = c_size - 1;
    }

    for (t = 0; t < t_size; t++) {
        int c;
        int best_c = 0;
        float best_logit = -1e30f;
        float max_logit = -1e30f;
        float exp_sum = 0.0f;
        const float *row = buf + (size_t)t * (size_t)t_stride;

        for (c = 0; c < c_size; c++) {
            float v = row[(size_t)c * (size_t)c_stride];
            if (v > best_logit) {
                best_logit = v;
                best_c = c;
            }
            if (v > max_logit)
                max_logit = v;
        }
        for (c = 0; c < c_size; c++) {
            float v = row[(size_t)c * (size_t)c_stride];
            exp_sum += expf(v - max_logit);
        }
        if (best_c == blank_idx)
            blank_top1_count++;
        if (best_c == blank_idx || best_c == prev) {
            prev = best_c;
            continue;
        }
        if (best_c >= 0 && best_c < ctx->ocr_key_count) {
            float prob;
            size_t left;
            exp_sum = (exp_sum > 1e-8f) ? exp_sum : 1e-8f;
            prob = expf(best_logit - max_logit) / exp_sum;
            left = text_len - strnlen(text, text_len);
            if (left > 1) {
                strncat(text, ctx->ocr_keys[best_c], left - 1);
                emitted++;
                conf_sum += prob;
            }
        }
        prev = best_c;
    }
    *conf_out = (emitted > 0) ? (conf_sum / (float)emitted) : 0.0f;
    if (diag) {
        diag->t_size = t_size;
        diag->c_size = c_size;
        diag->blank_idx = blank_idx;
        diag->blank_top1_ratio = (t_size > 0) ? ((float)blank_top1_count / (float)t_size) : 0.0f;
    }
    return 0;
}

static int ppm_next_token(FILE *fp, char *tok, size_t tok_sz)
{
    int ch;
    size_t n = 0;

    if (!fp || !tok || tok_sz == 0)
        return -1;
    tok[0] = '\0';

    while (1) {
        ch = fgetc(fp);
        if (ch == '#') {
            while (ch != '\n' && ch != EOF)
                ch = fgetc(fp);
            continue;
        }
        if (ch == EOF)
            return -1;
        if (!isspace((unsigned char)ch))
            break;
    }

    while (ch != EOF && !isspace((unsigned char)ch) && ch != '#') {
        if (n + 1 < tok_sz)
            tok[n++] = (char)ch;
        ch = fgetc(fp);
    }
    tok[n] = '\0';

    if (ch == '#') {
        while (ch != '\n' && ch != EOF)
            ch = fgetc(fp);
    }
    return (n > 0) ? 0 : -1;
}

static int read_ppm_rgb888(const char *path, uint8_t **rgb_out, int *w_out, int *h_out)
{
    FILE *fp = NULL;
    char tok[64];
    int w, h, maxv;
    int ch;
    uint8_t *buf = NULL;
    size_t need;
    size_t got;

    if (!path || !rgb_out || !w_out || !h_out)
        return -1;
    *rgb_out = NULL;
    *w_out = 0;
    *h_out = 0;

    fp = fopen(path, "rb");
    if (!fp)
        return -1;

    if (ppm_next_token(fp, tok, sizeof(tok)) < 0 || strcmp(tok, "P6") != 0)
        goto fail;
    if (ppm_next_token(fp, tok, sizeof(tok)) < 0)
        goto fail;
    w = atoi(tok);
    if (ppm_next_token(fp, tok, sizeof(tok)) < 0)
        goto fail;
    h = atoi(tok);
    if (ppm_next_token(fp, tok, sizeof(tok)) < 0)
        goto fail;
    maxv = atoi(tok);
    if (w <= 0 || h <= 0 || maxv != 255)
        goto fail;
    do {
        ch = fgetc(fp);
    } while (ch != EOF && isspace((unsigned char)ch));
    if (ch == EOF)
        goto fail;
    if (ungetc(ch, fp) == EOF)
        goto fail;

    need = (size_t)w * (size_t)h * 3U;
    buf = (uint8_t *)malloc(need);
    if (!buf)
        goto fail;
    got = fread(buf, 1, need, fp);
    if (got != need)
        goto fail;

    fclose(fp);
    *rgb_out = buf;
    *w_out = w;
    *h_out = h;
    return 0;

fail:
    if (fp)
        fclose(fp);
    free(buf);
    return -1;
}

static int run_model_ocr_input(struct batch_ctx *ctx, const uint8_t *ocr_in, int in_w, int in_h,
                               char *text, size_t text_len, float *conf_out,
                               struct ocr_diag *diag, double *infer_ms)
{
    struct ocr_model *m = &ctx->ocr_model;
    rknn_input in;
    rknn_output outs[4];
    const rknn_tensor_attr *out_attr;
    int t_size;
    int c_size;
    int t_stride;
    int c_stride;
    int ret = -1;
    uint32_t i;
    int64_t t0;
    int64_t t1;

    if (!ocr_in || !text || !conf_out)
        return -1;
    if (in_w != (int)m->in_w || in_h != (int)m->in_h) {
        fprintf(stderr,
                "[ocr-batch] input size mismatch: got=%dx%d expect=%ux%u\n",
                in_w, in_h, m->in_w, m->in_h);
        return -1;
    }
    if (diag)
        memset(diag, 0, sizeof(*diag));

    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.buf = (void *)ocr_in;
    in.size = m->in_w * m->in_h * 3U;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;

    t0 = mono_us();
    ret = rknn_inputs_set(m->ctx, 1, &in);
    if (ret < 0)
        return ret;
    ret = rknn_run(m->ctx, NULL);
    if (ret < 0)
        return ret;

    memset(outs, 0, sizeof(outs));
    for (i = 0; i < m->io_num.n_output; i++)
        outs[i].want_float = 1;
    ret = rknn_outputs_get(m->ctx, m->io_num.n_output, outs, NULL);
    if (ret < 0)
        return ret;
    t1 = mono_us();
    if (infer_ms)
        *infer_ms = (double)(t1 - t0) / 1000.0;

    out_attr = &m->output_attrs[0];
    if (!build_ocr_layout(out_attr, &t_size, &c_size, &t_stride, &c_stride)) {
        ret = -1;
        goto out_release;
    }
    if (ctx->ocr_blank_index < 0 || ctx->ocr_blank_index >= c_size) {
        if (c_size == ctx->ocr_key_count + 1)
            ctx->ocr_blank_index = ctx->ocr_key_count;
        else
            ctx->ocr_blank_index = c_size - 1;
    }
    if (!ctx->ocr_keysize_warned) {
        if (!(c_size == ctx->ocr_key_count || c_size == (ctx->ocr_key_count + 1))) {
            fprintf(stderr,
                    "[ocr-batch] WARN key/output mismatch: keys=%d c_size=%d\n",
                    ctx->ocr_key_count, c_size);
        }
        ctx->ocr_keysize_warned = true;
    }
    ret = ctc_decode_logits((const float *)outs[0].buf, t_size, c_size, t_stride, c_stride,
                            ctx, text, text_len, conf_out, diag);

out_release:
    rknn_outputs_release(m->ctx, m->io_num.n_output, outs);
    return ret;
}

static int run_model_ocr_path(struct batch_ctx *ctx, const char *ppm_path,
                              char *text, size_t text_len, float *conf_out,
                              struct ocr_diag *diag, double *infer_ms)
{
    uint8_t *rgb = NULL;
    int w = 0;
    int h = 0;
    int ret;

    ret = read_ppm_rgb888(ppm_path, &rgb, &w, &h);
    if (ret < 0) {
        fprintf(stderr, "[ocr-batch] failed to read %s\n", ppm_path);
        return ret;
    }
    ret = run_model_ocr_input(ctx, rgb, w, h, text, text_len, conf_out, diag, infer_ms);
    free(rgb);
    return ret;
}

int main(int argc, char **argv)
{
    const char *index_path = NULL;
    const char *model_path = NULL;
    const char *keys_path = NULL;
    const char *out_csv = NULL;
    struct batch_ctx ctx;
    FILE *fp_idx = NULL;
    FILE *fp_out = NULL;
    char header_line[MAX_LINE_LEN];
    struct csv_header hdr;
    int processed = 0;
    int failures = 0;
    int i;

    memset(&ctx, 0, sizeof(ctx));
    ctx.ocr_blank_index = -1;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--index") == 0 && i + 1 < argc) {
            index_path = argv[++i];
        } else if (strcmp(argv[i], "--model") == 0 && i + 1 < argc) {
            model_path = argv[++i];
        } else if (strcmp(argv[i], "--ocr-keys") == 0 && i + 1 < argc) {
            keys_path = argv[++i];
        } else if (strcmp(argv[i], "--out-csv") == 0 && i + 1 < argc) {
            out_csv = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown arg: %s\n", argv[i]);
            print_usage(argv[0]);
            return 2;
        }
    }

    if (!index_path || !model_path || !keys_path || !out_csv) {
        print_usage(argv[0]);
        return 2;
    }

    if (load_ocr_keys(&ctx, keys_path) < 0) {
        fprintf(stderr, "Failed to load OCR keys: %s\n", keys_path);
        return 3;
    }
    if (rknn_ocr_model_load(&ctx.ocr_model, "ocr", model_path) < 0) {
        fprintf(stderr, "Failed to load OCR model: %s\n", model_path);
        return 3;
    }

    fp_idx = fopen(index_path, "r");
    if (!fp_idx) {
        fprintf(stderr, "Open index failed: %s\n", strerror(errno));
        rknn_ocr_model_release(&ctx.ocr_model);
        return 4;
    }
    fp_out = fopen(out_csv, "w");
    if (!fp_out) {
        fprintf(stderr, "Open output failed: %s\n", strerror(errno));
        fclose(fp_idx);
        rknn_ocr_model_release(&ctx.ocr_model);
        return 4;
    }

    if (!fgets(header_line, sizeof(header_line), fp_idx)) {
        fprintf(stderr, "Empty index csv: %s\n", index_path);
        fclose(fp_out);
        fclose(fp_idx);
        rknn_ocr_model_release(&ctx.ocr_model);
        return 4;
    }
    if (parse_csv_header(header_line, &hdr) < 0) {
        fclose(fp_out);
        fclose(fp_idx);
        rknn_ocr_model_release(&ctx.ocr_model);
        return 4;
    }

    fprintf(fp_out,
            "sample_id,preproc_mode,plate_text_gt,plate_text_pred,conf,blank_top1,preproc_ms,infer_ms,image_path,scenario\n");

    while (1) {
        struct sample_row row;
        char pred_text[MAX_TEXT_LEN];
        char safe_sample_id[128];
        char safe_mode[64];
        char safe_gt[64];
        char safe_pred[MAX_TEXT_LEN];
        char safe_img[MAX_PATH_LEN];
        char safe_scenario[128];
        char conf_buf[32] = {0};
        char blank_buf[32] = {0};
        struct ocr_diag diag;
        float conf = 0.0f;
        double infer_ms = 0.0;
        int ret;
        int got = read_next_sample(fp_idx, &hdr, &row);
        if (got <= 0)
            break;

        memset(&diag, 0, sizeof(diag));
        pred_text[0] = '\0';
        ret = run_model_ocr_path(&ctx, row.ocrin_path, pred_text, sizeof(pred_text), &conf, &diag, &infer_ms);
        if (ret < 0) {
            failures++;
            pred_text[0] = '\0';
        } else {
            snprintf(conf_buf, sizeof(conf_buf), "%.4f", conf);
            snprintf(blank_buf, sizeof(blank_buf), "%.4f", diag.blank_top1_ratio);
        }

        csv_safe_text(row.sample_id, safe_sample_id, sizeof(safe_sample_id));
        csv_safe_text(row.preproc_mode, safe_mode, sizeof(safe_mode));
        csv_safe_text(row.plate_text_gt, safe_gt, sizeof(safe_gt));
        csv_safe_text(pred_text, safe_pred, sizeof(safe_pred));
        csv_safe_text(row.image_path, safe_img, sizeof(safe_img));
        csv_safe_text(row.scenario, safe_scenario, sizeof(safe_scenario));

        fprintf(fp_out, "%s,%s,%s,%s,%s,%s,%.3f,%.3f,%s,%s\n",
                safe_sample_id,
                safe_mode,
                safe_gt,
                safe_pred,
                conf_buf,
                blank_buf,
                row.preproc_ms,
                infer_ms,
                safe_img,
                safe_scenario);
        processed++;
    }

    fclose(fp_out);
    fclose(fp_idx);
    rknn_ocr_model_release(&ctx.ocr_model);
    fprintf(stderr, "[ocr-batch] processed=%d failures=%d out=%s\n", processed, failures, out_csv);
    return (failures == 0) ? 0 : 1;
}
