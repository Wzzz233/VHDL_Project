#include "ocr_decode.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OCR_DECODE_MAX_TEXT 64
#define OCR_DECODE_MAX_TOKENS 16
#define OCR_DECODE_TOPK 12
#define OCR_DECODE_BEAM_SIZE 20
#define OCR_DECODE_MAX_BEAMS 320

struct beam_state {
    int token_ids[OCR_DECODE_MAX_TOKENS];
    int token_count;
    char text[OCR_DECODE_MAX_TEXT];
    double pb;
    double pnb;
};

static double logaddexp2(double a, double b)
{
    double m;
    if (a == -INFINITY)
        return b;
    if (b == -INFINITY)
        return a;
    m = (a > b) ? a : b;
    return m + log(exp(a - m) + exp(b - m));
}

static size_t strnlen_local(const char *s, size_t max_len)
{
    size_t n = 0;
    if (!s)
        return 0;
    while (n < max_len && s[n] != '\0')
        n++;
    return n;
}

static bool append_utf8_token_local(char *dst, size_t dst_len, const char *token)
{
    size_t cur;
    size_t tok_len;

    if (!dst || dst_len == 0 || !token)
        return false;
    cur = strnlen_local(dst, dst_len);
    if (cur >= dst_len - 1)
        return false;
    tok_len = strnlen_local(token, 31);
    if (tok_len == 0 || tok_len >= dst_len - cur)
        return false;
    memcpy(dst + cur, token, tok_len);
    dst[cur + tok_len] = '\0';
    return true;
}

static bool token_equals(const char *a, const char *b)
{
    return a && b && strcmp(a, b) == 0;
}

static bool token_in_list(const char *token, const char *const *list, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        if (token_equals(token, list[i]))
            return true;
    }
    return false;
}

static bool token_is_province(const char *token)
{
    static const char *const provinces[] = {
        "京", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑",
        "苏", "浙", "皖", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤",
        "桂", "琼", "川", "贵", "云", "藏", "陕", "甘", "青", "宁",
        "新"
    };
    return token_in_list(token, provinces, (int)(sizeof(provinces) / sizeof(provinces[0])));
}

static bool token_is_alpha(const char *token)
{
    static const char *const alpha[] = {
        "A", "B", "C", "D", "E", "F", "G", "H", "J", "K", "L", "M", "N",
        "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"
    };
    return token_in_list(token, alpha, (int)(sizeof(alpha) / sizeof(alpha[0])));
}

static bool token_is_alnum(const char *token)
{
    static const char *const alnum[] = {
        "A", "B", "C", "D", "E", "F", "G", "H", "J", "K", "L", "M", "N",
        "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z",
        "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"
    };
    return token_in_list(token, alnum, (int)(sizeof(alnum) / sizeof(alnum[0])));
}

static bool token_is_digit(const char *token)
{
    static const char *const digits[] = {
        "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"
    };
    return token_in_list(token, digits, (int)(sizeof(digits) / sizeof(digits[0])));
}

static bool family_prefix_valid(enum ocr_decode_family family,
                                const int *token_ids, int token_count,
                                const char *const *keys, int key_count)
{
    int i;
    if (family == OCR_DECODE_FAMILY_NONE)
        return true;
    if (token_count == 0)
        return true;
    for (i = 0; i < token_count; i++) {
        if (token_ids[i] < 0 || token_ids[i] >= key_count)
            return false;
    }
    if (family == OCR_DECODE_FAMILY_EMBASSY7) {
        if (token_count > 7)
            return false;
        if (!token_equals(keys[token_ids[0]], "使"))
            return false;
        for (i = 1; i < token_count; i++) {
            if (!token_is_digit(keys[token_ids[i]]))
                return false;
        }
        return true;
    }
    if (!token_is_province(keys[token_ids[0]]))
        return false;
    if (token_count >= 2 && !token_is_alpha(keys[token_ids[1]]))
        return false;
    if (family == OCR_DECODE_FAMILY_GREEN8) {
        if (token_count > 8)
            return false;
        for (i = 2; i < token_count; i++) {
            if (!token_is_alnum(keys[token_ids[i]]))
                return false;
        }
        return true;
    }
    if (family == OCR_DECODE_FAMILY_NORMAL7) {
        if (token_count > 7)
            return false;
        for (i = 2; i < token_count; i++) {
            if (!token_is_alnum(keys[token_ids[i]]))
                return false;
        }
        return true;
    }
    if (family == OCR_DECODE_FAMILY_POLICE7) {
        if (token_count > 7)
            return false;
        for (i = 2; i < token_count && i < 6; i++) {
            if (!token_is_alnum(keys[token_ids[i]]))
                return false;
        }
        if (token_count >= 7 && !token_equals(keys[token_ids[6]], "警"))
            return false;
        return true;
    }
    return true;
}

static int family_max_token_count(enum ocr_decode_family family)
{
    if (family == OCR_DECODE_FAMILY_GREEN8)
        return 8;
    if (family == OCR_DECODE_FAMILY_NORMAL7 ||
        family == OCR_DECODE_FAMILY_POLICE7 ||
        family == OCR_DECODE_FAMILY_EMBASSY7)
        return 7;
    return OCR_DECODE_MAX_TOKENS;
}

static bool family_full_valid(enum ocr_decode_family family,
                              const int *token_ids, int token_count,
                              const char *const *keys, int key_count)
{
    if (!family_prefix_valid(family, token_ids, token_count, keys, key_count))
        return false;
    if (family == OCR_DECODE_FAMILY_GREEN8)
        return token_count == 8;
    if (family == OCR_DECODE_FAMILY_NORMAL7)
        return token_count == 7;
    if (family == OCR_DECODE_FAMILY_POLICE7)
        return token_count == 7;
    if (family == OCR_DECODE_FAMILY_EMBASSY7)
        return token_count == 7;
    return true;
}

static int beam_state_find(struct beam_state *states, int count,
                           const int *token_ids, int token_count)
{
    int i;
    for (i = 0; i < count; i++) {
        if (states[i].token_count != token_count)
            continue;
        if (token_count == 0)
            return i;
        if (memcmp(states[i].token_ids, token_ids, (size_t)token_count * sizeof(int)) == 0)
            return i;
    }
    return -1;
}

static int beam_state_upsert(struct beam_state *states, int *count,
                             const int *token_ids, int token_count,
                             const char *const *keys)
{
    int idx;
    idx = beam_state_find(states, *count, token_ids, token_count);
    if (idx >= 0)
        return idx;
    if (*count >= OCR_DECODE_MAX_BEAMS)
        return -1;
    idx = *count;
    (*count)++;
    memset(&states[idx], 0, sizeof(states[idx]));
    if (token_count > 0)
        memcpy(states[idx].token_ids, token_ids, (size_t)token_count * sizeof(int));
    states[idx].token_count = token_count;
    states[idx].pb = -INFINITY;
    states[idx].pnb = -INFINITY;
    states[idx].text[0] = '\0';
    for (int i = 0; i < token_count; i++) {
        if (!append_utf8_token_local(states[idx].text, sizeof(states[idx].text), keys[token_ids[i]]))
            return -1;
    }
    return idx;
}

static void pick_topk(const double *log_probs, int c_size, int blank_idx, int *idxs, int *idx_count)
{
    double best_val[OCR_DECODE_TOPK];
    int best_idx[OCR_DECODE_TOPK];
    int i, j, n = 0;
    bool have_blank = false;

    for (i = 0; i < OCR_DECODE_TOPK; i++) {
        best_val[i] = -INFINITY;
        best_idx[i] = -1;
    }
    for (i = 0; i < c_size; i++) {
        double v = log_probs[i];
        int pos = -1;
        for (j = 0; j < OCR_DECODE_TOPK; j++) {
            if (v > best_val[j]) {
                pos = j;
                break;
            }
        }
        if (pos < 0)
            continue;
        for (j = OCR_DECODE_TOPK - 1; j > pos; j--) {
            best_val[j] = best_val[j - 1];
            best_idx[j] = best_idx[j - 1];
        }
        best_val[pos] = v;
        best_idx[pos] = i;
    }
    for (i = 0; i < OCR_DECODE_TOPK; i++) {
        if (best_idx[i] < 0)
            continue;
        idxs[n++] = best_idx[i];
        if (best_idx[i] == blank_idx)
            have_blank = true;
    }
    if (!have_blank && blank_idx >= 0 && blank_idx < c_size)
        idxs[n++] = blank_idx;
    *idx_count = n;
}

static int compare_beam_desc(const void *pa, const void *pb)
{
    const struct beam_state *a = (const struct beam_state *)pa;
    const struct beam_state *b = (const struct beam_state *)pb;
    double sa = logaddexp2(a->pb, a->pnb);
    double sb = logaddexp2(b->pb, b->pnb);
    if (sa < sb)
        return 1;
    if (sa > sb)
        return -1;
    return 0;
}

static int constrained_decode(const float *buf, int t_size, int c_size, int t_stride, int c_stride,
                              const char *const *keys, int key_count, int blank_idx,
                              enum ocr_decode_family family,
                              char *text, size_t text_len)
{
    struct beam_state beams[OCR_DECODE_MAX_BEAMS];
    struct beam_state next_beams[OCR_DECODE_MAX_BEAMS];
    double *log_probs = NULL;
    int beam_count = 1;
    int t;
    int ret = -1;

    if (key_count <= 0 || blank_idx < 0 || blank_idx >= c_size)
        return -1;

    log_probs = (double *)malloc((size_t)c_size * sizeof(double));
    if (!log_probs)
        return -1;
    memset(beams, 0, sizeof(beams));
    beams[0].pb = 0.0;
    beams[0].pnb = -INFINITY;
    beams[0].token_count = 0;
    beams[0].text[0] = '\0';

    for (t = 0; t < t_size; t++) {
        const float *row = buf + (size_t)t * (size_t)t_stride;
        float max_logit = -1e30f;
        double exp_sum = 0.0;
        int idxs[OCR_DECODE_TOPK + 1];
        int idx_count = 0;
        int b;
        int next_count = 0;

        for (int c = 0; c < c_size; c++) {
            float v = row[(size_t)c * (size_t)c_stride];
            if (v > max_logit)
                max_logit = v;
        }
        for (int c = 0; c < c_size; c++) {
            float v = row[(size_t)c * (size_t)c_stride];
            exp_sum += exp((double)v - (double)max_logit);
        }
        if (exp_sum < 1e-12)
            exp_sum = 1e-12;
        for (int c = 0; c < c_size; c++) {
            float v = row[(size_t)c * (size_t)c_stride];
            log_probs[c] = (double)v - (double)max_logit - log(exp_sum);
        }
        pick_topk(log_probs, c_size, blank_idx, idxs, &idx_count);

        for (b = 0; b < beam_count; b++) {
            int blank_state;
            double total_score = logaddexp2(beams[b].pb, beams[b].pnb);
            blank_state = beam_state_upsert(next_beams, &next_count,
                                            beams[b].token_ids, beams[b].token_count, keys);
            if (blank_state < 0)
                goto cleanup;
            next_beams[blank_state].pb = logaddexp2(next_beams[blank_state].pb,
                                                    total_score + log_probs[blank_idx]);

            for (int i = 0; i < idx_count; i++) {
                int c = idxs[i];
                int new_ids[OCR_DECODE_MAX_TOKENS];
                int new_count;
                int state_idx;
                bool same_as_last;
                double score;

                if (c == blank_idx)
                    continue;
                new_count = beams[b].token_count + 1;
                if (new_count > OCR_DECODE_MAX_TOKENS)
                    continue;
                if (beams[b].token_count > 0)
                    memcpy(new_ids, beams[b].token_ids, (size_t)beams[b].token_count * sizeof(int));
                new_ids[new_count - 1] = c;
                if (!family_prefix_valid(family, new_ids, new_count, keys, key_count))
                    continue;
                state_idx = beam_state_upsert(next_beams, &next_count, new_ids, new_count, keys);
                if (state_idx < 0)
                    goto cleanup;
                same_as_last = (beams[b].token_count > 0 && beams[b].token_ids[beams[b].token_count - 1] == c);
                if (same_as_last)
                    score = beams[b].pb + log_probs[c];
                else
                    score = logaddexp2(beams[b].pb, beams[b].pnb) + log_probs[c];
                next_beams[state_idx].pnb = logaddexp2(next_beams[state_idx].pnb, score);
                if (same_as_last) {
                    int repeat_state = beam_state_upsert(next_beams, &next_count,
                                                         beams[b].token_ids, beams[b].token_count, keys);
                    if (repeat_state < 0)
                        goto cleanup;
                    next_beams[repeat_state].pnb = logaddexp2(next_beams[repeat_state].pnb,
                                                              beams[b].pnb + log_probs[c]);
                }
            }
        }

        qsort(next_beams, (size_t)next_count, sizeof(next_beams[0]), compare_beam_desc);
        if (family != OCR_DECODE_FAMILY_NONE) {
            bool picked[OCR_DECODE_MAX_BEAMS];
            int selected = 0;
            int max_tokens = family_max_token_count(family);
            memset(picked, 0, sizeof(picked));
            for (int len = 0; len <= max_tokens && selected < OCR_DECODE_BEAM_SIZE; len++) {
                for (int i = 0; i < next_count; i++) {
                    if (!picked[i] && next_beams[i].token_count == len) {
                        beams[selected++] = next_beams[i];
                        picked[i] = true;
                        break;
                    }
                }
            }
            for (int i = 0; i < next_count && selected < OCR_DECODE_BEAM_SIZE; i++) {
                if (!picked[i]) {
                    beams[selected++] = next_beams[i];
                    picked[i] = true;
                }
            }
            beam_count = selected;
        } else {
            beam_count = next_count > OCR_DECODE_BEAM_SIZE ? OCR_DECODE_BEAM_SIZE : next_count;
            memcpy(beams, next_beams, (size_t)beam_count * sizeof(beams[0]));
        }
    }

    for (int i = 0; i < beam_count; i++) {
        if (!family_full_valid(family, beams[i].token_ids, beams[i].token_count, keys, key_count))
            continue;
        {
            /* UTF-8 safe copy: never split a multi-byte character.
               Walk backwards from the truncation point to find a valid
               start byte (0xxxxxxx or 11xxxxxx) or continuation byte
               boundary. */
            size_t max_copy = text_len - 1;
            size_t src_len = strlen(beams[i].text);
            size_t n = src_len < max_copy ? src_len : max_copy;
            if (n > 0 && text_len > 0) {
                /* Find last byte that is NOT a UTF-8 continuation byte (10xxxxxx) */
                while (n > 0 && (beams[i].text[n] & 0xC0) == 0x80)
                    n--;
                memcpy(text, beams[i].text, n);
                text[n] = '\0';
            } else {
                text[0] = '\0';
            }
        }
        ret = 0;
        goto cleanup;
    }
    {
        int best_prefix = -1;
        for (int i = 0; i < beam_count; i++) {
            if (!family_prefix_valid(family, beams[i].token_ids, beams[i].token_count, keys, key_count))
                continue;
            if (best_prefix < 0 || beams[i].token_count > beams[best_prefix].token_count)
                best_prefix = i;
        }
        if (best_prefix >= 0) {
            /* UTF-8 safe copy (same logic as above) */
            size_t max_copy = text_len - 1;
            size_t src_len = strlen(beams[best_prefix].text);
            size_t n = src_len < max_copy ? src_len : max_copy;
            if (n > 0 && text_len > 0) {
                while (n > 0 && (beams[best_prefix].text[n] & 0xC0) == 0x80)
                    n--;
                memcpy(text, beams[best_prefix].text, n);
                text[n] = '\0';
            } else {
                text[0] = '\0';
            }
            ret = 0;
            goto cleanup;
        }
    }

    ret = -1;

cleanup:
    free(log_probs);
    return ret;
}

int ocr_decode_logits(const float *buf, int t_size, int c_size, int t_stride, int c_stride,
                      const char *const *keys, int key_count, int blank_idx,
                      enum ocr_decode_family family,
                      char *text, size_t text_len, float *conf_out,
                      struct ocr_decode_diag *diag)
{
    int t;
    int prev = -1;
    int emitted = 0;
    int blank_top1_count = 0;
    double conf_sum = 0.0;
    char greedy_text[OCR_DECODE_MAX_TEXT];

    if (!buf || !keys || !text || text_len == 0 || t_size <= 0 || c_size <= 1 || key_count <= 0)
        return -1;
    text[0] = '\0';
    greedy_text[0] = '\0';

    if (blank_idx < 0 || blank_idx >= c_size) {
        if (c_size == key_count + 1)
            blank_idx = key_count;
        else
            blank_idx = c_size - 1;
    }

    for (t = 0; t < t_size; t++) {
        int c;
        int best_c = 0;
        float best_logit = -1e30f;
        float max_logit = -1e30f;
        double exp_sum = 0.0;
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
            exp_sum += exp((double)v - (double)max_logit);
        }
        if (exp_sum < 1e-12)
            exp_sum = 1e-12;
        if (best_c == blank_idx)
            blank_top1_count++;
        if (best_c == blank_idx || best_c == prev) {
            prev = best_c;
            continue;
        }
        if (best_c >= 0 && best_c < key_count) {
            double prob = exp((double)best_logit - (double)max_logit) / exp_sum;
            if (append_utf8_token_local(greedy_text, sizeof(greedy_text), keys[best_c])) {
                emitted++;
                conf_sum += prob;
            }
        }
        prev = best_c;
    }

    strncpy(text, greedy_text, text_len - 1);
    text[text_len - 1] = '\0';
    if (conf_out)
        *conf_out = (emitted > 0) ? (float)(conf_sum / (double)emitted) : 0.0f;
    if (diag) {
        diag->t_size = t_size;
        diag->c_size = c_size;
        diag->blank_idx = blank_idx;
        diag->blank_top1_ratio = (t_size > 0) ? ((float)blank_top1_count / (float)t_size) : 0.0f;
    }

    /* Beam search 主路径:对所有 family (含 NONE) 总是跑 constrained beam
     * (带 family 前缀约束, 保留 police/embassy 专用解码规则);
     * greedy 仅作为 beam 失败时 fallback. CTC beam 搜索空间 ⊇ greedy 路径,
     * 零误伤 (诊断见 obsidian firstchar-decode-optimization-diag-2026-07-07:
     * bw=5 修 LEN 长度抖动零误伤, 蓝/绿退化切片 +1.4/+0.8pp).
     * 之前 NONE 直接返回 greedy、greedy 合法也返回 greedy, 会漏掉 greedy
     * 合法但 beam 能找到更高分候选的情况; 现统一走 beam. */
    if (constrained_decode(buf, t_size, c_size, t_stride, c_stride,
                           keys, key_count, blank_idx, family,
                           text, text_len) == 0)
        return 0;

    strncpy(text, greedy_text, text_len - 1);
    text[text_len - 1] = '\0';
    return 0;
}
