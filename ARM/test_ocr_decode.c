#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ocr_decode.h"

#define ARRAY_LEN(x) ((int)(sizeof(x) / sizeof((x)[0])))

static void fail(const char *msg)
{
    fprintf(stderr, "[FAIL] %s\n", msg);
    exit(1);
}

static void expect_str_eq(const char *name, const char *got, const char *want)
{
    if (strcmp(got, want) != 0) {
        fprintf(stderr, "[FAIL] %s got=%s want=%s\n", name, got, want);
        exit(1);
    }
}

static void set_row(float *buf, int t, int c_size, int best_idx, float best_logit, int second_idx, float second_logit)
{
    int c;
    float *row = buf + (size_t)t * (size_t)c_size;
    for (c = 0; c < c_size; c++)
        row[c] = -6.0f;
    row[best_idx] = best_logit;
    if (second_idx >= 0)
        row[second_idx] = second_logit;
}

static void greedy_decode_local(const float *buf, int t_size, int c_size, int blank_idx,
                                const char *const *keys, char *out, size_t out_len)
{
    int t;
    int prev = -1;
    out[0] = '\0';
    for (t = 0; t < t_size; t++) {
        int c;
        int best_c = 0;
        float best_logit = -1e30f;
        const float *row = buf + (size_t)t * (size_t)c_size;
        for (c = 0; c < c_size; c++) {
            if (row[c] > best_logit) {
                best_logit = row[c];
                best_c = c;
            }
        }
        if (best_c == blank_idx || best_c == prev) {
            prev = best_c;
            continue;
        }
        if (best_c >= 0 && best_c < blank_idx)
            strncat(out, keys[best_c], out_len - strlen(out) - 1);
        prev = best_c;
    }
}

static void test_green8_relaxed_allows_aa02222(void)
{
    static const char *const keys[] = {"陕", "A", "D", "0", "2"};
    enum ocr_decode_family family = OCR_DECODE_FAMILY_GREEN8;
    const int blank_idx = ARRAY_LEN(keys);
    const int c_size = blank_idx + 1;
    const int t_size = 15;
    float logits[15 * 6];
    char text[64];
    float conf = 0.0f;
    int ret;

    set_row(logits, 0, c_size, 0, 7.0f, -1, 0.0f);
    set_row(logits, 1, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 2, c_size, 1, 7.0f, 2, 6.0f);
    set_row(logits, 3, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 4, c_size, 1, 7.0f, 2, 6.0f);
    set_row(logits, 5, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 6, c_size, 3, 7.0f, 4, 5.0f);
    set_row(logits, 7, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 8, c_size, 4, 7.0f, -1, 0.0f);
    set_row(logits, 9, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 10, c_size, 4, 7.0f, -1, 0.0f);
    set_row(logits, 11, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 12, c_size, 4, 7.0f, -1, 0.0f);
    set_row(logits, 13, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 14, c_size, 4, 7.0f, -1, 0.0f);

    ret = ocr_decode_logits(logits, t_size, c_size, c_size, 1,
                            keys, ARRAY_LEN(keys), blank_idx,
                            family, text, sizeof(text), &conf, NULL);
    if (ret != 0)
        fail("family-aware decode returned non-zero");
    expect_str_eq("green8_exact", text, "陕AA02222");
}

static void test_green8_family_aware_recovers_short_greedy(void)
{
    static const char *const keys[] = {"陕", "A", "D", "0", "2"};
    const int blank_idx = ARRAY_LEN(keys);
    const int c_size = blank_idx + 1;
    const int t_size = 15;
    float logits[15 * 6];
    char greedy_text[64];
    char family_text[64];
    float conf = 0.0f;
    int ret;

    set_row(logits, 0, c_size, 0, 7.0f, -1, 0.0f);
    set_row(logits, 1, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 2, c_size, 1, 7.0f, 2, 6.0f);
    set_row(logits, 3, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 4, c_size, 1, 7.0f, 2, 6.0f);
    set_row(logits, 5, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 6, c_size, 3, 7.0f, 4, 5.0f);
    set_row(logits, 7, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 8, c_size, 4, 7.0f, -1, 0.0f);
    set_row(logits, 9, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 10, c_size, blank_idx, 6.0f, 4, 5.6f);
    set_row(logits, 11, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 12, c_size, 4, 7.0f, -1, 0.0f);
    set_row(logits, 13, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 14, c_size, 4, 7.0f, -1, 0.0f);

    greedy_decode_local(logits, t_size, c_size, blank_idx, keys, greedy_text, sizeof(greedy_text));
    expect_str_eq("greedy_short", greedy_text, "陕AA0222");

    ret = ocr_decode_logits(logits, t_size, c_size, c_size, 1,
                            keys, ARRAY_LEN(keys), blank_idx,
                            OCR_DECODE_FAMILY_GREEN8,
                            family_text, sizeof(family_text), &conf, NULL);
    if (ret != 0)
        fail("family-aware decode returned non-zero on recovery case");
    expect_str_eq("green8_recover", family_text, "陕AA02222");
}

static void test_none_family_falls_back_to_greedy(void)
{
    static const char *const keys[] = {"陕", "A", "D", "0", "2"};
    const int blank_idx = ARRAY_LEN(keys);
    const int c_size = blank_idx + 1;
    const int t_size = 15;
    float logits[15 * 6];
    char text[64];
    float conf = 0.0f;
    int ret;

    set_row(logits, 0, c_size, 0, 7.0f, -1, 0.0f);
    set_row(logits, 1, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 2, c_size, 1, 7.0f, 2, 6.0f);
    set_row(logits, 3, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 4, c_size, 1, 7.0f, 2, 6.0f);
    set_row(logits, 5, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 6, c_size, 3, 7.0f, 4, 5.0f);
    set_row(logits, 7, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 8, c_size, 4, 7.0f, -1, 0.0f);
    set_row(logits, 9, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 10, c_size, blank_idx, 6.0f, 4, 5.6f);
    set_row(logits, 11, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 12, c_size, 4, 7.0f, -1, 0.0f);
    set_row(logits, 13, c_size, blank_idx, 7.0f, -1, 0.0f);
    set_row(logits, 14, c_size, 4, 7.0f, -1, 0.0f);

    ret = ocr_decode_logits(logits, t_size, c_size, c_size, 1,
                            keys, ARRAY_LEN(keys), blank_idx,
                            OCR_DECODE_FAMILY_NONE,
                            text, sizeof(text), &conf, NULL);
    if (ret != 0)
        fail("none-family decode returned non-zero");
    expect_str_eq("none_family_greedy", text, "陕AA0222");
}

int main(void)
{
    test_green8_relaxed_allows_aa02222();
    test_green8_family_aware_recovers_short_greedy();
    test_none_family_falls_back_to_greedy();
    printf("[PASS] test_ocr_decode\n");
    return 0;
}
