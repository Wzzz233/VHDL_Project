// SPDX-License-Identifier: GPL-2.0
/* Common helpers shared across the live LPR pipeline modules. */

#include "lpr_common.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

int64_t lpr_mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

float lpr_sigmoidf(float x)
{
    if (x >= 0.0f) {
        float z = expf(-x);
        return 1.0f / (1.0f + z);
    }
    float z = expf(x);
    return z / (1.0f + z);
}

int lpr_load_file(const char *path, void **data_out, uint32_t *size_out)
{
    FILE *fp;
    long sz;
    void *data;
    fp = fopen(path, "rb");
    if (!fp)
        return -1;
    if (fseek(fp, 0, SEEK_END) < 0) { fclose(fp); return -1; }
    sz = ftell(fp);
    if (sz <= 0) { fclose(fp); return -1; }
    rewind(fp);
    data = malloc((size_t)sz);
    if (!data) { fclose(fp); return -1; }
    if (fread(data, 1, (size_t)sz, fp) != (size_t)sz) {
        free(data);
        fclose(fp);
        return -1;
    }
    fclose(fp);
    *data_out = data;
    *size_out = (uint32_t)sz;
    return 0;
}

int lpr_load_keys(const char *path, struct ocr_keys *keys)
{
    FILE *fp;
    char line[256];
    int n = 0;
    memset(keys, 0, sizeof(*keys));
    fp = fopen(path, "r");
    if (!fp)
        return -1;
    while (fgets(line, sizeof(line), fp) && n < MAX_OCR_KEYS) {
        char *s = line;
        char *nl;
        size_t len;
        nl = strchr(s, '\n'); if (nl) *nl = '\0';
        nl = strchr(s, '\r'); if (nl) *nl = '\0';
        /* strip BOM */
        if ((unsigned char)s[0] == 0xEF && (unsigned char)s[1] == 0xBB && (unsigned char)s[2] == 0xBF)
            s += 3;
        if (s[0] == '\0' || s[0] == '#')
            continue;
        len = strnlen(s, MAX_OCR_KEY_LEN - 1);
        memcpy(keys->keys[n], s, len);
        keys->keys[n][len] = '\0';
        n++;
    }
    fclose(fp);
    keys->count = n;
    return n > 0 ? 0 : -1;
}

const char *lpr_plate_color_str(enum plate_color c)
{
    switch (c) {
    case PLATE_COLOR_BLUE:   return "BLUE";
    case PLATE_COLOR_GREEN:  return "GREEN";
    case PLATE_COLOR_YELLOW: return "YELLOW";
    case PLATE_COLOR_WHITE:  return "WHITE";
    case PLATE_COLOR_BLACK:  return "BLACK";
    default:                 return "UNK";
    }
}
