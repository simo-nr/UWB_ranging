

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include <deca_device_api.h>

#include "load_data.h"

#define LINE_BUFFER_SIZE 256

extern void test_run_info(unsigned char *data);

static char str_to_print[DWT_CIR_LEN_MAX * 2 * 3]; /* Buffer for printing*/

static int parse_int_after_equals(const char *line, const char *key, int *out)
{
    const char *pos = strstr(line, key);
    if (!pos) return 0;

    pos = strchr(pos, '=');
    if (!pos) return 0;

    *out = atoi(pos + 1);
    return 1;
}

static int parse_fp_index(const char *line, char *hex_out, double *samples_out)
{
    const char *pos = strstr(line, "FpIndex=");
    if (!pos) return 0;

    pos += strlen("FpIndex=");

    char temp[32];
    int i = 0;
    while (*pos && *pos != ' ' && *pos != '\n' && i < 31) {
        temp[i++] = *pos++;
    }
    temp[i] = '\0';

    strcpy(hex_out, temp);

    unsigned int value = (unsigned int)strtoul(temp, NULL, 16);
    *samples_out = (double)value / 64.0;

    return 1;
}

cir_data_t load_cir_data(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        perror("Failed to open file");
        exit(EXIT_FAILURE);
    }

    int rx_minus_tx = 0;
    int peak_index = 0;
    char fp_index_hex[32] = {0};
    double fp_index_samples = 0.0;

    size_t capacity = 1024;
    size_t length = 0;

    double *re_vals = malloc(capacity * sizeof(double));
    double *im_vals = malloc(capacity * sizeof(double));

    if (!re_vals || !im_vals) {
        sprintf(str_to_print, "Memory allocation failed\r\n");
        test_run_info((unsigned char *)str_to_print);

        exit(EXIT_FAILURE);
    }

    char line[LINE_BUFFER_SIZE];

    while (fgets(line, sizeof(line), f)) {
        // Remove newline
        line[strcspn(line, "\r\n")] = 0;

        if (strlen(line) == 0)
            continue;

        if (strstr(line, "RX_TS - TX_TS")) {
            parse_int_after_equals(line, "RX_TS - TX_TS", &rx_minus_tx);
            continue;
        }

        if (strstr(line, "peakIndex=")) {
            parse_int_after_equals(line, "peakIndex", &peak_index);
        }

        if (strstr(line, "FpIndex=")) {
            parse_fp_index(line, fp_index_hex, &fp_index_samples);
        }

        if (!strchr(line, ','))
            continue;

        char *token1 = strtok(line, ",");
        char *token2 = strtok(NULL, ",");

        if (!token1 || !token2)
            continue;

        int re = atoi(token1);
        int im = atoi(token2);

        if (length >= capacity) {
            capacity *= 2;
            re_vals = realloc(re_vals, capacity * sizeof(double));
            im_vals = realloc(im_vals, capacity * sizeof(double));

            if (!re_vals || !im_vals) {
                sprintf(str_to_print, "Reallocation failed\r\n");
                test_run_info((unsigned char *)str_to_print);
                exit(EXIT_FAILURE);
            }
        }

        re_vals[length] = (double)re;
        im_vals[length] = (double)im;
        length++;
    }

    fclose(f);

    if (length == 0) {
        sprintf(str_to_print, "No CIR data found\r\n");
        test_run_info((unsigned char *)str_to_print);
        exit(EXIT_FAILURE);
    }

    float *mag = malloc(length * sizeof(float));
    if (!mag) {
        sprintf(str_to_print, "Memory allocation failed\r\n");
        test_run_info((unsigned char *)str_to_print);
        exit(EXIT_FAILURE);
    }

    for (size_t i = 0; i < length; i++) {
        double m = sqrt(re_vals[i] * re_vals[i] + im_vals[i] * im_vals[i]);
        mag[i] = (float)m;
    }

    free(re_vals);
    free(im_vals);

    if (fp_index_hex[0] == '\0') {
        sprintf(str_to_print, "Failed to parse header information\r\n");
        test_run_info((unsigned char *)str_to_print);
        exit(EXIT_FAILURE);
    }

    cir_data_t result;
    result.mag = mag;
    result.length = length;
    result.rx_minus_tx = rx_minus_tx;
    result.peak_index = peak_index;
    strcpy(result.fp_index_hex, fp_index_hex);
    result.fp_index_samples = fp_index_samples;

    return result;
}

void free_cir_data(cir_data_t *data)
{
    if (data && data->mag) {
        free(data->mag);
        data->mag = NULL;
    }
}