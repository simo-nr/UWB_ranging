
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <float.h>
#include <stddef.h>

#include <deca_device_api.h>
#include <string.h>

#include "analyzer.h"

extern void test_run_info(unsigned char *data);

static char str_to_print[DWT_CIR_LEN_MAX * 2 * 3]; /* Buffer for printing*/


static float max_value(const float *arr, size_t len)
{
    size_t i;
    float max_val;

    if (arr == NULL || len == 0) {
        return 0.0f;
    }

    max_val = arr[0];
    for (i = 1; i < len; i++) {
        if (arr[i] > max_val) {
            max_val = arr[i];
        }
    }

    return max_val;
}

int normalize_array(const float *mag, float *out, size_t len)
{
    size_t i;
    float max_mag;

    if (mag == NULL || out == NULL || len == 0) {
        return -1;
    }

    max_mag = max_value(mag, len);
    if (max_mag == 0.0f) {
        for (i = 0; i < len; i++) {
            out[i] = 0.0f;
        }
        return 0;
    }

    for (i = 0; i < len; i++) {
        out[i] = mag[i] / max_mag;
    }

    return 0;
}

int find_noise_window(const float *mag, size_t len)
{
    size_t i;
    size_t j;
    float min_sum;
    int min_index;

    if (mag == NULL || len <= WINDOW_LENGTH) {
        return -1;
    }

    min_sum = FLT_MAX;
    min_index = 0;

    for (i = 0; i < len - WINDOW_LENGTH; i++) {
        float window_sum = 0.0f;
        for (j = 0; j < WINDOW_LENGTH; j++) {
            window_sum += mag[i + j];
        }

        if (window_sum < min_sum) {
            min_sum = window_sum;
            min_index = (int)i;
        }
    }

    return min_index;
}

int find_first_peak(const float *mag, size_t len, int start_index)
{
    size_t i;

    if (mag == NULL || len == 0 || start_index < 0 || (size_t)start_index >= len) {
        return -1;
    }

    for (i = (size_t)start_index; i < len; i++) {
        if (mag[i] > NOISE_THRESHOLD) {
            return (int)i;
        }
    }

    return -1;
}

int detect_cir_start(const float *mag, size_t len)
{
    float *mag_norm;
    int noise_window_index;
    int first_peak_index;

    if (mag == NULL || len == 0) {
        return -1;
    }

    mag_norm = (float *)malloc(len * sizeof(float));
    if (mag_norm == NULL) {
        return -1;
    }

    if (normalize_array(mag, mag_norm, len) != 0) {
        free(mag_norm);
        return -1;
    }

    noise_window_index = find_noise_window(mag_norm, len);
    if (noise_window_index < 0) {
        free(mag_norm);
        return -1;
    }

    first_peak_index = find_first_peak(mag_norm, len, noise_window_index + WINDOW_LENGTH);
    free(mag_norm);
    return first_peak_index;
}

size_t detect_peaks(const float *mag,
                    size_t len,
                    float fp_index,
                    float *peaks_out,
                    size_t max_peaks,
                    float mag_norm_buf[]) {
    float *mag_norm;
    size_t peak_count = 0;
    size_t i;
    size_t n;
    size_t j;

    test_run_info((unsigned char *)"Detecting peaks...\n");

    if (mag == NULL || peaks_out == NULL || len < 3 || max_peaks == 0) {
        test_run_info((unsigned char *)"Invalid input to detect_peaks\n");
        return 0;
    }

    // mag_norm = (float *)malloc(len * sizeof(float));
    // if (mag_norm == NULL) {
    //     test_run_info((unsigned char *)"Failed to allocate mag_norm in detect_peaks\n");
    //     return 0;
    // }
    mag_norm = mag_norm_buf;

    if (normalize_array(mag, mag_norm, len) != 0) {
        test_run_info((unsigned char *)"Failed to normalize array in detect_peaks\n");
        // free(mag_norm);
        return 0;
    }

    i = 0;
    n = len - 1;
    while (i < n && peak_count < max_peaks) {
        if (i > 0 &&
            mag_norm[i] > PEAK_THRESHOLD &&
            mag_norm[i] > mag_norm[i - 1] &&
            mag_norm[i] > mag_norm[i + 1]) {
            peaks_out[peak_count++] = (float)i;
            i += SIGNAL_LENGTH;
        } else {
            i += 1;
        }
    }

    // print detected peaks
    test_run_info((unsigned char *)"Detected peaks at indices: [");
    for (j = 0; j < peak_count; j++) {
        sprintf(str_to_print, "%f", peaks_out[j]);
        test_run_info((unsigned char *)str_to_print);
        if (j + 1 < peak_count) {
            test_run_info((unsigned char *)", ");
        }
    }
    test_run_info((unsigned char *)"]\n\n");

    // sprintf(str_to_print, "Detected %lu peaks before fp_index adjustment\r\n",
    //     (unsigned long)peak_count);
    // test_run_info((unsigned char *)str_to_print);

    for (j = 0; j < peak_count; j++) {
        if (fabs((double)peaks_out[j] - (double)fp_index) < ((double)SIGNAL_LENGTH / 2.0)) {
            peaks_out[j] = fp_index;
        }
    }

    for (i = 0; i < peak_count; i++) {
        for (j = i + 1; j < peak_count; j++) {
            if (peaks_out[j] < peaks_out[i]) {
                float tmp = peaks_out[i];
                peaks_out[i] = peaks_out[j];
                peaks_out[j] = tmp;
            }
        }
    }

    // free(mag_norm);
    return peak_count;
}

float rotate_cir(const float *mag, size_t len, int start_index, float fp_index, float *rotated_out)
{
    size_t i;

    if (mag == NULL || rotated_out == NULL || len == 0 || start_index < 0 || (size_t)start_index >= len) {
        return fp_index;
    }

    for (i = 0; i < len; i++) {
        rotated_out[i] = mag[(start_index + (int)i) % (int)len];
    }

    return fp_index - (float)start_index;
}

size_t get_relative_time_ticks(uint64_t rx_time,
                               float fp_index,
                               const float *peaks,
                               size_t peak_count,
                               uint64_t *relative_times_out,
                               size_t max_out)
{
    size_t i;
    size_t count;

    if (peaks == NULL || relative_times_out == NULL) {
        return 0;
    }

    count = (peak_count < max_out) ? peak_count : max_out;

    for (i = 0; i < count; i++) {
        double difference = (double)peaks[i] - (double)fp_index;
        double relative_time = (double)rx_time + difference * TICKS_PER_CIR_SAMPLE;
        if (relative_time < 0.0) {
            relative_time = 0.0;
        }
        relative_times_out[i] = (uint64_t)relative_time;
    }

    return count;
}

tof_result_t tof_and_distance_from_absolute_rx(uint64_t total_time, uint32_t responder_id)
{
    uint64_t delta_i;
    uint64_t t_resp_i;
    double tau_dtu;
    double tau_s;
    double d_m;
    tof_result_t result;

    delta_i = (uint64_t)responder_id * DELTA_I_DTU;
    t_resp_i = RESP_TX_DELAY_DTU + TX_ANT_DLY_DTU + delta_i;
    tau_dtu = ((double)total_time - (double)t_resp_i) / 2.0;
    tau_s = tau_dtu * DTU_SECONDS;
    d_m = tau_s * C_M_PER_S;

    // printf("Responder %u: delta_i (DTU) = %llu, t_resp_i (DTU) = %llu\n",
    //        responder_id,
    //        (unsigned long long)delta_i,
    //        (unsigned long long)t_resp_i);

    sprintf(str_to_print, "Responder %u: delta_i (DTU) = %llu, t_resp_i (DTU) = %llu\r\n",
            responder_id,
            (unsigned long long)delta_i,
            (unsigned long long)t_resp_i);
    test_run_info((unsigned char *)str_to_print);
    
    // printf("Total time (DTU) = %llu, ToF (DTU) = %.2f, ToF (ns) = %.6f, Distance (m) = %.2f\n",
    //        (unsigned long long)total_time,
    //        tau_dtu,
    //        tau_s * 1e9,
    //        d_m);

    sprintf(str_to_print, "Total time (DTU) = %llu, ToF (DTU) = %.2f, ToF (ns) = %.6f, Distance (m) = %.2f\r\n",
            (unsigned long long)total_time,
            tau_dtu,
            tau_s * 1e9,
            d_m);
    test_run_info((unsigned char *)str_to_print);

    result.tau_dtu = tau_dtu;
    result.distance_m = d_m;
    return result;
}