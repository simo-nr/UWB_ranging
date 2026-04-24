#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <float.h>
#include <stddef.h>

#include <deca_device_api.h>

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
    float inv_max_mag;

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

    inv_max_mag = 1.0f / max_mag;
    for (i = 0; i < len; i++) {
        out[i] = mag[i] * inv_max_mag;
    }

    return 0;
}

int find_noise_window(const float *mag, size_t len)
{
    size_t i;
    float window_sum;
    float min_sum;
    int min_index;

    if (mag == NULL || len <= WINDOW_LENGTH) {
        return -1;
    }

    window_sum = 0.0f;
    for (i = 0; i < WINDOW_LENGTH; i++) {
        window_sum += mag[i];
    }

    min_sum = window_sum;
    min_index = 0;

    for (i = WINDOW_LENGTH; i < len; i++) {
        window_sum += mag[i];
        window_sum -= mag[i - WINDOW_LENGTH];

        if (window_sum < min_sum) {
            min_sum = window_sum;
            min_index = (int)(i - WINDOW_LENGTH + 1);
        }
    }

    return min_index;
}

int find_first_peak(const float *mag, size_t len, int start_index, float threshold)
{
    size_t i;

    if (mag == NULL || len == 0 || start_index < 0 || (size_t)start_index >= len) {
        return -1;
    }

    for (i = (size_t)start_index; i < len; i++) {
        if (mag[i] > threshold) {
            return (int)i;
        }
    }

    return -1;
}

int get_mean_and_var(const float *arr, size_t len, float *mean_out, float *var_out)
{
    size_t i;
    float sum = 0.0f;
    float sum_sq = 0.0f;

    if (arr == NULL || mean_out == NULL || var_out == NULL || len == 0) {
        return -1;
    }

    for (i = 0; i < len; i++) {
        sum += arr[i];
        sum_sq += arr[i] * arr[i];
    }

    *mean_out = sum / (float)len;
    *var_out = (sum_sq / (float)len) - (*mean_out * *mean_out);
    return 0;
}

int detect_cir_start(const float *mag, size_t len, float mag_norm_buf[], float *noise_threshold_out)
{
    int noise_window_index;
    float noise_mean;
    float noise_var;
    float threshold;

    if (mag == NULL || len == 0 || mag_norm_buf == NULL || noise_threshold_out == NULL) {
        return -1;
    }

    if (normalize_array(mag, mag_norm_buf, len) != 0) {
        return -1;
    }

    noise_window_index = find_noise_window(mag_norm_buf, len);
    if (noise_window_index < 0) {
        return -1;
    }

    if (get_mean_and_var(mag_norm_buf + noise_window_index, WINDOW_LENGTH, &noise_mean, &noise_var) != 0) {
        return -1;
    }

    if (noise_var < 0.0f) {
        noise_var = 0.0f;
    }

    threshold = noise_mean + 7.0f * sqrtf(noise_var);
    *noise_threshold_out = threshold;

    return find_first_peak(mag_norm_buf, len, noise_window_index + WINDOW_LENGTH, threshold);
}

size_t detect_peaks(const float *mag,
                    size_t len,
                    float fp_index,
                    float *peaks_out,
                    size_t max_peaks,
                    float mag_norm_buf[]) {
    size_t peak_count = 0;
    size_t i;
    size_t n;
    size_t j;
    float *mag_norm = mag_norm_buf;

    // test_run_info((unsigned char *)"Detecting peaks...\n");

    if (mag == NULL || peaks_out == NULL || mag_norm_buf == NULL || len < 3 || max_peaks == 0) {
        test_run_info((unsigned char *)"Invalid input to detect_peaks\n");
        return 0;
    }

    if (mag != mag_norm_buf) {
        if (normalize_array(mag, mag_norm, len) != 0) {
            test_run_info((unsigned char *)"Failed to normalize array in detect_peaks\n");
            // free(mag_norm);
            return 0;
        }
    }

    // i = 0;
    i = 1;
    n = len - 1;
    while (i < n && peak_count < max_peaks) {
        // if (i > 0 &&
        //     mag_norm[i] > PEAK_THRESHOLD &&
        //     mag_norm[i] > mag_norm[i - 1] &&
        //     mag_norm[i] > mag_norm[i + 1]) {
        float curr = mag_norm[i];

        if (curr > PEAK_THRESHOLD &&
            curr > mag_norm[i - 1] &&
            curr > mag_norm[i + 1]) {
            peaks_out[peak_count++] = (float)i;
            i += SIGNAL_LENGTH;
        } else {
            i += 1;
        }
    }

    // // print detected peaks
    // test_run_info((unsigned char *)"Detected peaks at indices: [");
    // for (j = 0; j < peak_count; j++) {
    //     sprintf(str_to_print, "%f", peaks_out[j]);
    //     test_run_info((unsigned char *)str_to_print);
    //     if (j + 1 < peak_count) {
    //         test_run_info((unsigned char *)", ");
    //     }
    // }
    // test_run_info((unsigned char *)"]\n\n");

    // sprintf(str_to_print, "Detected %lu peaks before fp_index adjustment\r\n",
    //     (unsigned long)peak_count);
    // test_run_info((unsigned char *)str_to_print);

    for (j = 0; j < peak_count; j++) {
        if (fabsf(peaks_out[j] - fp_index) < (SIGNAL_LENGTH / 2.0f)) {
            peaks_out[j] = fp_index;
        }
    }

    // for (i = 0; i < peak_count; i++) {
    //     for (j = i + 1; j < peak_count; j++) {
    //         if (peaks_out[j] < peaks_out[i]) {
    //             float tmp = peaks_out[i];
    //             peaks_out[i] = peaks_out[j];
    //             peaks_out[j] = tmp;
    //         }
    //     }
    // }

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
                               const ResponderPeak results[MAX_RESPONDERS],
                               size_t result_count,
                               uint64_t *relative_times_out,
                               size_t max_out)
{
    size_t i;
    size_t out_count = 0;

    if (results == NULL || relative_times_out == NULL) {
        return 0;
    }

    // count = (peak_count < max_out) ? peak_count : max_out;

    for (i = 0; i < result_count && out_count < max_out; i++) {
        if (!results[i].valid) {
            continue;
        }

        double difference = (double)results[i].peak - (double)fp_index;
        double relative_time = (double)rx_time + difference * TICKS_PER_CIR_SAMPLE;
        if (relative_time < 0.0) {
            relative_time = 0.0;
        }
        relative_times_out[out_count++] = (uint64_t)relative_time;
    }

    return out_count;
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
    d_m = tau_s * SPEED_OF_LIGHT;

    // printf("Responder %u: delta_i (DTU) = %llu, t_resp_i (DTU) = %llu\n",
    //        responder_id,
    //        (unsigned long long)delta_i,
    //        (unsigned long long)t_resp_i);

    // sprintf(str_to_print, "Responder %u: delta_i (DTU) = %llu, t_resp_i (DTU) = %llu\r\n",
    //         responder_id,
    //         (unsigned long long)delta_i,
    //         (unsigned long long)t_resp_i);
    // test_run_info((unsigned char *)str_to_print);
    
    // printf("Total time (DTU) = %llu, ToF (DTU) = %.2f, ToF (ns) = %.6f, Distance (m) = %.2f\n",
    //        (unsigned long long)total_time,
    //        tau_dtu,
    //        tau_s * 1e9,
    //        d_m);

    // sprintf(str_to_print, "Total time (DTU) = %llu, ToF (DTU) = %.2f, ToF (ns) = %.6f, Distance (m) = %.2f\r\n",
    //         (unsigned long long)total_time,
    //         tau_dtu,
    //         tau_s * 1e9,
    //         d_m);
    // test_run_info((unsigned char *)str_to_print);

    result.tau_dtu = tau_dtu;
    result.distance_m = d_m;
    return result;
}

void interval_peak_detection(const float *mag,
                             size_t len,
                             float fp_index,
                             float peak_threshold,
                             int *found,
                             ResponderPeak results[MAX_RESPONDERS],
                             float mag_norm_buf[])
{
    int interval_step;
    int interval_search_len;
    int responder_id = 0;
    int start;
    float *mag_norm = mag_norm_buf;

    if (mag == NULL || found == NULL || results == NULL || mag_norm_buf == NULL || len == 0) {
        return;
    }

    for (int i = 0; i < MAX_RESPONDERS; i++) {
        results[i].responder_id = i;
        results[i].peak = -1.0f;
        results[i].valid = 0;
    }

    // print peak threshold
    // printf("Using peak threshold: %f\n", peak_threshold);

    if (mag != mag_norm_buf) {
        if (normalize_array(mag, mag_norm, len) != 0) {
            // free(mag_norm);
            return;
        }
    }

    interval_step = (int)lround(INTENTIONAL_DELAY_NS);
    interval_search_len = interval_step - 10;

    for (start = -20; start < (int)len && responder_id < MAX_RESPONDERS; start += interval_step, responder_id++) {
        int slice_start;
        int slice_end;
        int max_index = 0;
        int found_peak = 0;

        slice_start = (start >= 0) ? start : 0;
        slice_end = start + interval_search_len;

        if (slice_end < 0) {
            // printf("Warning: Interval for responder %d is empty. Skipping.\n", responder_id);
            continue;
        }

        if (slice_end > (int)len) {
            slice_end = (int)len;
        }

        // printf("checking from index %d to %d for responder %d\n",
        //        slice_start, slice_end, responder_id);

        if (slice_start >= slice_end) {
            // printf("Warning: Interval for responder %d is empty. Skipping.\n", responder_id);
            continue;
        }

        for (int j = 1; j < (slice_end - slice_start) - 1; j++) {
            float curr = mag_norm[slice_start + j];
            float prev = mag_norm[slice_start + j - 1];
            float next = mag_norm[slice_start + j + 1];

            if (curr > peak_threshold && curr > prev && curr > next) {
                max_index = j;
                found_peak = 1;
                break;
            }
        }

        if (!found_peak) {
            // printf("No peak found for responder %d in interval starting at index %d\n", responder_id, slice_start);
        }

        if (found_peak && mag_norm[slice_start + max_index] > peak_threshold) {
            float peak = (float)(slice_start + max_index);

            if (fabsf(peak - fp_index) < (SIGNAL_LENGTH / 2.0f)) {
                peak = fp_index;
            }

            found[responder_id] = 1;
            results[responder_id].peak = peak;
            results[responder_id].valid = 1;
        }
    }
}