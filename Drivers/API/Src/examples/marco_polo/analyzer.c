#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <float.h>
#include <stddef.h>

#include <deca_device_api.h>

#include "load_data.h"
#include "analyzer.h"

extern void test_run_info(unsigned char *data);

static char str_to_print[DWT_CIR_LEN_MAX * 2 * 3]; /* Buffer for printing*/


int normalise_and_find_noise_window(cir_data_t *cir_data, float mag_norm_buf[])
{
    size_t i;
    float inv_peak_amp;
    float window_sum;
    float min_sum;
    int min_index;

    if (cir_data == NULL ||
        cir_data->mag == NULL ||
        mag_norm_buf == NULL ||
        cir_data->length <= WINDOW_LENGTH ||
        cir_data->peak_amp <= 0.0f) {
        return -1;
    }

    inv_peak_amp = 1.0f / cir_data->peak_amp;

    window_sum = 0.0f;
    for (i = 0; i < WINDOW_LENGTH; i++) {
        float norm = cir_data->mag[i] * inv_peak_amp;
        mag_norm_buf[i] = norm;
        window_sum += norm;
    }

    min_sum = window_sum;
    min_index = 0;

    for (i = WINDOW_LENGTH; i < cir_data->length; i++) {
        float norm = cir_data->mag[i] * inv_peak_amp;
        mag_norm_buf[i] = norm;

        window_sum += norm;
        window_sum -= mag_norm_buf[i - WINDOW_LENGTH];

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

int detect_cir_start(cir_data_t cir_data, float mag_norm_buf[], float *noise_threshold_out)
{
    int noise_window_index;
    float noise_mean;
    float noise_var;
    float threshold;

    if (cir_data.mag == NULL || cir_data.length == 0 || mag_norm_buf == NULL || noise_threshold_out == NULL) {
        return -1;
    }

    noise_window_index = normalise_and_find_noise_window(&cir_data, mag_norm_buf);
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

    return find_first_peak(mag_norm_buf, cir_data.length, noise_window_index + WINDOW_LENGTH, threshold);
}

size_t find_interval_values(const float *mag,
                            size_t len,
                            float fp_index,
                            uint64_t rx_time,
                            float *interval_values_out,
                            size_t max_intervals)
{
    uint64_t revert;
    double first_interval_start_ticks;
    double interval_start_ticks;
    size_t interval_count = 0;

    if (mag == NULL || interval_values_out == NULL || len == 0 || max_intervals == 0) {
        return 0;
    }

    revert = rx_time % INTENTIONAL_DELAY_DTU;
    first_interval_start_ticks = fmod((double)fp_index * TICKS_PER_CIR_SAMPLE - (double)revert,
                                      (double)INTENTIONAL_DELAY_DTU);
    if (first_interval_start_ticks < 0.0) {
        first_interval_start_ticks += (double)INTENTIONAL_DELAY_DTU;
    }

    interval_start_ticks = first_interval_start_ticks;
    while ((interval_start_ticks / TICKS_PER_CIR_SAMPLE) < (double)len &&
           interval_count < max_intervals) {
        interval_values_out[interval_count++] = (float)(interval_start_ticks / TICKS_PER_CIR_SAMPLE);
        interval_start_ticks += (double)INTENTIONAL_DELAY_DTU;
    }

    return interval_count;
}

// size_t get_relative_time_ticks(uint64_t rx_time,
//                                float fp_index,
//                                ResponderPeak results[MAX_RESPONDERS],
//                                size_t result_count,
//                                uint64_t *relative_times_out,
//                                size_t max_out)
// {
//     size_t i;

//     if (results == NULL || relative_times_out == NULL) {
//         return 0;
//     }

//     for (i = 0; i < MAX_RESPONDERS; i++) {
//         if (!results[i].valid) {
//             continue;
//         }

//         double difference = (double)results[i].peak - (double)fp_index;
//         double relative_time = (double)rx_time + difference * TICKS_PER_CIR_SAMPLE;
//         if (relative_time < 0.0) {
//             relative_time = 0.0;
//         }
//         results[i].time = (int)relative_time;
//     }

//     return 0;
// }

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

    result.tau_dtu = tau_dtu;
    result.distance_m = d_m;
    return result;
}

// void interval_peak_detection_wrapped(const float *mag,
//                                      size_t len,
//                                      int start_index,
//                                      float fp_index,
//                                      float peak_threshold,
//                                      ResponderPeak results[MAX_RESPONDERS])
// {
//     int interval_step;
//     int interval_search_len;
//     int responder_id = 0;
//     int start;

//     if (mag == NULL || results == NULL || len == 0 || start_index < 0 || (size_t)start_index >= len) {
//         return;
//     }

//     for (int i = 0; i < MAX_RESPONDERS; i++) {
//         results[i].responder_id = i;
//         results[i].peak = -1.0f;
//         results[i].valid = 0;
//         results[i].time = 0;
//         results[i].distance = 0;
//     }

//     interval_step = (int)(INTENTIONAL_DELAY_NS + 0.5f);
//     interval_search_len = interval_step - 10;

//     for (start = -20; start < (int)len && responder_id < MAX_RESPONDERS; start += interval_step, responder_id++) {
//         int slice_start;
//         int slice_end;
//         int max_index = 0;
//         int found_peak = 0;

//         slice_start = (start >= 0) ? start : 0;
//         slice_end = start + interval_search_len;

//         if (slice_end < 0) {
//             continue;
//         }

//         if (slice_end > (int)len) {
//             slice_end = (int)len;
//         }

//         if (slice_start >= slice_end) {
//             continue;
//         }

//         for (int j = 1; j < (slice_end - slice_start) - 1; j++) {
//             int rotated_idx = slice_start + j;
//             int phys_idx = start_index + rotated_idx;
//             int prev_idx;
//             int next_idx;
//             float curr;
//             float prev;
//             float next;

//             if (phys_idx >= (int)len) {
//                 phys_idx -= (int)len;
//             }

//             prev_idx = phys_idx - 1;
//             if (prev_idx < 0) {
//                 prev_idx = (int)len - 1;
//             }

//             next_idx = phys_idx + 1;
//             if (next_idx >= (int)len) {
//                 next_idx = 0;
//             }

//             curr = mag[phys_idx];
//             prev = mag[prev_idx];
//             next = mag[next_idx];

//             if (curr > peak_threshold && curr > prev && curr > next) {
//                 max_index = rotated_idx;
//                 found_peak = 1;
//                 break;
//             }
//         }

//         if (found_peak) {
//             float peak = (float)max_index;

//             if (fabsf(peak - fp_index) < (SIGNAL_LENGTH / 2.0f)) {
//                 peak = fp_index;
//             }

//             results[responder_id].peak = peak;
//             results[responder_id].valid = 1;
//         }
//     }
// }

void interval_peak_detection_wrapped_interval(const float *mag,
                                     size_t len,
                                     int start_index,
                                     float fp_index,
                                     const float *intervals,
                                     size_t interval_count,
                                     float peak_threshold,
                                     ResponderPeak results[MAX_RESPONDERS])
{
    int interval_search_len;

    if (mag == NULL || intervals == NULL || results == NULL || len == 0 ||
        start_index < 0 || (size_t)start_index >= len) {
        return;
    }

    for (int i = 0; i < MAX_RESPONDERS; i++) {
        results[i].responder_id = i;
        results[i].peak = -1.0f;
        results[i].valid = 0;
        results[i].time = 0;
        results[i].distance = 0;
    }

    interval_search_len = (int)((INTENTIONAL_DELAY_DTU / TICKS_PER_CIR_SAMPLE) - 10U);

    for (int responder_id = 0; responder_id < MAX_RESPONDERS; responder_id++) {
        int start;
        int slice_start;
        int slice_end;
        int local_len;
        int max_index = 0;
        int found_peak = 0;

        /*
         * Python equivalent:
         * for responder_id, i in enumerate([0] + [int(x) for x in intervals]):
         */
        if (responder_id == 0) {
            start = 0;
        } else {
            if ((size_t)(responder_id - 1) >= interval_count) {
                break;
            }
            start = (int)intervals[responder_id - 1];
        }

        slice_start = (start >= 0) ? start : 0;
        slice_end = start + interval_search_len;

        if (slice_end < 0) {
            continue;
        }

        if (slice_end > (int)len) {
            slice_end = (int)len;
        }

        if (slice_start >= slice_end) {
            continue;
        }

        local_len = slice_end - slice_start;

        if (responder_id == 0) {
            /*
             * Python equivalent:
             * for j in range(0, len(interval)-1):
             *     if interval[j] > threshold and interval[j] > interval[j+1]:
             */
            for (int j = 0; j < local_len - 1; j++) {
                int rotated_idx = slice_start + j;
                int phys_idx = start_index + rotated_idx;
                int next_phys_idx;
                float curr;
                float next;

                while (phys_idx >= (int)len) {
                    phys_idx -= (int)len;
                }

                next_phys_idx = phys_idx + 1;
                if (next_phys_idx >= (int)len) {
                    next_phys_idx = 0;
                }

                curr = mag[phys_idx];
                next = mag[next_phys_idx];

                if (curr > peak_threshold && curr > next) {
                    max_index = j;
                    found_peak = 1;
                    break;
                }
            }
        } else {
            /*
             * Python equivalent:
             * for j in range(1, len(interval)-1):
             *     if interval[j] > threshold and interval[j] > interval[j-1] and interval[j] > interval[j+1]:
             */
            for (int j = 1; j < local_len - 1; j++) {
                int rotated_idx = slice_start + j;
                int phys_idx = start_index + rotated_idx;
                int prev_phys_idx;
                int next_phys_idx;
                float curr;
                float prev;
                float next;

                while (phys_idx >= (int)len) {
                    phys_idx -= (int)len;
                }

                prev_phys_idx = phys_idx - 1;
                if (prev_phys_idx < 0) {
                    prev_phys_idx = (int)len - 1;
                }

                next_phys_idx = phys_idx + 1;
                if (next_phys_idx >= (int)len) {
                    next_phys_idx = 0;
                }

                curr = mag[phys_idx];
                prev = mag[prev_phys_idx];
                next = mag[next_phys_idx];

                if (curr > peak_threshold && curr > prev && curr > next) {
                    max_index = j;
                    found_peak = 1;
                    break;
                }
            }
        }

        if (found_peak) {
            float peak = (float)(start + max_index);

            if (fabsf(peak - fp_index) < (SIGNAL_LENGTH / 2.0f)) {
                peak = fp_index;
            }

            results[responder_id].peak = peak;
            results[responder_id].valid = 1;
        }
    }
}

// size_t get_dist_intervals(uint64_t rx_time,
//                           float fp_index,
//                           ResponderPeak peaks[MAX_RESPONDERS],
//                           size_t peak_count,
//                           const float *intervals,
//                           size_t interval_count,
//                           tof_result_t *dist_out,
//                           uint32_t *responder_ids_out,
//                           size_t max_out)
// {
//     size_t i;
//     size_t out_count = 0;

//     (void)rx_time;
//     (void)fp_index;

//     if (peaks == NULL || intervals == NULL || dist_out == NULL || responder_ids_out == NULL ||
//         interval_count == 0 || max_out == 0) {
//         return 0;
//     }

//     for (i = 0; i < peak_count && i < MAX_RESPONDERS && out_count < max_out; i++) {
//         uint32_t responder_id;
//         float peak;
//         float interval_start;
//         float difference;
//         double relative_time;
//         tof_result_t result;

//         if (!peaks[i].valid) {
//             continue;
//         }

//         responder_id = peaks[i].responder_id;
//         peak = peaks[i].peak;

//         if (responder_id > 0) {
//             if ((size_t)(responder_id - 1) >= interval_count) {
//                 continue;
//             }
//             interval_start = intervals[responder_id - 1];
//         } else {
//             interval_start = intervals[interval_count - 1] - 1016.0f;
//         }

//         difference = peak - interval_start;
//         printf("Responder %u: Peak at index %.2f, interval start at index %.2f, difference in samples: %.2f\n",
//                responder_id,
//                peak,
//                interval_start,
//                difference);

//         relative_time = (double)difference * TICKS_PER_CIR_SAMPLE;

//         // peaks[i].time = (int)relative_time;
//         peaks[i].distance = relative_time * DTU_SECONDS * SPEED_OF_LIGHT / 2.0;


//         result.tau_dtu = relative_time;
//         result.distance_m = relative_time * DTU_SECONDS * SPEED_OF_LIGHT / 2.0;

//         responder_ids_out[out_count] = responder_id;
//         dist_out[out_count] = result;

//         out_count++;
//     }

//     return out_count;
// }

size_t get_distances(uint64_t rx_time,
                     float fp_index,
                     ResponderPeak results[MAX_RESPONDERS],
                     uint64_t *relative_times_out)
{
    size_t i;

    if (results == NULL || relative_times_out == NULL) {
        return 0;
    }

    for (i = 0; i < MAX_RESPONDERS; i++) {
        if (!results[i].valid) {
            continue;
        }

        double difference = (double)results[i].peak - (double)fp_index;
        double relative_time = (double)rx_time + difference * TICKS_PER_CIR_SAMPLE;
        if (relative_time < 0.0) {
            relative_time = 0.0;
        }
        results[i].time = (int)relative_time;
        results[i].distance = tof_and_distance_from_absolute_rx((uint64_t)relative_time, results[i].responder_id).distance_m;
    }

    return 0;
}
