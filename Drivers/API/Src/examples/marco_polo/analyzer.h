// detect_cir_start
// rotate_cir
// detect_peaks
// get_relative_time_ticks
// tof_and_distance_from_absolute_rx
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <float.h>
#include <stddef.h>
#include <shared_defines.h>


#define NOISE_THRESHOLD 0.14f
#define PEAK_THRESHOLD  0.2f
#define SIGNAL_LENGTH   60
#define WINDOW_LENGTH   120

#define TX_ANT_DLY_DTU  16385ULL
#define RESP_TX_DELAY_UUS 1500ULL
#define RESP_TX_DELAY_DTU (RESP_TX_DELAY_UUS * UUS_TO_DWT_TIME)

#define TICKS_PER_CIR_SAMPLE 64
#define MAX_RESPONDERS 7


typedef struct {
    double tau_dtu;
    double distance_m;
} tof_result_t;

typedef struct {
    int responder_id;
    float peak;
    int valid;
    int time;
    double distance;
} ResponderPeak;


int detect_cir_start(cir_data_t cir_data, float mag_norm_buf[], float *noise_threshold_out);

size_t detect_peaks(const float *mag,
                    size_t len,
                    float fp_index,
                    float *peaks_out,
                    size_t max_peaks,
                    float *mag_buf);
                    
size_t get_relative_time_ticks(uint64_t rx_time,
                               float fp_index,
                               ResponderPeak results[MAX_RESPONDERS],
                               size_t result_count,
                               uint64_t *relative_times_out,
                               size_t max_out);

tof_result_t tof_and_distance_from_absolute_rx(uint64_t total_time, uint32_t responder_id);

void interval_peak_detection_wrapped(const float *mag,
                                     size_t len,
                                     int start_index,
                                     float fp_index,
                                     float peak_threshold,
                                     ResponderPeak results[MAX_RESPONDERS]);