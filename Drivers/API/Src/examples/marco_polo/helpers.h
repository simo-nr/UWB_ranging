#ifndef HELPERS_H
#define HELPERS_H

#include <stdint.h>


void waitforsysstatus_timeout(uint32_t *lo_result,
                              uint32_t *hi_result,
                              uint32_t lo_mask,
                              uint32_t hi_mask,
                              uint32_t timeout_ms);

uint64_t get_relative_time(int index, int detected_peak_index, uint64_t detected_peak_time);

#endif