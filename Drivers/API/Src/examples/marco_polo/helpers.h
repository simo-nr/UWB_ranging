#ifndef HELPERS_H
#define HELPERS_H

#include <stdint.h>


// void waitforsysstatus_timeout(uint32_t *lo_result,
//                               uint32_t *hi_result,
//                               uint32_t lo_mask,
//                               uint32_t hi_mask,
//                               uint32_t timeout_ms);

// uint64_t get_relative_time(int index, int detected_peak_index, uint64_t detected_peak_time);

static inline uint32_t read_u32_le_reg(uint8_t reg_file, uint16_t subaddr);

static float cir_mag_from_buf(const uint8_t *ptr, dwt_cir_read_mode_e mode);

#endif