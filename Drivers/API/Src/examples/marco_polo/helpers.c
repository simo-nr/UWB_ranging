#include <stdint.h>
#include <stdbool.h>

#include <config_options.h>
#include <deca_device_api.h>
#include <deca_types.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <stdlib.h>

#include "helpers.h"

#define SYS_STATUS_ID 0x00

/*
 * You need to provide this.
 * It should return a monotonically increasing time in milliseconds or microseconds.
 * Example sources:
 *   - HAL_GetTick() on STM32 for ms
 *   - a hardware timer
 *   - FreeRTOS tick count
 */
extern uint32_t platform_get_time_ms(void);

/*
 * Return values for the wait function.
 */
typedef enum {
    WAIT_SYSSTATUS_OK = 0,
    WAIT_SYSSTATUS_TIMEOUT,
    WAIT_SYSSTATUS_NULL_TIMEOUT_SOURCE
} wait_sysstatus_result_t;

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Wait until the requested SYS_STATUS bits appear, or until timeout expires.
 *
 * This behaves like your original function, but it will return if the timeout expires.
 *
 * @param lo_result     Pointer that receives final SYS_STATUS low value, or NULL to ignore.
 * @param hi_result     Pointer that receives final SYS_STATUS high value, or NULL to ignore.
 * @param lo_mask       Bits to wait for in low 32 bits. Pass 0 to ignore low status.
 * @param hi_mask       Bits to wait for in high 32 bits. Pass 0 to ignore high status.
 * @param timeout_ms    Timeout in milliseconds.
 *
 * @return WAIT_SYSSTATUS_OK if one of the requested masks matched,
 *         WAIT_SYSSTATUS_TIMEOUT if timeout expired first.
 */
void waitforsysstatus_timeout(uint32_t *lo_result,
                              uint32_t *hi_result,
                              uint32_t lo_mask,
                              uint32_t hi_mask,
                              uint32_t timeout_ms)
{
    uint32_t lo_result_tmp = 0;
    uint32_t hi_result_tmp = 0;
    uint32_t start_ms = platform_get_time_ms();

    // If a mask has been passed into the function for the system status register (lower 32-bits)
    if (lo_mask)
    {
        while (!((lo_result_tmp = dwt_readsysstatuslo()) & lo_mask))
        {
            // Check for timeout
            if ((platform_get_time_ms() - start_ms) >= timeout_ms)
            {
                break;
            }

            // If a mask value is set for the system status register (higher 32-bits)
            if (hi_mask)
            {
                // If mask value for the system status register (higher 32-bits) is found
                if ((hi_result_tmp = dwt_readsysstatushi()) & hi_mask)
                {
                    break;
                }
            }
        }
    }
    // if only a mask value for the system status register (higher 32-bits) is set
    else if (hi_mask)
    {
        while (!((hi_result_tmp = dwt_readsysstatushi()) & hi_mask))
        {
            // Check for timeout
            if ((platform_get_time_ms() - start_ms) >= timeout_ms)
            {
                break;
            }
        }
    }

    if (lo_result != NULL)
    {
        *lo_result = lo_result_tmp;
    }

    if (hi_result != NULL)
    {
        *hi_result = hi_result_tmp;
    }
}


/**
 * get_relative_time_ticks - Compute relative time (in timestamp ticks)
 *                           for a given CIR sample index.
 * @index:               CIR sample index for which to compute the time.
 * @detected_peak_index: CIR index of the detected peak (reference point).
 * @detected_peak_time:  Absolute RX timestamp (in ticks) at the peak index.
 *
 * Return: Relative time in timestamp ticks for the given index
 *         with respect to the detected peak time.
 */
uint64_t get_relative_time_ticks(int index, int detected_peak_index, uint64_t detected_peak_time)
{
    // RX timestamp resolution: 1/(128*499.2x10^6) seconds per timestamp ticks 
    // CIR sample resolution: 1/(499.2x10^6) seconds per CIR sample
    // => each CIR sample corresponds to 128 timestamp ticks
    static const int ticks_per_cir_sample = 128;
    int diff = index - detected_peak_index;
    uint64_t relative_time = detected_peak_time + diff * ticks_per_cir_sample;
    return relative_time;
}

int waitforsysstatus_with_timeout(uint32_t mask, uint32_t timeout_count) {
    uint32_t status;
    uint32_t counter = 0;

    do {
        status = dwt_readsysstatuslo();
        if (status & mask) {
            return 1; // Success
        }
        counter++;
        
        // Add a small delay if necessary to reduce SPI traffic
        // SleepUs(100); 

    } while (counter < timeout_count);

    return 0; // Timeout failure
}