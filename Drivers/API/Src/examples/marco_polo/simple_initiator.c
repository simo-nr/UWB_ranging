/*! ----------------------------------------------------------------------------
 *  @file    simple_initiator.c
 *  @brief   Simple INITIATOR     example code
 *
 * @author Decawave
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <string.h>

#include "deca_private.h"
#include "helpers.h"
#include "load_data.h"
#include "analyzer.h"
#include <stdint.h>

#if defined(SIMPLE_INITIATOR)

#define CLEAR_ARRAY(array, size) for(int i = 0; i < size; i++) array[i] = 0

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "SIMPLE INITIATOR v1.0"

// sending stuff
static uint8_t tx_msg[] = { 0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E' };
#define FRAME_LENGTH (sizeof(tx_msg) + FCS_LEN) // The real length that is going to be transmitted
#define TX_DELAY_MS 3000

#define RX_RESP_TO_UUS 2000

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

static uint8_t cir_buf[DWT_CIR_LEN_MAX * 2 * 3];  /* A complex sample takes up to 2 32-bit words */

static char str_to_print[DWT_CIR_LEN_MAX * 2 * 3]; /* Buffer for printing*/

static float rotated_mags_buf[DWT_CIR_LEN_MAX];
static float mag_norm_buf[DWT_CIR_LEN_MAX];
static float cir_mag_buf[DWT_CIR_LEN_MAX];

extern dwt_config_t config_options;
extern dwt_txconfig_t txconfig_options;

#define TIMING_LOOP_COUNT 200U

// #define TIMING_TESTS
// #define FULL_TIMING
// #define LOOP_TIMING
// #define PROC_TIMING

#if defined(TIMING_TESTS) || defined(LOOP_TIMING)
uint32_t total_elapsed_cycles;

uint64_t total_loop_cycles_acc;
uint64_t total_tx_cycles_acc;
uint64_t total_wait_response_cycles_acc;
uint64_t total_diag_read_cycles_acc;
uint64_t total_cir_read_cycles_acc;
uint64_t total_cir_mag_cycles_acc;
uint64_t total_start_detect_cycles_acc;
uint64_t total_peak_detection_cycles_acc;
uint64_t total_distance_calc_cycles_acc;
uint64_t total_print_cycles_acc;
uint32_t measured_loop_count;

uint32_t proc_start_detect_cycles;
uint32_t proc_peak_detection_cycles;
uint32_t proc_distance_calc_cycles;

static void timing_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t timing_now_cycles(void)
{
    return DWT->CYCCNT;
}

static inline uint32_t cycles_to_us(uint32_t cycles)
{
    return (uint32_t)(((uint64_t)cycles * 1000000ULL) / (uint64_t)SystemCoreClock);
}

static inline uint32_t cycles_to_ms(uint32_t cycles)
{
    return (uint32_t)(((uint64_t)cycles * 1000ULL) / (uint64_t)SystemCoreClock);
}

/* Returns percentage scaled by 100 (e.g. 1234 means 12.34%). */
static inline uint32_t percent_x100(uint32_t part, uint32_t total)
{
    if (total == 0U)
    {
        return 0U;
    }
    return (uint32_t)(((uint64_t)part * 10000ULL) / (uint64_t)total);
}

static void print_timing_average_line(const char *name, uint64_t total_cycles, uint64_t avg_loop_cycles)
{
    uint64_t avg_cycles;
    uint32_t avg_us;
    uint32_t pct;

    if (measured_loop_count == 0U)
    {
        return;
    }

    avg_cycles = total_cycles / measured_loop_count;
    avg_us = cycles_to_us(avg_cycles);
    pct = percent_x100((uint32_t)avg_cycles, (uint32_t)avg_loop_cycles);

    printf("%-24s: %llu cycles, %lu us, %lu.%02lu %%\n",
           name,
           (unsigned long long)avg_cycles,
           (unsigned long)avg_us,
           (unsigned long)(pct / 100U),
           (unsigned long)(pct % 100U));
}

static void print_timing_averages(void)
{
    uint64_t avg_loop_cycles;
    uint32_t avg_loop_us;

    if (measured_loop_count == 0U)
    {
        printf("No successful measured loops.\n");
        return;
    }

    avg_loop_cycles = total_loop_cycles_acc / measured_loop_count;
    avg_loop_us = cycles_to_us(avg_loop_cycles);

    printf("\nTiming averages over %lu measured loops:\n", (unsigned long)measured_loop_count);
    printf("%-24s: %llu cycles, %lu us, 100.00 %%\n",
           "total loop",
           (unsigned long long)avg_loop_cycles,
           (unsigned long)avg_loop_us);

    print_timing_average_line("tx", total_tx_cycles_acc, avg_loop_cycles);
    print_timing_average_line("wait response", total_wait_response_cycles_acc, avg_loop_cycles);
    print_timing_average_line("read diagnostics", total_diag_read_cycles_acc, avg_loop_cycles);
    print_timing_average_line("read CIR", total_cir_read_cycles_acc, avg_loop_cycles);
    print_timing_average_line("CIR magnitude", total_cir_mag_cycles_acc, avg_loop_cycles);
    print_timing_average_line("detect CIR start", total_start_detect_cycles_acc, avg_loop_cycles);
    print_timing_average_line("peak detection", total_peak_detection_cycles_acc, avg_loop_cycles);
    print_timing_average_line("distance calc", total_distance_calc_cycles_acc, avg_loop_cycles);
    print_timing_average_line("printing", total_print_cycles_acc, avg_loop_cycles);
}
#endif


/* 
    Print the CIR data in a format that can be easily plotted.
*/
static void print_cir(uint8_t *buf, int n_samples, dwt_cir_read_mode_e mode) 
{
    uint8_t *ptr = buf;
    test_run_info((unsigned char *)"\n_________________________________\r\n");
    CLEAR_ARRAY(str_to_print, sizeof(str_to_print));

    if (mode == DWT_CIR_READ_FULL) {
        uint8_t lo_re, mid_re, hi_re, lo_img, mid_img, hi_img;
        uint8_t sign_re, sign_img;
        while(n_samples--){
            lo_re = *ptr++;
            mid_re = *ptr++;
            hi_re = *ptr++;
            sign_re = ((hi_re&0x80) == 0x80) ? 0xFF : 0;
            lo_img = *ptr++;
            mid_img = *ptr++;
            hi_img = *ptr++;
            sign_img = ((hi_img&0x80) == 0x80) ? 0xFF : 0;
            sprintf(str_to_print, "%ld,%ld,", (int32_t)((uint32_t)sign_re<<24 | (uint32_t)hi_re<<16 | (uint32_t)mid_re<<8 | lo_re), (int32_t)((uint32_t)sign_img<<24 | (uint32_t)hi_img<<16 | (uint32_t)mid_img<<8 | lo_img));
            test_run_info((unsigned char *)str_to_print);   
            nrf_delay_ms(1); // Delay to allow the UART to keep up with the data
        }
    }
    else {
        uint8_t lo_re, hi_re, lo_img, hi_img;
        while(n_samples--){
            lo_re = *ptr++;
            hi_re = *ptr++;
            lo_img = *ptr++;
            hi_img = *ptr++;
            sprintf(str_to_print, "%ld,%ld,", (int16_t)(hi_re<<8 | lo_re), (int16_t)(hi_img<<8 | lo_img));
            test_run_info((unsigned char *)str_to_print);
            nrf_delay_ms(1); // Delay to allow the UART to keep up with the data
        }
    }
    test_run_info((unsigned char *)"\n_________________________________\r\n");
}

static float cir_mag_from_buf(const uint8_t *ptr, dwt_cir_read_mode_e mode)
{
    int32_t re = 0;
    int32_t im = 0;
    float re_f;
    float im_f;

    if (mode == DWT_CIR_READ_FULL)
    {
        uint8_t lo_re = ptr[0];
        uint8_t mid_re = ptr[1];
        uint8_t hi_re = ptr[2];
        uint8_t lo_im = ptr[3];
        uint8_t mid_im = ptr[4];
        uint8_t hi_im = ptr[5];
        uint8_t sign_re = ((hi_re & 0x80U) == 0x80U) ? 0xFFU : 0U;
        uint8_t sign_im = ((hi_im & 0x80U) == 0x80U) ? 0xFFU : 0U;

        re = (int32_t)(((uint32_t)sign_re << 24) | ((uint32_t)hi_re << 16) | ((uint32_t)mid_re << 8) | lo_re);
        im = (int32_t)(((uint32_t)sign_im << 24) | ((uint32_t)hi_im << 16) | ((uint32_t)mid_im << 8) | lo_im);
    }
    else
    {
        uint8_t lo_re = ptr[0];
        uint8_t hi_re = ptr[1];
        uint8_t lo_im = ptr[2];
        uint8_t hi_im = ptr[3];

        re = (int16_t)(((uint16_t)hi_re << 8) | lo_re);
        im = (int16_t)(((uint16_t)hi_im << 8) | lo_im);
    }

    re_f = (float)re;
    im_f = (float)im;
    return sqrtf(re_f * re_f + im_f * im_f);
}

int calculate_distance(const cir_data_t data, ResponderPeak out_results[MAX_RESPONDERS]) 
{
    #if defined(TIMING_TESTS) && defined(PROC_TIMING)
    uint32_t t_proc_start = timing_now_cycles();
    #endif

    float noise_threshold;
    int start_index = detect_cir_start(data, mag_norm_buf, &noise_threshold);
    if (start_index == -1) {
        return 0;
    }

    float fp_index = (float)data.fp_index_samples - (float)start_index;
    if (fp_index < 0.0f) {
        fp_index += (float)data.length;
    }

    #if defined(TIMING_TESTS) && defined(PROC_TIMING)
    uint32_t t_start_detect_done = timing_now_cycles();
    #endif

    float interval_values[MAX_RESPONDERS + 1];
    size_t interval_count = find_interval_values(data.mag,
                                                data.length,
                                                fp_index,
                                                (uint64_t)data.rx_minus_tx,
                                                interval_values,
                                                MAX_RESPONDERS + 1);

    interval_peak_detection_wrapped_interval(mag_norm_buf,
                                             data.length,
                                             start_index,
                                             fp_index,
                                             interval_values,
                                             interval_count,
                                             noise_threshold,
                                             out_results);

    #if defined(TIMING_TESTS) && defined(PROC_TIMING)
    uint32_t t_peak_detection_done = timing_now_cycles();
    #endif

    uint64_t times[MAX_RESPONDERS];
    size_t time_count = get_distances((uint64_t)data.rx_minus_tx, fp_index, out_results, times);
    (void)time_count;

    #if defined(TIMING_TESTS) && defined(PROC_TIMING)
    uint32_t t_distance_calc_done = timing_now_cycles();
    proc_start_detect_cycles = t_start_detect_done - t_proc_start;
    proc_peak_detection_cycles = t_peak_detection_done - t_start_detect_done;
    proc_distance_calc_cycles = t_distance_calc_done - t_peak_detection_done;
    #endif
    
    return (int)interval_count;
}

/**
 * Application entry point.
 */
int simple_initiator(void)
{
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    uint32_t status_reg;
    /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */

    /* Print application name on the console. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW IC supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    /* Probe for the correct device driver. */
    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"PROBE FAILED");
        while (1) { };
    }

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

    if (dwt_initialise(DWT_READ_OTP_ALL) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED");
        while (1) { };
    }

    /* Enabling LEDs here for debug so that for each RX-enable the D2 LED will flash on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configure DW IC. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config_options))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };  
    }

    /* Configure the TX spectrum parameters (power PG delay and PG Count) */
    dwt_configuretxrf(&txconfig_options);

    /* Get the correct length of the Ipatov CIR*/
    uint32_t n_samples_ipatov;
    if (config_options.rxCode < PCODE_PRF64_START) {
        n_samples_ipatov = DWT_CIR_LEN_IP_PRF16;
    }
    else {
        n_samples_ipatov = DWT_CIR_LEN_IP_PRF64;
    }

    dwt_setrxtimeout(RX_RESP_TO_UUS);

    test_run_info((unsigned char *)"DW3000 initialized and configured. Starting main loop.\r\n");

    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

    printf("Waiting for button press to start...\n");
    
    //////////////// wait for button press to start the test ////////////////
    while (!bsp_board_button_state_get(0)) {
        Sleep(10);
    }
    while (bsp_board_button_state_get(0)) {
        Sleep(10);
    }

    printf("Button pressed, starting test!\n");

    #if defined(TIMING_TESTS) || defined(LOOP_TIMING)
    timing_init();
    uint64_t test_start_cycles = timing_now_cycles();

    uint32_t t_loop_start = 0;
    uint32_t t_tx_done = 0;
    uint32_t t_response_done = 0;
    uint32_t t_diag_done = 0;
    uint32_t t_cir_read_done = 0;
    uint32_t t_cir_mag_done = 0;
    uint32_t t_process_done = 0;
    uint32_t t_print_done = 0;

    bool loop_success = false;
    #endif

    int counter = 0;
    /* Run one warm-up loop plus TIMING_LOOP_COUNT measured loops. */
    while (counter < ((int)TIMING_LOOP_COUNT + 1))
    {
        printf("Starting round %d...\n", counter);
        printf("Waiting for button press to start round...\n");
        /* Wait for Button 1 (index 0) to be pressed. */
        while (!bsp_board_button_state_get(0))
        {
            /* Small delay to avoid a tight busy loop. */
            Sleep(10);
        }

        /* Simple debounce: wait until the button is released. */
        while (bsp_board_button_state_get(0))
        {
            Sleep(10);
        }
        printf("Button pressed, sending frame!\n");

        #if defined(TIMING_TESTS) || defined(LOOP_TIMING)
        t_loop_start = timing_now_cycles();
        #endif
        #if defined(TIMING_TESTS) || defined(LOOP_TIMING)
        t_tx_done = t_loop_start;
        t_response_done = t_loop_start;
        t_diag_done = t_loop_start;
        t_cir_read_done = t_loop_start;
        t_cir_mag_done = t_loop_start;
        t_process_done = t_loop_start;
        t_print_done = t_loop_start;
        #endif

        /////////////// sending frame ///////////////

        /* Write frame data to DW IC and prepare transmission. See NOTE 3 below.*/
        dwt_writetxdata(FRAME_LENGTH - FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. */

        /* In this example since the length of the transmitted frame does not change,
         * nor the other parameters of the dwt_writetxfctrl function, the
         * dwt_writetxfctrl call could be outside the main while(1) loop.
         */
        dwt_writetxfctrl(FRAME_LENGTH, 0, 0); /* Zero offset in TX buffer, no ranging. */

        /* Start transmission. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        /* Poll DW IC until TX frame sent event set. See NOTE 4 below.
         * STATUS register is 4 bytes long but, as the event we are looking
         * at is in the first byte of the register, we can use this simplest
         * API function to access it.*/
        waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
        
        uint64_t tx_ts = get_tx_timestamp_u64();

        /* Clear TX frame sent event. */
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        #if defined(TIMING_TESTS) || defined(LOOP_TIMING)
        t_tx_done = timing_now_cycles();
        #endif

        /* TESTING BREAKPOINT LOCATION #1 */

        /* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
         * the RX buffer.
         * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
         * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
        memset(rx_buffer, 0, sizeof(rx_buffer));

        /* Activate reception immediately. See NOTE 2 below. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 3 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it. */
        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

        #if defined(TIMING_TESTS) || defined(LOOP_TIMING)
        t_response_done = timing_now_cycles();
        #endif

        if (status_reg & DWT_INT_RXFCG_BIT_MASK)
        {
            uint64_t rx_ts = get_rx_timestamp_u64();
            int64_t tx_rx_diff = (int64_t)rx_ts - (int64_t)tx_ts;

            dwt_cirdiags_t diag;
            dwt_readdiagnostics_acc(&diag, DWT_ACC_IDX_IP_M);

            #if defined(TIMING_TESTS) || defined(LOOP_TIMING)
            t_diag_done = timing_now_cycles();
            loop_success = true;
            #endif

            /* Start reading CIR data from Ipatov offset */
            dwt_acc_idx_e acc_idx = DWT_ACC_IDX_IP_M ;
            
            /*
                Choose the mode you want to print the data:
                    - DWT_CIR_READ_FULL
                    - DWT_CIR_READ_LO
                    - DWT_CIR_READ_MID
                    - DWT_CIR_READ_HI
            */
            dwt_cir_read_mode_e modes = DWT_CIR_READ_FULL; 

            /* Ipatov data */
            int n_samples = n_samples_ipatov;
            dwt_readcir((uint32_t*)cir_buf, acc_idx, 0, n_samples, modes);

            #if defined(TIMING_TESTS) || defined(LOOP_TIMING)
            t_cir_read_done = timing_now_cycles();
            #endif

            #define PRINT_CIR
            #if defined(PRINT_CIR)
            sprintf(str_to_print, "RX_TS - TX_TS = %lld dtu\n", (long long)tx_rx_diff);
            test_run_info((unsigned char *)str_to_print);
            sprintf(str_to_print,
                    "API diag: peakIndex=%u peakAmp=%lu FpIndex=0x%04X (%u.%u/64) accumCount=%u\r\n",
                    diag.peakIndex,
                    (unsigned long)diag.peakAmp,
                    diag.FpIndex,
                    diag.FpIndex >> 6,
                    diag.FpIndex & 0x3F,
                    diag.accumCount);
            test_run_info((unsigned char *)str_to_print);
            print_cir(cir_buf, n_samples, modes);
            #endif

            /* make cir_data_t struct and calculate distance */
            cir_data_t cir_data;
            cir_data.mag = cir_mag_buf;
            cir_data.length = n_samples;
            /* convert Q10.6 diag.FpIndex to float */
            cir_data.fp_index_samples = (float)diag.FpIndex / 64.0f;
            cir_data.rx_minus_tx = (int)(rx_ts - tx_ts);
            cir_data.peak_amp = (float)diag.peakAmp * 0.25f;

            for (int i = 0; i < n_samples; i++) {
                cir_data.mag[i] = cir_mag_from_buf(&cir_buf[i * ((modes == DWT_CIR_READ_FULL) ? 6 : 4)], modes);
            }

            #if defined(TIMING_TESTS) || defined(LOOP_TIMING)
            t_cir_mag_done = timing_now_cycles();
            #endif

            ResponderPeak distance_results[MAX_RESPONDERS];
            int detected_count = calculate_distance(cir_data, distance_results);
            (void)detected_count;

            #if defined(TIMING_TESTS) || defined(LOOP_TIMING)
            t_process_done = timing_now_cycles();
            #endif

            printf("Test %d: [", counter);
            for (int i = 0; i < MAX_RESPONDERS; i++) {
                printf("%.3f, ", distance_results[i].distance);
            }
            printf("]\n");

            #if defined(TIMING_TESTS) || defined(LOOP_TIMING)
            t_print_done = timing_now_cycles();
            #endif

            /* Clear good RX frame event in the DW IC status register. */
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
        }
        else
        {
            /* Clear RX error events in the DW IC status register. */
            printf("RX failed, counter=%d, status_reg = 0x%08lX\n", counter, (unsigned long)status_reg);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        }

        #if defined(TIMING_TESTS) || defined(LOOP_TIMING)
        if (loop_success && counter > 0)
        {
            total_loop_cycles_acc += (uint32_t)(t_print_done - t_loop_start);
            total_tx_cycles_acc += (uint32_t)(t_tx_done - t_loop_start);
            total_wait_response_cycles_acc += (uint32_t)(t_response_done - t_tx_done);
            total_diag_read_cycles_acc += (uint32_t)(t_diag_done - t_response_done);
            total_cir_read_cycles_acc += (uint32_t)(t_cir_read_done - t_diag_done);
            total_cir_mag_cycles_acc += (uint32_t)(t_cir_mag_done - t_cir_read_done);

            total_start_detect_cycles_acc += proc_start_detect_cycles;
            total_peak_detection_cycles_acc += proc_peak_detection_cycles;
            total_distance_calc_cycles_acc += proc_distance_calc_cycles;

            total_print_cycles_acc += (uint32_t)(t_print_done - t_process_done);
            measured_loop_count++;
        }
        #endif

        /* sleep to let the UART finish printing */
        nrf_delay_ms(5);
        counter++;
    }

    #if defined(TIMING_TESTS) || defined(LOOP_TIMING)
    uint64_t test_end_cycles = timing_now_cycles();
    uint64_t total_test_cycles = test_end_cycles - test_start_cycles;
    uint32_t avg_loop_cycles = (uint32_t)(total_test_cycles / TIMING_LOOP_COUNT);
    printf("Average cycles per loop: %lu (%lu us)\n", avg_loop_cycles, cycles_to_us(avg_loop_cycles));

    print_timing_averages();
    #endif

    printf("Test completed!\n");

    return 0;
}
#endif
