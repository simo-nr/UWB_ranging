/*! ----------------------------------------------------------------------------
 *  @file    simple_rx.c
 *  @brief   Simple RX example code
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
#include "analyzer.h"
#include "load_data.h"
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

// #define TIMING_TESTS
#if defined(TIMING_TESTS)
uint32_t t_start;
uint32_t t_frame_rec;
uint32_t t_diag_read;
uint32_t t_cir_read;
uint32_t t_cir_processed;
uint32_t t_end;

uint32_t total_elapsed_cycles;
uint32_t total_elapsed_us;
uint32_t total_elapsed_ms;

uint32_t time_to_read_frame;
uint32_t time_to_read_diag;
uint32_t time_to_read_cir;
uint32_t time_to_process_cir;

uint32_t time_to_read_frame_us;
uint32_t time_to_read_diag_us;
uint32_t time_to_read_cir_us;
uint32_t time_to_process_cir_us;

uint32_t t_start_processing;
uint32_t t_cir_start_detect;
uint32_t t_cir_rotation;
uint32_t t_peak_detection;
uint32_t t_relative_time;
uint32_t t_distance_calc;

uint32_t start_detected;
uint32_t cir_rotation;
uint32_t peak_detection;
uint32_t relative_time_calc;
uint32_t distance_calc;

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
            // sprintf(str_to_print, "%ld,%ld,", "%d,%d,", (int16_t)(hi_re<<8 | lo_re), (int16_t)(hi_img<<8 | lo_img));
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

int calculate_distance(cir_data_t data) {
    #if defined(TIMING_TESTS)
    t_start_processing = timing_now_cycles();
    #endif

    float noise_threshold;
    int start_index = detect_cir_start(data.mag, data.length, mag_norm_buf, &noise_threshold);
    if (start_index == -1) {
        // test_run_info((unsigned char *)"No CIR start detected.\n");
        return 0;
    }

    #if defined(TIMING_TESTS)
    t_cir_start_detect = timing_now_cycles();
    #endif

    float *rotated_mags = rotated_mags_buf;

    float fp_index = rotate_cir(data.mag, data.length, start_index, (float)data.fp_index_samples, rotated_mags);

    #if defined(TIMING_TESTS)
    t_cir_rotation = timing_now_cycles();
    #endif

    // int found[MAX_RESPONDERS];
    ResponderPeak results[MAX_RESPONDERS];
    interval_peak_detection(rotated_mags, data.length, fp_index, noise_threshold, results, mag_norm_buf);

    #if defined(TIMING_TESTS)
    t_peak_detection = timing_now_cycles();
    #endif

    uint64_t times[MAX_RESPONDERS];
    size_t time_count = get_relative_time_ticks((uint64_t)data.rx_minus_tx, fp_index, results, MAX_RESPONDERS, times, MAX_RESPONDERS);

    #if defined(TIMING_TESTS)
    t_relative_time = timing_now_cycles();
    #endif

    printf("[");
    for(int i = 0; i < MAX_RESPONDERS; i++) {
        if(results[i].valid) {
            tof_result_t tof_result = tof_and_distance_from_absolute_rx(results[i].time, results[i].responder_id);
            // results[i].time = (int)tof_result.tau_dtu;
            results[i].distance = tof_result.distance_m;
        } 
        printf("%.3f, ", results[i].distance);
        // sprintf(str_to_print, (unsigned char *)"%d, ", results[i].distance);
        // test_run_info(str_to_print);
    }
    printf("]\n");

    // for (size_t i = 0; i < time_count; i++) {
    //     tof_result_t result = tof_and_distance_from_absolute_rx(times[i], i);
    //     (void)result;
    //     sprintf(str_to_print, "Peak %zu: tau_dtu = %f, distance_m = %f", i, result.tau_dtu, result.distance_m);
    //     test_run_info((unsigned char *)str_to_print);
    // }
    // test_run_info((unsigned char *)"");

    #if defined(TIMING_TESTS)
    t_distance_calc = timing_now_cycles();
    #endif

    return 0;
}

/**
 * Application entry point.
 */
int simple_initiator(void)
{
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    uint32_t status_reg;
    // /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
    // uint16_t frame_len;

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

    // // test print, hoping for MINDIAG = 0 
    // uint32_t cia_conf = read_u32_le_reg(0x0E, 0x00);
    // sprintf(str_to_print,
    //         "CIA_CONF=0x%08lX  MINDIAG=%lu\r\n",
    //         (unsigned long)cia_conf,
    //         (unsigned long)((cia_conf >> 20) & 1U));
    // test_run_info((unsigned char *)str_to_print);

    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

    // // test print
    // cia_conf = read_u32_le_reg(0x0E, 0x00);
    // sprintf(str_to_print,
    //         "CIA_CONF=0x%08lX  MINDIAG=%lu\r\n",
    //         (unsigned long)cia_conf,
    //         (unsigned long)((cia_conf >> 20) & 1U));
    // test_run_info((unsigned char *)str_to_print);

    printf("Waiting for button press to start...\n");
    
    //////////////// wait for button press to start the test ////////////////
    while (!bsp_board_button_state_get(0)) {
        Sleep(10);
    }
    while (bsp_board_button_state_get(0)) {
        Sleep(10);
    }

    printf("Button pressed, starting test!\n");

    #if defined(TIMING_TESTS)
    timing_init();
    t_start = timing_now_cycles();
    #endif

    int counter = 0;
    /* Loop forever, send frame when a button is pressed. */
    while (counter < 1100)
    {
        printf("Starting round %d...\n", counter);
        printf("Waiting for button press to start...\n");
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

        /////////////// sending frame /////////////

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
        
        // /* Read TX timestamp here */
        // uint8_t ts[5];
        // uint64_t tx_time = 0;

        // dwt_readtxtimestamp(ts);

        // tx_time = ((uint64_t)ts[0]) |
        //         ((uint64_t)ts[1] << 8) |
        //         ((uint64_t)ts[2] << 16) |
        //         ((uint64_t)ts[3] << 24) |
        //         ((uint64_t)ts[4] << 32);

        uint64_t tx_ts = get_tx_timestamp_u64();

        /* Clear TX frame sent event. */
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        // sprintf(str_to_print,"INIT Frame Sent %d\r\n", counter);
        // test_run_info((unsigned char *)str_to_print);

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
        #if defined(TIMING_TESTS)
        t_frame_rec = timing_now_cycles();
        #endif

        if (status_reg & DWT_INT_RXFCG_BIT_MASK)
        {
            uint64_t rx_ts = get_rx_timestamp_u64();
            int64_t tx_rx_diff = (int64_t)rx_ts - (int64_t)tx_ts;

            // sprintf(str_to_print, "RX_TS - TX_TS = %lld dtu\n", (long long)tx_rx_diff);
            // test_run_info((unsigned char *)str_to_print);

            dwt_cirdiags_t diag;
            if (dwt_readdiagnostics_acc(&diag, DWT_ACC_IDX_IP_M) == DWT_SUCCESS)
            {
                // sprintf(str_to_print,
                //         "API diag: peakIndex=%u peakAmp=%lu FpIndex=0x%04X (%u.%u/64) accumCount=%u\r\n",
                //         diag.peakIndex,
                //         (unsigned long)diag.peakAmp,
                //         diag.FpIndex,
                //         diag.FpIndex >> 6,
                //         diag.FpIndex & 0x3F,
                //         diag.accumCount);
                // test_run_info((unsigned char *)str_to_print);
            }
            #if defined(TIMING_TESTS)
            t_diag_read = timing_now_cycles();
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
            // find_and_print_cir_peak_from_buffer(cir_buf, n_samples, modes);
            
            #if defined(TIMING_TESTS)
            t_cir_read = timing_now_cycles();
            #endif

            // #define PRINT_CIR
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

            // make cir_data_t struct and calculate distance
            cir_data_t cir_data;
            cir_data.mag = cir_mag_buf;
            cir_data.length = n_samples;
            // convert Q10.6 diag.FpIndex to float
            cir_data.fp_index_samples = (float)diag.FpIndex / 64.0f;
            cir_data.rx_minus_tx = (int)(rx_ts - tx_ts);

            for (int i = 0; i < n_samples; i++) {
                cir_data.mag[i] = cir_mag_from_buf(&cir_buf[i * ((modes == DWT_CIR_READ_FULL) ? 6 : 4)], modes);
            }
            calculate_distance(cir_data);

            #if defined(TIMING_TESTS)
            t_cir_processed = timing_now_cycles();
            #endif
            
            /* Clear good RX frame event in the DW IC status register. */
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
        }
        else
        {
            /* Clear RX error events in the DW IC status register. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        }
        // sleep to let the UART finish printing
        nrf_delay_ms(30);
        counter++;
    }
    
    #if defined(TIMING_TESTS)
    t_end = timing_now_cycles();
    total_elapsed_cycles = t_end - t_start;
    total_elapsed_us = cycles_to_us(total_elapsed_cycles);
    total_elapsed_ms = cycles_to_ms(total_elapsed_cycles);

    time_to_read_frame = (t_frame_rec - t_start);
    time_to_read_diag = (t_diag_read - t_frame_rec);
    time_to_read_cir = (t_cir_read - t_diag_read);
    time_to_process_cir = (t_cir_processed - t_cir_read);

    start_detected = (t_cir_start_detect - t_start_processing);
    cir_rotation = (t_cir_rotation - t_cir_start_detect);
    peak_detection = (t_peak_detection - t_cir_rotation);
    relative_time_calc = (t_relative_time - t_peak_detection);
    distance_calc = (t_distance_calc - t_relative_time);

    uint32_t read_frame_pct_x100 = percent_x100(time_to_read_frame, total_elapsed_cycles);
    uint32_t read_diag_pct_x100 = percent_x100(time_to_read_diag, total_elapsed_cycles);
    uint32_t read_cir_pct_x100 = percent_x100(time_to_read_cir, total_elapsed_cycles);
    uint32_t process_cir_pct_x100 = percent_x100(time_to_process_cir, total_elapsed_cycles);

    uint32_t processing_total_cycles = t_distance_calc - t_start_processing;
    uint32_t start_detected_pct_x100 = percent_x100(start_detected, processing_total_cycles);
    uint32_t cir_rotation_pct_x100 = percent_x100(cir_rotation, processing_total_cycles);
    uint32_t peak_detection_pct_x100 = percent_x100(peak_detection, processing_total_cycles);
    uint32_t relative_time_pct_x100 = percent_x100(relative_time_calc, processing_total_cycles);
    uint32_t distance_calc_pct_x100 = percent_x100(distance_calc, processing_total_cycles);

    // time_to_read_frame_us = cycles_to_us(time_to_read_frame);
    // time_to_read_diag_us = cycles_to_us(time_to_read_diag);
    // time_to_read_cir_us = cycles_to_us(time_to_read_cir);
    // time_to_process_cir_us = cycles_to_us(time_to_process_cir);

    sprintf(str_to_print,
            "Total execution time: %lu cycles (%lu us, %lu ms)\r\n",
            (unsigned long)total_elapsed_cycles,
            (unsigned long)total_elapsed_us,
            (unsigned long)total_elapsed_ms);
    test_run_info((unsigned char *)str_to_print);

        sprintf(str_to_print,
            "Processing split (inside CIR processing): Start detect=%lu cycles (%lu.%02lu%%), Rotation=%lu cycles (%lu.%02lu%%), Peak=%lu cycles (%lu.%02lu%%), Relative time=%lu cycles (%lu.%02lu%%), Distance=%lu cycles (%lu.%02lu%%)\r\n",
            (unsigned long)start_detected,
            (unsigned long)(start_detected_pct_x100 / 100U),
            (unsigned long)(start_detected_pct_x100 % 100U),
            (unsigned long)cir_rotation,
            (unsigned long)(cir_rotation_pct_x100 / 100U),
            (unsigned long)(cir_rotation_pct_x100 % 100U),
            (unsigned long)peak_detection,
            (unsigned long)(peak_detection_pct_x100 / 100U),
            (unsigned long)(peak_detection_pct_x100 % 100U),
            (unsigned long)relative_time_calc,
            (unsigned long)(relative_time_pct_x100 / 100U),
            (unsigned long)(relative_time_pct_x100 % 100U),
            (unsigned long)distance_calc,
            (unsigned long)(distance_calc_pct_x100 / 100U),
            (unsigned long)(distance_calc_pct_x100 % 100U));
    test_run_info((unsigned char *)str_to_print);

    sprintf(str_to_print,
            "Main loop split (of total runtime): Read frame=%lu cycles (%lu.%02lu%%), Read diag=%lu cycles (%lu.%02lu%%), Read CIR=%lu cycles (%lu.%02lu%%), Process CIR=%lu cycles (%lu.%02lu%%)\r\n",
            (unsigned long)time_to_read_frame,
            (unsigned long)(read_frame_pct_x100 / 100U),
            (unsigned long)(read_frame_pct_x100 % 100U),
            (unsigned long)time_to_read_diag,
            (unsigned long)(read_diag_pct_x100 / 100U),
            (unsigned long)(read_diag_pct_x100 % 100U),
            (unsigned long)time_to_read_cir,
            (unsigned long)(read_cir_pct_x100 / 100U),
            (unsigned long)(read_cir_pct_x100 % 100U),
            (unsigned long)time_to_process_cir,
            (unsigned long)(process_cir_pct_x100 / 100U),
            (unsigned long)(process_cir_pct_x100 % 100U));
    test_run_info((unsigned char *)str_to_print);
    #endif

    printf("Test completed!\n");

    return 0;
}
#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW IC supports an extended
 *    frame length (up to 1023 bytes long) mode which is not used in this example.
 * 2. Manual reception activation is performed here but DW IC offers several features that can be used to handle more complex scenarios or to
 *    optimise system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 3. We use polled mode of operation here to keep the example as simple as possible, but RXFCG and error/timeout status events can be used to generate
 *    interrupts. Please refer to DW IC User Manual for more details on "interrupts".
 ****************************************************************************************************************************************************/
