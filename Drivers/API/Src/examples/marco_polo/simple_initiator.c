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


/* 
    Print the CIR data in a format that can be easily plotted.
*/
static void print_cir(uint8_t *buf, int n_samples, dwt_cir_read_mode_e mode) {
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
    int start_index = detect_cir_start(data.mag, data.length, mag_norm_buf);
    if (start_index == -1) {
        test_run_info((unsigned char *)"No CIR start detected.\n");
        return 0;
    }

    // TODO: compare these and pick
    sprintf(str_to_print, "Detected CIR start at index: %d\n", start_index);
    test_run_info((unsigned char *)str_to_print);

    /* Temporarily use the peak index from the header as the CIR start */
    start_index = (int)data.fp_index_samples;
    
    sprintf(str_to_print, "vs FP index found at: %d\n", start_index);
    test_run_info((unsigned char *)str_to_print);

    float *rotated_mags = rotated_mags_buf;

    float fp_index = rotate_cir(data.mag, data.length, start_index, (float)data.fp_index_samples, rotated_mags);

    float peaks[MAX_PEAKS];
    size_t peak_count = detect_peaks(rotated_mags, data.length, fp_index, peaks, MAX_PEAKS, mag_norm_buf);

    /* Print detected peaks */
    test_run_info((unsigned char *)"Detected peaks at indices: [");
    for (size_t i = 0; i < peak_count; i++) {
        sprintf(str_to_print, "%f", peaks[i]);
        test_run_info((unsigned char *)str_to_print);
        if (i + 1 < peak_count) {
            test_run_info((unsigned char *)", ");
        }
    }
    test_run_info((unsigned char *)"]\n\n");

    uint64_t times[MAX_PEAKS];
    size_t time_count = get_relative_time_ticks((uint64_t)data.rx_minus_tx, fp_index, peaks, peak_count, times, MAX_PEAKS);

    for (size_t i = 0; i < time_count; i++) {
        tof_result_t result = tof_and_distance_from_absolute_rx(times[i], i);
        (void)result;
        sprintf(str_to_print, "Peak %zu: tau_dtu = %f, distance_m = %f\n", i, result.tau_dtu, result.distance_m);
        test_run_info((unsigned char *)str_to_print);
    }

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

    int counter = 0;
    /* Loop forever, send frame when a button is pressed. */
    while (TRUE)
    {
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

        sprintf(str_to_print,"INIT Frame Sent %d\r\n", counter);
        test_run_info((unsigned char *)str_to_print);

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

        if (status_reg & DWT_INT_RXFCG_BIT_MASK)
        {
            uint64_t rx_ts = get_rx_timestamp_u64();
            int64_t tx_rx_diff = (int64_t)rx_ts - (int64_t)tx_ts;

            sprintf(str_to_print, "RX_TS - TX_TS = %lld dtu\n", (long long)tx_rx_diff);
            test_run_info((unsigned char *)str_to_print);

            dwt_cirdiags_t diag;
            if (dwt_readdiagnostics_acc(&diag, DWT_ACC_IDX_IP_M) == DWT_SUCCESS)
            {
                sprintf(str_to_print,
                        "API diag: peakIndex=%u peakAmp=%lu FpIndex=0x%04X (%u.%u/64) accumCount=%u\r\n",
                        diag.peakIndex,
                        (unsigned long)diag.peakAmp,
                        diag.FpIndex,
                        diag.FpIndex >> 6,
                        diag.FpIndex & 0x3F,
                        diag.accumCount);
                test_run_info((unsigned char *)str_to_print);
            }

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
            print_cir(cir_buf, n_samples, modes);

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
            
            /* Clear good RX frame event in the DW IC status register. */
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
        }
        else
        {
            /* Clear RX error events in the DW IC status register. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        }
        counter++;
    }
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
