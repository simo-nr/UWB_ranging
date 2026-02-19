/*! ----------------------------------------------------------------------------
 *  @file    TX_and_read_cir.c
 *  @brief   TX AND READ CIR example code
 *
 * @author Decawave
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

// #include <stdio.h>
#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>

#if defined(TX_AND_READ_CIR)

extern void test_run_info(unsigned char *data);

#define ARR_SZ(arr) (sizeof(arr)/sizeof(arr[0]))
#define CLEAR_ARRAY(array, size) for(int i = 0; i < size; i++) array[i] = 0

/* Example application name */
#define APP_NAME "TX AND READ CIR v1.0"


/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 */
static uint8_t tx_msg[] = { 0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E' };
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

#define FRAME_LENGTH (sizeof(tx_msg) + FCS_LEN) // The real length that is going to be transmitted

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 500


static uint8_t cir_buf[DWT_CIR_LEN_MAX * 2 * 3];  /* A complex sample takes up to 2 32-bit words */
static char str_to_print[DWT_CIR_LEN_MAX * 2 * 3]; /* Buffer mostly used to print the CIR data in print_cir*/
extern dwt_config_t config_options;

/* 
    Print the CIR data in a format that can be easily plotted.
*/
static void print_cir(uint8_t *buf, int n_samples, dwt_cir_read_mode_e mode) {
    uint8_t *ptr = buf;
    test_run_info((unsigned char *)"\n&_________________________________\r\n");
    CLEAR_ARRAY(str_to_print, sizeof(str_to_print));

    if (mode == DWT_CIR_READ_FULL) {
        uint8_t lo_re, mid_re, hi_re, lo_img, mid_img, hi_img;
        uint8_t sign_re, sign_img;
        int32_t re, im;

        while(n_samples--){
            lo_re = *ptr++;
            mid_re = *ptr++;
            hi_re = *ptr++;
            sign_re = ((hi_re & 0x80) == 0x80) ? 0xFF : 0;
            lo_img = *ptr++;
            mid_img = *ptr++;
            hi_img = *ptr++;
            sign_img = ((hi_img & 0x80) == 0x80) ? 0xFF : 0;

            re = (int32_t)((uint32_t)sign_re<<24 | 
                           (uint32_t)hi_re<<16 | 
                           (uint32_t)mid_re<<8 | 
                           lo_re);
            im = (int32_t)((uint32_t)sign_img<<24 | 
                           (uint32_t)hi_img<<16 | 
                           (uint32_t)mid_img<<8 | 
                           lo_img);

            sprintf(str_to_print, "%ld,%ld,", re, im);
            test_run_info((unsigned char *)str_to_print);   
            nrf_delay_ms(1); // Delay to allow the UART to keep up with the data
        }
    }
    else {
        uint8_t lo_re, hi_re, lo_img, hi_img;
        int16_t re, im;

        while (n_samples--) {
            lo_re = *ptr++;
            hi_re = *ptr++;
            lo_img = *ptr++;
            hi_img = *ptr++;

            re = (int16_t)((hi_re << 8) | lo_re);
            im = (int16_t)((hi_img << 8) | lo_img);

            sprintf(str_to_print, "%d,%d\n", (int)re, (int)im);
            test_run_info((unsigned char *)str_to_print);
            nrf_delay_ms(1);
        }
    }
    test_run_info((unsigned char *)"\n&_________________________________\r\n");
}

/**
 * Application entry point.
 */
int TX_and_read_cir(void)
{
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

    int res;

    /* Configure DW IC. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config_options))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    /* Get the correct length of the Ipatov CIR*/
    uint32_t n_samples_ipatov;
    if (config_options.rxCode < PCODE_PRF64_START) {
        n_samples_ipatov = DWT_CIR_LEN_IP_PRF16;
    }
    else {
        n_samples_ipatov = DWT_CIR_LEN_IP_PRF64;
    }

    /* Loop forever receiving frames. */
    while (TRUE)
    {
        // /* Write frame data to DW IC and prepare transmission. See NOTE 3 below.*/
        // dwt_writetxdata(FRAME_LENGTH - FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. */

        // /* In this example since the length of the transmitted frame does not change,
        //  * nor the other parameters of the dwt_writetxfctrl function, the
        //  * dwt_writetxfctrl call could be outside the main while(1) loop.
        //  */
        // dwt_writetxfctrl(FRAME_LENGTH, 0, 0); /* Zero offset in TX buffer, no ranging. */

        // /* Start transmission. */
        // dwt_starttx(DWT_START_TX_IMMEDIATE);
        // /* Poll DW IC until TX frame sent event set. See NOTE 4 below.
        //  * STATUS register is 4 bytes long but, as the event we are looking
        //  * at is in the first byte of the register, we can use this simplest
        //  * API function to access it.*/
        // waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);

        // /* Clear TX frame sent event. */
        // dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        // test_run_info((unsigned char *)"TX Frame Sent");
        test_run_info((unsigned char *)"TX Frame NOT Sent");

        Sleep(1000);

        sprintf(str_to_print,"Printing Ipatov CIR\r\n");
        test_run_info((unsigned char *)str_to_print);
        
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
        print_cir(cir_buf, n_samples, modes);

        /* Execute a delay between transmissions. */
        Sleep(TX_DELAY_MS);

        /* Increment the blink frame sequence number (modulo 256). */
        tx_msg[BLINK_FRAME_SN_IDX]++;
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
