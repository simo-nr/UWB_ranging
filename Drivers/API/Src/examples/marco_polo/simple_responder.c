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

#if defined(SIMPLE_RESPONDER)

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "SIMPLE RESPONDER v1.0"

// sending stuff
static uint8_t tx_msg[] = { 0xC5, 0, 'R', 'E', 'S', 'P', 'O', 'N', 'S', 'E' };
#define FRAME_LENGTH (sizeof(tx_msg) + FCS_LEN) // The real length that is going to be transmitted
// #define TX_DELAY_MS 500


/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

extern dwt_config_t config_options;
extern dwt_txconfig_t txconfig_options;


/**
 * Application entry point.
 */
int simple_responder(void)
{
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    uint32_t status_reg;
    /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
    uint16_t frame_len;

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

    /* Loop forever receiving frames. */
    while (TRUE)
    {
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
        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR), 0);

        if (status_reg & DWT_INT_RXFCG_BIT_MASK)
        {
            /* A frame has been received, copy it to our local buffer. */
            frame_len = dwt_getframelength(0);
            if (frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rx_buffer, frame_len - FCS_LEN, 0); /* No need to read the FCS/CRC. */
            }

            /* Clear good RX frame event in the DW IC status register. */
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

            test_run_info((unsigned char *)"Frame Received");

            // sending response
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

            /* Clear TX frame sent event. */
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

            test_run_info((unsigned char *)"RESPONSE Frame Sent");
        }
        else
        {
            /* Clear RX error events in the DW IC status register. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        }
        // /////////////// sending frame /////////////

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

        // test_run_info((unsigned char *)"INIT Frame Sent");

        // /* TESTING BREAKPOINT LOCATION #1 */

        // /* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
        //  * the RX buffer.
        //  * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
        //  * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
        // memset(rx_buffer, 0, sizeof(rx_buffer));

        // /* Activate reception immediately. See NOTE 2 below. */
        // dwt_rxenable(DWT_START_RX_IMMEDIATE);

        // /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 3 below.
        //  * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
        //  * function to access it. */
        // waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR), 0);

        // if (status_reg & DWT_INT_RXFCG_BIT_MASK)
        // {
        //     /* A frame has been received, copy it to our local buffer. */
        //     frame_len = dwt_getframelength(0);
        //     if (frame_len <= FRAME_LEN_MAX)
        //     {
        //         dwt_readrxdata(rx_buffer, frame_len - FCS_LEN, 0); /* No need to read the FCS/CRC. */
        //     }

        //     /* Clear good RX frame event in the DW IC status register. */
        //     dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

        //     test_run_info((unsigned char *)"Frame Received");
        //     Sleep(TX_DELAY_MS);
        // }
        // else
        // {
        //     /* Clear RX error events in the DW IC status register. */
        //     dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        // }
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
