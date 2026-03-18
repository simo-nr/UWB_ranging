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

#if defined(SENDING_TIMESTAMPS)

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "SENDING TIMESTAMPS v1.0"

// sending stuff
#define RESP_MSG_COMMON_LEN 10
#define RESP_MSG_POLL_RX_TS_IDX RESP_MSG_COMMON_LEN
#define RESP_MSG_RESP_TX_TS_IDX (RESP_MSG_POLL_RX_TS_IDX + 5)
#define RESP_MSG_DELAY_UUS_IDX (RESP_MSG_RESP_TX_TS_IDX + 5)
#define RESP_MSG_DELAY_UUS_LEN 4
#define RESP_TX_DELAY_UUS 1500

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

static uint8_t tx_msg[] = {
    0xC5, 0, 'R', 'E', 'S', 'P', 'O', 'N', 'S', 'E',
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0
};
#define FRAME_LENGTH (sizeof(tx_msg) + FCS_LEN) // The real length that is going to be transmitted

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

extern dwt_config_t config_options;
extern dwt_txconfig_t txconfig_options;


static void write_timestamp_to_msg(uint8_t *buffer, uint64_t ts)
{
    buffer[0] = (uint8_t)ts;
    buffer[1] = (uint8_t)(ts >> 8);
    buffer[2] = (uint8_t)(ts >> 16);
    buffer[3] = (uint8_t)(ts >> 24);
    buffer[4] = (uint8_t)(ts >> 32);
}

static void write_u32_to_msg(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8);
    buffer[2] = (uint8_t)(value >> 16);
    buffer[3] = (uint8_t)(value >> 24);
}

/**
 * Application entry point.
 */
int sending_timestamps(void)
{
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    uint32_t status_reg;
    /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
    uint16_t frame_len;

    uint64_t poll_rx_ts;
    uint64_t resp_tx_ts;
    uint64_t actual_tx_ts; // for scheduling tx
    uint32_t resp_tx_time;

    int16_t clock_offset_raw;
    double clock_offset_frac;
    double delay_scale;
    uint64_t corrected_delay_dtu;

    char str_to_print[200];

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

            poll_rx_ts = get_rx_timestamp_u64();

            clock_offset_raw = dwt_readclockoffset(); // TODO: remove this and use carrier integrator instead
            // clock is x parts per million off
            clock_offset_frac = ((double)clock_offset_raw / 67108864.0);
            
            uint8_t trim_base = dwt_getxtaltrim();
            int32_t carrier_integrator = dwt_readcarrierintegrator();

            /*
            * Positive clock_offset_ppm means responder(local) clock is faster than
            * initiator(remote). So to produce the same delay in the initiator domain,
            * we need a slightly larger local delay.
            */
            delay_scale = 1.0 + clock_offset_frac;
            corrected_delay_dtu = (uint64_t)(((double)RESP_TX_DELAY_UUS * (double)UUS_TO_DWT_TIME) * delay_scale);

            resp_tx_time = (uint32_t)((poll_rx_ts + corrected_delay_dtu) >> 8);
            dwt_setdelayedtrxtime(resp_tx_time);

            /* The actual 40-bit TX timestamp corresponding to the delayed transmission. */
            resp_tx_ts = ((((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY);

            write_timestamp_to_msg(&tx_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
            write_timestamp_to_msg(&tx_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);
            uint32_t scheduled_delay_dtu = (uint32_t)(resp_tx_ts - poll_rx_ts);
            write_u32_to_msg(&tx_msg[RESP_MSG_DELAY_UUS_IDX], scheduled_delay_dtu);

            /* Write frame data to DW IC and prepare delayed transmission. */
            dwt_writetxdata(FRAME_LENGTH - FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(FRAME_LENGTH, 0, 0); /* Zero offset in TX buffer, no ranging. */

            if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS)
            {
                waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
                dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

                actual_tx_ts = get_tx_timestamp_u64();

                double clock_offset_ppm = clock_offset_frac * 1e6;
                // print xtal trim, clock offset and delay info
                sprintf(str_to_print,
                        "trim_base=%u clock_offset_raw=%d clock_offset_ppm=%.3f",
                        (unsigned)trim_base,
                        (int)clock_offset_raw,
                        clock_offset_ppm);
                test_run_info((unsigned char *)str_to_print);

                sprintf(str_to_print,
                        "poll_rx_ts=0x%010llX clkoff_raw=%d clkoff_ppm=%.3f corrected_delay_dtu=%llu scheduled_delay_dtu=%u resp_tx_ts=0x%010llX\r\n",
                        (unsigned long long)poll_rx_ts,
                        (int)clock_offset_raw,
                        clock_offset_ppm,
                        (unsigned long long)corrected_delay_dtu,
                        (unsigned int)scheduled_delay_dtu,
                        (unsigned long long)resp_tx_ts);
                test_run_info((unsigned char *)str_to_print);

                uint64_t desired_tx_ts = poll_rx_ts + corrected_delay_dtu + TX_ANT_DLY;
                int64_t epsilon_dtu = (int64_t)resp_tx_ts - (int64_t)desired_tx_ts;
                double epsilon_ns = epsilon_dtu * (1e9 / (128.0 * 499.2e6));

                sprintf(str_to_print,
                        "actual_tx_ts=0x%010llX desired_tx_ts=0x%010llX epsilon=%lld dtu (%.3f ns)\r\n",
                        (unsigned long long)actual_tx_ts,
                        (unsigned long long)desired_tx_ts,
                        (long long)epsilon_dtu,
                        epsilon_ns);
                test_run_info((unsigned char *)str_to_print);

                sprintf(str_to_print,
                        "RESPONSE Frame Sent actual_tx_ts=0x%010llX\r\n",
                        (unsigned long long)actual_tx_ts);
                test_run_info((unsigned char *)str_to_print);
            }
            else
            {
                test_run_info((unsigned char *)"Delayed RESPONSE TX failed\r\n");
            }
        }
        else
        {
            // sprintf(str_to_print,
            //         "poll_rx_ts=0x%010llX clkoff_raw=%d clkoff_ppm=%.3f corrected_delay_dtu=%llu scheduled_delay_dtu=%u resp_tx_ts=0x%010llX\r\n",
            //         (unsigned long long)poll_rx_ts,
            //         (int)clock_offset_raw,
            //         clock_offset_ppm,
            //         (unsigned long long)corrected_delay_dtu,
            //         (unsigned int)scheduled_delay_dtu,
            //         (unsigned long long)resp_tx_ts);
            // test_run_info((unsigned char *)str_to_print);

            /* Clear RX error events in the DW IC status register. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        }
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
