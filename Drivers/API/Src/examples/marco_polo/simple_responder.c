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
#include <math.h>

#include "nrf.h"

#if defined(SIMPLE_RESPONDER)

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "SIMPLE RESPONDER v1.0"

/* sending stuff */
#define RESP_MSG_COMMON_LEN 10
#define RESP_MSG_POLL_RX_TS_IDX RESP_MSG_COMMON_LEN
#define RESP_MSG_RESP_TX_TS_IDX (RESP_MSG_POLL_RX_TS_IDX + 5)
#define RESP_MSG_DELAY_UUS_IDX (RESP_MSG_RESP_TX_TS_IDX + 5)
#define RESP_MSG_DELAY_UUS_LEN 4
#define RESP_TX_DELAY_UUS 1500

#define RESP_LED_FLASH_MS 30

// #define DELTA_I_DTU 0
// #define DELTA_I_DTU 1024  // ≈16 ns
// #define DELTA_I_DTU 8192 // ≈128 ns
// #define DELTA_I_DTU 32768 // ≈512 ns
#define DEVICE_INDEX 0 // unique value per device to determine delay

#define LEVEL3_TEST_ENABLE 1
#define LEVEL3_TRIM_DELTA -10
#define LEVEL3_TDET_US 560

static uint8_t tx_msg[] = {
    0xC5, 0, 'R'
};
#define FRAME_LENGTH (sizeof(tx_msg) + FCS_LEN) // The real length that is going to be transmitted

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

extern dwt_config_t config_options;
extern dwt_txconfig_t txconfig_options;


////////////// setting unique responder delays //////////////
uint64_t get_nrf_unique_id(void)
{
    uint64_t id0 = NRF_FICR->DEVICEID[0];
    uint64_t id1 = NRF_FICR->DEVICEID[1];
    return (id1 << 32) | id0;
}

// #if DEVICE_INDEX == 0 // 0x984333D21EA7858F
//     #define TX_ANT_DLY 16366
//     #define RX_ANT_DLY 16366
// #elif DEVICE_INDEX == 1
//     #define TX_ANT_DLY 16372
//     #define RX_ANT_DLY 16372
// #elif DEVICE_INDEX == 2
//     #define TX_ANT_DLY 16366
//     #define RX_ANT_DLY 16366
// #elif DEVICE_INDEX == 3
//     #define TX_ANT_DLY 16370
//     #define RX_ANT_DLY 16370
// #else
//     #error "Invalid DEVICE_INDEX"
// #endif

#define NRF_UID_0 0x984333D21EA7858FULL
#define NRF_UID_1 0xA430F6CC89EF287AULL
#define NRF_UID_2 0xBF8965F181CE8EF5ULL
#define NRF_UID_3 0xB0E656A4CFB9F2C4ULL

const uint32_t tx_rx_ant_dly[] = {16366, 16372, 16366, 16370};


static uint8_t clamp_trim(int val)
{
    if (val < 0) return 0;
    if (val > 63) return 63;
    return (uint8_t)val;
}

/**
 * Application entry point.
 */
int simple_responder(void)
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

    uint8_t trim_base;
    uint8_t trim_temp;
    int32_t carrier_integrator;
    uint64_t desired_tx_ts;
    int64_t scheduling_error_dtu;
    int64_t epsilon_dtu;
    double epsilon_ns;
    uint32_t scheduled_delay_dtu;

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

    dwt_setleds(DWT_LEDS_DISABLE);

    /* Configure DW IC. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config_options))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };  
    }

    /* Configure the TX spectrum parameters (power PG delay and PG Count) */
    dwt_configuretxrf(&txconfig_options);

    /* Pre-load static TX buffer and precompute constants to avoid doing this every loop iteration */
    dwt_writetxdata(FRAME_LENGTH - FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(FRAME_LENGTH, 0, 0); /* Zero offset in TX buffer, no ranging. */

    const double dtu_to_ns = 1e9 / (128.0 * 499.2e6);
    const double shift_per_step_ns_const = 1.48 * ((double)LEVEL3_TDET_US) / 1000.0;

    uint64_t nrf_uid = get_nrf_unique_id();
    sprintf(str_to_print, "Device Unique ID: 0x%016llX", (unsigned long long)nrf_uid); 
    test_run_info((unsigned char *)str_to_print);

    uint32_t device_index;
    if (nrf_uid == NRF_UID_0)
    {
        device_index = 0;
    }
    else if (nrf_uid == NRF_UID_1)
    {
        device_index = 1;
    }
    else if (nrf_uid == NRF_UID_2)
    {
        device_index = 2;
    }
    else if (nrf_uid == NRF_UID_3)
    {
        device_index = 3;
    } else {
        test_run_info((unsigned char *)"Unknown device ID, using default antenna delay");
        device_index = 0;
    }
    uint32_t TX_RX_ANT_DLY = tx_rx_ant_dly[device_index];
    
    sprintf(str_to_print, "Device Index: %lu, Antenna Delay: %lu", (unsigned long)device_index, (unsigned long)TX_RX_ANT_DLY);
    test_run_info((unsigned char *)str_to_print);

    /* Loop forever receiving frames. */
    while (TRUE)
    {
        /* TESTING BREAKPOINT LOCATION #1 */

        /* Note: explicit clearing of the whole RX buffer each loop is unnecessary
         * because `dwt_readrxdata()` overwrites the received portion. Skipping
         * the memset reduces CPU/time overhead in the hot path. */

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

            // test_run_info((unsigned char *)"Frame Received");

            poll_rx_ts = get_rx_timestamp_u64();

            clock_offset_raw = dwt_readclockoffset(); 
            // clock is x parts per million off
            clock_offset_frac = ((double)clock_offset_raw / 67108864.0);
            double clock_offset_ppm = clock_offset_frac * 1e6;
            
            trim_base = dwt_getxtaltrim();
            int trim_cfo_steps = (int)llround(clock_offset_ppm / 1.48);
            uint8_t trim_cfo = clamp_trim((int)trim_base - trim_cfo_steps);

            uint64_t resp_delay_dtu = (uint64_t)RESP_TX_DELAY_UUS * UUS_TO_DWT_TIME;
            uint64_t desired_delay_dtu = resp_delay_dtu + DELTA_I_DTU * device_index;
            
            desired_tx_ts = poll_rx_ts + desired_delay_dtu + TX_RX_ANT_DLY; // add responder specific delay delta_i

            resp_tx_time = (uint32_t)((poll_rx_ts + desired_delay_dtu) >> 8); // value for setdelayedtrxtime
            resp_tx_ts = ((((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_RX_ANT_DLY);
            
            scheduling_error_dtu = (int64_t)resp_tx_ts - (int64_t)desired_tx_ts;
            double scheduling_error_ns = scheduling_error_dtu * dtu_to_ns;

            /* Use precomputed shift/unit constant */
            double shift_per_step_ns = shift_per_step_ns_const;
            int trim_tx_steps = (int)llround((-scheduling_error_ns) / shift_per_step_ns);

            uint8_t trim_temp = clamp_trim((int)trim_cfo + trim_tx_steps);

            uint32_t precise_delay_us = LEVEL3_TDET_US;

            if (trim_tx_steps != 0)
            {
                double tdet_calc_us = (fabs(scheduling_error_ns) * 1000.0) / (1.48 * fabs((double)trim_tx_steps));

                if (tdet_calc_us < 0.0)
                    tdet_calc_us = 0.0;
                if (tdet_calc_us > 700.0)
                    tdet_calc_us = 700.0;

                precise_delay_us = (uint32_t)llround(tdet_calc_us);
            }
            
            if (trim_temp != trim_base)
            {
                dwt_setxtaltrim(trim_temp);
            }
            /* Write frame data to DW IC and prepare delayed transmission. */

            nrf_delay_us(precise_delay_us); 

            dwt_setdelayedtrxtime(resp_tx_time);
            
            if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS)
            {
                waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
                dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

                /* Flash only after the response transmission has completed. */
                dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
                nrf_delay_ms(1);
                dwt_setleds(DWT_LEDS_DISABLE);
                
                // actual_tx_ts = get_tx_timestamp_u64();
                if (trim_cfo != trim_temp)
                {
                    dwt_setxtaltrim(trim_cfo);
                }
            }
            else
            {
                // test_run_info((unsigned char *)"Delayed RESPONSE TX failed\r\n");
            }
        }
        else
        {
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
