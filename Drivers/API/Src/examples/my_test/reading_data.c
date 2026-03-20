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
#include "../marco_polo/helpers.h"

#if defined(READING_DATA)

#define CLEAR_ARRAY(array, size) for(int i = 0; i < size; i++) array[i] = 0

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "READING DATA v1.0"

// sending stuff
static uint8_t tx_msg[] = { 0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E' };
#define FRAME_LENGTH (sizeof(tx_msg) + FCS_LEN) // The real length that is going to be transmitted
#define TX_DELAY_MS 3000

#define RX_RESP_TO_UUS 2000


/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

static uint8_t cir_buf[DWT_CIR_LEN_MAX * 2 * 3];  /* A complex sample takes up to 2 32-bit words */

static char str_to_print[DWT_CIR_LEN_MAX * 2 * 3]; /* Buffer for printing*/

static uint64_t last_poll_tx_ts = 0;
static uint64_t decoded_poll_rx_ts = 0;
static uint64_t decoded_resp_tx_ts = 0;
static uint64_t decoded_scheduled_delay = 0;
static int decoded_ts_valid = 0;

extern dwt_config_t config_options;
extern dwt_txconfig_t txconfig_options;


/* RX timestamp is available after CIA processing completes (CIADONE). RX_TIME is a 40-bit value at 0x00:64. */
static void read_and_print_rxtime(void)
{
    uint8_t ts[5] = {0};
    uint64_t ts40 = 0;

    /* Read 5-octet RX timestamp using the provided API helper. */
    dwt_readrxtimestamp(ts, DWT_IP_M);

    /* Assemble little-endian 40-bit value. */
    ts40 = ((uint64_t)ts[0]) |
           ((uint64_t)ts[1] << 8) |
           ((uint64_t)ts[2] << 16) |
           ((uint64_t)ts[3] << 24) |
           ((uint64_t)ts[4] << 32);

    sprintf(str_to_print, "RX_TIME (40b) = 0x%02X%02X%02X%02X%02X  (%llu)\r\n",
            ts[4], ts[3], ts[2], ts[1], ts[0], (unsigned long long)ts40);
    test_run_info((unsigned char *)str_to_print);
}

static uint64_t read_timestamp_from_msg(const uint8_t *buffer)
{
    return ((uint64_t)buffer[0]) |
           ((uint64_t)buffer[1] << 8) |
           ((uint64_t)buffer[2] << 16) |
           ((uint64_t)buffer[3] << 24) |
           ((uint64_t)buffer[4] << 32);
}

static uint32_t read_u32_from_msg(const uint8_t *buffer)
{
    return ((uint32_t)buffer[0]) |
           ((uint32_t)buffer[1] << 8) |
           ((uint32_t)buffer[2] << 16) |
           ((uint32_t)buffer[3] << 24);
}


static void read_and_print_rx_message(void)
{
    uint16_t frame_len;
    uint16_t payload_len;
    int offset;
    uint64_t poll_rx_ts_from_msg;
    uint64_t resp_tx_ts_from_msg;
    uint64_t scheduled_delay_from_msg;

    frame_len = dwt_getframelength(0);
    if (frame_len < FCS_LEN)
    {
        sprintf(str_to_print, "Frame too short: len=%u\r\n", (unsigned)frame_len);
        test_run_info((unsigned char *)str_to_print);
        return;
    }

    payload_len = frame_len - FCS_LEN;
    if (payload_len > FRAME_LEN_MAX)
    {
        sprintf(str_to_print,
                "Frame too long for rx_buffer: len=%u payload=%u\r\n",
                (unsigned)frame_len,
                (unsigned)payload_len);
        test_run_info((unsigned char *)str_to_print);
        return;
    }

    dwt_readrxdata(rx_buffer, payload_len, 0);

    sprintf(str_to_print, "Frame Received len=%u payload=%u\r\n",
            (unsigned)frame_len,
            (unsigned)payload_len);
    test_run_info((unsigned char *)str_to_print);

    offset = sprintf(str_to_print, "RX bytes: ");
    for (uint16_t i = 0; i < payload_len && offset < (int)sizeof(str_to_print) - 4; i++)
    {
        offset += sprintf(&str_to_print[offset], "%02X ", rx_buffer[i]);
    }
    offset += sprintf(&str_to_print[offset], "\r\n");
    test_run_info((unsigned char *)str_to_print);

    offset = sprintf(str_to_print, "RX as text: ");
    for (uint16_t i = 0; i < payload_len && offset < (int)sizeof(str_to_print) - 2; i++)
    {
        char c = (char)rx_buffer[i];
        str_to_print[offset++] = ((c >= 32) && (c <= 126)) ? c : '.';
    }
    str_to_print[offset++] = '\r';
    str_to_print[offset++] = '\n';
    str_to_print[offset] = '\0';
    test_run_info((unsigned char *)str_to_print);

    if ((payload_len >= 24) &&
        (rx_buffer[2] == 'R') &&
        (rx_buffer[3] == 'E') &&
        (rx_buffer[4] == 'S') &&
        (rx_buffer[5] == 'P') &&
        (rx_buffer[6] == 'O') &&
        (rx_buffer[7] == 'N') &&
        (rx_buffer[8] == 'S') &&
        (rx_buffer[9] == 'E'))
    {
        poll_rx_ts_from_msg = read_timestamp_from_msg(&rx_buffer[10]);
        resp_tx_ts_from_msg = read_timestamp_from_msg(&rx_buffer[15]);
        // scheduled_delay_from_msg = read_timestamp_from_msg(&rx_buffer[20]);
        scheduled_delay_from_msg = read_u32_from_msg(&rx_buffer[20]);

        decoded_poll_rx_ts = poll_rx_ts_from_msg;
        decoded_resp_tx_ts = resp_tx_ts_from_msg;
        decoded_scheduled_delay = scheduled_delay_from_msg;
        decoded_ts_valid = 1;

        sprintf(str_to_print,
                "Decoded timestamps: poll_rx_ts=%llu (0x%010llX) resp_tx_ts=%llu (0x%010llX) scheduled_delay=%llu (0x%010llX)\r\n",
                (unsigned long long)poll_rx_ts_from_msg,
                (unsigned long long)poll_rx_ts_from_msg,
                (unsigned long long)resp_tx_ts_from_msg,
                (unsigned long long)resp_tx_ts_from_msg,
                (unsigned long long)decoded_scheduled_delay,
                (unsigned long long)decoded_scheduled_delay);
        test_run_info((unsigned char *)str_to_print);
    }
}

/* Read 32-bit little-endian value from DW3000 register file + sub-address */
static inline uint32_t read_u32_le_reg(uint8_t reg_file, uint16_t subaddr)
{
    uint8_t b[4];
    dwt_readfromdevice(reg_file, subaddr, 4, b);
    return ((uint32_t)b[0]) |
           ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) |
           ((uint32_t)b[3] << 24);
}

/**
 * Application entry point.
 */
int reading_data(void)
{
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    uint32_t status_reg;

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

    /* Set response frame timeout. */
    dwt_setrxtimeout(RX_RESP_TO_UUS);

    /* Get the correct length of the Ipatov CIR*/
    uint32_t n_samples_ipatov;
    if (config_options.rxCode < PCODE_PRF64_START) {
        n_samples_ipatov = DWT_CIR_LEN_IP_PRF16;
    }
    else {
        n_samples_ipatov = DWT_CIR_LEN_IP_PRF64;
    }

    test_run_info((unsigned char *)"DW3000 initialized and configured. Starting main loop.\r\n");

    // test print, hoping for MINDIAG = 0 
    uint32_t cia_conf = read_u32_le_reg(0x0E, 0x00);
    sprintf(str_to_print,
            "CIA_CONF=0x%08lX  MINDIAG=%lu\r\n",
            (unsigned long)cia_conf,
            (unsigned long)((cia_conf >> 20) & 1U));
    test_run_info((unsigned char *)str_to_print);

    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

    // test print
    cia_conf = read_u32_le_reg(0x0E, 0x00);
    sprintf(str_to_print,
            "CIA_CONF=0x%08lX  MINDIAG=%lu\r\n",
            (unsigned long)cia_conf,
            (unsigned long)((cia_conf >> 20) & 1U));
    test_run_info((unsigned char *)str_to_print);

    // theoretical value to compare against actual delay in responder
    uint64_t nominal_delay_dtu = 1500ULL * UUS_TO_DWT_TIME;

    int counter = 0;
    /* Loop forever, but only send a frame when a button is pressed. */
    while (TRUE)
    {
        // /* Wait for Button 1 (index 0) to be pressed. */
        // while (!bsp_board_button_state_get(0))
        // {
        //     /* Small delay to avoid a tight busy loop. */
        //     Sleep(10);
        // }

        // /* Simple debounce: wait until the button is released. */
        // while (bsp_board_button_state_get(0))
        // {
        //     Sleep(10);
        // }
        Sleep(10);

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

        /* Read TX timestamp here */
        uint8_t ts[5];
        uint64_t tx_time = 0;

        dwt_readtxtimestamp(ts);

        tx_time = ((uint64_t)ts[0]) |
                ((uint64_t)ts[1] << 8) |
                ((uint64_t)ts[2] << 16) |
                ((uint64_t)ts[3] << 24) |
                ((uint64_t)ts[4] << 32);

        last_poll_tx_ts = tx_time;

        sprintf(str_to_print, "TX_TIME (40b) = 0x%02X%02X%02X%02X%02X (%llu)\r\n",
                ts[4], ts[3], ts[2], ts[1], ts[0], (unsigned long long)tx_time);
        test_run_info((unsigned char *)str_to_print);

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
            read_and_print_rx_message();

            // read_and_print_sys_status();

            // if (wait_for_ci_done_ms(20))
            // {
            //     test_run_info((unsigned char *)"CIADONE observed before reading CIA diagnostics\r\n");
            // }
            // else
            // {
            //     test_run_info((unsigned char *)"CIADONE NOT observed within 20 ms\r\n");
            // }

            // read_and_print_sys_status();
            read_and_print_rxtime();
            /* Compute distance using SS-TWR if timestamps from responder were decoded */
            if (decoded_ts_valid)
            {
                uint8_t ts_local[5];
                uint64_t resp_rx_ts = 0;

                dwt_readrxtimestamp(ts_local, DWT_IP_M);

                resp_rx_ts = ((uint64_t)ts_local[0]) |
                             ((uint64_t)ts_local[1] << 8) |
                             ((uint64_t)ts_local[2] << 16) |
                             ((uint64_t)ts_local[3] << 24) |
                             ((uint64_t)ts_local[4] << 32);

                int64_t rtd_init = (int64_t)(resp_rx_ts - last_poll_tx_ts);
                int64_t rtd_resp = (int64_t)(decoded_resp_tx_ts - decoded_poll_rx_ts);

                double tof_dtu = (double)(rtd_init - rtd_resp) / 2.0;
                double tof_s = tof_dtu / (128.0 * 499.2e6);
                double distance = tof_s * 299792458.0;

                sprintf(str_to_print,
                        "SS-TWR: rtd_init=%lld rtd_resp=%lld ToF_dtu=%.0f Distance=%.3f m\r\n",
                        (long long)rtd_init,
                        (long long)rtd_resp,
                        tof_dtu,
                        distance);

                test_run_info((unsigned char *)str_to_print);

                int64_t resp_proc_time = (int64_t)(decoded_resp_tx_ts - decoded_poll_rx_ts);
                sprintf(str_to_print,
                        "Responder processing time: %lld dtu = %.9f ms\r\n",
                        (long long)resp_proc_time,
                        (double)resp_proc_time / (128.0 * 499.2e6) * 1e3);
                test_run_info((unsigned char *)str_to_print);

                int64_t delay_error_dtu = (int64_t)rtd_resp - (int64_t)nominal_delay_dtu;
                double delay_error_ns = delay_error_dtu * (1.0 / (128.0 * 499.2e6)) * 1e9;

                sprintf(str_to_print,
                        "rtd_resp=%llu dtu, error=%lld dtu (%.3f ns)\r\n",
                        (unsigned long long)rtd_resp,
                        (long long)delay_error_dtu,
                        delay_error_ns);
                    test_run_info((unsigned char *)str_to_print);

                decoded_ts_valid = 0;
            }
            // read_and_print_cia_diags();

            test_run_info((unsigned char *)"########## READING WITH BUILD IN API CALL ###########\r\n");

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
            // dwt_acc_idx_e acc_idx = DWT_ACC_IDX_IP_M ;
            
            /*
                Choose the mode you want to print the data:
                    - DWT_CIR_READ_FULL
                    - DWT_CIR_READ_LO
                    - DWT_CIR_READ_MID
                    - DWT_CIR_READ_HI
            */
            // dwt_cir_read_mode_e modes = DWT_CIR_READ_FULL; 

            /* Ipatov data */
            // int n_samples = n_samples_ipatov;
            // dwt_readcir((uint32_t*)cir_buf, acc_idx, 0, n_samples, modes);
            // find_and_print_cir_peak_from_buffer(cir_buf, n_samples, modes);
            // print_cir(cir_buf, n_samples, modes);
            
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
