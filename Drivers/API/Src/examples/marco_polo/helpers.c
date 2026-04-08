#include <stdint.h>
#include <stdbool.h>

#include <config_options.h>
#include <deca_device_api.h>
#include <deca_types.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <stdlib.h>
#include "deca_private.h"
#include <math.h>

#include "helpers.h"

extern void test_run_info(unsigned char *data);

static char str_to_print[DWT_CIR_LEN_MAX * 2 * 3]; /* Buffer for printing*/


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

static int wait_for_ci_done_ms(uint32_t timeout_ms)
{
    uint32_t elapsed = 0;

    while (elapsed < timeout_ms)
    {
        uint32_t sys_status_lo = dwt_readsysstatuslo();
        if (sys_status_lo & DWT_INT_CIADONE_BIT_MASK)
        {
            return 1;
        }

        Sleep(1);
        elapsed++;
    }

    return 0;
}


/* Print CIA diagnostics used to align RX_TIME with the CIR window.
 * IP_FP: first path location within the Ipatov (preamble) CIR, [10.6] format, relative to accumulator index 0.
 * IP_PEAKI/IP_PEAKA: index/amplitude of maximum-magnitude sample in the Ipatov CIR.
 */
static void read_and_print_cia_diags(void)
{
    /* --- IP_DIAG_8 @ 0x0C:48 (example: FP index field in low 16 bits) --- */
    uint32_t ip_diag_8 = read_u32_le_reg(0x0C, 0x48);

    uint16_t ip_fp_raw  = (uint16_t)(ip_diag_8 & 0xFFFFu);
    uint16_t ip_fp_int  = (uint16_t)(ip_fp_raw >> 6);
    uint16_t ip_fp_frac = (uint16_t)(ip_fp_raw & 0x3Fu);
    float fp_index = (float)ip_fp_int + ((float)ip_fp_frac / 64.0f);

    /* --- IP_DIAG_0 @ 0x0C:28 holds IP_PEAKA (bits 20:0) and IP_PEAKI (bits 30:21) --- */
    uint32_t ip_diag_0 = read_u32_le_reg(0x0C, 0x28);

    uint32_t ip_peaka = (ip_diag_0 & 0x001FFFFFu);
    uint16_t ip_peaki = (uint16_t)((ip_diag_0 >> 21) & 0x03FFu);

    sprintf(str_to_print,
        "IP_DIAG_8=0x%08lX  IP_DIAG_0=0x%08lX\r\n"
        "IP_FP(raw [10.6])=0x%04X  FP_index=%.6f (int=%u frac=%u/64)\r\n"
        "IP_PEAKI=%u  IP_PEAKA=%lu\r\n",
        (unsigned long)ip_diag_8,
        (unsigned long)ip_diag_0,
        (unsigned)ip_fp_raw,
        fp_index,
        (unsigned)ip_fp_int,
        (unsigned)ip_fp_frac,
        (unsigned)ip_peaki,
        (unsigned long)ip_peaka);

    test_run_info((unsigned char *)str_to_print);
}

static void check_if_not_zero(void) {
    uint8_t b[4];
    uint8_t reg_file = 0x0C;
    uint16_t subaddr = 0x48;
    dwt_readfromdevice(reg_file, subaddr, 4, b);
    // print raw bytes
    sprintf(str_to_print, "Raw bytes read from reg 0x%02X:%04X = 0x%02X%02X%02X%02X\r\n",
            reg_file, subaddr,
            b[3], b[2], b[1], b[0]);
    test_run_info((unsigned char *)str_to_print);
    // print output
    uint32_t val = ((uint32_t)b[0]) |
                   ((uint32_t)b[1] << 8) |
                   ((uint32_t)b[2] << 16) |
                   ((uint32_t)b[3] << 24);
    sprintf(str_to_print, "Value read from reg 0x%02X:%04X = 0x%02X%02X%02X%02X  (%lu)\r\n",
            reg_file, subaddr,
            b[3], b[2], b[1], b[0],
            (unsigned long)val);
    test_run_info((unsigned char *)str_to_print);
}

static void read_and_print_sys_status(void)
{
    uint32_t sys_status_lo = dwt_readsysstatuslo();

    sprintf(str_to_print,
            "SYS_STATUS_LO=0x%08lX  CIADONE=%lu  RXFCG=%lu\r\n",
            (unsigned long)sys_status_lo,
            (unsigned long)((sys_status_lo & DWT_INT_CIADONE_BIT_MASK) ? 1U : 0U),
            (unsigned long)((sys_status_lo & DWT_INT_RXFCG_BIT_MASK) ? 1U : 0U));
    test_run_info((unsigned char *)str_to_print);
}

static void find_and_print_cir_peak_from_buffer(uint8_t *buf, int n_samples, dwt_cir_read_mode_e mode)
{
    int bytes_per_sample = (mode == DWT_CIR_READ_FULL) ? 6 : 4;
    float best_mag = 0.0f;
    int best_idx = 0;

    for (int i = 0; i < n_samples; i++)
    {
        float mag = cir_mag_from_buf(&buf[i * bytes_per_sample], mode);
        if ((i == 0) || (mag > best_mag))
        {
            best_mag = mag;
            best_idx = i;
        }
    }

    sprintf(str_to_print,
            "CIR buffer max magnitude at index %d = %f\r\n",
            best_idx,
            best_mag);
    test_run_info((unsigned char *)str_to_print);
}