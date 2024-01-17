/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* ULP Example: using ADC in deep sleep

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This file contains assembly code which runs on the ULP.

   ULP wakes up to run this code at a certain period, determined by the values
   in SENS_ULP_CP_SLEEP_CYCx_REG registers. On each wake up, the program
   measures input voltage on the given ADC channel 'adc_oversampling_factor'
   times. Measurements are accumulated and average value is calculated.
   Average value is compared to the two thresholds: 'low_thr' and 'high_thr'.
   If the value is less than 'low_thr' or more than 'high_thr', ULP wakes up
   the chip from deep sleep.
*/

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files
 */
#include "soc/rtc_cntl_reg.h"
#include "soc/soc_ulp.h"
#include "ulp_config.h"

    .set adc_channel_co, CO_ADC_CHANNEL
    .set adc_channel_odor, ODOR_ADC_CHANNEL

    /* Configure the number of ADC samples to average on each measurement.
    For convenience, make it a power of 2. */
    .set adc_oversampling_factor_log, 2
    .set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

    /* Define variables, which go into .bss section (zero-initialized data) */
    .bss

    /* High threshold of ADC reading.
    Set by the main program. */
    .global high_thr_co
high_thr_co:
	.long 0

    .global high_thr_odor
high_thr_odor:
    .long 0

    .global last_result_co
last_result_co:
    .long 0

    .global last_result_odor
last_result_odor:
    .long 0

    .global wakeup_flag_co
wakeup_flag_co:
    .long 0

    .global wakeup_flag_odor
wakeup_flag_odor:
    .long 0

    .text
    .global entry
entry:
	/* do measurements using ADC */
	/* r0 will be used as accumulator */
	move r0, 0
	/* initialize the loop counter */
	stage_rst
measure_co:
	/* measure and add value to accumulator */
	adc r1, 0, adc_channel_co + 1
	add r0, r0, r1
	/* increment loop counter and check exit condition */
	stage_inc 1
	jumps measure_co, adc_oversampling_factor, lt

    rsh r0, r0, adc_oversampling_factor_log
    move r3, last_result_co
    st r0, r3, 0

    move r3, high_thr_co
    ld r3, r3, 0
    sub r3, r3, r0
    jump wake_up_co, ov

    /* Measurements for the second channel */
    move r0, 0
    stage_rst
measure_odor:
    adc r1, 0, adc_channel_odor + 1
    add r0, r0, r1
    stage_inc 1
    jumps measure_odor, adc_oversampling_factor, lt

    rsh r0, r0, adc_oversampling_factor_log
    move r3, last_result_odor
    st r0, r3, 0

    move r3, high_thr_odor
    ld r3, r3, 0
    sub r3, r3, r0
    jump wake_up_odor, ov

exit:
	halt

wake_up_co:
	/* Check if the system can be woken up */
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	and r0, r0, 1
	jump exit, eq

    move r3, wakeup_flag_co
    move r2, 1
    st r2, r3, 0
	/* Wake up the SoC, end program */
	wake
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
	halt

wake_up_odor:
    /* Check if the system can be woken up */
    READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
    and r0, r0, 1
    jump exit, eq

    /* Set flag for Channel 'odor' wake-up */
    move r3, wakeup_flag_odor
    move r2, 1
    st r2, r3, 0
    /* Wake up the SoC, end program */
    wake
    WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
    halt