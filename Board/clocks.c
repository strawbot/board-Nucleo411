// Clocks — Nucleo411 board-specific clock code (STM32F411, LL driver)
//
// TimbreOS/clocks.c provides: timestamp_to_utc, epoch_to_tm, tm_to_epoch,
// over_due, micro_sleep, print_build_banner, blink_leds, show_timer,
// set_delta_alarm, delta_alarm, get_ticks, init_clocks.
//
// This file provides only what is board-specific:
//   set_utc() — writes epoch to the STM32F4 hardware RTC via LL driver.

#include "tea.h"
#include "gpio.h"
#include "printers.h"
#include "cli.h"
#include "project_defs.h"
#include "tim.h"
#include "stm32f4xx_ll_rtc.h"
#include "rev.h"
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

void print_RTC();

// Write a UTC Unix timestamp into the STM32F4 RTC peripheral (LL driver).
void set_utc(Long utc) {
    time_t t_val = (time_t)utc;
    struct tm tm_buf;
    struct tm *t = gmtime_r(&t_val, &tm_buf);

    // tm_wday: 0=Sun..6=Sat  →  RTC: 1=Mon..7=Sun
    uint8_t wday = (t->tm_wday == 0) ? 7U : (uint8_t)t->tm_wday;

    LL_RTC_DisableWriteProtection(RTC);
    LL_RTC_EnableInitMode(RTC);
    while (!LL_RTC_IsActiveFlag_INIT(RTC)) {}

    LL_RTC_TIME_Config(RTC,
        LL_RTC_TIME_FORMAT_AM_OR_24,
        __LL_RTC_CONVERT_BIN2BCD((uint8_t)t->tm_hour),
        __LL_RTC_CONVERT_BIN2BCD((uint8_t)t->tm_min),
        __LL_RTC_CONVERT_BIN2BCD((uint8_t)t->tm_sec));

    LL_RTC_DATE_Config(RTC,
        wday,
        __LL_RTC_CONVERT_BIN2BCD((uint8_t)t->tm_mday),
        __LL_RTC_CONVERT_BIN2BCD((uint8_t)(t->tm_mon + 1)),
        __LL_RTC_CONVERT_BIN2BCD((uint8_t)(t->tm_year % 100)));

    LL_RTC_DisableInitMode(RTC);
    LL_RTC_EnableWriteProtection(RTC);

    // Update backup register so MX_RTC_Init won't clobber the time on next boot
    LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR0, 0x32F2);
}
