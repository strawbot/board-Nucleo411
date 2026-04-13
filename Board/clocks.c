// Clocks

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

// Parse __TIMESTAMP__ string ("Www Mmm DD HH:MM:SS YYYY") into a UTC Unix timestamp.
static Long timestamp_to_utc(const char *ts) {
    static const char * const mon_names[] = {
        "Jan","Feb","Mar","Apr","May","Jun",
        "Jul","Aug","Sep","Oct","Nov","Dec"
    };
    struct tm t = {0};
    char wday[4], mon[4];
    int day, hour, min, sec, year;

    sscanf(ts, "%3s %3s %d %d:%d:%d %d",
           wday, mon, &day, &hour, &min, &sec, &year);

    t.tm_mday  = day;
    t.tm_hour  = hour;
    t.tm_min   = min;
    t.tm_sec   = sec;
    t.tm_year  = year - 1900;
    t.tm_isdst = 0;
    for (int i = 0; i < 12; i++) {
        if (strncmp(mon, mon_names[i], 3) == 0) { t.tm_mon = i; break; }
    }
    // mktime() on newlib bare-metal treats struct tm as UTC (no tz offset)
    return (Long)mktime(&t);
}

// Write a UTC Unix timestamp into the STM32F4 RTC peripheral (LL driver).
void set_utc(Long utc) {
    time_t t_val = (time_t)utc;
    struct tm tm_buf;                       /* caller-owned: no malloc       */
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

void over_due() { /* incCtr(overDueTea); */ }

// must translate to local timer scale; UTC seconds to DELTA seconds; 1 if same clock
void set_delta_alarm(Long t) { if (t > 0xFFFF) print("#");
	LL_TIM_SetAutoReload(TIM10, t - 1);  // count to t ticks
	LL_TIM_SetCounter(TIM10, 0);  // reset counter
	LL_TIM_EnableCounter(TIM10);  // start — stops automatically at ARR in one-pulse mode
}	

void delta_alarm() { LL_TIM_ClearFlag_UPDATE(TIM10);  now(*alarmEvent); }

Long get_ticks() { return LL_TIM_GetCounter(TIM2); } // 100 us ticks

void show_timer() {
	print("  ticks/S:");
	printDec(ONE_SECOND);
	print("  Timer:");
	dotnb(8, 8, TIM10->CNT, 16);
	print("  "), print_RTC();
}

void micro_sleep() { __WFI(); }

#define set_pin(pin) LL_GPIO_SetOutputPin(pin##_GPIO_Port, pin##_Pin)
#define reset_pin(pin) LL_GPIO_ResetOutputPin(pin##_GPIO_Port, pin##_Pin)

#define gron() set_pin(LD2)
#define groff() reset_pin(LD2)
#define ON_TIME 2

static void blink_leds() {
	Long t;
	static enum {GREEN1,SPACE1,GREEN2,SPACE2} color = SPACE2;
	switch (color) {
	case GREEN1:  		gron();		color = SPACE1; t=ON_TIME;	break;
	case SPACE1:		groff();	color = GREEN2; t=200-ON_TIME;	break;
	case GREEN2:  		gron();		color = SPACE2; t=ON_TIME;	break;
	case SPACE2:		groff();	color = GREEN1; t=800-ON_TIME;	break;
	}
	in(msec(t), blink_leds);
}

// compare times over an interval: sysTicks();
void init_clocks() {
	// start a 32 bit counter based on processor frequency
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	SysTick->CTRL  = 0;                   /* Disable SysTick — timebase is now TIM2 */

	never(alarmEvent);
	LL_TIM_SetOnePulseMode(TIM10, LL_TIM_ONEPULSEMODE_SINGLE);
	LL_TIM_EnableIT_UPDATE(TIM10);
	LL_TIM_EnableCounter(TIM2);
	later(blink_leds);
	namedAction(blink_leds);
	print("\nBuilt: "), print(BUILD_TS);
    Long utc = timestamp_to_utc(BUILD_TS);
    print("  utc: "),printuDec(utc);
	set_utc(utc);
}
