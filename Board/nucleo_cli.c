// discovery_cli.c — CLI command implementations for the Discovery board.
// All functions are void(void) — output via print() / printers.h to emitq.

#include <stdbool.h>
#include <stdint.h>

#include "tea.h"
#include "printers.h"
#include "stm32f4xx_ll_rcc.h"
#include "nucleo_cli.h"

/* ── Stack overflow detection ────────────────────────────────────────────────
 *
 * The linker exports _sstack (bottom of stack region) and _estack (top /
 * initial SP).  On Cortex-M the stack grows downward, so _sstack is where
 * overflow hits first.
 *
 * stack_canary_init() fills the entire unused portion of the stack with
 * 0xDEADBEEF as early as possible in main().  After that:
 *   - stack_bottom_intact()   → returns 0 if SP ever went below _sstack
 *   - stack_high_water_bytes()→ scans from _sstack upward for the first
 *                               non-canary word; difference from _estack
 *                               gives max bytes ever used (high-water mark)
 */
#define CANARY_WORD  0xDEADBEEFu

extern uint32_t _estack;   /* linker: top of stack (initial SP) */
extern uint32_t _Min_Stack_Size; // hopefully the stack size declared
extern uint32_t _ebss; // start of heap/bottom of free memory

#define _sstack _ebss  /* linker: bottom of stack region */

void stack_canary_init(void)
{
    uint32_t sp;
    __asm volatile ("mov %0, sp" : "=r" (sp));

    volatile uint32_t *p     = (volatile uint32_t *)&_sstack;
    volatile uint32_t *limit = (volatile uint32_t *)(sp - 64u); /* safety margin */

    while (p < limit) *p++ = CANARY_WORD;
}

/* Scan from _sstack upward; return max bytes ever used by the stack. */
static uint32_t stack_high_water_bytes(void)
{
    volatile uint32_t *p   = (volatile uint32_t *)&_sstack;
    volatile uint32_t *top = (volatile uint32_t *)&_estack;
    while (p < top && *p == CANARY_WORD) p++;
    return (uint32_t)((uintptr_t)top - (uintptr_t)p);
}

/* Returns 0 if any of the lowest 4 canary words have been overwritten. */
static int stack_bottom_intact(void)
{
    volatile uint32_t *p = (volatile uint32_t *)&_sstack;
    for (int i = 0; i < 4; i++) {
        print(".");
        if (p[i] != CANARY_WORD) return 0;
    }
    return 1;
}

void show_sys(void) {
    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);
    print("SYSCLK:  "); printDec(clocks.SYSCLK_Frequency / 1000000); print(" MHz"); printCr();
    print("HCLK:    "); printDec(clocks.HCLK_Frequency   / 1000000); print(" MHz"); printCr();
    print("PCLK1:   "); printDec(clocks.PCLK1_Frequency  / 1000000); print(" MHz"); printCr();
    print("PCLK2:   "); printDec(clocks.PCLK2_Frequency  / 1000000); print(" MHz"); printCr();
    print("uptime:  "); printDec(get_ticks());                        print(" ticks"); printCr();
    show_timer();
    printCr();

    /* _Min_Stack_Size: linker symbol whose *address* is the configured budget.
     * hwm is shown against this limit so the fraction is meaningful.
     * The canary at _sstack (end of BSS) catches overflow into data/BSS
     * regardless of the budget — both conditions are reported separately.  */
    uint32_t limit = (uint32_t)&_Min_Stack_Size;
    uint32_t hwm   = stack_high_water_bytes();
    print("stack:   ");
    if (!stack_bottom_intact()) {
        print("*** OVERFLOW into BSS ***  hwm="); printDec(hwm);
        print("/"); printDec(limit); print(" B");
        pdump((Byte *)&_sstack, 10);
    } else if (hwm > limit) {
        print("WARNING hwm="); printDec(hwm);
        print("/"); printDec(limit); print(" B  exceeds _Min_Stack_Size");
    } else {
        print("OK  hwm="); printDec(hwm);
        print("/"); printDec(limit); print(" B");
    }
    printCr();
}

// ── Hardware timer survey ─────────────────────────────────────────────────────
//
// show_timers() — one-line summary of every STM32F407 timer peripheral:
//   TIM1–TIM14  bus, clock-gate state, CEN, direction, OPM, PSC, ARR, CNT
//               active CC channels: mode (PWM1/PWM2/IC/tog/…) and CCR value
//   RTC         enabled / time-of-day (BCD shadow register)
//
// ARR and CNT are shown in hex — more useful for live counter reads.
// PSC is decimal (small value, usually meaningful as a divider).
// Only channels that have their CCxE output-enable bit set are printed.
// If a timer's APB clock is gated off its registers are not touched.

// OC mode names indexed by CCMR OCxM bits [2:0]
static const char * const s_oc_mode[8] = {
    "frz","act","!act","tog","lo","hi","PWM1","PWM2"
};

// Print one CC channel (output or IC) if its enable bit is set in CCER.
// idx: 0-based channel number (0=CH1 … 3=CH4)
// ccmr_byte: the 8-bit CCMR field for this channel
// ccer: full 32-bit CCER register value read before calling
// ccr: this channel's compare register value
#define SHOW_CHAN(idx, ccmr_byte, ccer, ccr) do {                         \
    if ((uint32_t)(ccer) >> ((idx) * 4u) & 1u) {                          \
        uint8_t _b = (uint8_t)(ccmr_byte);                                \
        print(" CH"); printDec((idx) + 1u); print(":");                   \
        if (_b & 3u) print("IC");                                         \
        else { print(s_oc_mode[(_b >> 4u) & 7u]); print("="); printDec(ccr); } \
    }                                                                      \
} while (0)

void print_RTC() {
    // RTC — backup domain; gate-check via BDCR.RTCEN (bit 15)
    if (RCC->BDCR & (1u << 15)) {
        // Read TR first to latch the shadow registers, then DR
        uint32_t tr = RTC->TR;
        uint32_t dr = RTC->DR;
        // BCD-decode time (H,M,S) and date (DD,MM) from shadow registers
        uint8_t hh = (uint8_t)(((tr >> 20) & 3u) * 10u + ((tr >> 16) & 0xFu));
        uint8_t mn = (uint8_t)(((tr >> 12) & 7u) * 10u + ((tr >>  8) & 0xFu));
        uint8_t ss = (uint8_t)(((tr >>  4) & 7u) * 10u + ( tr        & 0xFu));
        uint8_t yy = (uint8_t)(((dr >> 20) & 3u) * 10u + ((dr >> 16) & 0xFu));
        uint8_t mo = (uint8_t)(((dr >> 12) & 1u) * 10u + ((dr >>  8) & 0xFu));
        uint8_t dd = (uint8_t)(((dr >>  4) & 3u) * 10u + ( dr        & 0xFu));
        uint16_t yyyy = (uint16_t)(2000u + yy);
        print("RTC: ");
        printDec0(yyyy); print("-");
        if (mo < 10) {print("0");} printDec0(mo); print("-");
        if (dd < 10) {print("0");} printDec0(dd);
        print("  ");
        if (hh < 10) {print("0");} printDec0(hh); print(":");
        if (mn < 10) {print("0");} printDec0(mn); print(":");
        if (ss < 10) {print("0");} printDec0(ss);
    }
}

void show_timers(void)
{
    // Static descriptor table — all 14 timer instances.
    // Fields:
    //   tim       — peripheral pointer (CMSIS compile-time constant address)
    //   name      — display string
    //   bus       — 1=APB1, 2=APB2  (determines which RCC_APBxENR to check)
    //   rcc_bit   — bit mask in RCC_APBxENR
    //   has_dir   — CR1.DIR / CR1.CMS valid (TIM1-5, TIM8 only)
    //   basic     — no CC channels (TIM6, TIM7)
    //   nch       — number of CC channels: 1, 2, or 4  (0 for basic)
    static const struct {
        TIM_TypeDef *tim;
        char         name[6];
        uint8_t      bus;
        uint32_t     rcc_bit;
        uint8_t      has_dir;
        uint8_t      basic;
        uint8_t      nch;
    } td[] = {
        { TIM1,  "TIM1 ", 2, 1u<< 0, 1, 0, 4 },
        { TIM2,  "TIM2 ", 1, 1u<< 0, 1, 0, 4 },
        { TIM3,  "TIM3 ", 1, 1u<< 1, 1, 0, 4 },
        { TIM4,  "TIM4 ", 1, 1u<< 2, 1, 0, 4 },
        { TIM5,  "TIM5 ", 1, 1u<< 3, 1, 0, 4 },
        { TIM9,  "TIM9 ", 2, 1u<<16, 0, 0, 2 },
        { TIM10, "TIM10", 2, 1u<<17, 0, 0, 1 },
        { TIM11, "TIM11", 2, 1u<<18, 0, 0, 1 },
    };

    // Column positions — shared by header and every data row
    // Timer name + ": " always occupies cols 0-6 ("TIM14: " = 7 chars)
    #define CT_ON   8   // "On"  — CEN state or "clk-off"
    #define CT_DIR  12  // "Dir" — up / dn / CA1 / CA2 / CA3
    #define CT_OPM  17  // "OPM" — present only when one-pulse mode is active
    #define CT_PSC  22  // "PSC" — prescaler value (decimal)
    #define CT_ARR  30  // "ARR" — auto-reload (8 hex digits + 'h')
    #define CT_CNT  41  // "CNT" — live counter  (8 hex digits + 'h')

    // Header — identical tabTo() calls mirror the data rows below
    print("Timer");
    tabTo(CT_ON);  print("On");
    tabTo(CT_DIR); print("Dir");
    tabTo(CT_OPM); print("OPM");
    tabTo(CT_PSC); print("PSC");
    tabTo(CT_ARR); print("ARR");
    tabTo(CT_CNT); print("CNT");
    printCr();

    for (unsigned i = 0; i < sizeof(td)/sizeof(td[0]); i++) {
        TIM_TypeDef *t = td[i].tim;

        // Check APB clock gate before touching any registers
        bool clk = (td[i].bus == 1)
            ? ((RCC->APB1ENR & td[i].rcc_bit) != 0)
            : ((RCC->APB2ENR & td[i].rcc_bit) != 0);

        if (!clk) { continue; }

        print(td[i].name); print(": ");

        uint32_t cr1 = t->CR1;

        // CEN
        tabTo(CT_ON); print((cr1 & 1u) ? "ON" : "--");

        // Direction / center-aligned mode (only valid on TIM1-5 and TIM8)
        tabTo(CT_DIR);
        if (td[i].has_dir) {
            uint8_t cms = (uint8_t)((cr1 >> 5) & 3u);
            if      (cms == 0 && !(cr1 & (1u<<4))) print("up");
            else if (cms == 0)                     print("dn");
            else { print("CA"); printDec(cms); }
        } else {
            print("up");
        }

        // One-pulse mode — only printed when set; tabTo handles the gap
        if (cr1 & (1u<<3)) { tabTo(CT_OPM); print("OPM"); }

        // PSC (decimal), ARR and CNT (hex)
        tabTo(CT_PSC); printDec(t->PSC);
        tabTo(CT_ARR); dotnb(8, 8, t->ARR, 16); print("h");
        tabTo(CT_CNT); dotnb(8, 8, t->CNT, 16); print("h");

        // Active CC channels — indent to CT_DIR so they sit under Dir column
        if (!td[i].basic && td[i].nch > 0) {
            uint32_t ccer = t->CCER;
            if (ccer & 0x1111u) {
                uint32_t ccmr1 = t->CCMR1;
                uint32_t ccmr2 = (td[i].nch >= 4) ? t->CCMR2 : 0u;
                printCr(); tabTo(CT_DIR);
                SHOW_CHAN(0, (uint8_t) ccmr1,        ccer, t->CCR1);
                if (td[i].nch >= 2)
                    SHOW_CHAN(1, (uint8_t)(ccmr1>>8), ccer, t->CCR2);
                if (td[i].nch >= 4) {
                    SHOW_CHAN(2, (uint8_t) ccmr2,        ccer, t->CCR3);
                    SHOW_CHAN(3, (uint8_t)(ccmr2>>8),    ccer, t->CCR4);
                }
            }
        }
        printCr();
    }
    print_RTC();
    printCr();
}

void do_reboot(void) {
    print("rebooting..."); printCr();
    NVIC_SystemReset();
}

bool visible_word(char *s) { return true; } // for word filtering; default is none
