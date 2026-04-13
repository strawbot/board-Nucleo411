/**
 * @file    adc_driver.c
 * @brief   Multi-channel ADC driver implementation – STM32F411RE, LL drivers.
 *
 * Hardware resources used
 * -----------------------
 *   ADC1           – all channels, single-shot software-triggered (no DMA)
 *   GPIOA pin 0    – IN0  (ADC1_IN0, supply sense divider output)
 *   GPIOA pin 1    – IN1  (ADC1_IN1, sense resistor node)
 *   (internal channels need no GPIO)
 *
 * ADC1 – single-shot reads (all channels, sequential)
 * ----------------------------------------------------
 *   External channels (via ADC_SimBurstRead / ADC_GetLastRaw):
 *     IN0  (PA0)   28-cycle sample  (~1.93 µs @ 21 MHz ADC clk)
 *     IN1  (PA1)   28-cycle sample
 *
 *   Internal channels (via ADC_Driver_Update / read_internal_channels):
 *     Pass A  TSVREFE=1, VBATE=0:
 *       TEMP    (CH16)  480-cycle sample  (~22.9 µs)
 *       VREFINT (CH17)  480-cycle sample
 *     Pass B  TSVREFE=0, VBATE=1:
 *       VBAT    (CH18)  480-cycle sample
 *
 *   RM0383 §13.3.3 (ADC_CCR) warns: "VBATE and TSVREFE bits cannot be set
 *   at the same time – when both are set, only the VBAT conversion is
 *   performed."  The two-pass sequence keeps them mutually exclusive.
 *
 * Note: STM32F411 has only one ADC (ADC1).  The ADC_Sim* API retains its
 * original signatures but implements sequential single-shot reads instead
 * of true simultaneous dual-ADC capture.
 *
 * Calibration
 * -----------
 *   VDDA is derived at run-time from the factory VREFINT calibration word
 *   stored at 0x1FFF7A2A (measured at 3.3 V, 30 °C).
 *
 *   Temperature uses the two-point factory calibration:
 *     CAL1 (30 °C)  @ 0x1FFF7A2C
 *     CAL2 (110 °C) @ 0x1FFF7A2E
 *
 *   VBAT: the F411 silicon connects VBAT through a /2 resistor bridge to IN18.
 */

#include "adc_driver.h"

#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rcc.h"
#include "cmsis_compiler.h"   /* __disable_irq / __enable_irq              */

#include <string.h>           /* memcpy                                    */
#include "printers.h"         /* print / printDec / tabTo / printCr / maybeCr */

/* =========================================================================
 * Private constants
 * ====================================================================== */

/*
 * stm32f4xx_ll_adc.h already defines VREFINT_CAL_ADDR, TEMPSENSOR_CAL1_ADDR,
 * and TEMPSENSOR_CAL2_ADDR.  Guard our definitions so they are only provided
 * if an older or stripped-down LL header omits them.
 */
#ifndef VREFINT_CAL_ADDR
#define VREFINT_CAL_ADDR         ((const uint16_t *)0x1FFF7A2AU)
#endif

/** Voltage at which factory VREFINT calibration was performed (mV). */
#define VREFINT_CAL_VREF_MV      3300U

#ifndef TEMPSENSOR_CAL1_ADDR
#define TEMPSENSOR_CAL1_ADDR     ((const uint16_t *)0x1FFF7A2CU)  /* 30 °C  */
#endif
#ifndef TEMPSENSOR_CAL2_ADDR
#define TEMPSENSOR_CAL2_ADDR     ((const uint16_t *)0x1FFF7A2EU)  /* 110 °C */
#endif
#define TEMPSENSOR_CAL1_TEMP_C   30.0f
#define TEMPSENSOR_CAL2_TEMP_C   110.0f

/**
 * VBAT hardware bridge divider on STM32F411 (RM0383 §13).
 * The ADC sees VBAT/2; multiply the raw reading back by 2 to recover VBAT.
 */
#define VBAT_BRIDGE_DIVIDER      2U

/** ADC full-scale value for 12-bit resolution. */
#define ADC_FULL_SCALE           4095.0f

/*
 * ADC clock prescaler.
 *
 * STM32F411 @ 84 MHz: APB2 = 84 MHz (APB2 prescaler /1).  ADC maximum
 * clock = 36 MHz (RM0383 §13.3.1).  DIV4 → 21 MHz is the correct choice.
 *
 * DIV2 → 42 MHz is over-spec and gives inaccurate readings for the
 * high-impedance temperature sensor.
 *
 * If ADC_PRESCALER is already defined in a project-wide header this
 * fallback is skipped.  Verify it equals LL_ADC_CLOCK_SYNC_PCLK_DIV4.
 */
#ifndef ADC_PRESCALER
#define ADC_PRESCALER            LL_ADC_CLOCK_SYNC_PCLK_DIV4
#endif

/* =========================================================================
 * Module state
 * ====================================================================== */

/* Last calibrated VDDA — updated by ADC_Driver_Update(), read by ADC_GetVDDA_mv().
 * Initialised to the nominal 3300 mV so callers get a reasonable value before
 * the first full update. */
static float s_vdda_mv = 3300.0f;

/* =========================================================================
 * Private helper – GPIO
 * ====================================================================== */

static void adc_gpio_init(void)
{
    /* Enable GPIOA clock (PA0 = IN0, PA1 = IN1).
     * Note: PA3 is USART2_RX on the Nucleo-F411RE and must NOT be
     * configured as analog.  PB0/PB1/PC2 are not used on this board. */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef gpio_cfg = {
        .Mode       = LL_GPIO_MODE_ANALOG,
        .Pull       = LL_GPIO_PULL_NO,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,  /* ignored in analog mode */
        .Speed      = LL_GPIO_SPEED_FREQ_LOW,
        .Alternate  = LL_GPIO_AF_0
    };

    /* PA0  → ADC1_IN0  (supply sense divider output) */
    gpio_cfg.Pin = LL_GPIO_PIN_0;
    LL_GPIO_Init(GPIOA, &gpio_cfg);

    /* PA1  → ADC1_IN1  (sense resistor node) */
    gpio_cfg.Pin = LL_GPIO_PIN_1;
    LL_GPIO_Init(GPIOA, &gpio_cfg);
}

/* =========================================================================
 * Private helper – ADC1 (all channels, single-shot, no DMA)
 *
 * STM32F411 has only one ADC.  All channels — external (IN0, IN1) and
 * internal (TEMP, VREFINT, VBAT) — are read via software-triggered
 * single-shot conversions on ADC1.
 * ====================================================================== */

static void adc1_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

    /* ----- Common (ADC clock prescaler) --------------------------------- */

    /* ADC clock: synchronous, APB2 with chosen prescaler.
     * F411 @ 84 MHz: APB2 = 84 MHz.  DIV4 → 21 MHz.                      */
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), ADC_PRESCALER);

    /*
     * Enable TSVREFE only – do NOT set VBATE here.
     * RM0383 §13.3.3: "When both VBATE and TSVREFE bits are set, only the
     * VBAT conversion is performed."  VBATE is toggled only during the
     * dedicated VBAT read in read_internal_channels().
     */
    LL_ADC_SetCommonPathInternalCh(
        __LL_ADC_COMMON_INSTANCE(ADC1),
        LL_ADC_PATH_INTERNAL_TEMPSENSOR |
        LL_ADC_PATH_INTERNAL_VREFINT);

    /* ----- ADC1 instance ------------------------------------------------ */

    LL_ADC_SetResolution    (ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment (ADC1, LL_ADC_DATA_ALIGN_RIGHT);

    /* Single-rank mode: one channel at a time, selected per-call.         */
    LL_ADC_SetSequencersScanMode  (ADC1, LL_ADC_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetTriggerSource   (ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetContinuousMode  (ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetSequencerLength (ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);

    /* No DMA – results are read directly from DR after each conversion.   */
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

    /* ----- Sample times for external channels --------------------------- */

    /* 28 cycles @ 21 MHz ≈ 1.33 µs sample + 0.60 µs convert = 1.93 µs.  */
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_28CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_28CYCLES);

    /* ----- Sample times for internal channels --------------------------- */

    /* 480 cycles @ 21 MHz ≈ 22.9 µs  (≥ 10 µs required by datasheet).   */
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_480CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT,    LL_ADC_SAMPLINGTIME_480CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VBAT,       LL_ADC_SAMPLINGTIME_480CYCLES);

    /* ----- Enable ------------------------------------------------------- */

    LL_ADC_Enable(ADC1);

    /* t_STAB stabilisation delay before first conversion.
     * At 84 MHz core, 500 NOPs ≈ 6 µs.                                   */
    for (volatile uint32_t i = 0U; i < 500U; i++) { __NOP(); }

    /* EOC set after each individual conversion (not only at sequence end). */
    LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);

    /* No continuous start – conversions are triggered on demand only.     */
}

/* =========================================================================
 * Public API
 * ====================================================================== */

void ADC_Driver_Init(void)
{
    adc_gpio_init();   /* PA0, PA1 → analog mode                         */
    adc1_init();       /* ADC1: all channels, single-shot, no DMA        */
}

/* =========================================================================
 * Private helper – single-shot regular conversion on ADC1
 *
 * Sets rank-1 to the requested channel, fires a software trigger, waits
 * for EOC, and returns the 12-bit result.  ADC1 has no continuous mode
 * and no DMA, so there is no ongoing conversion to interrupt.
 * ====================================================================== */

static uint16_t adc1_read_channel(uint32_t channel)
{
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);

    /*
     * Clear any stale EOC/EOCS flag before triggering.  If a previous
     * conversion left the flag set (e.g. DR was read by the compiler but
     * the flag persisted due to an optimisation artefact), the poll below
     * would return immediately and read old data.
     *
     * STM32F4 LL naming: the EOC status-register bit is exposed as
     * LL_ADC_IsActiveFlag_EOCS / LL_ADC_ClearFlag_EOCS (the "EOCS" suffix
     * reflects that the bit's meaning is controlled by the EOCS bit in CR2).
     */
    LL_ADC_ClearFlag_EOCS(ADC1);

    LL_ADC_REG_StartConversionSWStart(ADC1);
    while (!LL_ADC_IsActiveFlag_EOCS(ADC1)) {}
    const uint16_t raw = (uint16_t)LL_ADC_REG_ReadConversionData12(ADC1);
    LL_ADC_ClearFlag_EOCS(ADC1);
    return raw;
}

/* =========================================================================
 * Private helper – on-demand reads for all three internal channels
 *
 * Pass A (TSVREFE=1, VBATE=0): TEMP then VREFINT.
 * Pass B (TSVREFE=0, VBATE=1): VBAT.
 *
 * RM0383 §13.3.3: VBATE and TSVREFE must never be set simultaneously –
 * when both are set, only the VBAT conversion is performed (CH16 receives
 * the VBAT signal instead of the temperature sensor).  The two-pass
 * sequence guarantees they are never asserted at the same time.
 *
 * External channels (IN0, IN1) are not affected by the CCR path-select
 * bits; those gate only the internal multiplexers.
 * ====================================================================== */

static void read_internal_channels(uint16_t *temp_raw,
                                   uint16_t *vref_raw,
                                   uint16_t *vbat_raw)
{
    /* ------------------------------------------------------------------
     * Pass A: TSVREFE=1, VBATE=0  →  TEMP + VREFINT
     * ---------------------------------------------------------------- */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                   LL_ADC_PATH_INTERNAL_TEMPSENSOR |
                                   LL_ADC_PATH_INTERNAL_VREFINT);

    /* Allow the internal mux switch to settle (~200 NOPs ≈ 1.2 µs).    */
    for (volatile uint32_t i = 0U; i < 200U; i++) { __NOP(); }

    *temp_raw = adc1_read_channel(LL_ADC_CHANNEL_TEMPSENSOR);
    *vref_raw = adc1_read_channel(LL_ADC_CHANNEL_VREFINT);

    /* ------------------------------------------------------------------
     * Pass B: TSVREFE=0, VBATE=1  →  VBAT
     * ---------------------------------------------------------------- */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                   LL_ADC_PATH_INTERNAL_VBAT);

    for (volatile uint32_t i = 0U; i < 200U; i++) { __NOP(); }

    *vbat_raw = adc1_read_channel(LL_ADC_CHANNEL_VBAT);

    /* Restore TSVREFE so the sensor stays powered between calls.        */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                   LL_ADC_PATH_INTERNAL_TEMPSENSOR |
                                   LL_ADC_PATH_INTERNAL_VREFINT);
}

/* ------------------------------------------------------------------------- */

void ADC_Driver_Update(ADC_Results_t *results)
{
    /* ------------------------------------------------------------------
     * Step 1: single-shot reads on ADC1 for IN0 (vsup) and IN1 (vsense).
     *
     * STM32F411 has only one ADC, so reads are sequential (not simultaneous).
     * adc1_read_channel() points ADC1 rank-1 at each channel in turn.
     * ---------------------------------------------------------------- */
    results->raw[ADC_IDX_IN0] = adc1_read_channel(LL_ADC_CHANNEL_0);
    results->raw[ADC_IDX_IN1] = adc1_read_channel(LL_ADC_CHANNEL_1);

    /* ------------------------------------------------------------------
     * Step 2: single-shot reads on ADC1 for TEMP, VREFINT, and VBAT.
     *
     * adc1_read_channel() temporarily points ADC1 rank-1 at the internal
     * channel under measurement.  Restore rank-1 to IN0 afterward so
     * that the next ADC_SimTrigger() call picks up the right channel.
     * ---------------------------------------------------------------- */
    read_internal_channels(&results->raw[ADC_IDX_TEMP],
                           &results->raw[ADC_IDX_VREFINT],
                           &results->raw[ADC_IDX_VBAT]);

    /* Restore ADC1 rank-1 → IN0 for subsequent ADC_SimTrigger() calls. */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);

    /* ------------------------------------------------------------------
     * Step 3: calibrate VDDA from the VREFINT factory cal word.
     *
     *   VDDA_mV = VREFINT_CAL_VREF_MV × VREFINT_CAL / VREFINT_raw
     * ---------------------------------------------------------------- */
    const uint16_t vrefint_cal = *VREFINT_CAL_ADDR;
    const float vdda_mv =
        (float)VREFINT_CAL_VREF_MV * (float)vrefint_cal
        / (float)results->raw[ADC_IDX_VREFINT];

    results->vdda_mv = vdda_mv;
    s_vdda_mv        = vdda_mv;     /* cache for ADC_GetVDDA_mv() callers */

    /* ------------------------------------------------------------------
     * Step 4: external channel voltages.
     *
     *   V_pin_mV = raw × VDDA_mV / 4095
     * ---------------------------------------------------------------- */
    for (int i = ADC_IDX_IN0; i <= ADC_IDX_IN1; i++)
    {
        results->voltage_mv[i] = (float)results->raw[i] * vdda_mv / ADC_FULL_SCALE;
    }

    /* ------------------------------------------------------------------
     * Step 5: VREFINT voltage (sanity check – should be ~1210 mV).
     * ---------------------------------------------------------------- */
    results->voltage_mv[ADC_IDX_VREFINT] =
        (float)results->raw[ADC_IDX_VREFINT] * vdda_mv / ADC_FULL_SCALE;

    results->voltage_mv[ADC_IDX_TEMP] = 0.0f;   /* not a real voltage — moved earlier */

    /* ------------------------------------------------------------------
     * Step 6: die temperature – two-point factory calibration.
     *
     * The factory cal words (CAL1 @ 30 °C, CAL2 @ 110 °C) were sampled at
     * exactly VDDA = 3300 mV.  The temperature sensor output is an absolute
     * voltage (not ratiometric to VDDA): for the same die temperature, a
     * lower VDDA raises the raw ADC count because the ADC full-scale drops.
     *
     * Normalise raw_temp back to the 3300 mV reference so it is directly
     * comparable to CAL1/CAL2:
     *
     *   ts_scaled = raw_temp × VDDA_mV / VREFINT_CAL_VREF_MV
     *
     * Note the direction: multiply by VDDA/3300 (not 3300/VDDA).
     * Multiplying the wrong way compounds the error instead of cancelling
     * it, and produces wildly high temperatures when VDDA < 3300 mV.
     *
     *   T(°C) = (CAL2_TEMP − CAL1_TEMP) × (ts_scaled − CAL1)
     *           ─────────────────────────────────────────────── + CAL1_TEMP
     *                        (CAL2 − CAL1)
     * ---------------------------------------------------------------- */
    const uint16_t ts_cal1 = *TEMPSENSOR_CAL1_ADDR;
    const uint16_t ts_cal2 = *TEMPSENSOR_CAL2_ADDR;

    const float ts_scaled =
        (float)results->raw[ADC_IDX_TEMP]
        * vdda_mv / (float)VREFINT_CAL_VREF_MV;

    results->temperature_c =
        (TEMPSENSOR_CAL2_TEMP_C - TEMPSENSOR_CAL1_TEMP_C)
        * (ts_scaled - (float)ts_cal1)
        / (float)(ts_cal2 - ts_cal1)
        + TEMPSENSOR_CAL1_TEMP_C;

    /* ------------------------------------------------------------------
     * Step 7: VBAT – F411 hardware divides VBAT by 2 before the ADC.
     *
     *   VBAT_mV = raw_vbat × VDDA_mV / 4095 × VBAT_BRIDGE_DIVIDER (×2)
     * ---------------------------------------------------------------- */
    const float vbat_mv =
        (float)results->raw[ADC_IDX_VBAT]
        * vdda_mv / ADC_FULL_SCALE
        * (float)VBAT_BRIDGE_DIVIDER;

    results->voltage_mv[ADC_IDX_VBAT] = vbat_mv;
    results->vbat_v = vbat_mv / 1000.0f;
}

/* =========================================================================
 * ADC_GetLastRaw / ADC_GetVDDA_mv  — accessors
 *
 * ADC_GetLastRaw() performs two sequential single-shot reads on ADC1.
 * No DMA buffer exists on the F411.  Prefer ADC_SimBurstRead() for
 * oversampled readings in the muscle-wire path.
 * ====================================================================== */

void ADC_GetLastRaw(uint16_t *in0_out, uint16_t *in1_out)
{
    *in0_out = adc1_read_channel(LL_ADC_CHANNEL_0);
    *in1_out = adc1_read_channel(LL_ADC_CHANNEL_1);
    /* Restore rank-1 → IN0 for subsequent ADC_SimTrigger() calls.        */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
}

float ADC_GetVDDA_mv(void)
{
    return s_vdda_mv;
}

/* =========================================================================
 * Sequential single-shot  (ADC1: IN0 then IN1)
 *
 * STM32F411 has only one ADC (ADC1).  The ADC_Sim* API retains identical
 * signatures so that muscle_wire.c compiles unchanged.  Internally each
 * "pair" is two back-to-back single-shot reads on ADC1 (IN0 for vsup,
 * IN1 for vsense), not a true simultaneous dual-ADC capture.
 *
 * Conflict handling with ADC_Driver_Update
 * -----------------------------------------
 * ADC_Driver_Update() calls adc1_read_channel() for VREFINT/TEMP/VBAT,
 * which temporarily points ADC1 rank-1 at those internal channels.
 * ADC_SimTrigger() and ADC_SimBurstRead() unconditionally restore rank-1
 * to IN0 before each trigger, so stale assignments are harmless.
 * ====================================================================== */

void ADC_SimInit(void)
{
    /* Sample times for external channels were already set in adc1_init().
     * This function exists for API compatibility with the F407 version and
     * to confirm rank-1 points at IN0 ready for the first trigger.        */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
    LL_ADC_ClearFlag_EOCS(ADC1);
}

void ADC_SimTrigger(void)
{
    /* Re-assert IN0 on ADC1 rank-1.  A preceding ADC_Driver_Update() call
     * may have left rank-1 pointing at VREFINT, TEMP, or VBAT.            */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
    LL_ADC_ClearFlag_EOCS(ADC1);
    LL_ADC_REG_StartConversionSWStart(ADC1);
}

bool ADC_SimReady(void)
{
    /* Single ADC — just check ADC1 EOC.                                   */
    return (LL_ADC_IsActiveFlag_EOCS(ADC1) != 0U);
}

void ADC_SimRead(uint16_t *raw_vsup, uint16_t *raw_vsense)
{
    /* Read IN0 result that ADC_SimTrigger() started.                      */
    *raw_vsup = (uint16_t)(ADC1->DR & 0x0FFFu);   /* clears EOC           */

    /* Now read IN1 (vsense) with a fresh single-shot.                     */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
    LL_ADC_ClearFlag_EOCS(ADC1);
    LL_ADC_REG_StartConversionSWStart(ADC1);
    while (!LL_ADC_IsActiveFlag_EOCS(ADC1)) {}
    *raw_vsense = (uint16_t)(ADC1->DR & 0x0FFFu);

    /* Restore rank-1 → IN0 for the next trigger cycle.                   */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
}

/* =========================================================================
 * ADC_SimBurstRead — N sequential pairs on ADC1, averaged
 *
 * For each of N iterations: read IN0 (vsup) then IN1 (vsense) via separate
 * single-shot conversions.  Average the results.
 *
 * Blocking time: N × ~3.86 µs (two 28-cycle reads @ 21 MHz).
 *   N=4 → ~15 µs.  Safe in ISR context at the expected call rate.
 * ====================================================================== */

void ADC_SimBurstRead(volatile uint16_t *raw_vsup, volatile uint16_t *raw_vsense, uint8_t n)
{
    uint32_t sum0 = 0u;
    uint32_t sum1 = 0u;

    for (uint8_t i = 0u; i < n; i++)
    {
        /* ── IN0 (vsup) ─────────────────────────────────────────────────── */
        LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
        LL_ADC_ClearFlag_EOCS(ADC1);
        LL_ADC_REG_StartConversionSWStart(ADC1);
        while (!LL_ADC_IsActiveFlag_EOCS(ADC1)) {}
        sum0 += ADC1->DR & 0x0FFFu;    /* reading DR clears EOC            */

        /* ── IN1 (vsense) ───────────────────────────────────────────────── */
        LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
        LL_ADC_ClearFlag_EOCS(ADC1);
        LL_ADC_REG_StartConversionSWStart(ADC1);
        while (!LL_ADC_IsActiveFlag_EOCS(ADC1)) {}
        sum1 += ADC1->DR & 0x0FFFu;
    }

    /* Restore rank-1 → IN0 for subsequent ADC_SimTrigger() calls.        */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);

    *raw_vsup   = (uint16_t)(sum0 / (uint32_t)n);
    *raw_vsense = (uint16_t)(sum1 / (uint32_t)n);
}

/* =========================================================================
 * ADC_Driver_PrintAll  (uses printers.h API)
 * ====================================================================== */

/*
 * Column layout:
 *   Col  0 : channel label
 *   Col 12 : raw 12-bit code
 *   Col 20 : converted value + unit
 */
#define COL_RAW    12
#define COL_VALUE  20

/* Print one row: label | raw code | float value | unit string. */
static void print_row(const char *label,
                      uint32_t    raw,
                      float       value,
                      uint8_t     decimals,
                      const char *unit)
{
    maybeCr();
    print(label);   tabTo(COL_RAW);
    printDec(raw);  tabTo(COL_VALUE);
    printFloat(value, decimals);
    print(unit);
    printCr();
}

/* ------------------------------------------------------------------------- */

void ADC_Driver_PrintAll(const ADC_Results_t *results)
{
    maybeCr();
    print("--- ADC readings (STM32F411RE) ---");
    printCr();

    /* Header */
    print("Channel");  tabTo(COL_RAW);
    print("Raw");      tabTo(COL_VALUE);
    print("Value");
    printCr();

    /* VDDA – derived from VREFINT, no raw code of its own */
    maybeCr();
    print("VDDA");     tabTo(COL_RAW);
    print("----");     tabTo(COL_VALUE);
    printFloat(results->vdda_mv, 2);
    print(" mV (cal)");
    printCr();

    /* External channels – 2 decimal places in mV */
    print_row("IN0  PA0",  results->raw[ADC_IDX_IN0],     results->voltage_mv[ADC_IDX_IN0],     2, " mV");
    print_row("IN1  PA1",  results->raw[ADC_IDX_IN1],     results->voltage_mv[ADC_IDX_IN1],     2, " mV");

    /* Internal channels */
    print_row("TEMP",      results->raw[ADC_IDX_TEMP],    results->temperature_c,               1, " C (die)");
    print_row("VREFINT",   results->raw[ADC_IDX_VREFINT], results->voltage_mv[ADC_IDX_VREFINT], 2, " mV");
    print_row("VBAT",      results->raw[ADC_IDX_VBAT],    results->vbat_v,                      3, " V (x2)");
}

void show_adc() {
    ADC_Results_t adc;
    ADC_Driver_Update(&adc);
    ADC_Driver_PrintAll(&adc);
}

