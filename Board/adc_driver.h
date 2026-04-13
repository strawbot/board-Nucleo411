/**
 * @file    adc_driver.h
 * @brief   Multi-channel ADC driver for STM32F411RE (Nucleo-F411RE)
 *
 * Reads 5 channels via single-shot polling on the sole ADC (ADC1):
 *   IN0  (PA0), IN1 (PA1),
 *   Internal temperature sensor (CH16),
 *   VREFINT (CH17), VBAT/2 (CH18)
 *
 * Built on top of the ST LL (Low-Level) driver – no HAL required.
 * No DMA is used; all conversions are software-triggered single-shot.
 *
 * -----------------------------------------------------------------------
 * CubeMX setup notes
 * -----------------------------------------------------------------------
 *  • Do NOT enable ADC1 or DMA in CubeMX for these channels; this driver
 *    owns ADC1 entirely.
 *  • Do set your system/APB2 clocks in CubeMX as usual.
 *  • If printf-based output is desired, enable UART and retarget _write()
 *    (or use ITM / semihosting as preferred).
 * -----------------------------------------------------------------------
 */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * Configuration
 * ---------------------------------------------------------------------- */

/** Total number of channels managed by this driver. */
#define ADC_NUM_CHANNELS        5U

/**
 * ADC clock prescaler.
 * APB2 is 84 MHz on the F411 at 84 MHz core (APB2 prescaler = /1).
 * /4  → 21 MHz  (recommended; fits 10 µs internal-channel requirement)
 * /8  → 10.5 MHz (more margin, lower throughput)
 */
#define ADC_PRESCALER           LL_ADC_CLOCK_SYNC_PCLK_DIV4

/**
 * Software oversampling factor for ADC_SimBurstRead / ADC_GetLastRaw.
 *
 * ADC_SimBurstRead() takes ADC_OVERSAMPLE sequential pairs and averages
 * them, acting as a box-filter FIR.  No DMA is used.
 *
 * At 21 MHz ADC clock with 28-cycle sample time:
 *   one pair ≈ 1.93 µs  →  20 pairs ≈ 38.6 µs total
 */
#define ADC_OVERSAMPLE          20U

/* -------------------------------------------------------------------------
 * Channel index – sequential read order (rank 1 … rank 5 on ADC1)
 * ---------------------------------------------------------------------- */
typedef enum
{
    ADC_IDX_IN0     = 0,  /**< PA0  – ADC1_IN0  (supply sense divider)  */
    ADC_IDX_IN1     = 1,  /**< PA1  – ADC1_IN1  (sense resistor node)   */
    ADC_IDX_TEMP    = 2,  /**< Die temperature sensor – ADC1_IN16        */
    ADC_IDX_VREFINT = 3,  /**< Internal 1.21 V reference – ADC1_IN17    */
    ADC_IDX_VBAT    = 4,  /**< VBAT/2 bridge – ADC1_IN18 (F411: ÷2)    */
} ADC_ChannelIdx_t;

/* -------------------------------------------------------------------------
 * Result structure
 * ---------------------------------------------------------------------- */
typedef struct
{
    uint16_t raw[ADC_NUM_CHANNELS]; /**< Raw 12-bit ADC codes (0–4095)   */

    /**
     * Calibrated voltages in millivolts.
     *   [IN0]      – PA0 pin voltage 0–VDDA  (supply sense divider output)
     *   [IN1]      – PA1 pin voltage 0–VDDA  (sense resistor node)
     *   [TEMP]     – NOT a voltage; see temperature_c below
     *   [VREFINT]  – internal reference (~1210 mV, useful sanity check)
     *   [VBAT]     – actual VBAT in mV (÷2 bridge divider already removed)
     */
    float voltage_mv[ADC_NUM_CHANNELS];

    float vdda_mv;        /**< Calibrated VDDA derived from VREFINT (mV)  */
    float temperature_c;  /**< Die temperature (°C), factory-cal formula   */
    float vbat_v;         /**< Battery voltage (V) = voltage_mv[VBAT]/1000 */
} ADC_Results_t;

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief  Initialise GPIO pins (PA0, PA1) and ADC1 in single-shot mode.
 *         No DMA is used; all conversions are software-triggered.
 *         Call once before the main loop (or after every low-power wakeup).
 */
void ADC_Driver_Init(void);

/**
 * @brief  Snapshot the DMA buffer and compute calibrated values.
 *
 * Safe to call from any context (briefly disables interrupts during copy).
 * Call at whatever rate you need; conversions run independently in hardware.
 *
 * @param[out] results  Populated with the latest readings.
 */
void ADC_Driver_Update(ADC_Results_t *results);

/**
 * @brief  Print a formatted table of all readings via printf().
 *
 * Requires printf to be redirected to a UART / ITM port.
 *
 * @param[in]  results  Previously filled by ADC_Driver_Update().
 */
void ADC_Driver_PrintAll(const ADC_Results_t *results);

/**
 * @brief  Read the two external muscle-wire channels via single-shot ADC1.
 *
 * Fires one software-triggered conversion per channel and polls EOC.
 * Blocking time ≈ 2 × 1.93 µs = 3.86 µs at 21 MHz ADC clock.
 *
 * @param[out] in0_out  Raw 12-bit code for IN0 (PA0 — supply sense).
 * @param[out] in1_out  Raw 12-bit code for IN1 (PA1 — node sense).
 */
void ADC_GetLastRaw(uint16_t *in0_out, uint16_t *in1_out);

/**
 * @brief  Return the most recently calibrated VDDA in millivolts.
 *
 * Updated by each call to ADC_Driver_Update().  Defaults to 3300 mV until
 * a full update has been performed.  Used by lightweight callers that
 * obtain raw codes via ADC_GetLastRaw() and need to convert to voltages.
 *
 * Formula: voltage_mv = raw * ADC_GetVDDA_mv() / 4095.0f
 */
float ADC_GetVDDA_mv(void);

/* -------------------------------------------------------------------------
 * Sequential single-shot sampling on ADC1  (IN0 then IN1)
 *
 * STM32F411 has only one ADC, so true simultaneous dual-ADC capture is not
 * available.  The Sim* API is retained with identical signatures so that
 * muscle_wire.c compiles unchanged, but internally each "pair" is two
 * back-to-back single-shot reads on ADC1.
 *
 * Supply-rail noise no longer cancels in the ratio (samples are ~2 µs
 * apart), but at the measurement frequencies used by the muscle-wire driver
 * (~10 Hz) this is not significant.
 *
 * Sampling time: 28 ADC clock cycles per channel.
 *   28-cycle sample + 12.5-cycle convert = 40.5 cycles @ 21 MHz ≈ 1.93 µs
 *   per channel.  One "pair" (IN0 then IN1) ≈ 3.86 µs.
 *
 * Preferred usage (muscle-wire measurement path):
 *   ADC_SimInit();                              // once, from MW_Init()
 *   ADC_SimBurstRead(&vsup, &vsense, 4);        // 4 pairs ≈ 15 µs, averaged
 *
 * Low-level primitives (single-pair):
 *   ADC_SimTrigger();
 *   while (!ADC_SimReady()) {}
 *   ADC_SimRead(&vsup, &vsense);
 * ---------------------------------------------------------------------- */

/** @brief  Configure ADC1 for sequential single-shot on IN0 and IN1.
 *          Sets sample times; call once from MW_Init() after ADC_Driver_Init(). */
void ADC_SimInit(void);

/** @brief  Arm one conversion of IN0 (supply sense).
 *          Re-sets ADC1 rank-1 to IN0 and clears stale EOC flag. */
void ADC_SimTrigger(void);

/** @brief  Return true when ADC1 EOC flag is set (IN0 conversion done). */
bool ADC_SimReady(void);

/** @brief  Read one pair after ADC_SimReady() returns true.
 *
 *  Reads ADC1 IN0 (vsup) then immediately triggers and reads IN1 (vsense).
 *  Sequential, not simultaneous.
 *
 *  @param[out] raw_vsup   12-bit result — IN0 (PA0, supply divider).
 *  @param[out] raw_vsense 12-bit result — IN1 (PA1, sense resistor). */
void ADC_SimRead(uint16_t *raw_vsup, uint16_t *raw_vsense);

/** @brief  Take N sequential pairs back-to-back and return the average.
 *
 *  N × (trigger IN0 → wait → read, trigger IN1 → wait → read), averaged.
 *  Blocking time: N × ~3.86 µs (two 28-cycle reads @ 21 MHz ADC clock).
 *  Safe to call from ISR context for small N (e.g. N=4 → ~15 µs).
 *
 *  @param[out] raw_vsup    Averaged 12-bit result for IN0 (supply divider).
 *  @param[out] raw_vsense  Averaged 12-bit result for IN1 (sense resistor).
 *  @param[in]  n           Number of pairs to accumulate (1–16). */
void ADC_SimBurstRead(volatile uint16_t *raw_vsup, volatile uint16_t *raw_vsense, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* ADC_DRIVER_H */