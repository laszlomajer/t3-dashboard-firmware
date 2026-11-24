#ifndef T3_CONFIG_H
#define T3_CONFIG_H

/*
 * T3 MSP430G2553 - compile-time application configuration.
 *
 * This file contains DATA-PROCESSING parameters only. Do not place clock,
 * GPIO, ADC peripheral, timer, interrupt, I2C, UART or main-loop settings here.
 */

/* ---------------- Fuel level filter ---------------- */
#ifndef FUEL_FILTER_ENABLED
#define FUEL_FILTER_ENABLED          1
#endif

/* 0 = no filtering, 1 = symmetric EMA, 2 = asymmetric EMA. */
#ifndef FUEL_FILTER_MODE
#define FUEL_FILTER_MODE             2
#endif

/* ADC rises when the calibrated fuel level falls. */
#ifndef FUEL_FILTER_FALL_DIV
#define FUEL_FILTER_FALL_DIV         16
#endif
#ifndef FUEL_FILTER_RISE_DIV
#define FUEL_FILTER_RISE_DIV         32
#endif

/* Used only when FUEL_FILTER_MODE == 1. */
#ifndef FUEL_FILTER_EMA_DIV
#define FUEL_FILTER_EMA_DIV          16
#endif

/* ---------------- Vehicle warning thresholds ---------------- */
#define FUEL_WARN_PERCENT            10U
#define COOLANT_TEMP_WARNING_C       95U
#define COOLANT_LEVEL_ADC_THRESHOLD  500U

/* ---------------- Display calibration / dimmer ---------------- */
#define DIMMER_LOOKUP_OFFSET         20U
#define PWM_PERCENT_MIN              0U
#define PWM_PERCENT_MAX              100U

/* ---------------- Lookup dimensions ---------------- */
#define ADC_LOOKUP_SIZE              256U
#define COOLANT_TEMP_MIN_C           0U
#define COOLANT_TEMP_MAX_C           115U
#define FUEL_PERCENT_MIN             0U
#define FUEL_PERCENT_MAX             100U

/* Compile-time sanity checks supported by the MSP430 C compiler. */
#if (FUEL_FILTER_MODE < 0) || (FUEL_FILTER_MODE > 2)
#error Invalid FUEL_FILTER_MODE (use 0, 1 or 2)
#endif

#if (FUEL_FILTER_FALL_DIV < 1) || (FUEL_FILTER_RISE_DIV < 1) || (FUEL_FILTER_EMA_DIV < 1)
#error Fuel filter divisors must be greater than zero
#endif

#endif /* T3_CONFIG_H */
