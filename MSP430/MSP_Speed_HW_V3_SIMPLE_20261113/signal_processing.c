#include <stdint.h>
#include "config.h"
#include "lookup_tables.h"
#include "signal_processing.h"

/* The ADC acquisition remains in main.c and is not modified. */
extern unsigned int adc_values[];

/* Keep channel indices aligned with the original adc_values[] mapping. */
#ifndef ADC_FUEL_INDEX
#define ADC_FUEL_INDEX          0
#define ADC_COOLANT_TEMP_INDEX  1
#define ADC_COOLANT_LEVEL_INDEX 2
#endif

volatile uint8_t coolant_temp_c = 40U;
volatile uint8_t fuel_level_percent = 0U;
volatile uint8_t fuel_gauge_percent_filtered = 0U;
volatile uint8_t coolant_level_ok = 0U;

#if (FUEL_FILTER_ENABLED != 0) && (FUEL_FILTER_MODE != 0)
/* Q5 fixed-point EMA state retains sub-ADC-count precision without float. */
static int32_t fuel_filter_q5 = 0;
static uint8_t fuel_filter_initialized = 0U;
#endif

uint8_t adc_to_lookup_index(uint16_t adc_value)
{
    uint16_t index = (uint16_t)(adc_value >> 2);

    if (index >= ADC_LOOKUP_SIZE)
        index = ADC_LOOKUP_SIZE - 1U;

    return (uint8_t)index;
}

uint8_t clamp_pwm_percent(int16_t value)
{
    if (value < (int16_t)PWM_PERCENT_MIN)
        return (uint8_t)PWM_PERCENT_MIN;

    if (value > (int16_t)PWM_PERCENT_MAX)
        return (uint8_t)PWM_PERCENT_MAX;

    return (uint8_t)value;
}

/*
 * Fuel filter update.
 * MODE 0: disabled (raw ADC-derived physical percentage).
 * MODE 1: symmetric EMA using FUEL_FILTER_EMA_DIV.
 * MODE 2: asymmetric EMA; faster while fuel is consumed, slower while
 *         refuelling. This is the existing 1/8 and 1/32 behavior by default.
 */
static uint8_t update_fuel_gauge_filter(uint16_t raw_adc)
{
#if (FUEL_FILTER_ENABLED == 0) || (FUEL_FILTER_MODE == 0)
    (void)raw_adc;
    return fuel_level_percent;
#else
    int32_t target_q5;
    int32_t difference;
    int32_t divisor;
    uint16_t filtered_adc;

    if (!fuel_filter_initialized)
    {
        fuel_filter_q5 = ((int32_t)raw_adc) << 5;
        fuel_filter_initialized = 1U;
    }
    else
    {
        target_q5 = ((int32_t)raw_adc) << 5;
        difference = target_q5 - fuel_filter_q5;

#if (FUEL_FILTER_MODE == 1)
        divisor = FUEL_FILTER_EMA_DIV;
#else
        /* Existing sensor characteristic: ADC rising means fuel falling. */
        divisor = (difference > 0) ? FUEL_FILTER_FALL_DIV : FUEL_FILTER_RISE_DIV;
#endif
        fuel_filter_q5 += difference / divisor;
    }

    /* Round Q5 back to an integer ADC value and protect the 10-bit range. */
    filtered_adc = (uint16_t)((fuel_filter_q5 + 16L) >> 5);
    if (filtered_adc > 1023U)
        filtered_adc = 1023U;

    return fuel_percent_lookup[adc_to_lookup_index(filtered_adc)];
#endif
}

/*
 * Central sensor conversion stage:
 * raw ADC -> calibrated physical/logical values.
 *
 * This function deliberately does not touch peripheral registers, I2C,
 * UART, interrupts or the main-loop scheduler.
 */
void update_physical_sensor_values(void)
{
    uint8_t index;
    uint16_t raw_fuel_adc;

    index = adc_to_lookup_index((uint16_t)adc_values[ADC_COOLANT_TEMP_INDEX]);
    coolant_temp_c = coolant_temp_lookup[index];

    raw_fuel_adc = (uint16_t)adc_values[ADC_FUEL_INDEX];
    index = adc_to_lookup_index(raw_fuel_adc);
    fuel_level_percent = fuel_percent_lookup[index];

    /* The filtered value is intentionally reserved for the analogue gauge. */
    fuel_gauge_percent_filtered = update_fuel_gauge_filter(raw_fuel_adc);

    /* AC conductivity probe: retain the established threshold and boolean meaning. */
    coolant_level_ok =
        ((uint16_t)adc_values[ADC_COOLANT_LEVEL_INDEX] >= COOLANT_LEVEL_ADC_THRESHOLD)
        ? 1U : 0U;
}
