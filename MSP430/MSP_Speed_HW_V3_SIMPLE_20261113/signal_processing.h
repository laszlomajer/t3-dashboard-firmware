#ifndef T3_SIGNAL_PROCESSING_H
#define T3_SIGNAL_PROCESSING_H

#include <stdint.h>

/* Values in engineering units / logical states. */
extern volatile uint8_t coolant_temp_c;
extern volatile uint8_t fuel_level_percent;
extern volatile uint8_t fuel_gauge_percent_filtered;
extern volatile uint8_t coolant_level_ok;

/* Called at the same point in the existing main-loop processing sequence. */
void update_physical_sensor_values(void);

/* Defensive index and output helpers used by the application layer. */
uint8_t adc_to_lookup_index(uint16_t adc_value);
uint8_t clamp_pwm_percent(int16_t value);

#endif /* T3_SIGNAL_PROCESSING_H */
