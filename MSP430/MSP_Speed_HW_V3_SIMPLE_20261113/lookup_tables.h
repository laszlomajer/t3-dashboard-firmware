// lookup_tables.h
#ifndef LOOKUP_TABLES_H
#define LOOKUP_TABLES_H

#include <stdint.h>

#define LOOKUP_TABLE_SIZE 256



extern const uint8_t fuel_percent_lookup[LOOKUP_TABLE_SIZE];

extern const uint8_t coolant_temp_lookup[LOOKUP_TABLE_SIZE];

extern const uint8_t dcbus_lookup[LOOKUP_TABLE_SIZE];

extern const uint8_t dimmer_lookup[LOOKUP_TABLE_SIZE];

/* Physical-value indexed gauge calibration tables.
 * Coolant index: integer degrees Celsius (0..115).
 * Fuel index: integer percent (0..100).
 */
extern const uint8_t coolant_gauge_pwm_lookup[116];
extern const uint8_t fuel_gauge_pwm_lookup[101];

#endif // LOOKUP_TABLES_H