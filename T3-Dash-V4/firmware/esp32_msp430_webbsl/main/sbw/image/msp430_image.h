#pragma once

#include <stdint.h>

/* Image data is parsed from the uploaded Intel HEX before calling Program_MSP. */
extern uint16_t *eprom;
extern const uint32_t eprom_address[1];
extern const uint32_t eprom_length_of_sections[1];
extern const uint32_t eprom_sections;
