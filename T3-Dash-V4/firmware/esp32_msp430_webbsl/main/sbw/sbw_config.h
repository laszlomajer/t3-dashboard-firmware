#pragma once

#include "driver/gpio.h"

/* Default pins for a classic ESP32 DevKit; change to match your board/wiring. */
#define SBW_TEST_GPIO GPIO_NUM_18  /* MSP430 TEST / SBWTCK */
#define SBW_RESET_GPIO GPIO_NUM_19 /* MSP430 RST / SBWTDIO (bidirectional) */
