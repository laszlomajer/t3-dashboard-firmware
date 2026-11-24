#pragma once

#include <stdbool.h>
#include <stdint.h>

void GPIO_IF_setOutputLowOnPin(uint32_t pin);
void GPIO_IF_setOutputHighOnPin(uint32_t pin);
void GPIO_IF_setAsInputPin(uint32_t pin);
void GPIO_IF_setAsOutputPin(uint32_t pin);
int GPIO_IF_getPinStatus(uint32_t pin);
void SBW_port_init(void);
void SBW_port_enter_critical(void);
void SBW_port_exit_critical(void);
