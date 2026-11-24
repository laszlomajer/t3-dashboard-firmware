#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "gpio_if.h"
#include "sbw_config.h"

/* The TI bit macros require uninterrupted sub-microsecond GPIO transitions.
 * A short critical section per SBW shift operation is provided by the port. */
static portMUX_TYPE sbw_gpio_mux = portMUX_INITIALIZER_UNLOCKED;

void GPIO_IF_setOutputLowOnPin(uint32_t pin)
{
    gpio_set_level((gpio_num_t)pin, 0);
}

void GPIO_IF_setOutputHighOnPin(uint32_t pin)
{
    gpio_set_level((gpio_num_t)pin, 1);
}

void GPIO_IF_setAsInputPin(uint32_t pin)
{
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
}

void GPIO_IF_setAsOutputPin(uint32_t pin)
{
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
}

int GPIO_IF_getPinStatus(uint32_t pin)
{
    return gpio_get_level((gpio_num_t)pin);
}

void SBW_port_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << SBW_TEST_GPIO) | (1ULL << SBW_RESET_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(SBW_TEST_GPIO, 1);
    gpio_set_level(SBW_RESET_GPIO, 1);
}

void SBW_port_enter_critical(void)
{
    portENTER_CRITICAL(&sbw_gpio_mux);
}

void SBW_port_exit_critical(void)
{
    portEXIT_CRITICAL(&sbw_gpio_mux);
}

void MsDelay(uint32_t time)
{
    vTaskDelay(pdMS_TO_TICKS(time));
}

void usDelay(uint32_t time)
{
    esp_rom_delay_us(time);
}
