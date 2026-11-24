idf_component_register(
    SRCS "main.c" "sbw/SBW430.c" "sbw/gpio_if.c"
    INCLUDE_DIRS "." "sbw"
    REQUIRES freertos heap esp_driver_gpio esp_rom esp_wifi esp_event esp_netif esp_http_server nvs_flash esp_timer
)
