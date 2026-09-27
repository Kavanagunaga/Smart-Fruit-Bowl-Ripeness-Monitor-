idf_component_register(
    SRCS
        "bme_isr.c"
        "bme68x.c"
        "bme68x.h"
        "bme68x_defs.h"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_driver_i2c
        esp_timer
        esp_hw_support
        mqtt esp_wifi nvs_flash
        esp_driver_gpio
        lwip
        esp_netif
         esp_event
)
