idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio driver led_strip bt nvs_flash spiffs fatfs esp_adc esp_driver_spi)
