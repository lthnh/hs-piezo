idf_component_register(SRCS "main.c" "bluetooth.c" "adc.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_driver_spi bt nvs_flash)