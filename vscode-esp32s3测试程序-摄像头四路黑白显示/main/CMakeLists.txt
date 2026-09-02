idf_component_register(
    SRCS
        "main.c"
        "tft18_status_display.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        usb
        esp_psram
        esp_driver_uart
        esp_driver_gpio
        esp_driver_ledc
        esp_driver_spi
        esp_timer
        esp_jpeg
)
