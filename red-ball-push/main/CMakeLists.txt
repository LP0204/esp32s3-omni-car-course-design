idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        usb
        esp_psram
        esp_driver_uart
        esp_driver_gpio
        esp_driver_ledc
        esp_timer
        esp_jpeg
)
