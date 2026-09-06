idf_component_register(
    SRCS "main.c" "motor.c" "ultrasonic.c" "follower.c" "tft.c" "ball_vision.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb esp_ringbuf esp_psram esp_timer esp_driver_gpio
                 esp_driver_ledc esp_driver_uart esp_driver_spi
                 esp_rom heap freertos
)
