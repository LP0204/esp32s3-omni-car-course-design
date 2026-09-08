idf_component_register(
    SRCS "main.c" "tft18_sensor_display.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_timer esp_hw_support
)
