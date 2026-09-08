#pragma once

#include <stdint.h>
#include "esp_err.h"

esp_err_t tft18_sensor_display_init(void);
uint8_t tft18_sensor_display_read_raw(void);
void tft18_sensor_display_update(uint8_t raw);
void tft18_sensor_display_set_motor_commands(int left, int right, int rear);
void tft18_sensor_display_set_distance_mm(int distance_mm);
