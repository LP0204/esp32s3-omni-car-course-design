#pragma once

#include <stdint.h>

#include "esp_err.h"

/* Initialize the IR inputs and start the 200 ms background display task. */
esp_err_t tft18_sensor_display_init(void);

/* bit3..bit0 = OUT4..OUT1; raw 0=black, raw 1=white. */
uint8_t tft18_sensor_display_read_raw(void);

/* Compatibility helper: publish a raw IR sample for the next frame. */
esp_err_t tft18_sensor_display_update(uint8_t raw);

/* Publish logical motor commands in the range -100..100. */
void tft18_sensor_display_set_motor_commands(int left_percent,
                                              int right_percent,
                                              int back_percent);

/* Publish HC-SR04 distance in mm; pass -1 when unavailable. */
void tft18_sensor_display_set_distance_mm(int distance_mm);
