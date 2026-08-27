#pragma once

#include <stdint.h>

#include "esp_err.h"

/* Initialize the four IR inputs and LQ_TFT18SPIV33 display. */
esp_err_t tft18_sensor_display_init(void);

/* bit3..bit0 = OUT4..OUT1; raw 0=black, raw 1=white. */
uint8_t tft18_sensor_display_read_raw(void);

/* Display OUT1, OUT2, OUT3, OUT4 from left to right. */
esp_err_t tft18_sensor_display_update(uint8_t raw);
