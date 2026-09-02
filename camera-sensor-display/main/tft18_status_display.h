#pragma once

#include <stdint.h>

#include "esp_err.h"

/* LQ_TFT18SPIV33 status display used by the vision line follower. */
esp_err_t tft18_status_display_init(void);

/* Publish logical motor commands in the range -100..100. */
void tft18_status_display_set_motor_commands(int left_percent,
                                              int right_percent,
                                              int back_percent);

/* HC-SR04 distance in millimetres; a negative value is displayed as D:---. */
void tft18_status_display_set_distance_mm(int distance_mm);

/* Four virtual camera sensors, bit3..bit0 = leftmost..rightmost; 1=black. */
void tft18_status_display_set_virtual_black_mask(uint8_t black_mask);
