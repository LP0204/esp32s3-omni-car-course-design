/*
 * HC-SR04 超声波测距 - ESP-IDF 版
 *
 * 接线：TRIG=GPIO18, ECHO=GPIO21
 * 注意：标准 HC-SR04 的 ECHO 高电平约 5V，ESP32-S3 GPIO 只能承受 3.3V，
 * 必须经过分压/电平转换后再接 GPIO21。
 */

#include "ultrasonic.h"

#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define US_TRIG GPIO_NUM_18
#define US_ECHO GPIO_NUM_21

#define ULTRA_SAMPLE_MS          30
#define ULTRA_TIMEOUT_US         4000
#define ULTRA_NO_ECHO_TIMEOUT_MS 500
#define ULTRA_DISPLAY_MAX_CM     400.0f   /* 屏幕显示距离上限（HC-SR04 量程） */

static SemaphoreHandle_t s_us_mutex;
static uint32_t s_last_sample_ms = 0;

static float s_raw_cm = -1.0f;
static float s_display_cm = -1.0f;
static float s_history[3] = { -1.0f, -1.0f, -1.0f };
static int s_hist_index = 0;
static int s_hist_count = 0;
static uint32_t s_last_valid_ms = 0;

static float median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

float ultrasonic_read_once_cm(void)
{
    if (s_us_mutex == NULL) {
        return -1.0f;
    }
    xSemaphoreTake(s_us_mutex, portMAX_DELAY);

    gpio_set_level(US_TRIG, 0);
    esp_rom_delay_us(2);
    gpio_set_level(US_TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(US_TRIG, 0);

    /* 等待回波上升沿（超时 = 无回波） */
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(US_ECHO) == 0) {
        if (esp_timer_get_time() - t0 >= ULTRA_TIMEOUT_US) {
            xSemaphoreGive(s_us_mutex);
            return -1.0f;
        }
    }
    int64_t t1 = esp_timer_get_time();

    /* 测量高电平宽度（上限 ULTRA_TIMEOUT_US） */
    while (gpio_get_level(US_ECHO) == 1) {
        if (esp_timer_get_time() - t1 >= ULTRA_TIMEOUT_US) {
            break;
        }
    }
    int64_t t2 = esp_timer_get_time();
    xSemaphoreGive(s_us_mutex);

    float echo_us = (float)(t2 - t1);
    if (echo_us <= 0.0f) {
        return -1.0f;
    }
    return echo_us * 0.0343f * 0.5f;
}

/*
 * 每 ULTRA_SAMPLE_MS 测一次：
 *  - 障碍触发使用“原始距离”，避免滤波带来的额外延迟；
 *  - 屏幕显示使用 3 点中值滤波后的距离。
 */
void ultrasonic_update(uint32_t now)
{
    if (now - s_last_sample_ms < ULTRA_SAMPLE_MS) {
        return;
    }
    s_last_sample_ms = now;

    float d = ultrasonic_read_once_cm();
    s_raw_cm = d;

    if (d > 1.5f && d < ULTRA_DISPLAY_MAX_CM) {
        s_last_valid_ms = now;
        s_history[s_hist_index] = d;
        s_hist_index = (s_hist_index + 1) % 3;
        if (s_hist_count < 3) {
            s_hist_count++;
        }

        if (s_hist_count == 1) {
            s_display_cm = s_history[0];
        } else if (s_hist_count == 2) {
            s_display_cm = (s_history[0] + s_history[1]) * 0.5f;
        } else {
            s_display_cm = median3(s_history[0], s_history[1], s_history[2]);
        }
    } else if (now - s_last_valid_ms > ULTRA_NO_ECHO_TIMEOUT_MS) {
        s_display_cm = -1.0f;
    }
}

float ultrasonic_raw_cm(void)
{
    return s_raw_cm;
}

float ultrasonic_display_cm(void)
{
    return s_display_cm;
}

void ultrasonic_init(void)
{
    s_us_mutex = xSemaphoreCreateMutex();

    gpio_set_direction(US_TRIG, GPIO_MODE_OUTPUT);
    gpio_set_level(US_TRIG, 0);
    gpio_set_direction(US_ECHO, GPIO_MODE_INPUT);
}
