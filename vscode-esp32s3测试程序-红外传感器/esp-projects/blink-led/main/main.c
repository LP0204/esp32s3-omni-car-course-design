#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * 四路红外循迹模块接线：
 *   VCC  -> ESP32-S3 3V3
 *   GND  -> ESP32-S3 GND
 *   OUT1 -> GPIO4
 *   OUT2 -> GPIO5
 *   OUT3 -> GPIO6
 *   OUT4 -> GPIO7
 *
 * 所给模块资料说明：黑色输出低电平，白色输出高电平。
 * 如果实测逻辑相反，把 BLACK_LEVEL 改为 1 即可。
 */
#define IR_OUT1_GPIO GPIO_NUM_4
#define IR_OUT2_GPIO GPIO_NUM_5
#define IR_OUT3_GPIO GPIO_NUM_6
#define IR_OUT4_GPIO GPIO_NUM_7

#define BLACK_LEVEL 0
#define SAMPLE_COUNT 5
#define SAMPLE_INTERVAL_MS 5
#define PRINT_INTERVAL_MS 300

static const char *TAG = "IR_TEST";

static const gpio_num_t sensor_gpios[4] = {
    IR_OUT1_GPIO,
    IR_OUT2_GPIO,
    IR_OUT3_GPIO,
    IR_OUT4_GPIO,
};

static int read_sensor_stable(gpio_num_t gpio)
{
    int high_count = 0;

    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        high_count += gpio_get_level(gpio);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }

    return high_count > (SAMPLE_COUNT / 2) ? 1 : 0;
}

static const char *surface_name(int level)
{
    return level == BLACK_LEVEL ? "黑" : "白";
}

void app_main(void)
{
    uint64_t pin_mask = 0;
    for (int i = 0; i < 4; ++i) {
        pin_mask |= 1ULL << sensor_gpios[i];
    }

    gpio_config_t io_config = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));

    ESP_LOGI(TAG, "四路红外黑白识别测试开始");
    ESP_LOGI(TAG, "接线: OUT1/2/3/4 -> GPIO4/5/6/7, VCC -> 3V3, GND -> GND");
    ESP_LOGI(TAG, "判定规则: 黑=%d, 白=%d", BLACK_LEVEL, 1 - BLACK_LEVEL);

    while (1) {
        int level[4];
        int black_count = 0;
        uint8_t raw_bits = 0;

        for (int i = 0; i < 4; ++i) {
            level[i] = read_sensor_stable(sensor_gpios[i]);
            raw_bits |= (uint8_t)(level[i] << i);
            if (level[i] == BLACK_LEVEL) {
                ++black_count;
            }
        }

        ESP_LOGI(TAG,
                 "OUT1(GPIO4)=%d[%s]  OUT2(GPIO5)=%d[%s]  "
                 "OUT3(GPIO6)=%d[%s]  OUT4(GPIO7)=%d[%s]  "
                 "raw=0x%X  黑色通道数=%d",
                 level[0], surface_name(level[0]),
                 level[1], surface_name(level[1]),
                 level[2], surface_name(level[2]),
                 level[3], surface_name(level[3]),
                 raw_bits, black_count);

        vTaskDelay(pdMS_TO_TICKS(PRINT_INTERVAL_MS));
    }
}
