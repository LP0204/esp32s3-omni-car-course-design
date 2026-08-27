#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ======================== User-adjustable parameters ======================== */
#define TARGET_ANGLE_DEG 90.0f     /* positive=right, negative=left */
#define TURN_POWER_PERCENT 45      /* 35..100 */
#define TURN_MS_PER_DEGREE 8.0f    /* calibrate on the real car */

/* Logical +1 means this wheel contributes to clockwise chassis rotation. */
#define LEFT_ELECTRICAL_SIGN (-1)
#define BACK_ELECTRICAL_SIGN (-1)
#define RIGHT_ELECTRICAL_SIGN 1

#define MIN_EFFECTIVE_DUTY_PERCENT 35
#define PWM_FREQUENCY_HZ 20000
#define DIRECTION_DEAD_TIME_US 300
#define CONTROL_INTERVAL_MS 10
#define BUTTON_DEBOUNCE_MS 40

#define BUTTON_GPIO GPIO_NUM_0

#define LEFT_IN1 GPIO_NUM_8
#define LEFT_IN2 GPIO_NUM_9
#define LEFT_PWM GPIO_NUM_10
#define MOTOR_STBY GPIO_NUM_11
#define BACK_IN1 GPIO_NUM_12
#define BACK_IN2 GPIO_NUM_13
#define BACK_PWM GPIO_NUM_14
#define RIGHT_IN1 GPIO_NUM_15
#define RIGHT_IN2 GPIO_NUM_16
#define RIGHT_PWM GPIO_NUM_17

typedef struct {
    gpio_num_t in1;
    gpio_num_t in2;
    gpio_num_t pwm_gpio;
    ledc_channel_t pwm_channel;
    int electrical_sign;
    int last_direction;
} motor_t;

static const char *TAG = "ANGLE_TURN_TEST";

static motor_t left_motor = {
    .in1 = LEFT_IN1, .in2 = LEFT_IN2, .pwm_gpio = LEFT_PWM,
    .pwm_channel = LEDC_CHANNEL_0,
    .electrical_sign = LEFT_ELECTRICAL_SIGN,
};
static motor_t back_motor = {
    .in1 = BACK_IN1, .in2 = BACK_IN2, .pwm_gpio = BACK_PWM,
    .pwm_channel = LEDC_CHANNEL_1,
    .electrical_sign = BACK_ELECTRICAL_SIGN,
};
static motor_t right_motor = {
    .in1 = RIGHT_IN1, .in2 = RIGHT_IN2, .pwm_gpio = RIGHT_PWM,
    .pwm_channel = LEDC_CHANNEL_2,
    .electrical_sign = RIGHT_ELECTRICAL_SIGN,
};

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static uint32_t percent_to_duty(int percent)
{
    const uint32_t maximum = (1U << LEDC_TIMER_10_BIT) - 1U;
    return maximum * (uint32_t)clamp_int(percent, 0, 100) / 100U;
}

static void set_pwm(motor_t *motor, int percent)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->pwm_channel,
                                  percent_to_duty(percent)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->pwm_channel));
}

static void motor_stop(motor_t *motor)
{
    set_pwm(motor, 0);
    ESP_ERROR_CHECK(gpio_set_level(motor->in1, 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2, 0));
    motor->last_direction = 0;
}

static void motor_write(motor_t *motor, int command_percent)
{
    command_percent = clamp_int(command_percent, -100, 100);
    if (command_percent == 0) {
        motor_stop(motor);
        return;
    }
    int direction = command_percent > 0 ? 1 : -1;
    direction *= motor->electrical_sign;
    int magnitude = abs(command_percent);
    if (magnitude < MIN_EFFECTIVE_DUTY_PERCENT) {
        magnitude = MIN_EFFECTIVE_DUTY_PERCENT;
    }
    if (motor->last_direction != 0 && motor->last_direction != direction) {
        set_pwm(motor, 0);
        esp_rom_delay_us(DIRECTION_DEAD_TIME_US);
    }
    ESP_ERROR_CHECK(gpio_set_level(motor->in1, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2, direction < 0));
    set_pwm(motor, magnitude);
    motor->last_direction = direction;
}

static void stop_chassis(void)
{
    motor_stop(&left_motor);
    motor_stop(&back_motor);
    motor_stop(&right_motor);
}

static void rotate_chassis(int signed_power)
{
    /* Logical + commands rotate the chassis clockwise. */
    motor_write(&left_motor, signed_power);
    motor_write(&back_motor, signed_power);
    motor_write(&right_motor, signed_power);
}

static void configure_hardware(void)
{
    const gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << LEFT_IN1) | (1ULL << LEFT_IN2) |
                        (1ULL << BACK_IN1) | (1ULL << BACK_IN2) |
                        (1ULL << RIGHT_IN1) | (1ULL << RIGHT_IN2) |
                        (1ULL << MOTOR_STBY),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&outputs));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY, 0));

    const gpio_config_t button = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button));

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    motor_t *motors[] = {&left_motor, &back_motor, &right_motor};
    for (int i = 0; i < 3; ++i) {
        const ledc_channel_config_t channel = {
            .gpio_num = motors[i]->pwm_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[i]->pwm_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }
    stop_chassis();
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY, 1));
}

static bool button_pressed_event(void)
{
    static int previous = 1;
    static int64_t last_change_ms = 0;
    const int current = gpio_get_level(BUTTON_GPIO);
    const int64_t now_ms = esp_timer_get_time() / 1000;

    if (current != previous && now_ms - last_change_ms >= BUTTON_DEBOUNCE_MS) {
        previous = current;
        last_change_ms = now_ms;
        return current == 0;
    }
    return false;
}

void app_main(void)
{
    configure_hardware();
    const float absolute_angle = TARGET_ANGLE_DEG >= 0.0f
                                     ? TARGET_ANGLE_DEG : -TARGET_ANGLE_DEG;
    const int64_t turn_duration_ms = (int64_t)(absolute_angle *
                                               TURN_MS_PER_DEGREE + 0.5f);
    const int power = clamp_int(TURN_POWER_PERCENT, 35, 100) *
                      (TARGET_ANGLE_DEG >= 0.0f ? 1 : -1);
    bool running = false;
    int64_t start_ms = 0;

    ESP_LOGW(TAG, "先架空小车检查三轮方向，再进行地面测试");
    ESP_LOGI(TAG, "BOOT: start/emergency-stop; target=%.1fdeg; power=%d%%; time=%lldms",
             (double)TARGET_ANGLE_DEG, power, turn_duration_ms);

    while (true) {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        if (button_pressed_event()) {
            if (running) {
                stop_chassis();
                running = false;
                ESP_LOGI(TAG, "emergency stop");
            } else if (turn_duration_ms > 0) {
                rotate_chassis(power);
                start_ms = now_ms;
                running = true;
                ESP_LOGI(TAG, "fixed-angle turn started");
            }
        }
        if (running && now_ms - start_ms >= turn_duration_ms) {
            stop_chassis();
            running = false;
            ESP_LOGI(TAG, "target time reached; stopped automatically");
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
    }
}
