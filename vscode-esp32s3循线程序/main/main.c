#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ------------------------- 安全开关与可调参数 ------------------------- */

/*
 * 初次烧录保持为 0：只读传感器并计算三个轮子的指令，不驱动电机。
 * 完成三个电机通道和方向校准后，再改为 1。
 */
#define ENABLE_MOTOR_OUTPUT 0

/* 传感器资料给出的逻辑：黑线为低电平，白底为高电平。 */
#define BLACK_LEVEL 0

/* 假定从车头向前看，IR1 到 IR4 按从左到右排列。相反则改为 -1。 */
#define SENSOR_ORDER_SIGN 1

/* 如果检测到左侧黑线后车辆反而向右修正，把它改为 -1。 */
#define STEERING_SIGN 1

/* 电机接线极性；某个轮子的正负方向相反时只修改对应值。 */
#define RIGHT_ELECTRICAL_SIGN 1
#define LEFT_ELECTRICAL_SIGN 1
#define BACK_ELECTRICAL_SIGN 1

/* 三全向轮底盘的前进/旋转速度合成符号。 */
#define RIGHT_FORWARD_SIGN 1
#define LEFT_FORWARD_SIGN (-1)
#define BACK_FORWARD_SIGN 0
#define RIGHT_TURN_SIGN 1
#define LEFT_TURN_SIGN 1
#define BACK_TURN_SIGN 1

#define BASE_SPEED_PERCENT 45
#define MIN_EFFECTIVE_DUTY_PERCENT 35
#define MAX_TURN_PERCENT 50

/* 定点 PD 参数：实际 Kp=12/100，Kd=4/100。 */
#define KP_NUMERATOR 12
#define KD_NUMERATOR 4
#define CONTROL_GAIN_DENOMINATOR 100

#define PWM_FREQUENCY_HZ 20000
#define CONTROL_INTERVAL_MS 10
#define SENSOR_SAMPLE_COUNT 3
#define SENSOR_SAMPLE_INTERVAL_MS 1
#define START_DELAY_MS 3000
#define LOG_INTERVAL_MS 200

/* ------------------------------ GPIO ------------------------------ */

#define IR1_GPIO GPIO_NUM_4
#define IR2_GPIO GPIO_NUM_5
#define IR3_GPIO GPIO_NUM_6
#define IR4_GPIO GPIO_NUM_7

#define RIGHT_IN1 GPIO_NUM_8
#define RIGHT_IN2 GPIO_NUM_9
#define RIGHT_PWM GPIO_NUM_10

#define MOTOR_STBY GPIO_NUM_11

#define LEFT_IN1 GPIO_NUM_12
#define LEFT_IN2 GPIO_NUM_13
#define LEFT_PWM GPIO_NUM_14

#define BACK_IN1 GPIO_NUM_15
#define BACK_IN2 GPIO_NUM_16
#define BACK_PWM GPIO_NUM_17

typedef struct {
    const char *name;
    gpio_num_t in1;
    gpio_num_t in2;
    gpio_num_t pwm_gpio;
    ledc_channel_t pwm_channel;
    int electrical_sign;
    int last_direction;
} motor_t;

typedef enum {
    FOLLOW_TRACKING,
    FOLLOW_LINE_LOST,
    FOLLOW_INTERSECTION,
} follow_state_t;

static const char *TAG = "LINE_FOLLOW";

static const gpio_num_t sensor_gpios[4] = {
    IR1_GPIO,
    IR2_GPIO,
    IR3_GPIO,
    IR4_GPIO,
};

/* 由左到右的位置权重，数值放大100倍以避免浮点运算。 */
static const int sensor_weights[4] = {-300, -100, 100, 300};

static motor_t right_motor = {
    .name = "RIGHT",
    .in1 = RIGHT_IN1,
    .in2 = RIGHT_IN2,
    .pwm_gpio = RIGHT_PWM,
    .pwm_channel = LEDC_CHANNEL_0,
    .electrical_sign = RIGHT_ELECTRICAL_SIGN,
    .last_direction = 0,
};

static motor_t left_motor = {
    .name = "LEFT",
    .in1 = LEFT_IN1,
    .in2 = LEFT_IN2,
    .pwm_gpio = LEFT_PWM,
    .pwm_channel = LEDC_CHANNEL_1,
    .electrical_sign = LEFT_ELECTRICAL_SIGN,
    .last_direction = 0,
};

static motor_t back_motor = {
    .name = "BACK",
    .in1 = BACK_IN1,
    .in2 = BACK_IN2,
    .pwm_gpio = BACK_PWM,
    .pwm_channel = LEDC_CHANNEL_2,
    .electrical_sign = BACK_ELECTRICAL_SIGN,
    .last_direction = 0,
};

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint32_t percent_to_duty(int percent)
{
    const uint32_t max_duty = (1U << LEDC_TIMER_10_BIT) - 1U;
    percent = clamp_int(percent, 0, 100);
    return (max_duty * (uint32_t)percent) / 100U;
}

static void set_motor_pwm(motor_t *motor, int percent)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                  motor->pwm_channel,
                                  percent_to_duty(percent)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                     motor->pwm_channel));
}

static void motor_write(motor_t *motor, int command_percent)
{
    command_percent = clamp_int(command_percent, -100, 100);

    if (!ENABLE_MOTOR_OUTPUT || command_percent == 0) {
        set_motor_pwm(motor, 0);
        ESP_ERROR_CHECK(gpio_set_level(motor->in1, 0));
        ESP_ERROR_CHECK(gpio_set_level(motor->in2, 0));
        motor->last_direction = 0;
        return;
    }

    int direction = command_percent > 0 ? 1 : -1;
    direction *= motor->electrical_sign;
    int magnitude = abs(command_percent);

    if (magnitude < MIN_EFFECTIVE_DUTY_PERCENT) {
        magnitude = MIN_EFFECTIVE_DUTY_PERCENT;
    }

    /* 反转前先关PWM，避免H桥方向切换瞬间产生大电流。 */
    if (direction != motor->last_direction) {
        set_motor_pwm(motor, 0);
    }

    ESP_ERROR_CHECK(gpio_set_level(motor->in1, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2, direction < 0));
    set_motor_pwm(motor, magnitude);
    motor->last_direction = direction;
}

static void stop_all_motors(void)
{
    motor_write(&right_motor, 0);
    motor_write(&left_motor, 0);
    motor_write(&back_motor, 0);
}

static void mix_omni_commands(int forward_percent,
                              int turn_percent,
                              int *right_command,
                              int *left_command,
                              int *back_command)
{
    *right_command = clamp_int(RIGHT_FORWARD_SIGN * forward_percent +
                                   RIGHT_TURN_SIGN * turn_percent,
                               -100, 100);
    *left_command = clamp_int(LEFT_FORWARD_SIGN * forward_percent +
                                  LEFT_TURN_SIGN * turn_percent,
                              -100, 100);
    *back_command = clamp_int(BACK_FORWARD_SIGN * forward_percent +
                                  BACK_TURN_SIGN * turn_percent,
                              -100, 100);
}

static uint8_t read_black_mask(void)
{
    int black_votes[4] = {0, 0, 0, 0};

    for (int sample = 0; sample < SENSOR_SAMPLE_COUNT; ++sample) {
        for (int i = 0; i < 4; ++i) {
            if (gpio_get_level(sensor_gpios[i]) == BLACK_LEVEL) {
                ++black_votes[i];
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_SAMPLE_INTERVAL_MS));
    }

    uint8_t mask = 0;
    for (int i = 0; i < 4; ++i) {
        if (black_votes[i] > SENSOR_SAMPLE_COUNT / 2) {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

static int count_black_sensors(uint8_t mask)
{
    int count = 0;
    for (int i = 0; i < 4; ++i) {
        count += (mask >> i) & 1U;
    }
    return count;
}

static int calculate_line_error(uint8_t black_mask, int black_count)
{
    int weighted_sum = 0;
    for (int i = 0; i < 4; ++i) {
        if (black_mask & (1U << i)) {
            weighted_sum += sensor_weights[i];
        }
    }
    return SENSOR_ORDER_SIGN * weighted_sum / black_count;
}

static const char *state_name(follow_state_t state)
{
    switch (state) {
    case FOLLOW_TRACKING:
        return "TRACK";
    case FOLLOW_LINE_LOST:
        return "LOST-STOP";
    case FOLLOW_INTERSECTION:
        return "INTERSECTION-STOP";
    default:
        return "UNKNOWN";
    }
}

static void configure_sensors(void)
{
    const gpio_config_t input_config = {
        .pin_bit_mask = (1ULL << IR1_GPIO) | (1ULL << IR2_GPIO) |
                        (1ULL << IR3_GPIO) | (1ULL << IR4_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_config));
}

static void configure_motors(void)
{
    const gpio_config_t output_config = {
        .pin_bit_mask =
            (1ULL << RIGHT_IN1) | (1ULL << RIGHT_IN2) |
            (1ULL << MOTOR_STBY) |
            (1ULL << LEFT_IN1) | (1ULL << LEFT_IN2) |
            (1ULL << BACK_IN1) | (1ULL << BACK_IN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_config));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY, 0));

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    motor_t *motors[] = {&right_motor, &left_motor, &back_motor};
    for (int i = 0; i < 3; ++i) {
        const ledc_channel_config_t channel_config = {
            .gpio_num = motors[i]->pwm_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[i]->pwm_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    }

    stop_all_motors();
}

void app_main(void)
{
    configure_sensors();
    configure_motors();

    ESP_LOGI(TAG, "三轮全向小车循线程序启动");
    ESP_LOGI(TAG, "IR1~IR4 -> GPIO4~GPIO7，黑线电平=%d", BLACK_LEVEL);
    ESP_LOGW(TAG, "电机输出：%s",
             ENABLE_MOTOR_OUTPUT ? "已启用" : "安全关闭（只输出计算日志）");
    ESP_LOGI(TAG, "%d秒后开始控制", START_DELAY_MS / 1000);
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));

    if (ENABLE_MOTOR_OUTPUT) {
        ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY, 1));
    }

    int last_error = 0;
    TickType_t last_log_tick = 0;

    while (1) {
        uint8_t black_mask = read_black_mask();
        int black_count = count_black_sensors(black_mask);
        follow_state_t state;
        int error = last_error;
        int turn_command = 0;
        int right_command = 0;
        int left_command = 0;
        int back_command = 0;

        if (black_count == 0) {
            state = FOLLOW_LINE_LOST;
            stop_all_motors();
        } else if (black_count >= 3) {
            state = FOLLOW_INTERSECTION;
            stop_all_motors();
        } else {
            state = FOLLOW_TRACKING;
            error = calculate_line_error(black_mask, black_count);
            int derivative = error - last_error;
            turn_command = STEERING_SIGN *
                           (KP_NUMERATOR * error + KD_NUMERATOR * derivative) /
                           CONTROL_GAIN_DENOMINATOR;
            turn_command = clamp_int(turn_command,
                                     -MAX_TURN_PERCENT,
                                     MAX_TURN_PERCENT);

            mix_omni_commands(BASE_SPEED_PERCENT,
                               turn_command,
                               &right_command,
                               &left_command,
                               &back_command);
            motor_write(&right_motor, right_command);
            motor_write(&left_motor, left_command);
            motor_write(&back_motor, back_command);
            last_error = error;
        }

        TickType_t now = xTaskGetTickCount();
        if (now - last_log_tick >= pdMS_TO_TICKS(LOG_INTERVAL_MS)) {
            ESP_LOGI(TAG,
                     "mask=0x%X state=%s error=%d turn=%d cmd[R,L,B]=[%d,%d,%d]",
                     black_mask,
                     state_name(state),
                     error,
                     turn_command,
                     right_command,
                     left_command,
                     back_command);
            last_log_tick = now;
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
    }
}
