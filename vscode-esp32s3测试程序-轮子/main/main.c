#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Motor D：左前轮。 */
#define DIN1 GPIO_NUM_8
#define DIN2 GPIO_NUM_9
#define PWMD GPIO_NUM_10

#define STBY GPIO_NUM_11

/* Motor B。 */
#define BIN1 GPIO_NUM_12
#define BIN2 GPIO_NUM_13
#define PWMB GPIO_NUM_14

/* Motor A：右前轮。 */
#define AIN1 GPIO_NUM_15
#define AIN2 GPIO_NUM_16
#define PWMA GPIO_NUM_17

#define PWM_FREQUENCY_HZ 20000
#define DEFAULT_SPEED_PERCENT 100
#define SPEED_STEP_PERCENT 10
#define DIRECTION_DEAD_TIME_MS 50

typedef struct {
    const char *name;
    gpio_num_t in1;
    gpio_num_t in2;
    gpio_num_t pwm_gpio;
    ledc_channel_t pwm_channel;
} motor_t;

typedef enum {
    MOTION_STOPPED,
    MOTION_A_FORWARD,
    MOTION_A_REVERSE,
    MOTION_B_FORWARD,
    MOTION_B_REVERSE,
    MOTION_D_FORWARD,
    MOTION_D_REVERSE,
    MOTION_ALL_FORWARD,
    MOTION_ALL_REVERSE,
} motion_mode_t;

static const char *TAG = "WHEEL_TEST";

static motor_t motor_a = {
    .name = "Motor A（右前轮）",
    .in1 = AIN1,
    .in2 = AIN2,
    .pwm_gpio = PWMA,
    .pwm_channel = LEDC_CHANNEL_2,
};

static motor_t motor_b = {
    .name = "Motor B（后轮）",
    .in1 = BIN1,
    .in2 = BIN2,
    .pwm_gpio = PWMB,
    .pwm_channel = LEDC_CHANNEL_1,
};

static motor_t motor_d = {
    .name = "Motor D（左前轮）",
    .in1 = DIN1,
    .in2 = DIN2,
    .pwm_gpio = PWMD,
    .pwm_channel = LEDC_CHANNEL_0,
};

static int speed_percent = DEFAULT_SPEED_PERCENT;
static motion_mode_t current_motion = MOTION_STOPPED;

static int clamp_speed(int value)
{
    if (value < 10) {
        return 10;
    }
    if (value > 100) {
        return 100;
    }
    return value;
}

static uint32_t percent_to_duty(int percent)
{
    const uint32_t max_duty = (1U << LEDC_TIMER_10_BIT) - 1U;
    return (max_duty * (uint32_t)percent) / 100U;
}

static void set_pwm(motor_t *motor, int percent)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                  motor->pwm_channel,
                                  percent_to_duty(percent)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,
                                     motor->pwm_channel));
}

/* 与已经验证的Arduino程序一致：停止时只关闭三个PWM/使能脚。 */
static void stop_all_pwm(void)
{
    set_pwm(&motor_a, 0);
    set_pwm(&motor_b, 0);
    set_pwm(&motor_d, 0);
}

static void set_direction(motor_t *motor, int direction)
{
    ESP_ERROR_CHECK(gpio_set_level(motor->in1, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2, direction < 0));
}

static void prepare_direction_change(void)
{
    stop_all_pwm();
    vTaskDelay(pdMS_TO_TICKS(DIRECTION_DEAD_TIME_MS));
}

static void run_single(motor_t *motor,
                       int direction,
                       motion_mode_t new_motion)
{
    prepare_direction_change();
    set_direction(motor, direction);
    set_pwm(motor, speed_percent);
    current_motion = new_motion;
    ESP_LOGI(TAG,
             "%s %s，速度 %d%%",
             motor->name,
             direction > 0 ? "正转" : "反转",
             speed_percent);
}

static void run_all(int direction, motion_mode_t new_motion)
{
    prepare_direction_change();
    set_direction(&motor_a, direction);
    set_direction(&motor_b, direction);
    set_direction(&motor_d, direction);
    set_pwm(&motor_a, speed_percent);
    set_pwm(&motor_b, speed_percent);
    set_pwm(&motor_d, speed_percent);
    current_motion = new_motion;
    ESP_LOGI(TAG,
             "三个电机同时%s，速度 %d%%",
             direction > 0 ? "正转" : "反转",
             speed_percent);
}

static void stop_all(void)
{
    stop_all_pwm();
    current_motion = MOTION_STOPPED;
    ESP_LOGI(TAG, "所有电机已停止");
}

static void update_running_speed(void)
{
    switch (current_motion) {
    case MOTION_A_FORWARD:
    case MOTION_A_REVERSE:
        set_pwm(&motor_a, speed_percent);
        break;
    case MOTION_B_FORWARD:
    case MOTION_B_REVERSE:
        set_pwm(&motor_b, speed_percent);
        break;
    case MOTION_D_FORWARD:
    case MOTION_D_REVERSE:
        set_pwm(&motor_d, speed_percent);
        break;
    case MOTION_ALL_FORWARD:
    case MOTION_ALL_REVERSE:
        set_pwm(&motor_a, speed_percent);
        set_pwm(&motor_b, speed_percent);
        set_pwm(&motor_d, speed_percent);
        break;
    case MOTION_STOPPED:
    default:
        break;
    }
}

static void set_speed(int requested_percent)
{
    speed_percent = clamp_speed(requested_percent);
    update_running_speed();
    ESP_LOGI(TAG, "速度设置为 %d%%", speed_percent);
    if (speed_percent < 30) {
        ESP_LOGW(TAG, "低于30%%时，电机可能只有声音而无法启动");
    }
}

static void print_help(void)
{
    ESP_LOGI(TAG, "========== 三轮电机测试命令 ==========");
    ESP_LOGI(TAG, "A/a：Motor A（右前轮 GPIO15/16/17）正转/反转");
    ESP_LOGI(TAG, "B/b：Motor B（后轮 GPIO12/13/14）正转/反转");
    ESP_LOGI(TAG, "D/d：Motor D（左前轮 GPIO8/9/10）正转/反转");
    ESP_LOGI(TAG, "F/R：三个电机同时正转/反转");
    ESP_LOGI(TAG, "S：停止所有电机");
    ESP_LOGI(TAG, "+/-：速度增加/减少10%%");
    ESP_LOGI(TAG, "1~9：速度设为10%%~90%%，0：速度设为100%%");
    ESP_LOGI(TAG, "H：重新显示帮助");
    ESP_LOGI(TAG, "注意：三轮同向转动不等于车体直线前进");
    ESP_LOGI(TAG, "=====================================");
}

static void configure_motor_hardware(void)
{
    const gpio_config_t output_config = {
        .pin_bit_mask =
            (1ULL << AIN1) | (1ULL << AIN2) | (1ULL << STBY) |
            (1ULL << BIN1) | (1ULL << BIN2) |
            (1ULL << DIN1) | (1ULL << DIN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_config));

    ESP_ERROR_CHECK(gpio_set_level(STBY, 0));
    ESP_ERROR_CHECK(gpio_set_level(AIN1, 0));
    ESP_ERROR_CHECK(gpio_set_level(AIN2, 0));
    ESP_ERROR_CHECK(gpio_set_level(BIN1, 0));
    ESP_ERROR_CHECK(gpio_set_level(BIN2, 0));
    ESP_ERROR_CHECK(gpio_set_level(DIN1, 0));
    ESP_ERROR_CHECK(gpio_set_level(DIN2, 0));

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    motor_t *motors[] = {&motor_a, &motor_b, &motor_d};
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

    stop_all_pwm();
    ESP_ERROR_CHECK(gpio_set_level(STBY, 1));
}

static void handle_command(char command)
{
    switch (command) {
    case 'A':
        run_single(&motor_a, 1, MOTION_A_FORWARD);
        break;
    case 'a':
        run_single(&motor_a, -1, MOTION_A_REVERSE);
        break;
    case 'B':
        run_single(&motor_b, 1, MOTION_B_FORWARD);
        break;
    case 'b':
        run_single(&motor_b, -1, MOTION_B_REVERSE);
        break;
    case 'D':
        run_single(&motor_d, 1, MOTION_D_FORWARD);
        break;
    case 'd':
        run_single(&motor_d, -1, MOTION_D_REVERSE);
        break;
    case 'F':
        run_all(1, MOTION_ALL_FORWARD);
        break;
    case 'R':
        run_all(-1, MOTION_ALL_REVERSE);
        break;
    case 'S':
    case 's':
        stop_all();
        break;
    case '+':
        set_speed(speed_percent + SPEED_STEP_PERCENT);
        break;
    case '-':
        set_speed(speed_percent - SPEED_STEP_PERCENT);
        break;
    case '0':
        set_speed(100);
        break;
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
        set_speed((command - '0') * 10);
        break;
    case 'H':
    case 'h':
        print_help();
        break;
    case '\r':
    case '\n':
        break;
    default:
        ESP_LOGW(TAG, "未知命令 '%c'，输入 H 查看帮助", command);
        break;
    }
}

void app_main(void)
{
    configure_motor_hardware();
    setvbuf(stdin, NULL, _IONBF, 0);

    ESP_LOGW(TAG, "测试前必须把小车架空");
    ESP_LOGI(TAG, "默认速度 %d%%，PWM频率 %dHz",
             speed_percent,
             PWM_FREQUENCY_HZ);
    print_help();

    while (1) {
        int received = getchar();
        if (received == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        handle_command((char)received);
    }
}
