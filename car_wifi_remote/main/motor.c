/*
 * 三轮全向底盘电机驱动（与循线小车同一套接线/方向标定）
 * 左前: GPIO8/9/10   后轮: GPIO12/13/14   右前: GPIO15/16/17   STBY: GPIO11
 * 速度范围 0..255（8-bit LEDC），负号表示反向
 */
#include "motor.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#define LF_IN1  GPIO_NUM_8
#define LF_IN2  GPIO_NUM_9
#define LF_PWM  GPIO_NUM_10

#define REAR_IN1 GPIO_NUM_12
#define REAR_IN2 GPIO_NUM_13
#define REAR_PWM GPIO_NUM_14

#define RF_IN1  GPIO_NUM_15
#define RF_IN2  GPIO_NUM_16
#define RF_PWM  GPIO_NUM_17

#define MOTOR_STBY GPIO_NUM_11

/* 方向校准（与 camera_uvc_test 一致） */
#define LF_DIR_SIGN   -1
#define RF_DIR_SIGN   -1
#define REAR_DIR_SIGN  1

#define PWM_FREQ 5000

static int clamp_speed(int s)
{
    if (s > 255) return 255;
    if (s < -255) return -255;
    return s;
}

static void set_wheel(gpio_num_t in1, gpio_num_t in2,
                      ledc_channel_t channel, int speed)
{
    speed = clamp_speed(speed);
    uint32_t duty = (speed > 0) ? (uint32_t)speed
                  : (speed < 0) ? (uint32_t)(-speed) : 0;

    if (speed > 0) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    } else if (speed < 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
    } else {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void motor_set_lf(int speed)
{
    set_wheel(LF_IN1, LF_IN2, LEDC_CHANNEL_0, speed * LF_DIR_SIGN);
}

void motor_set_rf(int speed)
{
    set_wheel(RF_IN1, RF_IN2, LEDC_CHANNEL_1, speed * RF_DIR_SIGN);
}

void motor_set_rear(int speed)
{
    set_wheel(REAR_IN1, REAR_IN2, LEDC_CHANNEL_2, speed * REAR_DIR_SIGN);
}

void motor_stop_all(void)
{
    motor_set_lf(0);
    motor_set_rf(0);
    motor_set_rear(0);
}

void motor_brake_all(void)
{
    const struct { gpio_num_t in1; gpio_num_t in2; ledc_channel_t ch; } w[3] = {
        { LF_IN1, LF_IN2, LEDC_CHANNEL_0 },
        { RF_IN1, RF_IN2, LEDC_CHANNEL_1 },
        { REAR_IN1, REAR_IN2, LEDC_CHANNEL_2 },
    };
    for (int i = 0; i < 3; i++) {
        gpio_set_level(w[i].in1, 1);
        gpio_set_level(w[i].in2, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, w[i].ch, 255);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, w[i].ch);
    }
}

void motor_init(void)
{
    const gpio_num_t motor_pins[] = {
        LF_IN1, LF_IN2, REAR_IN1, REAR_IN2, RF_IN1, RF_IN2, MOTOR_STBY,
    };
    for (size_t i = 0; i < sizeof(motor_pins) / sizeof(motor_pins[0]); i++) {
        gpio_set_direction(motor_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(motor_pins[i], 0);
    }

    const ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    const ledc_channel_config_t chans[] = {
        { .gpio_num = LF_PWM,   .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0,
          .duty = 0, .hpoint = 0 },
        { .gpio_num = RF_PWM,   .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0,
          .duty = 0, .hpoint = 0 },
        { .gpio_num = REAR_PWM, .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = LEDC_CHANNEL_2, .timer_sel = LEDC_TIMER_0,
          .duty = 0, .hpoint = 0 },
    };
    for (size_t i = 0; i < sizeof(chans) / sizeof(chans[0]); i++) {
        ledc_channel_config(&chans[i]);
    }

    gpio_set_level(MOTOR_STBY, 1);
    motor_stop_all();
}
