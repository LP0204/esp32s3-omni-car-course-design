/*
 * 云台舵机控制：50Hz 方波，脉宽 500~2500us 对应 0~180°
 * 实测：左右舵机在 GPIO5，上下舵机在 GPIO4（LEDC timer1/ch3,ch4，14-bit）
 */
#include "servo.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#define PAN_GPIO   GPIO_NUM_5
#define TILT_GPIO  GPIO_NUM_4

#define SERVO_FREQ_HZ    50
#define SERVO_PERIOD_US  20000
#define PULSE_MIN_US     500
#define PULSE_MAX_US     2500

static int s_pan_deg = 90;
static int s_tilt_deg = 90;

static uint32_t deg_to_duty(int deg)
{
    if (deg < 0) deg = 0;
    if (deg > 180) deg = 180;
    uint32_t pulse_us = PULSE_MIN_US +
        (uint32_t)((PULSE_MAX_US - PULSE_MIN_US) * deg / 180);
    return (uint32_t)((uint64_t)pulse_us * 16383 / SERVO_PERIOD_US);
}

static void write_pulse(ledc_channel_t ch, int deg)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, deg_to_duty(deg));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

void servo_set_pan_deg(int deg)
{
    if (deg < 0) deg = 0;
    if (deg > 180) deg = 180;
    s_pan_deg = deg;
    write_pulse(LEDC_CHANNEL_3, deg);
}

void servo_set_tilt_deg(int deg)
{
    if (deg < 0) deg = 0;
    if (deg > 180) deg = 180;
    s_tilt_deg = deg;
    write_pulse(LEDC_CHANNEL_4, deg);
}

int servo_pan_deg(void)  { return s_pan_deg; }
int servo_tilt_deg(void) { return s_tilt_deg; }

void servo_init(void)
{
    gpio_set_direction(PAN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(TILT_GPIO, GPIO_MODE_OUTPUT);

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    const ledc_channel_config_t chans[] = {
        { .gpio_num = PAN_GPIO,   .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = LEDC_CHANNEL_3, .timer_sel = LEDC_TIMER_1,
          .duty = 0, .hpoint = 0 },
        { .gpio_num = TILT_GPIO,  .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = LEDC_CHANNEL_4, .timer_sel = LEDC_TIMER_1,
          .duty = 0, .hpoint = 0 },
    };
    for (size_t i = 0; i < sizeof(chans) / sizeof(chans[0]); i++) {
        ledc_channel_config(&chans[i]);
    }

    servo_set_pan_deg(90);
    servo_set_tilt_deg(90);
}
