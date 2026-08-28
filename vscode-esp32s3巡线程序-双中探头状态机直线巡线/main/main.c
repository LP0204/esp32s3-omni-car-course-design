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
#include "tft18_sensor_display.h"

/* ======================== User-adjustable parameters ======================== */
/* Calibrate these two values separately if the two motors have different speed. */
#define LEFT_BASE_POWER_PERCENT 50
#define RIGHT_BASE_POWER_PERCENT 50

/* A center-sensor deviation uses soft correction; an outer sensor uses hard correction. */
#define SOFT_CORRECTION_PERCENT 15
#define HARD_CORRECTION_PERCENT 30

/* 1: line on the left slows the left wheel; set to -1 only if the car corrects oppositely. */
#define STEERING_SIGN 1

/*
 * The filtered all-white result is treated as a line disappearance.  Zero
 * makes the chassis stop on the first valid all-white sample, minimizing
 * overshoot.  Set to 10-20 ms only if the sensors produce occasional
 * all-white glitches and a short forward hold is really needed.
 */
#define ALL_WHITE_STOP_DELAY_MS 0

/* Logical +1 means this wheel contributes to clockwise chassis rotation. */
#define LEFT_ELECTRICAL_SIGN (-1)
#define RIGHT_ELECTRICAL_SIGN 1
#define LEFT_FORWARD_SIGN 1
#define RIGHT_FORWARD_SIGN (-1)

#define MIN_EFFECTIVE_DUTY_PERCENT 35
#define PWM_FREQUENCY_HZ 20000
#define DIRECTION_DEAD_TIME_US 300
#define CONTROL_INTERVAL_MS 10
#define BUTTON_DEBOUNCE_MS 40
#define SENSOR_SAMPLE_COUNT 5
#define SENSOR_SAMPLE_GAP_US 150
#define OBSERVATION_CONFIRM_COUNT 2

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

/* black_mask bit3..bit0 = OUT4..OUT1. */
#define BLACK_OUT4 (1U << 3) /* far left */
#define BLACK_OUT3 (1U << 2) /* left center */
#define BLACK_OUT2 (1U << 1) /* right center */
#define BLACK_OUT1 (1U << 0) /* far right */

typedef struct {
    gpio_num_t in1;
    gpio_num_t in2;
    gpio_num_t pwm_gpio;
    ledc_channel_t pwm_channel;
    int electrical_sign;
    int last_direction;
} motor_t;

typedef enum {
    OBS_CENTERED,
    OBS_LINE_LEFT,
    OBS_LINE_RIGHT,
    OBS_FAR_LEFT,
    OBS_FAR_RIGHT,
    OBS_LOST,
    OBS_AMBIGUOUS,
} line_observation_t;

typedef enum {
    STATE_STOPPED,
    STATE_TRACK_CENTER,
    STATE_CORRECT_LEFT,
    STATE_CORRECT_RIGHT,
    STATE_RECOVER_LEFT,
    STATE_RECOVER_RIGHT,
    STATE_LOST_FORWARD,
    STATE_FAULT_STOP,
} line_state_t;

typedef struct {
    bool initialized;
    line_observation_t candidate;
    line_observation_t stable;
    unsigned candidate_count;
} observation_filter_t;

static const char *TAG = "STATE_LINE";

static motor_t left_motor = {
    .in1 = LEFT_IN1, .in2 = LEFT_IN2, .pwm_gpio = LEFT_PWM,
    .pwm_channel = LEDC_CHANNEL_0,
    .electrical_sign = LEFT_ELECTRICAL_SIGN,
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

static void lock_rear_wheel(void)
{
    ESP_ERROR_CHECK(gpio_set_level(BACK_IN1, 0));
    ESP_ERROR_CHECK(gpio_set_level(BACK_IN2, 0));
    ESP_ERROR_CHECK(gpio_set_level(BACK_PWM, 0));
}

static void stop_chassis(void)
{
    motor_stop(&left_motor);
    motor_stop(&right_motor);
    lock_rear_wheel();
    tft18_sensor_display_set_motor_commands(0, 0, 0);
}

static int usable_power(int percent)
{
    if (percent <= 0) return 0;
    return clamp_int(percent, MIN_EFFECTIVE_DUTY_PERCENT, 100);
}

static void drive_front_wheels(int left_percent, int right_percent)
{
    lock_rear_wheel();
    const int left_command = LEFT_FORWARD_SIGN * usable_power(left_percent);
    const int right_command = RIGHT_FORWARD_SIGN * usable_power(right_percent);
    motor_write(&left_motor, left_command);
    motor_write(&right_motor, right_command);
    tft18_sensor_display_set_motor_commands(left_command, right_command, 0);
}

static void drive_with_correction(int correction_direction, int correction)
{
    correction_direction *= STEERING_SIGN;
    const int left = LEFT_BASE_POWER_PERCENT + correction_direction * correction;
    const int right = RIGHT_BASE_POWER_PERCENT - correction_direction * correction;
    drive_front_wheels(left, right);
}

static void configure_hardware(void)
{
    const gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << LEFT_IN1) | (1ULL << LEFT_IN2) |
                        (1ULL << BACK_IN1) | (1ULL << BACK_IN2) |
                        (1ULL << BACK_PWM) |
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

    motor_t *motors[] = {&left_motor, &right_motor};
    for (int i = 0; i < 2; ++i) {
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

static uint8_t read_black_mask_filtered(void)
{
    unsigned black_votes[4] = {0};
    for (unsigned sample = 0; sample < SENSOR_SAMPLE_COUNT; ++sample) {
        const uint8_t raw = tft18_sensor_display_read_raw();
        for (unsigned bit = 0; bit < 4; ++bit) {
            if (((raw >> bit) & 1U) == 0) {
                ++black_votes[bit];
            }
        }
        if (sample + 1 < SENSOR_SAMPLE_COUNT) {
            esp_rom_delay_us(SENSOR_SAMPLE_GAP_US);
        }
    }

    uint8_t black_mask = 0;
    for (unsigned bit = 0; bit < 4; ++bit) {
        if (black_votes[bit] > SENSOR_SAMPLE_COUNT / 2) {
            black_mask |= 1U << bit;
        }
    }
    return black_mask;
}

static line_observation_t classify_line(uint8_t black_mask)
{
    const bool far_left = (black_mask & BLACK_OUT4) != 0;
    const bool left_center = (black_mask & BLACK_OUT3) != 0;
    const bool right_center = (black_mask & BLACK_OUT2) != 0;
    const bool far_right = (black_mask & BLACK_OUT1) != 0;

    if (black_mask == 0) return OBS_LOST;
    if (far_left && far_right) return OBS_AMBIGUOUS;
    if (left_center && right_center) return OBS_CENTERED;

    /* Crossed patterns cannot be produced by one narrow continuous line. */
    if ((far_left && (right_center || far_right)) ||
        (far_right && (left_center || far_left))) {
        return OBS_AMBIGUOUS;
    }
    if (far_left) return OBS_FAR_LEFT;
    if (far_right) return OBS_FAR_RIGHT;
    if (left_center) return OBS_LINE_LEFT;
    if (right_center) return OBS_LINE_RIGHT;
    return OBS_AMBIGUOUS;
}

static line_observation_t confirm_observation(observation_filter_t *filter,
                                               line_observation_t measured)
{
    if (!filter->initialized) {
        filter->initialized = true;
        filter->candidate = measured;
        filter->stable = measured;
        filter->candidate_count = OBSERVATION_CONFIRM_COUNT;
        return filter->stable;
    }

    if (measured != filter->candidate) {
        filter->candidate = measured;
        filter->candidate_count = 1;
    } else if (filter->candidate_count < OBSERVATION_CONFIRM_COUNT) {
        ++filter->candidate_count;
    }
    if (filter->candidate_count >= OBSERVATION_CONFIRM_COUNT) {
        filter->stable = filter->candidate;
    }
    return filter->stable;
}

static const char *state_name(line_state_t state)
{
    switch (state) {
    case STATE_STOPPED: return "STOPPED";
    case STATE_TRACK_CENTER: return "TRACK_CENTER";
    case STATE_CORRECT_LEFT: return "CORRECT_LEFT";
    case STATE_CORRECT_RIGHT: return "CORRECT_RIGHT";
    case STATE_RECOVER_LEFT: return "RECOVER_LEFT";
    case STATE_RECOVER_RIGHT: return "RECOVER_RIGHT";
    case STATE_LOST_FORWARD: return "LOST_FORWARD";
    case STATE_FAULT_STOP: return "FAULT_STOP";
    default: return "UNKNOWN";
    }
}

static void apply_state(line_state_t state)
{
    switch (state) {
    case STATE_TRACK_CENTER:
        drive_front_wheels(LEFT_BASE_POWER_PERCENT, RIGHT_BASE_POWER_PERCENT);
        break;
    case STATE_CORRECT_LEFT:
        drive_with_correction(-1, SOFT_CORRECTION_PERCENT);
        break;
    case STATE_CORRECT_RIGHT:
        drive_with_correction(1, SOFT_CORRECTION_PERCENT);
        break;
    case STATE_RECOVER_LEFT:
        drive_with_correction(-1, HARD_CORRECTION_PERCENT);
        break;
    case STATE_RECOVER_RIGHT:
        drive_with_correction(1, HARD_CORRECTION_PERCENT);
        break;
    case STATE_LOST_FORWARD:
        drive_front_wheels(LEFT_BASE_POWER_PERCENT, RIGHT_BASE_POWER_PERCENT);
        break;
    case STATE_STOPPED:
    case STATE_FAULT_STOP:
    default:
        stop_chassis();
        break;
    }
}

static void enter_state(line_state_t *state, line_state_t next)
{
    if (*state == next) return;
    *state = next;
    apply_state(next);
    ESP_LOGI(TAG, "state -> %s", state_name(next));
}

void app_main(void)
{
    configure_hardware();
    ESP_ERROR_CHECK(tft18_sensor_display_init());

    bool running = false;
    line_state_t state = STATE_STOPPED;
    observation_filter_t observation_filter = {0};
    int64_t lost_since_ms = -1;
    uint8_t previous_display_raw = 0x10;
    unsigned log_divider = 0;

    ESP_LOGW(TAG, "first test with wheels lifted; BOOT starts/stops tracking");
    ESP_LOGI(TAG, "OUT4/OUT3/OUT2/OUT1 = far-left/left-center/right-center/far-right");
    ESP_LOGI(TAG, "base left=%d%% right=%d%%; rear wheel locked LOW",
             LEFT_BASE_POWER_PERCENT, RIGHT_BASE_POWER_PERCENT);

    while (true) {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        const uint8_t black_mask = read_black_mask_filtered();
        const uint8_t display_raw = (uint8_t)(~black_mask) & 0x0f;
        const line_observation_t measured = classify_line(black_mask);
        /* Do not wait for the normal two-cycle confirmation when the line
         * has disappeared: that extra 20 ms is visible vehicle overshoot. */
        const line_observation_t observed =
            (measured == OBS_LOST) ? OBS_LOST :
            confirm_observation(&observation_filter, measured);

        if (display_raw != previous_display_raw) {
            ESP_ERROR_CHECK(tft18_sensor_display_update(display_raw));
            previous_display_raw = display_raw;
        }

        if (button_pressed_event()) {
            if (running) {
                running = false;
                enter_state(&state, STATE_STOPPED);
                ESP_LOGI(TAG, "manual stop");
            } else if (observed == OBS_LOST || observed == OBS_AMBIGUOUS) {
                enter_state(&state, STATE_FAULT_STOP);
                ESP_LOGW(TAG, "start rejected: place a center sensor on the black line");
            } else {
                running = true;
                lost_since_ms = -1;
                ESP_LOGI(TAG, "tracking started");
            }
        }

        if (running) {
            switch (observed) {
            case OBS_CENTERED:
                lost_since_ms = -1;
                enter_state(&state, STATE_TRACK_CENTER);
                break;

            case OBS_LINE_LEFT:
                lost_since_ms = -1;
                enter_state(&state, STATE_CORRECT_LEFT);
                break;

            case OBS_LINE_RIGHT:
                lost_since_ms = -1;
                enter_state(&state, STATE_CORRECT_RIGHT);
                break;

            case OBS_FAR_LEFT:
                lost_since_ms = -1;
                enter_state(&state, STATE_RECOVER_LEFT);
                break;

            case OBS_FAR_RIGHT:
                lost_since_ms = -1;
                enter_state(&state, STATE_RECOVER_RIGHT);
                break;

            case OBS_LOST:
                if (lost_since_ms < 0) lost_since_ms = now_ms;
                if (now_ms - lost_since_ms >= ALL_WHITE_STOP_DELAY_MS) {
                    running = false;
                    enter_state(&state, STATE_FAULT_STOP);
                    ESP_LOGW(TAG, "all sensors white; line disappeared; stopped immediately");
                } else {
                    /* Optional tolerance, enabled only when the parameter is
                     * changed above zero. */
                    enter_state(&state, STATE_LOST_FORWARD);
                }
                break;

            case OBS_AMBIGUOUS:
            default:
                running = false;
                enter_state(&state, STATE_FAULT_STOP);
                ESP_LOGW(TAG, "ambiguous sensor pattern 0x%X; stopped", black_mask);
                break;
            }
        }

        if (++log_divider >= 50) {
            log_divider = 0;
            ESP_LOGI(TAG, "black_mask=0x%X state=%s running=%d",
                     black_mask, state_name(state), running);
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
    }
}
