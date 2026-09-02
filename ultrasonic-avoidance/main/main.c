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

/* ------------------------- Safety and tuning ------------------------- */

/* First test at 0. Set to 1 only after distance and wheel directions pass. */
#define ENABLE_MOTOR_OUTPUT 0

/* Unified convention: logical +1 makes that wheel contribute to clockwise yaw. */
#define RIGHT_ELECTRICAL_SIGN 1
#define LEFT_ELECTRICAL_SIGN (-1)
#define BACK_ELECTRICAL_SIGN (-1)

/* Calibrated straight ratio from the Arduino implementation: about 114:126. */
#define LEFT_FORWARD_POWER_PERCENT 45
#define RIGHT_FORWARD_POWER_PERCENT 49
#define STRAFE_POWER_PERCENT 41
#define MIN_EFFECTIVE_DUTY_PERCENT 35

#define OBSTACLE_TRIGGER_MM 90
#define EMERGENCY_TRIGGER_MM 60
#define OBSTACLE_CONFIRM_COUNT 2
#define SENSOR_FAILURE_LIMIT 3
#define REARM_DISTANCE_MM 150
#define REARM_CONFIRM_COUNT 5

#define ULTRASONIC_SAMPLE_MS 30
#define ULTRASONIC_TIMEOUT_US 4500
#define MIN_VALID_DISTANCE_MM 15
#define MAX_VALID_DISTANCE_MM 700
#define DISTANCE_CLEAR_MM (MAX_VALID_DISTANCE_MM + 1)

#define AVOID_STOP_MS 120
#define STRAFE_OUT_MS 900
#define PASS_FORWARD_MS 1200
#define STRAFE_BACK_MIN_MS 250
#define STRAFE_BACK_MAX_MS 2600
#define REACQUIRE_PAUSE_MS 120
#define LINE_REACQUIRE_CONFIRM_COUNT 3

#define PWM_FREQUENCY_HZ 20000
#define DIRECTION_DEAD_TIME_MS 50
#define BUTTON_DEBOUNCE_MS 40
#define CONTROL_INTERVAL_MS 10
#define LOG_INTERVAL_MS 500

/* ------------------------------ GPIO ------------------------------ */

#define BUTTON_GPIO GPIO_NUM_0

#define IR_OUT4 GPIO_NUM_7
#define IR_OUT3 GPIO_NUM_6
#define IR_OUT2 GPIO_NUM_5
#define IR_OUT1 GPIO_NUM_4
#define IR_CENTER_MASK 0x06

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

/* HC-SR04 ECHO must be divided/translated to 3.3 V before GPIO21. */
#define ULTRASONIC_TRIG GPIO_NUM_18
#define ULTRASONIC_ECHO GPIO_NUM_21

typedef struct {
    gpio_num_t in1;
    gpio_num_t in2;
    gpio_num_t pwm_gpio;
    ledc_channel_t pwm_channel;
    int electrical_sign;
    int last_direction;
} motor_t;

typedef enum {
    STATE_IDLE_FORWARD,
    STATE_STOP,
    STATE_STRAFE_LEFT,
    STATE_FORWARD_PASS,
    STATE_STRAFE_RIGHT_FIND_LINE,
    STATE_REACQUIRE_PAUSE,
    STATE_SENSOR_FAULT,
    STATE_LINE_SEARCH_TIMEOUT,
    STATE_MANUAL_STOP,
} avoid_state_t;

static const char *TAG = "LATERAL_AVOID";

static motor_t left_motor = {
    .in1 = LEFT_IN1, .in2 = LEFT_IN2, .pwm_gpio = LEFT_PWM,
    .pwm_channel = LEDC_CHANNEL_0,
    .electrical_sign = LEFT_ELECTRICAL_SIGN,
};
static motor_t right_motor = {
    .in1 = RIGHT_IN1, .in2 = RIGHT_IN2, .pwm_gpio = RIGHT_PWM,
    .pwm_channel = LEDC_CHANNEL_1,
    .electrical_sign = RIGHT_ELECTRICAL_SIGN,
};
static motor_t back_motor = {
    .in1 = BACK_IN1, .in2 = BACK_IN2, .pwm_gpio = BACK_PWM,
    .pwm_channel = LEDC_CHANNEL_2,
    .electrical_sign = BACK_ELECTRICAL_SIGN,
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

static int command_direction(const motor_t *motor, int command)
{
    if (command == 0) return 0;
    return (command > 0 ? 1 : -1) * motor->electrical_sign;
}

static bool needs_dead_time(const motor_t *motor, int command)
{
    const int next = command_direction(motor, command);
    return next != 0 && motor->last_direction != 0 &&
           next != motor->last_direction;
}

static void write_motor_now(motor_t *motor, int command)
{
    command = clamp_int(command, -100, 100);
    if (!ENABLE_MOTOR_OUTPUT || command == 0) {
        set_pwm(motor, 0);
        ESP_ERROR_CHECK(gpio_set_level(motor->in1, 0));
        ESP_ERROR_CHECK(gpio_set_level(motor->in2, 0));
        motor->last_direction = 0;
        return;
    }

    const int direction = command_direction(motor, command);
    int magnitude = abs(command);
    if (magnitude < MIN_EFFECTIVE_DUTY_PERCENT) {
        magnitude = MIN_EFFECTIVE_DUTY_PERCENT;
    }
    ESP_ERROR_CHECK(gpio_set_level(motor->in1, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2, direction < 0));
    set_pwm(motor, magnitude);
    motor->last_direction = direction;
}

static void drive_commands(int left, int right, int back)
{
    if (ENABLE_MOTOR_OUTPUT &&
        (needs_dead_time(&left_motor, left) ||
         needs_dead_time(&right_motor, right) ||
         needs_dead_time(&back_motor, back))) {
        set_pwm(&left_motor, 0);
        set_pwm(&right_motor, 0);
        set_pwm(&back_motor, 0);
        vTaskDelay(pdMS_TO_TICKS(DIRECTION_DEAD_TIME_MS));
    }
    write_motor_now(&left_motor, left);
    write_motor_now(&right_motor, right);
    write_motor_now(&back_motor, back);
    tft18_sensor_display_set_motor_commands(
        ENABLE_MOTOR_OUTPUT ? left : 0,
        ENABLE_MOTOR_OUTPUT ? right : 0,
        ENABLE_MOTOR_OUTPUT ? back : 0);
}

static void stop_all_motors(void)
{
    drive_commands(0, 0, 0);
}

static void drive_forward(void)
{
    /* Forward logical commands: left positive, right negative, rear stopped. */
    drive_commands(LEFT_FORWARD_POWER_PERCENT,
                   -RIGHT_FORWARD_POWER_PERCENT,
                   0);
}

static void strafe_left(void)
{
    /* With the unified wheel signs, rear logical positive moves the car left. */
    drive_commands(0, 0, STRAFE_POWER_PERCENT);
}

static void strafe_right(void)
{
    drive_commands(0, 0, -STRAFE_POWER_PERCENT);
}

static bool wait_for_level(gpio_num_t gpio, int level, int timeout_us,
                           int64_t *event_time_us)
{
    const int64_t deadline = esp_timer_get_time() + timeout_us;
    while (gpio_get_level(gpio) != level) {
        if (esp_timer_get_time() >= deadline) return false;
    }
    *event_time_us = esp_timer_get_time();
    return true;
}

static int measure_distance_once_mm(void)
{
    /* ECHO already high before triggering indicates wiring/sensor failure. */
    if (gpio_get_level(ULTRASONIC_ECHO) != 0) return -1;

    gpio_set_level(ULTRASONIC_TRIG, 0);
    esp_rom_delay_us(2);
    gpio_set_level(ULTRASONIC_TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(ULTRASONIC_TRIG, 0);

    int64_t start_us;
    int64_t end_us;
    if (!wait_for_level(ULTRASONIC_ECHO, 1, ULTRASONIC_TIMEOUT_US, &start_us)) {
        /* No echo inside the short avoidance range means the path is clear. */
        return DISTANCE_CLEAR_MM;
    }
    if (!wait_for_level(ULTRASONIC_ECHO, 0, ULTRASONIC_TIMEOUT_US, &end_us)) {
        return -1;
    }
    const int distance_mm = (int)(((end_us - start_us) * 10 + 29) / 58);
    if (distance_mm < MIN_VALID_DISTANCE_MM) return -1;
    return distance_mm <= MAX_VALID_DISTANCE_MM ? distance_mm
                                                 : DISTANCE_CLEAR_MM;
}

/* bit3..bit0 = OUT4..OUT1, 1 means black. */
static uint8_t read_ir_black_bits(void)
{
    uint8_t bits = 0;
    if (gpio_get_level(IR_OUT4) == 0) bits |= 0x08;
    if (gpio_get_level(IR_OUT3) == 0) bits |= 0x04;
    if (gpio_get_level(IR_OUT2) == 0) bits |= 0x02;
    if (gpio_get_level(IR_OUT1) == 0) bits |= 0x01;
    return bits;
}

static const char *state_name(avoid_state_t state)
{
    switch (state) {
    case STATE_IDLE_FORWARD: return "IDLE-FORWARD";
    case STATE_STOP: return "STOP";
    case STATE_STRAFE_LEFT: return "STRAFE-LEFT";
    case STATE_FORWARD_PASS: return "FORWARD-PASS";
    case STATE_STRAFE_RIGHT_FIND_LINE: return "STRAFE-RIGHT-FIND-LINE";
    case STATE_REACQUIRE_PAUSE: return "REACQUIRE-PAUSE";
    case STATE_SENSOR_FAULT: return "SENSOR-FAULT";
    case STATE_LINE_SEARCH_TIMEOUT: return "LINE-SEARCH-TIMEOUT";
    case STATE_MANUAL_STOP: return "MANUAL-STOP";
    default: return "UNKNOWN";
    }
}

static void enter_state(avoid_state_t *state, avoid_state_t next,
                        int64_t now_ms, int64_t *state_start_ms)
{
    if (*state == next) return;
    *state = next;
    *state_start_ms = now_ms;
    ESP_LOGI(TAG, "state -> %s", state_name(next));
}

static bool button_pressed_event(void)
{
    static int previous = 1;
    static int64_t last_change_ms;
    const int current = gpio_get_level(BUTTON_GPIO);
    const int64_t now_ms = esp_timer_get_time() / 1000;
    if (current != previous && now_ms - last_change_ms >= BUTTON_DEBOUNCE_MS) {
        previous = current;
        last_change_ms = now_ms;
        return current == 0;
    }
    return false;
}

static void configure_hardware(void)
{
    const gpio_config_t motor_gpio = {
        .pin_bit_mask = (1ULL << LEFT_IN1) | (1ULL << LEFT_IN2) |
                        (1ULL << RIGHT_IN1) | (1ULL << RIGHT_IN2) |
                        (1ULL << BACK_IN1) | (1ULL << BACK_IN2) |
                        (1ULL << MOTOR_STBY),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&motor_gpio));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY, 0));

    const gpio_config_t input_gpio = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO) |
                        (1ULL << IR_OUT4) | (1ULL << IR_OUT3) |
                        (1ULL << IR_OUT2) | (1ULL << IR_OUT1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_gpio));

    const gpio_config_t trigger_gpio = {
        .pin_bit_mask = 1ULL << ULTRASONIC_TRIG,
        .mode = GPIO_MODE_OUTPUT,
    };
    const gpio_config_t echo_gpio = {
        .pin_bit_mask = 1ULL << ULTRASONIC_ECHO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&trigger_gpio));
    ESP_ERROR_CHECK(gpio_config(&echo_gpio));
    ESP_ERROR_CHECK(gpio_set_level(ULTRASONIC_TRIG, 0));

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    motor_t *motors[] = {&left_motor, &right_motor, &back_motor};
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
    stop_all_motors();
    if (ENABLE_MOTOR_OUTPUT) ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY, 1));
}

void app_main(void)
{
    configure_hardware();
    ESP_ERROR_CHECK(tft18_sensor_display_init());

    bool running = false;
    bool obstacle_armed = true;
    avoid_state_t state = STATE_MANUAL_STOP;
    int64_t state_start_ms = esp_timer_get_time() / 1000;
    int64_t last_ultrasonic_ms = -ULTRASONIC_SAMPLE_MS;
    int64_t last_log_ms = -LOG_INTERVAL_MS;
    int last_distance_mm = -1;
    int obstacle_hits = 0;
    int invalid_hits = 0;
    int rearm_hits = 0;
    int line_hits = 0;

    ESP_LOGI(TAG, "HC-SR04 TRIG=GPIO18 ECHO=GPIO21; ECHO must be 3.3V");
    ESP_LOGI(TAG, "BOOT starts/stops; motor output=%s",
             ENABLE_MOTOR_OUTPUT ? "ON" : "SAFE-OFF");
    ESP_LOGI(TAG, "avoid path: stop -> left -> forward -> right until center line");

    while (true) {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        const uint8_t ir_bits = read_ir_black_bits();

        if (button_pressed_event()) {
            if (running) {
                running = false;
                stop_all_motors();
                enter_state(&state, STATE_MANUAL_STOP, now_ms, &state_start_ms);
            } else {
                running = true;
                obstacle_armed = true;
                obstacle_hits = 0;
                invalid_hits = 0;
                rearm_hits = 0;
                line_hits = 0;
                enter_state(&state, STATE_IDLE_FORWARD, now_ms, &state_start_ms);
            }
        }

        bool new_distance = false;
        if (now_ms - last_ultrasonic_ms >= ULTRASONIC_SAMPLE_MS) {
            last_ultrasonic_ms = now_ms;
            last_distance_mm = measure_distance_once_mm();
            tft18_sensor_display_set_distance_mm(last_distance_mm);
            new_distance = true;
        }

        if (running && state == STATE_IDLE_FORWARD && new_distance) {
            if (last_distance_mm < 0) {
                obstacle_hits = 0;
                if (++invalid_hits >= SENSOR_FAILURE_LIMIT) {
                    running = false;
                    stop_all_motors();
                    enter_state(&state, STATE_SENSOR_FAULT, now_ms,
                                &state_start_ms);
                    ESP_LOGW(TAG, "three consecutive invalid echoes; stopped");
                }
            } else {
                invalid_hits = 0;
                if (!obstacle_armed) {
                    if (last_distance_mm >= REARM_DISTANCE_MM) {
                        if (++rearm_hits >= REARM_CONFIRM_COUNT) {
                            obstacle_armed = true;
                            rearm_hits = 0;
                            ESP_LOGI(TAG, "obstacle detector re-armed");
                        }
                    } else {
                        rearm_hits = 0;
                    }
                } else if (last_distance_mm <= EMERGENCY_TRIGGER_MM) {
                    enter_state(&state, STATE_STOP, now_ms, &state_start_ms);
                    obstacle_hits = 0;
                    ESP_LOGW(TAG, "emergency obstacle at %d.%d cm",
                             last_distance_mm / 10, last_distance_mm % 10);
                } else if (last_distance_mm <= OBSTACLE_TRIGGER_MM) {
                    if (++obstacle_hits >= OBSTACLE_CONFIRM_COUNT) {
                        enter_state(&state, STATE_STOP, now_ms, &state_start_ms);
                        obstacle_hits = 0;
                        ESP_LOGW(TAG, "confirmed obstacle at %d.%d cm",
                                 last_distance_mm / 10, last_distance_mm % 10);
                    }
                } else {
                    obstacle_hits = 0;
                }
            }
        }

        if (running) {
            const int64_t elapsed_ms = now_ms - state_start_ms;
            switch (state) {
            case STATE_IDLE_FORWARD:
                drive_forward();
                break;

            case STATE_STOP:
                stop_all_motors();
                if (elapsed_ms >= AVOID_STOP_MS) {
                    enter_state(&state, STATE_STRAFE_LEFT, now_ms,
                                &state_start_ms);
                }
                break;

            case STATE_STRAFE_LEFT:
                strafe_left();
                if (elapsed_ms >= STRAFE_OUT_MS) {
                    enter_state(&state, STATE_FORWARD_PASS, now_ms,
                                &state_start_ms);
                }
                break;

            case STATE_FORWARD_PASS:
                drive_forward();
                if (elapsed_ms >= PASS_FORWARD_MS) {
                    line_hits = 0;
                    enter_state(&state, STATE_STRAFE_RIGHT_FIND_LINE, now_ms,
                                &state_start_ms);
                }
                break;

            case STATE_STRAFE_RIGHT_FIND_LINE:
                strafe_right();
                if (elapsed_ms >= STRAFE_BACK_MIN_MS &&
                    (ir_bits & IR_CENTER_MASK) != 0) {
                    if (++line_hits >= LINE_REACQUIRE_CONFIRM_COUNT) {
                        stop_all_motors();
                        enter_state(&state, STATE_REACQUIRE_PAUSE, now_ms,
                                    &state_start_ms);
                    }
                } else {
                    line_hits = 0;
                }
                if (state == STATE_STRAFE_RIGHT_FIND_LINE &&
                    elapsed_ms >= STRAFE_BACK_MAX_MS) {
                    running = false;
                    stop_all_motors();
                    enter_state(&state, STATE_LINE_SEARCH_TIMEOUT, now_ms,
                                &state_start_ms);
                    ESP_LOGW(TAG, "center sensors did not reacquire line; stopped");
                }
                break;

            case STATE_REACQUIRE_PAUSE:
                stop_all_motors();
                if (elapsed_ms >= REACQUIRE_PAUSE_MS) {
                    obstacle_armed = false;
                    rearm_hits = 0;
                    enter_state(&state, STATE_IDLE_FORWARD, now_ms,
                                &state_start_ms);
                }
                break;

            case STATE_SENSOR_FAULT:
            case STATE_LINE_SEARCH_TIMEOUT:
            case STATE_MANUAL_STOP:
            default:
                stop_all_motors();
                break;
            }
        }

        if (now_ms - last_log_ms >= LOG_INTERVAL_MS) {
            last_log_ms = now_ms;
            if (last_distance_mm >= 0) {
                ESP_LOGI(TAG, "distance=%d.%dcm IR=0x%X state=%s run=%d armed=%d",
                         last_distance_mm / 10, last_distance_mm % 10,
                         ir_bits, state_name(state), running, obstacle_armed);
            } else {
                ESP_LOGI(TAG, "distance=invalid IR=0x%X state=%s run=%d armed=%d",
                         ir_bits, state_name(state), running, obstacle_armed);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
    }
}
