/*
 * ESP32-S3 camera four-channel black/white display test.
 *
 * UVC frames are converted into the calibrated four-bit black/white mask.
 * This test firmware only captures camera frames, calculates the calibrated
 * four-bit mask and displays it on TFT18.  It never initializes or drives
 * the motors, HC-SR04 or BOOT control.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#include "tft18_status_display.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

/* ======================== User-adjustable parameters ===================== */

/* Straight-line motor calibration.  Tune these separately if the two front
 * motors have different mechanical speeds. */
#define TRACK_BASE_LEFT_PERCENT       36  /* zip: 90/255=35.3% */
#define TRACK_BASE_RIGHT_PERCENT      40  /* zip: 100/255=39.2% */
#define TRACK_MAX_PERCENT             68
#define TRACK_MIN_MOVING_PERCENT      25

/* Controller gains for normalized image error (-1.0 .. +1.0). */
#define TRACK_KP_PERCENT              32.0f
#define TRACK_KD_PERCENT              10.0f
#define TRACK_HEADING_GAIN_PERCENT    24.0f
#define HARD_TURN_ERROR               0.42f
#define HARD_TURN_POWER_PERCENT       42
#define CORNER_HEADING_THRESHOLD      0.16f
#define CORNER_TURN_POWER_PERCENT     38

/* A one-frame miss is ignored; a persistent miss starts an active search. */
#define SHORT_LOSS_GRACE_MS           100
#define LOST_SETTLE_MS                120
#define SEARCH_SWITCH_MS              900
#define SEARCH_TIMEOUT_MS             4400
#define SEARCH_POWER_PERCENT          35

/* HC-SR04 avoidance, based on the working vehicle program but with two-sample
 * confirmation and a separate emergency threshold. */
#define OBSTACLE_TRIGGER_MM            90
#define OBSTACLE_EMERGENCY_MM          60
#define OBSTACLE_CONFIRM_COUNT          2
#define OBSTACLE_REARM_MM             150
#define OBSTACLE_REARM_COUNT            3
#define AVOID_STOP_MS                 120
#define AVOID_CLEAR_MM                200
#define AVOID_CLEAR_CONFIRM_MS        150
#define AVOID_STRAFE_OUT_MAX_MS      1800
#define AVOID_FORWARD_MS             1400
#define AVOID_STRAFE_BACK_MIN_MS      250
#define AVOID_STRAFE_BACK_MAX_MS     2600
#define AVOID_REACQUIRE_PAUSE_MS      120
#define STRAFE_FRONT_LEFT_PERCENT      28 /* zip: 70/255 */
#define STRAFE_FRONT_RIGHT_PERCENT     30 /* zip: 77/255 */
#define STRAFE_REAR_PERCENT            55 /* zip: 140/255 */
#define AVOID_FORWARD_LEFT_PERCENT     43 /* zip: 110/255 */
#define AVOID_FORWARD_RIGHT_PERCENT    51 /* zip: 130/255 */

/* Camera processing resolution and bands, measured from image top.
 * These ranges follow camera_uvc_test_901_bien(1).zip exactly:
 * near = bottom 8 rows; far = the 24 rows immediately above near. */
#define VISION_WIDTH                  160
#define VISION_HEIGHT                 120
#define FAR_BAND_Y0                    88
#define FAR_BAND_Y1                   111
#define NEAR_BAND_Y0                  112
#define NEAR_BAND_Y1                  119
#define TRACK_TARGET_X                80.0f

/* Camera x runs opposite to the physical 1->4 sensor direction.  The ranges
 * below come from the user's three physical line-placement tests. */
#define VIRTUAL_SENSOR_Y0             NEAR_BAND_Y0
#define VIRTUAL_SENSOR_Y1             NEAR_BAND_Y1
#define REFERENCE_NEAR_THRESHOLD       110
#define REFERENCE_MIN_BLACK_PERCENT     10
#define LINE_RUN_MIN_WIDTH_PX            2
#define NEAR_BAND_MIN_VALID_ROWS          3
#define FAR_BAND_MIN_VALID_ROWS           4
#define THRESHOLD_MIN                 35
#define THRESHOLD_MAX                 190
#define THRESHOLD_BIAS                 8
#define FRAME_PROCESS_MIN_GAP_US      (40U * 1000U) /* up to about 25 fps */

/* Set to 0 if the physical camera image is already upright. */
#define CAMERA_ROTATE_180             1

/* ============================== Hardware ================================ */

#define USB_D_MINUS_GPIO              19
#define USB_D_PLUS_GPIO               20
#define CAMERA_MODEL_NAME             "JQ-CAM12-720D-V1"

#define BUTTON_GPIO                   GPIO_NUM_0
#define ULTRASONIC_TRIG               GPIO_NUM_18
#define ULTRASONIC_ECHO               GPIO_NUM_21
#define LEFT_IN1                      GPIO_NUM_8
#define LEFT_IN2                      GPIO_NUM_9
#define LEFT_PWM                      GPIO_NUM_10
#define MOTOR_STBY                    GPIO_NUM_11
#define BACK_IN1                      GPIO_NUM_12
#define BACK_IN2                      GPIO_NUM_13
#define BACK_PWM                      GPIO_NUM_14
#define RIGHT_IN1                     GPIO_NUM_15
#define RIGHT_IN2                     GPIO_NUM_16
#define RIGHT_PWM                     GPIO_NUM_17

/* Tested motor convention: logical +1 on all three wheels rotates clockwise. */
#define LEFT_ELECTRICAL_SIGN          (-1)
#define BACK_ELECTRICAL_SIGN          (-1)
#define RIGHT_ELECTRICAL_SIGN         1
#define LEFT_FORWARD_SIGN             1
#define RIGHT_FORWARD_SIGN            (-1)

#define PWM_FREQUENCY_HZ               5000
#define DIRECTION_DEAD_TIME_US         300
#define BUTTON_DEBOUNCE_MS            40
#define CONTROL_INTERVAL_MS           10
#define ULTRASONIC_SAMPLE_MS           30
#define ULTRASONIC_TIMEOUT_US          4000
#define MIN_VALID_DISTANCE_MM          20
#define MAX_VALID_DISTANCE_MM          4000
#define ULTRASONIC_DISPLAY_HOLD_MS     500

/* Optional low-bandwidth computer preview. */
#define PREVIEW_WIDTH                 80
#define PREVIEW_HEIGHT                60
#define PREVIEW_PIXELS                (PREVIEW_WIDTH * PREVIEW_HEIGHT)
#define PREVIEW_FRAME_BYTES           (4U + 2U + 2U + 2U + PREVIEW_PIXELS + 2U)
#define STREAM_BAUD                   921600

#define MAX_JPEG_BYTES                (512U * 1024U)
#define DECODE_MAX_W                  160
#define DECODE_MAX_H                  120
#define DECODE_BUFFER_BYTES           (DECODE_MAX_W * DECODE_MAX_H * 2U)
#define JPEG_WORK_BYTES               4096
#define UVC_FRAME_BUFFERS              3
#define UVC_URB_COUNT                  4
#define UVC_URB_SIZE                  (8U * 1024U)
#define MAX_PROFILES                   8

typedef struct {
    gpio_num_t in1;
    gpio_num_t in2;
    gpio_num_t pwm_gpio;
    ledc_channel_t pwm_channel;
    int electrical_sign;
    int last_direction;
} motor_t;

typedef enum {
    STATE_STOPPED = 0,
    STATE_SEARCH = 1,
    STATE_TRACK = 2,
    STATE_HARD_LEFT = 3,
    STATE_HARD_RIGHT = 4,
    STATE_LOST_LEFT = 5,
    STATE_LOST_RIGHT = 6,
    STATE_FAULT_STOP = 7,
} follower_state_t;

typedef enum {
    AVOID_IDLE,
    AVOID_STOP,
    AVOID_STRAFE_LEFT,
    AVOID_FORWARD_PASS,
    AVOID_STRAFE_RIGHT,
    AVOID_REACQUIRE_PAUSE,
} avoidance_state_t;

typedef struct {
    bool valid;
    float center;
    int valid_rows;
} band_result_t;

typedef struct {
    bool valid;
    bool near_valid;
    bool far_valid;
    float near_center;
    float far_center;
    float error;
    float heading;
    int corner_direction; /* -1 left, +1 right, 0 normal */
    int threshold;
    int black_percent;
    uint8_t virtual_black_mask;
    uint16_t virtual_black_count[4];
    uint32_t sequence;
    int64_t timestamp_ms;
} vision_result_t;

static const char *TAG = "CAMERA_4CH_TEST";

static motor_t s_left_motor = {
    .in1 = LEFT_IN1, .in2 = LEFT_IN2, .pwm_gpio = LEFT_PWM,
    .pwm_channel = LEDC_CHANNEL_0, .electrical_sign = LEFT_ELECTRICAL_SIGN,
};
static motor_t s_back_motor = {
    .in1 = BACK_IN1, .in2 = BACK_IN2, .pwm_gpio = BACK_PWM,
    .pwm_channel = LEDC_CHANNEL_1, .electrical_sign = BACK_ELECTRICAL_SIGN,
};
static motor_t s_right_motor = {
    .in1 = RIGHT_IN1, .in2 = RIGHT_IN2, .pwm_gpio = RIGHT_PWM,
    .pwm_channel = LEDC_CHANNEL_2, .electrical_sign = RIGHT_ELECTRICAL_SIGN,
};

static SemaphoreHandle_t s_frame_ready;
static SemaphoreHandle_t s_frame_mutex;
static SemaphoreHandle_t s_vision_mutex;
static uint8_t *s_jpeg_buffer;
static uint8_t *s_decode_buffer;
static uint8_t s_jpeg_work[JPEG_WORK_BYTES];
static volatile size_t s_jpeg_size;
static volatile uint32_t s_input_width;
static volatile uint32_t s_input_height;
static volatile uint32_t s_camera_frame_count;

static uint8_t s_gray[VISION_WIDTH * VISION_HEIGHT];
static uint8_t s_preview[PREVIEW_PIXELS];
static uint8_t s_serial_frame[PREVIEW_FRAME_BYTES];
static vision_result_t s_latest_vision;
static volatile bool s_stream_enabled;
static volatile int s_ultrasonic_raw_mm = -1;
static volatile int s_ultrasonic_display_mm = -1;
static volatile uint32_t s_ultrasonic_sequence;

static uint8_t s_device_addr;
static uint8_t s_stream_index;
static volatile bool s_device_connected;
static volatile bool s_stream_task_started;
static int s_profile_count;
static uvc_host_stream_format_t s_profiles[MAX_PROFILES];

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float absolute_float(float value)
{
    return value < 0.0f ? -value : value;
}

/* =============================== Motors ================================= */

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
    if (magnitude < TRACK_MIN_MOVING_PERCENT) magnitude = TRACK_MIN_MOVING_PERCENT;

    if (motor->last_direction != 0 && motor->last_direction != direction) {
        set_pwm(motor, 0);
        esp_rom_delay_us(DIRECTION_DEAD_TIME_US);
    }
    ESP_ERROR_CHECK(gpio_set_level(motor->in1, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2, direction < 0));
    set_pwm(motor, magnitude);
    motor->last_direction = direction;
}

static int usable_front_power(int percent)
{
    percent = clamp_int(percent, 0, TRACK_MAX_PERCENT);
    if (percent == 0) return 0;
    if (percent < TRACK_MIN_MOVING_PERCENT) return TRACK_MIN_MOVING_PERCENT;
    return percent;
}

static void stop_chassis(void)
{
    motor_stop(&s_left_motor);
    motor_stop(&s_back_motor);
    motor_stop(&s_right_motor);
    tft18_status_display_set_motor_commands(0, 0, 0);
}

static void drive_front_wheels(int left_percent, int right_percent)
{
    const int left_command = LEFT_FORWARD_SIGN * usable_front_power(left_percent);
    const int right_command = RIGHT_FORWARD_SIGN * usable_front_power(right_percent);
    motor_stop(&s_back_motor);
    motor_write(&s_left_motor, left_command);
    motor_write(&s_right_motor, right_command);
    tft18_status_display_set_motor_commands(left_command, right_command, 0);
}

static void rotate_chassis(int signed_power)
{
    signed_power = clamp_int(signed_power, -100, 100);
    motor_write(&s_left_motor, signed_power);
    motor_write(&s_back_motor, signed_power);
    motor_write(&s_right_motor, signed_power);
    tft18_status_display_set_motor_commands(signed_power, signed_power,
                                             signed_power);
}

static void drive_all_commands(int left, int right, int back)
{
    left = clamp_int(left, -100, 100);
    right = clamp_int(right, -100, 100);
    back = clamp_int(back, -100, 100);
    motor_write(&s_left_motor, left);
    motor_write(&s_right_motor, right);
    motor_write(&s_back_motor, back);
    tft18_status_display_set_motor_commands(left, right, back);
}

/* The Arduino reference expresses every wheel command in signed 8-bit PWM.
 * Keep that convention here so its lookup table, lost-line turn and obstacle
 * motions can be copied without silently changing the motor ratios. */
static void motor_write_reference_raw(motor_t *motor, int raw_command)
{
    raw_command = clamp_int(raw_command, -255, 255);
    if (raw_command == 0) {
        motor_stop(motor);
        return;
    }

    int direction = raw_command > 0 ? 1 : -1;
    direction *= motor->electrical_sign;
    const int magnitude = abs(raw_command);
    if (motor->last_direction != 0 && motor->last_direction != direction) {
        set_pwm(motor, 0);
        esp_rom_delay_us(DIRECTION_DEAD_TIME_US);
    }
    ESP_ERROR_CHECK(gpio_set_level(motor->in1, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2, direction < 0));
    const uint32_t duty = ((1U << LEDC_TIMER_10_BIT) - 1U) *
                          (uint32_t)magnitude / 255U;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->pwm_channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->pwm_channel));
    motor->last_direction = direction;
}

static int raw255_to_percent(int raw)
{
    const int magnitude = abs(clamp_int(raw, -255, 255));
    const int percent = (magnitude * 100 + 127) / 255;
    return raw < 0 ? -percent : percent;
}

/* Convert the reference program's logical wheel signs to this ESP-IDF
 * project's tested electrical convention. */
static void drive_reference_raw(int left_front, int right_front, int rear)
{
    motor_write_reference_raw(&s_left_motor, left_front);
    motor_write_reference_raw(&s_right_motor, -right_front);
    motor_write_reference_raw(&s_back_motor, -rear);
    tft18_status_display_set_motor_commands(raw255_to_percent(left_front),
                                             raw255_to_percent(right_front),
                                             raw255_to_percent(rear));
}

static void strafe_left(void)
{
    /* Converted from the working 70:77:140 raw-speed calibration. */
    drive_all_commands(-STRAFE_FRONT_LEFT_PERCENT,
                       -STRAFE_FRONT_RIGHT_PERCENT,
                       STRAFE_REAR_PERCENT);
}

static void strafe_right(void)
{
    drive_all_commands(STRAFE_FRONT_LEFT_PERCENT,
                       STRAFE_FRONT_RIGHT_PERCENT,
                       -STRAFE_REAR_PERCENT);
}

static void configure_motor_hardware(void)
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

    const gpio_config_t ultrasonic_trigger = {
        .pin_bit_mask = 1ULL << ULTRASONIC_TRIG,
        .mode = GPIO_MODE_OUTPUT,
    };
    const gpio_config_t ultrasonic_echo = {
        .pin_bit_mask = 1ULL << ULTRASONIC_ECHO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ultrasonic_trigger));
    ESP_ERROR_CHECK(gpio_config(&ultrasonic_echo));
    ESP_ERROR_CHECK(gpio_set_level(ULTRASONIC_TRIG, 0));

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    motor_t *motors[] = {&s_left_motor, &s_back_motor, &s_right_motor};
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
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
    static int stable_level = 1;
    static int sampled_level = 1;
    static int64_t changed_at_ms;
    const int current = gpio_get_level(BUTTON_GPIO);
    const int64_t now_ms = esp_timer_get_time() / 1000;

    if (current != sampled_level) {
        sampled_level = current;
        changed_at_ms = now_ms;
    }
    if (sampled_level != stable_level &&
        now_ms - changed_at_ms >= BUTTON_DEBOUNCE_MS) {
        stable_level = sampled_level;
        return stable_level == 0;
    }
    return false;
}

/* ============================= HC-SR04 ================================== */

static void configure_ultrasonic_hardware(void)
{
    const gpio_config_t trigger = {
        .pin_bit_mask = 1ULL << ULTRASONIC_TRIG,
        .mode = GPIO_MODE_OUTPUT,
    };
    const gpio_config_t echo = {
        .pin_bit_mask = 1ULL << ULTRASONIC_ECHO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&trigger));
    ESP_ERROR_CHECK(gpio_config(&echo));
    ESP_ERROR_CHECK(gpio_set_level(ULTRASONIC_TRIG, 0));
    ESP_LOGI(TAG, "HC-SR04 enabled: TRIG=GPIO18 ECHO=GPIO21");
}

static bool wait_for_gpio_level(gpio_num_t gpio, int level, int timeout_us,
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
    if (gpio_get_level(ULTRASONIC_ECHO) != 0) return -1;
    gpio_set_level(ULTRASONIC_TRIG, 0);
    esp_rom_delay_us(2);
    gpio_set_level(ULTRASONIC_TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(ULTRASONIC_TRIG, 0);

    int64_t echo_start_us;
    int64_t echo_end_us;
    if (!wait_for_gpio_level(ULTRASONIC_ECHO, 1, ULTRASONIC_TIMEOUT_US,
                             &echo_start_us) ||
        !wait_for_gpio_level(ULTRASONIC_ECHO, 0, ULTRASONIC_TIMEOUT_US,
                             &echo_end_us)) {
        return -1;
    }
    const int distance_mm = (int)(((echo_end_us - echo_start_us) * 10 + 29) / 58);
    if (distance_mm < MIN_VALID_DISTANCE_MM ||
        distance_mm > MAX_VALID_DISTANCE_MM) return -1;
    return distance_mm;
}

static int median_three(int a, int b, int c)
{
    if (a > b) { const int temporary = a; a = b; b = temporary; }
    if (b > c) { const int temporary = b; b = c; c = temporary; }
    if (a > b) { const int temporary = a; a = b; b = temporary; }
    return b;
}

static void ultrasonic_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    int history[3] = {-1, -1, -1};
    int history_count = 0;
    int history_index = 0;
    int64_t last_valid_ms = 0;

    while (true) {
        const int distance_mm = measure_distance_once_mm();
        const int64_t now_ms = esp_timer_get_time() / 1000;
        s_ultrasonic_raw_mm = distance_mm;
        ++s_ultrasonic_sequence;

        if (distance_mm >= 0) {
            last_valid_ms = now_ms;
            history[history_index] = distance_mm;
            history_index = (history_index + 1) % 3;
            if (history_count < 3) ++history_count;
            if (history_count == 1) {
                s_ultrasonic_display_mm = history[0];
            } else if (history_count == 2) {
                s_ultrasonic_display_mm = (history[0] + history[1]) / 2;
            } else {
                s_ultrasonic_display_mm = median_three(history[0], history[1], history[2]);
            }
        } else if (now_ms - last_valid_ms > ULTRASONIC_DISPLAY_HOLD_MS) {
            s_ultrasonic_display_mm = -1;
        }
        tft18_status_display_set_distance_mm(s_ultrasonic_display_mm);
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ULTRASONIC_SAMPLE_MS));
    }
}

/* =============================== Vision ================================= */

static uint8_t rgb565_to_gray(const uint8_t *pixel)
{
    const uint16_t value = (uint16_t)(pixel[0] | ((uint16_t)pixel[1] << 8));
    const uint32_t r = ((value >> 11) & 0x1fU) * 255U / 31U;
    const uint32_t g = ((value >> 5) & 0x3fU) * 255U / 63U;
    const uint32_t b = (value & 0x1fU) * 255U / 31U;
    return (uint8_t)((299U * r + 587U * g + 114U * b) / 1000U);
}

static void make_control_gray(const uint8_t *rgb565, size_t stride,
                              uint32_t width, uint32_t height)
{
    for (int y = 0; y < VISION_HEIGHT; ++y) {
        uint32_t source_y = (uint32_t)y * height / VISION_HEIGHT;
        for (int x = 0; x < VISION_WIDTH; ++x) {
            uint32_t source_x = (uint32_t)x * width / VISION_WIDTH;
#if CAMERA_ROTATE_180
            source_x = width - 1U - source_x;
            source_y = height - 1U - ((uint32_t)y * height / VISION_HEIGHT);
#endif
            const uint8_t *pixel = rgb565 + (size_t)source_y * stride + source_x * 2U;
            s_gray[y * VISION_WIDTH + x] = rgb565_to_gray(pixel);
        }
    }
}

static int otsu_threshold(void)
{
    uint32_t histogram[256] = {0};
    uint32_t total = 0;
    uint64_t gray_sum = 0;
    for (int y = 36; y < VISION_HEIGHT; ++y) {
        for (int x = 4; x < VISION_WIDTH - 4; ++x) {
            const uint8_t gray = s_gray[y * VISION_WIDTH + x];
            ++histogram[gray];
            ++total;
            gray_sum += gray;
        }
    }

    uint32_t background_count = 0;
    uint64_t background_sum = 0;
    uint64_t best_variance = 0;
    int best = 100;
    for (int threshold = 0; threshold < 256; ++threshold) {
        background_count += histogram[threshold];
        if (background_count == 0) continue;
        const uint32_t foreground_count = total - background_count;
        if (foreground_count == 0) break;
        background_sum += (uint64_t)threshold * histogram[threshold];
        const int64_t difference =
            (int64_t)background_sum * total - (int64_t)gray_sum * background_count;
        const uint64_t magnitude = (uint64_t)(difference < 0 ? -difference : difference);
        const uint64_t variance =
            (magnitude / background_count) * (magnitude / foreground_count);
        if (variance > best_variance) {
            best_variance = variance;
            best = threshold;
        }
    }
    return clamp_int(best, THRESHOLD_MIN, THRESHOLD_MAX);
}

static bool is_black_smoothed(int x, int y, int threshold)
{
    const int row = y * VISION_WIDTH;
    const int x0 = x > 0 ? x - 1 : x;
    const int x2 = x + 1 < VISION_WIDTH ? x + 1 : x;
    const int average = (s_gray[row + x0] + s_gray[row + x] +
                         s_gray[row + x2]) / 3;
    return average < threshold;
}

static band_result_t find_band(int y0, int y1, int threshold, float expected,
                               int minimum_valid_rows)
{
    float center_sum = 0.0f;
    int accepted_rows = 0;
    for (int y = y0; y <= y1; y += 2) {
        int best_start = -1;
        int best_end = -1;
        float best_score = 10000.0f;
        int run_start = -1;

        for (int x = 2; x <= VISION_WIDTH - 2; ++x) {
            const bool black = x < VISION_WIDTH - 2 &&
                               is_black_smoothed(x, y, threshold);
            if (black && run_start < 0) run_start = x;
            if ((!black || x == VISION_WIDTH - 2) && run_start >= 0) {
                const int run_end = black ? x : x - 1;
                const int width = run_end - run_start + 1;
                if (width >= LINE_RUN_MIN_WIDTH_PX && width <= 72) {
                    const float center = (run_start + run_end) * 0.5f;
                    const float score = absolute_float(center - expected) - width * 0.12f;
                    if (score < best_score) {
                        best_score = score;
                        best_start = run_start;
                        best_end = run_end;
                    }
                }
                run_start = -1;
            }
        }
        if (best_start >= 0) {
            center_sum += (best_start + best_end) * 0.5f;
            ++accepted_rows;
        }
    }

    band_result_t result = {
        .valid = accepted_rows >= minimum_valid_rows,
        .center = accepted_rows == 0 ? expected : center_sum / accepted_rows,
        .valid_rows = accepted_rows,
    };
    return result;
}

static uint8_t make_reference_black_mask(uint16_t black_count[4])
{
    /* CH4-on-line trace: near=65.8..73.5 while the old CH4 [40,60)
     * counted zero.  Move CH4 inward and place the CH3/CH4 boundary at 70. */
    static const int x0_by_channel[4] = {90, 80, 70, 50};
    static const int x1_by_channel[4] = {100, 90, 80, 70};
    uint8_t mask = 0;

    /* Use the archive's raw grayscale, fixed threshold 110 and bottom eight
     * rows, but require 10% occupancy.  The real centred-car trace contained
     * 6 isolated dark pixels in CH1, proving the archive's one-pixel rule
     * produces false positives on this camera. */
    for (int zone = 0; zone < 4; ++zone) {
        const int x0 = x0_by_channel[zone];
        const int x1 = x1_by_channel[zone];
        uint16_t black = 0;
        for (int y = VIRTUAL_SENSOR_Y0; y <= VIRTUAL_SENSOR_Y1; ++y) {
            for (int x = x0; x < x1; ++x) {
                if (s_gray[y * VISION_WIDTH + x] < REFERENCE_NEAR_THRESHOLD) {
                    ++black;
                }
            }
        }
        black_count[zone] = black;
        const int total = (x1 - x0) * (VIRTUAL_SENSOR_Y1 - VIRTUAL_SENSOR_Y0 + 1);
        if ((int)black * 100 >= total * REFERENCE_MIN_BLACK_PERCENT) {
            mask |= (uint8_t)(1U << (3 - zone));
        }
    }
    return mask;
}

static vision_result_t analyze_gray_image(void)
{
    static float expected_center = TRACK_TARGET_X;
    static uint32_t sequence;
    static int filtered_threshold = 100;
    static float last_far_center = TRACK_TARGET_X;
    static int64_t last_far_valid_ms;
    static int far_missing_frames;
    /* The un_poco_bien reference improved its fixed threshold from 100 to
     * 110.  Preserve that useful tendency as a small bias on top of Otsu,
     * instead of giving up adaptive illumination handling for a fixed 110. */
    const int raw_threshold = clamp_int(otsu_threshold() + THRESHOLD_BIAS,
                                        THRESHOLD_MIN, THRESHOLD_MAX);
    filtered_threshold = (3 * filtered_threshold + raw_threshold + 2) / 4;
    const int threshold = filtered_threshold;
    const band_result_t near = find_band(NEAR_BAND_Y0, NEAR_BAND_Y1,
                                         threshold, expected_center,
                                         NEAR_BAND_MIN_VALID_ROWS);
    const band_result_t far = find_band(FAR_BAND_Y0, FAR_BAND_Y1, threshold,
                                        near.valid ? near.center : expected_center,
                                        FAR_BAND_MIN_VALID_ROWS);
    const int64_t now_ms = esp_timer_get_time() / 1000;

    int corner_direction = 0;
    if (far.valid) {
        far_missing_frames = 0;
        last_far_center = far.center;
        last_far_valid_ms = now_ms;
        if (near.valid) {
            /* Image x is reversed relative to physical left/right. */
            const float delta = near.center - far.center;
            if (absolute_float(delta) >=
                CORNER_HEADING_THRESHOLD * VISION_WIDTH * 0.5f) {
                corner_direction = delta < 0.0f ? -1 : 1;
            }
        }
    } else {
        ++far_missing_frames;
        /* A sharp corner can make the far band vanish abruptly. Require two
         * consecutive misses and a meaningful last-far/near separation so a
         * single shadow does not force a turn. */
        if (near.valid && far_missing_frames >= 2 && far_missing_frames <= 5 &&
            last_far_valid_ms != 0 && now_ms - last_far_valid_ms <= 450) {
            const float delta = near.center - last_far_center;
            if (absolute_float(delta) >= 8.0f) {
                corner_direction = delta < 0.0f ? -1 : 1;
            }
        }
    }

    int black_count = 0;
    int sample_count = 0;
    for (int y = 36; y < VISION_HEIGHT; y += 2) {
        for (int x = 2; x < VISION_WIDTH - 2; x += 2) {
            black_count += is_black_smoothed(x, y, threshold) ? 1 : 0;
            ++sample_count;
        }
    }

    uint16_t reference_counts[4] = {0};
    const uint8_t reference_mask = make_reference_black_mask(reference_counts);
    vision_result_t result = {
        .valid = near.valid || far.valid,
        .near_valid = near.valid,
        .far_valid = far.valid,
        .near_center = near.center,
        .far_center = far.center,
        .corner_direction = corner_direction,
        .threshold = threshold,
        .black_percent = sample_count == 0 ? 0 : black_count * 100 / sample_count,
        .virtual_black_mask = reference_mask,
        .sequence = ++sequence,
        .timestamp_ms = now_ms,
    };
    for (int zone = 0; zone < 4; ++zone) {
        result.virtual_black_count[zone] = reference_counts[zone];
    }

    /* Physical right corresponds to decreasing image x. */
    const float image_center = TRACK_TARGET_X;
    const float control_center = near.valid ? near.center : far.center;
    result.error = (image_center - control_center) / (VISION_WIDTH * 0.5f);
    result.heading = (near.valid && far.valid)
                         ? (near.center - far.center) / (VISION_WIDTH * 0.5f)
                         : 0.0f;
    if (result.valid) expected_center = control_center;
    return result;
}

static void publish_vision(const vision_result_t *result)
{
    static int64_t last_log_ms;
    if (xSemaphoreTake(s_vision_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_latest_vision = *result;
        xSemaphoreGive(s_vision_mutex);
    }
    tft18_status_display_set_virtual_black_mask(result->virtual_black_mask);
    if (result->timestamp_ms - last_log_ms >= 500) {
        last_log_ms = result->timestamp_ms;
        const uint8_t bits = result->virtual_black_mask;
        ESP_LOGI(TAG,
                 "CH1..CH4=%c%c%c%c counts=%u/%u/%u/%u threshold=%d seq=%lu",
                 (bits & 8U) ? 'B' : 'W', (bits & 4U) ? 'B' : 'W',
                 (bits & 2U) ? 'B' : 'W', (bits & 1U) ? 'B' : 'W',
                 result->virtual_black_count[0], result->virtual_black_count[1],
                 result->virtual_black_count[2], result->virtual_black_count[3],
                 REFERENCE_NEAR_THRESHOLD, (unsigned long)result->sequence);
    }
}

static bool read_latest_vision(vision_result_t *result)
{
    if (xSemaphoreTake(s_vision_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;
    *result = s_latest_vision;
    xSemaphoreGive(s_vision_mutex);
    return result->sequence != 0;
}

/* =========================== Control state machine ======================= */

static const char *state_name(follower_state_t state)
{
    switch (state) {
    case STATE_STOPPED: return "STOPPED";
    case STATE_SEARCH: return "SEARCH";
    case STATE_TRACK: return "TRACK";
    case STATE_HARD_LEFT: return "HARD_LEFT";
    case STATE_HARD_RIGHT: return "HARD_RIGHT";
    case STATE_LOST_LEFT: return "LOST_LEFT";
    case STATE_LOST_RIGHT: return "LOST_RIGHT";
    case STATE_FAULT_STOP: return "FAULT_STOP";
    default: return "UNKNOWN";
    }
}

static const char *avoidance_name(avoidance_state_t state)
{
    switch (state) {
    case AVOID_IDLE: return "IDLE";
    case AVOID_STOP: return "STOP";
    case AVOID_STRAFE_LEFT: return "STRAFE_LEFT";
    case AVOID_FORWARD_PASS: return "FORWARD_PASS";
    case AVOID_STRAFE_RIGHT: return "STRAFE_RIGHT";
    case AVOID_REACQUIRE_PAUSE: return "REACQUIRE_PAUSE";
    default: return "UNKNOWN";
    }
}

static void set_state(follower_state_t *state, follower_state_t next)
{
    if (*state == next) return;
    *state = next;
    ESP_LOGI(TAG, "state -> %s", state_name(next));
}

static void __attribute__((unused)) control_task_continuous_unused(void *argument)
{
    (void)argument;
    bool running = false;
    bool obstacle_armed = true;
    follower_state_t state = STATE_STOPPED;
    avoidance_state_t avoidance = AVOID_IDLE;
    uint32_t last_sequence = 0;
    uint32_t last_control_sequence = 0;
    uint32_t last_ultrasonic_sequence = 0;
    float previous_error = 0.0f;
    int last_seen_direction = 1;
    int64_t last_seen_ms = 0;
    int64_t search_started_ms = 0;
    int64_t last_log_ms = 0;
    int64_t avoidance_started_ms = 0;
    int64_t obstacle_clear_since_ms = 0;
    int obstacle_hits = 0;
    int rearm_hits = 0;

    ESP_LOGW(TAG, "hybrid edition: lift wheels for first test; BOOT starts/stops");
    ESP_LOGI(TAG, "TFT D is HC-SR04 distance: TRIG=GPIO18 ECHO=GPIO21 (3.3V max)");

    while (true) {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        vision_result_t vision = {0};
        const bool have_frame = read_latest_vision(&vision);

        if (button_pressed_event()) {
            running = !running;
            previous_error = 0.0f;
            last_seen_ms = now_ms;
            search_started_ms = 0;
            avoidance = AVOID_IDLE;
            obstacle_armed = true;
            obstacle_hits = 0;
            rearm_hits = 0;
            if (!running) {
                stop_chassis();
                set_state(&state, STATE_STOPPED);
                ESP_LOGI(TAG, "manual stop");
            } else {
                ESP_LOGI(TAG, "autonomous control started");
            }
        }

        /* Vision diagnostics must also run while motors are stopped.  This
         * lets the user place the car on the line and calibrate safely before
         * pressing BOOT. */
        if (have_frame && vision.sequence != last_sequence) {
            last_sequence = vision.sequence;
            if (now_ms - last_log_ms >= 500) {
                last_log_ms = now_ms;
                ESP_LOGI(TAG,
                         "state=%s avoid=%s run=%d valid=%d near=%.1f far=%.1f err=%.2f head=%.2f corner=%d black=%d%% ref=0x%X z=%u/%u/%u/%u auto_th=%d D=%dmm seq=%lu",
                         state_name(state), avoidance_name(avoidance), running, vision.valid,
                         (double)vision.near_center, (double)vision.far_center,
                         (double)vision.error, (double)vision.heading,
                         vision.corner_direction, vision.black_percent,
                         vision.virtual_black_mask,
                         vision.virtual_black_count[0], vision.virtual_black_count[1],
                         vision.virtual_black_count[2], vision.virtual_black_count[3],
                         vision.threshold, s_ultrasonic_raw_mm,
                         (unsigned long)vision.sequence);
            }
        }

        if (!running) {
            vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
            continue;
        }

        if (!have_frame || now_ms - vision.timestamp_ms > 600) {
            stop_chassis();
            set_state(&state, STATE_FAULT_STOP);
            vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
            continue;
        }

        /* Count only new HC-SR04 samples, not repeated 20 ms control ticks. */
        const uint32_t ultrasonic_sequence = s_ultrasonic_sequence;
        const int distance_mm = s_ultrasonic_raw_mm;
        if (ultrasonic_sequence != last_ultrasonic_sequence) {
            last_ultrasonic_sequence = ultrasonic_sequence;
            if (!obstacle_armed) {
                if (distance_mm >= OBSTACLE_REARM_MM) {
                    if (++rearm_hits >= OBSTACLE_REARM_COUNT) {
                        obstacle_armed = true;
                        rearm_hits = 0;
                        ESP_LOGI(TAG, "obstacle detector re-armed");
                    }
                } else {
                    rearm_hits = 0;
                }
            } else if (avoidance == AVOID_IDLE && distance_mm >= 0) {
                if (distance_mm <= OBSTACLE_EMERGENCY_MM) {
                    obstacle_hits = OBSTACLE_CONFIRM_COUNT;
                } else if (distance_mm <= OBSTACLE_TRIGGER_MM) {
                    ++obstacle_hits;
                } else {
                    obstacle_hits = 0;
                }
                if (obstacle_hits >= OBSTACLE_CONFIRM_COUNT) {
                    avoidance = AVOID_STOP;
                    avoidance_started_ms = now_ms;
                    obstacle_clear_since_ms = 0;
                    obstacle_hits = 0;
                    obstacle_armed = false;
                    stop_chassis();
                    ESP_LOGW(TAG, "obstacle confirmed at %d.%d cm",
                             distance_mm / 10, distance_mm % 10);
                }
            }
        }

        /* Avoidance owns all motors until its complete out-pass-return cycle
         * finishes. Vision is still updated so the return motion can find the
         * black line instead of relying only on elapsed time. */
        if (avoidance != AVOID_IDLE) {
            const int64_t elapsed_ms = now_ms - avoidance_started_ms;
            switch (avoidance) {
            case AVOID_STOP:
                stop_chassis();
                if (elapsed_ms >= AVOID_STOP_MS) {
                    avoidance = AVOID_STRAFE_LEFT;
                    avoidance_started_ms = now_ms;
                    ESP_LOGI(TAG, "avoid -> STRAFE_LEFT");
                }
                break;

            case AVOID_STRAFE_LEFT:
                strafe_left();
                if (elapsed_ms >= AVOID_STRAFE_OUT_MAX_MS) {
                    running = false;
                    stop_chassis();
                    set_state(&state, STATE_FAULT_STOP);
                    ESP_LOGE(TAG, "avoidance left strafe timed out");
                } else if (distance_mm < 0 || distance_mm > AVOID_CLEAR_MM) {
                    if (obstacle_clear_since_ms == 0) obstacle_clear_since_ms = now_ms;
                    if (now_ms - obstacle_clear_since_ms >= AVOID_CLEAR_CONFIRM_MS) {
                        avoidance = AVOID_FORWARD_PASS;
                        avoidance_started_ms = now_ms;
                        obstacle_clear_since_ms = 0;
                        ESP_LOGI(TAG, "avoid -> FORWARD_PASS");
                    }
                } else {
                    obstacle_clear_since_ms = 0;
                }
                break;

            case AVOID_FORWARD_PASS:
                drive_front_wheels(AVOID_FORWARD_LEFT_PERCENT,
                                   AVOID_FORWARD_RIGHT_PERCENT);
                if (elapsed_ms >= AVOID_FORWARD_MS) {
                    avoidance = AVOID_STRAFE_RIGHT;
                    avoidance_started_ms = now_ms;
                    ESP_LOGI(TAG, "avoid -> STRAFE_RIGHT");
                }
                break;

            case AVOID_STRAFE_RIGHT:
                strafe_right();
                if ((elapsed_ms >= AVOID_STRAFE_BACK_MIN_MS && vision.near_valid) ||
                    elapsed_ms >= AVOID_STRAFE_BACK_MAX_MS) {
                    stop_chassis();
                    avoidance = AVOID_REACQUIRE_PAUSE;
                    avoidance_started_ms = now_ms;
                    ESP_LOGI(TAG, "avoid -> REACQUIRE_PAUSE, line=%d", vision.near_valid);
                }
                break;

            case AVOID_REACQUIRE_PAUSE:
                stop_chassis();
                if (elapsed_ms >= AVOID_REACQUIRE_PAUSE_MS) {
                    avoidance = AVOID_IDLE;
                    last_seen_ms = now_ms;
                    search_started_ms = 0;
                    previous_error = 0.0f;
                    ESP_LOGI(TAG, "avoidance finished; return to line control");
                }
                break;

            case AVOID_IDLE:
            default:
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
            continue;
        }

        if (vision.valid) {
            last_seen_ms = now_ms;
            search_started_ms = 0;
            if (vision.error > 0.05f || vision.heading > 0.08f) last_seen_direction = 1;
            if (vision.error < -0.05f || vision.heading < -0.08f) last_seen_direction = -1;

            float derivative = 0.0f;
            if (vision.sequence != last_control_sequence) {
                derivative = vision.error - previous_error;
                previous_error = vision.error;
                last_control_sequence = vision.sequence;
            }
            const float correction = TRACK_KP_PERCENT * vision.error +
                                     TRACK_KD_PERCENT * derivative +
                                     TRACK_HEADING_GAIN_PERCENT * vision.heading;

            if (vision.corner_direction != 0) {
                last_seen_direction = vision.corner_direction;
                rotate_chassis(vision.corner_direction * CORNER_TURN_POWER_PERCENT);
                set_state(&state, vision.corner_direction < 0
                                      ? STATE_HARD_LEFT : STATE_HARD_RIGHT);
            } else if (!vision.near_valid ||
                       absolute_float(vision.error) >= HARD_TURN_ERROR) {
                const int direction = correction >= 0.0f ? 1 : -1;
                if (direction > 0) {
                    drive_front_wheels(HARD_TURN_POWER_PERCENT, 0);
                    set_state(&state, STATE_HARD_RIGHT);
                } else {
                    drive_front_wheels(0, HARD_TURN_POWER_PERCENT);
                    set_state(&state, STATE_HARD_LEFT);
                }
            } else {
                const int correction_percent = (int)clamp_float(
                    correction, -(float)TRACK_MAX_PERCENT,
                    (float)TRACK_MAX_PERCENT);
                drive_front_wheels(TRACK_BASE_LEFT_PERCENT + correction_percent,
                                   TRACK_BASE_RIGHT_PERCENT - correction_percent);
                set_state(&state, STATE_TRACK);
            }
        } else {
            previous_error = 0.0f;
            const int64_t lost_ms = now_ms - last_seen_ms;
            if (lost_ms <= SHORT_LOSS_GRACE_MS) {
                drive_front_wheels(TRACK_MIN_MOVING_PERCENT,
                                   TRACK_MIN_MOVING_PERCENT);
                set_state(&state, STATE_SEARCH);
            } else if (lost_ms <= SHORT_LOSS_GRACE_MS + LOST_SETTLE_MS) {
                stop_chassis();
                set_state(&state, STATE_SEARCH);
            } else {
                if (search_started_ms == 0) search_started_ms = now_ms;
                const int64_t search_ms = now_ms - search_started_ms;
                if (search_ms >= SEARCH_TIMEOUT_MS) {
                    running = false;
                    stop_chassis();
                    set_state(&state, STATE_FAULT_STOP);
                    ESP_LOGW(TAG, "line search timed out after %d ms", SEARCH_TIMEOUT_MS);
                } else {
                    int direction = last_seen_direction;
                    if (((search_ms / SEARCH_SWITCH_MS) & 1) != 0) direction = -direction;
                    rotate_chassis(direction * SEARCH_POWER_PERCENT);
                    set_state(&state, direction < 0 ? STATE_LOST_LEFT : STATE_LOST_RIGHT);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
    }
}

/* ================= Reference four-channel controller ==================== */

/* bit3=CH1(leftmost), bit0=CH4(rightmost), 1=black.
 * This is copied from line_follower_muy_bien_ultrasonic(7).ino. */
static const int8_t s_reference_turn_table[16] = {
    /*0000*/ 0,   /*0001*/ 90,  /*0010*/ 0,   /*0011*/ 70,
    /*0100*/ 0,   /*0101*/ 50,  /*0110*/ 0,   /*0111*/ 70,
    /*1000*/ -90, /*1001*/ 0,   /*1010*/ -50, /*1011*/ 0,
    /*1100*/ -70, /*1101*/ 0,   /*1110*/ -70, /*1111*/ 0,
};

#define REF_LEFT_BASE_RAW             114
#define REF_RIGHT_BASE_RAW            126
#define REF_REAR_GAIN                 0.5f
#define REF_CURVE_SCALE               0.7f
#define REF_LOST_STOP_MS              150
#define REF_LOST_TURN_RAW              70
#define REF_LOST_TURN_MAX_MS         4400

static void reference_lost_turn(int last_side)
{
    if (last_side < 0) {
        drive_reference_raw(-REF_LOST_TURN_RAW, REF_LOST_TURN_RAW,
                             REF_LOST_TURN_RAW);
    } else {
        /* No history follows the Arduino reference and searches right. */
        drive_reference_raw(REF_LOST_TURN_RAW, -REF_LOST_TURN_RAW,
                            -REF_LOST_TURN_RAW);
    }
}

static void reference_strafe_left(void)
{
    drive_reference_raw(-70, 77, -140);
}

static void reference_strafe_right(void)
{
    drive_reference_raw(70, -77, 140);
}

static int black_bit_count(uint8_t bits)
{
    int count = 0;
    for (int bit = 0; bit < 4; ++bit) count += (bits >> bit) & 1U;
    return count;
}

static void control_task(void *argument)
{
    (void)argument;
    bool running = false;
    follower_state_t state = STATE_STOPPED;
    avoidance_state_t avoidance = AVOID_IDLE;
    uint32_t last_frame_sequence = 0;
    uint32_t last_ultrasonic_sequence = 0;
    uint8_t last_bits = 0xff;
    int previous_steer = 0;
    int last_side = 0; /* -1=left, +1=right, 0=unknown */
    bool lost_active = false;
    int lost_phase = 0;
    int64_t lost_phase_started_ms = 0;
    int64_t avoidance_started_ms = 0;
    int64_t clear_since_ms = 0;
    int64_t last_log_ms = 0;

    ESP_LOGW(TAG, "reference four-channel control: lift wheels for first test");
    ESP_LOGI(TAG, "BOOT starts/stops; motors use camera mask + Arduino lookup table only");

    while (true) {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        vision_result_t vision = {0};
        const bool have_frame = read_latest_vision(&vision);
        const uint8_t bits = have_frame ? (vision.virtual_black_mask & 0x0fU) : 0;

        if (button_pressed_event()) {
            if (running) {
                running = false;
                avoidance = AVOID_IDLE;
                lost_active = false;
                previous_steer = 0;
                stop_chassis();
                set_state(&state, STATE_STOPPED);
                ESP_LOGI(TAG, "BTN: STOP");
            } else if (!have_frame || now_ms - vision.timestamp_ms > 600) {
                ESP_LOGW(TAG, "BTN: start rejected (camera frame unavailable)");
            } else if (bits == 0) {
                ESP_LOGW(TAG, "BTN: start rejected (four channels are all white)");
            } else {
                running = true;
                avoidance = AVOID_IDLE;
                lost_active = false;
                previous_steer = 0;
                last_side = 0;
                ESP_LOGI(TAG, "BTN: GO bits=0x%X", bits);
            }
        }

        /* Display and terminal diagnosis remain live before BOOT. */
        if (have_frame && vision.sequence != last_frame_sequence) {
            last_frame_sequence = vision.sequence;
            if (now_ms - last_log_ms >= 500) {
                last_log_ms = now_ms;
                ESP_LOGI(TAG,
                         "state=%s avoid=%s run=%d bits=%c%c%c%c count=%u/%u/%u/%u D=%dmm seq=%lu",
                         state_name(state), avoidance_name(avoidance), running,
                         (bits & 8U) ? '1' : '0', (bits & 4U) ? '1' : '0',
                         (bits & 2U) ? '1' : '0', (bits & 1U) ? '1' : '0',
                         vision.virtual_black_count[0], vision.virtual_black_count[1],
                         vision.virtual_black_count[2], vision.virtual_black_count[3],
                         s_ultrasonic_raw_mm, (unsigned long)vision.sequence);
            }
        }

        if (!running) {
            vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
            continue;
        }

        if (!have_frame || now_ms - vision.timestamp_ms > 600) {
            stop_chassis();
            set_state(&state, STATE_FAULT_STOP);
            vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
            continue;
        }

        /* Reference program: one fresh reading at or below 9 cm immediately
         * starts the fixed left-side obstacle bypass. */
        if (s_ultrasonic_sequence != last_ultrasonic_sequence) {
            last_ultrasonic_sequence = s_ultrasonic_sequence;
            const int distance_mm = s_ultrasonic_raw_mm;
            if (avoidance == AVOID_IDLE && distance_mm > 15 &&
                distance_mm <= OBSTACLE_TRIGGER_MM) {
                avoidance = AVOID_STOP;
                avoidance_started_ms = now_ms;
                clear_since_ms = 0;
                lost_active = false;
                previous_steer = 0;
                stop_chassis();
                ESP_LOGW(TAG, "AVOID: obstacle at %dmm, stop", distance_mm);
            }
        }

        if (avoidance != AVOID_IDLE) {
            const int64_t elapsed_ms = now_ms - avoidance_started_ms;
            const int distance_mm = s_ultrasonic_raw_mm;
            switch (avoidance) {
            case AVOID_STOP:
                stop_chassis();
                if (elapsed_ms >= 120) {
                    avoidance = AVOID_STRAFE_LEFT;
                    avoidance_started_ms = now_ms;
                    ESP_LOGI(TAG, "AVOID: strafe LEFT");
                }
                break;
            case AVOID_STRAFE_LEFT:
                reference_strafe_left();
                if (distance_mm < 0 || distance_mm > 200) {
                    if (clear_since_ms == 0) clear_since_ms = now_ms;
                    if (now_ms - clear_since_ms >= 150) {
                        avoidance = AVOID_FORWARD_PASS;
                        avoidance_started_ms = now_ms;
                        clear_since_ms = 0;
                        ESP_LOGI(TAG, "AVOID: obstacle clear, forward");
                    }
                } else {
                    clear_since_ms = 0;
                }
                break;
            case AVOID_FORWARD_PASS:
                drive_reference_raw(110, 130, 0);
                if (elapsed_ms >= 1100) {
                    avoidance = AVOID_STRAFE_RIGHT;
                    avoidance_started_ms = now_ms;
                    ESP_LOGI(TAG, "AVOID: strafe RIGHT, search line");
                }
                break;
            case AVOID_STRAFE_RIGHT:
                reference_strafe_right();
                if ((elapsed_ms >= 250 && bits != 0) || elapsed_ms >= 2600) {
                    stop_chassis();
                    avoidance = AVOID_REACQUIRE_PAUSE;
                    avoidance_started_ms = now_ms;
                    ESP_LOGI(TAG, "AVOID: return finished bits=0x%X", bits);
                }
                break;
            case AVOID_REACQUIRE_PAUSE:
                stop_chassis();
                if (elapsed_ms >= 120) {
                    avoidance = AVOID_IDLE;
                    lost_active = false;
                    previous_steer = 0;
                    ESP_LOGI(TAG, "AVOID: back to line following");
                }
                break;
            case AVOID_IDLE:
            default:
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
            continue;
        }

        if (bits == 0) {
            if (!lost_active) {
                lost_active = true;
                lost_phase = 0;
                lost_phase_started_ms = now_ms;
                previous_steer = 0;
                stop_chassis();
                ESP_LOGW(TAG, "LOST: stop");
            }
            if (lost_phase == 0 && now_ms - lost_phase_started_ms >= REF_LOST_STOP_MS) {
                lost_phase = 1;
                lost_phase_started_ms = now_ms;
                ESP_LOGI(TAG, "LOST: turn %s", last_side < 0 ? "LEFT" : "RIGHT");
            }
            if (lost_phase == 1) {
                if (now_ms - lost_phase_started_ms >= REF_LOST_TURN_MAX_MS) {
                    lost_phase = 2;
                    stop_chassis();
                    ESP_LOGW(TAG, "LOST: search timeout, remain stopped");
                } else {
                    reference_lost_turn(last_side);
                    set_state(&state, last_side < 0 ? STATE_LOST_LEFT : STATE_LOST_RIGHT);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
            continue;
        }

        if (lost_active) {
            lost_active = false;
            previous_steer = 0;
            ESP_LOGI(TAG, "LOST: line reacquired");
        }

        const int target = s_reference_turn_table[bits];
        if (target < 0) last_side = -1;
        if (target > 0) last_side = 1;
        const int steer = (target + 3 * previous_steer) / 4;
        previous_steer = steer;
        const float scale = black_bit_count(bits) == 3 ? REF_CURVE_SCALE : 1.0f;
        const int left_raw = (int)((REF_LEFT_BASE_RAW + steer) * scale);
        const int right_raw = (int)((REF_RIGHT_BASE_RAW - steer) * scale);
        const int rear_raw = (int)(-steer * REF_REAR_GAIN);
        drive_reference_raw(left_raw, right_raw, rear_raw);
        set_state(&state, STATE_TRACK);

        if (bits != last_bits) {
            last_bits = bits;
            ESP_LOGI(TAG, "TRACK bits=0x%X target=%d steer=%d LF=%d RF=%d B=%d",
                     bits, target, steer, left_raw, right_raw, rear_raw);
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
    }
}

/* =========================== Optional PC preview ========================= */

static uint16_t crc16_xmodem(const uint8_t *data, size_t length)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                   : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void make_preview(void)
{
    for (int y = 0; y < PREVIEW_HEIGHT; ++y) {
        for (int x = 0; x < PREVIEW_WIDTH; ++x) {
            s_preview[y * PREVIEW_WIDTH + x] =
                s_gray[(y * VISION_HEIGHT / PREVIEW_HEIGHT) * VISION_WIDTH +
                       x * VISION_WIDTH / PREVIEW_WIDTH];
        }
    }
}

static void send_preview_frame(int threshold)
{
    const size_t payload_offset = 10;
    s_serial_frame[0] = 'G'; s_serial_frame[1] = 'R';
    s_serial_frame[2] = 'A'; s_serial_frame[3] = 'Y';
    s_serial_frame[4] = PREVIEW_WIDTH & 0xff;
    s_serial_frame[5] = PREVIEW_WIDTH >> 8;
    s_serial_frame[6] = PREVIEW_HEIGHT & 0xff;
    s_serial_frame[7] = PREVIEW_HEIGHT >> 8;
    s_serial_frame[8] = threshold & 0xff;
    s_serial_frame[9] = threshold >> 8;
    memcpy(s_serial_frame + payload_offset, s_preview, PREVIEW_PIXELS);
    const uint16_t crc = crc16_xmodem(s_preview, PREVIEW_PIXELS);
    s_serial_frame[payload_offset + PREVIEW_PIXELS] = crc & 0xff;
    s_serial_frame[payload_offset + PREVIEW_PIXELS + 1] = crc >> 8;
    uart_write_bytes(UART_NUM_0, s_serial_frame, sizeof(s_serial_frame));
}

/* ================================ UVC ==================================== */

static const char *format_name(enum uvc_host_stream_format format)
{
    switch (format) {
    case UVC_VS_FORMAT_MJPEG: return "MJPEG";
    case UVC_VS_FORMAT_YUY2: return "YUY2";
    case UVC_VS_FORMAT_H264: return "H264";
    case UVC_VS_FORMAT_H265: return "H265";
    case UVC_VS_FORMAT_NV12: return "NV12";
    default: return "UNDEFINED";
    }
}

static esp_jpeg_image_scale_t choose_decode_scale(uint32_t width, uint32_t height)
{
    if (width <= DECODE_MAX_W && height <= DECODE_MAX_H) return JPEG_IMAGE_SCALE_0;
    if (width / 2U <= DECODE_MAX_W && height / 2U <= DECODE_MAX_H) return JPEG_IMAGE_SCALE_1_2;
    if (width / 4U <= DECODE_MAX_W && height / 4U <= DECODE_MAX_H) return JPEG_IMAGE_SCALE_1_4;
    return JPEG_IMAGE_SCALE_1_8;
}

static void camera_process_task(void *argument)
{
    (void)argument;
    int64_t last_decode_us = 0;
    while (true) {
        if (xSemaphoreTake(s_frame_ready, portMAX_DELAY) != pdTRUE) continue;
        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_decode_us < FRAME_PROCESS_MIN_GAP_US) continue;
        last_decode_us = now_us;

        if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(200)) != pdTRUE) continue;
        esp_jpeg_image_cfg_t config = {
            .indata = s_jpeg_buffer,
            .indata_size = (uint32_t)s_jpeg_size,
            .outbuf = s_decode_buffer,
            .outbuf_size = DECODE_BUFFER_BYTES,
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            .out_scale = JPEG_IMAGE_SCALE_1_4,
            .flags = {.swap_color_bytes = 0},
            .advanced = {
                .working_buffer = s_jpeg_work,
                .working_buffer_size = sizeof(s_jpeg_work),
            },
        };
        esp_jpeg_image_output_t info = {0};
        esp_err_t error = esp_jpeg_get_image_info(&config, &info);
        if (error == ESP_OK) config.out_scale = choose_decode_scale(info.width, info.height);
        esp_jpeg_image_output_t output = {0};
        if (error == ESP_OK) error = esp_jpeg_decode(&config, &output);
        xSemaphoreGive(s_frame_mutex);

        if (error != ESP_OK || output.width == 0 || output.height == 0) {
            ESP_LOGW(TAG, "MJPEG decode failed: %s", esp_err_to_name(error));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        const size_t stride = output.output_len / output.height;
        make_control_gray(s_decode_buffer, stride, output.width, output.height);
        const vision_result_t result = analyze_gray_image();
        publish_vision(&result);
        if (s_stream_enabled) {
            make_preview();
            send_preview_frame(result.threshold);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void usb_host_task(void *argument)
{
    (void)argument;
    while (true) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}

static void stream_event_callback(const uvc_host_stream_event_data_t *event,
                                  void *user_context)
{
    (void)user_context;
    if (event->type == UVC_HOST_TRANSFER_ERROR) {
        ESP_LOGE(TAG, "USB transfer error: %s",
                 esp_err_to_name(event->transfer_error.error));
    } else if (event->type == UVC_HOST_DEVICE_DISCONNECTED) {
        ESP_LOGW(TAG, "camera disconnected");
        s_device_connected = false;
        uvc_host_stream_close(event->device_disconnected.stream_hdl);
    } else if (event->type == UVC_HOST_FRAME_BUFFER_OVERFLOW) {
        ESP_LOGW(TAG, "camera frame buffer overflow");
    }
}

static bool camera_frame_callback(const uvc_host_frame_t *frame,
                                  void *user_context)
{
    (void)user_context;
    if (frame->data_len == 0 || frame->data_len > MAX_JPEG_BYTES) return true;
    if (xSemaphoreTake(s_frame_mutex, 0) != pdTRUE) return true;
    memcpy(s_jpeg_buffer, frame->data, frame->data_len);
    s_jpeg_size = frame->data_len;
    s_input_width = frame->vs_format.h_res;
    s_input_height = frame->vs_format.v_res;
    ++s_camera_frame_count;
    xSemaphoreGive(s_frame_mutex);
    xSemaphoreGive(s_frame_ready);
    return true;
}

static bool profile_is_listed(const uvc_host_frame_info_t *format)
{
    for (int i = 0; i < s_profile_count; ++i) {
        if (s_profiles[i].format == format->format &&
            s_profiles[i].h_res == format->h_res &&
            s_profiles[i].v_res == format->v_res) return true;
    }
    return false;
}

static void add_profile(const uvc_host_frame_info_t *format)
{
    if (s_profile_count >= MAX_PROFILES || profile_is_listed(format)) return;
    s_profiles[s_profile_count++] = (uvc_host_stream_format_t) {
        .h_res = format->h_res,
        .v_res = format->v_res,
        .fps = 0,
        .format = format->format,
    };
}

static void build_mjpeg_profiles(const uvc_host_frame_info_t *formats, size_t count)
{
    static const struct { uint16_t width; uint16_t height; } preferred[] = {
        {640, 480}, {1280, 720}, {960, 540}, {800, 480}, {320, 240},
    };
    s_profile_count = 0;
    for (size_t p = 0; p < sizeof(preferred) / sizeof(preferred[0]); ++p) {
        for (size_t i = 0; i < count; ++i) {
            if (formats[i].format == UVC_VS_FORMAT_MJPEG &&
                formats[i].h_res == preferred[p].width &&
                formats[i].v_res == preferred[p].height) add_profile(&formats[i]);
        }
    }
    for (size_t i = 0; i < count; ++i) {
        if (formats[i].format == UVC_VS_FORMAT_MJPEG) add_profile(&formats[i]);
    }
}

static void uvc_stream_task(void *argument)
{
    (void)argument;
    int profile_index = 0;
    while (true) {
        if (s_profile_count == 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        uvc_host_stream_config_t config = {0};
        config.event_cb = stream_event_callback;
        config.frame_cb = camera_frame_callback;
        config.usb.dev_addr = s_device_addr;
        config.usb.vid = UVC_HOST_ANY_VID;
        config.usb.pid = UVC_HOST_ANY_PID;
        config.usb.uvc_stream_index = s_stream_index;
        config.vs_format = s_profiles[profile_index];
        config.advanced.number_of_frame_buffers = UVC_FRAME_BUFFERS;
        config.advanced.frame_heap_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
        config.advanced.number_of_urbs = UVC_URB_COUNT;
        config.advanced.urb_size = UVC_URB_SIZE;

        ESP_LOGI(TAG, "trying %s %ux%u", format_name(config.vs_format.format),
                 (unsigned)config.vs_format.h_res,
                 (unsigned)config.vs_format.v_res);
        uvc_host_stream_hdl_t stream = NULL;
        esp_err_t error = uvc_host_stream_open(&config, pdMS_TO_TICKS(5000), &stream);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "stream open failed: %s", esp_err_to_name(error));
            profile_index = (profile_index + 1) % s_profile_count;
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        s_device_connected = true;
        error = uvc_host_stream_start(stream);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "stream start failed: %s", esp_err_to_name(error));
            uvc_host_stream_close(stream);
            profile_index = (profile_index + 1) % s_profile_count;
            continue;
        }
        ESP_LOGI(TAG, "camera streaming; control is local, PC viewer optional");
        while (s_device_connected) vTaskDelay(pdMS_TO_TICKS(1000));
        profile_index = 0;
    }
}

static void uvc_driver_event_callback(const uvc_host_driver_event_data_t *event,
                                      void *user_context)
{
    (void)user_context;
    if (event->type != UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) return;
    size_t count = 0;
    esp_err_t error = uvc_host_get_frame_list(event->device_connected.dev_addr,
                                               event->device_connected.uvc_stream_index,
                                               NULL, &count);
    if (error != ESP_OK || count == 0) {
        ESP_LOGE(TAG, "no UVC formats: %s", esp_err_to_name(error));
        return;
    }
    uvc_host_frame_info_t *formats = calloc(count, sizeof(*formats));
    if (formats == NULL) return;
    error = uvc_host_get_frame_list(event->device_connected.dev_addr,
                                    event->device_connected.uvc_stream_index,
                                    (uvc_host_frame_info_t (*)[])formats, &count);
    if (error != ESP_OK) {
        free(formats);
        return;
    }
    s_device_addr = event->device_connected.dev_addr;
    s_stream_index = event->device_connected.uvc_stream_index;
    build_mjpeg_profiles(formats, count);
    free(formats);
    ESP_LOGI(TAG, "UVC camera connected, %d MJPEG profiles", s_profile_count);
    if (s_profile_count > 0 && !s_stream_task_started) {
        s_stream_task_started = true;
        assert(xTaskCreate(uvc_stream_task, "uvc_stream", 4096, NULL, 10, NULL) == pdPASS);
    }
}

/* ============================== Startup ================================= */

static void init_serial(void)
{
    const uart_config_t config = {
        .baud_rate = STREAM_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    if (!uart_is_driver_installed(UART_NUM_0)) {
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 16384, 0, NULL, 0));
    }
    uart_vfs_dev_use_driver(UART_NUM_0);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
}

static void serial_command_task(void *argument)
{
    (void)argument;
    printf("\n=== Camera four-channel black/white display test ===\n");
    printf("Camera %s: D-=GPIO%d D+=GPIO%d\n",
           CAMERA_MODEL_NAME, USB_D_MINUS_GPIO, USB_D_PLUS_GPIO);
    printf("TFT updates automatically; v starts PC gray preview; x stops preview.\n\n");
    while (true) {
        const int character = getchar();
        if (character == 'v' || character == 'V') {
            s_stream_enabled = true;
            printf("preview ON\n");
        } else if (character == 'x' || character == 'X') {
            s_stream_enabled = false;
            printf("preview OFF\n");
        } else if (character != EOF && character != '\n' && character != '\r') {
            printf("commands: v / x\n");
        }
    }
}

static bool allocate_buffers(void)
{
    s_jpeg_buffer = heap_caps_malloc(MAX_JPEG_BYTES,
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_decode_buffer = heap_caps_malloc(DECODE_BUFFER_BYTES,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_jpeg_buffer == NULL || s_decode_buffer == NULL) {
        ESP_LOGE(TAG, "PSRAM allocation failed");
        return false;
    }
    return true;
}

void app_main(void)
{
    s_frame_ready = xSemaphoreCreateCounting(1, 0);
    s_frame_mutex = xSemaphoreCreateMutex();
    s_vision_mutex = xSemaphoreCreateMutex();
    if (s_frame_ready == NULL || s_frame_mutex == NULL ||
        s_vision_mutex == NULL || !allocate_buffers()) {
        ESP_LOGE(TAG, "initialization failed");
        return;
    }

    init_serial();
    configure_ultrasonic_hardware();
    ESP_ERROR_CHECK(tft18_status_display_init());
    assert(xTaskCreate(ultrasonic_task, "ultrasonic", 3072, NULL, 2, NULL) == pdPASS);
    assert(xTaskCreate(serial_command_task, "serial_command", 3072, NULL, 3, NULL) == pdPASS);
    assert(xTaskCreate(camera_process_task, "camera_process", 8192, NULL, 6, NULL) == pdPASS);

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    assert(xTaskCreate(usb_host_task, "usb_host", 4096, NULL, 15, NULL) == pdPASS);

    const uvc_host_driver_config_t uvc_config = {
        .driver_task_stack_size = 4 * 1024,
        .driver_task_priority = 16,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = uvc_driver_event_callback,
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_config));
    ESP_LOGI(TAG, "ready; waiting for camera; TFT four channels update automatically");
}
