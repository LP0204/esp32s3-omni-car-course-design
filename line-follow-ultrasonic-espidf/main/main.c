#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tft18_sensor_display.h"

/* ---------------- 接口定义：与 Arduino 原程序完全一致 ---------------- */
#define LF_IN1 GPIO_NUM_8
#define LF_IN2 GPIO_NUM_9
#define LF_PWM GPIO_NUM_10
#define REAR_IN1 GPIO_NUM_12
#define REAR_IN2 GPIO_NUM_13
#define REAR_PWM GPIO_NUM_14
#define RF_IN1 GPIO_NUM_15
#define RF_IN2 GPIO_NUM_16
#define RF_PWM GPIO_NUM_17
#define MOTOR_STBY GPIO_NUM_11

#define OUT4 GPIO_NUM_7
#define OUT3 GPIO_NUM_6
#define OUT2 GPIO_NUM_5
#define OUT1 GPIO_NUM_4
#define US_TRIG GPIO_NUM_18
#define US_ECHO GPIO_NUM_21
#define BOOT_BTN GPIO_NUM_0

/* ---------------- 参数：数值沿用 .ino，可直接在此处调整 ---------------- */
#define IR_ON_BLACK_LEVEL 0
#define BOOT_DEBOUNCE_MS 40
#define CONTROL_PERIOD_MS 10
#define PWM_FREQ 5000
#define PWM_BITS LEDC_TIMER_8_BIT
#define LF_PWM_CH LEDC_CHANNEL_0
#define RF_PWM_CH LEDC_CHANNEL_1
#define REAR_PWM_CH LEDC_CHANNEL_2

#define OBSTACLE_TRIGGER_CM 9.0f
#define OBSTACLE_CONFIRM_COUNT 1
#define ULTRA_SAMPLE_MS 30
#define ULTRA_TIMEOUT_US 4000
#define ULTRA_NO_ECHO_TIMEOUT_MS 500
#define OBSTACLE_CLEAR_CM 20.0f
#define AVOID_CLEAR_CONFIRM_MS 150
#define AVOID_STOP_MS 120
#define STRAFE_LF_SPEED 70
#define STRAFE_RF_SPEED 77
#define STRAFE_REAR_SPEED 140
#define PASS_FORWARD_MS 1100
#define PASS_LF_SPEED 110
#define PASS_RF_SPEED 130
#define STRAFE_BACK_MIN_MS 250
#define STRAFE_BACK_MAX_MS 2600
#define REACQUIRE_PAUSE_MS 120

#define LEFT_SIDE 1
#define RIGHT_SIDE 2
#define LOST_STOP_MS 150
#define LOST_TURN_SPEED 70
#define LOST_TURN_MAX_MS 4400

#define LF_SPEED 114
#define RF_SPEED 126
#define REAR_GAIN 0.5f
#define CURVE_SPEED_SCALE 0.7f
#define LF_DIR_SIGN (-1)
#define RF_DIR_SIGN (-1)
#define REAR_DIR_SIGN 1

static const char *TAG = "LINE_AVOID";

typedef struct {
    gpio_num_t in1, in2, pwm_gpio;
    ledc_channel_t channel;
    int direction_sign;
    int command;
} wheel_t;

static wheel_t s_lf = {LF_IN1, LF_IN2, LF_PWM, LF_PWM_CH, LF_DIR_SIGN, 0};
static wheel_t s_rf = {RF_IN1, RF_IN2, RF_PWM, RF_PWM_CH, RF_DIR_SIGN, 0};
static wheel_t s_rear = {REAR_IN1, REAR_IN2, REAR_PWM, REAR_PWM_CH, REAR_DIR_SIGN, 0};

typedef enum {
    AVOID_IDLE = 0,
    AVOID_STOP,
    AVOID_STRAFE_OUT,
    AVOID_FORWARD_PASS,
    AVOID_STRAFE_BACK,
    AVOID_REACQUIRE_PAUSE,
} avoid_state_t;

static const int8_t turn_table[16] = {
    0, 90, 0, 70, 0, 50, 0, 70,
    -90, 0, -50, 0, -70, 0, -70, 0,
};

static int clamp255(int v)
{
    if (v > 255) return 255;
    if (v < -255) return -255;
    return v;
}

static uint32_t duty255(int v)
{
    v = abs(clamp255(v));
    return (uint32_t)v;
}

static void wheel_write(wheel_t *wheel, int command)
{
    command = clamp255(command);
    wheel->command = command;
    int electrical = command * wheel->direction_sign;
    if (electrical > 0) {
        gpio_set_level(wheel->in1, 1);
        gpio_set_level(wheel->in2, 0);
    } else if (electrical < 0) {
        gpio_set_level(wheel->in1, 0);
        gpio_set_level(wheel->in2, 1);
    } else {
        gpio_set_level(wheel->in1, 0);
        gpio_set_level(wheel->in2, 0);
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, wheel->channel, duty255(electrical));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, wheel->channel);
}

static int command_percent(int command)
{
    return (command * 100) / 255;
}

static void drive(int lf, int rf, int rear)
{
    wheel_write(&s_lf, lf);
    wheel_write(&s_rf, rf);
    wheel_write(&s_rear, rear);
    tft18_sensor_display_set_motor_commands(command_percent(lf),
                                             command_percent(rf),
                                             command_percent(rear));
}

static void stop_all(void)
{
    drive(0, 0, 0);
}

static uint8_t read_ir_bits(void)
{
    uint8_t bits = 0;
    if (gpio_get_level(OUT4) == IR_ON_BLACK_LEVEL) bits |= 0x08;
    if (gpio_get_level(OUT3) == IR_ON_BLACK_LEVEL) bits |= 0x04;
    if (gpio_get_level(OUT2) == IR_ON_BLACK_LEVEL) bits |= 0x02;
    if (gpio_get_level(OUT1) == IR_ON_BLACK_LEVEL) bits |= 0x01;
    return bits;
}

static bool wait_for_level(gpio_num_t pin, int level, int timeout_us,
                           int64_t *timestamp)
{
    const int64_t deadline = esp_timer_get_time() + timeout_us;
    while (gpio_get_level(pin) != level) {
        if (esp_timer_get_time() >= deadline) return false;
    }
    *timestamp = esp_timer_get_time();
    return true;
}

static float ultrasonic_once_cm(void)
{
    if (gpio_get_level(US_ECHO) != 0) return -1.0f;
    gpio_set_level(US_TRIG, 0);
    esp_rom_delay_us(2);
    gpio_set_level(US_TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(US_TRIG, 0);
    int64_t start, end;
    if (!wait_for_level(US_ECHO, 1, ULTRA_TIMEOUT_US, &start)) return -1.0f;
    if (!wait_for_level(US_ECHO, 0, ULTRA_TIMEOUT_US, &end)) return -1.0f;
    return (float)(end - start) * 0.0343f * 0.5f;
}

static float median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

static bool s_running;
static int s_last_side;
static bool s_lost_active;
static int s_lost_phase;
static int64_t s_lost_phase_start;
static int s_prev_steer;
static uint8_t s_last_bits = 0xff;
static avoid_state_t s_avoid = AVOID_IDLE;
static int64_t s_avoid_start;
static int s_obstacle_hits;
static int64_t s_clear_since;
static float s_ultra_raw = -1.0f;
static float s_ultra_display = -1.0f;
static float s_ultra_history[3] = {-1.0f, -1.0f, -1.0f};
static int s_ultra_count;
static int s_ultra_index;
static int64_t s_last_ultra_ms;
static int64_t s_last_valid_ultra_ms;

static const char *avoid_name(avoid_state_t state)
{
    switch (state) {
    case AVOID_IDLE: return "IDLE";
    case AVOID_STOP: return "STOP";
    case AVOID_STRAFE_OUT: return "STRAFE_OUT";
    case AVOID_FORWARD_PASS: return "FORWARD_PASS";
    case AVOID_STRAFE_BACK: return "STRAFE_BACK";
    case AVOID_REACQUIRE_PAUSE: return "REACQUIRE_PAUSE";
    default: return "?";
    }
}

static void set_avoid(avoid_state_t state, int64_t now)
{
    if (s_avoid != state) {
        s_avoid = state;
        s_avoid_start = now;
        ESP_LOGI(TAG, "avoid state -> %s", avoid_name(state));
    }
}

static void reset_run_state(void)
{
    s_last_side = 0;
    s_lost_active = false;
    s_lost_phase = 0;
    s_prev_steer = 0;
    s_obstacle_hits = 0;
    s_clear_since = 0;
    set_avoid(AVOID_IDLE, esp_timer_get_time() / 1000);
}

static void lost_turn(void)
{
    if (s_last_side == LEFT_SIDE) {
        drive(-LOST_TURN_SPEED, LOST_TURN_SPEED, LOST_TURN_SPEED);
    } else {
        drive(LOST_TURN_SPEED, -LOST_TURN_SPEED, -LOST_TURN_SPEED);
    }
}

static void strafe(int direction)
{
    if (direction < 0) {
        drive(-STRAFE_LF_SPEED, STRAFE_RF_SPEED, -STRAFE_REAR_SPEED);
    } else {
        drive(STRAFE_LF_SPEED, -STRAFE_RF_SPEED, STRAFE_REAR_SPEED);
    }
}

static void update_ultrasonic(int64_t now_ms)
{
    if (now_ms - s_last_ultra_ms < ULTRA_SAMPLE_MS) return;
    s_last_ultra_ms = now_ms;
    s_ultra_raw = ultrasonic_once_cm();
    if (s_ultra_raw > 1.5f && s_ultra_raw < 100.0f) {
        s_last_valid_ultra_ms = now_ms;
        s_ultra_history[s_ultra_index] = s_ultra_raw;
        s_ultra_index = (s_ultra_index + 1) % 3;
        if (s_ultra_count < 3) ++s_ultra_count;
        if (s_ultra_count == 1) s_ultra_display = s_ultra_history[0];
        else if (s_ultra_count == 2) s_ultra_display = (s_ultra_history[0] + s_ultra_history[1]) * 0.5f;
        else s_ultra_display = median3(s_ultra_history[0], s_ultra_history[1], s_ultra_history[2]);
    } else if (now_ms - s_last_valid_ultra_ms > ULTRA_NO_ECHO_TIMEOUT_MS) {
        s_ultra_display = -1.0f;
    }
    tft18_sensor_display_set_distance_mm(s_ultra_display > 0 ? (int)(s_ultra_display * 10.0f) : -1);
}

static void begin_avoidance(int64_t now_ms)
{
    s_obstacle_hits = 0;
    s_clear_since = 0;
    s_lost_active = false;
    s_prev_steer = 0;
    stop_all();
    set_avoid(AVOID_STOP, now_ms);
    ESP_LOGW(TAG, "obstacle detected at %.1f cm", s_ultra_raw);
}

static bool handle_avoidance(int64_t now_ms, uint8_t bits)
{
    if (s_avoid == AVOID_IDLE) return false;
    int64_t elapsed = now_ms - s_avoid_start;
    switch (s_avoid) {
    case AVOID_STOP:
        stop_all();
        if (elapsed >= AVOID_STOP_MS) set_avoid(AVOID_STRAFE_OUT, now_ms);
        break;
    case AVOID_STRAFE_OUT:
        strafe(-1);
        if (!(s_ultra_raw > 1.5f && s_ultra_raw <= OBSTACLE_CLEAR_CM)) {
            if (s_clear_since == 0) s_clear_since = now_ms;
            else if (now_ms - s_clear_since >= AVOID_CLEAR_CONFIRM_MS) {
                s_clear_since = 0;
                set_avoid(AVOID_FORWARD_PASS, now_ms);
            }
        } else s_clear_since = 0;
        break;
    case AVOID_FORWARD_PASS:
        drive(PASS_LF_SPEED, PASS_RF_SPEED, 0);
        if (elapsed >= PASS_FORWARD_MS) set_avoid(AVOID_STRAFE_BACK, now_ms);
        break;
    case AVOID_STRAFE_BACK:
        strafe(1);
        if (elapsed >= STRAFE_BACK_MIN_MS && bits != 0) {
            stop_all(); set_avoid(AVOID_REACQUIRE_PAUSE, now_ms);
        } else if (elapsed >= STRAFE_BACK_MAX_MS) {
            stop_all(); set_avoid(AVOID_REACQUIRE_PAUSE, now_ms);
            ESP_LOGW(TAG, "line reacquire timeout; returning to LOST logic");
        }
        break;
    case AVOID_REACQUIRE_PAUSE:
        stop_all();
        if (elapsed >= REACQUIRE_PAUSE_MS) {
            s_clear_since = 0; s_lost_active = false; s_prev_steer = 0;
            set_avoid(AVOID_IDLE, now_ms);
        }
        break;
    default: set_avoid(AVOID_IDLE, now_ms); break;
    }
    return true;
}

static void line_follow(int64_t now_ms, uint8_t bits)
{
    if (bits == 0) {
        if (!s_lost_active) {
            s_lost_active = true; s_lost_phase = 0; s_lost_phase_start = now_ms;
            stop_all(); s_prev_steer = 0; ESP_LOGW(TAG, "line lost: stop");
        }
        if (s_lost_phase == 0) {
            if (now_ms - s_lost_phase_start >= LOST_STOP_MS) {
                s_lost_phase = 1; s_lost_phase_start = now_ms;
                ESP_LOGI(TAG, "line lost: search %s", s_last_side == LEFT_SIDE ? "LEFT" : "RIGHT");
            }
        } else if (s_lost_phase == 1) {
            if (now_ms - s_lost_phase_start >= LOST_TURN_MAX_MS) {
                s_lost_phase = 2; stop_all();
            } else lost_turn();
        }
        return;
    }
    int target = turn_table[bits & 0x0f];
    if (target < 0) s_last_side = LEFT_SIDE;
    else if (target > 0) s_last_side = RIGHT_SIDE;
    if (s_lost_active) { s_lost_active = false; s_prev_steer = 0; ESP_LOGI(TAG, "line reacquired"); }
    int steer = (target + 3 * s_prev_steer) / 4;
    s_prev_steer = steer;
    int count = ((bits & 8) != 0) + ((bits & 4) != 0) + ((bits & 2) != 0) + ((bits & 1) != 0);
    float scale = count == 3 ? CURVE_SPEED_SCALE : 1.0f;
    int lf = (int)((LF_SPEED + steer) * scale);
    int rf = (int)((RF_SPEED - steer) * scale);
    int rear = (int)(-steer * REAR_GAIN);
    drive(lf, rf, rear);
    if (bits != s_last_bits) {
        ESP_LOGI(TAG, "IR=0b%d%d%d%d target=%d lf=%d rf=%d rear=%d",
                 (bits>>3)&1,(bits>>2)&1,(bits>>1)&1,bits&1,target,lf,rf,rear);
        s_last_bits = bits;
    }
}

static bool boot_pressed(int64_t now_ms)
{
    static int previous = 1;
    static int64_t last_change;
    int current = gpio_get_level(BOOT_BTN);
    if (current != previous && now_ms - last_change >= BOOT_DEBOUNCE_MS) {
        previous = current; last_change = now_ms; return current == 0;
    }
    return false;
}

static void configure_hardware(void)
{
    const gpio_config_t outputs = {
        .pin_bit_mask=(1ULL<<LF_IN1)|(1ULL<<LF_IN2)|(1ULL<<REAR_IN1)|(1ULL<<REAR_IN2)|
                      (1ULL<<RF_IN1)|(1ULL<<RF_IN2)|(1ULL<<MOTOR_STBY)|(1ULL<<US_TRIG),
        .mode=GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&outputs));
    const gpio_config_t inputs = {
        .pin_bit_mask=(1ULL<<BOOT_BTN)|(1ULL<<OUT4)|(1ULL<<OUT3)|(1ULL<<OUT2)|(1ULL<<OUT1),
        .mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&inputs));
    const gpio_config_t echo = {.pin_bit_mask=1ULL<<US_ECHO,.mode=GPIO_MODE_INPUT,.pull_down_en=GPIO_PULLDOWN_ENABLE};
    ESP_ERROR_CHECK(gpio_config(&echo));
    gpio_set_level(MOTOR_STBY, 1); gpio_set_level(US_TRIG, 0);
    const ledc_timer_config_t timer = {.speed_mode=LEDC_LOW_SPEED_MODE,.duty_resolution=PWM_BITS,.timer_num=LEDC_TIMER_0,.freq_hz=PWM_FREQ,.clk_cfg=LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    wheel_t *wheels[] = {&s_lf,&s_rf,&s_rear};
    for (int i=0;i<3;++i) {
        const ledc_channel_config_t c = {.gpio_num=wheels[i]->pwm_gpio,.speed_mode=LEDC_LOW_SPEED_MODE,.channel=wheels[i]->channel,.intr_type=LEDC_INTR_DISABLE,.timer_sel=LEDC_TIMER_0,.duty=0,.hpoint=0};
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }
    stop_all();
}

static void serial_command(void)
{
    uint8_t c;
    if (uart_read_bytes(UART_NUM_0, &c, 1, 0) != 1) return;
    if (c == 'g' || c == 'G') {
        s_running = true; reset_run_state(); ESP_LOGI(TAG, "GO");
    } else if (c == 's' || c == 'S') {
        s_running = false; reset_run_state(); stop_all(); ESP_LOGI(TAG, "STOP");
    } else if (c == 'b' || c == 'B') {
        ESP_LOGI(TAG, "IR bits=0x%02x", read_ir_bits());
    } else if (c == 'u' || c == 'U') {
        ESP_LOGI(TAG, "distance=%.1f cm", ultrasonic_once_cm());
    }
}

void app_main(void)
{
    configure_hardware();
    const uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_cfg));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(tft18_sensor_display_init());
    ESP_LOGI(TAG, "ready: BOOT starts/stops; g=go s=stop b=IR u=distance");
    ESP_LOGI(TAG, "IR OUT4..OUT1=GPIO7,6,5,4; HC-SR04=TRIG18 ECHO21");
    int64_t last_log = 0;
    while (true) {
        int64_t now = esp_timer_get_time() / 1000;
        uint8_t bits = read_ir_bits();
        tft18_sensor_display_update((uint8_t)(~bits) & 0x0f);
        serial_command();
        if (boot_pressed(now)) {
            if (s_running) {
                s_running = false; reset_run_state(); stop_all(); ESP_LOGI(TAG, "BOOT -> STOP");
            } else if (bits == 0) {
                ESP_LOGW(TAG, "BOOT start rejected: no black line under sensors");
            } else {
                s_running = true; reset_run_state(); ESP_LOGI(TAG, "BOOT -> GO");
            }
        }
        update_ultrasonic(now);
        if (s_running && s_avoid == AVOID_IDLE && s_ultra_raw > 1.5f && s_ultra_raw <= OBSTACLE_TRIGGER_CM) {
            if (++s_obstacle_hits >= OBSTACLE_CONFIRM_COUNT) begin_avoidance(now);
        } else if (s_avoid == AVOID_IDLE) s_obstacle_hits = 0;
        if (s_running) {
            if (!handle_avoidance(now, bits)) line_follow(now, bits);
        }
        if (now - last_log >= 1000) {
            last_log = now;
            ESP_LOGI(TAG, "run=%d avoid=%s IR=0x%02x D=%s%.1fcm motor=%d/%d/%d",
                     s_running, avoid_name(s_avoid), bits,
                     s_ultra_raw > 0 ? "" : "?", s_ultra_raw > 0 ? s_ultra_raw : 0.0f,
                     s_lf.command, s_rf.command, s_rear.command);
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
