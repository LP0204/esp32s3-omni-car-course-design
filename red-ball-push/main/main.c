/*
 * ESP32-S3 red-ball push controller
 *
 * The UVC camera pipeline is derived from the verified camera-color-viewer
 * project. This version runs fully on the ESP32-S3 and does not transmit
 * images to a computer.
 *
 * Data path (adapted for the JQ-CAM12-720D-V1 reference project):
 * UVC MJPEG camera -> usb_host_uvc v2 -> esp_jpeg -> 80x60 red sampling
 * -> red centroid -> motor state machine.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
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
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

static const char *TAG = "RED_BALL";

/* 接线情况2：ESP-Claw 三合一摄像头。D-/D+ 由 ESP32-S3 原生 USB 使用。 */
#define USB_D_MINUS_GPIO 19
#define USB_D_PLUS_GPIO 20
#define CAMERA_MODEL_NAME "JQ-CAM12-720D-V1"

/* A 640x480 MJPEG frame is normally below 100 KB.  The frame and decode
 * buffers must be in PSRAM, not internal RAM. */
#define MAX_JPEG_BYTES (512U * 1024U)
#define DECODE_MAX_W 160
#define DECODE_MAX_H 120
#define DECODE_BUFFER_BYTES (DECODE_MAX_W * DECODE_MAX_H * 2U)
#define JPEG_WORK_BYTES 4096

/* Red detection samples 80x60 points directly from the decoded frame. */
#define RED_MAP_WIDTH 80
#define RED_MAP_HEIGHT 60
#define RED_PROCESS_PERIOD_US (100U * 1000U) /* 10 FPS target. */
#define DECODE_TASK_YIELD_MS 2
#define CAMERA_REQUEST_FPS 10.0f

/* This matches the normal physical camera mounting on the car. Set to 0 if
 * you deliberately want the unrotated raw orientation. */
#define CAMERA_ROTATE_180 1

#define UVC_FRAME_BUFFERS 3
#define UVC_URB_COUNT 4
#define UVC_URB_SIZE (8U * 1024U)
#define MAX_PROFILES 8

/* Red-ball control: deliberately only one colour metric. */
#define REDNESS_THRESHOLD 90
#define RED_MIN_CHANNEL 90
#define RED_PIXELS_TO_TRIGGER 20
#define RED_CONFIRM_FRAMES 3
#define CENTER_CONFIRM_FRAMES 3
#define CENTER_DEADBAND_PX 5
#define ALIGN_LOST_FRAME_LIMIT 3
#define TURN_STEP_POWER 120
#define SEARCH_STEP_MS 200
#define ALIGN_INITIAL_STEP_MS 80
#define ALIGN_MIN_STEP_MS 20
#define TURN_BRAKE_MS 35
#define TURN_SETTLE_MS 120
#define LEFT_STRAIGHT_POWER_PERCENT 51
#define RIGHT_STRAIGHT_POWER_PERCENT 65
#define PUSH_FORWARD_MS 900
#define PUSH_PAUSE_MS 500
#define PUSH_RETURN_MS 930

/* 接线情况2。直行仅驱动两前轮；后轮停止。 */
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
#define LEFT_ELECTRICAL_SIGN (-1)
#define RIGHT_ELECTRICAL_SIGN 1
#define LEFT_FORWARD_SIGN 1
#define RIGHT_FORWARD_SIGN (-1)

static SemaphoreHandle_t s_frame_ready;
static SemaphoreHandle_t s_frame_mutex;
static uint8_t *s_jpeg_buffer;
static uint8_t *s_decode_buffer;
static uint8_t s_jpeg_work[JPEG_WORK_BYTES];
static volatile size_t s_jpeg_size;
static volatile uint32_t s_input_width;
static volatile uint32_t s_input_height;

static volatile uint16_t s_red_pixels;
static volatile int16_t s_red_center_x = RED_MAP_WIDTH / 2;
static volatile uint32_t s_red_sequence;

static uint8_t s_device_addr;
static uint8_t s_stream_index;
static volatile bool s_device_connected;
static volatile bool s_stream_task_started;
static int s_profile_count;
static uvc_host_stream_format_t s_profiles[MAX_PROFILES];

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

static void update_red_measurement(const uint8_t *rgb565, size_t stride,
                                   uint32_t width, uint32_t height)
{
    uint16_t count = 0;
    uint32_t x_sum = 0;
    for (int y = 0; y < RED_MAP_HEIGHT; ++y) {
        const uint32_t source_y = (uint32_t)y * height / RED_MAP_HEIGHT;
        const uint8_t *row = rgb565 + (size_t)source_y * stride;
        for (int x = 0; x < RED_MAP_WIDTH; ++x) {
            const uint32_t source_x = (uint32_t)x * width / RED_MAP_WIDTH;
            const uint8_t *source = row + source_x * 2U;
            const uint16_t value = (uint16_t)(source[0] | ((uint16_t)source[1] << 8));
            const int red = ((value >> 11) & 0x1fU) * 255 / 31;
            const int green = ((value >> 5) & 0x3fU) * 255 / 63;
            const int blue = (value & 0x1fU) * 255 / 31;
            const int redness = red - (green + blue) / 2;
            if (red > RED_MIN_CHANNEL && redness > REDNESS_THRESHOLD) {
                const int logical_x = CAMERA_ROTATE_180 ?
                                      RED_MAP_WIDTH - 1 - x : x;
                ++count;
                x_sum += (uint32_t)logical_x;
            }
        }
    }
    s_red_pixels = count;
    if (count > 0) s_red_center_x = (int16_t)(x_sum / count);
    ++s_red_sequence;
}

static uint32_t duty_from_percent(int percent)
{
    if (percent < 0) percent = -percent;
    if (percent > 100) percent = 100;
    return ((1U << LEDC_TIMER_10_BIT) - 1U) * (uint32_t)percent / 100U;
}

static void motor_write(gpio_num_t in1, gpio_num_t in2, ledc_channel_t channel,
                        int electrical_sign, int command_percent)
{
    if (command_percent == 0) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
        ESP_ERROR_CHECK(gpio_set_level(in1, 0));
        ESP_ERROR_CHECK(gpio_set_level(in2, 0));
        return;
    }
    const int direction = (command_percent > 0 ? 1 : -1) * electrical_sign;
    ESP_ERROR_CHECK(gpio_set_level(in1, direction > 0));
    ESP_ERROR_CHECK(gpio_set_level(in2, direction < 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty_from_percent(command_percent)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static void chassis_stop(void)
{
    motor_write(LEFT_IN1, LEFT_IN2, LEDC_CHANNEL_0, LEFT_ELECTRICAL_SIGN, 0);
    motor_write(BACK_IN1, BACK_IN2, LEDC_CHANNEL_1, -1, 0);
    motor_write(RIGHT_IN1, RIGHT_IN2, LEDC_CHANNEL_2, RIGHT_ELECTRICAL_SIGN, 0);
}

static void motor_brake(gpio_num_t in1, gpio_num_t in2, ledc_channel_t channel)
{
    ESP_ERROR_CHECK(gpio_set_level(in1, 1));
    ESP_ERROR_CHECK(gpio_set_level(in2, 1));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel,
                                  (1U << LEDC_TIMER_10_BIT) - 1U));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

/* Short brake after each turn pulse suppresses mechanical coasting. */
static void chassis_brake(void)
{
    motor_brake(LEFT_IN1, LEFT_IN2, LEDC_CHANNEL_0);
    motor_brake(BACK_IN1, BACK_IN2, LEDC_CHANNEL_1);
    motor_brake(RIGHT_IN1, RIGHT_IN2, LEDC_CHANNEL_2);
}

static void chassis_straight(bool forward)
{
    const int sign = forward ? 1 : -1;
    motor_write(BACK_IN1, BACK_IN2, LEDC_CHANNEL_1, -1, 0);
    motor_write(LEFT_IN1, LEFT_IN2, LEDC_CHANNEL_0, LEFT_ELECTRICAL_SIGN,
                sign * LEFT_FORWARD_SIGN * LEFT_STRAIGHT_POWER_PERCENT);
    motor_write(RIGHT_IN1, RIGHT_IN2, LEDC_CHANNEL_2, RIGHT_ELECTRICAL_SIGN,
                sign * RIGHT_FORWARD_SIGN * RIGHT_STRAIGHT_POWER_PERCENT);
}

/* The project-wide motor convention is: a positive command on all three
 * wheels rotates the chassis clockwise.  Therefore a negative command is a
 * left/counter-clockwise rotation. */
static void chassis_rotate(int clockwise_percent)
{
    motor_write(LEFT_IN1, LEFT_IN2, LEDC_CHANNEL_0, LEFT_ELECTRICAL_SIGN,
                clockwise_percent);
    motor_write(BACK_IN1, BACK_IN2, LEDC_CHANNEL_1, -1,
                clockwise_percent);
    motor_write(RIGHT_IN1, RIGHT_IN2, LEDC_CHANNEL_2, RIGHT_ELECTRICAL_SIGN,
                clockwise_percent);
}

static void configure_motor_hardware(void)
{
    const gpio_config_t output = {
        .pin_bit_mask = (1ULL << LEFT_IN1) | (1ULL << LEFT_IN2) |
                        (1ULL << BACK_IN1) | (1ULL << BACK_IN2) |
                        (1ULL << RIGHT_IN1) | (1ULL << RIGHT_IN2) | (1ULL << MOTOR_STBY),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&output));
    const gpio_config_t button = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button));
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    const gpio_num_t pwm_pins[] = {LEFT_PWM, BACK_PWM, RIGHT_PWM};
    for (int i = 0; i < 3; ++i) {
        const ledc_channel_config_t channel = {
            .gpio_num = pwm_pins[i], .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i, .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }
    chassis_stop();
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY, 1));
}

static void usb_host_task(void *argument)
{
    (void)argument;
    while (true) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static void stream_event_callback(const uvc_host_stream_event_data_t *event, void *user_context)
{
    (void)user_context;
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG, "USB transfer error: %s",
                 esp_err_to_name(event->transfer_error.error));
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "camera disconnected");
        s_device_connected = false;
        uvc_host_stream_close(event->device_disconnected.stream_hdl);
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "camera frame buffer overflow");
        break;
    default:
        break;
    }
}

static bool camera_frame_callback(const uvc_host_frame_t *frame, void *user_context)
{
    (void)user_context;
    if (frame->data_len == 0 || frame->data_len > MAX_JPEG_BYTES) {
        ESP_LOGW(TAG, "dropped frame with invalid length: %u", (unsigned)frame->data_len);
        return true;
    }
    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return true; /* Decoder is busy: keep USB host responsive by dropping this frame. */
    }
    memcpy(s_jpeg_buffer, frame->data, frame->data_len);
    s_jpeg_size = frame->data_len;
    s_input_width = frame->vs_format.h_res;
    s_input_height = frame->vs_format.v_res;
    xSemaphoreGive(s_frame_mutex);
    xSemaphoreGive(s_frame_ready); /* Counting semaphore depth is one: newest frame wins. */
    return true;
}

static esp_jpeg_image_scale_t choose_decode_scale(uint32_t source_width, uint32_t source_height)
{
    if (source_width <= DECODE_MAX_W && source_height <= DECODE_MAX_H) {
        return JPEG_IMAGE_SCALE_0;
    }
    if (source_width / 2U <= DECODE_MAX_W && source_height / 2U <= DECODE_MAX_H) {
        return JPEG_IMAGE_SCALE_1_2;
    }
    if (source_width / 4U <= DECODE_MAX_W && source_height / 4U <= DECODE_MAX_H) {
        return JPEG_IMAGE_SCALE_1_4;
    }
    return JPEG_IMAGE_SCALE_1_8;
}

static void camera_process_task(void *argument)
{
    (void)argument;
    int64_t last_decode_us = 0;
    int64_t last_report_us = 0;
    uint32_t processed_frames = 0;
    uint32_t last_report_frames = 0;

    while (true) {
        if (xSemaphoreTake(s_frame_ready, portMAX_DELAY) != pdTRUE) continue;

        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_decode_us < RED_PROCESS_PERIOD_US) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        last_decode_us = now_us;

        if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(DECODE_TASK_YIELD_MS));
            continue;
        }
        const size_t jpeg_size = s_jpeg_size;
        const uint32_t input_width = s_input_width;
        const uint32_t input_height = s_input_height;
        (void)input_width;
        (void)input_height;

        esp_jpeg_image_cfg_t config = {
            .indata = s_jpeg_buffer,
            .indata_size = (uint32_t)jpeg_size,
            .outbuf = s_decode_buffer,
            .outbuf_size = DECODE_BUFFER_BYTES,
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            .out_scale = JPEG_IMAGE_SCALE_1_4,
            .flags = { .swap_color_bytes = 0 },
            .advanced = {
                .working_buffer = s_jpeg_work,
                .working_buffer_size = sizeof(s_jpeg_work),
            },
        };

        esp_jpeg_image_output_t info;
        esp_err_t error = esp_jpeg_get_image_info(&config, &info);
        if (error == ESP_OK) {
            config.out_scale = choose_decode_scale(info.width, info.height);
        }
        esp_jpeg_image_output_t output;
        if (error == ESP_OK) {
            error = esp_jpeg_decode(&config, &output);
        }
        xSemaphoreGive(s_frame_mutex);

        if (error != ESP_OK || output.width == 0 || output.height == 0) {
            ESP_LOGW(TAG, "MJPEG decode failed: %s (%u bytes)",
                     esp_err_to_name(error), (unsigned)jpeg_size);
            /* A malformed MJPEG frame is expected occasionally on a USB
             * camera. Do not immediately start another expensive decode. */
            vTaskDelay(pdMS_TO_TICKS(DECODE_TASK_YIELD_MS));
            continue;
        }

        const size_t stride = output.output_len / output.height;
        update_red_measurement(s_decode_buffer, stride, output.width, output.height);
        ++processed_frames;

        const int64_t report_us = esp_timer_get_time();
        if (last_report_us == 0 || report_us - last_report_us >= 1000000) {
            const float fps = last_report_us == 0 ? 0.0f :
                (float)(processed_frames - last_report_frames) * 1000000.0f /
                (float)(report_us - last_report_us);
            last_report_us = report_us;
            last_report_frames = processed_frames;
            ESP_LOGI(TAG, "image %ux%u -> red map %dx%d at %.1f fps, red=%u x=%d",
                     (unsigned)output.width, (unsigned)output.height,
                     RED_MAP_WIDTH, RED_MAP_HEIGHT, (double)fps,
                     s_red_pixels, s_red_center_x);
        }

        /* esp_jpeg_decode() is CPU-intensive. This real sleep (rather than
         * taskYIELD) guarantees that the Idle task runs and prevents the
         * Task Watchdog warning observed during continuous preview. */
        vTaskDelay(pdMS_TO_TICKS(DECODE_TASK_YIELD_MS));
    }
}

static bool profile_is_listed(const uvc_host_frame_info_t *format, float fps)
{
    for (int i = 0; i < s_profile_count; ++i) {
        if (s_profiles[i].format == format->format &&
            s_profiles[i].h_res == format->h_res &&
            s_profiles[i].v_res == format->v_res &&
            s_profiles[i].fps == fps) {
            return true;
        }
    }
    return false;
}

static void add_profile(const uvc_host_frame_info_t *format, float fps)
{
    if (s_profile_count >= MAX_PROFILES || profile_is_listed(format, fps)) return;
    s_profiles[s_profile_count++] = (uvc_host_stream_format_t) {
        .h_res = format->h_res,
        .v_res = format->v_res,
        .fps = fps,
        .format = format->format,
    };
}

static void build_mjpeg_profiles(const uvc_host_frame_info_t *formats, size_t count)
{
    static const struct { uint16_t width; uint16_t height; } preferred[] = {
        /* Red-ball detection only samples 80x60 points. Starting with 720p
         * makes JPEG decoding the bottleneck, so use the smallest common
         * MJPEG mode first. */
        {320, 240}, {640, 480}, {800, 480}, {960, 540}, {1280, 720},
    };
    s_profile_count = 0;

    for (size_t preference = 0; preference < sizeof(preferred) / sizeof(preferred[0]); ++preference) {
        for (size_t index = 0; index < count && s_profile_count < MAX_PROFILES; ++index) {
            if (formats[index].format != UVC_VS_FORMAT_MJPEG ||
                formats[index].h_res != preferred[preference].width ||
                formats[index].v_res != preferred[preference].height) {
                continue;
            }
            add_profile(&formats[index], CAMERA_REQUEST_FPS);
            add_profile(&formats[index], 0); /* Fallback if 10 FPS is unsupported. */
        }
    }
    for (size_t index = 0; index < count && s_profile_count < MAX_PROFILES; ++index) {
        if (formats[index].format != UVC_VS_FORMAT_MJPEG) continue;
        add_profile(&formats[index], CAMERA_REQUEST_FPS);
        add_profile(&formats[index], 0);
    }
}

static void uvc_stream_task(void *argument)
{
    (void)argument;
    int profile_index = 0;
    while (true) {
        if (s_profile_count == 0) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        uvc_host_stream_config_t config = { 0 };
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

        ESP_LOGI(TAG, "trying %s %ux%u@%.1f", format_name(config.vs_format.format),
                 (unsigned)config.vs_format.h_res, (unsigned)config.vs_format.v_res,
                 (double)config.vs_format.fps);
        uvc_host_stream_hdl_t stream = NULL;
        esp_err_t error = uvc_host_stream_open(&config, pdMS_TO_TICKS(5000), &stream);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "stream open failed: %s", esp_err_to_name(error));
            profile_index = (profile_index + 1) % s_profile_count;
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        s_device_connected = true;
        error = uvc_host_stream_start(stream);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "stream start failed: %s", esp_err_to_name(error));
            uvc_host_stream_close(stream);
            profile_index = (profile_index + 1) % s_profile_count;
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        ESP_LOGI(TAG, "STREAMING_STARTED: offline red detection active");
        while (s_device_connected) vTaskDelay(pdMS_TO_TICKS(1000));
        profile_index = 0;
    }
}

static void uvc_driver_event_callback(const uvc_host_driver_event_data_t *event, void *user_context)
{
    (void)user_context;
    if (event->type != UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) return;

    const uint8_t device_addr = event->device_connected.dev_addr;
    const uint8_t stream_index = event->device_connected.uvc_stream_index;
    size_t format_count = 0;
    esp_err_t error = uvc_host_get_frame_list(device_addr, stream_index, NULL, &format_count);
    if (error != ESP_OK || format_count == 0) {
        ESP_LOGE(TAG, "No UVC frame formats parsed: %s (%u). Check descriptor log.",
                 esp_err_to_name(error), (unsigned)format_count);
        return;
    }

    uvc_host_frame_info_t *formats = calloc(format_count, sizeof(*formats));
    if (formats == NULL) {
        ESP_LOGE(TAG, "unable to allocate UVC format list");
        return;
    }
    error = uvc_host_get_frame_list(device_addr, stream_index,
                                    (uvc_host_frame_info_t (*)[])formats, &format_count);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "get frame list failed: %s", esp_err_to_name(error));
        free(formats);
        return;
    }

    ESP_LOGI(TAG, "UVC camera connected: address=%u stream=%u", device_addr, stream_index);
    for (size_t i = 0; i < format_count; ++i) {
        ESP_LOGI(TAG, "format[%u] %s %ux%u", (unsigned)i,
                 format_name(formats[i].format), (unsigned)formats[i].h_res,
                 (unsigned)formats[i].v_res);
    }
    s_device_addr = device_addr;
    s_stream_index = stream_index;
    build_mjpeg_profiles(formats, format_count);
    free(formats);

    if (s_profile_count == 0) {
        ESP_LOGE(TAG, "camera exposes no MJPEG mode; this diagnostic handles MJPEG only");
        return;
    }
    if (!s_stream_task_started) {
        s_stream_task_started = true;
        assert(xTaskCreate(uvc_stream_task, "uvc_stream", 4096, NULL, 10, NULL) == pdPASS);
    }
}

typedef enum {
    RED_IDLE,
    RED_SEARCH,
    RED_ALIGN,
    RED_PUSH,
    RED_PAUSE,
    RED_RETURN,
    RED_COMPLETE,
} red_state_t;

static bool boot_pressed(void)
{
    static int raw_previous = 1;
    static int stable_level = 1;
    static int64_t raw_changed_us = 0;
    const int64_t now_us = esp_timer_get_time();
    const int raw_level = gpio_get_level(BUTTON_GPIO);

    if (raw_level != raw_previous) {
        raw_previous = raw_level;
        raw_changed_us = now_us;
    }
    if (raw_level != stable_level && now_us - raw_changed_us >= 40000) {
        stable_level = raw_level;
        return stable_level == 0;
    }
    return false;
}

static void red_ball_control_task(void *argument)
{
    (void)argument;
    red_state_t state = RED_IDLE;
    uint32_t seen_sequence = 0;
    int detect_confirmations = 0;
    int center_confirmations = 0;
    int lost_frames = 0;
    int align_step_ms = ALIGN_INITIAL_STEP_MS;
    int previous_error_side = 0;
    bool search_should_step = false;
    bool turn_step_active = false;
    bool turn_brake_active = false;
    int64_t turn_step_ends_ms = 0;
    int64_t turn_brake_ends_ms = 0;
    int64_t settle_until_ms = 0;
    int64_t state_started_ms = 0;
    while (true) {
        const int64_t now_ms = esp_timer_get_time() / 1000;

        if (turn_step_active && now_ms >= turn_step_ends_ms) {
            chassis_brake();
            turn_step_active = false;
            turn_brake_active = true;
            turn_brake_ends_ms = now_ms + TURN_BRAKE_MS;
        }
        if (turn_brake_active && now_ms >= turn_brake_ends_ms) {
            chassis_stop();
            turn_brake_active = false;
            settle_until_ms = now_ms + TURN_SETTLE_MS;
        }

        if (boot_pressed()) {
            if (state == RED_IDLE || state == RED_COMPLETE) {
                state = RED_SEARCH;
                detect_confirmations = 0;
                center_confirmations = 0;
                lost_frames = 0;
                align_step_ms = ALIGN_INITIAL_STEP_MS;
                previous_error_side = 0;
                search_should_step = true;
                turn_step_active = false;
                turn_brake_active = false;
                settle_until_ms = now_ms;
                chassis_stop();
                ESP_LOGI(TAG, "BOOT: searching red ball with %d ms left steps",
                         SEARCH_STEP_MS);
            } else {
                state = RED_IDLE;
                turn_step_active = false;
                turn_brake_active = false;
                search_should_step = false;
                chassis_stop();
                ESP_LOGI(TAG, "BOOT: stopped");
            }
        }

        if ((state == RED_SEARCH || state == RED_ALIGN) &&
            !turn_step_active && !turn_brake_active &&
            now_ms >= settle_until_ms &&
            s_red_sequence != seen_sequence) {
            seen_sequence = s_red_sequence;
            const bool red_detected = s_red_pixels >= RED_PIXELS_TO_TRIGGER;

            if (state == RED_SEARCH) {
                detect_confirmations = red_detected ? detect_confirmations + 1 : 0;
                search_should_step = !red_detected;
                if (detect_confirmations >= RED_CONFIRM_FRAMES) {
                    state = RED_ALIGN;
                    center_confirmations = 0;
                    lost_frames = 0;
                    align_step_ms = ALIGN_INITIAL_STEP_MS;
                    previous_error_side = 0;
                    search_should_step = false;
                    ESP_LOGI(TAG, "red detected: pixels=%u center_x=%d; aligning",
                             s_red_pixels, s_red_center_x);
                }
            }

            if (state == RED_ALIGN) {
                if (!red_detected) {
                    ++lost_frames;
                    center_confirmations = 0;
                    if (lost_frames >= ALIGN_LOST_FRAME_LIMIT) {
                        state = RED_SEARCH;
                        detect_confirmations = 0;
                        align_step_ms = ALIGN_INITIAL_STEP_MS;
                        previous_error_side = 0;
                        search_should_step = true;
                        ESP_LOGW(TAG, "red lost while aligning; resume stepped search");
                    }
                } else {
                    lost_frames = 0;
                    const int error = s_red_center_x - RED_MAP_WIDTH / 2;
                    const int distance = abs(error);
                    if (distance <= CENTER_DEADBAND_PX) {
                        chassis_stop();
                        ++center_confirmations;
                        if (center_confirmations >= CENTER_CONFIRM_FRAMES) {
                            state = RED_PUSH;
                            state_started_ms = now_ms;
                            previous_error_side = 0;
                            ESP_LOGI(TAG,
                                     "red centered: x=%d pixels=%u; push forward",
                                     s_red_center_x, s_red_pixels);
                        }
                    } else {
                        center_confirmations = 0;
                        const int error_side = error < 0 ? -1 : 1;
                        if (previous_error_side != 0 &&
                            error_side != previous_error_side) {
                            align_step_ms /= 2;
                            if (align_step_ms < ALIGN_MIN_STEP_MS) {
                                align_step_ms = ALIGN_MIN_STEP_MS;
                            }
                            ESP_LOGI(TAG,
                                     "center crossed; reverse with half step=%d ms",
                                     align_step_ms);
                        }
                        previous_error_side = error_side;
                        chassis_rotate(error_side * TURN_STEP_POWER);
                        turn_step_active = true;
                        turn_step_ends_ms = now_ms + align_step_ms;
                        ESP_LOGI(TAG, "align step: error=%d direction=%s duration=%d ms",
                                 error, error_side < 0 ? "left" : "right",
                                 align_step_ms);
                    }
                }
            }
        }

        if (state == RED_SEARCH && search_should_step &&
            !turn_step_active && !turn_brake_active &&
            now_ms >= settle_until_ms) {
            chassis_rotate(-TURN_STEP_POWER);
            turn_step_active = true;
            turn_step_ends_ms = now_ms + SEARCH_STEP_MS;
            search_should_step = false;
        } else if (state == RED_PUSH) {
            chassis_straight(true);
            if (now_ms - state_started_ms >= PUSH_FORWARD_MS) {
                chassis_stop();
                state = RED_PAUSE;
                state_started_ms = now_ms;
                ESP_LOGI(TAG, "forward complete; pause");
            }
        } else if (state == RED_PAUSE) {
            chassis_stop();
            if (now_ms - state_started_ms >= PUSH_PAUSE_MS) {
                state = RED_RETURN;
                state_started_ms = now_ms;
                ESP_LOGI(TAG, "pause complete; return backward");
            }
        } else if (state == RED_RETURN) {
            chassis_straight(false);
            if (now_ms - state_started_ms >= PUSH_RETURN_MS) {
                chassis_stop();
                state = RED_COMPLETE;
                ESP_LOGI(TAG, "return complete");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static bool allocate_buffers(void)
{
    s_jpeg_buffer = heap_caps_malloc(MAX_JPEG_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_decode_buffer = heap_caps_malloc(DECODE_BUFFER_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
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
    if (s_frame_ready == NULL || s_frame_mutex == NULL || !allocate_buffers()) {
        ESP_LOGE(TAG, "initialization failed");
        return;
    }
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
    configure_motor_hardware();
    assert(xTaskCreate(red_ball_control_task, "red_ball_control", 4096, NULL, 5, NULL) == pdPASS);
    ESP_LOGI(TAG, "camera=%s D-=GPIO%d D+=GPIO%d; offline 10 FPS red detection",
             CAMERA_MODEL_NAME, USB_D_MINUS_GPIO, USB_D_PLUS_GPIO);
    ESP_LOGI(TAG, "waiting for UVC camera; no computer viewer is required");
    ESP_LOGI(TAG, "red control ready: press BOOT to start");
}
