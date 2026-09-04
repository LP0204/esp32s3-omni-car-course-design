/*
 * ESP32-S3 red-ball push controller
 *
 * The UVC camera pipeline below is retained from the verified
 * camera-color-viewer project. Motor control is isolated in its own task.
 *
 * Data path (adapted for the JQ-CAM12-720D-V1 reference project):
 * UVC MJPEG camera -> usb_host_uvc v2 -> esp_jpeg -> 80x60 RGB565 image
 * -> CRC-protected UART packet -> tools/color_viewer.py -> Mac browser.
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

/* The camera captures 720p. An 80x60 RGB565 preview preserves colour while
 * staying within the 921600-baud UART bandwidth. */
#define STREAM_WIDTH 80
#define STREAM_HEIGHT 60
#define STREAM_PIXELS (STREAM_WIDTH * STREAM_HEIGHT)
#define STREAM_COLOR_BYTES (STREAM_PIXELS * 2U)
#define STREAM_FRAME_BYTES (4U + 2U + 2U + STREAM_COLOR_BYTES + 2U)
#define STREAM_BAUD 921600
/* An 80x60 RGB565 frame takes about 105 ms to send at 921600 baud. */
#define FRAME_DECODE_MIN_GAP_US (120U * 1000U)
#define DECODE_TASK_YIELD_MS 10

/* This matches the normal physical camera mounting on the car. Set to 0 if
 * you deliberately want the unrotated raw orientation. */
#define CAMERA_ROTATE_180 1

#define UVC_FRAME_BUFFERS 3
#define UVC_URB_COUNT 4
#define UVC_URB_SIZE (8U * 1024U)
#define MAX_PROFILES 8

/* Red-ball control: deliberately only one colour metric. */
#define DEFAULT_REDNESS_THRESHOLD 71
#define RED_MIN_CHANNEL 90
#define RED_PIXELS_TO_TRIGGER 20
#define RED_CONFIRM_FRAMES 3
#define CENTER_CONFIRM_FRAMES 3
#define CENTER_DEADBAND_PX 3
#define CENTER_FINE_ZONE_PX 10
#define ALIGN_LOST_FRAME_LIMIT 3
#define SEARCH_LEFT_POWER 20
#define ALIGN_COARSE_POWER 23
#define ALIGN_FINE_POWER 18
#define LEFT_STRAIGHT_POWER_PERCENT 52
#define RIGHT_STRAIGHT_POWER_PERCENT 52
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
static volatile uint32_t s_camera_frame_count;

static uint8_t s_color[STREAM_COLOR_BYTES];
static uint8_t s_serial_frame[STREAM_FRAME_BYTES];
static volatile bool s_stream_enabled;
static volatile uint8_t s_redness_threshold = DEFAULT_REDNESS_THRESHOLD;
static volatile uint16_t s_red_pixels;
static volatile int16_t s_red_center_x = STREAM_WIDTH / 2;
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

static void send_color_frame(void)
{
    const size_t payload_offset = 8;
    s_serial_frame[0] = 'R';
    s_serial_frame[1] = 'G';
    s_serial_frame[2] = 'B';
    s_serial_frame[3] = '5';
    s_serial_frame[4] = STREAM_WIDTH & 0xff;
    s_serial_frame[5] = STREAM_WIDTH >> 8;
    s_serial_frame[6] = STREAM_HEIGHT & 0xff;
    s_serial_frame[7] = STREAM_HEIGHT >> 8;
    memcpy(s_serial_frame + payload_offset, s_color, STREAM_COLOR_BYTES);

    const uint16_t crc = crc16_xmodem(s_color, STREAM_COLOR_BYTES);
    s_serial_frame[payload_offset + STREAM_COLOR_BYTES] = crc & 0xff;
    s_serial_frame[payload_offset + STREAM_COLOR_BYTES + 1] = crc >> 8;

    /* One write prevents a second task from splitting a binary image packet. */
    uart_write_bytes(UART_NUM_0, s_serial_frame, sizeof(s_serial_frame));
}

static void make_stream_color(const uint8_t *rgb565, size_t stride,
                              uint32_t width, uint32_t height)
{
    for (int y = 0; y < STREAM_HEIGHT; ++y) {
        const uint32_t source_y = (uint32_t)y * height / STREAM_HEIGHT;
        const uint8_t *row = rgb565 + (size_t)source_y * stride;
        for (int x = 0; x < STREAM_WIDTH; ++x) {
            const uint32_t source_x = (uint32_t)x * width / STREAM_WIDTH;
            const size_t destination = ((size_t)y * STREAM_WIDTH + x) * 2U;
            const uint8_t *source = row + source_x * 2U;
            s_color[destination] = source[0];
            s_color[destination + 1U] = source[1];
        }
    }

#if CAMERA_ROTATE_180
    for (size_t i = 0; i < STREAM_PIXELS / 2U; ++i) {
        const size_t first = i * 2U;
        const size_t second = (STREAM_PIXELS - 1U - i) * 2U;
        const uint8_t low = s_color[first];
        const uint8_t high = s_color[first + 1U];
        s_color[first] = s_color[second];
        s_color[first + 1U] = s_color[second + 1U];
        s_color[second] = low;
        s_color[second + 1U] = high;
    }
#endif
}

static void update_red_measurement(void)
{
    uint16_t count = 0;
    uint32_t x_sum = 0;
    for (size_t i = 0; i < STREAM_PIXELS; ++i) {
        const uint16_t value = (uint16_t)(s_color[i * 2U] | ((uint16_t)s_color[i * 2U + 1U] << 8));
        const int red = ((value >> 11) & 0x1fU) * 255 / 31;
        const int green = ((value >> 5) & 0x3fU) * 255 / 63;
        const int blue = (value & 0x1fU) * 255 / 31;
        const int redness = red - (green + blue) / 2;
        if (red > RED_MIN_CHANNEL && redness > s_redness_threshold) {
            ++count;
            x_sum += (uint32_t)(i % STREAM_WIDTH);
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
    ++s_camera_frame_count;
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
    uint32_t last_report_frames = 0;

    while (true) {
        if (xSemaphoreTake(s_frame_ready, portMAX_DELAY) != pdTRUE) continue;

        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_decode_us < FRAME_DECODE_MIN_GAP_US) {
            vTaskDelay(pdMS_TO_TICKS(10));
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
        make_stream_color(s_decode_buffer, stride, output.width, output.height);
        update_red_measurement();
        if (s_stream_enabled) send_color_frame();

        if (last_report_us == 0 || now_us - last_report_us >= 1000000) {
            const float fps = last_report_us == 0 ? 0.0f :
                (float)(s_camera_frame_count - last_report_frames) * 1000000.0f /
                (float)(now_us - last_report_us);
            last_report_us = now_us;
            last_report_frames = s_camera_frame_count;
            ESP_LOGI(TAG, "image %ux%u -> RGB565 %dx%d, camera %.1f fps, stream=%s",
                     (unsigned)output.width, (unsigned)output.height,
                     STREAM_WIDTH, STREAM_HEIGHT, (double)fps,
                     s_stream_enabled ? "on" : "off");
        }

        /* esp_jpeg_decode() is CPU-intensive. This real sleep (rather than
         * taskYIELD) guarantees that the Idle task runs and prevents the
         * Task Watchdog warning observed during continuous preview. */
        vTaskDelay(pdMS_TO_TICKS(DECODE_TASK_YIELD_MS));
    }
}

static bool profile_is_listed(const uvc_host_frame_info_t *format)
{
    for (int i = 0; i < s_profile_count; ++i) {
        if (s_profiles[i].format == format->format &&
            s_profiles[i].h_res == format->h_res &&
            s_profiles[i].v_res == format->v_res) {
            return true;
        }
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
        /* Red-ball detection only transmits an 80x60 image.  Starting with
         * 720p makes JPEG decoding the bottleneck (about 0.7 FPS), so use
         * the smallest common MJPEG mode first. */
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
            add_profile(&formats[index]);
        }
    }
    for (size_t index = 0; index < count && s_profile_count < MAX_PROFILES; ++index) {
        if (formats[index].format != UVC_VS_FORMAT_MJPEG) continue;
        add_profile(&formats[index]);
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

        ESP_LOGI(TAG, "trying %s %ux%u", format_name(config.vs_format.format),
                 (unsigned)config.vs_format.h_res, (unsigned)config.vs_format.v_res);
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
        ESP_LOGI(TAG, "STREAMING_STARTED (open the PC viewer to send v)");
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
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 32768, 0, NULL, 0));
    }
    uart_vfs_dev_use_driver(UART_NUM_0);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
}

static void serial_command_task(void *argument)
{
    (void)argument;
    printf("\n=== ESP32-S3 Camera Image Viewer ===\n");
    printf("Camera: %s; D-=GPIO%d, D+=GPIO%d; serial=%d baud\n",
           CAMERA_MODEL_NAME, USB_D_MINUS_GPIO, USB_D_PLUS_GPIO, STREAM_BAUD);
    printf("Commands: v=start colour stream, x=stop, r<byte>=red threshold\n");
    printf("Run tools/color_viewer.py on the Mac; it automatically sends v.\n\n");

    while (true) {
        const int character = getchar();
        if (character == EOF || character == '\n' || character == '\r') continue;
        switch ((char)character) {
        case 'v': case 'V':
            s_stream_enabled = true;
            printf("colour stream ON\n");
            break;
        case 'x': case 'X':
            s_stream_enabled = false;
            printf("colour stream OFF\n");
            break;
        case 'r': case 'R': {
            const int threshold = getchar();
            if (threshold != EOF) {
                s_redness_threshold = (uint8_t)threshold;
                printf("red threshold=%u\n", s_redness_threshold);
            }
            break;
        }
        default:
            printf("commands: v / x\n");
            break;
        }
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
    int64_t state_started_ms = 0;
    while (true) {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        if (boot_pressed()) {
            if (state == RED_IDLE || state == RED_COMPLETE) {
                state = RED_SEARCH;
                detect_confirmations = 0;
                center_confirmations = 0;
                lost_frames = 0;
                chassis_rotate(-SEARCH_LEFT_POWER);
                ESP_LOGI(TAG, "BOOT: searching red ball, rotating left at %d%%",
                         SEARCH_LEFT_POWER);
            } else {
                state = RED_IDLE;
                chassis_stop();
                ESP_LOGI(TAG, "BOOT: stopped");
            }
        }

        if ((state == RED_SEARCH || state == RED_ALIGN) &&
            s_red_sequence != seen_sequence) {
            seen_sequence = s_red_sequence;
            const bool red_detected = s_red_pixels >= RED_PIXELS_TO_TRIGGER;

            if (state == RED_SEARCH) {
                detect_confirmations = red_detected ? detect_confirmations + 1 : 0;
                if (detect_confirmations >= RED_CONFIRM_FRAMES) {
                    state = RED_ALIGN;
                    center_confirmations = 0;
                    lost_frames = 0;
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
                        chassis_rotate(-SEARCH_LEFT_POWER);
                        ESP_LOGW(TAG, "red lost while aligning; resume left search");
                    }
                } else {
                    lost_frames = 0;
                    const int error = s_red_center_x - STREAM_WIDTH / 2;
                    const int distance = abs(error);
                    if (distance <= CENTER_DEADBAND_PX) {
                        chassis_stop();
                        ++center_confirmations;
                        if (center_confirmations >= CENTER_CONFIRM_FRAMES) {
                            state = RED_PUSH;
                            state_started_ms = now_ms;
                            ESP_LOGI(TAG,
                                     "red centered: x=%d pixels=%u; push forward",
                                     s_red_center_x, s_red_pixels);
                        }
                    } else {
                        center_confirmations = 0;
                        const int power = distance <= CENTER_FINE_ZONE_PX ?
                                          ALIGN_FINE_POWER : ALIGN_COARSE_POWER;
                        /* Target left of centre -> rotate left.  If it crosses
                         * the centre, error changes sign and the chassis
                         * automatically rotates back to correct overshoot. */
                        chassis_rotate(error < 0 ? -power : power);
                    }
                }
            }
        }

        if (state == RED_SEARCH) {
            chassis_rotate(-SEARCH_LEFT_POWER);
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
    init_serial();
    assert(xTaskCreate(serial_command_task, "serial_command", 4096, NULL, 3, NULL) == pdPASS);
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
    ESP_LOGI(TAG, "waiting for UVC camera; run PC viewer after STREAMING_STARTED");
    ESP_LOGI(TAG, "red control ready: press BOOT to start");
}
