/*
 * ESP32-S3 camera UART viewer
 *
 * This is intentionally a camera-only diagnostic program.  It does not touch
 * the motors, servos, ultrasonic sensor or line-following code.
 *
 * Data path (adapted for the JQ-CAM12-720D-V1 reference project):
 * UVC MJPEG camera -> usb_host_uvc v2 -> esp_jpeg -> 80x60 grayscale image
 * -> CRC-protected UART packet -> tools/gray_viewer.py -> Mac browser.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

static const char *TAG = "camera_view";

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

/* Sending a full image through UART is not viable.  The camera still captures
 * 720p, but the proven visualization format sends an 80x60 gray preview.  Its
 * 4800-byte frames remain responsive at 921600 baud while preserving focus,
 * framing and black/white contrast information. */
#define STREAM_WIDTH 80
#define STREAM_HEIGHT 60
#define STREAM_PIXELS (STREAM_WIDTH * STREAM_HEIGHT)
#define STREAM_FRAME_BYTES (4U + 2U + 2U + 2U + STREAM_PIXELS + 2U)
#define STREAM_BAUD 921600
/* Match the supplied visualization build's responsive processing cadence.
 * UART transmission still applies natural back-pressure if a frame is late. */
#define FRAME_DECODE_MIN_GAP_US (80U * 1000U)
#define DECODE_TASK_YIELD_MS 10

/* The browser displays a simultaneous binary image with this threshold.  Use
 * m / n in the terminal (or the page buttons) to adjust it by 5. */
#define THRESHOLD_DEFAULT 100
#define THRESHOLD_MIN 10
#define THRESHOLD_MAX 220

/* This matches the normal physical camera mounting on the car. Set to 0 if
 * you deliberately want the unrotated raw orientation. */
#define CAMERA_ROTATE_180 1

#define UVC_FRAME_BUFFERS 3
#define UVC_URB_COUNT 4
#define UVC_URB_SIZE (8U * 1024U)
#define MAX_PROFILES 8

static SemaphoreHandle_t s_frame_ready;
static SemaphoreHandle_t s_frame_mutex;
static uint8_t *s_jpeg_buffer;
static uint8_t *s_decode_buffer;
static uint8_t s_jpeg_work[JPEG_WORK_BYTES];
static volatile size_t s_jpeg_size;
static volatile uint32_t s_input_width;
static volatile uint32_t s_input_height;
static volatile uint32_t s_camera_frame_count;

static uint8_t s_gray[STREAM_PIXELS];
static uint8_t s_serial_frame[STREAM_FRAME_BYTES];
static volatile bool s_stream_enabled;
static volatile int s_threshold = THRESHOLD_DEFAULT;

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

static void send_gray_frame(void)
{
    const size_t payload_offset = 10;
    s_serial_frame[0] = 'G';
    s_serial_frame[1] = 'R';
    s_serial_frame[2] = 'A';
    s_serial_frame[3] = 'Y';
    s_serial_frame[4] = STREAM_WIDTH & 0xff;
    s_serial_frame[5] = STREAM_WIDTH >> 8;
    s_serial_frame[6] = STREAM_HEIGHT & 0xff;
    s_serial_frame[7] = STREAM_HEIGHT >> 8;
    s_serial_frame[8] = s_threshold & 0xff;
    s_serial_frame[9] = s_threshold >> 8;
    memcpy(s_serial_frame + payload_offset, s_gray, STREAM_PIXELS);

    const uint16_t crc = crc16_xmodem(s_gray, STREAM_PIXELS);
    s_serial_frame[payload_offset + STREAM_PIXELS] = crc & 0xff;
    s_serial_frame[payload_offset + STREAM_PIXELS + 1] = crc >> 8;

    /* One write prevents a second task from splitting a binary image packet. */
    uart_write_bytes(UART_NUM_0, s_serial_frame, sizeof(s_serial_frame));
}

static uint8_t rgb565_to_gray(const uint8_t *pixel)
{
    const uint16_t value = (uint16_t)(pixel[0] | ((uint16_t)pixel[1] << 8));
    const uint32_t r = ((value >> 11) & 0x1fU) * 255U / 31U;
    const uint32_t g = ((value >> 5) & 0x3fU) * 255U / 63U;
    const uint32_t b = (value & 0x1fU) * 255U / 31U;
    return (uint8_t)((299U * r + 587U * g + 114U * b) / 1000U);
}

static void make_stream_gray(const uint8_t *rgb565, size_t stride,
                             uint32_t width, uint32_t height)
{
    for (int y = 0; y < STREAM_HEIGHT; ++y) {
        const uint32_t source_y = (uint32_t)y * height / STREAM_HEIGHT;
        const uint8_t *row = rgb565 + (size_t)source_y * stride;
        for (int x = 0; x < STREAM_WIDTH; ++x) {
            const uint32_t source_x = (uint32_t)x * width / STREAM_WIDTH;
            s_gray[y * STREAM_WIDTH + x] = rgb565_to_gray(row + source_x * 2U);
        }
    }

#if CAMERA_ROTATE_180
    for (size_t i = 0; i < STREAM_PIXELS / 2U; ++i) {
        const uint8_t temporary = s_gray[i];
        s_gray[i] = s_gray[STREAM_PIXELS - 1U - i];
        s_gray[STREAM_PIXELS - 1U - i] = temporary;
    }
#endif
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
        make_stream_gray(s_decode_buffer, stride, output.width, output.height);
        if (s_stream_enabled) send_gray_frame();

        if (last_report_us == 0 || now_us - last_report_us >= 1000000) {
            const float fps = last_report_us == 0 ? 0.0f :
                (float)(s_camera_frame_count - last_report_frames) * 1000000.0f /
                (float)(now_us - last_report_us);
            last_report_us = now_us;
            last_report_frames = s_camera_frame_count;
            ESP_LOGI(TAG, "image %ux%u -> gray %dx%d, camera %.1f fps, stream=%s",
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
        {1280, 720}, {960, 540}, {800, 480}, {640, 480}, {320, 240},
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
    printf("Commands: v=start image stream, x=stop, m/n=threshold +/-5\n");
    printf("Run tools/gray_viewer.py on the Mac; it automatically sends v.\n\n");

    while (true) {
        const int character = getchar();
        if (character == EOF || character == '\n' || character == '\r') continue;
        switch ((char)character) {
        case 'v': case 'V':
            s_stream_enabled = true;
            printf("gray stream ON\n");
            break;
        case 'x': case 'X':
            s_stream_enabled = false;
            printf("gray stream OFF\n");
            break;
        case 'm': case 'M':
            s_threshold = s_threshold + 5 > THRESHOLD_MAX ? THRESHOLD_MAX : s_threshold + 5;
            printf("threshold=%d\n", s_threshold);
            break;
        case 'n': case 'N':
            s_threshold = s_threshold - 5 < THRESHOLD_MIN ? THRESHOLD_MIN : s_threshold - 5;
            printf("threshold=%d\n", s_threshold);
            break;
        default:
            printf("commands: v / x / m / n\n");
            break;
        }
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
    ESP_LOGI(TAG, "waiting for UVC camera; run PC viewer after STREAMING_STARTED");
}
