/*
 * USB UVC 摄像头取流（移植自 camera_uvc_test 工程，去掉软件解码）：
 * 帧回调把最新一帧 JPEG 原样拷进 PSRAM 缓冲，HTTP /stream 直接转发给浏览器。
 * 摄像头：JQ-CAM12，D-=GPIO19 / D+=GPIO20，5V+GND
 */
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "usb/usb_helpers.h"
#include "audio_uac.h"
#include "uvc_stream.h"

static const char *TAG = "uvc_stream";

#define MAX_JPEG_BYTES   (256 * 1024)
#define UVC_NUM_FRAME_BUFFERS 3
#define UVC_NUM_URBS     4
#define UVC_URB_SIZE     (8 * 1024)
#define MAX_PROFILES     8
#define CAMERA_REQUEST_FPS 15.0f

static SemaphoreHandle_t s_frame_ready;
static SemaphoreHandle_t s_frame_mutex;
static uint8_t *s_jpeg_buf;
static volatile size_t s_frame_len;
static volatile uint32_t s_frame_w;
static volatile uint32_t s_frame_h;

static uint8_t  s_dev_addr;
static uint8_t  s_stream_index;
static volatile bool s_device_connected;
static volatile bool s_stream_task_started;
static volatile bool s_pause_requested;
static int  s_profile_count;
static uvc_host_stream_format_t s_profiles[MAX_PROFILES];

/* 只读 USB 客户端：打印完整配置描述符，判断设备里是否有 Audio 接口 */
static usb_host_client_handle_t s_desc_client;
static volatile uint8_t s_desc_new_addr;

static void desc_event_cb(const usb_host_client_event_msg_t *ev, void *arg)
{
    (void)arg;
    if (ev->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        s_desc_new_addr = ev->new_dev.address;
    }
}

static void desc_dump_task(void *arg)
{
    (void)arg;
    while (1) {
        usb_host_client_handle_events(s_desc_client, portMAX_DELAY);
        uint8_t addr = s_desc_new_addr;
        if (addr == 0) {
            continue;
        }
        s_desc_new_addr = 0;
        vTaskDelay(pdMS_TO_TICKS(500));   /* 等 UVC 驱动先完成枚举 */
        usb_device_handle_t hdl = NULL;
        if (usb_host_device_open(s_desc_client, addr, &hdl) != ESP_OK) {
            continue;
        }
        const usb_config_desc_t *cfg = NULL;
        if (usb_host_get_active_config_descriptor(hdl, &cfg) == ESP_OK && cfg) {
            ESP_LOGI(TAG, "--- USB interface summary: total=%d ---",
                     (int)cfg->bNumInterfaces);
            for (int ifn = 0; ifn < (int)cfg->bNumInterfaces; ifn++) {
                int nalt = usb_parse_interface_number_of_alternate(cfg, ifn);
                for (int alt = 0; alt < nalt; alt++) {
                    int off = 0;
                    const usb_intf_desc_t *d =
                        usb_parse_interface_descriptor(cfg, ifn, alt, &off);
                    if (d == NULL) {
                        break;
                    }
                    ESP_LOGI(TAG,
                             "IF%d alt%d class=0x%02X sub=0x%02X proto=0x%02X eps=%d",
                             ifn, alt,
                             d->bInterfaceClass, d->bInterfaceSubClass,
                             d->bInterfaceProtocol, (int)d->bNumEndpoints);
                    /* 打印接口与端点之间的类专用描述符（音频格式/采样率） */
                    {
                        const uint8_t *q = (const uint8_t *)d + d->bLength;
                        const uint8_t *end = (const uint8_t *)cfg + cfg->wTotalLength;
                        while (q + 2 <= end && q[1] != 0x05 && q[0] >= 2) {
                            if (q[1] == 0x24) {
                                char hex[160];
                                int hn = 0;
                                for (int i = 0; i < (int)q[0] && i < 60 && hn < 150; i++) {
                                    hn += snprintf(hex + hn, sizeof(hex) - hn, "%02X ", q[i]);
                                }
                                ESP_LOGI(TAG, "  classdesc len=%u sub=0x%02X: %s",
                                         (unsigned)q[0], q[2], hex);
                            }
                            q += q[0];
                        }
                    }
                    const uint8_t *p = (const uint8_t *)d + d->bLength;
                    const uint8_t *end = (const uint8_t *)cfg + cfg->wTotalLength;
                    for (int e = 0; e < (int)d->bNumEndpoints && p + 2 <= end;) {
                        uint8_t len = p[0];
                        uint8_t dtype = p[1];
                        if (dtype == 0x05 && len >= 7 && p + len <= end) {
                            uint8_t addr = p[2];
                            uint8_t attr = p[3];
                            uint16_t mps = (uint16_t)(p[4] | (p[5] << 8));
                            ESP_LOGI(TAG, "  ep addr=0x%02X %s attr=0x%02X mps=%u",
                                     addr, (addr & 0x80) ? "IN" : "OUT",
                                     attr, (unsigned)mps);
                            e++;
                        }
                        p += len;
                    }
                }
            }
        }
        usb_host_device_close(s_desc_client, hdl);
    }
}

static void start_desc_dump_client(void)
{
    const usb_host_client_config_t ccfg = {
        .is_synchronous = false,
        .max_num_event_msg = 3,
        .async = {
            .client_event_callback = desc_event_cb,
            .callback_arg = NULL,
        },
    };
    if (usb_host_client_register(&ccfg, &s_desc_client) != ESP_OK) {
        ESP_LOGW(TAG, "desc dump client register failed");
        return;
    }
    xTaskCreate(desc_dump_task, "usb_desc", 4096, NULL, 5, NULL);
}

static const char *FORMAT_STR[] = {
    "UNDEFINED", "MJPEG", "YUY2", "H264", "H265", "NV12"
};

static void usb_lib_task(void *arg)
{
    (void)arg;
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: all devices freed");
        }
    }
}

static void stream_event_cb(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG, "USB transfer error: %s",
                 esp_err_to_name(event->transfer_error.error));
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "UVC device disconnected");
        s_device_connected = false;
        uvc_host_stream_close(event->device_disconnected.stream_hdl);
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "Frame buffer overflow (frame too large)");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "Frame buffer underflow");
        break;
    default:
        break;
    }
}

static bool frame_cb(const uvc_host_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    static uint32_t s_log_cnt = 0;
    if (frame->data_len == 0 || frame->data_len > MAX_JPEG_BYTES) {
        ESP_LOGW(TAG, "frame len %u out of range", (unsigned)frame->data_len);
        return true;
    }
    if ((++s_log_cnt % 25) == 1) {
        ESP_LOGI(TAG, "frame cb #%u len=%u %ux%u",
                 (unsigned)s_log_cnt, (unsigned)frame->data_len,
                 (unsigned)frame->vs_format.h_res,
                 (unsigned)frame->vs_format.v_res);
    }
    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return true;   /* 正在被 HTTP 发送，本帧丢弃 */
    }
    memcpy(s_jpeg_buf, frame->data, frame->data_len);
    s_frame_len = frame->data_len;
    s_frame_w = frame->vs_format.h_res;
    s_frame_h = frame->vs_format.v_res;
    xSemaphoreGive(s_frame_mutex);
    xSemaphoreGive(s_frame_ready);
    return true;
}

bool uvc_stream_wait_frame(uint32_t timeout_ms)
{
    return xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

bool uvc_stream_lock_frame(const uint8_t **data, size_t *len)
{
    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return false;
    }
    *data = s_jpeg_buf;
    *len = s_frame_len;
    return true;
}

void uvc_stream_unlock_frame(void)
{
    xSemaphoreGive(s_frame_mutex);
}

void uvc_stream_set_paused(bool paused)
{
    if (s_pause_requested == paused) {
        return;
    }
    s_pause_requested = paused;

    if (paused) {
        /* 丢弃暂停前的最后一帧，避免浏览器误以为摄像头仍在更新。 */
        if (s_frame_mutex != NULL &&
            xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_frame_len = 0;
            xSemaphoreGive(s_frame_mutex);
        }
        if (s_frame_ready != NULL) {
            while (xSemaphoreTake(s_frame_ready, 0) == pdTRUE) {
            }
        }
    }
    ESP_LOGI(TAG, "camera %s requested", paused ? "pause" : "resume");
}

bool uvc_stream_is_paused(void)
{
    return s_pause_requested;
}

static void build_profiles(const uvc_host_frame_info_t *list, size_t n)
{
    static const struct { int w; int h; } pref[] = {
        { 480, 320 }, { 640, 480 }, { 320, 240 },
        { 800, 480 }, { 1280, 720 }, { 960, 540 },
    };
    bool used[MAX_PROFILES] = { false };
    s_profile_count = 0;

    for (int pi = 0; pi < (int)(sizeof(pref) / sizeof(pref[0])); pi++) {
        for (int fmt_pass = 0; fmt_pass < 2 && s_profile_count < MAX_PROFILES; fmt_pass++) {
            for (size_t i = 0; i < n && s_profile_count < MAX_PROFILES; i++) {
                if (used[i]) {
                    continue;
                }
                if (list[i].h_res != (unsigned)pref[pi].w ||
                    list[i].v_res != (unsigned)pref[pi].h) {
                    continue;
                }
                bool is_mjpeg = (list[i].format == UVC_VS_FORMAT_MJPEG);
                if ((fmt_pass == 0) != is_mjpeg) {
                    continue;
                }
                s_profiles[s_profile_count].h_res = list[i].h_res;
                s_profiles[s_profile_count].v_res = list[i].v_res;
                s_profiles[s_profile_count].fps = CAMERA_REQUEST_FPS;
                s_profiles[s_profile_count].format = list[i].format;
                ESP_LOGI(TAG, "profile[%d] = %s %ux%u@%.0f",
                         s_profile_count, FORMAT_STR[list[i].format],
                         list[i].h_res, list[i].v_res,
                         (double)CAMERA_REQUEST_FPS);
                s_profile_count++;
                if (s_profile_count < MAX_PROFILES) {
                    s_profiles[s_profile_count] = s_profiles[s_profile_count - 1];
                    s_profiles[s_profile_count].fps = 0;
                    s_profile_count++;
                }
                used[i] = true;
            }
        }
    }

    for (size_t i = 0; i < n && s_profile_count < MAX_PROFILES; i++) {
        if (used[i]) {
            continue;
        }
        s_profiles[s_profile_count].h_res = list[i].h_res;
        s_profiles[s_profile_count].v_res = list[i].v_res;
        s_profiles[s_profile_count].fps = 0;
        s_profiles[s_profile_count].format = list[i].format;
        ESP_LOGI(TAG, "profile[%d] = %s %ux%u (fallback)",
                 s_profile_count, FORMAT_STR[list[i].format],
                 list[i].h_res, list[i].v_res);
        s_profile_count++;
    }
}

static void stream_task(void *arg)
{
    (void)arg;
    int prof_idx = 0;

    while (1) {
        if (s_profile_count == 0) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        uvc_host_stream_config_t cfg = { 0 };
        cfg.event_cb = stream_event_cb;
        cfg.frame_cb = frame_cb;
        cfg.user_ctx = NULL;
        cfg.usb.dev_addr = s_dev_addr;
        cfg.usb.vid = UVC_HOST_ANY_VID;
        cfg.usb.pid = UVC_HOST_ANY_PID;
        cfg.usb.uvc_stream_index = s_stream_index;
        cfg.vs_format = s_profiles[prof_idx];
        cfg.advanced.number_of_frame_buffers = UVC_NUM_FRAME_BUFFERS;
        cfg.advanced.frame_size = 0;
        cfg.advanced.frame_heap_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
        cfg.advanced.number_of_urbs = UVC_NUM_URBS;
        cfg.advanced.urb_size = UVC_URB_SIZE;

        ESP_LOGI(TAG, "trying stream: %s %ux%u@%.1f",
                 FORMAT_STR[cfg.vs_format.format],
                 cfg.vs_format.h_res, cfg.vs_format.v_res,
                 (double)cfg.vs_format.fps);

        uvc_host_stream_hdl_t h = NULL;
        esp_err_t err = uvc_host_stream_open(&cfg, pdMS_TO_TICKS(5000), &h);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "stream open failed: %s", esp_err_to_name(err));
            prof_idx = (prof_idx + 1) % s_profile_count;
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        s_device_connected = true;
        ESP_LOGI(TAG, "stream opened");
        bool stream_running = false;
        bool start_failed = false;

        while (s_device_connected) {
            if (s_pause_requested) {
                if (stream_running) {
                    err = uvc_host_stream_stop(h);
                    if (err == ESP_OK) {
                        stream_running = false;
                        ESP_LOGI(TAG, "camera paused for piano");
                    } else {
                        ESP_LOGW(TAG, "stream stop failed: %s", esp_err_to_name(err));
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            if (!stream_running) {
                err = uvc_host_stream_start(h);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "stream start failed: %s", esp_err_to_name(err));
                    s_device_connected = false;
                    uvc_host_stream_close(h);
                    start_failed = true;
                    break;
                }
                stream_running = true;
                ESP_LOGI(TAG, "camera streaming");
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (start_failed) {
            prof_idx = (prof_idx + 1) % s_profile_count;
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        prof_idx = 0;
    }
}

static void uvc_driver_event_cb(const uvc_host_driver_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    if (event->type != UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) {
        return;
    }

    uint8_t dev_addr = event->device_connected.dev_addr;
    uint8_t stream_idx = event->device_connected.uvc_stream_index;
    ESP_LOGI(TAG, "UVC device connected: addr=%d stream=%d",
             dev_addr, stream_idx);

    size_t n = 0;
    esp_err_t err = uvc_host_get_frame_list(dev_addr, stream_idx, NULL, &n);
    if (err != ESP_OK || n == 0) {
        ESP_LOGE(TAG, "No UVC frame formats parsed (err=%s n=%d)",
                 esp_err_to_name(err), (int)n);
        return;
    }

    uvc_host_frame_info_t *list = calloc(n, sizeof(uvc_host_frame_info_t));
    assert(list);
    err = uvc_host_get_frame_list(dev_addr, stream_idx,
                                  (uvc_host_frame_info_t (*)[])list, &n);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "get frame list failed: %s", esp_err_to_name(err));
        free(list);
        return;
    }

    for (size_t i = 0; i < n; i++) {
        ESP_LOGI(TAG, "Camera format: %s %ux%u",
                 FORMAT_STR[list[i].format], list[i].h_res, list[i].v_res);
    }

    s_dev_addr = dev_addr;
    s_stream_index = stream_idx;
    build_profiles(list, n);
    free(list);

    if (!s_stream_task_started) {
        s_stream_task_started = true;
        BaseType_t ok = xTaskCreate(stream_task, "uvc_stream", 4096,
                                    NULL, 10, NULL);
        assert(ok == pdTRUE);
    }
}

esp_err_t uvc_stream_init(void)
{
    s_frame_ready = xSemaphoreCreateCounting(1, 0);
    s_frame_mutex = xSemaphoreCreateMutex();
    if (s_frame_ready == NULL || s_frame_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_jpeg_buf = (uint8_t *)heap_caps_malloc(MAX_JPEG_BYTES,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_jpeg_buf == NULL) {
        ESP_LOGE(TAG, "PSRAM JPEG buffer allocation failed");
        return ESP_ERR_NO_MEM;
    }

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install: %s", esp_err_to_name(err));
        return err;
    }

    BaseType_t ok = xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 15, NULL);
    assert(ok == pdTRUE);

    start_desc_dump_client();

    const uvc_host_driver_config_t uvc_config = {
        .driver_task_stack_size = 4 * 1024,
        .driver_task_priority = 16,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = uvc_driver_event_cb,
        .user_ctx = NULL,
    };
    err = uvc_host_install(&uvc_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uvc_host_install: %s", esp_err_to_name(err));
        return err;
    }

    err = audio_uac_install();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "USB speaker driver disabled: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "waiting for USB UVC device ...");
    return ESP_OK;
}
