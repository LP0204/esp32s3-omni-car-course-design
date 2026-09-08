/*
 * camera_uvc_test - ESP32-S3 USB UVC 摄像头循线 + 超声避障 + 屏显
 *
 * 链路：
 *   1. USB Host 枚举 JQ-CAM12（usb_host_uvc v2 原生驱动，支持 Bulk 流）
 *   2. MJPEG 取流 -> esp_jpeg 软件解码 -> RGB565 -> 80x60 灰度
 *   3. 黑线中心 -> 4 区分区位图（bit3=最左 ... bit0=最右，1=有黑线）
 *   4. 控制任务（10ms）：16 状态查表 + 弯道降速 + 丢线寻线 +
 *      超声避障状态机 + BOOT 键启停
 *   5. TFT18 每 0.2s 显示三轮速度与超声距离
 *
 * 接线（任务2）：
 *   左前轮: GPIO8/9/10   后轮: GPIO12/13/14   右前轮: GPIO15/16/17
 *   STBY:   GPIO11
 *   USB 摄像头: D-=GPIO19, D+=GPIO20, 5V+GND
 *   HC-SR04: TRIG=GPIO18, ECHO=GPIO21（5V 必须分压到 3.3V）
 *   BOOT 键: GPIO0（烧录后默认不运行，按下启动/停止）
 *   TFT18: SCLK=39 MOSI=40 CS=41 DC=42 RST=47
 *   云台舵机: GPIO4/5（任务2 第二阶段推球用，本阶段未启用）
 *
 * 串口命令（UART0，921600）：g=开始  s=停车  b=打印线位位图  u=测距
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"

#include "driver/uart.h"
#include "driver/uart_vfs.h"

#include "usb/usb_host.h"
#include "usb/uvc_host.h"

#include "jpeg_decoder.h"

#include "motor.h"
#include "ultrasonic.h"
#include "follower.h"
#include "tft.h"
#include "ball_vision.h"
#include "green_vision.h"

static const char *TAG = "uvc_test";

/* ================= 参数 ================= */

#define MAX_JPEG_BYTES        (256 * 1024)   /* MJPEG 帧缓冲（640x480 一般 <100KB） */
#define RGB565_BYTES_PER_PIX  2
#define DEC_OUT_MAX_W         160     /* 1/4 缩放下 640x480 = 160x120，够用 */
#define DEC_OUT_MAX_H         120
#define DEC_OUT_BUF_BYTES     (DEC_OUT_MAX_W * DEC_OUT_MAX_H * RGB565_BYTES_PER_PIX)

#define DETECT_W              160            /* 检测灰度宽（解码 640x480@1/4 正好 160x120） */
#define DETECT_H              120            /* 检测灰度高 */
#define STREAM_W              80             /* 串口灰度流宽（保持 80x60，查看器不掉帧） */
#define STREAM_H              60             /* 串口灰度流高 */
#define LINE_BLACK_THRESHOLD_DEFAULT 110      /* 默认阈值：亮度低于该值视为黑线 */
#define LINE_THRESHOLD_MIN    10
#define LINE_THRESHOLD_MAX    200
#define LINE_CONTRAST_MIN     40             /* 线像素与行背景的最小灰度差（高对比校验） */
/* 关注区域（80x60 坐标）：底边 80、高 45 的等腰三角形，顶点 (40,14)。
 * 检测图 160x120 放大 2 倍：顶点 (80,29)，高 90。三角形内才是真线，
 * 其余区域基本是干扰。 */
#define ROI_APEX_X           (DETECT_W / 2)   /* 160/2 = 80 */
#define ROI_APEX_Y           (DETECT_H - 1 - 90)  /* 120-1-90 = 29 */
#define ROI_HEIGHT           90                /* 45 * 2 */
#define LINE_MIN_RUN_PX       1              /* 每行黑色段最短像素数：细线可能只有 1px */
#define LINE_MIN_ROWS         4              /* 至少多少行找到线才算有线 */
#define PRINT_INTERVAL_MS     200

/* 摄像头镜头装反（画面上下颠倒 180°）时改为 1：
 * 对灰度图旋转 180° 后再检测和发送，小车转向与查看器画面都会恢复正常 */
#define CAMERA_ROTATE_180     1

/* 四路位图镜像：实测车左在图像中落在 CH1=[90,100)（bit3），
 * 与红外查表约定 bit3=车左一致，因此不镜像，位图直接喂查表。 */
#define CAMERA_MIRROR         0

/* UVC 流缓冲配置 */
#define UVC_NUM_FRAME_BUFFERS 3
#define UVC_NUM_URBS          4
#define UVC_URB_SIZE          (8 * 1024)
#define MAX_PROFILES          8

#define CONTROL_PERIOD_MS     10

/* 与 red-ball-push / green-ball-push 一致：优先请求 10fps，
 * 降低 USB 带宽压力，减少坏帧导致的 JPEG 解码风暴 */
#define CAMERA_REQUEST_FPS    25.0f

/* 串口波特率：图像流 80x60=4800B/帧，921600 才能达到 ~10fps 实时显示 */
#define SERIAL_BAUD           115200

/* 帧解码最小间隔：摄像头帧率可能高于软件解码速度，
 * 若不限速 cam_proc 会连续解码占满 CPU，饿死 IDLE 任务触发看门狗。
 * 80ms ≈ 12.5fps，对 10ms 控制周期足够。 */
#define FRAME_DECODE_MIN_GAP_US (80 * 1000)

/* ================= 全局状态 ================= */

static SemaphoreHandle_t  s_frame_ready;      /* 计数信号量，上限 1 */
static SemaphoreHandle_t  s_frame_mutex;

static uint8_t *s_jpeg_buf;                   /* JPEG 输入缓冲（PSRAM） */
static uint8_t *s_dec_buf;                    /* 解码输出缓冲（PSRAM） */
static size_t   s_dec_buf_bytes;
static uint8_t  s_work[4096];                 /* TJpgDec 工作区 */

static volatile size_t   s_frame_size;
static volatile uint32_t s_frame_w;
static volatile uint32_t s_frame_h;
static volatile uint32_t s_fps_count;
static volatile uint8_t  s_cur_bits;          /* 最新线位位图，供控制任务读取 */
static uint8_t  s_detect_gray[DETECT_W * DETECT_H];  /* 160x120 检测灰度 */
static uint8_t  s_stream_gray[STREAM_W * STREAM_H];  /* 80x60 串口灰度流 */
static volatile bool s_gray_stream_enabled;   /* 串口 'v' 开启灰度流 */
static uint8_t  s_near_bits;                  /* 近带位图（诊断/日志） */
static uint8_t  s_far_bits;                   /* 远带位图（诊断/日志） */
static int  s_far_cx = -1;                    /* 远带暗像素质心 x（160 坐标，PID 预告用） */
static volatile uint32_t s_cam_seq = 0;       /* 已处理帧号：控制端按新帧算 D 项 */

/* TFT 显示数据源：返回近带四区原始判定（tft.c 调用） */
uint8_t camera_near_bits(void)
{
    return s_near_bits;
}
uint8_t camera_far_bits(void)
{
    return s_far_bits;
}
int camera_far_cx(void)
{
    return s_far_cx;
}
uint32_t camera_seq(void)
{
    return s_cam_seq;
}
static int  s_line_threshold = LINE_BLACK_THRESHOLD_DEFAULT; /* 可串口实时调节 */

/* 串口灰度帧：magic(4) + w(2) + h(2) + th(2) + near(1) + far(1)
 * + data(4800) + crc16(2) */
static uint8_t s_gray_frame[4 + 2 + 2 + 2 + 2 + STREAM_W * STREAM_H + 2];

/* 驱动事件里保存的设备信息与候选流格式 */
static uint8_t  s_dev_addr;
static uint8_t  s_stream_index;
static volatile bool s_device_connected;
static volatile bool s_stream_task_started;
static int      s_profile_count;
static uvc_host_stream_format_t s_profiles[MAX_PROFILES];

static const char *FORMAT_STR[] = {
    "UNDEFINED", "MJPEG", "YUY2", "H264", "H265", "NV12"
};

/* ================= USB Host 事件任务 ================= */

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

/* ================= 流事件回调 ================= */

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

/* ================= 取帧回调 ================= */

static bool frame_cb(const uvc_host_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;

    if (frame->data_len == 0 || frame->data_len > MAX_JPEG_BYTES) {
        ESP_LOGW(TAG, "frame len %d out of range", (int)frame->data_len);
        return true;
    }
    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return true;   /* 处理任务正在解码，本帧丢弃 */
    }
    memcpy(s_jpeg_buf, frame->data, frame->data_len);
    s_frame_size = frame->data_len;
    s_frame_w = frame->vs_format.h_res;
    s_frame_h = frame->vs_format.v_res;
    s_fps_count++;
    xSemaphoreGive(s_frame_mutex);
    xSemaphoreGive(s_frame_ready);
    return true;
}

/* ================= 黑线检测 + 4 区分区 ================= */

/* 4 路“虚拟红外”（un_poco_bien 版区间，0-based 列）：
 *   近带：bit3=[50,70)  bit2=[70,80)  bit1=[80,90)  bit0=[90,110)
 *   远带：bit3=[30,70)  bit2=[70,80)  bit1=[80,90)  bit0=[90,130)
 * 判黑：灰阶 < 阈值（默认 110，可实时调），区间内出现任意黑像素即置位。 */
#define NEAR_ZONE_EDGES     { 52, 66, 80, 94, 108 }
#define FAR_ZONE_EDGES      { 30, 70, 80, 90, 130 }
#define DETECT_BOTTOM_ROWS   8
#define DETECT_FAR_BAND_ROWS 24  /* 远带：底部 8 行之上再往上看 24 行（锐角弯提前转向） */

/*
 * 指定行带 y0..y0+n_rows-1 × 4 区间，按“阈值二值”判断每个区间“有/没有黑线”：
 * 区间内只要存在灰度 < TH 的像素，该位就置 1（有黑点即判黑）。
 * out_black/out_cx 仅为诊断（暗像素总数 / 暗像素质心）。
 */
static uint8_t zone_bits_from_gray(const uint8_t *gray,
                                   const int *edges,
                                   int y0, int n_rows,
                                   int *out_black, int *out_cx)
{
    int total_black = 0;
    long sum_x = 0;
    int black_px = 0;
    uint8_t bits = 0;

    /* 统计整行带暗像素（诊断用） */
    for (int y = y0; y < y0 + n_rows && y < DETECT_H; y++) {
        const uint8_t *row = gray + (size_t)y * DETECT_W;
        for (int x = 0; x < DETECT_W; x++) {
            if (row[x] < s_line_threshold) {
                total_black++;
                sum_x += x;
                black_px++;
            }
        }
    }

    /* 每个区间：出现任意黑像素即判“有线” */
    for (int z = 0; z < 4; z++) {
        int x0 = edges[z];
        int x1 = edges[z + 1] - 1;
        bool has_black = false;
        for (int y = y0; y < y0 + n_rows && y < DETECT_H && !has_black; y++) {
            const uint8_t *row = gray + (size_t)y * DETECT_W;
            for (int x = x0; x <= x1; x++) {
                if (row[x] < s_line_threshold) {
                    has_black = true;
                    break;
                }
            }
        }
        if (has_black) {
            bits |= (uint8_t)(0x08 >> z);
        }
    }

    if (out_black) *out_black = total_black;
    if (out_cx) *out_cx = (black_px > 0) ? (int)(sum_x / black_px) : -1;
    return bits;
}

static uint8_t detect_zones_from_rgb565(const uint8_t *rgb, size_t stride_bytes,
                                        uint32_t w, uint32_t h,
                                        int *out_black, int *out_cx)
{
    /* 采样缩放 + RGB565 -> 灰度 */
    for (int dy = 0; dy < DETECT_H; dy++) {
        int sy = (int)((uint32_t)dy * h / DETECT_H);
        const uint8_t *row = rgb + (size_t)sy * stride_bytes;
        for (int dx = 0; dx < DETECT_W; dx++) {
            int sx = (int)((uint32_t)dx * w / DETECT_W);
            const uint8_t *px = row + (size_t)sx * 2;
            uint16_t p = (uint16_t)(px[0] | (px[1] << 8));
            uint8_t r8 = (uint8_t)(((p >> 11) & 0x1F) * 255 / 31);
            uint8_t g8 = (uint8_t)(((p >> 5) & 0x3F) * 255 / 63);
            uint8_t b8 = (uint8_t)((p & 0x1F) * 255 / 31);
            s_detect_gray[dy * DETECT_W + dx] =
                (uint8_t)((r8 * 299u + g8 * 587u + b8 * 114u) / 1000u);
        }
    }

#if CAMERA_ROTATE_180
    /* 线性数组整体反转 = 图像旋转 180°（行、列同时翻转）。
     * 在检测与串口发送之前修正，保证 cx/位图和查看器画面方向一致。 */
    for (size_t i = 0; i < (DETECT_W * DETECT_H) / 2; i++) {
        uint8_t t = s_detect_gray[i];
        s_detect_gray[i] = s_detect_gray[DETECT_W * DETECT_H - 1 - i];
        s_detect_gray[DETECT_W * DETECT_H - 1 - i] = t;
    }
#endif

    static const int near_edges[5] = NEAR_ZONE_EDGES;
    static const int far_edges[5] = FAR_ZONE_EDGES;

    /* 近带（底部 8 行，窄区间）与远带（上方 24 行，宽区间）分别出位图 */
    uint8_t near_bits = zone_bits_from_gray(
        (const uint8_t *)s_detect_gray, near_edges,
        DETECT_H - DETECT_BOTTOM_ROWS, DETECT_BOTTOM_ROWS,
        out_black, out_cx);
    int far_cx = -1;
    uint8_t far_bits = zone_bits_from_gray(
        (const uint8_t *)s_detect_gray, far_edges,
        DETECT_H - DETECT_BOTTOM_ROWS - DETECT_FAR_BAND_ROWS,
        DETECT_FAR_BAND_ROWS, NULL, &far_cx);
    s_near_bits = near_bits;
    s_far_bits = far_bits;
    s_far_cx = far_cx;

    /* 有效位图：近带有线用近带；近带空、远带有线用远带兜底（无弯道强制） */
    uint8_t bits = (near_bits != 0) ? near_bits : far_bits;

#if CAMERA_MIRROR
    bits = (uint8_t)(((bits & 0x01) << 3) | ((bits & 0x02) << 1) |
                     ((bits & 0x04) >> 1) | ((bits & 0x08) >> 3));
#endif

    /* 降采样 160x120 -> 80x60（2x2 平均），供串口查看器实时显示 */
    for (int sy = 0; sy < STREAM_H; sy++) {
        for (int sx = 0; sx < STREAM_W; sx++) {
            int y = sy * 2;
            int x = sx * 2;
            s_stream_gray[sy * STREAM_W + sx] = (uint8_t)(
                ((int)s_detect_gray[y * DETECT_W + x] +
                 (int)s_detect_gray[y * DETECT_W + x + 1] +
                 (int)s_detect_gray[(y + 1) * DETECT_W + x] +
                 (int)s_detect_gray[(y + 1) * DETECT_W + x + 1]) / 4);
        }
    }
    return bits;
}

/* ================= 串口灰度图实时显示 ================= */

static uint16_t crc16_xmodem(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* 整帧一次性写入串口（单次 uart_write_bytes，避免被日志插队破坏帧） */
static void send_gray_frame(void)
{
    const size_t payload_off = 4 + 2 + 2 + 2 + 2;  /* magic + w + h + th + near/far */
    const size_t frame_len = payload_off + STREAM_W * STREAM_H + 2;

    s_gray_frame[0] = 'G';
    s_gray_frame[1] = 'R';
    s_gray_frame[2] = 'A';
    s_gray_frame[3] = 'Y';
    s_gray_frame[4] = (uint8_t)(STREAM_W & 0xff);
    s_gray_frame[5] = (uint8_t)(STREAM_W >> 8);
    s_gray_frame[6] = (uint8_t)(STREAM_H & 0xff);
    s_gray_frame[7] = (uint8_t)(STREAM_H >> 8);
    /* 当前阈值随帧携带，查看器每帧读取，保证实时显示 */
    s_gray_frame[8] = (uint8_t)(s_line_threshold & 0xff);
    s_gray_frame[9] = (uint8_t)(s_line_threshold >> 8);
    /* 四路判定/远带随帧携带，查看器实时显示 */
    s_gray_frame[10] = s_near_bits;
    s_gray_frame[11] = s_far_bits;
    memcpy(s_gray_frame + payload_off, s_stream_gray, STREAM_W * STREAM_H);

    uint16_t crc = crc16_xmodem(s_stream_gray, STREAM_W * STREAM_H);
    s_gray_frame[payload_off + STREAM_W * STREAM_H] = (uint8_t)(crc & 0xff);
    s_gray_frame[payload_off + STREAM_W * STREAM_H + 1] = (uint8_t)(crc >> 8);

    uart_write_bytes(UART_NUM_0, s_gray_frame, frame_len);
}

/* 解码缩放宽高选择：
 *   循线阶段沿用原逻辑（输出约 80x60，省 CPU）；
 *   击球阶段（需要识别红球颜色）与独立 red-ball-push 一致，
 *   尽量解到 160x120，避免红色被 8x8 平均稀释导致判红失败。 */
static esp_jpeg_image_scale_t choose_decode_scale(uint32_t w, uint32_t h,
                                                  bool ball_mode)
{
    if (ball_mode) {
        if (w <= DEC_OUT_MAX_W && h <= DEC_OUT_MAX_H) {
            return JPEG_IMAGE_SCALE_0;
        }
        if (w / 2 <= DEC_OUT_MAX_W && h / 2 <= DEC_OUT_MAX_H) {
            return JPEG_IMAGE_SCALE_1_2;
        }
        if (w / 4 <= DEC_OUT_MAX_W && h / 4 <= DEC_OUT_MAX_H) {
            return JPEG_IMAGE_SCALE_1_4;
        }
        return JPEG_IMAGE_SCALE_1_8;
    }
    return (w >= 640) ? JPEG_IMAGE_SCALE_1_8 : JPEG_IMAGE_SCALE_1_4;
}

/* ================= 摄像头处理任务 ================= */

static void camera_proc_task(void *arg)
{
    (void)arg;

    int64_t t_last = 0;
    int64_t last_decode_us = 0;
    uint32_t last_total = 0;
    float fps = 0.0f;
    int64_t last_print = 0;

    while (1) {
        if (xSemaphoreTake(s_frame_ready, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* 限速：解码间隔不足时丢弃本帧并真正让出 CPU（阻塞 10ms），
         * 保证 cam_proc 有阻塞间隙，IDLE 任务能喂看门狗 */
        int64_t now_us = esp_timer_get_time();
        bool color_mode = ball_vision_is_enabled() || green_vision_is_enabled();
        int64_t min_gap_us = color_mode
                             ? (140 * 1000)   /* 击球：解码约 7fps，减轻 CPU/坏帧压力 */
                             : FRAME_DECODE_MIN_GAP_US;
        if (now_us - last_decode_us < min_gap_us) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        last_decode_us = now_us;

        if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }

        size_t jsize = s_frame_size;

        esp_jpeg_image_cfg_t cfg = {
            .indata = s_jpeg_buf,
            .indata_size = (uint32_t)jsize,
            .outbuf = s_dec_buf,
            .outbuf_size = (uint32_t)s_dec_buf_bytes,
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            .out_scale = JPEG_IMAGE_SCALE_1_4,
            .flags = { .swap_color_bytes = 0 },
            .advanced = {
                .working_buffer = s_work,
                .working_buffer_size = sizeof(s_work),
            },
        };

        esp_jpeg_image_output_t info;
        if (esp_jpeg_get_image_info(&cfg, &info) != ESP_OK) {
            xSemaphoreGive(s_frame_mutex);
            ESP_LOGW(TAG, "get_info failed (size=%u)", (unsigned)jsize);
            continue;
        }

        cfg.out_scale = choose_decode_scale(info.width, info.height,
                                            color_mode);

        esp_jpeg_image_output_t out;
        esp_err_t err = esp_jpeg_decode(&cfg, &out);
        xSemaphoreGive(s_frame_mutex);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "decode failed err=0x%x size=%u", err, (unsigned)jsize);
            /* 坏帧会反复到达：失败后主动让出，避免 cam_proc 独占 CPU 触发看门狗 */
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        size_t stride = out.output_len / out.height;

        /* 击球阶段：循线黑线检测完全退出；红/绿两阶段互斥，
         * 只对当前阶段对应的颜色做检测 */
        if (ball_vision_is_enabled()) {
            ball_vision_update(s_dec_buf, stride, out.width, out.height);
            /* 160x120 解码较慢：每帧主动让出，避免饿死 IDLE 触发看门狗 */
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        if (green_vision_is_enabled()) {
            green_vision_update(s_dec_buf, stride, out.width, out.height);
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        if (follower_ball_phase_active()) {
            /* 红→绿切换瞬间两个颜色检测都未开启时，也绝不回循线 */
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        int black = 0;
        int cx = -1;
        uint8_t bits = detect_zones_from_rgb565(s_dec_buf, stride,
                                                out.width, out.height,
                                                &black, &cx);
        s_cur_bits = bits;
        s_cam_seq++;   /* 每成功处理一帧推进一次，供控制端按帧差分 */

        /* 串口灰度流（'v' 开启）：供电脑端 tools/gray_viewer.py 实时显示 */
        if (s_gray_stream_enabled) {
            send_gray_frame();
        }

        int64_t now = esp_timer_get_time();
        if (t_last == 0) {
            t_last = now;
            last_total = s_fps_count;
        } else if (now - t_last >= 1000000) {
            fps = (float)(s_fps_count - last_total) * 1000000.0f / (float)(now - t_last);
            t_last = now;
            last_total = s_fps_count;
        }

        if (now - last_print >= PRINT_INTERVAL_MS * 1000) {
            last_print = now;
            ESP_LOGI(TAG, "LINE w=%u h=%u black=%d cx=%d near=0b%d%d%d%d far=0b%d%d%d%d bits=0b%d%d%d%d fps=%.1f",
                     (unsigned)out.width, (unsigned)out.height, black, cx,
                     (s_near_bits >> 3) & 1, (s_near_bits >> 2) & 1,
                     (s_near_bits >> 1) & 1, s_near_bits & 1,
                     (s_far_bits >> 3) & 1, (s_far_bits >> 2) & 1,
                     (s_far_bits >> 1) & 1, s_far_bits & 1,
                     (bits >> 3) & 1, (bits >> 2) & 1, (bits >> 1) & 1, bits & 1,
                     (double)fps);
        }
        /* 每帧处理完主动让出，保证 IDLE 任务能喂看门狗 */
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/* ================= 候选流格式 ================= */

static void build_profiles(const uvc_host_frame_info_t *list, size_t n)
{
    static const struct { int w; int h; } pref[] = {
        /* 与 red-ball-push 一致：优先 320x240（解码省 CPU，
         * 击球阶段 1/2 缩放出 160x120 更利于判红） */
        { 320, 240 }, { 640, 480 }, { 480, 320 },
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
                    s_profiles[s_profile_count].fps = 0;   /* 兜底：任意帧率 */
                    ESP_LOGI(TAG, "profile[%d] = %s %ux%u@any (fallback)",
                             s_profile_count, FORMAT_STR[list[i].format],
                             list[i].h_res, list[i].v_res);
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

/* ================= 取流任务 ================= */

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

        err = uvc_host_stream_start(h);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "stream start failed: %s", esp_err_to_name(err));
            uvc_host_stream_close(h);
            prof_idx = (prof_idx + 1) % s_profile_count;
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        ESP_LOGI(TAG, "streaming ...");

        while (s_device_connected) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        prof_idx = 0;
    }
}

/* ================= UVC 驱动事件回调 ================= */

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
        ESP_LOGE(TAG, "No UVC frame formats parsed (err=%s n=%d) - "
                 "check CONFIG_UVC_PRINTF_CONFIGURATION_DESCRIPTOR log",
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

/* ================= 缓冲分配 ================= */

static bool alloc_buffers(void)
{
    s_jpeg_buf = (uint8_t *)heap_caps_malloc(MAX_JPEG_BYTES,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_dec_buf = (uint8_t *)heap_caps_malloc(DEC_OUT_BUF_BYTES,
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (s_jpeg_buf == NULL || s_dec_buf == NULL) {
        ESP_LOGE(TAG, "PSRAM buffer allocation failed");
        return false;
    }
    s_dec_buf_bytes = DEC_OUT_BUF_BYTES;
    ESP_LOGI(TAG, "buffers: jpeg_in=%u rgb_out=%u work=%u",
             (unsigned)MAX_JPEG_BYTES, (unsigned)s_dec_buf_bytes,
             (unsigned)sizeof(s_work));
    return true;
}

/* ================= 串口 ================= */

static void init_serial(void)
{
    const uart_config_t uart_config = {
        .baud_rate = SERIAL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    /* TX 环形缓冲 8KB：够放一帧多灰度图，防止写帧时卡死日志 */
    uart_driver_install(UART_NUM_0, 256, 8192, 0, NULL, 0);
    uart_vfs_dev_use_driver(UART_NUM_0);

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
}

static void print_menu(void)
{
    printf("\n");
    printf("========== Camera Line Follower + Avoid (ESP32-S3) ==========\n");
    printf("Wheels: LF=8/9/10  REAR=12/13/14  RF=15/16/17  STBY=11\n");
    printf("Camera: USB D-=19 D+=20 | Ultrasonic: TRIG=18 ECHO=21\n");
    printf("TFT18: SCLK=39 MOSI=40 CS=41 DC=42 RST=47\n");
    printf("Serial baud: %d (VS Code monitor must be 921600)\n", SERIAL_BAUD);
    printf("Commands: g=go, s=stop, b=line bits, u=distance, t=motor test\n");
    printf("          v=gray stream ON, x=gray stream OFF\n");
    printf("          n/m = threshold -5/+5 (current %d)\n", s_line_threshold);
    printf("          c = TFT bits: near <-> control\n");
    printf("          p = force ball phase (search RED, push), i = ball vision info\n");
    printf("          (v: run tools/gray_viewer.py on PC to view image)\n");
    printf("BOOT button: start/stop (default stopped after flash)\n");
    printf("Press BOOT to start line following ...\n");
    printf("============================================================\n");
    printf("\n");
}

static void serial_task(void *arg)
{
    (void)arg;
    print_menu();

    while (1) {
        int ch = getchar();
        if (ch == EOF || ch == '\n' || ch == '\r') {
            continue;
        }
        switch ((char)ch) {
        case 'g':
        case 'G':
            follower_enable(true);
            printf("Line following started.\n");
            break;
        case 's':
        case 'S':
            follower_enable(false);
            printf("Stopped.\n");
            break;
        case 'b':
        case 'B':
            printf("CAM bits = 0x%02X (0x8=left ... 0x1=right)\n",
                   follower_cur_bits());
            break;
        case 'u':
        case 'U': {
            float d = ultrasonic_read_once_cm();
            printf("Distance = ");
            if (d > 0.0f) {
                printf("%.1f cm\n", d);
            } else {
                printf("no echo\n");
            }
            break;
        }
        case 't':
        case 'T': {
            /* 电机自检：先停车，再依次转左前/右前/后轮各 1 秒 */
            follower_enable(false);
            printf("Motor test: LF 1s -> RF 1s -> REAR 1s ...\n");
            motor_set_lf(120);
            motor_set_rf(0);
            motor_set_rear(0);
            vTaskDelay(pdMS_TO_TICKS(1000));
            motor_set_lf(0);
            motor_set_rf(120);
            motor_set_rear(0);
            vTaskDelay(pdMS_TO_TICKS(1000));
            motor_set_lf(0);
            motor_set_rf(0);
            motor_set_rear(120);
            vTaskDelay(pdMS_TO_TICKS(1000));
            motor_stop_all();
            printf("Motor test done.\n");
            break;
        }
        case 'v':
        case 'V':
            s_gray_stream_enabled = true;
            printf("Gray stream ON (run tools/gray_viewer.py on PC)\n");
            break;
        case 'x':
        case 'X':
            s_gray_stream_enabled = false;
            printf("Gray stream OFF\n");
            break;
        case 'n':
        case 'N':
            s_line_threshold -= 5;
            if (s_line_threshold < LINE_THRESHOLD_MIN) {
                s_line_threshold = LINE_THRESHOLD_MIN;
            }
            printf("\r\nTH=%d\n", s_line_threshold);
            break;
        case 'm':
        case 'M':
            s_line_threshold += 5;
            if (s_line_threshold > LINE_THRESHOLD_MAX) {
                s_line_threshold = LINE_THRESHOLD_MAX;
            }
            printf("\r\nTH=%d\n", s_line_threshold);
            break;
        case 'c':
        case 'C':
            tft_toggle_bits_mode();
            break;
        case 'p':
        case 'P':
            /* 调试：跳过巡线/避障/T 终点，直接进入“找红球推球” */
            follower_force_ball_phase();
            break;
        case 'i':
        case 'I': {
            if (green_vision_is_enabled()) {
                green_vision_t gv;
                if (green_vision_get(&gv)) {
                    printf("GREEN VIS: found=%d cx=%d pix=%d (map center=40)\n",
                           gv.green_found ? 1 : 0, gv.green_cx, gv.green_pixels);
                } else {
                    printf("GREEN VIS: no frame yet\n");
                }
            } else if (ball_vision_is_enabled()) {
                ball_vision_t bv;
                if (ball_vision_get(&bv)) {
                    printf("BALL VIS: found=%d cx=%d pix=%d (map center=40)\n",
                           bv.ball_found ? 1 : 0, bv.ball_cx, bv.ball_pixels);
                } else {
                    printf("BALL VIS: no frame yet\n");
                }
            } else {
                printf("VIS: no ball phase active\n");
            }
            break;
        }
        default:
            printf("Unknown command. Use g / s / b / u / t / v / x / n / m / c / p / i.\n");
            break;
        }
    }
}

/* ================= 超声测距任务 =================
 * 超声波读数是阻塞式（最长约 4~8ms），放在独立任务里，
 * 避免占用 10ms 控制周期。 */

static void ultrasonic_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        ultrasonic_update(now);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(30));
    }
}

/* ================= 控制任务 ================= */

static void control_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        follower_tick(now, s_cur_bits);
        /* 10ms 固定节拍：必须真正阻塞，否则会饿死 IDLE 任务触发看门狗 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

/* ================= 主程序 ================= */

void app_main(void)
{
    s_frame_ready = xSemaphoreCreateCounting(1, 0);
    s_frame_mutex = xSemaphoreCreateMutex();
    if (s_frame_ready == NULL || s_frame_mutex == NULL) {
        ESP_LOGE(TAG, "sync objects allocation failed");
        return;
    }
    if (!alloc_buffers()) {
        return;
    }

    init_serial();
    motor_init();
    ultrasonic_init();
    ball_vision_init();
    green_vision_init();
    follower_init();
    tft_init();

    BaseType_t ok = xTaskCreate(control_task, "control", 4096, NULL, 8, NULL);
    assert(ok == pdTRUE);
    ok = xTaskCreate(ultrasonic_task, "ultra", 3072, NULL, 7, NULL);
    assert(ok == pdTRUE);
    ok = xTaskCreate(serial_task, "serial_cmd", 4096, NULL, 3, NULL);
    assert(ok == pdTRUE);
    ok = xTaskCreate(camera_proc_task, "cam_proc", 8192, NULL, 6, NULL);
    assert(ok == pdTRUE);
    ok = xTaskCreate(follower_ball_task, "ball_ctl", 4096, NULL, 5, NULL);
    assert(ok == pdTRUE);

    /* 安装 USB Host 库 */
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    ok = xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 15, NULL);
    assert(ok == pdTRUE);

    /* 安装 UVC 驱动（v2 原生，支持 Bulk 流） */
    const uvc_host_driver_config_t uvc_config = {
        .driver_task_stack_size = 4 * 1024,
        .driver_task_priority = 16,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = uvc_driver_event_cb,
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_config));

    ESP_LOGI(TAG, "waiting for USB UVC device ...");
}
