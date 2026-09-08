/*
 * LQ_TFT18SPIV33（ILI9163，160x128，SPI）显示驱动 - ESP-IDF 版
 *
 * 由 Arduino 版移植：
 *   - SPI2（FSPI）：SCLK=GPIO39, MOSI=GPIO40, CS=GPIO41, DC=GPIO42, RST=GPIO47
 *   - 整帧 40KB 放 PSRAM，20MHz 分块 SPI 发送
 *   - 界面：L/R/B 三轮速度 + D 超声距离 + 底部 4 个线位状态框
 *   - 刷新在独立低优先级任务里执行，不阻塞 10ms 控制周期
 */

#include "tft.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "motor.h"
#include "ultrasonic.h"
#include "follower.h"

extern uint8_t camera_near_bits(void);

#define TFT_PIN_SCLK GPIO_NUM_39
#define TFT_PIN_MOSI GPIO_NUM_40
#define TFT_PIN_CS   GPIO_NUM_41
#define TFT_PIN_DC   GPIO_NUM_42
#define TFT_PIN_RST  GPIO_NUM_47

#define TFT_WIDTH      160
#define TFT_HEIGHT     128
#define TFT_SPI_HZ     (20 * 1000 * 1000)
#define TFT_SPI_CHUNK  4092

#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_YELLOW  0xFFE0
#define COLOR_GREEN   0x07E0
#define COLOR_RED     0xF800

static spi_device_handle_t s_spi;
static uint8_t *s_frame;                 /* 160*128*2 = 40KB，PSRAM */
static SemaphoreHandle_t s_refresh_sem;
static uint8_t s_bits_mode = 0;          /* 0=近带判定(默认) 1=实际控制位图 */

/* ================= 底层 SPI ================= */

static void tft_tx(const uint8_t *data, size_t length)
{
    while (length != 0) {
        size_t chunk = length > TFT_SPI_CHUNK ? TFT_SPI_CHUNK : length;
        spi_transaction_t t = {
            .length = chunk * 8,
            .tx_buffer = data,
        };
        spi_device_polling_transmit(s_spi, &t);
        data += chunk;
        length -= chunk;
    }
}

static void tft_cmd(uint8_t command)
{
    gpio_set_level(TFT_PIN_DC, 0);
    tft_tx(&command, 1);
    gpio_set_level(TFT_PIN_DC, 1);
}

static void tft_cmd_data(uint8_t command, const uint8_t *data, size_t length)
{
    tft_cmd(command);
    if (length != 0) {
        tft_tx(data, length);
    }
}

static void tft_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t column[] = { (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xff),
                         (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xff) };
    uint8_t row[] = { (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xff),
                      (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xff) };
    tft_cmd_data(0x2A, column, sizeof(column));
    tft_cmd_data(0x2B, row, sizeof(row));
    tft_cmd(0x2C);
}

static void tft_init_panel(void)
{
    gpio_set_direction(TFT_PIN_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(TFT_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(TFT_PIN_DC, 0);
    gpio_set_level(TFT_PIN_RST, 1);

    /* 硬件复位 */
    gpio_set_level(TFT_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(TFT_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    tft_cmd(0x11); /* sleep out */
    vTaskDelay(pdMS_TO_TICKS(120));

    uint8_t pixelFormat[] = {0x55};       /* RGB565 */
    uint8_t gammaCurve[] = {0x04};
    uint8_t gammaEnable[] = {0x01};
    uint8_t frameRate[] = {0x00, 0x00};
    uint8_t inversion[] = {0x07};
    uint8_t power1[] = {0x0A, 0x02};
    uint8_t power2[] = {0x02};
    uint8_t vcom1[] = {0x4F, 0x5A};
    uint8_t vcom2[] = {0x40};
    uint8_t orientation[] = {0xA0};        /* landscape, BGR */
    uint8_t entryMode[] = {0x00};
    uint8_t positiveGamma[] = {
        0x3F, 0x25, 0x1C, 0x1E, 0x20, 0x12, 0x2A, 0x90,
        0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    uint8_t negativeGamma[] = {
        0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15, 0xA7,
        0x3D, 0x18, 0x25, 0x2A, 0x2B, 0x2B, 0x3A
    };

    tft_cmd_data(0x3A, pixelFormat, sizeof(pixelFormat));
    tft_cmd_data(0x26, gammaCurve, sizeof(gammaCurve));
    tft_cmd_data(0xF2, gammaEnable, sizeof(gammaEnable));
    tft_cmd_data(0xE0, positiveGamma, sizeof(positiveGamma));
    tft_cmd_data(0xE1, negativeGamma, sizeof(negativeGamma));
    tft_cmd_data(0xB1, frameRate, sizeof(frameRate));
    tft_cmd_data(0xB4, inversion, sizeof(inversion));
    tft_cmd_data(0xC0, power1, sizeof(power1));
    tft_cmd_data(0xC1, power2, sizeof(power2));
    tft_cmd_data(0xC5, vcom1, sizeof(vcom1));
    tft_cmd_data(0xC7, vcom2, sizeof(vcom2));
    tft_cmd_data(0x36, orientation, sizeof(orientation));
    tft_cmd_data(0xB7, entryMode, sizeof(entryMode));
    tft_cmd(0x29); /* display on */
    vTaskDelay(pdMS_TO_TICKS(20));
}

/* ================= 帧缓冲绘图 ================= */

static void frame_pixel(int x, int y, uint16_t color)
{
    if ((unsigned)x >= TFT_WIDTH || (unsigned)y >= TFT_HEIGHT) {
        return;
    }
    size_t index = ((size_t)y * TFT_WIDTH + x) * 2;
    s_frame[index] = color >> 8;
    s_frame[index + 1] = color & 0xff;
}

static void frame_fill(uint16_t color)
{
    for (size_t pixel = 0; pixel < TFT_WIDTH * TFT_HEIGHT; ++pixel) {
        s_frame[pixel * 2] = color >> 8;
        s_frame[pixel * 2 + 1] = color & 0xff;
    }
}

static void frame_rect(int x, int y, int width, int height, uint16_t color)
{
    for (int py = y; py < y + height; ++py) {
        for (int px = x; px < x + width; ++px) {
            frame_pixel(px, py, color);
        }
    }
}

/* 5x8 点阵：仅收录界面用到的字符 */
static const uint8_t *glyph(char c)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
    static const uint8_t digits[10][5] = {
        {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}
    };
    static const uint8_t b[5]     = {0x7F,0x49,0x49,0x49,0x36};
    static const uint8_t w[5]     = {0x3F,0x40,0x38,0x40,0x3F};
    static const uint8_t l[5]     = {0x7F,0x40,0x40,0x40,0x40};
    static const uint8_t r[5]     = {0x7F,0x09,0x19,0x29,0x46};
    static const uint8_t d[5]     = {0x7F,0x41,0x41,0x22,0x1C};
    static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t minus_[5]= {0x08,0x08,0x08,0x08,0x08};
    static const uint8_t dot[5]   = {0x00,0x60,0x60,0x00,0x00};
    static const uint8_t ques[5]  = {0x02,0x01,0x51,0x09,0x06};

    if (c >= '0' && c <= '9') return digits[c - '0'];
    if (c == 'B') return b;
    if (c == 'W') return w;
    if (c == 'L') return l;
    if (c == 'R') return r;
    if (c == 'D') return d;
    if (c == ':') return colon;
    if (c == '-') return minus_;
    if (c == '.') return dot;
    if (c == '?') return ques;
    return blank;
}

static void frame_char(int x, int y, char c, int scale,
                       uint16_t foreground, uint16_t background)
{
    const uint8_t *bitmap = glyph(c);
    for (int column = 0; column < 6; ++column) {
        const uint8_t bits = column < 5 ? bitmap[column] : 0;
        for (int row = 0; row < 8; ++row) {
            const uint16_t color = (bits & (1U << row)) ? foreground : background;
            frame_rect(x + column * scale, y + row * scale, scale, scale, color);
        }
    }
}

static void frame_text(int x, int y, const char *text, int scale,
                       uint16_t foreground, uint16_t background)
{
    while (*text != '\0') {
        frame_char(x, y, *text, scale, foreground, background);
        x += 6 * scale;
        ++text;
    }
}

static void frame_show(void)
{
    tft_set_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
    tft_tx(s_frame, (size_t)TFT_WIDTH * TFT_HEIGHT * 2);
}

/*
 * 整屏刷新：L/R/B 三轮速度 + D 超声距离 + 底部 4 个线位状态框
 * （显示顺序从左到右 OUT4、OUT3、OUT2、OUT1）
 */
static void draw_ui(uint8_t bits, int lf, int rf, int rear, float dist_cm)
{
    char buf[20];
    frame_fill(COLOR_BLACK);

    snprintf(buf, sizeof(buf), "L:%d", lf);
    frame_text(4, 4, buf, 2, COLOR_GREEN, COLOR_BLACK);

    snprintf(buf, sizeof(buf), "R:%d", rf);
    frame_text(4, 29, buf, 2, COLOR_GREEN, COLOR_BLACK);

    snprintf(buf, sizeof(buf), "B:%d", rear);
    frame_text(4, 54, buf, 2, COLOR_GREEN, COLOR_BLACK);

    if (dist_cm > 0.0f) {
        snprintf(buf, sizeof(buf), "D:%.1f", dist_cm);
    } else {
        snprintf(buf, sizeof(buf), "D:?");
    }
    frame_text(4, 79, buf, 2, COLOR_YELLOW, COLOR_BLACK);

    /* 小屏幕色块顺序与查看器相反：最左=bit0，最右=bit3 */
    for (int index = 0; index < 4; ++index) {
        bool is_black = ((bits >> index) & 1U) != 0;
        int x = 5 + index * 39;
        frame_rect(x, 105, 33, 20, COLOR_YELLOW);
        /* 击球阶段：循线已暂停。红球阶段显示红色，绿球阶段显示绿色 */
        if (follower_green_phase_active()) {
            frame_rect(x + 2, 107, 29, 16, COLOR_GREEN);
        } else if (follower_ball_phase_active()) {
            frame_rect(x + 2, 107, 29, 16, COLOR_RED);
        } else {
            frame_rect(x + 2, 107, 29, 16,
                       is_black ? COLOR_BLACK : COLOR_WHITE);
        }
    }

    frame_show();
}

/* ================= 显示任务 ================= */

static void display_task(void *arg)
{
    (void)arg;
    while (1) {
        if (xSemaphoreTake(s_refresh_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
            continue;
        }
        uint8_t bits = (s_bits_mode == 0) ? camera_near_bits()
                                          : follower_cur_bits();
        draw_ui(bits,
                motor_shown_lf(), motor_shown_rf(), motor_shown_rear(),
                ultrasonic_display_cm());
    }
}

void tft_toggle_bits_mode(void)
{
    s_bits_mode ^= 1;
    printf("TFT bits: %s\n", s_bits_mode == 0 ? "NEAR" : "CONTROL");
}

void tft_request_refresh(void)
{
    xSemaphoreGive(s_refresh_sem);
}

void tft_init(void)
{
    s_frame = heap_caps_malloc((size_t)TFT_WIDTH * TFT_HEIGHT * 2,
                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(s_frame != NULL);

    s_refresh_sem = xSemaphoreCreateCounting(1, 0);
    assert(s_refresh_sem != NULL);

    spi_bus_config_t bus_cfg = {
        .sclk_io_num = TFT_PIN_SCLK,
        .mosi_io_num = TFT_PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_SPI_CHUNK + 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfg = {
        .mode = 0,
        .clock_speed_hz = TFT_SPI_HZ,
        .spics_io_num = TFT_PIN_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_spi));

    tft_init_panel();

    BaseType_t ok = xTaskCreate(display_task, "tft_disp", 8192,
                                NULL, 2, NULL);
    assert(ok == pdTRUE);
}
