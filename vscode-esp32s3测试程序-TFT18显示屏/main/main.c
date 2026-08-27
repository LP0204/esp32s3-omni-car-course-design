#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* LQ_TFT18SPIV33: ILI9163 series, write-only SPI, 3.3 V. */
#define TFT_PIN_SCLK  GPIO_NUM_39
#define TFT_PIN_MOSI  GPIO_NUM_40
#define TFT_PIN_CS    GPIO_NUM_41
#define TFT_PIN_DC    GPIO_NUM_42
#define TFT_PIN_RST   GPIO_NUM_47

#define TFT_WIDTH     160
#define TFT_HEIGHT    128
#define TFT_SPI_HZ    (20 * 1000 * 1000)
#define TFT_SPI_CHUNK 4092

/* IR order from the front of the car: far-left to far-right. */
#define IR_OUT4       GPIO_NUM_7
#define IR_OUT3       GPIO_NUM_6
#define IR_OUT2       GPIO_NUM_5
#define IR_OUT1       GPIO_NUM_4

/* HC-SR04: ECHO must be divided/translated to 3.3 V before GPIO21. */
#define ULTRASONIC_TRIG GPIO_NUM_18
#define ULTRASONIC_ECHO GPIO_NUM_21
#define ULTRASONIC_SAMPLE_MS 30
#define ULTRASONIC_TIMEOUT_US 4000
#define MIN_VALID_DISTANCE_MM 15
#define MAX_VALID_DISTANCE_MM 700
#define DISPLAY_REFRESH_MS 200
#define LOG_INTERVAL_MS 500

#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_YELLOW  0xFFE0
#define COLOR_GREEN   0x07E0

static const char *TAG = "TFT18_FULL_TEST";
static spi_device_handle_t s_tft;
static uint8_t s_frame[TFT_WIDTH * TFT_HEIGHT * 2];

/* This diagnostic project does not drive motors, so these remain zero. */
static int s_left_command;
static int s_right_command;
static int s_back_command;
static int s_distance_mm = -1;

static esp_err_t tft_tx(const void *data, size_t length)
{
    const uint8_t *cursor = data;
    while (length != 0) {
        const size_t chunk = length > TFT_SPI_CHUNK ? TFT_SPI_CHUNK : length;
    spi_transaction_t transaction = {
            .length = chunk * 8,
            .tx_buffer = cursor,
        };
        ESP_RETURN_ON_ERROR(spi_device_polling_transmit(s_tft, &transaction),
                            TAG, "SPI transmit failed");
        cursor += chunk;
        length -= chunk;
    }
    return ESP_OK;
}

static esp_err_t tft_command(uint8_t command)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(TFT_PIN_DC, 0), TAG, "DC failed");
    return tft_tx(&command, 1);
}

static esp_err_t tft_data(const void *data, size_t length)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(TFT_PIN_DC, 1), TAG, "DC failed");
    return tft_tx(data, length);
}

static esp_err_t tft_command_data(uint8_t command,
                                  const uint8_t *data,
                                  size_t length)
{
    ESP_RETURN_ON_ERROR(tft_command(command), TAG,
                        "command 0x%02x failed", command);
    if (length != 0) {
        ESP_RETURN_ON_ERROR(tft_data(data, length), TAG,
                            "data for 0x%02x failed", command);
    }
    return ESP_OK;
}

static esp_err_t tft_set_window(uint16_t x0, uint16_t y0,
                                uint16_t x1, uint16_t y1)
{
    const uint8_t column[] = {x0 >> 8, x0 & 0xff, x1 >> 8, x1 & 0xff};
    const uint8_t row[] = {y0 >> 8, y0 & 0xff, y1 >> 8, y1 & 0xff};
    ESP_RETURN_ON_ERROR(tft_command_data(0x2A, column, sizeof(column)),
                        TAG, "set column failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x2B, row, sizeof(row)),
                        TAG, "set row failed");
    return tft_command(0x2C);
}

static esp_err_t tft_init(void)
{
    const gpio_config_t control_gpio = {
        .pin_bit_mask = (1ULL << TFT_PIN_DC) | (1ULL << TFT_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&control_gpio), TAG,
                        "control GPIO init failed");

    const spi_bus_config_t bus = {
        .mosi_io_num = TFT_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = TFT_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_SPI_CHUNK,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO),
                        TAG, "SPI bus init failed");

    const spi_device_interface_config_t device = {
        .clock_speed_hz = TFT_SPI_HZ,
        .mode = 0,
        .spics_io_num = TFT_PIN_CS,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &device, &s_tft),
                        TAG, "SPI device init failed");

    gpio_set_level(TFT_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(TFT_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(tft_command(0x11), TAG, "sleep-out failed");
    vTaskDelay(pdMS_TO_TICKS(120));

    const uint8_t pixel_format[] = {0x55};
    const uint8_t gamma_curve[] = {0x04};
    const uint8_t gamma_enable[] = {0x01};
    const uint8_t frame_rate[] = {0x00, 0x00};
    const uint8_t inversion[] = {0x07};
    const uint8_t power1[] = {0x0A, 0x02};
    const uint8_t power2[] = {0x02};
    const uint8_t vcom1[] = {0x4F, 0x5A};
    const uint8_t vcom2[] = {0x40};
    const uint8_t orientation[] = {0xA0};
    const uint8_t entry_mode[] = {0x00};
    const uint8_t positive_gamma[] = {
        0x3F, 0x25, 0x1C, 0x1E, 0x20, 0x12, 0x2A, 0x90,
        0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    const uint8_t negative_gamma[] = {
        0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15, 0xA7,
        0x3D, 0x18, 0x25, 0x2A, 0x2B, 0x2B, 0x3A,
    };

    ESP_RETURN_ON_ERROR(tft_command_data(0x3A, pixel_format,
                                          sizeof(pixel_format)), TAG, "pixel format");
    ESP_RETURN_ON_ERROR(tft_command_data(0x26, gamma_curve,
                                          sizeof(gamma_curve)), TAG, "gamma curve");
    ESP_RETURN_ON_ERROR(tft_command_data(0xF2, gamma_enable,
                                          sizeof(gamma_enable)), TAG, "gamma enable");
    ESP_RETURN_ON_ERROR(tft_command_data(0xE0, positive_gamma,
                                          sizeof(positive_gamma)), TAG, "positive gamma");
    ESP_RETURN_ON_ERROR(tft_command_data(0xE1, negative_gamma,
                                          sizeof(negative_gamma)), TAG, "negative gamma");
    ESP_RETURN_ON_ERROR(tft_command_data(0xB1, frame_rate,
                                          sizeof(frame_rate)), TAG, "frame rate");
    ESP_RETURN_ON_ERROR(tft_command_data(0xB4, inversion,
                                          sizeof(inversion)), TAG, "inversion");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC0, power1, sizeof(power1)), TAG, "power1");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC1, power2, sizeof(power2)), TAG, "power2");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC5, vcom1, sizeof(vcom1)), TAG, "vcom1");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC7, vcom2, sizeof(vcom2)), TAG, "vcom2");
    ESP_RETURN_ON_ERROR(tft_command_data(0x36, orientation,
                                          sizeof(orientation)), TAG, "orientation");
    ESP_RETURN_ON_ERROR(tft_command_data(0xB7, entry_mode,
                                          sizeof(entry_mode)), TAG, "entry mode");
    ESP_RETURN_ON_ERROR(tft_command(0x29), TAG, "display-on failed");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static inline void frame_pixel(int x, int y, uint16_t color)
{
    if ((unsigned)x >= TFT_WIDTH || (unsigned)y >= TFT_HEIGHT) return;
    const size_t offset = ((size_t)y * TFT_WIDTH + x) * 2;
    s_frame[offset] = color >> 8;
    s_frame[offset + 1] = color & 0xff;
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
        for (int px = x; px < x + width; ++px) frame_pixel(px, py, color);
    }
}

static const uint8_t *glyph(char character)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
    static const uint8_t digits[10][5] = {
        {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
    };
    static const uint8_t b[5] = {0x7F,0x49,0x49,0x49,0x36};
    static const uint8_t l[5] = {0x7F,0x40,0x40,0x40,0x40};
    static const uint8_t r[5] = {0x7F,0x09,0x19,0x29,0x46};
    static const uint8_t d[5] = {0x7F,0x41,0x41,0x22,0x1C};
    static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t minus[5] = {0x08,0x08,0x08,0x08,0x08};
    static const uint8_t dot[5] = {0x00,0x60,0x60,0x00,0x00};
    static const uint8_t question[5] = {0x02,0x01,0x51,0x09,0x06};

    if (character >= '0' && character <= '9') return digits[character - '0'];
    if (character == 'B') return b;
    if (character == 'L') return l;
    if (character == 'R') return r;
    if (character == 'D') return d;
    if (character == ':') return colon;
    if (character == '-') return minus;
    if (character == '.') return dot;
    if (character == '?') return question;
    return blank;
}

static void frame_character(int x, int y, char character, int scale,
                            uint16_t foreground, uint16_t background)
{
    const uint8_t *bitmap = glyph(character);
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
        frame_character(x, y, *text, scale, foreground, background);
        x += 6 * scale;
        ++text;
    }
}

static esp_err_t frame_show(void)
{
    ESP_RETURN_ON_ERROR(tft_set_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1),
                        TAG, "window failed");
    gpio_set_level(TFT_PIN_DC, 1);
    return tft_tx(s_frame, sizeof(s_frame));
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

static int read_ultrasonic_once_mm(void)
{
    gpio_set_level(ULTRASONIC_TRIG, 0);
    esp_rom_delay_us(2);
    gpio_set_level(ULTRASONIC_TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(ULTRASONIC_TRIG, 0);

    int64_t start_us;
    int64_t end_us;
    if (!wait_for_level(ULTRASONIC_ECHO, 1, ULTRASONIC_TIMEOUT_US, &start_us) ||
        !wait_for_level(ULTRASONIC_ECHO, 0, ULTRASONIC_TIMEOUT_US, &end_us)) {
        return -1;
    }
    const int distance_mm = (int)(((end_us - start_us) * 10 + 29) / 58);
    return distance_mm >= MIN_VALID_DISTANCE_MM &&
           distance_mm <= MAX_VALID_DISTANCE_MM ? distance_mm : -1;
}

static int median3(int a, int b, int c)
{
    if (a > b) { int t = a; a = b; b = t; }
    if (b > c) { int t = b; b = c; c = t; }
    if (a > b) { int t = a; a = b; b = t; }
    return b;
}

static void update_distance(void)
{
    static int history[3];
    static int history_count;
    static int history_index;
    static int invalid_count;
    const int measured = read_ultrasonic_once_mm();

    if (measured < 0) {
        if (++invalid_count >= 3) s_distance_mm = -1;
        return;
    }
    invalid_count = 0;
    history[history_index] = measured;
    history_index = (history_index + 1) % 3;
    if (history_count < 3) ++history_count;

    if (history_count == 1) {
        s_distance_mm = history[0];
    } else if (history_count == 2) {
        s_distance_mm = (history[0] + history[1]) / 2;
    } else {
        s_distance_mm = median3(history[0], history[1], history[2]);
    }
}

static void draw_full_ui(uint8_t ir_bits)
{
    char text[20];
    frame_fill(COLOR_BLACK);

    snprintf(text, sizeof(text), "L:%d", s_left_command);
    frame_text(4, 4, text, 2, COLOR_GREEN, COLOR_BLACK);
    snprintf(text, sizeof(text), "R:%d", s_right_command);
    frame_text(4, 29, text, 2, COLOR_GREEN, COLOR_BLACK);
    snprintf(text, sizeof(text), "B:%d", s_back_command);
    frame_text(4, 54, text, 2, COLOR_GREEN, COLOR_BLACK);
    if (s_distance_mm >= 0) {
        snprintf(text, sizeof(text), "D:%d.%d", s_distance_mm / 10,
                 s_distance_mm % 10);
    } else {
        snprintf(text, sizeof(text), "D:?");
    }
    frame_text(4, 79, text, 2, COLOR_YELLOW, COLOR_BLACK);

    /* Bottom boxes are physical left-to-right: OUT4, OUT3, OUT2, OUT1. */
    for (int index = 0; index < 4; ++index) {
        const bool black = ((ir_bits >> (3 - index)) & 1U) != 0;
        const int x = 5 + index * 39;
        frame_rect(x, 105, 33, 20, COLOR_YELLOW);
        frame_rect(x + 2, 107, 29, 16, black ? COLOR_BLACK : COLOR_WHITE);
    }
    ESP_ERROR_CHECK(frame_show());
}

void app_main(void)
{
    const gpio_config_t ir_gpio = {
        .pin_bit_mask = (1ULL << IR_OUT4) | (1ULL << IR_OUT3) |
                        (1ULL << IR_OUT2) | (1ULL << IR_OUT1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ir_gpio));

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
    ESP_ERROR_CHECK(tft_init());

    ESP_LOGI(TAG, "full TFT UI: motor commands, HC-SR04 and four IR channels");
    ESP_LOGW(TAG, "HC-SR04 ECHO must be divided to 3.3 V before GPIO21");

    int64_t last_ultrasonic_ms = -ULTRASONIC_SAMPLE_MS;
    int64_t last_display_ms = -DISPLAY_REFRESH_MS;
    int64_t last_log_ms = -LOG_INTERVAL_MS;

    while (true) {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        const uint8_t ir_bits = read_ir_black_bits();
        if (now_ms - last_ultrasonic_ms >= ULTRASONIC_SAMPLE_MS) {
            last_ultrasonic_ms = now_ms;
            update_distance();
        }
        if (now_ms - last_display_ms >= DISPLAY_REFRESH_MS) {
            last_display_ms = now_ms;
            draw_full_ui(ir_bits);
        }
        if (now_ms - last_log_ms >= LOG_INTERVAL_MS) {
            last_log_ms = now_ms;
            ESP_LOGI(TAG, "IR=0x%X distance=%s%d.%dcm cmd[L,R,B]=[%d,%d,%d]",
                     ir_bits, s_distance_mm < 0 ? "invalid " : "",
                     s_distance_mm < 0 ? 0 : s_distance_mm / 10,
                     s_distance_mm < 0 ? 0 : s_distance_mm % 10,
                     s_left_command, s_right_command, s_back_command);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
