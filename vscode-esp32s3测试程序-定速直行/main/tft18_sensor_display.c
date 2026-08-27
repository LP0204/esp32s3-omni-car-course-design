#include "tft18_sensor_display.h"

#include <stdbool.h>
#include <stddef.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
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

#define IR_OUT4       GPIO_NUM_7
#define IR_OUT3       GPIO_NUM_6
#define IR_OUT2       GPIO_NUM_5
#define IR_OUT1       GPIO_NUM_4

#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_YELLOW  0xFFE0

static const char *TAG = "SENSOR_DISPLAY";
static spi_device_handle_t s_tft;
static uint8_t s_frame[TFT_WIDTH * TFT_HEIGHT * 2];

static esp_err_t tft_tx(const void *data, size_t length)
{
    const uint8_t *cursor = data;
    while (length != 0) {
        const size_t chunk = length > TFT_SPI_CHUNK ? TFT_SPI_CHUNK : length;
        spi_transaction_t transaction = {
            .length = chunk * 8,
            .tx_buffer = cursor,
        };
        const esp_err_t error = spi_device_polling_transmit(s_tft,
                                                             &transaction);
        if (error != ESP_OK) {
            return error;
        }
        cursor += chunk;
        length -= chunk;
    }
    return ESP_OK;
}

static esp_err_t tft_command(uint8_t command)
{
    gpio_set_level(TFT_PIN_DC, 0);
    return tft_tx(&command, 1);
}

static esp_err_t tft_data(const void *data, size_t length)
{
    gpio_set_level(TFT_PIN_DC, 1);
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
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus,
                                            SPI_DMA_CH_AUTO),
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
                                          sizeof(pixel_format)),
                        TAG, "pixel format failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x26, gamma_curve,
                                          sizeof(gamma_curve)),
                        TAG, "gamma curve failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xF2, gamma_enable,
                                          sizeof(gamma_enable)),
                        TAG, "gamma enable failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xE0, positive_gamma,
                                          sizeof(positive_gamma)),
                        TAG, "positive gamma failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xE1, negative_gamma,
                                          sizeof(negative_gamma)),
                        TAG, "negative gamma failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xB1, frame_rate,
                                          sizeof(frame_rate)),
                        TAG, "frame rate failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xB4, inversion,
                                          sizeof(inversion)),
                        TAG, "inversion failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC0, power1, sizeof(power1)),
                        TAG, "power1 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC1, power2, sizeof(power2)),
                        TAG, "power2 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC5, vcom1, sizeof(vcom1)),
                        TAG, "vcom1 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xC7, vcom2, sizeof(vcom2)),
                        TAG, "vcom2 failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0x36, orientation,
                                          sizeof(orientation)),
                        TAG, "orientation failed");
    ESP_RETURN_ON_ERROR(tft_command_data(0xB7, entry_mode,
                                          sizeof(entry_mode)),
                        TAG, "entry mode failed");
    ESP_RETURN_ON_ERROR(tft_command(0x29), TAG, "display-on failed");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static inline void frame_pixel(int x, int y, uint16_t color)
{
    if ((unsigned)x >= TFT_WIDTH || (unsigned)y >= TFT_HEIGHT) {
        return;
    }
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
        for (int px = x; px < x + width; ++px) {
            frame_pixel(px, py, color);
        }
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
    static const uint8_t black[5] = {0x7F,0x49,0x49,0x49,0x36};
    static const uint8_t white[5] = {0x3F,0x40,0x38,0x40,0x3F};
    if (character >= '0' && character <= '9') {
        return digits[character - '0'];
    }
    if (character == 'B') return black;
    if (character == 'W') return white;
    return blank;
}

static void frame_character(int x, int y, char character, int scale,
                            uint16_t foreground, uint16_t background)
{
    const uint8_t *bitmap = glyph(character);
    for (int column = 0; column < 6; ++column) {
        const uint8_t bits = column < 5 ? bitmap[column] : 0;
        for (int row = 0; row < 8; ++row) {
            const uint16_t color = (bits & (1U << row))
                                       ? foreground : background;
            frame_rect(x + column * scale, y + row * scale,
                       scale, scale, color);
        }
    }
}

static esp_err_t frame_show(void)
{
    ESP_RETURN_ON_ERROR(tft_set_window(0, 0, TFT_WIDTH - 1,
                                        TFT_HEIGHT - 1),
                        TAG, "window failed");
    gpio_set_level(TFT_PIN_DC, 1);
    return tft_tx(s_frame, sizeof(s_frame));
}

esp_err_t tft18_sensor_display_init(void)
{
    const gpio_config_t ir_gpio = {
        .pin_bit_mask = (1ULL << IR_OUT4) | (1ULL << IR_OUT3) |
                        (1ULL << IR_OUT2) | (1ULL << IR_OUT1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&ir_gpio), TAG, "IR GPIO init failed");
    ESP_RETURN_ON_ERROR(tft_init(), TAG, "TFT init failed");
    ESP_LOGI(TAG, "four-sensor display initialized");
    return ESP_OK;
}

uint8_t tft18_sensor_display_read_raw(void)
{
    return (gpio_get_level(IR_OUT4) << 3) |
           (gpio_get_level(IR_OUT3) << 2) |
           (gpio_get_level(IR_OUT2) << 1) |
           gpio_get_level(IR_OUT1);
}

esp_err_t tft18_sensor_display_update(uint8_t raw)
{
    frame_fill(COLOR_BLACK);
    for (int index = 0; index < 4; ++index) {
        const bool is_black = ((raw >> index) & 1U) == 0;
        const int x = 1 + index * 40;
        const uint16_t fill = is_black ? COLOR_BLACK : COLOR_WHITE;
        const uint16_t text = is_black ? COLOR_WHITE : COLOR_BLACK;

        frame_rect(x, 1, 38, 126, COLOR_YELLOW);
        frame_rect(x + 2, 3, 34, 122, fill);
        frame_character(x + 11, 22, (char)('1' + index), 2, text, fill);
        frame_character(x + 11, 78, is_black ? 'B' : 'W', 2, text, fill);
    }
    return frame_show();
}
