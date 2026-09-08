#include "tft18_sensor_display.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TFT_SCLK GPIO_NUM_39
#define TFT_MOSI GPIO_NUM_40
#define TFT_CS   GPIO_NUM_41
#define TFT_DC   GPIO_NUM_42
#define TFT_RST  GPIO_NUM_47
#define IR_OUT4  GPIO_NUM_7
#define IR_OUT3  GPIO_NUM_6
#define IR_OUT2  GPIO_NUM_5
#define IR_OUT1  GPIO_NUM_4
#define TFT_W 160
#define TFT_H 128
#define TFT_HZ (20 * 1000 * 1000)
#define TFT_CHUNK 4092
#define REFRESH_MS 200

#define C_BLACK  0x0000
#define C_WHITE  0xffff
#define C_YELLOW 0xffe0
#define C_GREEN  0x07e0

static const char *TAG = "TFT18";
static spi_device_handle_t s_dev;
static uint8_t s_frame[TFT_W * TFT_H * 2];
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_left, s_right, s_rear, s_distance = -1;
static uint8_t s_black_bits;
static bool s_started;

static esp_err_t tx(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    while (len) {
        size_t n = len > TFT_CHUNK ? TFT_CHUNK : len;
        spi_transaction_t t = {.length = n * 8, .tx_buffer = p};
        esp_err_t err = spi_device_polling_transmit(s_dev, &t);
        if (err != ESP_OK) return err;
        p += n;
        len -= n;
    }
    return ESP_OK;
}

static esp_err_t cmd(uint8_t v)
{
    gpio_set_level(TFT_DC, 0);
    return tx(&v, 1);
}

static esp_err_t data(const void *p, size_t n)
{
    gpio_set_level(TFT_DC, 1);
    return tx(p, n);
}

static esp_err_t cmd_data(uint8_t c, const void *p, size_t n)
{
    ESP_RETURN_ON_ERROR(cmd(c), TAG, "command failed");
    return n ? data(p, n) : ESP_OK;
}

static esp_err_t set_window(void)
{
    const uint8_t col[] = {0, 0, 0, TFT_W - 1};
    const uint8_t row[] = {0, 0, 0, TFT_H - 1};
    ESP_RETURN_ON_ERROR(cmd_data(0x2a, col, sizeof(col)), TAG, "column");
    ESP_RETURN_ON_ERROR(cmd_data(0x2b, row, sizeof(row)), TAG, "row");
    return cmd(0x2c);
}

static esp_err_t init_tft(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << TFT_DC) | (1ULL << TFT_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "control gpio");
    const spi_bus_config_t bus = {
        .mosi_io_num = TFT_MOSI, .miso_io_num = -1, .sclk_io_num = TFT_SCLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = TFT_CHUNK,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO),
                        TAG, "spi bus");
    const spi_device_interface_config_t dev = {
        .clock_speed_hz = TFT_HZ, .mode = 0, .spics_io_num = TFT_CS, .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &dev, &s_dev), TAG, "spi device");
    gpio_set_level(TFT_RST, 0); vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(TFT_RST, 1); vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(cmd(0x11), TAG, "sleep out");
    vTaskDelay(pdMS_TO_TICKS(120));
    const uint8_t pf[] = {0x55}, gamma[] = {0x04}, ge[] = {0x01};
    const uint8_t fr[] = {0,0}, inv[] = {0x07}, p1[] = {0x0a,0x02};
    const uint8_t p2[] = {0x02}, v1[] = {0x4f,0x5a}, v2[] = {0x40};
    const uint8_t madctl[] = {0xa0}, entry[] = {0};
    const uint8_t pg[] = {0x3f,0x25,0x1c,0x1e,0x20,0x12,0x2a,0x90,0x24,0x11,0,0,0,0,0};
    const uint8_t ng[] = {0x20,0x20,0x20,0x20,0x05,0,0x15,0xa7,0x3d,0x18,0x25,0x2a,0x2b,0x2b,0x3a};
    ESP_RETURN_ON_ERROR(cmd_data(0x3a,pf,1),TAG,"pf");
    ESP_RETURN_ON_ERROR(cmd_data(0x26,gamma,1),TAG,"gamma");
    ESP_RETURN_ON_ERROR(cmd_data(0xf2,ge,1),TAG,"gamma enable");
    ESP_RETURN_ON_ERROR(cmd_data(0xe0,pg,sizeof(pg)),TAG,"pg");
    ESP_RETURN_ON_ERROR(cmd_data(0xe1,ng,sizeof(ng)),TAG,"ng");
    ESP_RETURN_ON_ERROR(cmd_data(0xb1,fr,2),TAG,"fr");
    ESP_RETURN_ON_ERROR(cmd_data(0xb4,inv,1),TAG,"inv");
    ESP_RETURN_ON_ERROR(cmd_data(0xc0,p1,2),TAG,"p1");
    ESP_RETURN_ON_ERROR(cmd_data(0xc1,p2,1),TAG,"p2");
    ESP_RETURN_ON_ERROR(cmd_data(0xc5,v1,2),TAG,"v1");
    ESP_RETURN_ON_ERROR(cmd_data(0xc7,v2,1),TAG,"v2");
    ESP_RETURN_ON_ERROR(cmd_data(0x36,madctl,1),TAG,"madctl");
    ESP_RETURN_ON_ERROR(cmd_data(0xb7,entry,1),TAG,"entry");
    ESP_RETURN_ON_ERROR(cmd(0x29),TAG,"display on");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static void pixel(int x, int y, uint16_t c)
{
    if ((unsigned)x >= TFT_W || (unsigned)y >= TFT_H) return;
    size_t i = ((size_t)y * TFT_W + x) * 2;
    s_frame[i] = c >> 8; s_frame[i + 1] = c;
}

static void rect(int x, int y, int w, int h, uint16_t c)
{
    for (int yy = y; yy < y + h; ++yy)
        for (int xx = x; xx < x + w; ++xx) pixel(xx, yy, c);
}

static void fill(uint16_t c)
{
    for (size_t i = 0; i < TFT_W * TFT_H; ++i) {
        s_frame[i * 2] = c >> 8; s_frame[i * 2 + 1] = c;
    }
}

static const uint8_t *glyph(char c)
{
    static const uint8_t blank[5]={0};
    static const uint8_t d[10][5]={{0x3e,0x51,0x49,0x45,0x3e},{0,0x42,0x7f,0x40,0},
      {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},{0x18,0x14,0x12,0x7f,0x10},
      {0x27,0x45,0x45,0x45,0x39},{0x3c,0x4a,0x49,0x49,0x30},{1,0x71,9,5,3},
      {0x36,0x49,0x49,0x49,0x36},{6,0x49,0x49,0x29,0x1e}};
    static const uint8_t L[5]={0x7f,0x40,0x40,0x40,0x40};
    static const uint8_t R[5]={0x7f,9,0x19,0x29,0x46};
    static const uint8_t B[5]={0x7f,0x49,0x49,0x49,0x36};
    static const uint8_t D[5]={0x7f,0x41,0x41,0x22,0x1c};
    static const uint8_t colon[5]={0,0x36,0x36,0,0}, minus[5]={8,8,8,8,8};
    static const uint8_t dot[5]={0,0x60,0x60,0,0}, q[5]={2,1,0x51,9,6};
    if (c>='0'&&c<='9') return d[c-'0'];
    if (c == 'L') return L;
    if (c == 'R') return R;
    if (c == 'B') return B;
    if (c == 'D') return D;
    if (c == ':') return colon;
    if (c == '-') return minus;
    if (c == '.') return dot;
    if (c == '?') return q;
    return blank;
}

static void text(int x,int y,const char *s,int scale,uint16_t fg,uint16_t bg)
{
    while (*s) { const uint8_t *g=glyph(*s++); for(int col=0;col<6;++col){
        uint8_t bits=col<5?g[col]:0; for(int row=0;row<8;++row)
            rect(x+col*scale,y+row*scale,scale,scale,(bits&(1U<<row))?fg:bg);
    } x+=6*scale; }
}

static void draw(void)
{
    int l,r,b,dist; uint8_t bits;
    portENTER_CRITICAL(&s_lock); l=s_left;r=s_right;b=s_rear;dist=s_distance;bits=s_black_bits;portEXIT_CRITICAL(&s_lock);
    char buf[24]; fill(C_BLACK);
    snprintf(buf,sizeof(buf),"L:%d",l); text(4,4,buf,2,C_GREEN,C_BLACK);
    snprintf(buf,sizeof(buf),"R:%d",r); text(4,29,buf,2,C_GREEN,C_BLACK);
    snprintf(buf,sizeof(buf),"B:%d",b); text(4,54,buf,2,C_GREEN,C_BLACK);
    if(dist>=0) snprintf(buf,sizeof(buf),"D:%d.%d",dist/10,dist%10); else snprintf(buf,sizeof(buf),"D:?");
    text(4,79,buf,2,C_YELLOW,C_BLACK);
    for(int i=0;i<4;++i){bool black=(bits>>(3-i))&1;int x=5+i*39;rect(x,105,33,20,C_YELLOW);rect(x+2,107,29,16,black?C_BLACK:C_WHITE);}
}

static void display_task(void *arg)
{
    (void)arg; TickType_t last=xTaskGetTickCount();
    while(true){draw(); if(set_window()==ESP_OK){gpio_set_level(TFT_DC,1);tx(s_frame,sizeof(s_frame));} xTaskDelayUntil(&last,pdMS_TO_TICKS(REFRESH_MS));}
}

esp_err_t tft18_sensor_display_init(void)
{
    if(s_started)return ESP_OK;
    const gpio_config_t in={.pin_bit_mask=(1ULL<<IR_OUT4)|(1ULL<<IR_OUT3)|(1ULL<<IR_OUT2)|(1ULL<<IR_OUT1),.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE};
    ESP_RETURN_ON_ERROR(gpio_config(&in),TAG,"IR gpio");
    ESP_RETURN_ON_ERROR(init_tft(),TAG,"TFT init");
    if(xTaskCreate(display_task,"tft18",4096,NULL,1,NULL)!=pdPASS)return ESP_ERR_NO_MEM;
    s_started=true; return ESP_OK;
}

uint8_t tft18_sensor_display_read_raw(void)
{
    return (gpio_get_level(IR_OUT4)<<3)|(gpio_get_level(IR_OUT3)<<2)|(gpio_get_level(IR_OUT2)<<1)|gpio_get_level(IR_OUT1);
}

void tft18_sensor_display_update(uint8_t raw)
{
    portENTER_CRITICAL(&s_lock); s_black_bits=(uint8_t)(~raw)&0x0f; portEXIT_CRITICAL(&s_lock);
}

void tft18_sensor_display_set_motor_commands(int left,int right,int rear)
{
    if (left > 100) left = 100;
    if (left < -100) left = -100;
    if (right > 100) right = 100;
    if (right < -100) right = -100;
    if (rear > 100) rear = 100;
    if (rear < -100) rear = -100;
    portENTER_CRITICAL(&s_lock);s_left=left;s_right=right;s_rear=rear;portEXIT_CRITICAL(&s_lock);
}

void tft18_sensor_display_set_distance_mm(int distance_mm)
{
    portENTER_CRITICAL(&s_lock);s_distance=distance_mm;portEXIT_CRITICAL(&s_lock);
}
