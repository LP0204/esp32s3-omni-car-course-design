/*
 * 绿球检测（整体移植 green-ball-push）
 *
 * 判绿：G > GREEN_MIN_CHANNEL 且 绿度 = G - (R+B)/2 > GREENNESS_THRESHOLD。
 * 采样方式、镜头旋转镜像、cx 坐标约定与 red-ball-push 完全一致，
 * 只替换颜色判定，与红球检测互不干扰。
 */

#include "green_vision.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define CAMERA_ROTATE_180  1

#define GREEN_MAP_W         80
#define GREEN_MAP_H         60

#define GREEN_MIN_CHANNEL   75    /* G 通道最低亮度 */
#define GREENNESS_THRESHOLD 30    /* 绿度下限 */

static bool s_enabled = false;
static SemaphoreHandle_t s_mutex = NULL;
static green_vision_t s_result;
static uint32_t s_frame_id = 0;

void green_vision_init(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
    memset(&s_result, 0, sizeof(s_result));
    s_enabled = false;
}

void green_vision_set_enabled(bool enable)
{
    s_enabled = enable;
    if (!enable && s_mutex) {
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memset(&s_result, 0, sizeof(s_result));
            xSemaphoreGive(s_mutex);
        }
    }
}

bool green_vision_is_enabled(void)
{
    return s_enabled;
}

void green_vision_update(const uint8_t *rgb565, size_t stride_bytes,
                         uint32_t w, uint32_t h)
{
    if (!s_enabled || rgb565 == NULL || w == 0 || h == 0) {
        return;
    }

    uint16_t count = 0;
    uint32_t x_sum = 0;

    for (int gy = 0; gy < GREEN_MAP_H; gy++) {
        uint32_t sy = (uint32_t)gy * h / GREEN_MAP_H;
        const uint8_t *row = rgb565 + (size_t)sy * stride_bytes;
        for (int gx = 0; gx < GREEN_MAP_W; gx++) {
            uint32_t sx = (uint32_t)gx * w / GREEN_MAP_W;
            const uint8_t *px = row + (size_t)sx * 2;
            uint16_t p = (uint16_t)(px[0] | (px[1] << 8));

            int r8 = ((p >> 11) & 0x1F) * 255 / 31;
            int g8 = ((p >> 5)  & 0x3F) * 255 / 63;
            int b8 = ( p        & 0x1F) * 255 / 31;
            int greenness = g8 - (r8 + b8) / 2;

            if (g8 > GREEN_MIN_CHANNEL && greenness > GREENNESS_THRESHOLD) {
                int logical_x = CAMERA_ROTATE_180 ?
                                (GREEN_MAP_W - 1 - gx) : gx;
                count++;
                x_sum += (uint32_t)logical_x;
            }
        }
    }

    green_vision_t v;
    memset(&v, 0, sizeof(v));
    v.green_found = (count > 0);
    v.green_pixels = count;
    if (count > 0) {
        v.green_cx = (int)(x_sum / count);
    }
    v.frame_id = ++s_frame_id;

    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_result = v;
        xSemaphoreGive(s_mutex);
    }
}

bool green_vision_get(green_vision_t *out)
{
    if (out == NULL || s_mutex == NULL) {
        return false;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return false;
    }
    *out = s_result;
    xSemaphoreGive(s_mutex);
    return true;
}
