/*
 * 红球检测（整体移植 red-ball-push 最新版）
 *
 * 判红：R > 90 且 R - (G+B)/2 > 90（red-ball-push 当前代码阈值）。
 * 采样：解码图直接映射到 80x60 网格，统计红点数并求横向质心；
 * 镜头装反只把 x 取镜像（对横向质心而言与 180° 旋转等价）。
 * 不再做连通域/黑球洞检测——击球动作完全交给 follower.c 的
 * 脉冲式找球/对准 + 定时推球状态机。
 */

#include "ball_vision.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define CAMERA_ROTATE_180  1

#define RED_MAP_W           80
#define RED_MAP_H           60

#define RED_MIN_R           90
#define RED_REDNESS_MIN     90

static bool s_enabled = false;
static SemaphoreHandle_t s_mutex = NULL;
static ball_vision_t s_result;
static uint32_t s_frame_id = 0;

void ball_vision_init(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
    memset(&s_result, 0, sizeof(s_result));
    s_enabled = false;
}

void ball_vision_set_enabled(bool enable)
{
    s_enabled = enable;
    if (!enable && s_mutex) {
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memset(&s_result, 0, sizeof(s_result));
            xSemaphoreGive(s_mutex);
        }
    }
}

bool ball_vision_is_enabled(void)
{
    return s_enabled;
}

void ball_vision_update(const uint8_t *rgb565, size_t stride_bytes,
                        uint32_t w, uint32_t h)
{
    if (!s_enabled || rgb565 == NULL || w == 0 || h == 0) {
        return;
    }

    uint16_t count = 0;
    uint32_t x_sum = 0;

    for (int gy = 0; gy < RED_MAP_H; gy++) {
        uint32_t sy = (uint32_t)gy * h / RED_MAP_H;
        const uint8_t *row = rgb565 + (size_t)sy * stride_bytes;
        for (int gx = 0; gx < RED_MAP_W; gx++) {
            uint32_t sx = (uint32_t)gx * w / RED_MAP_W;
            const uint8_t *px = row + (size_t)sx * 2;
            uint16_t p = (uint16_t)(px[0] | (px[1] << 8));

            int r8 = ((p >> 11) & 0x1F) * 255 / 31;
            int g8 = ((p >> 5)  & 0x3F) * 255 / 63;
            int b8 = ( p        & 0x1F) * 255 / 31;
            int redness = r8 - (g8 + b8) / 2;

            if (r8 > RED_MIN_R && redness > RED_REDNESS_MIN) {
                int logical_x = CAMERA_ROTATE_180 ?
                                (RED_MAP_W - 1 - gx) : gx;
                count++;
                x_sum += (uint32_t)logical_x;
            }
        }
    }

    ball_vision_t v;
    memset(&v, 0, sizeof(v));
    v.ball_found = (count > 0);
    v.ball_pixels = count;
    if (count > 0) {
        v.ball_cx = (int)(x_sum / count);
    }
    v.frame_id = ++s_frame_id;

    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_result = v;
        xSemaphoreGive(s_mutex);
    }
}

bool ball_vision_get(ball_vision_t *out)
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
