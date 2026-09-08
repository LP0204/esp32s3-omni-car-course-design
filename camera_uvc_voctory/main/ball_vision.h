#ifndef BALL_VISION_H
#define BALL_VISION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 红球检测快照（移植自 red-ball-push 最新版）：
 * 在 80x60 网格上直接对解码图采样判红，ball_cx 是 0..79 的横向质心，
 * 画面中心 = 40。 */
typedef struct {
    bool ball_found;      /* 本帧至少有一个红点 */
    int  ball_cx;         /* 横向质心 0..79 */
    int  ball_pixels;     /* 红点数 */
    uint32_t frame_id;    /* 单调递增帧号：控制端只在有新帧时决策/计数 */
} ball_vision_t;

void ball_vision_init(void);
void ball_vision_set_enabled(bool enable);
bool ball_vision_is_enabled(void);

/* 每解码完一帧 RGB565 后调用；未使能时直接返回。 */
void ball_vision_update(const uint8_t *rgb565, size_t stride_bytes,
                        uint32_t width, uint32_t height);

bool ball_vision_get(ball_vision_t *out);

#endif /* BALL_VISION_H */
