#ifndef GREEN_VISION_H
#define GREEN_VISION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 绿球检测快照（独立于红球检测，移植自 green-ball-push）：
 * 80x60 网格逐点判绿，cx 为 0..79 横向质心，中心 = 40。 */
typedef struct {
    bool green_found;
    int  green_cx;
    int  green_pixels;
    uint32_t frame_id;
} green_vision_t;

void green_vision_init(void);
void green_vision_set_enabled(bool enable);
bool green_vision_is_enabled(void);
void green_vision_update(const uint8_t *rgb565, size_t stride_bytes,
                         uint32_t width, uint32_t height);
bool green_vision_get(green_vision_t *out);

#endif /* GREEN_VISION_H */
