#ifndef UVC_STREAM_H
#define UVC_STREAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* 初始化 USB Host + UVC 驱动并自动开始取流（阻塞安装，随后后台取帧） */
esp_err_t uvc_stream_init(void);

/* 钢琴页暂停 UVC USB 传输以给 UAC 音频让出带宽；
 * 回到遥控主页后恢复取流。调用本身不阻塞 HTTP 任务。 */
void uvc_stream_set_paused(bool paused);
bool uvc_stream_is_paused(void);

/* 等待一帧新 JPEG（timeout_ms）。成功后配合 lock/unlock 读取 */
bool uvc_stream_wait_frame(uint32_t timeout_ms);
bool uvc_stream_lock_frame(const uint8_t **data, size_t *len);
void uvc_stream_unlock_frame(void);

#endif
