#ifndef AUDIO_UAC_H
#define AUDIO_UAC_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * USB Audio Class 扬声器：摄像头板是 USB 复合设备时，
 * UVC 负责图像，UAC TX 负责向其 2pin 扬声器接口输出 PCM。
 */
esp_err_t audio_uac_install(void);

/* 1..21 对应 C3..B5（低/中/高各 7 音）；0 表示静音。 */
void audio_uac_note_set(uint8_t note);
/* 和弦接口：pressed=true 按下，false 松开；每个音符独立管理释放包络。 */
void audio_uac_note_event(uint8_t note, bool pressed);
void audio_uac_note_stop(void);

bool audio_uac_ready(void);
uint32_t audio_uac_sample_rate(void);
uint8_t audio_uac_current_note(void);
/* 连续音量：0 = 静音，100 = USB 音频设备最大音量，101..200 为软件增强。 */
void audio_uac_set_volume_percent(uint16_t percent);

#endif
