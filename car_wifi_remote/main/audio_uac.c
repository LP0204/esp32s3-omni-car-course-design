/*
 * Wi-Fi 键盘钢琴的 USB 扬声器输出。
 *
 * 电脑只经 Wi-Fi 发送音符编号；ESP32-S3 本地合成 PCM，随后经 USB-UAC
 * 发给摄像头板上的扬声器。这样不会把连续音频占用在 Wi-Fi 链路上。
 */
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/uac_host.h"

#include "audio_uac.h"

static const char *TAG = "uac_piano";

#define AUDIO_EVENT_QUEUE_LEN      8
#define AUDIO_DRIVER_TASK_PRIO     10
#define AUDIO_RENDER_TASK_PRIO      8
#define AUDIO_RENDER_STACK          4096
#define AUDIO_RENDER_MS               10
#define AUDIO_PREBUFFER_CHUNKS          3
#define AUDIO_WRITE_TIMEOUT_MS        100
#define AUDIO_ATTACK_MS                5
#define AUDIO_RELEASE_MS             300
#define AUDIO_DEVICE_BUFFER_BYTES  16000
#define AUDIO_DEVICE_BUFFER_THRESHOLD 4000
#define AUDIO_PCM_BUFFER_BYTES      8192
#define AUDIO_VOLUME_PERCENT          55
#define AUDIO_AMPLITUDE             9000
#define AUDIO_NOTE_COUNT              22 /* index 0 = silence, 1..21 = notes */

typedef struct {
    uint32_t phase;
    uint32_t attack_remaining;
    uint32_t release_remaining;
    uint32_t release_total;
    uint16_t level_q15;
    uint8_t active;
} audio_voice_t;

typedef enum {
    AUDIO_EVT_DRIVER,
    AUDIO_EVT_DEVICE,
} audio_evt_group_t;

typedef struct {
    audio_evt_group_t group;
    union {
        struct {
            uint8_t addr;
            uint8_t iface_num;
            uac_host_driver_event_t event;
        } driver;
        struct {
            uac_host_device_handle_t handle;
            uac_host_device_event_t event;
        } device;
    } data;
} audio_evt_t;

static QueueHandle_t s_event_queue;
static uac_host_device_handle_t s_speaker;
static volatile bool s_ready;
static volatile uint8_t s_last_note;
static volatile uint32_t s_sample_rate;
static volatile uint16_t s_volume_pct = AUDIO_VOLUME_PERCENT;
static volatile uint16_t s_gain_pct = 100;
/* Voice state is shared by the HTTP task and the PCM render task. A
 * generation number lets the render task discard a stale snapshot when a
 * key event arrives while a chunk is being generated. */
static portMUX_TYPE s_voice_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_voice_generation;
static audio_voice_t s_voices[AUDIO_NOTE_COUNT];
static uint8_t s_channels;
static uint8_t s_subframe_size;
static uint8_t s_bits;
static uint8_t s_pcm[AUDIO_PCM_BUFFER_BYTES];

/* 低音 C3..B3 = 1..7，中音 C4..B4 = 8..14，高音 C5..B5 = 15..21 */
static const uint16_t s_note_hz[] = {
    0,             /* 0 = 静音 */
    131, 147, 165, 175, 196, 220, 248,      /* C3..B3（B3 微调 248Hz） */
    262, 294, 330, 349, 392, 440, 494,      /* C4..B4 */
    523, 587, 659, 698, 784, 880, 988,      /* C5..B5 */
};

/* 一个周期 64 点的正弦表；采样时使用相邻点线性插值，
 * 既避免在实时音频任务中调用浮点 sinf，也减少波表阶梯谐波。 */
static const int16_t s_sine64[64] = {
       0,  3212,  6393,  9512, 12539, 15446, 18204, 20787,
   23170, 25329, 27245, 28898, 30273, 31356, 32137, 32609,
   32767, 32609, 32137, 31356, 30273, 28898, 27245, 25329,
   23170, 20787, 18204, 15446, 12539,  9512,  6393,  3212,
       0, -3212, -6393, -9512,-12539,-15446,-18204,-20787,
  -23170,-25329,-27245,-28898,-30273,-31356,-32137,-32609,
  -32767,-32609,-32137,-31356,-30273,-28898,-27245,-25329,
  -23170,-20787,-18204,-15446,-12539, -9512, -6393, -3212,
};

static int16_t sine_sample_interpolated(uint32_t phase)
{
    const uint32_t index = phase >> 26;
    const uint32_t next = (index + 1U) & 63U;
    const uint32_t fraction = (phase >> 10) & 0xffffU;
    const int32_t first = s_sine64[index];
    const int32_t delta = (int32_t)s_sine64[next] - first;
    return (int16_t)(first + (delta * (int32_t)fraction) / 65536);
}

static uint32_t envelope_samples(uint32_t duration_ms)
{
    uint32_t samples = (uint32_t)(((uint64_t)s_sample_rate * duration_ms) / 1000U);
    return samples == 0 ? 1 : samples;
}

static void driver_event_cb(uint8_t addr, uint8_t iface_num,
                            const uac_host_driver_event_t event, void *arg)
{
    (void)arg;
    if (s_event_queue == NULL) {
        return;
    }
    const audio_evt_t item = {
        .group = AUDIO_EVT_DRIVER,
        .data.driver = { .addr = addr, .iface_num = iface_num, .event = event },
    };
    xQueueSend(s_event_queue, &item, 0);
}

static void device_event_cb(uac_host_device_handle_t handle,
                            const uac_host_device_event_t event, void *arg)
{
    (void)arg;
    if (s_event_queue == NULL) {
        return;
    }
    const audio_evt_t item = {
        .group = AUDIO_EVT_DEVICE,
        .data.device = { .handle = handle, .event = event },
    };
    xQueueSend(s_event_queue, &item, 0);
}

static bool frequency_supported(const uac_host_dev_alt_param_t *p, uint32_t freq)
{
    if (p->sample_freq_type == 0) {
        return freq >= p->sample_freq_lower && freq <= p->sample_freq_upper;
    }
    for (uint8_t i = 0; i < p->sample_freq_type && i < UAC_FREQ_NUM_MAX; i++) {
        if (p->sample_freq[i] == freq) {
            return true;
        }
    }
    return false;
}

static uint32_t fallback_frequency(const uac_host_dev_alt_param_t *p)
{
    static const uint32_t preferred[] = { 48000, 44100, 32000, 16000, 8000 };
    for (size_t i = 0; i < sizeof(preferred) / sizeof(preferred[0]); i++) {
        if (frequency_supported(p, preferred[i])) {
            return preferred[i];
        }
    }
    if (p->sample_freq_type > 0) {
        return p->sample_freq[0];
    }
    return p->sample_freq_lower;
}

static bool choose_speaker_format(uac_host_device_handle_t handle,
                                  uac_host_dev_alt_param_t *chosen,
                                  uint32_t *chosen_rate)
{
    uac_host_dev_info_t info = { 0 };
    if (uac_host_get_device_info(handle, &info) != ESP_OK) {
        return false;
    }

    for (uint8_t alt = 1; alt <= info.iface_alt_num; alt++) {
        uac_host_dev_alt_param_t p = { 0 };
        if (uac_host_get_device_alt_param(handle, alt, &p) != ESP_OK) {
            continue;
        }
        if (p.format != 1 || p.channels == 0 || p.channels > 2 ||
            p.subframe_size < 2 || p.subframe_size > 4 ||
            (p.bit_resolution != 16 && p.bit_resolution != 24 && p.bit_resolution != 32)) {
            continue;
        }
        const uint32_t rate = fallback_frequency(&p);
        if (rate == 0) {
            continue;
        }

        *chosen = p;
        *chosen_rate = rate;
        return true;
    }
    return false;
}

static void open_speaker(uint8_t addr, uint8_t iface_num)
{
    if (s_speaker != NULL) {
        return;
    }

    const uac_host_device_config_t dev_cfg = {
        .addr = addr,
        .iface_num = iface_num,
        .buffer_size = AUDIO_DEVICE_BUFFER_BYTES,
        .buffer_threshold = AUDIO_DEVICE_BUFFER_THRESHOLD,
        .callback = device_event_cb,
        .callback_arg = NULL,
    };
    uac_host_device_handle_t handle = NULL;
    esp_err_t err = uac_host_device_open(&dev_cfg, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "open speaker interface %u failed: %s", iface_num, esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "USB speaker found (addr=%u interface=%u)", addr, iface_num);
    uac_host_printf_device_param(handle);

    uac_host_dev_alt_param_t p = { 0 };
    uint32_t rate = 0;
    if (!choose_speaker_format(handle, &p, &rate)) {
        ESP_LOGE(TAG, "no usable PCM speaker format");
        uac_host_device_close(handle);
        return;
    }

    const uac_host_stream_config_t stream_cfg = {
        .channels = p.channels,
        .bit_resolution = p.bit_resolution,
        .sample_freq = rate,
        .flags = 0,
    };
    err = uac_host_device_start(handle, &stream_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start speaker stream failed: %s", esp_err_to_name(err));
        uac_host_device_close(handle);
        return;
    }

    s_speaker = handle;
    s_channels = p.channels;
    s_subframe_size = p.subframe_size;
    s_bits = p.bit_resolution;
    s_sample_rate = rate;
    portENTER_CRITICAL(&s_voice_mux);
    memset(s_voices, 0, sizeof(s_voices));
    s_last_note = 0;
    s_voice_generation++;
    portEXIT_CRITICAL(&s_voice_mux);
    s_ready = true;

    const uint8_t hw_volume = (s_volume_pct > 100) ? 100 : (uint8_t)s_volume_pct;
    const uint16_t requested_gain = (s_volume_pct > 100) ? s_volume_pct : 100;
    err = uac_host_device_set_volume(handle, hw_volume);
    if (err == ESP_OK) {
        /* The UAC device performs the volume scaling. Keep PCM at full
         * amplitude for 0..100%; values above 100% add software boost. */
        s_gain_pct = requested_gain;
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        /* Some UAC speakers expose no Feature Unit; use a safe software
         * fallback. */
        s_gain_pct = s_volume_pct;
        ESP_LOGW(TAG, "speaker has no UAC volume control; using software volume");
    } else {
        ESP_LOGW(TAG, "set speaker volume: %s", esp_err_to_name(err));
    }
    err = uac_host_device_set_mute(handle, false);
    if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "unmute speaker: %s", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "speaker ready: %" PRIu32 " Hz, %u ch, %u-bit, %u-byte samples",
             rate, p.channels, p.bit_resolution, p.subframe_size);
}

static void audio_control_task(void *arg)
{
    (void)arg;
    audio_evt_t item;
    while (true) {
        if (xQueueReceive(s_event_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (item.group == AUDIO_EVT_DRIVER) {
            if (item.data.driver.event == UAC_HOST_DRIVER_EVENT_TX_CONNECTED) {
                open_speaker(item.data.driver.addr, item.data.driver.iface_num);
            }
            continue;
        }

        if (item.data.device.event == UAC_HOST_DRIVER_EVENT_DISCONNECTED) {
            ESP_LOGW(TAG, "USB speaker disconnected");
            if (item.data.device.handle == s_speaker) {
                portENTER_CRITICAL(&s_voice_mux);
                s_ready = false;
                s_last_note = 0;
                memset(s_voices, 0, sizeof(s_voices));
                s_voice_generation++;
                s_speaker = NULL;
                s_sample_rate = 0;
                portEXIT_CRITICAL(&s_voice_mux);
            }
            uac_host_device_close(item.data.device.handle);
        } else if (item.data.device.event == UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR) {
            ESP_LOGW(TAG, "speaker transfer error");
        }
    }
}

static void write_pcm_sample(uint8_t *dst, int32_t sample)
{
    if (s_bits == 16) {
        const int16_t s = (int16_t)sample;
        dst[0] = (uint8_t)(s & 0xff);
        dst[1] = (uint8_t)((uint16_t)s >> 8);
    } else if (s_bits == 24) {
        const int32_t s = sample * 256;
        dst[0] = (uint8_t)(s & 0xff);
        dst[1] = (uint8_t)((uint32_t)s >> 8);
        dst[2] = (uint8_t)((uint32_t)s >> 16);
    } else { /* 32-bit */
        const int32_t s = sample * 65536;
        dst[0] = (uint8_t)(s & 0xff);
        dst[1] = (uint8_t)((uint32_t)s >> 8);
        dst[2] = (uint8_t)((uint32_t)s >> 16);
        dst[3] = (uint8_t)((uint32_t)s >> 24);
    }
}

static size_t make_pcm_chunk(audio_voice_t voices[AUDIO_NOTE_COUNT],
                             uint32_t *generation_out)
{
    uint32_t rate;
    uint8_t channels;
    uint8_t subframe;
    portENTER_CRITICAL(&s_voice_mux);
    rate = s_sample_rate;
    channels = s_channels;
    subframe = s_subframe_size;
    memcpy(voices, s_voices, sizeof(s_voices));
    *generation_out = s_voice_generation;
    portEXIT_CRITICAL(&s_voice_mux);
    if (!s_ready || rate == 0 || channels == 0 || subframe == 0) {
        return 0;
    }

    size_t frames = ((size_t)rate * AUDIO_RENDER_MS) / 1000;
    if (frames == 0) {
        frames = 1;
    }
    const size_t frame_bytes = (size_t)channels * subframe;
    if (frames * frame_bytes > sizeof(s_pcm)) {
        frames = sizeof(s_pcm) / frame_bytes;
    }

    uint8_t sounding = 0;
    for (uint8_t n = 1; n < AUDIO_NOTE_COUNT; n++) {
        if (voices[n].active) {
            sounding++;
        }
    }
    /* Scale the mixed voices only when their theoretical peak would exceed
     * 16-bit PCM full scale. This preserves the loudness of a single note,
     * while also keeping boosted chords free of digital clipping. */
    const uint32_t full_amp = (AUDIO_AMPLITUDE * (uint32_t)s_gain_pct) / 100U;
    const uint64_t peak = (uint64_t)full_amp * sounding;
    uint8_t divisor = (peak == 0) ? 1 : (uint8_t)((peak + 32766U) / 32767U);
    if (divisor == 0) {
        divisor = 1;
    }
    const int32_t amp = (int32_t)(full_amp / divisor);
    uint32_t phase_step[AUDIO_NOTE_COUNT] = { 0 };
    for (uint8_t n = 1; n < AUDIO_NOTE_COUNT; n++) {
        phase_step[n] = (uint32_t)(((uint64_t)s_note_hz[n] << 32) / rate);
    }
    uint8_t *p = s_pcm;
    for (size_t i = 0; i < frames; i++) {
        int64_t mixed = 0;
        for (uint8_t n = 1; n < AUDIO_NOTE_COUNT; n++) {
            audio_voice_t *voice = &voices[n];
            if (!voice->active) {
                continue;
            }
            const int32_t wave = sine_sample_interpolated(voice->phase);
            int32_t sample = (wave * amp) / 32767;
            sample = (int32_t)(((int64_t)sample * voice->level_q15) / 32767);

            if (voice->attack_remaining != 0) {
                const uint32_t gap = 32767U - voice->level_q15;
                const uint32_t step = (gap + voice->attack_remaining - 1U) /
                                      voice->attack_remaining;
                uint32_t next_level = voice->level_q15 + step;
                voice->level_q15 = (uint16_t)(next_level > 32767U ? 32767U : next_level);
                voice->attack_remaining--;
                if (voice->attack_remaining == 0) {
                    voice->level_q15 = 32767;
                }
            } else if (voice->release_remaining != 0 && voice->release_total != 0) {
                /* Each released key keeps its own pitch and fades linearly
                 * for 300 ms, independently of the other chord voices. */
                const uint32_t step = (voice->level_q15 +
                                       voice->release_remaining - 1U) /
                                      voice->release_remaining;
                voice->level_q15 = (uint16_t)(voice->level_q15 > step ?
                                              voice->level_q15 - step : 0);
                voice->release_remaining--;
                if (voice->release_remaining == 0) {
                    voice->active = 0;
                    voice->release_total = 0;
                    voice->level_q15 = 0;
                }
            }
            mixed += sample;
            voice->phase += phase_step[n];
        }
        if (mixed > 32767) {
            mixed = 32767;
        } else if (mixed < -32768) {
            mixed = -32768;
        }
        for (uint8_t ch = 0; ch < channels; ch++) {
            write_pcm_sample(p, (int32_t)mixed);
            p += subframe;
        }
    }

    return frames * frame_bytes;
}

static esp_err_t render_and_queue_chunk(void)
{
    audio_voice_t rendered_voices[AUDIO_NOTE_COUNT];
    uint32_t generation = 0;
    const size_t bytes = make_pcm_chunk(rendered_voices, &generation);
    if (bytes == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t err = uac_host_device_write(s_speaker, s_pcm, bytes,
                                                 pdMS_TO_TICKS(AUDIO_WRITE_TIMEOUT_MS));
    if (err != ESP_OK) {
        /* 写入失败时不提交相位和包络进度，下次重新生成同一段
         * 音频，避免丢掉10 ms导致相位跳变和咔哒声。 */
        return err;
    }

    /* 只有 PCM 确实进入 UAC 环形缓冲后，才提交这块的相位和包络进度。 */
    portENTER_CRITICAL(&s_voice_mux);
    if (generation == s_voice_generation) {
        memcpy(s_voices, rendered_voices, sizeof(rendered_voices));
    }
    portEXIT_CRITICAL(&s_voice_mux);
    return ESP_OK;
}

static void audio_render_task(void *arg)
{
    (void)arg;
    bool primed = false;
    TickType_t last_wake = 0;
    const TickType_t period = pdMS_TO_TICKS(AUDIO_RENDER_MS);

    while (true) {
        if (!s_ready || s_speaker == NULL) {
            primed = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (!primed) {
            bool prebuffer_ok = true;
            for (uint8_t i = 0; i < AUDIO_PREBUFFER_CHUNKS; i++) {
                if (render_and_queue_chunk() != ESP_OK) {
                    prebuffer_ok = false;
                    break;
                }
            }
            if (!prebuffer_ok) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            /* 先预存30 ms，再以10 ms的绝对周期补数据。生成和写入耗时
             * 不会像相对延时那样不断累加到播放周期中。 */
            last_wake = xTaskGetTickCount();
            primed = true;
        }

        vTaskDelayUntil(&last_wake, period);
        const esp_err_t err = render_and_queue_chunk();
        if (err == ESP_ERR_TIMEOUT) {
            /* 环形缓冲暂时写满：从当前时刻重新对齐，不丢相位进度。 */
            last_wake = xTaskGetTickCount();
            taskYIELD();
        } else if (err != ESP_OK) {
            ESP_LOGW(TAG, "PCM write failed: %s", esp_err_to_name(err));
            primed = false;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

esp_err_t audio_uac_install(void)
{
    s_event_queue = xQueueCreate(AUDIO_EVENT_QUEUE_LEN, sizeof(audio_evt_t));
    if (s_event_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const uac_host_driver_config_t config = {
        .create_background_task = true,
        .task_priority = AUDIO_DRIVER_TASK_PRIO,
        .stack_size = 4096,
        .core_id = tskNO_AFFINITY,
        .callback = driver_event_cb,
        .callback_arg = NULL,
    };
    esp_err_t err = uac_host_install(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uac_host_install: %s", esp_err_to_name(err));
        return err;
    }

    BaseType_t ok = xTaskCreate(audio_control_task, "uac_control", 4096,
                                NULL, AUDIO_DRIVER_TASK_PRIO, NULL);
    if (ok != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }
    ok = xTaskCreate(audio_render_task, "uac_render", AUDIO_RENDER_STACK,
                     NULL, AUDIO_RENDER_TASK_PRIO, NULL);
    if (ok != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "USB audio ready; waiting for camera speaker interface");
    return ESP_OK;
}

void audio_uac_note_event(uint8_t note, bool pressed)
{
    if (note >= AUDIO_NOTE_COUNT) {
        return;
    }

    bool changed = false;
    bool log_on = false;
    bool log_off = false;
    uint8_t log_note = note;

    portENTER_CRITICAL(&s_voice_mux);
    if (!pressed || note == 0) {
        /* Release each sounding voice independently. This preserves chords
         * while giving every released key its own 300 ms tail. */
        if (note == 0) {
            for (uint8_t n = 1; n < AUDIO_NOTE_COUNT; n++) {
                audio_voice_t *voice = &s_voices[n];
                if (voice->active && voice->release_remaining == 0) {
                    uint32_t total = envelope_samples(AUDIO_RELEASE_MS);
                    voice->attack_remaining = 0;
                    voice->release_total = total;
                    voice->release_remaining = total;
                    changed = true;
                }
            }
        } else {
            audio_voice_t *voice = &s_voices[note];
            if (voice->active && voice->release_remaining == 0) {
                uint32_t total = envelope_samples(AUDIO_RELEASE_MS);
                voice->attack_remaining = 0;
                voice->release_total = total;
                voice->release_remaining = total;
                changed = true;
            }
        }
        if (changed) {
            s_voice_generation++;
            log_off = true;
        }
    } else {
        audio_voice_t *voice = &s_voices[note];
        if (!voice->active) {
            voice->active = 1;
            voice->phase = 0;
            voice->level_q15 = 0;
            voice->attack_remaining = envelope_samples(AUDIO_ATTACK_MS);
            voice->release_remaining = 0;
            voice->release_total = 0;
            changed = true;
        } else if (voice->release_remaining != 0) {
            /* 重新按下时保留相位和当前音量，再用短淡入回到满幅，
             * 避免波形或包络突然跳变造成哒声。 */
            const uint32_t full_attack = envelope_samples(AUDIO_ATTACK_MS);
            const uint32_t gap = 32767U - voice->level_q15;
            voice->attack_remaining = (uint32_t)
                (((uint64_t)full_attack * gap + 32766U) / 32767U);
            voice->release_remaining = 0;
            voice->release_total = 0;
            changed = true;
        }
        s_last_note = note;
        if (changed) {
            s_voice_generation++;
            log_on = true;
        }
    }
    portEXIT_CRITICAL(&s_voice_mux);

    if (log_on) {
        ESP_LOGD(TAG, "chord note on=%u (%u Hz)", log_note, s_note_hz[log_note]);
    } else if (log_off) {
        if (log_note == 0) {
            ESP_LOGD(TAG, "chord all notes release: %" PRIu32 "ms", (uint32_t)AUDIO_RELEASE_MS);
        } else {
            ESP_LOGD(TAG, "chord note off=%u: %" PRIu32 "ms release",
                     log_note, (uint32_t)AUDIO_RELEASE_MS);
        }
    }
}

void audio_uac_note_set(uint8_t note)
{
    /* Backward-compatible shorthand: note>0 means key down, zero means all
     * keys released. New clients should use audio_uac_note_event(). */
    audio_uac_note_event(note, note != 0);
}

void audio_uac_note_stop(void)
{
    audio_uac_note_event(0, false);
}

bool audio_uac_ready(void)
{
    return s_ready;
}

uint32_t audio_uac_sample_rate(void)
{
    return s_sample_rate;
}

uint8_t audio_uac_current_note(void)
{
    return s_last_note;
}

void audio_uac_set_volume_percent(uint16_t percent)
{
    if (percent > 200) {
        percent = 200;
    }
    s_volume_pct = percent;

    /* Apply immediately when the USB speaker is already connected. If it is
     * not connected yet, the value is retained and applied on connection. */
    if (!s_ready || s_speaker == NULL) {
        ESP_LOGI(TAG, "volume target=%u%% (pending speaker connection)", percent);
        return;
    }

    const uint8_t hw_volume = (percent > 100) ? 100 : (uint8_t)percent;
    const uint16_t requested_gain = (percent > 100) ? percent : 100;
    const esp_err_t err = uac_host_device_set_volume(s_speaker, hw_volume);
    if (err == ESP_OK) {
        s_gain_pct = requested_gain;
        ESP_LOGI(TAG, "UAC volume=%u%%, software gain=%u%%",
                 hw_volume, requested_gain);
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        s_gain_pct = percent;
        ESP_LOGW(TAG, "UAC volume unsupported; software volume=%u%%", percent);
    } else {
        ESP_LOGW(TAG, "set UAC volume %u%% failed: %s",
                 percent, esp_err_to_name(err));
    }
}
