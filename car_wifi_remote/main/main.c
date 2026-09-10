/*
 * 手机 WiFi 遥控小车（摄像头实时画面 + 云台 + 按钮/摇杆双模式）
 *  - 按钮模式：前/后/横移/原地转/停止
 *  - 摇杆模式：上下=直行，左右=转向，斜向按角度混合直行与转向
 *  - 三档速度：中速 ×0.8 / 高速 ×1.0 / 超高速 ×1.25
 * 端口：80=控制+页面，81=MJPEG 视频流
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_https_server.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "motor.h"
#include "servo.h"
#include "uvc_stream.h"
#include "audio_uac.h"

#define AP_SSID       "CAR-REMOTE"
#define AP_PASS       "12345678"
#define AP_CHANNEL    1
#define AP_MAX_CONN   4

#define HTTP_PORT      80
#define STREAM_PORT    81
#define HTTPS_PORT     443
#define CMD_BUF_LEN    96

/* 速度基准（高速档 = 原始值） */
#define LF_FWD_SPEED      150
#define RF_FWD_SPEED      165
#define TURN_SPEED        90
#define STRAFE_LF_SPEED   90
#define STRAFE_RF_SPEED   99
#define STRAFE_REAR_SPEED 180

#define SERVO_STEP_DEG    12
#define AUTO_STOP_MS      1500
#define PIANO_NOTE_COUNT  21
#define PIANO_MASK_ALL    ((1U << PIANO_NOTE_COUNT) - 1U)
#define PIANO_LINK_TIMEOUT_MS 1000

static const char *TAG = "car_remote";
static int64_t s_last_cmd_us = 0;
static int s_speed_pct = 100;   /* 中=80 高=100 超=125 */
static int s_pan = 90;
static int s_tilt = 90;
static esp_timer_handle_t s_voice_stop_timer = NULL;
static SemaphoreHandle_t s_piano_state_mutex = NULL;
static uint32_t s_piano_session = 0;
static uint32_t s_piano_sequence = 0;
static uint32_t s_piano_mask = 0;
static int64_t s_piano_last_rx_us = 0;

/* 按当前速度档位缩放 */
static int scale(int v)
{
    return v * s_speed_pct / 100;
}

/* ---------------- 运动 ---------------- */

static void motion_forward(void)
{
    motor_set_lf(scale(LF_FWD_SPEED));
    motor_set_rf(scale(RF_FWD_SPEED));
    motor_set_rear(0);
}

static void motion_backward(void)
{
    motor_set_lf(-scale(LF_FWD_SPEED));
    motor_set_rf(-scale(RF_FWD_SPEED));
    motor_set_rear(0);
}

/* 原地转：三联动 1:1:1 */
static void motion_left(void)
{
    motor_set_lf(-scale(TURN_SPEED));
    motor_set_rf(scale(TURN_SPEED));
    motor_set_rear(scale(TURN_SPEED));
}

static void motion_right(void)
{
    motor_set_lf(scale(TURN_SPEED));
    motor_set_rf(-scale(TURN_SPEED));
    motor_set_rear(-scale(TURN_SPEED));
}

/* 横移：左前/右前/后轮 1:1:2 */
static void motion_strafe_left(void)
{
    motor_set_lf(-scale(STRAFE_LF_SPEED));
    motor_set_rf(scale(STRAFE_RF_SPEED));
    motor_set_rear(-scale(STRAFE_REAR_SPEED));
}

static void motion_strafe_right(void)
{
    motor_set_lf(scale(STRAFE_LF_SPEED));
    motor_set_rf(-scale(STRAFE_RF_SPEED));
    motor_set_rear(scale(STRAFE_REAR_SPEED));
}

static void motion_stop(void)
{
    motor_stop_all();
}

static void run_action(const char *act)
{
    if (act == NULL) {
        motion_stop();
        return;
    }
    if (strcmp(act, "forward") == 0) {
        motion_forward();
    } else if (strcmp(act, "back") == 0) {
        motion_backward();
    } else if (strcmp(act, "left") == 0) {
        motion_left();
    } else if (strcmp(act, "right") == 0) {
        motion_right();
    } else if (strcmp(act, "strleft") == 0) {
        motion_strafe_left();
    } else if (strcmp(act, "strright") == 0) {
        motion_strafe_right();
    } else {
        motion_stop();
    }
    s_last_cmd_us = esp_timer_get_time();
}

/* 摇杆：x/y ∈ [-100,100]，y>0 前进，x>0 右转
 *  |x| 小 -> 直行；|y| 小 -> 原地转；都大 -> 差速混合 */
static void joy_motion(int x, int y)
{
    if (x < -100) x = -100;
    if (x > 100)  x = 100;
    if (y < -100) y = -100;
    if (y > 100)  y = 100;

    int drive_l = LF_FWD_SPEED * s_speed_pct * y / 10000;
    int drive_r = RF_FWD_SPEED * s_speed_pct * y / 10000;
    int rot     = TURN_SPEED * s_speed_pct * x / 10000;

    if (x > -12 && x < 12) {
        motor_set_lf(drive_l);
        motor_set_rf(drive_r);
        motor_set_rear(0);
    } else if (y > -12 && y < 12) {
        int v = (rot < 0) ? -rot : rot;
        if (x < 0) {
            motor_set_lf(-v);
            motor_set_rf(v);
            motor_set_rear(v);
        } else {
            motor_set_lf(v);
            motor_set_rf(-v);
            motor_set_rear(-v);
        }
    } else {
        /* 斜向：转向分量随前进/后退翻转，保证左右方向始终一致 */
        const int r = (y < 0) ? -rot : rot;
        motor_set_lf(drive_l + r);
        motor_set_rf(drive_r - r);
        motor_set_rear(0);
    }
    s_last_cmd_us = esp_timer_get_time();
}

/* ---------------- 云台 ---------------- */

static void servo_action(const char *act)
{
    if (act == NULL) {
        return;
    }
    if (strcmp(act, "up") == 0) {
        s_tilt -= SERVO_STEP_DEG;
        servo_set_tilt_deg(s_tilt);
    } else if (strcmp(act, "down") == 0) {
        s_tilt += SERVO_STEP_DEG;
        servo_set_tilt_deg(s_tilt);
    } else if (strcmp(act, "left") == 0) {
        s_pan += SERVO_STEP_DEG;
        servo_set_pan_deg(s_pan);
    } else if (strcmp(act, "right") == 0) {
        s_pan -= SERVO_STEP_DEG;
        servo_set_pan_deg(s_pan);
    } else if (strcmp(act, "reset") == 0) {
        s_pan = 90;
        s_tilt = 90;
        servo_set_pan_deg(s_pan);
        servo_set_tilt_deg(s_tilt);
    }
    s_pan = servo_pan_deg();
    s_tilt = servo_tilt_deg();
}

/* ---------------- 查询参数 ---------------- */

static void get_query_param(httpd_req_t *req, const char *key,
                            char *out, size_t outlen, const char *def)
{
    char query[CMD_BUF_LEN];
    strncpy(out, def, outlen);
    int qlen = httpd_req_get_url_query_len(req);
    if (qlen <= 0 || qlen >= (int)sizeof(query)) {
        return;
    }
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return;
    }
    if (httpd_query_key_value(query, key, out, outlen) != ESP_OK) {
        strncpy(out, def, outlen);
    }
}

/* ---------------- HTTP：控制服务（端口 80） ---------------- */

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char act[16] = "stop";
    get_query_param(req, "act", act, sizeof(act), "stop");
    run_action(act);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* 浏览器语音识别后的短动作接口：动作开始后按 ms 自动停车。 */
static void voice_stop_timer_cb(void *arg)
{
    (void)arg;
    motion_stop();
    ESP_LOGI(TAG, "voice timed action stopped");
}

static esp_err_t voicecmd_handler(httpd_req_t *req)
{
    char act[16] = "stop";
    char ms_str[16] = "0";
    get_query_param(req, "act", act, sizeof(act), "stop");
    get_query_param(req, "ms", ms_str, sizeof(ms_str), "0");

    int ms = atoi(ms_str);
    if (ms < 0) {
        ms = 0;
    } else if (ms > 5000) {
        ms = 5000;
    }

    run_action(act);
    if (s_voice_stop_timer) {
        esp_timer_stop(s_voice_stop_timer);
        if (ms > 0 && strcmp(act, "stop") != 0) {
            esp_timer_start_once(s_voice_stop_timer, (uint64_t)ms * 1000);
        }
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t speed_handler(httpd_req_t *req)
{
    char lvl[16] = "high";
    get_query_param(req, "level", lvl, sizeof(lvl), "high");
    if (strcmp(lvl, "med") == 0) {
        s_speed_pct = 80;
    } else if (strcmp(lvl, "ultra") == 0) {
        s_speed_pct = 125;
    } else {
        s_speed_pct = 100;
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t volume_handler(httpd_req_t *req)
{
    char vs[16] = "55";
    get_query_param(req, "pct", vs, sizeof(vs), "55");
    int pct = atoi(vs);
    if (pct < 0) {
        pct = 0;
    } else if (pct > 200) {
        pct = 200;
    }
    audio_uac_set_volume_percent((uint16_t)pct);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t joy_handler(httpd_req_t *req)
{
    char xs[16] = "0", ys[16] = "0";
    get_query_param(req, "x", xs, sizeof(xs), "0");
    get_query_param(req, "y", ys, sizeof(ys), "0");
    joy_motion(atoi(xs), atoi(ys));
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t servo_handler(httpd_req_t *req)
{
    char act[16] = "reset";
    get_query_param(req, "act", act, sizeof(act), "reset");
    servo_action(act);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* ---------------- 钢琴：有序全状态同步 ----------------
 *
 * 每个 WebSocket 消息都携带 session:sequence:mask（十六进制）。mask 的
 * bit0..bit20 对应音符 1..21。WebSocket 保证消息顺序，完整 mask 又允许
 * ESP32 在重连或丢掉旧会话时一次恢复全部和弦状态。
 */
static void piano_apply_mask_locked(uint32_t new_mask)
{
    new_mask &= PIANO_MASK_ALL;
    const uint32_t changed = s_piano_mask ^ new_mask;
    s_piano_mask = new_mask;

    for (uint8_t note = 1; note <= PIANO_NOTE_COUNT; note++) {
        const uint32_t bit = 1U << (note - 1U);
        if ((changed & bit) != 0) {
            audio_uac_note_event(note, (new_mask & bit) != 0);
        }
    }
}

static bool piano_sequence_is_newer(uint32_t incoming, uint32_t current)
{
    return (int32_t)(incoming - current) > 0;
}

static bool piano_parse_state(const char *text, uint32_t *session,
                              uint32_t *sequence, uint32_t *mask)
{
    char trailing = '\0';
    return sscanf(text, "%" SCNx32 ":%" SCNx32 ":%" SCNx32 "%c",
                  session, sequence, mask, &trailing) == 3;
}

static esp_err_t piano_ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "piano WebSocket connected");
        return ESP_OK;
    }

    httpd_ws_frame_t frame = { 0 };
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) {
        return err;
    }
    if (frame.len == 0 || frame.len >= 64) {
        ESP_LOGW(TAG, "ignored piano frame len=%u", (unsigned)frame.len);
        return ESP_OK;
    }

    uint8_t payload[64] = { 0 };
    frame.payload = payload;
    err = httpd_ws_recv_frame(req, &frame, frame.len);
    if (err != ESP_OK) {
        return err;
    }
    if (frame.type != HTTPD_WS_TYPE_TEXT) {
        return ESP_OK;
    }

    uint32_t session = 0;
    uint32_t sequence = 0;
    uint32_t mask = 0;
    if (!piano_parse_state((const char *)payload, &session, &sequence, &mask) ||
        session == 0 || (mask & ~PIANO_MASK_ALL) != 0) {
        ESP_LOGW(TAG, "ignored invalid piano state");
        return ESP_OK;
    }

    if (s_piano_state_mutex == NULL ||
        xSemaphoreTake(s_piano_state_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        /* 不因一次短暂竞争关闭长期连接；下一帧携带完整状态会自动恢复。 */
        return ESP_OK;
    }

    if (session != s_piano_session) {
        /* 新页面接管时先释放旧会话，避免残留和弦。 */
        piano_apply_mask_locked(0);
        s_piano_session = session;
        s_piano_sequence = sequence;
        piano_apply_mask_locked(mask);
    } else if (sequence == s_piano_sequence ||
               piano_sequence_is_newer(sequence, s_piano_sequence)) {
        /* 相同序号只作为心跳；更新的序号同时更新完整按键状态。 */
        if (sequence != s_piano_sequence) {
            s_piano_sequence = sequence;
            piano_apply_mask_locked(mask);
        }
    }
    s_piano_last_rx_us = esp_timer_get_time();
    xSemaphoreGive(s_piano_state_mutex);
    return ESP_OK;
}

static esp_err_t note_handler(httpd_req_t *req)
{
    char ns[16] = "0";
    char os[8] = "1";
    get_query_param(req, "note", ns, sizeof(ns), "0");
    get_query_param(req, "on", os, sizeof(os), "1");
    int note = atoi(ns);
    if (note < 0 || note > 21) {
        note = 0;
    }
    const bool pressed = (atoi(os) != 0) && note != 0;
    audio_uac_note_event((uint8_t)note, pressed);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t ping_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static const char INDEX_HTML[] =
"<!DOCTYPE html>\n"
"<html lang='zh'>\n"
"<head>\n"
"<meta charset='utf-8'>\n"
"<meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no'>\n"
"<title>小车遥控</title>\n"
"<style>\n"
"*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;}\n"
"body{margin:0;font-family:-apple-system,'PingFang SC',sans-serif;background:linear-gradient(160deg,#eaf6ff,#f6fbff);color:#1d2a3a;user-select:none;}\n"
".card{background:#fff;border-radius:18px;box-shadow:0 6px 18px rgba(31,88,146,.10);padding:12px;margin:10px auto;max-width:430px;}\n"
".top{display:flex;align-items:center;justify-content:space-between;}\n"
"h1{font-size:18px;margin:0;color:#155e9e;}\n"
"#status{font-size:12px;color:#4a90c2;}\n"
"#cam{width:100%;border-radius:12px;background:#0b1520;transform:rotate(180deg);min-height:140px;object-fit:contain;display:block;}\n"
".spdRow{display:flex;align-items:center;gap:10px;margin-top:10px;}\n"
"#spd{flex:1;accent-color:#155e9e;}\n"
"#spdLabel{font-size:14px;color:#155e9e;min-width:50px;text-align:right;font-weight:600;}\n"
".switch{display:flex;background:#e3f0fa;border-radius:12px;padding:3px;margin-top:10px;}\n"
".switch button{flex:1;border:none;background:transparent;color:#34688f;padding:8px 0;border-radius:10px;font-size:14px;}\n"
".switch button.on{background:#fff;color:#155e9e;box-shadow:0 2px 6px rgba(31,88,146,.15);font-weight:600;}\n"
".pianoGo{width:100%;margin-top:8px;border:none;border-radius:12px;padding:10px 0;color:#fff;font-size:14px;background:linear-gradient(180deg,#ffd479,#f0a93c);box-shadow:0 3px 0 rgba(20,50,80,.15);}\n"
".pad{display:grid;grid-template-columns:1fr 1fr 1fr;grid-template-rows:auto auto auto;gap:8px;margin-top:10px;}\n"
".btn{border:none;border-radius:12px;color:#fff;touch-action:none;cursor:pointer;box-shadow:0 3px 0 rgba(20,50,80,.18);}\n"
".btn:active{transform:translateY(1px);box-shadow:0 1px 0 rgba(20,50,80,.18);}\n"
".rotL{grid-area:1/1;}.fwd{grid-area:1/2;}.rotR{grid-area:1/3;}\n"
".strL{grid-area:2/1;}.bck{grid-area:2/2;}.strR{grid-area:2/3;}\n"
".bA{background:linear-gradient(180deg,#4ecdc4,#2e9e97);padding:14px 0;font-size:15px;}\n"
".bB{background:linear-gradient(180deg,#ff9f6e,#f07c3e);padding:14px 0;font-size:15px;}\n"
".bStop{background:linear-gradient(180deg,#ff8f8f,#e85d5d);padding:14px 0;font-size:15px;}\n"
".s1{background:linear-gradient(180deg,#7ec8ff,#4a9fe8);padding:14px 0;font-size:14px;}\n"
".s2{background:linear-gradient(180deg,#5cb8ff,#3a8fd6);padding:14px 0;font-size:14px;}\n"
".joyWrap{position:relative;width:230px;height:230px;margin:16px auto;background:#e9f3fb;border:2px solid #bfdcef;border-radius:50%;touch-action:none;}\n"
".knob{position:absolute;left:50%;top:50%;width:74px;height:74px;margin:-37px 0 0 -37px;background:radial-gradient(circle at 35% 30%,#7ec8ff,#2f7ec2);border-radius:50%;box-shadow:0 4px 12px rgba(21,94,158,.35);}\n"
".tip{font-size:11px;color:#7d96ab;text-align:center;line-height:1.6;margin-top:6px;}\n"
"h2{font-size:13px;color:#3c7aa6;margin:10px 0 4px;text-align:left;}\n"
".sv{display:grid;grid-template-columns:1fr 1fr 1fr;grid-template-rows:auto auto auto;gap:6px;}\n"
".sUp{grid-area:1/2;}.sLv{grid-area:2/1;}.sReset{grid-area:2/2;}.sRv{grid-area:2/3;}.sDn{grid-area:3/2;}\n"
".sv .btn{background:linear-gradient(180deg,#b7a6f6,#9378dd);padding:12px 0;font-size:14px;}\n"
".sv .r{background:linear-gradient(180deg,#9ad2c8,#6fb8aa);}\n"
".hide{display:none;}\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<div class='card'>\n"
"<div class='top'><h1>小车遥控</h1><div id='status'>连接中…</div></div>\n"
"<img id='cam' src='http://192.168.4.1:81/stream' alt='画面加载中…'>\n"
"<div class='spdRow'>\n"
"<span style='font-size:13px;color:#7d96ab;'>速度</span>\n"
"<input type='range' id='spd' min='0' max='2' step='1' value='1'>\n"
"<span id='spdLabel'>高速</span>\n"
"</div>\n"
"<div class='switch'>\n"
"<button id='modeBtn' class='on'>按钮控制</button>\n"
"<button id='modeJoy'>摇杆控制</button>\n"
"</div>\n"
"<button class='pianoGo' onclick='goPiano()'>🎹 钢琴</button>\n"
"<button class='pianoGo' onclick='goVoice()' style='background:linear-gradient(180deg,#9ad2c8,#6fb8aa)'>🎤 语音控制</button>\n"
"</div>\n"
"<div class='card'>\n"
"<div id='panelBtn'>\n"
"<div class='pad'>\n"
"<button class='btn s2 rotL' id='rotL'>左转</button>\n"
"<button class='btn bA fwd' id='fwd'>▲ 前进</button>\n"
"<button class='btn s2 rotR' id='rotR'>右转</button>\n"
"<button class='btn s1 strL' id='strL'>← 左移</button>\n"
"<button class='btn bB bck' id='bck'>▼ 后退</button>\n"
"<button class='btn s1 strR' id='strR'>右移 →</button>\n"
"</div>\n"
"</div>\n"
"<div id='panelJoy' class='hide'>\n"
"<div class='joyWrap' id='joy'><div class='knob' id='knob'></div></div>\n"
"<div class='tip'>按住拖动摇杆：上下=前进/后退，左右=转向，斜向自动混合；松手停止。</div>\n"
"</div>\n"
"<h2>云台控制</h2>\n"
"<div class='sv'>\n"
"<button class='btn sUp' id='sUp'>抬起</button>\n"
"<button class='btn r sLv' id='sL'>左转</button>\n"
"<button class='btn sReset' id='sReset'>复位</button>\n"
"<button class='btn r sRv' id='sR'>右转</button>\n"
"<button class='btn sDn' id='sDn'>低下</button>\n"
"</div>\n"
"<div class='tip'>电脑键盘：W/S/A/D 前进/后退/左移/右移，Q/E 左转/右转；"
"I/K/J/L 云台上下左右，R 复位。1.5 秒无指令自动停车。</div>\n"
"</div>\n"
"<script>\n"
"function send(u){fetch(u,{cache:'no-store'}).catch(function(){});}\n"
"function goPiano(){location.href='/piano';}\n"
"function goVoice(){location.href='https://192.168.4.1/voice';}\n"
"var spdLv=['med','high','ultra'],spdNm=['中速','高速','超高速'];\n"
"var spd=document.getElementById('spd'),spdLb=document.getElementById('spdLabel');\n"
"function onSpd(){var i=parseInt(spd.value,10);spdLb.textContent=spdNm[i];send('/speed?level='+spdLv[i]);}\n"
"spd.addEventListener('input',onSpd);\n"
"spd.addEventListener('change',onSpd);\n"
"function bindHold(id,act){\n"
"  var el=document.getElementById(id),timer=null;\n"
"  function start(e){e.preventDefault();send('/cmd?act='+act);timer=setInterval(function(){send('/cmd?act='+act);},120);}\n"
"  function end(){if(timer){clearInterval(timer);timer=null;}send('/cmd?act=stop');}\n"
"  el.addEventListener('touchstart',start,{passive:false});\n"
"  el.addEventListener('touchend',end);\n"
"  el.addEventListener('touchcancel',end);\n"
"  el.addEventListener('mousedown',start);\n"
"  el.addEventListener('mouseup',end);\n"
"  el.addEventListener('mouseleave',end);\n"
"}\n"
"bindHold('fwd','forward');\n"
"bindHold('bck','back');\n"
"bindHold('rotL','left');\n"
"bindHold('rotR','right');\n"
"bindHold('strL','strleft');\n"
"bindHold('strR','strright');\n"
"function bindServo(id,act,hold){\n"
"  var el=document.getElementById(id),timer=null;\n"
"  function start(e){e.preventDefault();send('/servo?act='+act);if(hold)timer=setInterval(function(){send('/servo?act='+act);},150);}\n"
"  function end(){if(timer){clearInterval(timer);timer=null;}}\n"
"  el.addEventListener('touchstart',start,{passive:false});\n"
"  el.addEventListener('touchend',end);\n"
"  el.addEventListener('touchcancel',end);\n"
"  if(hold){el.addEventListener('mousedown',start);el.addEventListener('mouseup',end);el.addEventListener('mouseleave',end);}\n"
"}\n"
"bindServo('sUp','up',true);\n"
"bindServo('sDn','down',true);\n"
"bindServo('sL','left',true);\n"
"bindServo('sR','right',true);\n"
"document.getElementById('sReset').onclick=function(){send('/servo?act=reset');};\n"
"var keyActs={'w':['cmd','forward'],'s':['cmd','back'],'a':['cmd','strleft'],'d':['cmd','strright'],'q':['cmd','left'],'e':['cmd','right'],'i':['servo','up'],'k':['servo','down'],'j':['servo','left'],'l':['servo','right'],'r':['servo','reset']};\n"
"var keyTimers={};\n"
"window.addEventListener('keydown',function(e){var k=e.key.toLowerCase();if(!keyActs[k]||e.repeat||e.ctrlKey||e.metaKey||e.altKey)return;e.preventDefault();var p=keyActs[k];send('/'+p[0]+'?act='+p[1]);if(p[1]!=='reset'){if(keyTimers[k])clearInterval(keyTimers[k]);keyTimers[k]=setInterval(function(){send('/'+p[0]+'?act='+p[1]);},100);}});\n"
"window.addEventListener('keyup',function(e){var k=e.key.toLowerCase();if(!keyActs[k])return;if(keyTimers[k]){clearInterval(keyTimers[k]);delete keyTimers[k];}if(keyActs[k][0]==='cmd')send('/cmd?act=stop');});\n"
"function setMode(m){\n"
"  var btn=m==='btn';\n"
"  document.getElementById('panelBtn').classList.toggle('hide',!btn);\n"
"  document.getElementById('panelJoy').classList.toggle('hide',btn);\n"
"  document.getElementById('modeBtn').classList.toggle('on',btn);\n"
"  document.getElementById('modeJoy').classList.toggle('on',!btn);\n"
"}\n"
"document.getElementById('modeBtn').onclick=function(){setMode('btn');};\n"
"document.getElementById('modeJoy').onclick=function(){setMode('joy');};\n"
"var joy=document.getElementById('joy'),knob=document.getElementById('knob'),joyOn=false;\n"
"function joyPos(e){\n"
"  var r=joy.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2;\n"
"  var dx=(e.touches?e.touches[0].clientX:e.clientX)-cx;\n"
"  var dy=(e.touches?e.touches[0].clientY:e.clientY)-cy;\n"
"  var max=r.width/2-8,dist=Math.sqrt(dx*dx+dy*dy);\n"
"  if(dist>max){dx=dx/dist*max;dy=dy/dist*max;}\n"
"  knob.style.left=(50+dx/max*50)+'%';\n"
"  knob.style.top=(50+dy/max*50)+'%';\n"
"  var x=Math.round(dx/max*100),y=Math.round(-dy/max*100);\n"
"  send('/joy?x='+x+'&y='+y);\n"
"}\n"
"function joyEnd(){joyOn=false;knob.style.left='50%';knob.style.top='50%';send('/cmd?act=stop');}\n"
"joy.addEventListener('touchstart',function(e){e.preventDefault();joyOn=true;joyPos(e);},{passive:false});\n"
"joy.addEventListener('touchmove',function(e){e.preventDefault();if(joyOn)joyPos(e);},{passive:false});\n"
"joy.addEventListener('touchend',joyEnd);\n"
"joy.addEventListener('touchcancel',joyEnd);\n"
"joy.addEventListener('mousedown',function(e){joyOn=true;joyPos(e);});\n"
"document.addEventListener('mousemove',function(e){if(joyOn)joyPos(e);});\n"
"document.addEventListener('mouseup',joyEnd);\n"
"setInterval(function(){\n"
"  fetch('/ping',{cache:'no-store'}).then(function(){document.getElementById('status').textContent='已连接 192.168.4.1';})\n"
"  .catch(function(){document.getElementById('status').textContent='连接断开';});\n"
"},1000);\n"
"</script>\n"
"</body>\n"
"</html>\n";

static const char PIANO_HTML[] =
"<!DOCTYPE html>\n"
"<html lang='zh'>\n"
"<head>\n"
"<meta charset='utf-8'>\n"
"<meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no'>\n"
"<title>钢琴</title>\n"
"<style>\n"
"*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;}\n"
"body{margin:0;font-family:-apple-system,'PingFang SC',sans-serif;background:linear-gradient(160deg,#eaf6ff,#f6fbff);color:#1d2a3a;user-select:none;padding:12px;}\n"
".card{background:#fff;border-radius:18px;box-shadow:0 6px 18px rgba(31,88,146,.10);padding:16px;margin:0 auto;max-width:500px;}\n"
"h1{font-size:20px;margin:0 0 4px;color:#155e9e;text-align:center;}\n"
".hint{font-size:12px;color:#7d96ab;text-align:center;margin-bottom:14px;}\n"
".volRow{display:flex;align-items:center;gap:10px;margin:10px 0 6px;}\n"
"#vol{flex:1;accent-color:#155e9e;}\n"
"#volLabel{font-size:14px;color:#155e9e;min-width:50px;text-align:right;font-weight:600;}\n"
".row{display:flex;gap:6px;margin-bottom:8px;}\n"
".key{flex:1;height:84px;border:none;border-radius:10px;background:linear-gradient(180deg,#fff,#e4eef6);color:#155e9e;box-shadow:0 3px 0 rgba(20,50,80,.18);touch-action:none;}\n"
".key b{display:block;font-size:22px;}\n"
".key span{font-size:11px;color:#8ba6bd;}\n"
".key.on{background:linear-gradient(180deg,#7ec8ff,#2f7ec2);color:#fff;box-shadow:0 1px 0 rgba(20,50,80,.18);transform:translateY(2px);}\n"
".back{display:block;text-align:center;margin-top:14px;color:#155e9e;font-size:14px;text-decoration:none;}\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<div class='card'>\n"
"<h1>钢琴</h1>\n"
"<div class='hint'>触屏点按发声；电脑键盘：高音 QWERTYU / 中音 ASDFGHJ / 低音 ZXCVBNM<br><span id='pianoConn'>音符通道连接中…</span></div>\n"
"<div class='volRow'><span style='font-size:13px;color:#7d96ab;'>音量</span>\n"
"<input type='range' id='vol' min='0' max='200' step='1' value='55'>\n"
"<span id='volLabel'>55%</span></div>\n"
"<div id='rows'></div>\n"
"<a class='back' href='/'>← 返回遥控</a>\n"
"</div>\n"
"<script>\n"
"var names=['C','D','E','F','G','A','B'];\n"
"var octaves=[5,4,3];\n"
"var keyRows=['qwertyu','asdfghj','zxcvbnm'];\n"
"function request(path){fetch(path,{cache:'no-store'}).catch(function(){});}\n"
"var conn=document.getElementById('pianoConn'),ws=null,retryTimer=null,leaving=false;\n"
"var sid=(((Date.now()>>>0)^((Math.random()*4294967295)>>>0))>>>0)||1,seq=0,mask=0;\n"
"function statePacket(){seq=(seq+1)>>>0;return sid.toString(16)+':'+seq.toString(16)+':'+mask.toString(16);}\n"
"function sendState(){if(ws&&ws.readyState===WebSocket.OPEN){try{ws.send(statePacket());}catch(e){}}}\n"
"function connectPiano(){\n"
"  if(leaving||(ws&&(ws.readyState===WebSocket.OPEN||ws.readyState===WebSocket.CONNECTING)))return;\n"
"  conn.textContent='音符通道连接中…';ws=new WebSocket('ws://'+location.host+'/piano-ws');\n"
"  ws.onopen=function(){conn.textContent='音符通道已连接';sendState();};\n"
"  ws.onclose=function(){conn.textContent='音符通道已断开，正在重连…';ws=null;if(!leaving){clearTimeout(retryTimer);retryTimer=setTimeout(connectPiano,250);}};\n"
"  ws.onerror=function(){conn.textContent='音符通道连接失败';};\n"
"}\n"
"var volEl=document.getElementById('vol'),volLb=document.getElementById('volLabel');\n"
"function onVol(){var pct=parseInt(volEl.value,10);volLb.textContent=pct<=100?pct+'%':('增强 '+pct+'%');request('/volume?pct='+pct);}\n"
"volEl.addEventListener('input',onVol);volEl.addEventListener('change',onVol);\n"
"var rows=document.getElementById('rows'),keyMap={},held={};\n"
"octaves.forEach(function(oct,r){\n"
"  var div=document.createElement('div');div.className='row';\n"
"  names.forEach(function(nm,c){\n"
"    var btn=document.createElement('button');btn.className='key';\n"
"    var f=(2-r)*7+c+1,key=keyRows[r][c];\n"
"    btn.innerHTML='<b>'+nm+oct+'</b><span>键 '+key.toUpperCase()+'</span>';\n"
"    function dn(e){e.preventDefault();if(held[key]!==undefined)return;held[key]=f;mask=(mask|(1<<(f-1)))>>>0;sendState();btn.classList.add('on');}\n"
"    function up(){if(held[key]===undefined)return;delete held[key];mask=(mask&~(1<<(f-1)))>>>0;sendState();btn.classList.remove('on');}\n"
"    btn.addEventListener('touchstart',dn,{passive:false});\n"
"    btn.addEventListener('touchend',up);\n"
"    btn.addEventListener('touchcancel',up);\n"
"    btn.addEventListener('mousedown',dn);\n"
"    btn.addEventListener('mouseup',up);\n"
"    btn.addEventListener('mouseleave',up);\n"
"    keyMap[key]=btn;div.appendChild(btn);\n"
"  });\n"
"  rows.appendChild(div);\n"
"});\n"
"window.addEventListener('keydown',function(e){var k=e.key.toLowerCase();if(keyMap[k]&&!e.repeat){keyMap[k].dispatchEvent(new MouseEvent('mousedown'));}});\n"
"window.addEventListener('keyup',function(e){var k=e.key.toLowerCase();if(keyMap[k]){keyMap[k].dispatchEvent(new MouseEvent('mouseup'));}});\n"
"function releaseAll(){Object.keys(held).forEach(function(k){if(keyMap[k])keyMap[k].classList.remove('on');});held={};mask=0;sendState();}\n"
"window.addEventListener('blur',releaseAll);\n"
"document.addEventListener('visibilitychange',function(){if(document.hidden)releaseAll();});\n"
"window.addEventListener('pagehide',function(){leaving=true;releaseAll();if(ws)ws.close();});\n"
"setInterval(sendState,250);\n"
"connectPiano();\n"
"</script>\n"
"</body>\n"
"</html>\n";

/* 语音控制页面。识别在手机/电脑浏览器完成，ESP32 只接收已识别的动作。 */
static const char VOICE_HTML[] =
"<!DOCTYPE html>\n"
"<html lang='zh'>\n"
"<head>\n"
"<meta charset='utf-8'>\n"
"<meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no'>\n"
"<title>语音控制</title>\n"
"<style>\n"
"*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;}\n"
"body{margin:0;font-family:-apple-system,'PingFang SC',sans-serif;background:linear-gradient(160deg,#e8f7f2,#f4fbf9);color:#1d2a3a;user-select:none;padding:16px;}\n"
".card{background:#fff;border-radius:18px;box-shadow:0 6px 18px rgba(31,88,146,.10);padding:18px;margin:0 auto;max-width:480px;}\n"
"h1{font-size:20px;margin:0 0 4px;color:#166d5f;text-align:center;}\n"
".hint{font-size:12px;color:#7d96ab;text-align:center;margin-bottom:14px;}\n"
"#st{font-size:14px;color:#2e8b7a;text-align:center;min-height:22px;margin:10px 0 4px;}\n"
"#heard{font-size:12px;color:#8ba6bd;text-align:center;min-height:18px;margin-bottom:10px;}\n"
".micWrap{display:flex;justify-content:center;margin:14px 0;}\n"
"#mic{width:150px;height:150px;border-radius:50%;border:none;background:radial-gradient(circle at 35% 30%,#6fd4bd,#2e9e8a);color:#fff;font-size:15px;box-shadow:0 6px 16px rgba(46,158,138,.35);touch-action:none;}\n"
"#mic.on{background:radial-gradient(circle at 35% 30%,#ff9a7a,#e05f3e);box-shadow:0 6px 16px rgba(224,95,62,.4);}\n"
"#mic b{display:block;font-size:38px;margin-bottom:4px;}\n"
".row{display:flex;gap:8px;margin-top:8px;}\n"
".btn{flex:1;border:none;border-radius:12px;padding:12px 0;color:#fff;font-size:15px;touch-action:none;box-shadow:0 3px 0 rgba(20,50,80,.16);}\n"
".stopBtn{background:linear-gradient(180deg,#ff8f8f,#e85d5d);}\n"
".cmdGrid{margin-top:12px;font-size:12px;color:#5d768b;line-height:1.8;}\n"
".cmdGrid td{padding:2px 6px;}\n"
".back{display:block;text-align:center;margin-top:16px;color:#2e8b7a;font-size:14px;text-decoration:none;}\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<div class='card'>\n"
"<h1>语音控制</h1>\n"
"<div class='hint'>点麦克风开始聆听，说完自动识别并执行；再点一次或说“停”结束</div>\n"
"<div id='st'>点麦克风开始</div>\n"
"<div id='heard'></div>\n"
"<div class='micWrap'><button id='mic'><b>🎤</b>开始聆听</button></div>\n"
"<div class='row'><button class='btn stopBtn' id='stopBtn'>■ 停车</button></div>\n"
"<div class='cmdGrid'><table>\n"
"<tr><td>前进</td><td>后退 / 倒车</td><td>向左转</td></tr>\n"
"<tr><td>向右转</td><td>向左行 / 左移</td><td>向右行 / 右移</td></tr>\n"
"<tr><td>一直走（前进约 1.5 秒）</td><td>停</td><td>再点麦克风可结束聆听</td></tr>\n"
"</table></div>\n"
"<a class='back' href='http://192.168.4.1/'>← 返回遥控</a>\n"
"</div>\n"
"<script>\n"
"var SR=window.SpeechRecognition||window.webkitSpeechRecognition;\n"
"var mic=document.getElementById('mic'),st=document.getElementById('st'),heard=document.getElementById('heard');\n"
"var rec=null,listening=false,cur='',fired=false,lastT=0,manualStop=false;\n"
"function send(u){fetch(u,{cache:'no-store'}).catch(function(){});}\n"
"function setSt(s){st.textContent=s;}\n"
"var acts=[\n"
" ['stop',0,['停','停车','停止','刹车','停下']],\n"
" ['forward',1000,['前进','往前走','向前走','直行']],\n"
" ['back',1000,['后退','倒车','向后走']],\n"
" ['strleft',800,['向左行','左行','向左移','左移']],\n"
" ['strright',800,['向右行','右行','向右移','右移']],\n"
" ['left',250,['向左转','左转','向左','往左转']],\n"
" ['right',250,['向右转','右转','向右','往右转']],\n"
" ['forward',1500,['一直走']]\n"
"];\n"
"function runCmd(t){\n"
"  for(var i=0;i<acts.length;i++){\n"
"    for(var j=0;j<acts[i][2].length;j++){\n"
"      if(t.indexOf(acts[i][2][j])>=0){\n"
"        var a=acts[i][0],ms=acts[i][1];\n"
"        send(ms>0?'/voicecmd?act='+a+'&ms='+ms:'/cmd?act='+a);\n"
"        fired=true;setSt(a==='stop'?'已停车':'执行：'+acts[i][2][j]);\n"
"        heard.textContent='识别：'+t;return true;\n"
"      }\n"
"    }\n"
"  }\n"
"  return false;\n"
"}\n"
"function stopRec(){manualStop=true;if(rec){try{rec.stop();}catch(e){}}listening=false;mic.classList.remove('on');mic.innerHTML='<b>🎤</b>开始聆听';}\n"
"function toggleMic(){var n=Date.now();if(n-lastT<400)return;lastT=n;if(listening)stopRec();else startRec();}\n"
"function startRec(){\n"
"  if(listening)return;if(!SR){setSt('请使用 Chrome/Edge 打开，并允许麦克风权限');return;}\n"
"  rec=new SR();rec.lang='zh-CN';rec.continuous=true;rec.interimResults=true;\n"
"  cur='';fired=false;manualStop=false;heard.textContent='';\n"
"  rec.onresult=function(e){var t='';for(var i=0;i<e.results.length;i++)t+=e.results[i][0].transcript;\n"
"    if(e.results[e.results.length-1].isFinal)cur=t;heard.textContent='听到：'+t;if(cur&&!fired)runCmd(cur);};\n"
"  rec.onerror=function(e){if(e.error==='not-allowed')setSt('未授权麦克风');else if(e.error==='no-speech')setSt('没听清，再试一次');else setSt('识别出错：'+e.error);};\n"
"  rec.onend=function(){listening=false;mic.classList.remove('on');mic.innerHTML='<b>🎤</b>开始聆听';if(!fired&&!manualStop)setSt('没听清，再试一次');rec=null;};\n"
"  listening=true;mic.classList.add('on');mic.innerHTML='<b>🔴</b>聆听中…';setSt('请说出指令');try{rec.start();}catch(e){}\n"
"}\n"
"mic.addEventListener('touchstart',function(e){e.preventDefault();toggleMic();},{passive:false});\n"
"mic.addEventListener('mousedown',function(e){e.preventDefault();toggleMic();});\n"
"if(!SR)setSt('请使用 Chrome/Edge 打开，并允许麦克风权限');\n"
"function stopAll(){send('/cmd?act=stop');setSt('已停车');}\n"
"document.getElementById('stopBtn').addEventListener('touchstart',function(e){e.preventDefault();stopAll();},{passive:false});\n"
"document.getElementById('stopBtn').addEventListener('mousedown',function(e){e.preventDefault();stopAll();});\n"
"window.addEventListener('blur',stopAll);\n"
"</script>\n"
"</body>\n"
"</html>\n";

static esp_err_t voice_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, VOICE_HTML);
    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    uvc_stream_set_paused(false);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, INDEX_HTML);
    return ESP_OK;
}

static esp_err_t piano_handler(httpd_req_t *req)
{
    uvc_stream_set_paused(true);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, PIANO_HTML);
    return ESP_OK;
}

static void register_uris(httpd_handle_t server)
{
    httpd_uri_t cmd   = { .uri = "/cmd",   .method = HTTP_GET,
                          .handler = cmd_handler, .user_ctx = NULL };
    httpd_uri_t speed = { .uri = "/speed", .method = HTTP_GET,
                          .handler = speed_handler, .user_ctx = NULL };
    httpd_uri_t volume = { .uri = "/volume", .method = HTTP_GET,
                           .handler = volume_handler, .user_ctx = NULL };
    httpd_uri_t joy   = { .uri = "/joy",   .method = HTTP_GET,
                          .handler = joy_handler, .user_ctx = NULL };
    httpd_uri_t servo = { .uri = "/servo", .method = HTTP_GET,
                          .handler = servo_handler, .user_ctx = NULL };
    httpd_uri_t note  = { .uri = "/note", .method = HTTP_GET,
                          .handler = note_handler, .user_ctx = NULL };
    httpd_uri_t voicecmd = { .uri = "/voicecmd", .method = HTTP_GET,
                             .handler = voicecmd_handler, .user_ctx = NULL };
    httpd_uri_t ping  = { .uri = "/ping",  .method = HTTP_GET,
                          .handler = ping_handler, .user_ctx = NULL };
    httpd_uri_t idx   = { .uri = "/",      .method = HTTP_GET,
                          .handler = index_handler, .user_ctx = NULL };
    httpd_uri_t piano = { .uri = "/piano", .method = HTTP_GET,
                          .handler = piano_handler, .user_ctx = NULL };
    httpd_uri_t voice = { .uri = "/voice", .method = HTTP_GET,
                          .handler = voice_handler, .user_ctx = NULL };
    httpd_uri_t piano_ws = { .uri = "/piano-ws", .method = HTTP_GET,
                             .handler = piano_ws_handler, .user_ctx = NULL,
                             .is_websocket = true };
    httpd_register_uri_handler(server, &cmd);
    httpd_register_uri_handler(server, &speed);
    httpd_register_uri_handler(server, &volume);
    httpd_register_uri_handler(server, &joy);
    httpd_register_uri_handler(server, &servo);
    httpd_register_uri_handler(server, &note);
    httpd_register_uri_handler(server, &voicecmd);
    httpd_register_uri_handler(server, &ping);
    httpd_register_uri_handler(server, &idx);
    httpd_register_uri_handler(server, &piano);
    httpd_register_uri_handler(server, &voice);
    httpd_register_uri_handler(server, &piano_ws);
}

/* HTTPS 语音服务：给浏览器提供更容易获得麦克风权限的安全上下文。 */
static void register_https_uris(httpd_handle_t server)
{
    httpd_uri_t cmd = { .uri = "/cmd", .method = HTTP_GET,
                        .handler = cmd_handler, .user_ctx = NULL };
    httpd_uri_t voicecmd = { .uri = "/voicecmd", .method = HTTP_GET,
                             .handler = voicecmd_handler, .user_ctx = NULL };
    httpd_uri_t ping = { .uri = "/ping", .method = HTTP_GET,
                         .handler = ping_handler, .user_ctx = NULL };
    httpd_uri_t voice = { .uri = "/voice", .method = HTTP_GET,
                          .handler = voice_handler, .user_ctx = NULL };
    httpd_register_uri_handler(server, &cmd);
    httpd_register_uri_handler(server, &voicecmd);
    httpd_register_uri_handler(server, &ping);
    httpd_register_uri_handler(server, &voice);
}

/* ---------------- HTTP：MJPEG 视频流（端口 81） ---------------- */

#define BOUNDARY "frame"

static esp_err_t stream_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=" BOUNDARY);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char hdr[128];
    uint32_t sent = 0;
    while (1) {
        if (!uvc_stream_wait_frame(3000)) {
            continue;
        }
        const uint8_t *data = NULL;
        size_t len = 0;
        if (!uvc_stream_lock_frame(&data, &len) || len == 0) {
            uvc_stream_unlock_frame();
            continue;
        }
        int n = snprintf(hdr, sizeof(hdr),
                         "--%s\r\nContent-Type: image/jpeg\r\n"
                         "Content-Length: %u\r\n\r\n",
                         BOUNDARY, (unsigned)len);
        esp_err_t err = httpd_resp_send_chunk(req, hdr, n);
        if (err == ESP_OK) {
            err = httpd_resp_send_chunk(req, (const char *)data, len);
        }
        if (err == ESP_OK) {
            err = httpd_resp_send_chunk(req, "\r\n", 2);
        }
        uvc_stream_unlock_frame();
        if (err != ESP_OK) {
            break;
        }
        sent++;
        if ((sent % 25) == 1) {
            ESP_LOGI(TAG, "stream sent frame #%u len=%u",
                     (unsigned)sent, (unsigned)len);
        }
    }
    return ESP_FAIL;
}

static httpd_handle_t start_http_server(uint16_t port)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    config.ctrl_port = 32768 + (port - HTTP_PORT);
    config.max_uri_handlers = 16;
    config.lru_purge_enable = true;
    config.stack_size = 8192;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "httpd start failed on port %d", port);
        return NULL;
    }
    return server;
}

static httpd_handle_t start_https_server(void)
{
    httpd_handle_t server = NULL;
    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();

    extern const unsigned char cert_pem_start[] asm("_binary_cert_pem_start");
    extern const unsigned char cert_pem_end[]   asm("_binary_cert_pem_end");
    extern const unsigned char key_pem_start[]  asm("_binary_key_pem_start");
    extern const unsigned char key_pem_end[]    asm("_binary_key_pem_end");

    conf.port_secure = HTTPS_PORT;
    conf.transport_mode = HTTPD_SSL_TRANSPORT_SECURE;
    conf.servercert = cert_pem_start;
    conf.servercert_len = cert_pem_end - cert_pem_start;
    conf.prvtkey_pem = key_pem_start;
    conf.prvtkey_len = key_pem_end - key_pem_start;
    conf.httpd.max_uri_handlers = 16;
    conf.httpd.ctrl_port = 32768 + (HTTPS_PORT - HTTP_PORT);
    conf.httpd.stack_size = 8192;
    if (httpd_ssl_start(&server, &conf) != ESP_OK) {
        ESP_LOGE(TAG, "https server start failed on port %d", HTTPS_PORT);
        return NULL;
    }
    ESP_LOGI(TAG, "https server on port %d", HTTPS_PORT);
    return server;
}

/* ---------------- WiFi AP ---------------- */

static void wifi_ap_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = 0,
            .password = AP_PASS,
            .channel = AP_CHANNEL,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP started: SSID=%s IP=192.168.4.1", AP_SSID);
}

/* ---------------- 断连保护 ---------------- */

static void watchdog_task(void *arg)
{
    (void)arg;
    while (1) {
        if (esp_timer_get_time() - s_last_cmd_us > (int64_t)AUTO_STOP_MS * 1000) {
            motor_stop_all();
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void piano_watchdog_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (s_piano_state_mutex == NULL ||
            xSemaphoreTake(s_piano_state_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
            continue;
        }
        const int64_t elapsed_us = esp_timer_get_time() - s_piano_last_rx_us;
        if (s_piano_mask != 0 &&
            elapsed_us > (int64_t)PIANO_LINK_TIMEOUT_MS * 1000) {
            ESP_LOGW(TAG, "piano link timeout; releasing all notes");
            piano_apply_mask_locked(0);
        }
        xSemaphoreGive(s_piano_state_mutex);
    }
}

/* ---------------- 主程序 ---------------- */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    motor_init();
    motor_stop_all();
    servo_init();

    const esp_timer_create_args_t vstop_args = {
        .callback = voice_stop_timer_cb,
        .name = "voice_stop",
    };
    ESP_ERROR_CHECK(esp_timer_create(&vstop_args, &s_voice_stop_timer));
    s_piano_state_mutex = xSemaphoreCreateMutex();
    if (s_piano_state_mutex == NULL) {
        ESP_LOGE(TAG, "failed to create piano state mutex");
        return;
    }

    wifi_ap_start();

    esp_err_t uerr = uvc_stream_init();
    if (uerr != ESP_OK) {
        ESP_LOGW(TAG, "camera init failed (%s), control only",
                 esp_err_to_name(uerr));
    }

    httpd_handle_t ctl = start_http_server(HTTP_PORT);
    if (ctl) {
        register_uris(ctl);
        ESP_LOGI(TAG, "control server on port %d", HTTP_PORT);
    }

    httpd_handle_t vid = start_http_server(STREAM_PORT);
    if (vid) {
        httpd_uri_t stream = { .uri = "/stream", .method = HTTP_GET,
                               .handler = stream_handler, .user_ctx = NULL };
        httpd_register_uri_handler(vid, &stream);
        ESP_LOGI(TAG, "video stream on port %d/stream", STREAM_PORT);
    }

    httpd_handle_t secure = start_https_server();
    if (secure) {
        register_https_uris(secure);
    }

    xTaskCreate(watchdog_task, "wdt_car", 2048, NULL, 5, NULL);
    xTaskCreate(piano_watchdog_task, "wdt_piano", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "ready: join %s then open http://192.168.4.1/", AP_SSID);
    ESP_LOGI(TAG, "voice page: https://192.168.4.1/voice");
}
