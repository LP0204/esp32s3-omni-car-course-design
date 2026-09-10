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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
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

static const char *TAG = "car_remote";
static int64_t s_last_cmd_us = 0;
static int s_speed_pct = 100;   /* 中=80 高=100 超=125 */
static int s_pan = 90;
static int s_tilt = 90;

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
"<div class='hint'>触屏点按发声；电脑键盘：高音 QWERTYU / 中音 ASDFGHJ / 低音 ZXCVBNM</div>\n"
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
"/* Keep note commands in browser order. Separate fetches can otherwise\n"
" * arrive out of order when several keys are pressed together. */\n"
"var noteQueue=Promise.resolve();\n"
"function send(n,down){var path='/note?note='+n+'&on='+(down===false?0:1);noteQueue=noteQueue.then(function(){return fetch(path,{cache:'no-store'});}).catch(function(){});}\n"
"var volEl=document.getElementById('vol'),volLb=document.getElementById('volLabel');\n"
"function onVol(){var pct=parseInt(volEl.value,10);volLb.textContent=pct<=100?pct+'%':('增强 '+pct+'%');request('/volume?pct='+pct);}\n"
"volEl.addEventListener('input',onVol);volEl.addEventListener('change',onVol);\n"
"var rows=document.getElementById('rows'),keyMap={},held={},order=[],cur=0;\n"
"octaves.forEach(function(oct,r){\n"
"  var div=document.createElement('div');div.className='row';\n"
"  names.forEach(function(nm,c){\n"
"    var btn=document.createElement('button');btn.className='key';\n"
"    var f=(2-r)*7+c+1,key=keyRows[r][c];\n"
"    btn.innerHTML='<b>'+nm+oct+'</b><span>键 '+key.toUpperCase()+'</span>';\n"
"    function dn(e){e.preventDefault();if(held[key]!==undefined)return;held[key]=f;order.push(key);cur=f;send(f,true);btn.classList.add('on');}\n"
"    function up(){if(held[key]===undefined)return;delete held[key];order=order.filter(function(k){return k!==key;});send(f,false);if(cur===f){var k=order.length?order[order.length-1]:null;cur=k===null?0:held[k];if(cur)send(cur,true);}btn.classList.remove('on');}\n"
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
"window.addEventListener('blur',function(){if(!order.length)return;order.slice().forEach(function(k){if(keyMap[k])keyMap[k].dispatchEvent(new MouseEvent('mouseup'));});});\n"
"setInterval(function(){if(cur)send(cur,true);},800);\n"
"</script>\n"
"</body>\n"
"</html>\n";

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
    httpd_uri_t ping  = { .uri = "/ping",  .method = HTTP_GET,
                          .handler = ping_handler, .user_ctx = NULL };
    httpd_uri_t idx   = { .uri = "/",      .method = HTTP_GET,
                          .handler = index_handler, .user_ctx = NULL };
    httpd_uri_t piano = { .uri = "/piano", .method = HTTP_GET,
                          .handler = piano_handler, .user_ctx = NULL };
    httpd_register_uri_handler(server, &cmd);
    httpd_register_uri_handler(server, &speed);
    httpd_register_uri_handler(server, &volume);
    httpd_register_uri_handler(server, &joy);
    httpd_register_uri_handler(server, &servo);
    httpd_register_uri_handler(server, &note);
    httpd_register_uri_handler(server, &ping);
    httpd_register_uri_handler(server, &idx);
    httpd_register_uri_handler(server, &piano);
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

    xTaskCreate(watchdog_task, "wdt_car", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "ready: join %s then open http://192.168.4.1/", AP_SSID);
}
