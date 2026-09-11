# car_wifi_remote

基于 ESP32-S3 的 Wi‑Fi 遥控小车工程，包含：

- 手机/平板网页遥控三轮全向底盘；
- USB 摄像头实时 MJPEG 画面；
- GPIO4/GPIO5 两路 MG90S 云台舵机控制；
- 摄像头模块内置扬声器的 Wi‑Fi 钢琴页面；
- 浏览器语音识别控制页面（中文动作指令）；
- 21 个音符和弦、连续音量调节、按键释放渐弱。

## 工作方式

ESP32-S3 启动后建立自己的 Wi‑Fi 热点，不依赖路由器或手机热点。手机、平板或电脑连接小车热点后，用浏览器访问：

```text
http://192.168.4.1/
```

控制网页运行在 80 端口，摄像头 MJPEG 流运行在 81 端口。进入钢琴页面时，程序暂停摄像头 USB 取流，把资源优先留给音频；返回遥控页面后自动恢复摄像头。

语音页面运行在 HTTPS 443 端口，地址为：

```text
https://192.168.4.1/voice
```

第一次打开时浏览器会提示本地自签名证书，选择继续访问即可；随后允许浏览器使用麦克风。语音识别由手机/电脑浏览器完成，ESP32 只接收识别后的动作，不需要把企业 Wi‑Fi 账号配置到小车中。Chrome/Edge 的中文语音识别通常需要浏览器具备外网访问能力；如果手机只连接小车热点且没有外网，页面仍能打开，但识别服务可能不可用。

## 接线

以下接线以本工程当前 `main` 驱动为准：

| 功能 | ESP32-S3 GPIO |
|---|---:|
| 左前轮 IN1 / IN2 / PWM | GPIO8 / GPIO9 / GPIO10 |
| 后轮 IN1 / IN2 / PWM | GPIO12 / GPIO13 / GPIO14 |
| 右前轮 IN1 / IN2 / PWM | GPIO15 / GPIO16 / GPIO17 |
| 电机驱动 STBY | GPIO11 |
| 云台左右舵机（PAN） | GPIO5 |
| 云台上下舵机（TILT） | GPIO4 |
| USB 摄像头 D− / D+ | GPIO19 / GPIO20 |
| 摄像头供电 | 5V + GND |

摄像头方盒内的扬声器由 USB 音频接口驱动，不需要在本工程外另接扬声器线。舵机和摄像头应使用稳定的 5V 供电，并与 ESP32-S3 共地。

## 编译与烧录

在工程根目录执行：

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbserial-0001 flash
```

串口名称会因电脑和 USB 转串口芯片不同而变化。macOS 可以先查看：

```bash
ls /dev/cu.*
```

烧录完成后可以打开串口监视器查看启动日志：

```bash
idf.py -p /dev/cu.usbserial-0001 monitor
```

如果使用 VS Code ESP-IDF 扩展，也可以直接执行 **Build Project** 和 **Flash Device**。

工程启用了 ESP-IDF 标准的 1.5 MB 单应用分区；执行 Flash Device 时会同时更新分区表，无需单独操作。

## 使用步骤

1. 给小车上电，等待串口出现 Wi‑Fi AP 启动信息；
2. 手机连接热点 `CAR-REMOTE`，密码为 `12345678`；
3. 浏览器访问 `http://192.168.4.1/`；
4. 按住方向按钮移动小车，松开后停止；
5. 在“云台控制”区域按住按钮控制上下、左右和复位；
6. 点击“钢琴”进入 `http://192.168.4.1/piano`。
7. 点击“语音控制”，或直接打开 `https://192.168.4.1/voice`，说“前进、后退、左转、右转、左移、右移、停车”等指令。

小车指令超过 1.5 秒没有刷新时会自动停车，时间常量为 `AUTO_STOP_MS`。

电脑键盘控制：

| 按键 | 功能 |
|---|---|
| W / S | 前进 / 后退 |
| A / D | 左移 / 右移 |
| Q / E | 原地左转 / 原地右转 |
| I / K | 云台抬起 / 低下 |
| J / L | 云台左转 / 右转 |
| R | 云台复位 |

## 钢琴页面

钢琴页面提供低音、中音、高音三排共 21 个音符，可同时按下多个按键演奏和弦。电脑键盘布局为：

- 高音：`Q W E R T Y U`；
- 中音：`A S D F G H J`；
- 低音：`Z X C V B N M`。

当前音符频率如下：

| 音区 | C | D | E | F | G | A | B |
|---|---:|---:|---:|---:|---:|---:|---:|
| 低音 | 131 | 147 | 165 | 175 | 196 | 220 | 248 Hz |
| 中音 | 262 | 294 | 330 | 349 | 392 | 440 | 494 Hz |
| 高音 | 523 | 587 | 659 | 698 | 784 | 880 | 988 Hz |

松开按键后音符保持约 300 ms 并逐渐减弱；新按键可以立即加入和弦。当前三排默认振幅分别为：高音 `QWERTYU` 7000、中音 `ASDFGHJ` 9000、低音 `ZXCVBNM` 10000。音量滑块范围为 0～140%：0 为静音，100% 为设备音量范围，超过 100% 为受限的软件增益。为减少大音量时高次谐波造成的“音调变高”听感，混音峰值限制为 24000，低音峰值限制为 16000。

钢琴按键通过一条 WebSocket 长连接传输。每次变化发送当前全部 21 键状态、页面会话号和递增序号，避免快速输入时 HTTP 请求排队或乱序；页面每 250 ms 同步一次完整状态，链路中断超过 1 秒时 ESP32 自动释放所有音符。页面上会显示“音符通道已连接”，看到该提示后再开始演奏。仍被同时按住的按键会组成和弦；松键后的声音渐弱 300 ms，但在新按键按下时会立即清除已有尾音，避免快速演奏时声音堆积。

## 可调参数

主要参数位置：

- 电机速度：`main/main.c` 顶部的 `LF_FWD_SPEED`、`RF_FWD_SPEED`、`TURN_SPEED`；
- 云台单次步进角：`main/main.c` 中的 `SERVO_STEP_DEG`；
- 舵机脉宽范围、PWM 频率：`main/servo.c` 中的 `PULSE_MIN_US`、`PULSE_MAX_US`、`SERVO_FREQ_HZ`；
- 三排基础振幅、低音峰值、混音峰值和音频缓冲：`main/audio_uac.c` 顶部的 `AUDIO_AMPLITUDE_HIGH`、`AUDIO_AMPLITUDE_MIDDLE`、`AUDIO_AMPLITUDE_LOW`、`AUDIO_LOW_PEAK_LIMIT`、`AUDIO_CHORD_PEAK_LIMIT`、`AUDIO_DEVICE_BUFFER_BYTES`、`AUDIO_PREBUFFER_CHUNKS`。

## HTTP 接口

```text
GET /cmd?act=forward|back|left|right|strleft|strright|stop
GET /joy?x=-100..100&y=-100..100
GET /speed?level=med|high|ultra
GET /servo?act=up|down|left|right|reset
GET /note?note=1..21&on=1|0
GET /volume?pct=0..140
GET /stream                 （81 端口）
GET /ping
GET /voicecmd?act=forward|back|left|right|strleft|strright|stop&ms=0..5000
WS  /piano-ws               （21键有序全状态同步）
```

语音页同时在 HTTPS 443 端口提供 `/voice`、`/voicecmd`、`/cmd` 和 `/ping`，用于满足浏览器麦克风权限和跨页面指令请求。

## 目录说明

```text
main/main.c          Wi‑Fi AP、网页、HTTP 路由和动作映射
main/motor.c         三轮电机驱动
main/servo.c         两路舵机 PWM 驱动
main/uvc_stream.c    USB 摄像头取流与 MJPEG 输出
main/audio_uac.c     USB 音频识别、PCM 合成与缓冲
phone/control.html   遥控网页原型
phone/piano.html     钢琴网页原型（与固件内嵌页面保持同步）
（语音页面由 `main/main.c` 内嵌，烧录后直接由 ESP32 提供）
```
