# Line Follow + Ultrasonic Avoidance (ESP-IDF)

这是 `line_follower_muy_bien_ultrasonic(7).ino` 的 ESP-IDF 版本，适合在 VS Code 的 ESP-IDF 扩展中打开、编译和烧录。Arduino 原文件没有被覆盖，仍可用 Arduino IDE 保留运行。

## 功能

- 四路红外循迹：16 状态查表、弯道降速、丢线停车后按最近压线方向找线。
- HC-SR04 超声避障：停车 → 三轮左移 → 前进越过障碍 → 三轮右移找回黑线 → 恢复循迹。
- LQ_TFT18SPIV33（ILI9163）显示左/右前轮、后轮 PWM 指令，超声距离和四路黑白状态。
- BOOT 键运行时启动/停止；烧录后默认停止。
- 串口 115200：`g` 开始，`s` 停止，`b` 打印红外位图，`u` 测一次距离。

## 打开和烧录

在 VS Code 中打开本文件夹（不是只打开 `main` 文件夹），选择 ESP32-S3 后执行 Build、Flash、Monitor。也可以在 ESP-IDF 终端中运行：

```text
idf.py build
idf.py -p /dev/cu.usbserial-0001 flash monitor
```

实际串口名称如果不同，在 `.vscode/settings.json` 中修改 `idf.port`，或直接替换上面命令中的端口。

## 接线

| 模块 | GPIO |
|---|---|
| 左前轮 IN1/IN2/PWM | 8 / 9 / 10 |
| 后轮 IN1/IN2/PWM | 12 / 13 / 14 |
| 右前轮 IN1/IN2/PWM | 15 / 16 / 17 |
| 电机 STBY | 11 |
| 红外 OUT4/OUT3/OUT2/OUT1（从车头左到右） | 7 / 6 / 5 / 4 |
| HC-SR04 TRIG/ECHO | 18 / 21 |
| TFT SCLK/MOSI/CS/DC/RST | 39 / 40 / 41 / 42 / 47 |
| BOOT | GPIO0 |

HC-SR04 的 ECHO 必须先分压或电平转换到 3.3 V，再接 GPIO21。

## 常调参数

所有主要参数在 `main/main.c` 顶部：

- 直行速度：`LF_SPEED`、`RF_SPEED`
- 弯道降速：`CURVE_SPEED_SCALE`
- 丢线找线：`LOST_STOP_MS`、`LOST_TURN_SPEED`、`LOST_TURN_MAX_MS`
- 避障：`OBSTACLE_TRIGGER_CM`、`STRAFE_LF_SPEED`、`STRAFE_RF_SPEED`、`STRAFE_REAR_SPEED`、`PASS_FORWARD_MS`
- 电机正反方向：`LF_DIR_SIGN`、`RF_DIR_SIGN`、`REAR_DIR_SIGN`

屏幕上的 L/R/B 是当前 PWM 指令百分比，不是编码器测得的真实 RPM。
