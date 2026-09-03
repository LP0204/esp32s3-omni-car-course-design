# ESP32-S3 摄像头彩色 80×60 图像测试

本工程仅用于确认 ESP-Claw 三合一摄像头（JQ-CAM12-720D-V1）可以输出**彩色**图像。

`UVC 摄像头 → ESP32-S3 MJPEG 解码 → 80×60 RGB565 彩色帧 → 串口 → Mac 浏览器`

它不会初始化电机、舵机、超声波或 TFT 显示屏。摄像头仍以 720p 采集，ESP32-S3 将其缩小为 80×60 彩色画面后传给电脑；选择这个尺寸是为了在 921600 baud 串口下保持实时性。

## 接线

| 摄像头 | ESP32-S3 |
|---|---|
| USB D− | GPIO19 |
| USB D＋ | GPIO20 |
| 5V | 5V |
| GND | GND |

电脑通过开发板的 USB-UART/CH340 串口连接。

## 使用

1. 单独在 VS Code 打开本文件夹。
2. 关闭 **Monitor Device**，否则它会占用串口。
3. 按 `⌘⇧P`，选择 **Tasks: Run Task**。
4. 选择 **Camera: Flash and Open Preview**。

任务会烧录固件、启动电脑端查看器，并自动在 Safari 打开 `http://127.0.0.1:8766/`。终端保持运行是正常的：它正在持续接收彩色图像；按 `Ctrl+C` 停止。
