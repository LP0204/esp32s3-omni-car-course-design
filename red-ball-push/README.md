# Red Ball Push

基于 ESP32-S3、ESP-Claw 三合一摄像头和三轮底盘的红球识别与推球工程。

摄像头初始化、MJPEG 解码与浏览器图像预览直接保留自已验证可用的 `camera-color-viewer`；电机控制被放到独立任务，不改动该采集链路。

## 逻辑

1. 上电后小车保持停止，摄像头照常向浏览器发送 80×60 彩色图。
2. 按一次 **BOOT**：小车以 20% 功率向左原地慢速旋转，寻找红球。
3. 对每帧图像仅计算红度 `R - (G+B)/2 > 阈值` 且 `R > 90` 的红色像素数和红色区域横向中心；浏览器中唯一的红度滑块会同步更新单片机阈值。
4. 连续三帧达到 `RED_PIXELS_TO_TRIGGER` 后进入对准状态：红色在画面左侧就左转，在右侧就右转；接近中线时自动降速。
5. 中心越过中线时误差符号会改变，小车立即反向微调，避免继续过冲。连续三帧位于中线 ±3 像素内才算对准。
6. 对准后两前轮向前运动 `PUSH_FORWARD_MS`，停止 `PUSH_PAUSE_MS`，再向后运动 `PUSH_RETURN_MS`，最后停止。
7. 运行中再次按 BOOT 会立即停止。

直行只使用左右前轮，后轮始终停止。

## 可调参数

均位于 `main/main.c` 顶部：

- `SEARCH_LEFT_POWER`：找球时左转功率，默认 20%。
- `ALIGN_COARSE_POWER` / `ALIGN_FINE_POWER`：对准时粗调/微调功率，默认 23%/18%。
- `CENTER_DEADBAND_PX`：画面中线允许误差，默认 ±3 像素。
- `LEFT_STRAIGHT_POWER_PERCENT`：左前轮前进和后退功率，默认 52%。
- `RIGHT_STRAIGHT_POWER_PERCENT`：右前轮前进和后退功率，默认 52%。
- `PUSH_FORWARD_MS` / `PUSH_PAUSE_MS` / `PUSH_RETURN_MS`：前进、停顿和后退时间。

## 接线

| 功能 | ESP32-S3 |
|---|---|
| 摄像头 D− / D+ | GPIO19 / GPIO20 |
| 左前轮 IN1 / IN2 / PWM | GPIO8 / GPIO9 / GPIO10 |
| 后轮 IN1 / IN2 / PWM | GPIO12 / GPIO13 / GPIO14 |
| 右前轮 IN1 / IN2 / PWM | GPIO15 / GPIO16 / GPIO17 |
| 电机 STBY | GPIO11 |

用 VS Code 的 **Flash Device** 烧录。用 `tools/flash_and_view.sh` 可烧录后自动打开图像预览。
