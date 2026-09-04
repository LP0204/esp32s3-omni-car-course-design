# Red Ball Push

基于 ESP32-S3、ESP-Claw 三合一摄像头和三轮底盘的红球识别与推球工程。

摄像头初始化、MJPEG 解码与浏览器图像预览直接保留自已验证可用的 `camera-color-viewer`；电机控制被放到独立任务，不改动该采集链路。

## 逻辑

1. 上电后小车保持停止，摄像头照常向浏览器发送 80×60 彩色图。
2. 按一次 **BOOT**：开始寻找红球。
3. 对每帧图像仅计算红度 `R - (G+B)/2 > 71` 且 `R > 90` 的红色像素数。
4. 连续三帧达到 `RED_PIXELS_TO_TRIGGER` 后，两个前轮前冲。
5. 达到 `PUSH_FORWARD_MS` 后，两前轮反转 `PUSH_RETURN_MS`，然后停止。
6. 运行中再次按 BOOT 会立即停止。

直行只使用左右前轮，后轮始终停止。

## 接线

| 功能 | ESP32-S3 |
|---|---|
| 摄像头 D− / D+ | GPIO19 / GPIO20 |
| 左前轮 IN1 / IN2 / PWM | GPIO8 / GPIO9 / GPIO10 |
| 后轮 IN1 / IN2 / PWM | GPIO12 / GPIO13 / GPIO14 |
| 右前轮 IN1 / IN2 / PWM | GPIO15 / GPIO16 / GPIO17 |
| 电机 STBY | GPIO11 |

用 VS Code 的 **Flash Device** 烧录。用 `tools/flash_and_view.sh` 可烧录后自动打开图像预览。
