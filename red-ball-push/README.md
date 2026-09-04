# Red Ball Push

基于 ESP32-S3、ESP-Claw 三合一摄像头和三轮底盘的离线红球识别与推球工程。

摄像头、红色识别和电机控制全部在 ESP32-S3 上运行，不需要连接电脑，也不再通过串口传输图像。

## 逻辑

1. 上电后小车保持停止；摄像头优先请求 320×240@10 FPS，并在单片机内生成 80×60 红度采样结果。
2. 按一次 **BOOT**：小车以短脉冲方式向左原地寻找红球；每次转动后先短刹车，再等待车身稳定并读取新画面。
3. 对每帧图像仅计算红度 `R - (G+B)/2 > 71` 且 `R > 90` 的红色像素数和红色区域横向中心。
4. 连续三帧达到 `RED_PIXELS_TO_TRIGGER` 后进入对准状态：红色在画面左侧就执行一次左转脉冲，在右侧就执行一次右转脉冲，然后刹停并重新判断。
5. 红色中心越过中线时说明发生过冲，程序反向转动，并把脉冲时长减半；最低缩至20ms。连续三帧位于中线 ±3 像素内才算对准。
6. 对准后两前轮向前运动 `PUSH_FORWARD_MS`，停止 `PUSH_PAUSE_MS`，再向后运动 `PUSH_RETURN_MS`，最后停止。
7. 运行中再次按 BOOT 会立即停止。

直行只使用左右前轮，后轮始终停止。

## 可调参数

均位于 `main/main.c` 顶部：

- `TURN_STEP_POWER`：每次小角度旋转的功率，默认20%。
- `SEARCH_STEP_MS`：找球阶段每次左转脉冲时间，默认70ms。
- `ALIGN_INITIAL_STEP_MS`：首次对准脉冲时间，默认80ms。
- `ALIGN_MIN_STEP_MS`：过冲后允许缩小到的最短脉冲，默认20ms。
- `TURN_BRAKE_MS` / `TURN_SETTLE_MS`：每次旋转后的短刹车和稳定等待时间。
- `CENTER_DEADBAND_PX`：画面中线允许误差，默认 ±3 像素。
- `LEFT_STRAIGHT_POWER_PERCENT`：左前轮前进和后退功率，当前57%。
- `RIGHT_STRAIGHT_POWER_PERCENT`：右前轮前进和后退功率，当前63%。
- `PUSH_FORWARD_MS` / `PUSH_PAUSE_MS` / `PUSH_RETURN_MS`：前进、停顿和后退时间。

## 接线

| 功能 | ESP32-S3 |
|---|---|
| 摄像头 D− / D+ | GPIO19 / GPIO20 |
| 左前轮 IN1 / IN2 / PWM | GPIO8 / GPIO9 / GPIO10 |
| 后轮 IN1 / IN2 / PWM | GPIO12 / GPIO13 / GPIO14 |
| 右前轮 IN1 / IN2 / PWM | GPIO15 / GPIO16 / GPIO17 |
| 电机 STBY | GPIO11 |

用 VS Code 的 **Flash Device** 烧录。烧录结束后拔掉电脑连接也可独立运行，按 BOOT 开始或停止。
