# Green Ball Push（离线版）

基于 ESP32-S3 和 JQ-CAM12-720D-V1（ESP-Claw 三合一摄像头）的绿色目标推送工程。摄像头取帧、JPEG 解码、绿度识别和电机控制全部在单片机上完成；工程不再传输图像到电脑，也不需要浏览器查看器。

## 运行逻辑

1. 上电后摄像头持续取图，并在单片机内把图像按 80×60 网格采样。
2. 每个采样点计算绿度 `G - (R+B)/2`。绿度大于阈值且 G 通道亮度足够时判为绿色。
3. 按下开发板 **BOOT**（GPIO0）后，小车先以右转小脉冲搜索绿色目标；连续 3 帧检测到至少 3 个绿色采样点后进入对准状态。
4. 对准时根据绿色质心相对画面中心的误差做小角度转动；每次转动结束立即对三只轮施加全功率主动刹车 90 ms，再等待车体稳定。如果越过中心，自动反向并将步长缩短为原来的 2/3，最小 20 ms。绿色质心在中心 ±5 个采样点内连续稳定 3 帧后，执行：
   - 前进 900 ms；
   - 停止 500 ms；
   - 后退 930 ms；
   - 完成后停止，等待下一次 BOOT。
5. 动作期间再次按 BOOT 会立即停止电机并回到空闲状态。

单片机只输出低速文字日志，断开电脑后仍可独立运行。电脑端不再打开图像窗口；如需查看运行状态，可用 VS Code 的 Monitor Device，以 115200 波特率查看日志。

## 接线情况 2

| 模块 | ESP32-S3 |
|---|---|
| 摄像头 USB D− | GPIO19 |
| 摄像头 USB D＋ | GPIO20 |
| 摄像头供电 | 5V + GND |
| 右前轮 | GPIO15 / GPIO16 / GPIO17 |
| 后轮 | GPIO12 / GPIO13 / GPIO14 |
| 左前轮 | GPIO8 / GPIO9 / GPIO10 |
| 电机 STBY | GPIO11 |
| BOOT 按键 | GPIO0 |

## 使用方法

1. 在 VS Code 打开本文件夹，确认 `.vscode/settings.json` 中的 `idf.port` 是开发板当前串口。
2. 使用 ESP-IDF 的 **Build** 和 **Flash Device** 烧录；不需要运行任何查看器脚本。
3. 烧录完成后按一次 BOOT，看到日志 `BOOT: searching green` 即开始向右搜索。
4. 将绿色目标放入摄像头视野。日志会依次显示 `green detected`、`align step`、`green centered`，然后小车执行前进、停顿、后退。

日志中的 `green` 是最终满足绿度和亮度条件的采样点数，`bright` 是达到最低 G 通道亮度的点数，`metric` 是只满足绿度条件的点数，`maxG` 是当前帧最大绿度，`peak` 是最大 G 通道值。若 `green` 长期为 0，同时 `maxG` 小于 30，说明阈值过高或光照不足；若 `green` 有数值但小于 `need=3`，说明目标在画面中太小，可根据日志再调整触发点数。

## 主要参数

参数均在 `main/main.c` 顶部：

```c
#define GREENNESS_THRESHOLD 30 // 绿度阈值；越小越容易判定为绿色
#define GREEN_MIN_CHANNEL 75   // G 通道最低亮度
#define GREEN_PIXELS_TO_TRIGGER 3 // 远距离目标的最少绿色采样点数
#define GREEN_CONFIRM_FRAMES 3
#define TURN_STEP_POWER 70          // 旋转 PWM 命令
#define SEARCH_STEP_MS 40           // 找不到目标时，每次向右转动的时间
#define TURN_BRAKE_MS 90            // 每次旋转后的主动急刹时间
#define TURN_SETTLE_MS 200          // 每次转动后的稳定等待时间
#define ALIGN_INITIAL_STEP_MS 50     // 对准初始转动步长
#define ALIGN_MIN_STEP_MS 20         // 过冲后步长下限
#define OVERSHOOT_REVERSE_NUMERATOR 2
#define OVERSHOOT_REVERSE_DENOMINATOR 3
```

与 `red-ball-push` 保持一致的运动参数为：

```c
#define LEFT_STRAIGHT_POWER_PERCENT 58
#define RIGHT_STRAIGHT_POWER_PERCENT 62
#define PUSH_FORWARD_MS 900
#define PUSH_PAUSE_MS 500
#define PUSH_RETURN_MS 930
```

绿度阈值或动作速度/时间调整后，重新 Build 并 Flash 即可生效。
