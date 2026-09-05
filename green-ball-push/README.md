# Green Ball Push

基于 ESP32-S3 和 JQ-CAM12-720D-V1（ESP-Claw 三合一摄像头）的绿色目标测试工程。

## 功能

- 单片机在 80×60 采样图上计算绿度：`G - (R+B)/2`。
- 连续 3 帧检测到至少 20 个绿色像素后执行：前进 → 停顿 → 后退。
- 本版本不转向，方便先验证绿色识别和直行/返回动作。
- 电脑端可查看摄像头原始 JPEG 图像，优先选择 **640×480（480p）**；如果摄像头不提供该模式，会按日志中的备用模式运行。预览串口恢复为 921600，使用双缓冲；实际帧率需要上板测量。
- 浏览器提供绿度阈值滑块，调整会实时下发到单片机并立即影响绿度图和识别。

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

## 使用

1. 在 VS Code 单独打开本文件夹，确认 `.vscode/settings.json` 中的 `idf.port` 是当前串口；若单独使用 Monitor Device，应用日志波特率为 921600，启动日志仍为 115200。查看图像时请关闭 Monitor。
2. 运行任务 **Green Ball: Flash and Open 480p Preview**。任务会烧录固件、启动电脑端查看器，并打开 `http://127.0.0.1:8767/`。
3. 浏览器会同时显示摄像头原图和单片机实际使用的 80×60 绿度二值图；两幅图都已按车上安装方向旋转 180°。把绿色目标放在摄像头视野中后，按一次开发板 **BOOT**，开始等待绿色目标。
4. 识别成功后日志会显示 `green detected`，小车前进 `900 ms`，停顿 `500 ms`，再后退 `930 ms`。
5. 再按一次 BOOT 可停止电机。查看器退出用终端 `Ctrl+C`。

若只想烧录而不看图像，可在 ESP-IDF 面板直接执行 Flash；串口日志仍会显示绿度采样结果。

## 绿度参数

在 `main/main.c` 顶部调整：

```c
#define GREENNESS_THRESHOLD_DEFAULT 42  // 默认绿度优势阈值，越小越容易判绿
#define GREEN_MIN_CHANNEL 75    // G 通道最低亮度
#define GREEN_PIXELS_TO_TRIGGER 20
#define GREEN_CONFIRM_FRAMES 3
```

电机直行功率和动作时间也在同一处：`LEFT_STRAIGHT_POWER_PERCENT`、`RIGHT_STRAIGHT_POWER_PERCENT`、`PUSH_FORWARD_MS`、`PUSH_PAUSE_MS`、`PUSH_RETURN_MS`。

## 电脑端查看器

`tools/green_viewer.py` 接收 `JPG4`（原始 JPEG）和 `GMAP`（80×60 绿度二值图）串口帧并校验 CRC16，在浏览器中显示 640×480 原图和单片机实际判定图。页面将两幅图同步旋转 180°，并通过 `/set-threshold` 将滑块值以 `gNNN` 命令发给 ESP32。它每秒发送一次 `v` 开启预览；关闭查看器时发送 `x`。图像传输不会参与绿度计算，也不会改变电机状态机。
