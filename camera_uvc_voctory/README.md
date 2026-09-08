# camera_uvc_test - ESP32-S3 摄像头循线 + 超声避障 + 屏显（实验任务2）

整车固件：USB 摄像头（JQ-CAM12-720P-V1）循线 + HC-SR04 避障 + TFT18 屏显。
循线/避障/BOOT 启停逻辑由 Arduino 版（line_follower_muy_bien_ultrasonic.ino）
原样移植，只是“线位置输入”从四路红外换成 USB 摄像头：

1. `usb_host_uvc v2.5.2` 原生驱动枚举并抓取 MJPEG 流（支持 Bulk 视频流）。
2. `esp_jpeg` 软件解码为 RGB565，生成 **160x120 检测灰度图**
   （640x480 的 1/4 解码尺寸，细线能保留 1~2px）。
3. 取最底部 8 行（离车最近，约车前 6~7cm），分 4 个区间
   `bit3=[52,66) bit2=[66,80) bit1=[80,94) bit0=[94,108)`（各宽 14px），
   区间内只要出现灰度 < 阈值（默认 110 可实时调）的黑像素即判“有线”，
   与四路红外逻辑一一对应，16 状态查表无需改动。底部 8 行之上另取 24 行
   作为“远带”，仅在近带无线时兜底使用（弯道模式已移除）；
   日志 `LINE ... near=0b.. far=0b.. bits=0b..` 可观察两带。
   （串口灰度流仍为 80x60，查看器不掉帧。）
4. 10ms 控制任务：16 状态查表 + 弯道降速 + 丢线寻线 + 超声避障状态机
   + BOOT 键启停（默认停车，按一次开始，再按一次停止）。
5. TFT18 每 0.2s 显示：`L/R/B` 三轮指令速度 + `D` 超声距离 + 底部 4 线位框。

## 击球阶段（任务2 第二阶段，整体移植 red-ball-push 最新逻辑）

触发过一次避障并重新回到循线后，识别到 T 型终点（四路全黑 `0b1111`）会先停车
3 秒（`T_LINE_STOP_MS`），然后自动进入击球阶段，循线/避障不再接管电机：

1. **搜球**：红度检测照搬 `red-ball-push` 当前代码——80x60 网格逐点判红
   （`R > 90` 且 `R - (G+B)/2 > 90`），统计红点数与横向质心 `cx`（0..79，中心 40）。
   小车以“短脉冲左转 → 短刹 → 稳定 120ms → 再读一帧”的方式逐步左转寻找；
   连续 `BALL_CONFIRM_FRAMES` 帧红点 >= `BALL_DETECT_PIXELS` 后进入对准。
2. **对准**：红球偏左/偏右就发一个转向脉冲（初始 80ms），每次脉冲后短刹 35ms
   并稳定 120ms 再判一次；若红球中心越过中线（发生过冲）则反向并把脉冲时长
   减半，最短 20ms。连续 `BALL_CENTER_CONFIRM_FRAMES` 帧落在 ±5px（80 网格）
   内才认为对准。
3. **推球**：两前轮直行 900ms（左 130 / 右 166，8-bit 速度）→ 停 500ms →
   后退 930ms → 停车完成。推球阶段不读红球，纯定时。

所有脉冲时长/刹车/稳定时间/推力都在 `follower.c` 顶部“击球阶段参数”块；
判红阈值在 `ball_vision.c` 顶部。

**调试命令**（串口，无需跑完全程）：
- `p`：直接进入击球阶段（找红球、对准、推球）。
- `i`：打印当前视觉快照，例如
  `BALL VIS: found=1 cx=38 pix=43 (map center=40)`，
  用于现场确认判红阈值和红球横向位置是否合适。

日志以 `BALL:` 开头，按
`SEARCH -> ALIGN -> PUSH -> PAUSE -> RETURN -> DONE` 打印当前状态；
对准过程中的每个脉冲与过冲减半都会打印。

串口（921600）每 200ms 打印摄像头检测结果：

```text
I (1234) uvc_test: LINE w=80 h=60 black=1234 cx=41 bits=0b0110 fps=16.5
```

`bits` 语义与红外/Arduino 版一致：`bit3=最左区 ... bit0=最右区`，`1` 表示该区有黑线。

控制日志（与 Arduino 版格式一致）：

```text
CAM=0b0110 tg=0 lf=114 rf=126 rear=0
AVOID TRIGGER: 7.5 cm
AVOID: strafe LEFT
AVOID: obstacle clear, pass obstacle
AVOID: strafe RIGHT, searching line
AVOID: line reacquired, bits=0x6
```

如果摄像头描述符无法解析（没有输出任何格式），日志会打印：

```text
E (xxxx) uvc_test: No UVC frame formats parsed ... check CONFIG_UVC_PRINTF_CONFIGURATION_DESCRIPTOR log
```

此时把连接时打印的完整配置描述符（`uvc_test: ...` 与随后的描述符转储）贴回来，
我们据此手工修复这颗摄像头的描述符解析。

## 接线

| 功能 | 引脚 |
|---|---|
| 左前轮 | GPIO8 / 9 / 10 |
| 后轮 | GPIO12 / 13 / 14 |
| 右前轮 | GPIO15 / 16 / 17 |
| STBY | GPIO11 |
| 摄像头 D- / D+ / 5V / GND | GPIO19 / GPIO20 / 5V / GND |
| HC-SR04 TRIG / ECHO | GPIO18 / GPIO21（ECHO 5V 必须分压到 3.3V） |
| BOOT 键 | GPIO0（按下低电平） |
| TFT18 SCLK / MOSI / CS / DC / RST | GPIO39 / 40 / 41 / 42 / 47 |
| 云台舵机（任务2 第二阶段） | GPIO4 / 5 |

摄像头占用 GPIO19/20，因此本工程控制台走 UART0（GPIO43/44），
`sdkconfig.defaults` 已关闭 USB-Serial-JTAG 控制台。
串口波特率为 **921600**（图像流需要高速率，VS Code 监视器也选 921600）。

## 电脑端实时查看摄像头灰度图

固件会把摄像头看到的 80x60 灰度图通过串口发到电脑，实时调试阈值很方便：

1. 电脑装 pyserial：`pip install pyserial`
2. 运行查看器（会自动向串口发 `v` 开启灰度流，关闭窗口自动发 `x` 关闭）：

   ```bash
   python3 tools/gray_viewer.py /dev/cu.usbserial-0001
   ```

3. 窗口里并排显示放大后的 80x60 灰度图与二值图，标签显示实时帧率。
   **按 + / -（或 m / n）可实时调节黑线阈值（±5）**，二值图即时变化，
   直到黑线清晰变黑即可，无需重新烧录。
   关闭 VS Code 监视器后再运行查看器（同一串口不能同时被两个程序占用）。

也可手动控制：串口发 `v` 开启灰度流、`x` 关闭。

## 编译 / 烧录

1. VS Code 打开本文件夹，确认当前 ESP-IDF 为 v5.4.x，目标 `esp32s3`。
2. `ESP-IDF: Build App`，然后 `ESP-IDF: Flash your Project`（UART，串口 921600，
   监视器也要选 921600）。
3. 烧录后小车默认不运行；确认摄像头画面输出正常（`LINE ...` 日志）后，
   按开发板 **BOOT 键** 开始循迹+避障，再按一次停止。
4. 也可用串口命令：`g`=开始，`s`=停车，`b`=线位位图，`u`=测一次距离。

## 常见问题

- **按 BOOT 不启动**：日志提示 `start rejected (no line under sensors)`，
  说明当前画面里没识别到黑线（或阈值不合适），调整 `LINE_BLACK_THRESHOLD`。
- **转向方向反了**：镜头装反（画面上下颠倒）时 `main/main.c` 里
  `CAMERA_ROTATE_180` 应为 1（默认已开）；若旋转后左右仍反，
  再把 `CAMERA_MIRROR` 改为 1。
- **避障太早/太晚**：调 `main/follower.c` 里 `OBSTACLE_TRIGGER_CM`（当前 9cm）、
  `OBSTACLE_CLEAR_CM`（20cm）与横移三参数 `STRAFE_LF/RF/REAR_SPEED`。
- **枚举成功但 Formats 列表为空 / 协商全失败**：旧版 libuvc 驱动解析不了
  JQ-CAM12 的 VS 描述符。本工程已切换到 usb_host_uvc v2.5.2 原生驱动
  （`main/idf_component.yml` 里 `usb_host_uvc: "^2.5.2"`）。若 v2 仍解析不出
  格式，把连接时打印的完整描述符贴回来。
- **流打开失败**：日志会打印 `stream open failed: ESP_ERR_NOT_FOUND`，
  并自动试下一个候选格式；每个失败间隔 3 秒重试。
- **黑线阈值不合适**：默认 100（`LINE_BLACK_THRESHOLD_DEFAULT`）。
  最方便的是用查看器窗口的 `+`/`-` 实时调节；灰色误判多就调小，
  黑线识别不到就调大。
- **黑线很细识别不到**：检测已是 160x120 + 逐行最暗段（`LINE_MIN_RUN_PX=1`），
  若仍丢线，可调低 `LINE_BLACK_THRESHOLD` 或 `LINE_CONTRAST_MIN`（当前 40）。
- **车不转向/蛇形**：摄像头把线中心映射成多位置位（`bits_from_cx`），
  与红外 16 状态表配合；若左右反了把 `CAMERA_MIRROR` 改为 1。
- **车往“平均比较暗”的地方偏**：检测已改为逐行最长黑色段（`line_center_from_gray`），
  不再用所有暗像素的加权中心；若仍有灰斑干扰，调小 `LINE_BLACK_THRESHOLD`，
  或调大 `LINE_MIN_RUN_PX`。
- **PSRAM 未启用导致分配失败**：确认 `sdkconfig` 里有 `CONFIG_SPIRAM=y`。
- **枚举报 "Configuration descriptor larger than control transfer max length"**：
  摄像头配置描述符超过默认 256B 上限，`sdkconfig.defaults` 已把
  `CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE` 设为 2048。
- **解码失败**：JQ-CAM12 的 MJPEG 帧若不带 Huffman 表，需要 `CONFIG_JD_DEFAULT_HUFFMAN=y`
  （`sdkconfig.defaults` 已默认开启，并已关闭 ROM 版 TJpgDec）。
