# ESP32-S3 摄像头串口图像查看

这个工程只做一件事：使用 ESP32-S3 的原生 USB 主机读取 ESP-Claw 三合一摄像头
（型号 JQ-CAM12-720D-V1），以 **1280×720** 取图，在 ESP32-S3 上把 MJPEG 解码并缩小成
**80×60 灰度图**，
再通过开发板的 USB-UART/CH340 串口传到 macOS 的浏览器显示。它不包含电机、舵机、
超声波或巡线控制，适合先单独验证“摄像头能否稳定成像”。

采用 `usb_host_uvc` v2 的 Bulk UVC 取流和 CRC 校验帧，而不是直接把整张 JPEG 通过
串口发送；后者容易超出串口带宽、卡顿或丢帧。

## 接线

| 摄像头 | ESP32-S3 |
|---|---|
| USB D− | GPIO19 |
| USB D＋ | GPIO20 |
| 5V | 5V |
| GND | GND |

电脑仍然通过 USB-UART/CH340 与开发板连接。烧录完成后，电脑端和开发板使用同一
串口：ESP32 固件把 UART0 设置为 **921600 baud** 发送灰度图。

## 一键使用（推荐）

关闭 **Monitor Device** 后，在 VS Code 按 `⌘⇧P`，输入并选择 **Tasks: Run Task**，再选择
**Camera: Flash and Open Preview**。它会自动完成烧录、首次安装查看器依赖、启动查看器并打开
浏览器 `http://127.0.0.1:8765/`。

之后每次要看图像都运行这一个任务；无需手动输入终端命令。任务运行期间保持其终端不要关闭。
ESP-IDF 面板里的普通 **Flash Device** 只负责烧录，不会启动浏览器查看器。

## 手动使用

1. 在 VS Code 中打开本目录，选择 ESP32-S3，构建并烧录 `build/camera_gray_viewer.bin`。
2. 摄像头接好并重新上电。不要同时打开 VS Code 的 Monitor Device，否则它会抢占串口。
3. 在本目录建立一个 Python 虚拟环境并安装电脑端依赖：

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install -r tools/requirements.txt
   ```

4. 查找串口后启动查看器（默认使用 `/dev/cu.usbserial-0001`）：

   ```bash
   python tools/gray_viewer.py /dev/cu.usbserial-0001
   ```

5. 脚本会自动打开浏览器地址 `http://127.0.0.1:8765/`，窗口中显示实时画面；如果没有自动打开，
   手动访问这个地址即可。

如果设备名称不同，先在终端执行 `ls /dev/cu.*`，然后修改 `.vscode/settings.json` 里的
`idf.port`。

## 预期现象

- 浏览器页面先显示“等待图像帧…”，随后并排放大显示 80×60 灰度图与二值图；黄色三角形是道路关注区，青色虚线是四路虚拟检测通道边界。
- 固件按约 80 ms 的最小间隔处理画面；实际帧率受摄像头和串口状态影响，但延迟明显低于旧版 160×90/300 ms 预览。
- 页面上的“阈值 −5 / +5”按钮会立刻改变二值图的黑白分界，用来确认黑线与白底的对比度；
  这个工程不会据此控制电机。
- 脚本会自动向开发板发送 `v` 开启图像流。若要手动操作，在串口发送 `v` 开始、`x` 停止，
  `m` / `n` 将二值阈值加 / 减 5。
- 若终端或浏览器始终没有帧，先关闭 Monitor Device；再检查 D−/D＋、5V、GND。若监视器中
  没有 `STREAMING_STARTED`，把完整日志发给我。

固件使用 2048 字节 USB 控制传输缓冲区，以容纳这款摄像头的复合 USB 描述符；这也是
之前出现 `Configuration descriptor larger than control transfer max length` 时需要的设置。
