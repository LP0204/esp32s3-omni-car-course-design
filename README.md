# ESP32-S3 Omni Car Course Design

清华大学“电子设计技术基础”小学期课程工程。项目基于 ESP32-S3 和三轮全向
底盘，逐步完成硬件单项测试、循迹、避障以及后续扩展功能。

## 工程目录

| 目录 | 内容 |
|---|---|
| `vscode-esp32s3测试程序-红外传感器` | 四路红外循迹传感器黑白识别测试 |
| `vscode-esp32s3测试程序-轮子` | 三个电机的转速、正反转和同时运行测试 |
| `vscode-esp32s3循线程序` | 三轮全向小车循线控制程序 |

## 开发环境

- ESP32-S3-DevKitC-1
- ESP-IDF 5.4.4
- Visual Studio Code + Espressif IDF 扩展
- macOS

每个子工程均为独立 ESP-IDF 工程，应在 VS Code 中单独打开对应工程根目录。
构建产物、自动生成的 `sdkconfig` 和本机编辑器配置不纳入版本管理。
