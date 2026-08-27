# LQ_TFT18SPIV33 完整信息显示测试

这是Arduino完整版界面的ESP-IDF 5.4.4 / VS Code移植版。屏幕同时显示三个电机指令、HC-SR04距离和底部四路红外状态。

本工程是显示与传感器测试程序，不驱动电机，因此：

- `L:0`：左前轮指令为0；
- `R:0`：右前轮指令为0；
- `B:0`：后轮指令为0；
- `D:xx.x`：HC-SR04测得的距离，单位厘米；无有效回波时显示 `D:?`；
- 屏幕底部四格从左到右对应物理位置OUT4、OUT3、OUT2、OUT1，黑格表示检测到黑色，白格表示检测到白色。

屏幕每200 ms更新一次，避免40 KB整帧刷新占用过多控制时间。距离显示使用最近三个有效值的中值；连续三次没有有效回波后显示 `D:?`。

## 接线

### 显示屏

| LQ_TFT18SPIV33 | ESP32-S3 |
|---|---|
| VCC | 3V3，禁止接5V |
| GND | GND |
| SCL/SCLK | GPIO39 |
| SDA/SDI/MOSI | GPIO40 |
| CS | GPIO41 |
| DC/A0 | GPIO42 |
| RST/RES | GPIO47 |
| BL/LED | 3V3 |

显示屏不需要连接MISO。

### 四路红外

| 物理位置（从车头看） | 模块输出 | ESP32-S3 |
|---|---|---|
| 最左 | OUT4 | GPIO7 |
| 左中 | OUT3 | GPIO6 |
| 右中 | OUT2 | GPIO5 |
| 最右 | OUT1 | GPIO4 |

程序按低电平为黑色、高电平为白色处理。

### HC-SR04

| HC-SR04 | ESP32-S3 |
|---|---|
| VCC | 5V |
| GND | GND |
| TRIG | GPIO18 |
| ECHO | 经分压或电平转换后接GPIO21 |

标准HC-SR04的ECHO约为5V，严禁直接接ESP32-S3。可使用 `1 kΩ + 2 kΩ` 电阻分压：

```text
HC-SR04 ECHO ---- 1 kΩ ----+---- GPIO21
                            |
                           2 kΩ
                            |
                           GND
```

显示屏、红外模块、HC-SR04、电机驱动板和ESP32-S3必须共地。
