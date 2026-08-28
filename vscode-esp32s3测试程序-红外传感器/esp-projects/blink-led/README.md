# ESP32-S3 四路红外黑白识别测试

## 接线与物理位置

| 传感器输出 | ESP32-S3 | 从车头向前看 |
|---|---:|---|
| OUT4 | GPIO7 | 最左 |
| OUT3 | GPIO6 | 左二 |
| OUT2 | GPIO5 | 右二 |
| OUT1 | GPIO4 | 最右 |
| VCC | 3V3 | — |
| GND | GND | — |

日志中的位图固定为 `bit3=OUT4 ... bit0=OUT1`。程序默认按照模块资料将低电平
识别为黑色、高电平识别为白色；如果实测相反，修改 `BLACK_LEVEL`。

烧录后打开 `Monitor Device`，依次用黑线遮住四个探头。日志按从左到右顺序
显示 OUT4、OUT3、OUT2、OUT1 的电平和黑白判断。
