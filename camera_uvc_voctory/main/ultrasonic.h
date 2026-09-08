#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

/* 初始化 TRIG/ECHO 引脚 */
void ultrasonic_init(void);

/* 由控制任务周期调用；内部按采样间隔限频，更新原始距离与显示距离 */
void ultrasonic_update(uint32_t now_ms);

/* 阻塞式测一次（串口 'u' 命令用），返回 cm，无效返回 -1 */
float ultrasonic_read_once_cm(void);

/* 最近一次原始距离（cm，-1 = 无有效回波） */
float ultrasonic_raw_cm(void);

/* 中值滤波后的显示距离（cm，-1 = 无有效回波） */
float ultrasonic_display_cm(void);

#endif /* ULTRASONIC_H */
