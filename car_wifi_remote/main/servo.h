#ifndef SERVO_H
#define SERVO_H

/* 云台舵机（实测）：GPIO5 = 左右（pan），GPIO4 = 上下（tilt） */
void servo_init(void);
void servo_set_pan_deg(int deg);    /* 0..180，90 = 中位 */
void servo_set_tilt_deg(int deg);   /* 0..180，90 = 水平 */
int  servo_pan_deg(void);
int  servo_tilt_deg(void);

#endif
