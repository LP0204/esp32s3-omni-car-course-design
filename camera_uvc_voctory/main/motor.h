#ifndef MOTOR_H
#define MOTOR_H

/* 三轮电机控制（逻辑速度 -255..255，正=向前） */
void motor_init(void);
void motor_set_lf(int speed);
void motor_set_rf(int speed);
void motor_set_rear(int speed);
void motor_stop_all(void);
/* 三轮同时短刹（IN1/IN2 全高 + 满占空比），用于脉冲转向后抑制惯性 */
void motor_brake_all(void);

/* 当前指令速度（供 TFT 显示，逻辑速度） */
int motor_shown_lf(void);
int motor_shown_rf(void);
int motor_shown_rear(void);

#endif /* MOTOR_H */
