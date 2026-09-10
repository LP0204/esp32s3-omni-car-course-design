#ifndef MOTOR_H
#define MOTOR_H

void motor_init(void);
void motor_set_lf(int speed);
void motor_set_rf(int speed);
void motor_set_rear(int speed);
void motor_stop_all(void);
void motor_brake_all(void);

#endif
