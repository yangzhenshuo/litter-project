#ifndef _motor_h
#define _motor_h

#include "headfile.h"

void motor_init(void);
void motor_control(int32 f_duty_l, int32 f_duty_r, int32 r_duty_l, int32 r_duty_r);
void MotorStopped(void);

#endif