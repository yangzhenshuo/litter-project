#ifndef _motor_h
#define _motor_h

#include "headfile.h"

void motor_init(void);
void SetMotorPwmAndDir(int32 LeftMotorPwm, int32 RightMotorPwm);
    
#endif