#include "motor.h"

#define DIR_L				D0
#define DIR_R				D2

#define PWM_L				PWM1_MODULE3_CHB_D1
#define PWM_R				PWM2_MODULE3_CHB_D3
/***********************************************************
 * @brief 电机控制引脚初始化
 * @param
 * @return
***********************************************************/
void motor_init(void)
{
	gpio_init(DIR_L, GPO, GPIO_HIGH, GPIO_PIN_CONFIG);
	gpio_init(DIR_R, GPO, GPIO_HIGH, GPIO_PIN_CONFIG);

	pwm_init(PWM_L, 10000, 0);												// PWM 通道2 初始化频率10KHz 占空比初始为0
	pwm_init(PWM_R, 10000, 0);												// PWM 通道4 初始化频率10KHz 占空比初始为0
}

/***********************************************************
 * @brief 双电机转速,方向设置
 * @param LeftMotorPwm 左轮pwm
 * @param RightMotorPwm 右轮pwm
 * @return
***********************************************************/
void SetMotorPwmAndDir(int32 LeftMotorPwm, int32 RightMotorPwm)
{
    if (LeftMotorPwm >= 0) //左轮正转
    {
        pwm_duty(PWM_L, 0);                    //左轮PWM输出
        pwm_duty(PWM_R, (uint32)LeftMotorPwm); //右轮PWM输出
    }

    else //左轮反转
    {
        pwm_duty(PWM_L, -(uint32)LeftMotorPwm); //左轮PWM输出
        pwm_duty(PWM_R,0);                     //右轮PWM输出
    }
    if (RightMotorPwm >= 0) //右轮正转
    {
        pwm_duty(PWM_L, 0);                     //左轮PWM输出
        pwm_duty(PWM_R, (uint32)RightMotorPwm); //右轮PWM输出
    }
    else //右轮反转
    {
        pwm_duty(PWM_L, -(uint32)RightMotorPwm); //左轮PWM输出
        pwm_duty(PWM_R, 0);                      //右轮PWM输出
    }
}