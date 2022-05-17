#include "motor.h"
#define f_DIR_L D0
#define f_DIR_R D1
#define r_DIR_L D14
#define r_DIR_R D15

#define f_PWM_L PWM2_MODULE3_CHA_D2
#define f_PWM_R PWM2_MODULE3_CHB_D3
#define r_PWM_L PWM1_MODULE0_CHA_D12
#define r_PWM_R PWM1_MODULE0_CHB_D13

/***********************************************************
 * @brief 电机控制引脚初始化
 * @param
 * @return
 ***********************************************************/
int motor_init(void)
{
  gpio_init(f_DIR_L, GPO, GPIO_HIGH, GPIO_PIN_CONFIG);
  gpio_init(f_DIR_R, GPO, GPIO_HIGH, GPIO_PIN_CONFIG);
  gpio_init(r_DIR_L, GPO, GPIO_HIGH, GPIO_PIN_CONFIG);
  gpio_init(r_DIR_R, GPO, GPIO_HIGH, GPIO_PIN_CONFIG);

  pwm_init(f_PWM_L, 17000, 0); //
  pwm_init(f_PWM_R, 17000, 0); //
  pwm_init(r_PWM_L, 17000, 0); //
  pwm_init(r_PWM_R, 17000, 0); // 初始化频率17KHz 占空比初始为0
	return 0;
}
/***********************************************************
 * @brief 电机转速,方向设置
 * @param LeftMotorPwm 左轮pwm
 * @param RightMotorPwm 右轮pwm
 * @return
 ***********************************************************/
void motor_control(int32 f_duty_l, int32 f_duty_r, int32 r_duty_l, int32 r_duty_r) //从前至后依次为右前轮，左前轮，右后轮，左后轮
{
  //对占空比限幅
  f_duty_l = limit(f_duty_l, PWM_DUTY_MAX);
  f_duty_r = limit(f_duty_r, PWM_DUTY_MAX);
  r_duty_l = limit(r_duty_l, PWM_DUTY_MAX);
  r_duty_r = limit(r_duty_r, PWM_DUTY_MAX);

  if (r_duty_l >= 0)
  {
    gpio_set(r_DIR_L, GPIO_LOW); // DIR输出高电平
    pwm_duty(r_PWM_L, r_duty_l); // 计算占空比
  }
  else
  {
    gpio_set(r_DIR_L, GPIO_HIGH); // DIR输出低电平
    pwm_duty(r_PWM_L, -r_duty_l); // 计算占空比
  }

  if (r_duty_r >= 0)
  {
    gpio_set(r_DIR_R, GPIO_LOW); // DIR输出高电平
    pwm_duty(r_PWM_R, r_duty_r); // 计算占空比
  }
  else
  {
    gpio_set(r_DIR_R, GPIO_HIGH); // DIR输出低电平
    pwm_duty(r_PWM_R, -r_duty_r); // 计算占空比
  }

  if (f_duty_l >= 0)
  {
    gpio_set(f_DIR_L, GPIO_HIGH); // DIR输出高电平
    pwm_duty(f_PWM_L, f_duty_l);  // 计算占空比
  }
  else
  {
    gpio_set(f_DIR_L, GPIO_LOW);  // DIR输出低电平
    pwm_duty(f_PWM_L, -f_duty_l); // 计算占空比
  }

  if (f_duty_r >= 0)
  {
    gpio_set(f_DIR_R, GPIO_HIGH); // DIR输出高电平
    pwm_duty(f_PWM_R, f_duty_r);  // 计算占空比
  }
  else
  {
    gpio_set(f_DIR_R, GPIO_LOW);  // DIR输出低电平
    pwm_duty(f_PWM_R, -f_duty_r); // 计算占空比
  }
}
/***********************************************************
 * @brief 双电机停转
 * @param
 * @return
 ***********************************************************/
void MotorStopped(void)
{
  //前轮停转
  pwm_duty(f_PWM_L, 0);
  pwm_duty(f_PWM_R, 0);
  //后轮停转
  pwm_duty(r_PWM_L, 0);
  pwm_duty(r_PWM_R, 0);
}
