/*
 * control.c
 * 控制算法 
 * Author: han
 */

#include "control.h"
#include "motor.h"
#include "SEEKFREE_ICM20602.h"
#include "encoder.h"
#include "projectmath.h"
#include "System.h"

#define MOTOR_PWM_MAX (900.0 * 50) //电机最大PWM

/*************速度控制参数******************/
static float f_LeftSpeed = 0;       //左前电机速度
static float f_RightSpeed = 0;      //右前机速度
static float r_LeftSpeed = 0;       //左后电机速度
static float r_RightSpeed = 0;      //右后电机速度
static uint8 f_LeftStallCount = 0;  //左前电机堵转次数
static uint8 f_RightStallCount = 0; //右前电机堵转次数
static uint8 r_LeftStallCount = 0;  //左后电机堵转次数
static uint8 r_RightStallCount = 0; //右后电机堵转次数

static float ChangeIA = 100; //变积分控制A系数
static float ChangeIB = 10;  //变积分控制B系数

/********左前轮********/
static float SpeedSet_L1=0;                //左前轮设置速度
static float f_LeftSpeedDeltaPrev = 0;     //左前轮前一次速度偏差
static float f_LeftSpeedDeltaPrevPrev = 0; //左前轮前前一次速度偏差

static float f_LeftSpeedControlBais = 0; //左前轮速度控制增量
static float f_LeftSpeedControlOut = 0;  //左前轮速度环输出

/********左后轮********/
static float SpeedSet_L2=0;                //左后轮设置速度
static float r_LeftSpeedDeltaPrev = 0;     //左后轮前一次速度偏差
static float r_LeftSpeedDeltaPrevPrev = 0; //左后轮前前一次速度偏差

static float r_LeftSpeedControlBais = 0; //左后轮速度控制增量
static float r_LeftSpeedControlOut = 0;  //左后轮速度环输出

/********右前轮********/
static float SpeedSet_R1=0;                //右前轮设置速度
static float f_RightSpeedDeltaPrev = 0;     //右前轮前一次速度偏差
static float f_RightSpeedDeltaPrevPrev = 0; //右前轮前前一次速度偏

static float f_RightSpeedControlBais = 0; //右前轮速度控制增量
static float f_RightSpeedControlOut = 0;  //右前轮速度环输出

/********右后轮********/
static float SpeedSet_R2=0;                //右后轮设置速度
static float r_RightSpeedDeltaPrev = 0;     //右后轮前一次速度偏差
static float r_RightSpeedDeltaPrevPrev = 0; //右后轮前前一次速度偏

static float r_RightSpeedControlBais = 0; //右后轮速度控制增量
static float r_RightSpeedControlOut = 0;  //右后轮速度环输出

static float MotorDifferential = 0; //电机用方向差速
/*************角度控制参数******************/
static float AngleDeltaPrev = 0;  //前一次角速度偏差

/***********************************************************
 * @brief 复位控制变量（不包括pid参数）
 * @param
 * @return
***********************************************************/
void ResetControlArgs(void)
{
    /***复位速度控制变量***/

    f_LeftSpeed = 0;       //左前电机速度
    f_RightSpeed = 0;      //右前机速度
    r_LeftSpeed = 0;       //左后电机速度
    r_RightSpeed = 0;      //右后电机速度
	
	  CarInfo.IsMotorStalled = 'F';//电机不堵转
    CarInfo.IsMotorDiffrientialOn = 'T';//电机差速开启
    f_LeftStallCount = 0;  //左前电机堵转次数
    f_RightStallCount = 0; //右前电机堵转次数
    r_LeftStallCount = 0;  //左后电机堵转次数
    r_RightStallCount = 0; //右后电机堵转次数
	 /*************角度控制参数******************/
   static float AngleDeltaPrev = 0;  //前一次角速度偏差
	
   /********左前轮********/
   static float SpeedSet_L1=0;                //左前轮设置速度
   static float f_LeftSpeedDeltaPrev = 0;     //左前轮前一次速度偏差
   static float f_LeftSpeedDeltaPrevPrev = 0; //左前轮前前一次速度偏差

   static float f_LeftSpeedControlBais = 0; //左前轮速度控制增量
   static float f_LeftSpeedControlOut = 0;  //左前轮速度环输出

   /********左后轮********/
   static float SpeedSet_L2=0;                //左后轮设置速度
   static float r_LeftSpeedDeltaPrev = 0;     //左后轮前一次速度偏差
   static float r_LeftSpeedDeltaPrevPrev = 0; //左后轮前前一次速度偏差

   static float r_LeftSpeedControlBais = 0; //左后轮速度控制增量
   static float r_LeftSpeedControlOut = 0;  //左后轮速度环输出

   /********右前轮********/
   static float SpeedSet_R1=0;                //右前轮设置速度
   static float f_RightSpeedDeltaPrev = 0;     //右前轮前一次速度偏差
   static float f_RightSpeedDeltaPrevPrev = 0; //右前轮前前一次速度偏

   static float f_RightSpeedControlBais = 0; //右前轮速度控制增量
   static float f_RightSpeedControlOut = 0;  //右前轮速度环输出

   /********右后轮********/
   static float SpeedSet_R2=0;                //右后轮设置速度
   static float r_RightSpeedDeltaPrev = 0;     //右后轮前一次速度偏差
   static float r_RightSpeedDeltaPrevPrev = 0; //右后轮前前一次速度偏

   static float r_RightSpeedControlBais = 0; //右后轮速度控制增量
   static float r_RightSpeedControlOut = 0;  //右后轮速度环输出
}
/***********************************************************
 * @brief 角速度控制计算,位置式,PD控制,不完全微分
 * @param
 * @return
***********************************************************/
static inline void AngleControlCal(void)
{
	  float AngleDelta;                                  //角速度偏差
    AngleDelta = CarInfo.AngleSet - icm.gyro_z; //计算偏差量
    //MotorDifferential = AdjustablePars.par_5 * AngleDelta; //计算电机差速
    //MotorDifferential = AdjustablePars.par_5 * CarInfo.RealSpeed * tan((float)SteerOut * 0.00175);
    //if (SystemSettings.FuzzyEnable == 'T')
    //SteerGetP(&AngleControlPid.P, ImageStatus.OFFLine, AngleDelta);                                   //模糊控制动态P
    CarInfo.SpeedSet_z= AngleControlPid.P * AngleDelta + AngleControlPid.D * (AngleDelta - AngleDeltaPrev); //位置式PD控制器
    AngleDeltaPrev = AngleDelta;                                                                         //保存前一次角度偏差
}
/***********************************************************
 * @brief 角度控制输出
 * @param
 * @return
***********************************************************/
void AngleControl()
{
	AngleControlCal();
}
/***********************************************************
 * @brief 获取电机速度以及更新运行距离（包含堵转检测），速度对应：160 -> 3m/s
 * @param
 * @return
***********************************************************/
void GetMotorSpeed(void)
{
    /****速度计算及更新运行距离****/
    EncoderPulseGet(); //获取编码器脉冲数
    f_LeftSpeed = (float)CarPulse.L1;
    f_RightSpeed = (float)CarPulse.R1;
	  r_LeftSpeed = (float)CarPulse.L2;
    r_RightSpeed = (float)CarPulse.R2;
    CarInfo.RunDistance += (CarPulse.L1 + CarPulse.R1) / 2; //更新小车运行距离
    CarInfo.RealSpeed = (f_LeftSpeed + f_RightSpeed) * 0.5;   //计算小车速度
    /**********堵转检测***********/
    if (CarInfo.IsRun == 'T' && (CarInfo.SpeedSet_z > 100 || CarInfo.SpeedSet_z < -100))
    {
        if (f_LeftSpeed <= 3 && f_LeftSpeed >= -3)
        {
            if (++f_LeftStallCount >= 100) //前轮左电机堵转连续超过500ms
            {
                CarInfo.IsMotorStalled = 'T';
            }
        }
        else
        {
            f_LeftStallCount = 0; //前轮左电机堵转次数
        }
				if (r_LeftSpeed <= 3 && r_LeftSpeed >= -3)
        {
            if (++r_LeftStallCount >= 100) //后轮左电机堵转连续超过500ms
            {
                CarInfo.IsMotorStalled = 'T';
            }
        }
        else
        {
            r_LeftStallCount = 0; //后轮左电机堵转次数
        }
        if (f_RightSpeed <= 3 && f_RightSpeed >= -3)
        {
            if (++f_RightStallCount >= 100) //前轮右电机堵转连续超过500ms
            {
                CarInfo.IsMotorStalled = 'T';
            }
        }
        else
        {
            f_RightStallCount = 0; //前轮右电机堵转次数
        }
				if (r_RightSpeed <= 3 && r_RightSpeed >= -3)
        {
            if (++r_RightStallCount >= 100) //后轮右电机堵转连续超过500ms
            {
                CarInfo.IsMotorStalled = 'T';
            }
        }
        else
        {
            r_RightStallCount = 0; //后轮右电机堵转次数
        }
    }
}
/***********************************************************
 * @brief 变积分函数
 * @param delta 偏差
 * @return 变积分系数
***********************************************************/
static inline float ChangeI(float delta)
{
    if (delta <= ChangeIB) //小偏差启用积分作用
    {
        return 1.0;
    }
    else if (delta > ChangeIA + ChangeIB) //大偏差取消积分作用
    {
        return 0.0;
    }
    else
    {
        return (ChangeIA + ChangeIB - delta) / ChangeIA;
    }
}
/***********************************************************
 * @brief 速度控制解算
 * @param
 * @return
***********************************************************/
void Speedmath(float x, float y, float z)
{
	if(z>100) z= 100;
   else if(z < -100) z = -100;   // 对转向速度限幅
	SpeedSet_L1=x+y-z*19.5;
	SpeedSet_R1=x-y+z*19.5;
	SpeedSet_L2=x-y-z*19.5;
	SpeedSet_R2=x+y+z*19.5;
}
/***********************************************************
 * @brief 速度控制计算，增量式，pid控制，变积分
 * @param
 * @return
***********************************************************/
static inline void SpeedControlCal(void)
{
    float SpeedDelta; //速度偏差
    /**************左前轮控制计算*************/
    SpeedDelta = SpeedSet_L1 - f_LeftSpeed; //计算偏差量
    //if (CarInfo.IsMotorDiffrientialOn == 'T')
    //SpeedDelta += MotorDifferential * (MotorDifferential >= 0 ? AdjustablePars.par_8 : 1);
    f_LeftSpeedControlBais = SpeedControlPid.P * (SpeedDelta - f_LeftSpeedDeltaPrev); //增量式PID控制
    if (SpeedControlPid.I != 0)
    {
        if (SystemSettings.ChangeIEnable == 'T')
        {
            f_LeftSpeedControlBais += ChangeI(DetAbs((SpeedSet_L1 + MotorDifferential), f_LeftSpeed)) * SpeedControlPid.I * SpeedDelta;
        }
        else
        {
            f_LeftSpeedControlBais += SpeedControlPid.I * SpeedDelta;
        }
    }
    if (SpeedControlPid.D != 0)
    {
        f_LeftSpeedControlBais += SpeedControlPid.D * (SpeedDelta + f_LeftSpeedDeltaPrevPrev - 2 * f_LeftSpeedDeltaPrev);
    }
    f_LeftSpeedDeltaPrevPrev = f_LeftSpeedDeltaPrev; //更新为上上次偏差
    f_LeftSpeedDeltaPrev = SpeedDelta;             //更新为上次偏差                                                                      //保存上一次偏差
    f_LeftSpeedControlOut += f_LeftSpeedControlBais;
		    /**************左后轮控制计算*************/
    SpeedDelta = SpeedSet_L2 - r_LeftSpeed; //计算偏差量
//    if (CarInfo.IsMotorDiffrientialOn == 'T')
//    SpeedDelta += MotorDifferential * (MotorDifferential >= 0 ? AdjustablePars.par_8 : 1);
    r_LeftSpeedControlBais = SpeedControlPid.P * (SpeedDelta - r_LeftSpeedDeltaPrev); //增量式PID控制
    if (SpeedControlPid.I != 0)
    {
        if (SystemSettings.ChangeIEnable == 'T')
        {
            r_LeftSpeedControlBais += ChangeI(DetAbs((SpeedSet_L2 + MotorDifferential), r_LeftSpeed)) * SpeedControlPid.I * SpeedDelta;
        }
        else
        {
            r_LeftSpeedControlBais += SpeedControlPid.I * SpeedDelta;
        }
    }
    if (SpeedControlPid.D != 0)
    {
        r_LeftSpeedControlBais += SpeedControlPid.D * (SpeedDelta + r_LeftSpeedDeltaPrevPrev - 2 * r_LeftSpeedDeltaPrev);
    }
    r_LeftSpeedDeltaPrevPrev = r_LeftSpeedDeltaPrev; //更新为上上次偏差
    r_LeftSpeedDeltaPrev = SpeedDelta;             //更新为上次偏差                                                                      //保存上一次偏差
    r_LeftSpeedControlOut += r_LeftSpeedControlBais;//增量输出
    /**************右前轮控制计算*************/
    SpeedDelta = SpeedSet_R1 - f_RightSpeed; //计算偏差量
   // if (CarInfo.IsMotorDiffrientialOn == 'T')
     //   SpeedDelta -= MotorDifferential * (MotorDifferential < 0 ? AdjustablePars.par_8 : 1);
    f_RightSpeedControlBais = SpeedControlPid.P * (SpeedDelta - f_RightSpeedDeltaPrev); //增量式PID控制
    if (SpeedControlPid.I != 0)
    {
        if (SystemSettings.ChangeIEnable == 'T')
        {
            f_RightSpeedControlBais += ChangeI(DetAbs((SpeedSet_R1 - MotorDifferential), f_RightSpeed)) * SpeedControlPid.I * SpeedDelta;
        }
        else
        {
            f_RightSpeedControlBais += SpeedControlPid.I * SpeedDelta;
        }
    }
    if (SpeedControlPid.D != 0)
    {
        f_RightSpeedControlBais += SpeedControlPid.D * (SpeedDelta + f_RightSpeedDeltaPrevPrev - 2 * f_RightSpeedDeltaPrev);
    }
    f_RightSpeedDeltaPrevPrev = f_RightSpeedDeltaPrev; //更新为上上次偏差
    f_RightSpeedDeltaPrev = SpeedDelta;              //更新为上次偏差                                                                     
    f_RightSpeedControlOut += f_RightSpeedControlBais; //增量输出
 /**************右后轮控制计算*************/
    SpeedDelta = SpeedSet_R2 - r_RightSpeed; //计算偏差量
   // if (CarInfo.IsMotorDiffrientialOn == 'T')
     //   SpeedDelta -= MotorDifferential * (MotorDifferential < 0 ? AdjustablePars.par_8 : 1);
    r_RightSpeedControlBais = SpeedControlPid.P * (SpeedDelta - r_RightSpeedDeltaPrev); //增量式PID控制
    if (SpeedControlPid.I != 0)
    {
        if (SystemSettings.ChangeIEnable == 'T')
        {
            r_RightSpeedControlBais += ChangeI(DetAbs((SpeedSet_R2 - MotorDifferential), r_RightSpeed)) * SpeedControlPid.I * SpeedDelta;
        }
        else
        {
            r_RightSpeedControlBais += SpeedControlPid.I * SpeedDelta;
        }
    }
    if (SpeedControlPid.D != 0)
    {
        r_RightSpeedControlBais += SpeedControlPid.D * (SpeedDelta + r_RightSpeedDeltaPrevPrev - 2 * r_RightSpeedDeltaPrev);
    }
    r_RightSpeedDeltaPrevPrev = r_RightSpeedDeltaPrev; //更新为上上次偏差
    r_RightSpeedDeltaPrev = SpeedDelta;              //更新为上次偏差                                                                     //±￡′?é?ò?′???2?
    r_RightSpeedControlOut += r_RightSpeedControlBais; //增量输出
}


/***********************************************************
 * @brief 电机控制量输出
 * @param
 * @return
***********************************************************/
static inline void MotorControlOut(void)
{
    if (f_LeftSpeedControlOut < -MOTOR_PWM_MAX) //前轮限幅
    {
        f_LeftSpeedControlOut = -MOTOR_PWM_MAX;
    }
    else if (f_LeftSpeedControlOut > MOTOR_PWM_MAX)
    {
        f_LeftSpeedControlOut = MOTOR_PWM_MAX;
    }
    if (f_RightSpeedControlOut < -MOTOR_PWM_MAX)
    {
        f_RightSpeedControlOut = -MOTOR_PWM_MAX;
    }
    else if (f_RightSpeedControlOut > MOTOR_PWM_MAX)
    {
        f_RightSpeedControlOut = MOTOR_PWM_MAX;
    }
		if (r_LeftSpeedControlOut < -MOTOR_PWM_MAX) //后轮限幅
    {
        r_LeftSpeedControlOut = -MOTOR_PWM_MAX;
    }
    else if (r_LeftSpeedControlOut > MOTOR_PWM_MAX)
    {
        r_LeftSpeedControlOut = MOTOR_PWM_MAX;
    }
    if (r_RightSpeedControlOut < -MOTOR_PWM_MAX)
    {
        r_RightSpeedControlOut = -MOTOR_PWM_MAX;
    }
    else if (r_RightSpeedControlOut > MOTOR_PWM_MAX)
    {
        r_RightSpeedControlOut = MOTOR_PWM_MAX;
    }
motor_control((int32)(f_LeftSpeedControlOut), (int32)(f_RightSpeedControlOut), (int32)(r_LeftSpeedControlOut), (int32)(r_RightSpeedControlOut));
}

/***********************************************************
 * @brief 速度控制
 * @param
 * @return
***********************************************************/
void SpeedControl(void)
{
    GetMotorSpeed();
	  Speedmath(CarInfo.SpeedSet_x,CarInfo.SpeedSet_y,CarInfo.SpeedSet_z);
    if (CarInfo.IsAiOn == 'F' && 0 < CarInfo.RealSpeed && CarInfo.RealSpeed < 50 && CarInfo.SpeedSet_z > 100)
    {
        f_LeftSpeedControlOut = f_RightSpeedControlOut = 15000;
			  r_LeftSpeedControlOut = r_RightSpeedControlOut = 15000;
    }
    else
    {
        SpeedControlCal();
    }
    MotorControlOut();
}/***********************************************************
 * @brief 位置控制
 * @param
 * @return
***********************************************************/
void PositionControl(void)
{
//    float PositonDelta;                                       //位置偏差
//    PositonDelta = CarInfo.PositionSet - CarInfo.RunDistance; //计算偏差量
//    PositionDeviationIntegrate += PositonDelta;
//    if (Prate = 100;       //100//200
//    else if (PositionDeviationIntegrate < -100) //100//200
//        PositionDeviationIntegrate = -100;      //100//2000
//    PositionControlOut = PositionCositionDeviationIntegrate > 100)       //100//200
//        PositionDeviationIntegontrolPid.P * PositonDelta + PositionControlPid.I * PositionDeviationIntegrate + PositionControlPid.D * (PositonDelta - PositionDeltaPrev);
//    PositionDeltaPrev = PositonDelta;
//    if (PositionControlOut > 80)
//    {
//        PositionControlOut = 80;
//    }
//    else if (PositionControlOut < -80)
//    {
//        PositionControlOut = -80;
//    }
//    CarInfo.SpeedSet = PositionControlOut;
}