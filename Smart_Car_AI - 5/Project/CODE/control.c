#include "control.h"
#include "motor.h"
#include "SEEKFREE_ICM20602.h"
#include "encoder.h"
#include "projectmath.h"
#include "System.h"
#include "position.h"
#include "image.h"

#define MOTOR_PWM_MAX (500.0 * 50) //电机最大PWM

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
static float SpeedSet_L1 = 0;              //左前轮设置速度
static float f_LeftSpeedDeltaPrev = 0;     //左前轮前一次速度偏差
static float f_LeftSpeedDeltaPrevPrev = 0; //左前轮前前一次速度偏差

static float f_LeftSpeedControlBais = 0; //左前轮速度控制增量
static float f_LeftSpeedControlOut = 0;  //左前轮速度环输出

/********左后轮********/
static float SpeedSet_L2 = 0;              //左后轮设置速度
static float r_LeftSpeedDeltaPrev = 0;     //左后轮前一次速度偏差
static float r_LeftSpeedDeltaPrevPrev = 0; //左后轮前前一次速度偏差

static float r_LeftSpeedControlBais = 0; //左后轮速度控制增量
static float r_LeftSpeedControlOut = 0;  //左后轮速度环输出

/********右前轮********/
static float SpeedSet_R1 = 0;               //右前轮设置速度
static float f_RightSpeedDeltaPrev = 0;     //右前轮前一次速度偏差
static float f_RightSpeedDeltaPrevPrev = 0; //右前轮前前一次速度偏

static float f_RightSpeedControlBais = 0; //右前轮速度控制增量
static float f_RightSpeedControlOut = 0;  //右前轮速度环输出

/********右后轮********/
static float SpeedSet_R2 = 0;               //右后轮设置速度
static float r_RightSpeedDeltaPrev = 0;     //右后轮前一次速度偏差
static float r_RightSpeedDeltaPrevPrev = 0; //右后轮前前一次速度偏

static float r_RightSpeedControlBais = 0; //右后轮速度控制增量
static float r_RightSpeedControlOut = 0;  //右后轮速度环输出

static float MotorDifferential = 0; //电机用方向差速
/*************角速度控制参数******************/
static float AngleDeltaPrev = 0;   //前一次角速度偏差
static float AngleControlBais = 0; //角速度输出增量
/*************位置控制参数******************/
static float PositionControlOut_x = 0;         // x轴位置输出量
static float PositionDeviationIntegrate_x = 0; // x轴累计位置差
static float PositionDeltaPrev_x = 0;          // x轴前一次位置偏差
static float PositionControlOut_y = 0;         // y轴位置输出量
static float PositionDeviationIntegrate_y = 0; // y轴累计位置差
static float PositionDeltaPrev_y = 0;          // y轴前一次位置偏差
/***********************************************************
 * @brief 复位控制变量（不包括pid参数）
 * @param
 * @return
 ***********************************************************/
void ResetControlArgs(void)
{
    /***复位速度控制变量***/

    f_LeftSpeed = 0;                  //左前电机速度
    f_RightSpeed = 0;                 //右前机速度
    r_LeftSpeed = 0;                  //左后电机速度
    r_RightSpeed = 0;                 //右后电机速度
    f_LeftStallCount = 0;             //左前电机堵转次数
    f_RightStallCount = 0;            //右前电机堵转次数
    r_LeftStallCount = 0;             //左后电机堵转次数
    r_RightStallCount = 0;            //右后电机堵转次数
                                      /*************角速度控制参数******************/
    AngleDeltaPrev = 0;               //前一次角速度偏差
    AngleControlBais = 0;             //角速度输出增量
                                      /*************位置控制参数******************/
    PositionControlOut_x = 0;         // x轴位置输出量
    PositionDeviationIntegrate_x = 0; // x轴累计位置差
    PositionDeltaPrev_x = 0;          // x轴前一次位置偏差
    PositionControlOut_y = 0;         // y轴位置输出量
    PositionDeviationIntegrate_y = 0; // y轴累计位置差
    PositionDeltaPrev_y = 0;          // y轴前一次位置偏差
    /********左前轮********/
    static float SpeedSet_L1 = 0;              //左前轮设置速度
    static float f_LeftSpeedDeltaPrev = 0;     //左前轮前一次速度偏差
    static float f_LeftSpeedDeltaPrevPrev = 0; //左前轮前前一次速度偏差

    static float f_LeftSpeedControlBais = 0; //左前轮速度控制增量
    static float f_LeftSpeedControlOut = 0;  //左前轮速度环输出

    /********左后轮********/
    static float SpeedSet_L2 = 0;              //左后轮设置速度
    static float r_LeftSpeedDeltaPrev = 0;     //左后轮前一次速度偏差
    static float r_LeftSpeedDeltaPrevPrev = 0; //左后轮前前一次速度偏差

    static float r_LeftSpeedControlBais = 0; //左后轮速度控制增量
    static float r_LeftSpeedControlOut = 0;  //左后轮速度环输出

    /********右前轮********/
    static float SpeedSet_R1 = 0;               //右前轮设置速度
    static float f_RightSpeedDeltaPrev = 0;     //右前轮前一次速度偏差
    static float f_RightSpeedDeltaPrevPrev = 0; //右前轮前前一次速度偏

    static float f_RightSpeedControlBais = 0; //右前轮速度控制增量
    static float f_RightSpeedControlOut = 0;  //右前轮速度环输出

    /********右后轮********/
    static float SpeedSet_R2 = 0;               //右后轮设置速度
    static float r_RightSpeedDeltaPrev = 0;     //右后轮前一次速度偏差
    static float r_RightSpeedDeltaPrevPrev = 0; //右后轮前前一次速度偏

    static float r_RightSpeedControlBais = 0; //右后轮速度控制增量
    static float r_RightSpeedControlOut = 0;  //右后轮速度环输出
}
/***********************************************************
 * @brief 小车姿态解算
 * @param
 * @return
 ***********************************************************/
static inline void CarPostureCal(void)
{
//    get_icm20602_accdata_spi(); //获取加速度计数据
    get_icm20602_gyro_spi();    // 获取陀螺仪角速度
//    float alpha = 0.3;
    float x, y, z;
    x = (float)icm_gyro_x - GyroOffset.x;
    y = (float)icm_gyro_y - GyroOffset.y;
    z = (float)icm_gyro_z - GyroOffset.z;
    if (x <= 10 && x >= -10)
        x = 0; //对差值进行滤波
    if (y <= 10 && y >= -10)
        y = 0;
    if (z <= 10 && z >= -10)
        z = 0;
//    icm.acc_x = (((float)icm_acc_x) * alpha) / 4096 + icm.acc_x * (1 - alpha);
//    icm.acc_y = (((float)icm_acc_y) * alpha) / 4096 + icm.acc_y * (1 - alpha);
//    icm.acc_z = (((float)icm_acc_z) * alpha) / 4096 + icm.acc_z * (1 - alpha);
    icm.gyro_x = x / 16.4;
    icm.gyro_y = y / 16.4;
    icm.gyro_z = z / 16.4;
    if (CarInfo.yaw > 360 || CarInfo.yaw < -360)
        CarInfo.yaw = 0;
    CarInfo.yaw += icm.gyro_z * dtt;
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
    CarInfo.RunDistance1 += (CarPulse.L1 + CarPulse.R1 + CarPulse.L2 + CarPulse.R2) / 4; //更新小车x轴运行距离
    CarInfo.RunDistance2 += (CarPulse.L1 - CarPulse.R1 - CarPulse.L2 + CarPulse.R2) / 4; //更新小车y轴运行距离
		CarInfo.delet1 = CarPulse.L1;
		CarInfo.delet2 = CarPulse.R1;
		CarInfo.delet3 = CarPulse.L2;
		CarInfo.delet4 = CarPulse.R2;
    // CarInfo.RealSpeed = (f_LeftSpeed + f_RightSpeed) * 0.5;   //计算小车速度
}
/***********************************************************
 * @brief 速度变积分函数
 * @param delta 偏差
 * @return 变积分系数
 ***********************************************************/
static inline float SpeedChangeI(float delta)
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
 * @brief 角度控制计算,位置式,Pi控制,变积分
 * @param
 * @return
 ***********************************************************/
static inline void AngleControlCal(void)
{
    float AngleDelta; 	//角速度偏差
	  if(CarInfo.Iscorrect=='F')
    {
			AngleDelta = CarInfo.AngleSet - CarInfo.yaw;
		}			//计算偏差量
		else AngleDelta=CarInfo.angle;
    CarInfo.SpeedSet_z = AngleControlPid.P * AngleDelta + AngleControlPid.D * (AngleDelta - AngleDeltaPrev); //位置式PD控制
    AngleDeltaPrev = AngleDelta;	//更新为上次偏差
}
/***********************************************************
 * @brief 角度控制输出
 * @param
 * @return
 ***********************************************************/
void AngleControl()
{
    CarPostureCal();
    AngleControlCal();
}
/***********************************************************
 * @brief 速度控制解算
 * @param
 * @return
 ***********************************************************/
static inline void Speedmath(float x, float y, float z)
{
    if (z > 400)
        z = 400;
    else if (z < -400)
        z = -400; // 对转向速度限幅
    SpeedSet_L1 = x + y - z * 0.2;
    SpeedSet_R1 = x - y + z * 0.2;
    SpeedSet_L2 = x - y - z * 0.2;
    SpeedSet_R2 = x + y + z * 0.2;
}
/***********************************************************
 * @brief 速度控制计算，增量式，pid控制，变积分
 * @param
 * @return
 ***********************************************************/
static inline void SpeedControlCal(void)
{
    float SpeedDelta1; //左前轮速度偏差
    float SpeedDelta2; //左后轮速度偏差
    float SpeedDelta3; //右前轮速度偏差
    float SpeedDelta4; //右后轮速度偏差
    /**************左前轮控制计算*************/
    SpeedDelta1 = SpeedSet_L1 - f_LeftSpeed; //计算偏差量
    // if (CarInfo.IsMotorDiffrientialOn == 'T')
    // SpeedDelta += MotorDifferential * (MotorDifferential >= 0 ? AdjustablePars.par_8 : 1);
    f_LeftSpeedControlBais = SpeedControlPid.P * (SpeedDelta1 - f_LeftSpeedDeltaPrev); //增量式PID控制
    if (SpeedControlPid.I != 0)
    {
        if (SystemSettings.ChangeIEnable == 'T')
        {
            f_LeftSpeedControlBais += SpeedChangeI(DetAbs(SpeedSet_L1, f_LeftSpeed)) * SpeedControlPid.I * SpeedDelta1;
        }
        else
        {
            f_LeftSpeedControlBais += SpeedControlPid.I * SpeedDelta1;
        }
    }
    if (SpeedControlPid.D != 0)
    {
        f_LeftSpeedControlBais += SpeedControlPid.D * (SpeedDelta1 + f_LeftSpeedDeltaPrevPrev - 2 * f_LeftSpeedDeltaPrev);
    }
    f_LeftSpeedDeltaPrevPrev = f_LeftSpeedDeltaPrev; //更新为上上次偏差
    f_LeftSpeedDeltaPrev = SpeedDelta1;              //更新为上次偏差                                                                      //保存上一次偏差
    f_LeftSpeedControlOut += f_LeftSpeedControlBais;
    /**************左后轮控制计算*************/
    SpeedDelta2 = SpeedSet_L2 - r_LeftSpeed;                                           //计算偏差量
    r_LeftSpeedControlBais = SpeedControlPid.P * (SpeedDelta2 - r_LeftSpeedDeltaPrev); //增量式PID控制
    if (SpeedControlPid.I != 0)
    {
        if (SystemSettings.ChangeIEnable == 'T')
        {
            r_LeftSpeedControlBais += SpeedChangeI(DetAbs(SpeedSet_L2, r_LeftSpeed)) * SpeedControlPid.I * SpeedDelta2;
        }
        else
        {
            r_LeftSpeedControlBais += SpeedControlPid.I * SpeedDelta2;
        }
    }
    if (SpeedControlPid.D != 0)
    {
        r_LeftSpeedControlBais += SpeedControlPid.D * (SpeedDelta2 + r_LeftSpeedDeltaPrevPrev - 2 * r_LeftSpeedDeltaPrev);
    }
    r_LeftSpeedDeltaPrevPrev = r_LeftSpeedDeltaPrev; //更新为上上次偏差
    r_LeftSpeedDeltaPrev = SpeedDelta2;              //更新为上次偏差                                                                      //保存上一次偏差
    r_LeftSpeedControlOut += r_LeftSpeedControlBais; //增量输出
    /**************右前轮控制计算*************/
    SpeedDelta3 = SpeedSet_R1 - f_RightSpeed;                                            //计算偏差量
    f_RightSpeedControlBais = SpeedControlPid.P * (SpeedDelta3 - f_RightSpeedDeltaPrev); //增量式PID控制
    if (SpeedControlPid.I != 0)
    {
        if (SystemSettings.ChangeIEnable == 'T')
        {
            f_RightSpeedControlBais += SpeedChangeI(DetAbs(SpeedSet_R1, f_RightSpeed)) * SpeedControlPid.I * SpeedDelta3;
        }
        else
        {
            f_RightSpeedControlBais += SpeedControlPid.I * SpeedDelta3;
        }
    }
    if (SpeedControlPid.D != 0)
    {
        f_RightSpeedControlBais += SpeedControlPid.D * (SpeedDelta3 + f_RightSpeedDeltaPrevPrev - 2 * f_RightSpeedDeltaPrev);
    }
    f_RightSpeedDeltaPrevPrev = f_RightSpeedDeltaPrev; //更新为上上次偏差
    f_RightSpeedDeltaPrev = SpeedDelta3;               //更新为上次偏差
    f_RightSpeedControlOut += f_RightSpeedControlBais; //增量输出
                                                       /**************右后轮控制计算*************/
    SpeedDelta4 = SpeedSet_R2 - r_RightSpeed;          //计算偏差量

    r_RightSpeedControlBais = SpeedControlPid.P * (SpeedDelta4 - r_RightSpeedDeltaPrev); //增量式PID控制
    if (SpeedControlPid.I != 0)
    {
        if (SystemSettings.ChangeIEnable == 'T')
        {
            r_RightSpeedControlBais += SpeedChangeI(DetAbs(SpeedSet_R2, r_RightSpeed)) * SpeedControlPid.I * SpeedDelta4;
        }
        else
        {
            r_RightSpeedControlBais += SpeedControlPid.I * SpeedDelta4;
        }
    }
    if (SpeedControlPid.D != 0)
    {
        r_RightSpeedControlBais += SpeedControlPid.D * (SpeedDelta4 + r_RightSpeedDeltaPrevPrev - 2 * r_RightSpeedDeltaPrev);
    }
    r_RightSpeedDeltaPrevPrev = r_RightSpeedDeltaPrev; //更新为上上次偏差
    r_RightSpeedDeltaPrev = SpeedDelta4;               //更新为上次偏差                                                                     //±￡′?é?ò?′???2?
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
    motor_control((int32)(f_RightSpeedControlOut), (int32)(f_LeftSpeedControlOut), (int32)(r_RightSpeedControlOut), (int32)(r_LeftSpeedControlOut));
}
/***********************************************************
 * @brief 速度控制
 * @param
 * @return
 ***********************************************************/
void SpeedControl(void)
{
    GetMotorSpeed();
    Speedmath(CarInfo.SpeedSet_x, CarInfo.SpeedSet_y, CarInfo.SpeedSet_z);

    SpeedControlCal();
    MotorControlOut();
}
/***********************************************************
 * @brief 位置控制
 * @param
 * @return
 ***********************************************************/
void PositionControl(void)
{
    float PositionDelta_x; //位置偏差
    if (CarInfo.Iscorrect == 'F')
    {
        PositionDelta_x = position.x * 200 - (CarInfo.RunDistance1 / 210) * 5.5 * PI;
    } //计算偏差量
    else
        PositionDelta_x = CarInfo.distance1;
    PositionDeviationIntegrate_x += PositionDelta_x*5;
    if (PositionDeviationIntegrate_x > 20)       // 100//200
        PositionDeviationIntegrate_x = 20;       // 100//200
    else if (PositionDeviationIntegrate_x < -20) // 100//200
        PositionDeviationIntegrate_x = -20;      // 100//2000
    PositionControlOut_x = PositionControlPid.P * PositionDelta_x + PositionControlPid.I * PositionDeviationIntegrate_x + PositionControlPid.D * (PositionDelta_x - PositionDeltaPrev_x);
    PositionDeltaPrev_x = PositionDelta_x;
    if (PositionControlOut_x > 80)
    {
        PositionControlOut_x = 80;
    }
    else if (PositionControlOut_x < -80)
    {
        PositionControlOut_x = -80;
    }
    CarInfo.SpeedSet_x = PositionControlOut_x;
    float PositionDelta_y; //位置偏差
    if (CarInfo.Iscorrect == 'F')
    {
        PositionDelta_y = position.y * 200 - (CarInfo.RunDistance2 / 220) * 5.5 * PI; //计算偏差量
    }
    else
        PositionDelta_y = CarInfo.distance2*5;
    PositionDeviationIntegrate_y += PositionDelta_y;
    if (PositionDeviationIntegrate_y > 20)       // 100//200
        PositionDeviationIntegrate_y = 20;       // 100//200
    else if (PositionDeviationIntegrate_y < -20) // 100//200
        PositionDeviationIntegrate_y = -20;      // 100//2000
    PositionControlOut_y = PositionControlPid.P * PositionDelta_y + PositionControlPid.I * PositionDeviationIntegrate_y + PositionControlPid.D * (PositionDelta_y - PositionDeltaPrev_y);
    PositionDeltaPrev_y = PositionDelta_y;
    if (PositionControlOut_y > 80)
    {
        PositionControlOut_y = 80;
    }
    else if (PositionControlOut_y < -80)
    {
        PositionControlOut_y = -80;
    }
    CarInfo.SpeedSet_y = PositionControlOut_y;
	  
}