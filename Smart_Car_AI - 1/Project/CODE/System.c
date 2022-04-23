/*
 * System.c
 * 软件驱动全局管理
 * Author: 杨镇硕
 */
 
#include "System.h"
#include "motor.h"
#include "encoder.h"
#include "display.h"
#include "button.h"
#include "timer_pit.h"
#include "buzzer.h"
#include "openart_mini.h"
#include "smotor.h"
#include "image.h"

SystemSettingsTypedef SystemSettings;//系统信息
CarInfoTypedef CarInfo;//小车信息
ControlPidTypedef SpeedControlPid = {1.0,0.5,0.0};//速度控制pid参数初始化
ControlPidTypedef AngleControlPid = {5, 0, 1};              //角度pid参数
CarPulseTypedef CarPulse = {0,0,0,0};//编码器初值
GyroOffsetTypedef GyroOffset = {0.0,0.0,0.0};//陀螺仪零飘初始化数据
ICMTypedef icm = {0.0,0.0,0.0,0.0,0.0,0.0};//ICM数据初始化
speedTypedef speed = {0.0,0.0};//速度初始化
positionTypedef position = {0.2,0.2};//位置初始化化
coordinateTypedef coordinate = {1,1};//初始坐标
float dtt = 0.001;//积分时间
/***********************************************************
 * @brief 运行时参数初始化
 * @param
 * @return
***********************************************************/
static inline void CarInfoInit(void)
{
    CarInfo.IsRun = 'F';                 //是否运行
    CarInfo.IsOutGarage = 'F';           //是否出库
    //CarInfo.IsCameraDetectRun = 'T';     //摄像头检测是否运作
    CarInfo.IsMotorStalled = 'F';        //电机是否堵转
    //CarInfo.IsMotorDiffrientialOn = 'T'; //电机差速是否开启

    CarInfo.IsAiOn = 'F';      //识别模式是否打开
    CarInfo.SpeedSet_x = 0;    //x轴速度设置
	  CarInfo.SpeedSet_y = 0;    //y轴速度设置
	  CarInfo.SpeedSet_z = 0;    //z轴速度设置
    CarInfo.RealSpeed = 0;   //真实速度
    CarInfo.AngleSet = 0;    //角度设置
    CarInfo.PositionSet = 0; //位置设置
		CarInfo.pitch = 0.0;
		CarInfo.roll = 0.0;
		CarInfo.yaw = 0.0;       //真实角度

    CarInfo.BinaryThreshold = 0; //二值化阈值
    CarInfo.RunDistance = 0;     //小车运行距离
}
/***********************************************************
 * @brief 设置初始化
 * @param
 * @return
***********************************************************/
static inline void SystemSettingsInit(void)
{
    SystemSettings.PostureReportEnable = 'F';     //是否启动姿态上报
    SystemSettings.ImageStatusReportEnable = 'F'; //是否启动图像处理模式上报
    SystemSettings.AiEnable = 'F';                //是否开启识别
    SystemSettings.FuzzyEnable = 'T';             //是否启动角度模糊控制
    SystemSettings.ChangeIEnable = 'F';           //是否启动速度变积分控制

    SystemSettings.BinaryMethod = 1; //二值化方法

   
    SystemSettings.ApriltagSearchSpeed = 10; //搜索Apriltag速度
    SystemSettings.TargetSearchSpeed = 10;   //搜索靶子速度
}
/***********************************************************
 * @brief 消除陀螺仪零点偏差
 * @param
 * @return
***********************************************************/
void gyro_offset_init(void)
{
	for (uint16_t i = 0; i < 100; ++i) 
	{
     get_icm20602_gyro_spi();    // 获取陀螺仪角速度
     GyroOffset.x += icm_gyro_x;
     GyroOffset.y += icm_gyro_y;
     GyroOffset.z += icm_gyro_z;
     systick_delay_ms(5);    // 最大 1Khz
   }

   GyroOffset.x /= 100;
   GyroOffset.y /= 100;
   GyroOffset.z /= 100;
}
/***********************************************************
 * @brief 硬件设备初始化
 * @param
 * @return
***********************************************************/
int hardware_init(void)
{
	//电机初始化
	//motor_init();
	//编码器初始化
	//encoder_init();
	//摄像头总转风初始化
	mt9v03x_csi_init();	
	//姿态传感器icm20602初始化
  //icm20602_init_spi();
	//陀螺仪零飘初始化
	//gyro_offset_init();
	//OpenArt初始化
  //openart_mini_init();
	//无线转串口通信初始化
	//seekfree_wireless_init();
	return 0;
}
/***********************************************************
 * @brief 软件初始化，软件线程或定时器初始化
 * @param
 * @return
***********************************************************/
int software_init(void)
{
	buzzer_thread_init();//蜂鸣器线程初始化
	display_thread_init();//显示线程初始化
	button_init();//按键检测定时器初始化
	//timer_pit_init();//定时器初始化
	//camera_thread_init();
	return 0;
}



 