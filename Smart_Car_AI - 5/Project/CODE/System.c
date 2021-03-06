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
#include "TSP.h"
#include "communicate.h"
#include "server.h"

SystemSettingsTypedef SystemSettings;                   //系统信息
CarInfoTypedef CarInfo;                                 //小车信息
ControlPidTypedef SpeedControlPid = {15.0,2.5,0.0};   //速度控制pid参数初始化
ControlPidTypedef AngleControlPid = {4.2, 0, 30};       //角度pid参数
ControlPidTypedef PositionControlPid = {0.06,0.05, 0}; //位置pid参数
CarPulseTypedef CarPulse = {0, 0, 0, 0};                //编码器初值
GyroOffsetTypedef GyroOffset = {0.0, 0.0, 0.0};         //陀螺仪零飘初始化数据
ICMTypedef icm = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};        // ICM数据初始化
speedTypedef speed = {0.0, 0.0};                        //速度初始化
positionTypedef position = {0,0};                      //位置初始化化
coordinateTypedef coordinate = {0, 0};                  //初始坐标
DotTypedef dot[20];                                     //目标坐标
float dtt = 0.020;                                      //积分时间
//事件
rt_event_t event1;
rt_event_t run_event;
//服务信号量
rt_sem_t server1_sem;
rt_sem_t server2_sem;
rt_sem_t server3_sem;

extern rt_timer_t timer1;
/***********************************************************
 * @brief 运行时参数初始化
 * @param
 * @return
 ***********************************************************/
static inline void CarInfoInit(void)
{
  CarInfo.SpeedSet_x = 0; // x轴速度设置
  CarInfo.SpeedSet_y = 0; // y轴速度设置
  CarInfo.SpeedSet_z = 0; // z轴速度设置

  CarInfo.RealSpeed = 0;   //真实速度
  CarInfo.AngleSet = 0;    //角度设置
  CarInfo.PositionSet = 0; //位置设置

  CarInfo.pitch = 0.0;
  CarInfo.roll = 0.0;
  CarInfo.yaw = 0.0; //真实角度

  CarInfo.delet1 = 0.0;
  CarInfo.delet2 = 0.0;
  CarInfo.delet3 = 0.0;
  CarInfo.delet4 = 0.0;

  CarInfo.goal_angle = 0.0;
	CarInfo.goal_ditance = 0.0;

  CarInfo.BinaryMethod = 2; //二值化方法(起始二值化化方法最大类间法）

  CarInfo.BinaryThreshold = 200; //二值化阈值
  CarInfo.RunDistance1 = 0.0;    //小车运行距离
  CarInfo.RunDistance2 = 0.0;
	CarInfo.dotnum = 0;
  CarInfo.Iscorrect = 'F'; //矫正姿态是否打开
	CarInfo.angle=0.0;       //总钻风角度差值
  CarInfo.distance1 = 0.0; //总钻风y轴差值
  CarInfo.distance2 = 0.0; //总钻风x轴差值

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
 * @brief 设置初始化
 * @param
 * @return
 ***********************************************************/
static inline void SystemSettingsInit(void)
{
  SystemSettings.IsFound_dot = 'F';             //是否开始目标点寻找
  SystemSettings.IsAiOn = 'F';                  //识别模式是否打开
  SystemSettings.Binary_start = 'T';            //阈值第一次求取
  SystemSettings.IsControl = 'T';              //是否开启控制
  SystemSettings.Complete = 'F'; 
  SystemSettings.Is_straight = 'F';                //是否开启识别
  SystemSettings.FuzzyEnable = 'T';             //是否启动角度模糊控制
  SystemSettings.ChangeIEnable = 'T';           //是否启动速度变积分控制
}
/***********************************************************
 * @brief 车体信息初始化
 * @param
 * @return
 ***********************************************************/
void CarInformation_init(void)
{
  /********运行时参数初始化初始化*********/
  CarInfoInit();
  /*************设置初始化**************/
  SystemSettingsInit();
}
/***********************************************************
 * @brief 事件初始化
 * @param
 * @return
 ***********************************************************/
int event_init(void)
{
	//创造二值化更新事件
  event1 = rt_event_create("event1", RT_IPC_FLAG_FIFO);
	//创造分布运动事件
	run_event = rt_event_create("event1", RT_IPC_FLAG_FIFO);
	//创造服务信号量
	server1_sem = rt_sem_create("server1", 0, RT_IPC_FLAG_FIFO);
	server2_sem = rt_sem_create("server2", 0, RT_IPC_FLAG_FIFO);
	server3_sem = rt_sem_create("server3", 0, RT_IPC_FLAG_FIFO);
	return 0;
}
/***********************************************************
 * @brief 硬件设备初始化
 * @param
 * @return
 ***********************************************************/
int hardware_init(void)
{
  
  //编码器初始化
  encoder_init();
  //姿态传感器icm20602初始化
  icm20602_init_spi();
	gyro_offset_init();
  //摄像头总转风初始化
  mt9v03x_csi_init();
  // OpenArt初始化
  openart_mini_init();
  //无线转串口通信初始化
  seekfree_wireless_init();
  return 0;
}
/***********************************************************
 * @brief 软件初始化，软件线程或定时器初始化
 * @param
 * @return
 ***********************************************************/
int thread_init(void)
{
	//电机初始化
  motor_init();
  buzzer_thread_init();      //蜂鸣器线程初始化（优先级13）
  display_thread_init();     //显示线程初始化(优先级14)
  server1_thread_init();     //服务1线程初始化（优先级11）
  server2_thread_init();     //服务2线程初始化（优先级11）
  BinaryRenew_thread_init(); //二值化更新线程（优先级12）
  return 0;
}
/***********************************************************
 * @brief 定时器初始化
 * @param
 * @return
 ***********************************************************/
int timer_init(void)
{
  button_init();     //按键检测定时器初始化
  timer1_pit_init(); //定时器初始化
  return 0;
}