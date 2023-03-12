#ifndef CODE_SYSTEM_H
#define CODE_SYSTEM_H

#include "headfile.h"
#define dtt 0.005
#define DIAN_PIN B13

int hardware_init(void);
int thread_init(void);
int timer_init(void);
int communication_init(void);

void stop_car(void);
void CarInformation_init(void);
void gyro_offset_init(void);
void smotor_up(void);
void smotor_down(void);
void smotor_down1(void);

typedef struct
{
  float x;
  float y;
  float z;
} GyroOffsetTypedef; //零点漂移

typedef struct
{
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
} ICMTypedef; // ICM数据

typedef struct
{
  float x;
  float y;
} speedTypedef; //加速度计得出的速度

typedef struct
{
  float x;
  float y;
} positionTypedef; //当前实时位置

typedef struct
{
  int x;
  int y;
} coordinateTypedef; //位置坐标化

typedef struct
{
  float SpeedSet_x; // x轴速度设置
  float SpeedSet_y; // y轴速度设置
  float SpeedSet_z; // z轴速度设置
  float SpeedSet;   //平面速度设置
  float delet1;
	float delet2;
	float delet3;
	float delet4;
  float RealSpeed; //真实速度
  float AngleSet;  //角度设置
  float pitch;     //俯仰角
  float roll;      //横滚角
  float yaw;       //航向角

  int32 PositionSet; //位置设置

  uint8 BinaryMethod; //二值化方法
  uint8 BinaryThreshold; //二值化阈值
	float white_proportion;//白点占比
	
  double RunDistance1;   //小车运行距离
  double RunDistance2;
  int dotnum;              //搜索到的目标点的数量
	int goal_x;				//目标角度
	int goal_y;			//目标距离
	
	int num;          //正在寻找点为第几个
  float distance1;
  float distance2;
} CarInfoTypedef;

typedef struct
{
  int16 L1; //前左轮的编码器值
  int16 L2; //后左轮的编码器值
  int16 R1; //前右轮的编码器值
  int16 R2; //后右轮的编码器值
} CarPulseTypedef;

typedef struct
{
  float P;
  float I;
  float D;
} ControlPidTypedef;

typedef struct
{
  uint8 IsFound_dot;             //是否开始目标点寻找
  uint8 IsAiOn;                  //识别模式是否打开
  uint8 Is_control;              //是否开启控制
  uint8 Binary_start;            //第一次求阈值
  uint8 IsCarry;                 //搬运标志位
  uint8 Is_arrival;              //是否到达设定位置
  uint8 FuzzyEnable;             //是否启动角度模糊控制
  uint8 ChangeIEnable;           //是否启动速度变积分控制
	uint8 Iscorrect;
} SystemSettingsTypedef;

//目标点的信息
typedef struct
{
  uint8 x;
  uint8 y;
	uint8 flag;
} DotTypedef;

extern GyroOffsetTypedef GyroOffset;
extern ICMTypedef icm;
extern speedTypedef speed;
extern positionTypedef position;
extern coordinateTypedef orignal_pos;
extern coordinateTypedef coordinate;
extern CarInfoTypedef CarInfo;
extern CarPulseTypedef CarPulse;
extern ControlPidTypedef SpeedControlPid;
extern ControlPidTypedef AngleControlPid;
extern ControlPidTypedef PositionPid_y;
extern ControlPidTypedef PositionPid_x;
extern ControlPidTypedef CorrectControlPid;
extern SystemSettingsTypedef SystemSettings;
extern DotTypedef dot[20];
extern int dot_num;
extern int rec[4][2];                                          //矩形框
//事件集
extern rt_event_t event1;
extern rt_event_t run_event;
//服务信号量
extern rt_sem_t server1_sem;
extern rt_sem_t server2_sem;
extern rt_sem_t server3_sem;
extern rt_sem_t server4_sem;
//开关信号量
extern rt_sem_t key1_sem;
extern rt_sem_t key2_sem;
extern rt_sem_t key3_sem;
extern rt_sem_t key4_sem;
#endif