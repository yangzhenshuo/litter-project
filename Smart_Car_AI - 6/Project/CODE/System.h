#ifndef CODE_SYSTEM_H
#define CODE_SYSTEM_H

#include "headfile.h"

int hardware_init(void);
int thread_init(void);
int timer_init(void);
int communication_init(void);

void CarInformation_init(void);

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
  double RunDistance1;   //小车运行距离
  double RunDistance2;
  int dotnum;              //搜索到的目标点的数量
	float goal_angle;				//目标角度
	float goal_ditance;			//目标距离
	
  uint8 Iscorrect;
  float distance1;
  float distance2;
	float angle;
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
  uint8 Is_rotation;             //是否开启转角控制
  uint8 Binary_start;            //第一次求阈值
  uint8 Complete; 
  uint8 Is_straight;             //是否开启直行控制
  uint8 FuzzyEnable;             //是否启动角度模糊控制
  uint8 ChangeIEnable;           //是否启动速度变积分控制

} SystemSettingsTypedef;

//目标点的信息
typedef struct
{
  uint8 x;
  uint8 y;
	uint8 flag;
} DotTypedef;

extern float dtt;
extern GyroOffsetTypedef GyroOffset;
extern ICMTypedef icm;
extern speedTypedef speed;
extern positionTypedef position;
extern coordinateTypedef coordinate;
extern CarInfoTypedef CarInfo;
extern CarPulseTypedef CarPulse;
extern ControlPidTypedef SpeedControlPid;
extern ControlPidTypedef AngleControlPid;
extern ControlPidTypedef PositionControlPid;
extern SystemSettingsTypedef SystemSettings;
extern DotTypedef dot[20];
extern int dot_num;
extern rt_event_t event1;
extern rt_event_t run_event;
extern rt_sem_t server1_sem; //路径规划信号量
extern rt_sem_t server2_sem;
extern rt_sem_t server3_sem;

#endif