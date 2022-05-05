#ifndef CODE_SYSTEM_H
#define CODE_SYSTEM_H

#include "headfile.h"

int hardware_init(void);
int thread_init(void);
int timer_init(void);

static inline void CarInfoInit(void);
static inline void SystemSettingsInit(void);
void CarInformation_init(void);
void SystemStart(void);
void System_stop(void);

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

  double current_angle; //当前车身偏离角度

  int32 PositionSet; //位置设置

  uint8 BinaryMethod; //二值化方法

  uint8 BinaryThreshold; //二值化阈值
  int32 RunDistance;     //小车运行距离
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
  uint8 Binary_start;     //第一次求阈值
  uint8 ImageStatusReportEnable; //是否启动图像处理模式上报
  uint8 AiEnable;                //是否开启识别
  uint8 FuzzyEnable;             //是否启动角度模糊控制
  uint8 ChangeIEnable;           //是否启动速度变积分控制

  uint16 ApriltagSearchSpeed; //搜索Apriltag速度

} SystemSettingsTypedef;

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

#endif