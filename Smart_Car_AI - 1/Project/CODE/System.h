#ifndef CODE_SYSTEM_H
#define CODE_SYSTEM_H

#include "headfile.h"

int hardware_init(void);
int software_init(void);

typedef struct
{
	float x;
	float y;
	float z;
}GyroOffsetTypedef;//零点漂移

typedef struct
{
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
}ICMTypedef;//ICM数据

typedef struct
{
	float x;
	float y;
}speedTypedef;//加速度计得出的速度

typedef struct
{
	float x;
	float y;
}positionTypedef;//当前实时位置

typedef struct
{
	int x;
	int y;
}coordinateTypedef;//位置坐标化

typedef struct
{
	uint8 IsRun;                 //是否运行
  uint8 IsOutGarage;           //是否出库
  uint8 IsCameraDetectRun;     //摄像头检测是否运作
  uint8 IsMotorStalled;        //电机是否堵转
  uint8 IsMotorDiffrientialOn; //电机差速是否开启

  uint8 IsAiOn;      //识别模式是否打开
  uint8 IsCounterOn; //两分钟定时器是否打开
  uint8 SecCount;    //定时器计数值（秒）

  float SpeedSet;  //速度设置
  float RealSpeed; //真实速度
  float AngleSet;  //角度设置
  float RealAngle; //真实角度
	float pitch;//俯仰角
	float roll;//横滚角
	float yaw;//航向角

  int32 PositionSet; //位置设置

  uint8 BinaryThreshold; //二值化阈值
  int32 RunDistance;     //小车运行距离
}CarInfoTypedef;

typedef struct
{
	int16 L1;//前左轮的编码器值
	int16 L2;//后左轮的编码器值
	int16 R1;//前右轮的编码器值
	int16 R2;//后右轮的编码器值
}CarPulseTypedef;

typedef struct
{
	float P;
	float I;
	float D;
}ControlPidTypedef;

typedef struct
{
  uint8 WirelessEnable; //是否启动无线串口
  uint8 ScreenEnable;   //是否启动显示屏

  uint8 PostureReportEnable;     //是否启动姿态上报
  uint8 ImageStatusReportEnable; //是否启动图像处理模式上报
  uint8 AiEnable;                //是否开启识别
  uint8 FuzzyEnable;             //是否启动角度模糊控制
  uint8 ChangeIEnable;           //是否启动速度变积分控制

  uint8 BinaryMethod; //二值化方法

  uint16 StraightSpeed;       //直道速度
  uint16 InTurnSpeed;         //入弯速度
  uint16 OutTurnSpeed;        //出弯速度
  uint16 InRampSpeed;         //入坡道速度
  uint16 OutRampSpeed;        //出坡道速度
  uint16 InForkSpeed;         //入三岔速度
  uint16 OutForkSpeed;        //出三岔速度
  uint16 InCircleSpeed;       //入圆环速度
  uint16 OutCircleSpeed;      //出圆环速度
  uint16 ApriltagSearchSpeed; //搜索Apriltag速度
  uint16 TargetSearchSpeed;   //搜索靶子速度

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
extern SystemSettingsTypedef SystemSettings;

static inline void CarInfoInit(void);
static inline void SystemSettingsInit(void);

#endif