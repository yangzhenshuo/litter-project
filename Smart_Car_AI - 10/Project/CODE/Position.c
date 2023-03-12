/*
 * Position.c
 * 定位
 * Author: 杨镇硕
 */
#include "Position.h"
#include "System.h"
/***********************************************************
 * @brief icm20602进行惯性导航的相对定位
 * @param
 * @return
***********************************************************/
//IMU定位
void location(void)
{
	icm.acc_x = (9.8*icm_acc_x)/4096.0;
  icm.acc_y = (9.8*icm_acc_y)/4096.0;
	icm.acc_z = (9.8*icm_acc_z)/4096.0;
  icm.gyro_x = (icm_gyro_x-GyroOffset.x)/16.4;
  icm.gyro_y = (icm_gyro_y-GyroOffset.y)/16.4;
  icm.gyro_z = (icm_gyro_z-GyroOffset.z)/16.4;
	//求得航向角yaw与当前位置
	if(CarInfo.yaw>180||CarInfo.yaw<-180)
		CarInfo.yaw = 0;
	CarInfo.yaw += icm.gyro_z*dtt;
	//x方向的速度
	speed.x += icm.acc_x*dtt;
	//y方向的速度
	speed.y += icm.acc_y*dtt;
	//位置更新
	position.x += speed.x*dtt+0.5*icm.acc_x*dtt*dtt;
	position.y += speed.y*dtt+0.5*icm.acc_y*dtt*dtt;
}
/***********************************************************
 * @brief 位置的坐标化
 * @param
 * @return
***********************************************************/
void coordinatograph(void)
{
	coordinate.x = position.x/0.2;
	coordinate.y = position.y/0.2;
}


