/*
 * Communicate.c
 * 上位机通信协议
 * Author: han
 */
#include "System.h"
#include "Communicate.h"
#include "SEEKFREE_WIRELESS.h"
#include "stdio.h"
#include "stdlib.h"
#include "image.h"
/***********************************************************
 * @brief 目标速度
 * @param
 * @return
 ***********************************************************/
static inline void Reportspeed(void)
{
  char temp[100];
  sprintf(temp, "%d,%d,%d,%d\n",(int)CarPulse.L1, (int)CarPulse.R1, (int)CarPulse.L2, (int)CarPulse.R2);
  seekfree_wireless_send_buff((uint8 *)temp, strlen(temp));
}
/***********************************************************
 * @brief 目标数据
 * @param
 * @return
 ***********************************************************/
static inline void Reportgoal(void)
{
  char temp[100];
  sprintf(temp, "(%d,%d),(%d,%d);\n",(int)CarInfo.goal_x, (int)CarInfo.goal_y, (int)position.x, (int)position.y);
  seekfree_wireless_send_buff((uint8 *)temp, strlen(temp));
}
/***********************************************************
 * @brief 状态
 * @param
 * @return
 ***********************************************************/
static inline void ReportStatus(void)
{
  char temp[100];
  sprintf(temp, "%c,%c,(%d,%d);\n", SystemSettings.IsFound_dot, SystemSettings.IsCarry, (int)position.x, (int)position.y);
  seekfree_wireless_send_buff((uint8 *)temp, strlen(temp));
}
/***********************************************************
 * @brief 目标数据
 * @param
 * @return
 ***********************************************************/
static inline void ReportMap(void)
{
  char temp[100];
  sprintf(temp, "(%d,%d),(%d,%d);\n", (int)CarInfo.goal_x, (int)CarInfo.goal_y, (int)CarInfo.AngleSet, (int)coordinate.y);
  seekfree_wireless_send_buff((uint8 *)temp, strlen(temp));
}
/***********************************************************
 * @brief 地图点的数据
 * @param
 * @return
 ***********************************************************/
static inline void ReportDot(void)
{
  char temp1[100];
  sprintf(temp1, "%d,%d->%d,%d->%d,%d->%d,%d->%d,%d->%d,%d->%d,%d->%d,%d;\n", dot[0].x, dot[0].y, dot[1].x, dot[1].y, dot[2].x, dot[2].y,
          dot[3].x, dot[3].y, dot[4].x, dot[4].y, dot[5].x, dot[5].y, dot[6].x, dot[6].y, dot[7].x, dot[7].y);
  seekfree_wireless_send_buff((uint8 *)temp1, strlen(temp1));
}
/***********************************************************
 * @brief 数据汇报
 * @param
 * @return
 ***********************************************************/
void Report_info(void)
{
	char test[] = "Map:";
	char test1[] = "Dot:";
	char test2[] = "Flag:";
	char test3[] = "goal:";
	char test4[] = "\n";
//  seekfree_wireless_send_buff((uint8 *)test3,sizeof(test3)-1);
//  Reportgoal();
	seekfree_wireless_send_buff((uint8 *)test2,sizeof(test2)-1);
  ReportStatus();
	seekfree_wireless_send_buff((uint8 *)test,sizeof(test)-1);
  ReportMap();
//	seekfree_wireless_send_buff((uint8 *)test1,sizeof(test1)-1);
//  ReportDot();
//	  Reportspeed();
	seekfree_wireless_send_buff((uint8 *)test4,sizeof(test4)-1);
}
/***********************************************************
 * @brief 数据汇报
 * @param
 * @return
 ***********************************************************/
void Report(void)
{
	char temp[100];
	char test[] = "\n";
	sprintf(temp, "%c,%c,%c, %c;\n", SystemSettings.Is_control, SystemSettings.IsFound_dot, SystemSettings.Iscorrect, SystemSettings.IsCarry);
  seekfree_wireless_send_buff((uint8 *)temp, strlen(temp));
	sprintf(temp, "(%d,%d),(%d,%d);\n",(int)CarInfo.goal_x, (int)CarInfo.goal_y, (int)CarInfo.white_proportion, (int)CarInfo.yaw);
  seekfree_wireless_send_buff((uint8 *)temp, strlen(temp));
	sprintf(temp, "(%d,%d);\n", (int)position.x, (int)position.y);
  seekfree_wireless_send_buff((uint8 *)temp, strlen(temp));
	sprintf(temp, "(%d,%d);\n", (int)barycentre[0], (int)barycentre[1]);
  seekfree_wireless_send_buff((uint8 *)temp, strlen(temp));
  seekfree_wireless_send_buff((uint8 *)test,sizeof(test)-1);
}