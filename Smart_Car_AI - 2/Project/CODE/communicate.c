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
 * @brief 车体数据
 * @param
 * @return
 ***********************************************************/
static inline void ReportStatus(void)
{
  char temp[100];
  sprintf(temp, "%d,%d,%d,%d;\n", CarInfo.BinaryThreshold, CarInfo.BinaryMethod, midpoint[0], midpoint[1]);
  seekfree_wireless_send_buff((uint8 *)temp, strlen(temp));
}
/***********************************************************
 * @brief 控制数据
 * @param
 * @return
 ***********************************************************/
static inline void Reportcontrol(void)
{
  char temp[100];
  sprintf(temp, "%d,%d,%d,%d,%d,%d;\n", (int)CarInfo.distance1, (int)CarInfo.distance2, (int)CarInfo.delet3, (int)CarInfo.delet4, (int)CarInfo.delet1, (int)CarInfo.delet2);
  seekfree_wireless_send_buff((uint8 *)temp, strlen(temp));
}
/***********************************************************
 * @brief 地图数据
 * @param
 * @return
 ***********************************************************/
static inline void ReportMap(void)
{
  char temp[100];
  sprintf(temp, "%d,%d,%d,%d;\n", BoxData.LeftBorder, BoxData.RightBorder, BoxData.HighBorder, BoxData.LowBorder);
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
  sprintf(temp1, "%d,%d->%d,%d->%d,%d->%d,%d->%d,%d->%d,%d;\n", Dot[0].x, Dot[0].y, Dot[1].x, Dot[1].y, Dot[2].x, Dot[2].y,
          Dot[3].x, Dot[3].y, Dot[4].x, Dot[4].y, Dot[5].x, Dot[5].y);
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
	char test1[] = " Dot:";
	char test2[] = "info:";
	char test3[] = "control:";
	seekfree_wireless_send_buff((uint8 *)test3,sizeof(test3)-1);
	Reportcontrol();
	seekfree_wireless_send_buff((uint8 *)test2,sizeof(test2)-1);
  ReportStatus();
//	seekfree_wireless_send_buff((uint8 *)test,sizeof(test)-1);
//  ReportMap();
//	seekfree_wireless_send_buff((uint8 *)test1,sizeof(test1)-1);
//  ReportDot();
}