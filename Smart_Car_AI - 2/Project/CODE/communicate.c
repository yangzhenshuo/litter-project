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
  sprintf(temp, "%d,%d,%d,%d;\n",(int)CarInfo.delet1, (int)CarInfo.delet2, (int)CarInfo.delet3, (int)CarInfo.delet4);
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
  sprintf(temp1, "%d,%d->%d,%d->%d,%d->%d,%d->%d,%d->%d,%d;\n", dot[0].x, dot[0].y, dot[1].x, dot[1].y, dot[2].x, dot[2].y,
          dot[3].x, dot[3].y, dot[4].x, dot[4].y, dot[5].x, dot[5].y);
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
//  	seekfree_wireless_send_buff((uint8 *)test3,sizeof(test3)-1);
//   	Reportcontrol();
//	seekfree_wireless_send_buff((uint8 *)test2,sizeof(test2)-1);
//  ReportStatus();
//	seekfree_wireless_send_buff((uint8 *)test,sizeof(test)-1);
//  ReportMap();
	seekfree_wireless_send_buff((uint8 *)test1,sizeof(test1)-1);
  ReportDot();
}