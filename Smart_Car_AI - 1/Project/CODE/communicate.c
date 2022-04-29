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

/***********************************************************
 * @brief 图像数据
 * @param
 * @return
***********************************************************/
void ReportImageStatus(void)
{
    char temp[100];
     sprintf(temp, "%d,%d;\n", CarInfo.BinaryThreshold, SystemSettings.BinaryMethod);
     seekfree_wireless_send_buff((uint8 *)temp, strlen(temp));
}
