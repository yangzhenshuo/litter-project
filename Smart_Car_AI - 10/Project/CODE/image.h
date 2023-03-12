#ifndef CODE_IMAGE_H
#define CODE_IMAGE_H

#include "SEEKFREE_MT9V03X_CSI.h"
#include "headfile.h"

#define IMAGE_H (MT9V03X_CSI_H)    //图像高度
#define IMAGE_W (MT9V03X_CSI_W)    //图像宽度
#define CENTER_POINT (IMAGE_W / 2) //图像中心横坐标
#define Black 0x00                 //黑色像素值
#define White 0xff                 //白色像素值
#define PixelValue(img, x, y) *(*((img) + y) + x)
//方框信息
typedef struct
{
  int LeftBorder;  ////左边界点
  int RightBorder; ////右边界点
  int HighBorder;  ////高边界点
  int LowBorder;   ////低边界点
} BoxDataTypedef;
//方框的具体信息
typedef struct
{
  int Wide; ////宽度
  int High; ////高度
} BoxTypedef;

extern BoxDataTypedef BoxData;
extern BoxTypedef Box;
extern uint8 BinaryImage[IMAGE_H][IMAGE_W];   //存放二值图
extern uint8 hist_eq_image[IMAGE_H][IMAGE_W]; //存放直方图均衡化图片
extern uint8 erode_image[IMAGE_H][IMAGE_W];   //腐蚀后的图
extern uint8 dilate_image[IMAGE_H][IMAGE_W];  //膨胀后的图
extern uint8 trackpoint[2];
extern uint8 barycentre[2];

void BinaryRenew_thread_init(void);
void Binary_renew(uint8 type);
void erode3(uint8 (*p)[IMAGE_W], uint8 (*p1)[IMAGE_W]);
void Binary_image(void);
void dilate3(uint8 (*p)[IMAGE_W], uint8 (*p1)[IMAGE_W]);
void opening(uint8 (*p)[IMAGE_W], uint8 (*p1)[IMAGE_W]);
void find_trackpoint(void);
void find_barycen(uint8 (*img)[IMAGE_W]);
double Computing_angle(uint8 (*p)[IMAGE_W]);

#endif