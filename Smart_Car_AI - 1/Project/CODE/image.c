/*
 * Image.c
 * 图像处理与识别检测
 * Author:杨镇硕、张赛尧
 */
#include "image.h"
#include "System.h"
#include "ProjectMath.h"
#include "SEEKFREE_MT9V03X_CSI.h"

uint8 BinaryImage[IMAGE_H][IMAGE_W];   //存放二值图
uint8 hist_eq_image[IMAGE_H][IMAGE_W]; //存放直方图均衡化图片

static int Ysite = 0, Xsite = 0; // X、Y坐标
static uint8 *PicTemp;           //保存单行图像

BoxDataTypedef BoxData;         //方框信息
ImageStatusTypedef ImageStatus; //图像状态信息
BoxTypedef Box;                 //方框的长宽
DotTypedef Dot[20];             //存放点的x，y坐标
uint16 Num = 0;                      //目标点的数量

/***********************************************************
 * @brief 摄像头线程的入口函数
 * @param
 * @return
 ***********************************************************/
 void camera_entry()
{
	rt_thread_mdelay(10);										// 延时等待finsh初始化完毕
	while(1)
	{
		if(mt9v03x_csi_finish_flag)
     {
			 //发送图片数据到上位机
       csi_seekfree_sendimg_03x(USART_1,BinaryImage[0],MT9V03X_CSI_W,MT9V03X_CSI_H);
			 mt9v03x_csi_finish_flag = 0;
		}
	}
 }
/***********************************************************
 * @brief 摄像头图线显示线程初始化
 * @param
 * @return
 ***********************************************************/
 void camera_thread_init()
{
	rt_thread_t camera;
	camera = rt_thread_create("camera",//线程名称
														camera_entry, //线程入口函数
														RT_NULL, //线程入口参数
														1024, //线程栈大小1024字节
														31, //线程优先级
														30);//线程时间片，同优先级线程起作用

     //启动线程
     if(RT_NULL != camera)
     {
         rt_thread_startup(camera);
     }
 }
/***********************************************************
 * @brief 大津算法
 * @param image 图像地址
 * @param col 图像宽度（范围1~IMAGE_W）
 * @param row 图像高度（范围1~IMAGE_H）
 * @return 阈值
 ***********************************************************/
static inline uint8 OtsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
  uint16 width = col;
  uint16 height = row;
  int pixelCount[GrayScale]; //每个灰度级的像素个数
  float pixelPro[GrayScale]; //每个灰度级的像素概率
  int16 i, j, pixelSum = width * height / 4;
  uint8 threshold = 0;
  uint8 *data = image; //指向像素数据的指针
  for (i = 0; i < GrayScale; i++)
  {
    pixelCount[i] = 0;
    pixelPro[i] = 0.0;
  }
  //统计灰度级中每个像素在整幅图像中的个数
  int grayTmp = 0;
  uint32 graySum = 0; //灰度值和
  for (i = 0; i < height; i += 2)
  {
    for (j = 0; j < width; j += 2)
    {
      grayTmp = (int)data[i * IMAGE_W + j];
      pixelCount[grayTmp]++; //将像素值作为计数数组的下标
      graySum += grayTmp;    //计算图片总灰度值和
    }
  }
  float grayEverage = (float)graySum / pixelSum; //计算图像平均灰度

  //计算每个像素在整幅图像中的比例
  for (i = 0; i < GrayScale / 2; i++)
  {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }

  //遍历灰度级[0,255]
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = 0; j < GrayScale / 2; j++)
  {
    w0 += pixelPro[i];        //背景部分每个灰度值的像素点所占比列之和，即背景部分的比例
    u0tmp += j * pixelPro[i]; //背景部分，每个灰度值点的比例*灰度值

    w1 = 1 - w0; //背景
    u1tmp = graySum / pixelSum - u0tmp;

    u0 = u0tmp / w0;                                          //背景平均灰度
    u1 = u1tmp / w1;                                          //前景平均灰度
    u = u0tmp + u1tmp;                                        //全局平均灰度
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2); //计算方差
    if (deltaTmp > deltaMax)
    {
      deltaMax = deltaTmp;
      threshold = j;
    }
    if (deltaTmp < deltaMax)
    {
      threshold += 0;
      break;
    }
  }
  return threshold;
}
/***********************************************************
 * @brief 迭代法求阈值
 * @param image 图像地址
 * @param col 图像宽度（范围1~IMAGE_W）
 * @param row 图像高度（范围1~IMAGE_H）
 * @return 阈值
 ***********************************************************/
#define Thres 128 //阈值
#define ERROR 2   //误差

int16 EdgeThres = 18;  //晚上20  白天25 18
float Threshold0 = 0;  //大津法阈值
float Threshold1 = 58; //黑白迭代阈值
float Threshold2 = 60; //固定阈值

void Iteration_Threshould(uint8 *image, uint16 col_w, uint16 row_h)
{
  uint16_t i = 0, j = 0, N0 = 0, N1 = 0, flag = 0;
  float T0, T1, T2, T_center;
  uint32_t S0 = 0, S1 = 0;
  T2 = Threshold1;     //初始预估值
  uint8 *data = image; //指向像素数据的指针
  do
  {
    //分割图像为两部分
    for (i = 0; i < row_h; i++)
    {
      for (j = 0; j < col_w; j++)
      {
        if (image[i * col_w + j] < T2)
        {
          S0 += data[i * col_w + j]; // G2部分的像素灰度值之和
          N0++;                      // G2部分的像素点个数
        }
        else
        {
          S1 += data[i * col_w + j]; // G1部分的像素灰度值之和
          N1++;                      // G1部分的像素点个数
        }
      }
    }
    T0 = S0 / N0;             // G2部分的平均灰度值
    T1 = S1 / N1;             // G1部分的平均灰度值
    T_center = (T0 + T1) / 2; //新的阈值
    if (T2 < T_center)
    {
      if ((T_center - T2) > ERROR)
      {
        flag = 1;
      }
      else
      {
        flag = 0;
      }
    }
    else
    {
      if ((T2 - T_center) > ERROR)
      {
        flag = 1;
      }
      else
      {
        flag = 0;
      }
    }
    T2 = T_center;
    Threshold1 = T2;
  } while (flag);
}
/***********************************************************
 * @brief 图像直方图均衡化（效果不佳）
 ***********************************************************/
void HistEqImageConvert(uint8 *RawImagePtr, uint8 *HistEqImagePtr)
{
  int gray[256] = {0};                //记录每个灰度级别下的像素个数
  float gray_prob[256] = {0};         //记录灰度分布频率
  float gray_distribution[256] = {0}; //记录原始图像的累计密度
  int gray_equal[256] = {0};          //均衡化后的灰度值
  int gray_sum = 0;                   //像素总数
  gray_sum = IMAGE_H * IMAGE_W;

  //统计每个灰度下的像素个数
  for (int i = 0; i < gray_sum; i++)
  {
    gray[RawImagePtr[i]]++;
  }

  //统计原始图像的灰度分布频率
  for (int i = 0; i < 256; i++)
  {
    gray_prob[i] = (float)gray[i] / gray_sum;
  }
  //计算累计密度
  gray_distribution[0] = gray_prob[0];
  for (int i = 1; i < 256; i++)
  {
    gray_distribution[i] = gray_distribution[i - 1] + gray_prob[i];
  }
  //重新计算均衡化后的灰度值，四舍五入。参考公式：(N-1)*T+0.5
  for (int i = 0; i < 256; i++)
  {
    gray_equal[i] = (uint8)(255 * gray_distribution[i] + 0.5); //(int)(a+0.5)
  }
  //直方图均衡化,更新原图每个点的像素值
  for (int i = 0; i < gray_sum; i++)
  {
    HistEqImagePtr[i] = gray_equal[RawImagePtr[i]];
  }
}
/***********************************************************
 * @brief 图像二值化转换
 * @param type 0:大津法；1：迭代法；2：固定可调阈值;3:直方图均衡加大津
 * @return
 ***********************************************************/
void BinaryImageConvert(uint8 type)
{
  uint8 Threshold;
  if (type == 0)
  {

    Threshold0 = OtsuThreshold(mt9v03x_csi_image[0], IMAGE_W, IMAGE_H);
    Threshold = Threshold0;
  }
  if (type == 1)
  {
    Iteration_Threshould(mt9v03x_csi_image[0], IMAGE_W, IMAGE_H);
    Threshold = Threshold1;
  }
  if (type == 2)
    Threshold = Threshold2;
  if (type == 3)
  {
    uint8 *HistImagePtr = hist_eq_image[0];
    HistEqImageConvert(mt9v03x_csi_image[0], HistImagePtr);
    Threshold0 = OtsuThreshold(HistImagePtr, IMAGE_W, IMAGE_H);
    Threshold = Threshold0;
  }
  for (int i = 0; i < IMAGE_H; i++)
    for (int j = 0; j < IMAGE_W; j++)
    {
      if (mt9v03x_csi_image[i][j] > Threshold)
      {
        BinaryImage[i][j] = White;
      }
      else
      {
        BinaryImage[i][j] = Black;
      }
    }
	CarInfo.BinaryThreshold = Threshold;
}

/***********************************************************
 * @brief 描方框四边的线;寻找矩形左下点和右上点来求框的四点坐标
 * @return
 ***********************************************************/
static inline void Big_Scan(void)
{
  for (Ysite = 119; Ysite >= 60; Ysite--) //从左到右找从上到下，找边框
  {
    PicTemp = BinaryImage[Ysite];
    if (*(PicTemp + CENTER_POINT) == Black) //寻找左下点
    {
      //寻找底边的最近点
      for (int i = 1; i < 60; i++)
      {
        PicTemp = BinaryImage[Ysite - i];
        if (*(PicTemp + CENTER_POINT) == White)
        {
          Ysite = Ysite - i + 1;
          break;
        }
      }
      for (Xsite = CENTER_POINT; Xsite > 0; Xsite--) //找左右边线
      {
        if (*(PicTemp + CENTER_POINT - Xsite) == Black) //一旦找到左下点就break
        {
          //寻找左边的最近点
          for (int j = 1; j < 60; j++)
          {
            if (*(PicTemp + CENTER_POINT - Xsite + j) == White)
            {
              Xsite = CENTER_POINT - Xsite + j - 1;
              break;
            }
          }
          break;
        }
      }
      BoxData.LeftBorder = Xsite;
      BoxData.HighBorder = Ysite;
      break;
    }
  }
  for (Ysite = 0; Ysite < 60; Ysite++)
  {
    PicTemp = BinaryImage[Ysite];
    if (*(PicTemp + CENTER_POINT) == Black) //如果底边图像中点为黑，寻找右上点
    {
      //寻找顶边的最近点
      for (int i = 1; i < 60; i++)
      {
        PicTemp = BinaryImage[Ysite + i];
        if (*(PicTemp + CENTER_POINT) == White)
        {
          Ysite = Ysite + i - 1;
          break;
        }
      }
      for (Xsite = 0; Xsite < CENTER_POINT; Xsite--) //找左右边线
      {
        if (*(PicTemp + CENTER_POINT + Xsite) == Black)
          break;
      }
      BoxData.RightBorder = CENTER_POINT + Xsite;
      BoxData.LowBorder = Ysite;
      break;
    }
  }
  Box.High = BoxData.HighBorder - BoxData.LowBorder;
  Box.Wide = BoxData.RightBorder - BoxData.LeftBorder;
}
/***********************************************************
 * @brief 寻找目标点并且变换坐标且给出任意两点的距离
 * @return
 ***********************************************************/
void Found_dot_info()
{
  float k0 = 0.0, k1 = 0.0; //变换系数；k0为高的变换系数，k1为宽的变换系数
  Big_Scan();
  k0 = 500 / Box.High; // 500和700分别为实际场地的长宽
  k1 = 700 / Box.Wide;
  for (int i = BoxData.LeftBorder + 1; i < BoxData.RightBorder; i++)
    for (int j = BoxData.LowBorder + 1; j < BoxData.HighBorder; j++)
    {
      if (BinaryImage[j][i] == Black && BinaryImage[j][i + 1] == Black)
      {
        Dot[Num].x = i;
        Dot[Num].y = j;
        Num++;
      }
    }
  for (int i = 0; i < Num; i++)
  {
    Dot[i].x = (int)(Dot[i].x - BoxData.LeftBorder) * k1 / 20;
    Dot[i].y = (int)(BoxData.HighBorder - Dot[i].y) * k0 / 20;
  }
}