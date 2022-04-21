#ifndef _openartart_mini_h
#define _openartart_mini_h

#include "headfile.h"

typedef struct
{
  uint8 NumberDetectResult;   //数字识别结果
  uint8 AprilTagDetectResult; //AprilTag识别结果
  uint8 DetectResult;           //当前识别结果0:未识别,1:水果,2:动物，3：交通工具
} OpenArtMiniInfoTypedef;     //OpenArt Mini信息

extern OpenArtMiniInfoTypedef OpenArtMiniInfo;

void SendCommand2OpenArtMini(uint8 type);
void OpenArtMiniSoftInit(void);
void openart_mini_init(void);

#endif
