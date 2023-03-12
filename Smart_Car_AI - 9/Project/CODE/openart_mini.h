#ifndef _openartart_mini_h
#define _openartart_mini_h

#include "headfile.h"

extern uint8 DetectResult;           //当前识别结果0:未识别,1:动物,2:交通工具,3:水果
void SendCommand(uint8 type);
void OpenArtMiniSoftInit(void);
void openart_mini_init(void);

#endif
