#ifndef CODE_PROJECTMATH_H
#define CODE_PROJECTMATH_H

#include "common.h"

#define PI 3.14159265358979f
#define DetAbs(x, y) ((x) > (y) ? (x) - (y) : (y) - (x)) //差绝对值
#define Abs(x) ((x) >= 0 ? (x) : -(x))

float Scale(float Input, float InputMin, float InputMax, float OutputMin, float OutputMax);
unsigned char XorGet(char *str, unsigned char len);
unsigned char XorCheck(char *str, unsigned char len, unsigned char checksum);
unsigned char SumGet(char *dat, char len);
unsigned short CRC16Calculate(unsigned char *buff, unsigned char len);

#endif
