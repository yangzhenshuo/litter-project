#include "ProjectMath.h"

/***********************************************************
 * @brief 标度变换
 * @param Input 输入
 * @param InputMin 输入最小值
 * @param InputMax 输入最大值
 * @param OutputMin 输出最小值
 * @param OutputMax 输出最大值
 * @return 转换后的结果
***********************************************************/
float Scale(float Input, float InputMin, float InputMax, float OutputMin, float OutputMax)
{
	float Output;
	Output = (Input - InputMin) / ((InputMax - InputMin) / (OutputMax - OutputMin)) + OutputMin;
	return Output;
}

/***********************************************************
 * @brief 计算异或结果
 * @param
 * @return
***********************************************************/
unsigned char XorGet(char *str, unsigned char len)
{
	unsigned char i = 0;
	unsigned char sum = 0;
	for (i = 0; i < len; i++)
	{
		sum ^= *str++;
	}
	return sum;
}

/***********************************************************
 * @brief 检验异或结果是否正确
 * @param
 * @return
***********************************************************/
unsigned char XorCheck(char *str, unsigned char len, unsigned char checksum)
{
	unsigned char sum = 0;
	sum = XorGet(str, len);
	if (sum == checksum)
		return 1;
	else
		return 0;
}

/***********************************************************
 * @brief 计算累加和
 * @param
 * @return
***********************************************************/
unsigned char SumGet(char *dat, char len)
{
	char i, sum = 0;
	for (i = 0; i < len; i++)
	{
		sum += *dat++;
	}
	return sum;
}

/***********************************************************
 * @brief 16位crc计算
 * @param
 * @return
***********************************************************/
unsigned short CRC16Calculate(unsigned char *buff, unsigned char len)
{
	unsigned short CRC_Temp;
	unsigned char i, j;
	CRC_Temp = 0xffff;

	for (i = 0; i < len; i++)
	{
		CRC_Temp ^= buff[i];
		for (j = 0; j < 8; j++)
		{
			if (CRC_Temp & 0x01)
				CRC_Temp = (CRC_Temp >> 1) ^ 0xa001;
			else
				CRC_Temp = CRC_Temp >> 1;
		}
	}
	return (CRC_Temp);
}