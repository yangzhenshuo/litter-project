#include "encoder.h"
#include "System.h"

#define ENCODER1_QTIMER		QTIMER_1
#define ENCODER1_A			QTIMER1_TIMER0_C0
#define ENCODER1_B			QTIMER1_TIMER1_C1

#define ENCODER2_QTIMER		QTIMER_1
#define ENCODER2_A			QTIMER1_TIMER2_C2
#define ENCODER2_B			QTIMER1_TIMER3_C24

int16 speedL,speedR;
/***********************************************************
 * @brief 编码器初始化
 * @param
 * @return
***********************************************************/
void encoder_init(void)
{
    qtimer_quad_init(ENCODER1_QTIMER, ENCODER1_A, ENCODER1_B);
	  qtimer_quad_init(ENCODER2_QTIMER, ENCODER2_A, ENCODER2_B);
}

/***********************************************************
 * @brief 获得编码器的值
 * @param
 * @return
***********************************************************/
void EncoderPulseGet(void)
{
    speedL = qtimer_quad_get(ENCODER1_QTIMER, ENCODER1_A);
    qtimer_quad_clear(ENCODER1_QTIMER, ENCODER1_A);
    speedR = qtimer_quad_get(ENCODER2_QTIMER, ENCODER2_A);
    qtimer_quad_clear(ENCODER2_QTIMER, ENCODER2_A);
}