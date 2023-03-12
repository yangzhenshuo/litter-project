#include "encoder.h"
#include "System.h"

#define ENCODER1_QTIMER		QTIMER_1
#define ENCODER1_A			QTIMER1_TIMER0_C0
#define ENCODER1_B			QTIMER1_TIMER1_C1

#define ENCODER2_QTIMER		QTIMER_1
#define ENCODER2_A			QTIMER1_TIMER2_C2
#define ENCODER2_B			QTIMER1_TIMER3_C24

#define ENCODER3_QTIMER   QTIMER_2
#define ENCODER3_A      QTIMER2_TIMER0_C3
#define ENCODER3_B      QTIMER2_TIMER3_C25

#define ENCODER4_QTIMER   QTIMER_3
#define ENCODER4_A      QTIMER3_TIMER2_B18
#define ENCODER4_B      QTIMER3_TIMER3_B19
/***********************************************************
 * @brief 编码器初始化
 * @param
 * @return
***********************************************************/
void encoder_init(void)
{
    qtimer_quad_init(ENCODER1_QTIMER, ENCODER1_A, ENCODER1_B);
	  qtimer_quad_init(ENCODER2_QTIMER, ENCODER2_A, ENCODER2_B);
	  qtimer_quad_init(ENCODER3_QTIMER, ENCODER3_A, ENCODER3_B);
	  qtimer_quad_init(ENCODER4_QTIMER, ENCODER4_A, ENCODER4_B);
}
/***********************************************************
 * @brief 获得编码器的值
 * @param
 * @return
***********************************************************/
void EncoderPulseGet(void)
{
    CarPulse.R1=qtimer_quad_get(ENCODER1_QTIMER, ENCODER1_A);
    qtimer_quad_clear(ENCODER1_QTIMER, ENCODER1_A);
    CarPulse.L1=-qtimer_quad_get(ENCODER2_QTIMER, ENCODER2_A);
    qtimer_quad_clear(ENCODER2_QTIMER, ENCODER2_A);
	  CarPulse.L2=-qtimer_quad_get(ENCODER3_QTIMER, ENCODER3_A);
    qtimer_quad_clear(ENCODER3_QTIMER, ENCODER3_A);
	  CarPulse.R2=qtimer_quad_get(ENCODER4_QTIMER, ENCODER4_A);
    qtimer_quad_clear(ENCODER4_QTIMER, ENCODER4_A);
}