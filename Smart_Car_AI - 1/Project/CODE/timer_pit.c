#include "encoder.h"
#include "motor.h"
#include "timer_pit.h"
#include "Position.h"

void timer1_pit_entry(void *parameter)
{
    static uint32 time;
    time++;
    //�ɼ���������������ٶ�����
    get_icm20602_gyro_spi();
		get_icm20602_accdata_spi();	
    if(0 == (time%100))
    {
        //�ɼ�����������
        EncoderPulseGet();
    }  
    
}

void timer2_pit_entry(void *parameter)
{
	static uint16 time;
	//ʵʱ��λ
	location();
	if(0 == time%100)
	{
		//λ�����껯
		coordinatograph();
	}
}

void timer_pit_init(void)
{
    rt_timer_t timer1,timer2;
    
    //������ʱ�� ��������
    timer1 = rt_timer_create("timer1", //��ʱ������
														 timer1_pit_entry, //��ʱ�������ָ��
														 RT_NULL, //��ڲ���
														 1, //��ʱʱ��Ϊ0.001s
														 RT_TIMER_FLAG_PERIODIC);//������
    timer2 = rt_timer_create("timer2",
															timer2_pit_entry,
															RT_NULL,
															1,
															RT_TIMER_FLAG_PERIODIC);
    //������ʱ��
    if(RT_NULL != timer1)
    {
        rt_timer_start(timer1);
    }
    if(RT_NULL != timer2)
		{
			rt_timer_start(timer2);
		}
}