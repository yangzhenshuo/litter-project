#include "encoder.h"
#include "motor.h"
#include "timer_pit.h"
#include "Position.h"

void timer1_pit_entry(void *parameter)
{
    static uint32 time;
    time++;
    //采集陀螺仪数据与加速度数据
    get_icm20602_gyro_spi();
		get_icm20602_accdata_spi();	
    if(0 == (time%100))
    {
        //采集编码器数据
        EncoderPulseGet();
    }  
    
}

void timer2_pit_entry(void *parameter)
{
	static uint16 time;
	//实时定位
	location();
	if(0 == time%100)
	{
		//位置坐标化
		coordinatograph();
	}
}

void timer_pit_init(void)
{
    rt_timer_t timer1,timer2;
    
    //创建定时器 周期运行
    timer1 = rt_timer_create("timer1", //定时器名称
														 timer1_pit_entry, //超时函数入口指针
														 RT_NULL, //入口参数
														 1, //超时时间为0.001s
														 RT_TIMER_FLAG_PERIODIC);//周期性
    timer2 = rt_timer_create("timer2",
															timer2_pit_entry,
															RT_NULL,
															1,
															RT_TIMER_FLAG_PERIODIC);
    //启动定时器
    if(RT_NULL != timer1)
    {
        rt_timer_start(timer1);
    }
    if(RT_NULL != timer2)
		{
			rt_timer_start(timer2);
		}
}
