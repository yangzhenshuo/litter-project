/*
 * timer_pit.c
 * 软件定时器
 * Author: 杨镇硕
 */
#include "timer_pit.h"
#include "control.h"
#include "System.h"
#include "communicate.h"
rt_timer_t timer1, timer2;
char test[] = "1\n";
char test1[] = "2\n";
char test2[] = "3\n";

void timer1_pit_entry(void *parameter)
{
  // rt_thread_mdelay(10);										// 延时等待finsh初始化完毕
  static uint32 time;
  time++;

  if (0 == (time % 5))
  {
    //速度控制（控制周期5ms
		if(SystemSettings.IsControl == 'T')
		{
			SpeedControl();
		}
    //seekfree_wireless_send_buff((uint8 *)test, sizeof(test) - 1);
  }
  if (0 == (time % 20))
  {
    //位置控制与角度控制（控制周期20ms）
		if(SystemSettings.IsControl == 'T')
		{
			AngleControl();
		}
		if(SystemSettings.IsControl== 'T' && SystemSettings.Is_straight == 'T')
		{
			PositionControl();
		}		
  }
	if (0 == time % 100)
	{
		Report_info(); //向上位机发送信息
		time = 0;
	}
}


void timer1_pit_init(void)
{
  //创建定时器1 周期运行
  timer1 = rt_timer_create("timer1",                //定时器名称
                           timer1_pit_entry,        //超时函数入口指针
                           RT_NULL,                 //入口参数
                           1,                       //超时时间为0.001s
                           RT_TIMER_FLAG_PERIODIC); //周期性

  //启动定时器
  if (RT_NULL != timer1)
  {
    rt_timer_start(timer1);
  }
}

