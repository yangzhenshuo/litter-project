/*
 * timer_pit.c
 * 软件定时器
 * Author: 杨镇硕
 */
#include "timer_pit.h"
#include "control.h"
#include "System.h"
#include "communicate.h"
#include "ProjectMath.h"
#include "buzzer.h"

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
		SpeedControl();
    //seekfree_wireless_send_buff((uint8 *)test, sizeof(test) - 1);
  }
  if (0 == (time % 20))
  {
    //位置控制与角度控制（控制周期20ms）
		if(SystemSettings.Is_rotation == 'T')
		{
			AngleControl();
		}
		if(SystemSettings.Is_straight == 'T')
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

void timer2_pit_entry(void *parameter)
{
	static uint32 time;
	time++;
	//0.5s判断一次是否旋转完成
	if(0 == time % 5)
	{
		if(SystemSettings.Is_rotation == 'T')
		{
			if(DetAbs(CarInfo.yaw, CarInfo.AngleSet) <= 1)
			{
				rt_event_send(run_event,(1));
			}
		}
		time = 0;
	}
	if(0 == time % 2)
	{
		if((CarInfo.RunDistance1 / 210) * 5.5 * PI > position.x*200 - 85 && SystemSettings.Is_straight == 'T')
		{
			rt_event_send(run_event, (2));
		}
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
void timer2_pit_init(void)
{
	timer2 =	rt_timer_create("timer2",                //定时器名称
                           timer2_pit_entry,        //超时函数入口指针
                           RT_NULL,                 //入口参数
                           50,                       //超时时间为0.05s
                           RT_TIMER_FLAG_PERIODIC); //周期性
	if (RT_NULL != timer2)
  {
    rt_timer_start(timer2);
  }
	
}

