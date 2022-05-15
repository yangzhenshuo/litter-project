/*
 * timer_pit.c
 * 软件定时器
 * Author: 杨镇硕
 */
#include "encoder.h"
#include "motor.h"
#include "timer_pit.h"
#include "Position.h"
#include "control.h"
#include "image.h"
#include "System.h"

rt_timer_t timer1, timer2;

void timer1_pit_entry(void *parameter)
{
  // rt_thread_mdelay(10);										// 延时等待finsh初始化完毕
  static uint32 time;
  time++;

  if (0 == (time % 5) && SystemSettings.Is_control == 'T')
  {
    //速度控制与角度控制（控制周期5ms）
    SpeedControl();
  }
  if (0 == (time % 20) && SystemSettings.Is_control == 'T')
  {
    //位置控制（控制周期20ms）
    AngleControl();
    PositionControl();
  }
  // if (0 == (time % 100))
  //{
  //  /*************每100ms重新计算一次阈值**************/
  // Binary_threshold(CarInfo.BinaryMethod);
  //}
}

void timer2_pit_entry(void *parameter)
{
  static uint16 time;
  if (0 == time % 100)
  {
    
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
  //创建定时器2 周期运行
  timer2 = rt_timer_create("timer2",
                           timer2_pit_entry,
                           RT_NULL,
                           1,
                           RT_TIMER_FLAG_PERIODIC);
  //启动定时器
  if (RT_NULL != timer2)
  {
    rt_timer_start(timer2);
  }
}
