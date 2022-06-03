/*
 * serve.c
 * 服务管理
 * Author: 杨镇硕
 */
#include "System.h"
#include "server.h"
#include "image.h"
#include "TSP.h"
#include "motor.h"
#include "ProjectMath.h"

extern rt_mailbox_t buzzer_mailbox;

/***********************************************
 * @brief 智能车开启进行目标点的搜寻并规划路线
 * @param
 * @return
 * *********************************************/
void server_find_dot(void *parameter)
{
  rt_thread_mdelay(10); //延迟等待Finsh组件初始化
	int i;
	float Hy;
	float Hx;
  while (1)
  {
    rt_sem_take(server1_sem, RT_WAITING_FOREVER);
		CarInfo.RunDistance1 = 0;
		CarInfo.RunDistance2 = 0;
		for(i = 0; i < CarInfo.dotnum ; i++)
		{
			if(dot[i].flag == 1)
			{
				//该点标志位置为零
				dot[i].flag =0;
				CarInfo.goal_x = dot[i].x;
				CarInfo.goal_y = dot[i].y;
				position.x = CarInfo.goal_y - orignal_pos.y - 1;
				position.y = CarInfo.goal_x - orignal_pos.x;
				break;
			}
		}
		if(CarInfo.num == CarInfo.dotnum)
		{
				CarInfo.goal_x = 0;
				CarInfo.goal_y = 0;
				position.x = CarInfo.goal_y - orignal_pos.y;
				position.y = CarInfo.goal_x - orignal_pos.x;
		}
		SystemSettings.Is_arrival = 'F';		
  }
}
/***********************************************
 * @brief 内部数值更新
 * @param
 * @return
 * *********************************************/
static inline void renew_System()
{
	rt_enter_critical();
	MotorStopped();
	rt_exit_critical();	
}
/***********************************************************
 * @brief 运动控制
 * @param
 * @return
 ***********************************************************/
void run_control(void *parameter)
{
	rt_thread_mdelay(10); //延迟等待Finsh组件初始化
	rt_uint32_t e;
	while(1)
	{
		rt_sem_take(server2_sem, RT_WAITING_FOREVER);
		SystemSettings.Is_control = 'T';			
		if(CarInfo.num < CarInfo.dotnum)
		{
			if(DetAbs(coordinate.x, CarInfo.goal_x) < 2 && DetAbs(coordinate.y, CarInfo.goal_y) < 2)
			{
				rt_mb_send(buzzer_mailbox, 500);
				SystemSettings.Is_arrival = 'T';
				renew_System();
				orignal_pos.x = CarInfo.goal_x;
				orignal_pos.y = CarInfo.goal_y - 1;
				SystemSettings.Iscorrect = 'T';
			}
		}
		else
		{
			if(DetAbs(coordinate.x, CarInfo.goal_x) == 0 && DetAbs(coordinate.y, CarInfo.goal_y) == 0)
			{
				rt_mb_send(buzzer_mailbox, 500);
				SystemSettings.Is_arrival = 'T';
				SystemSettings.Is_control = 'F';
				MotorStopped();
				orignal_pos.x = CarInfo.goal_x;
				orignal_pos.y = CarInfo.goal_y;				
			}					
		}
		}
}
/***********************************************************
 * @brief 视觉追踪
 * @param
 * @return
 ***********************************************************/
void visual_pursit(void *parameter)
{
	rt_thread_mdelay(10); //延迟等待Finsh组件初始化
	rt_uint32_t e;
	while(1)
	{
		rt_sem_take(server3_sem, RT_WAITING_FOREVER);
		find_trackpoint();
		if(trackpoint[0] == 118 && trackpoint[1] == 1)
		{
			MotorStopped();
		}
		//给出中点x轴像素坐标（控制到94）
		CarInfo.distance2 = trackpoint[0] - 82; //x轴偏差
		CarInfo.distance1 = 51 - trackpoint[1];//y轴偏差	
		if (Abs(CarInfo.distance1)<4 && Abs(CarInfo.distance2) < 5)
		{
			rt_mb_send(buzzer_mailbox, 100);
			SystemSettings.Iscorrect = 'F';
			SystemSettings.Is_control = 'F';			
			MotorStopped();
			SystemSettings.IsAiOn = 'T';			
		}				
	}
}
/***********************************************************
 * @brief 姿态矫正
 * @param
 * @return
 ***********************************************************/
void pos_rectify(void *parameter)
{
	rt_thread_mdelay(10); //延迟等待Finsh组件初始化
  char test[] = "rectify\n";
	rt_uint32_t e;
  //寻找顶点
  while (1)
  {
    rt_sem_take(server3_sem, RT_WAITING_FOREVER);
		 //更新设置角度
    CarInfo.AngleSet = CarInfo.angle - 90;  
    //判断是否旋转完成
    if(rt_event_recv(run_event, (1),
           RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
           RT_WAITING_FOREVER, &e) == RT_EOK)
     {
				if (CarInfo.distance1<4 && CarInfo.distance1>-3 && CarInfo.distance2 == 0)
				{
					SystemSettings.Iscorrect = 'F';
					MotorStopped();
//					SystemSettings.Is_rotation = 'F';            
					rt_mb_send(buzzer_mailbox, 100);
				}
    }
    //seekfree_wireless_send_buff((uint8 *)test, sizeof(test) - 1);
  }
}

/***********************************************************
 * @brief 巡点线程
 * @param
 * @return
 ***********************************************************/
void server1_thread_init(void)
{
  rt_thread_t server1;
  
  server1 = rt_thread_create("server1_thread", //线程名称
                             server_find_dot,  //线程入口函数
                             RT_NULL,          //入口参数
                             1024,             //线程栈
                             11,               //优先级
                             10);              //时间片
  if (RT_NULL != server1)
  {
    rt_thread_startup(server1);
  }
}
/***********************************************************
 * @brief	运动旋转线程初始化
 * @param
 * @return
 ***********************************************************/
void server2_thread_init(void)
{
  rt_thread_t server2;
  
  server2 = rt_thread_create("server2_thread", //线程名称
                             run_control,      //线程入口函数
                             RT_NULL,          //入口参数
                             1024,             //线程栈
                             11,               //优先级
                             10);              //时间片
  if (RT_NULL != server2)
  {
    rt_thread_startup(server2);
  }
}
/***********************************************************
 * @brief 运动直走线程初始化
 * @param
 * @return
 ***********************************************************/
void server3_thread_init(void)
{
  rt_thread_t server3;
  
  server3 = rt_thread_create("server3_thread", //线程名称
                             visual_pursit,      //线程入口函数
                             RT_NULL,          //入口参数
                             1024,             //线程栈
                             11,               //优先级
                             10);              //时间片
  if (RT_NULL != server3)
  {
    rt_thread_startup(server3);
  }
}
