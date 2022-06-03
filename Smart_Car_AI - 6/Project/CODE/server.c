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
	int n;
	int i;
	float Hy;
	float Hx;
  while (1)
  {
    rt_sem_take(server1_sem, RT_WAITING_FOREVER);
		for(i = 0; i < CarInfo.dotnum; i++)
		{
			if(dot[i].flag == 1)
			{
				//该点标志位置为零
				dot[i].flag =0;
				Hy=DetAbs(dot[i].y,coordinate.y);
				Hx=DetAbs(dot[i].x,coordinate.x);
				if(Hy>=Hx) 
				{
					Hy=Hy-1;
					if(dot[i].y>=coordinate.y) dot[i].y=dot[i].y-1;
					else dot[i].y=dot[i].y+1;
				}
				else 
				{
					Hx=Hx-1;
					if(dot[i].x>=coordinate.x) dot[i].x=dot[i].x-1;
					else dot[i].x=dot[i].x+1;
				}
				if(Hx == 0)
				{
					if (dot[i].y > coordinate.y)
						CarInfo.goal_angle = 0;
					else
						CarInfo.goal_angle = 180;
				}
				else 
				{
					if(dot[i].y <= coordinate.y && dot[i].x>coordinate.x)
					{
						CarInfo.goal_angle = -90-atan(Hy/Hx)*180/PI;
					}
					if(dot[i].y<=coordinate.y&&dot[i].x<coordinate.x)
					{
						CarInfo.goal_angle = atan(Hy/Hx)*180/PI+90;
					}
					if(dot[i].y>=coordinate.y&&dot[i].x>coordinate.x)
					{
						CarInfo.goal_angle = atan(Hy/Hx)*180/PI-90;
					}
					if(dot[i].y>=coordinate.y&&dot[i].x<coordinate.x)
					{
						CarInfo.goal_angle = 90-atan(Hy/Hx)*180/PI;
					}
				}
				CarInfo.goal_ditance = sqrt(Hx*Hx+Hy*Hy);					
				n++;
				//发送运动信号量
				rt_sem_release(server2_sem);
				//更新坐标位置
				coordinate.x = dot[i].x;
				coordinate.y = dot[i].y;
				break;
			}
		}
		if(n == CarInfo.dotnum -1)
		{
				Hy=DetAbs(0,coordinate.y);
				Hx=DetAbs(0,coordinate.x);
			//所有图片识别完成把原点位置给设定值
				if(Hx == 0)
				{
					if (dot[i].y > coordinate.y)
						CarInfo.goal_angle = 0;
					else
						CarInfo.goal_angle = 180;
				}
				else 
				{
					if(dot[i].y <= coordinate.y && dot[i].x>coordinate.x)
					{
						CarInfo.goal_angle = -90-atan(Hy/Hx)*180/PI;
					}
					if(dot[i].y<=coordinate.y&&dot[i].x<coordinate.x)
					{
						CarInfo.goal_angle = atan(Hy/Hx)*180/PI+90;
					}
					if(dot[i].y>=coordinate.y&&dot[i].x>coordinate.x)
					{
						CarInfo.goal_angle = atan(Hy/Hx)*180/PI-90;
					}
					if(dot[i].y>=coordinate.y&&dot[i].x<coordinate.x)
					{
						CarInfo.goal_angle = 90-atan(Hy/Hx)*180/PI;
					}
				}
				CarInfo.goal_ditance = sqrt(Hx*Hx+Hy*Hy);
				rt_sem_release(server2_sem);
		}
  }
}
/***********************************************************
 * @brief 运动旋转控制
 * @param
 * @return
 ***********************************************************/
void run_rotation(void *parameter)
{
	rt_thread_mdelay(10); //延迟等待Finsh组件初始化
	rt_uint32_t e;
	while(1)
	{
		rt_sem_take(server2_sem, RT_WAITING_FOREVER);
		CarInfo.AngleSet = CarInfo.goal_angle;
		SystemSettings.Is_rotation = 'T';
		if(rt_event_recv(run_event, (1),
           RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
           RT_WAITING_FOREVER, &e) == RT_EOK)
		{
				rt_mb_send(buzzer_mailbox, 500);
				rt_sem_release(server3_sem);
				SystemSettings.Is_rotation = 'F';
		}
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
	SystemSettings.Is_straight = 'F';
	SystemSettings.Is_rotation = 'F';
	MotorStopped();	
	CarInfo.RunDistance1 = 0;
	CarInfo.RunDistance2 = 0;
	position.x = 0;
	rt_exit_critical();	
}
/***********************************************************
 * @brief 运动直行控制
 * @param
 * @return
 ***********************************************************/
void run_straight(void *parameter)
{
	rt_thread_mdelay(10); //延迟等待Finsh组件初始化
	rt_uint32_t e;
	while(1)
	{
		rt_sem_take(server3_sem, RT_WAITING_FOREVER);
		rt_thread_mdelay(1000);
		position.x = CarInfo.goal_ditance;
		SystemSettings.Is_straight = 'T';
		if(rt_event_recv(run_event, (2),
           RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
           RT_WAITING_FOREVER, &e) == RT_EOK)
		{
			rt_mb_send(buzzer_mailbox, 100);
			//更新内部记录值
			renew_System();
			//发送矫正控制信号量
			
		}
		
		rt_thread_mdelay(2000);
		SystemSettings.IsFound_dot = 'T';
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
    CarInfo.AngleSet = Computing_angle(BinaryImage) - 90;  
    //判断是否旋转完成
    if(rt_event_recv(run_event, (1),
           RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
           RT_WAITING_FOREVER, &e) == RT_EOK)
     {
				find_midpoint();
				//给出中点x轴像素坐标（控制到94）
				CarInfo.distance2 = 94-midpoint[0]; //x轴偏差
				CarInfo.distance1 = midpoint[1] - 117;//y轴偏差
				if (CarInfo.distance1<3 && CarInfo.distance1>-3 && CarInfo.distance2 == 0)
				{
					CarInfo.Iscorrect = 'F';
					MotorStopped();
					SystemSettings.Is_rotation = 'F';            
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
                             run_rotation,      //线程入口函数
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
                             run_straight,      //线程入口函数
                             RT_NULL,          //入口参数
                             1024,             //线程栈
                             11,               //优先级
                             10);              //时间片
  if (RT_NULL != server3)
  {
    rt_thread_startup(server3);
  }
}
