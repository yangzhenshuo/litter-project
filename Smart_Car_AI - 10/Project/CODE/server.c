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
#include "openart_mini.h"

#define square(x1, y1, x2 ,y2) ((x2) - (x1))*((x2) - (x1))+((y2) - (y1))*((y2) - (y1));
extern rt_mailbox_t buzzer_mailbox;
/***********************************************
 * @brief 重新初始化里程计
 * @param
 * @return
 * *********************************************/
static inline void renew_Carpulse()
{
//	rt_enter_critical();      //临界保护
	CarInfo.RunDistance1 = 0;                    
	CarInfo.RunDistance2 = 0;
//	rt_exit_critical();	
}
/***********************************************
 * @brief 规划下一个点
 * @param p 目标点数组指针
 * @param pos 车体绝对位置
 * @return 下一个点的数组索引号
 * *********************************************/
static inline int sort_dot(DotTypedef *p, coordinateTypedef pos)
{
	DotTypedef ready_dot[20]; //处于准备状态的点
	int temp, j = 0;
	int s = 50000;            //初始距离
	int s1;                   //车与目标点的距离
	for(int i =0; i < 20; i++)//提取处于准备状态的目标点
	{
		if(p[i].flag == 1)
		{
			ready_dot[j].x = p[i].x;
			ready_dot[j].y = p[i].y;
			ready_dot[j].flag = i;
			j++;
		}
	}
	for(int i =0; i < j; i++)
	{
		s1 = square(pos.x, pos.y, ready_dot[i].x, ready_dot[i].y);
		if(s1 < s)
		{
			s = s1;
			temp = ready_dot[i].flag;
		}
	}
	return temp;
}
/***********************************************
 * @brief 智能车开启进行目标点的搜寻并规划路线（转角直行方案）
 * @param
 * @return
 * *********************************************/
static inline void alter_flag1()
{
	SystemSettings.Is_arrival = 'F';		         //关闭到达标志位
	SystemSettings.Is_control = 'T';
}
void server_find_dot(void *parameter)
{
  rt_thread_mdelay(10); //延迟等待Finsh组件初始化
	int i, j;
	float Hy;
	float Hx;
  while (1)
  {
    rt_sem_take(server1_sem, RT_WAITING_FOREVER);
		renew_Carpulse();                           //重新初始化里程计
		if(CarInfo.num < CarInfo.dotnum)            //没有找完所有图片
		{
			j = sort_dot(dot, orignal_pos);            //找到最近点的索引号
			dot[j].flag = 0;                           //状态变更为已被寻找
			Hy = DetAbs(dot[j].y ,orignal_pos.y);
			Hx = DetAbs(dot[j].x ,orignal_pos.x);
			if(Hy >= Hx) 
			{
				Hy = Hy-1;
				if(dot[j].y >= orignal_pos.y) 
				{
					CarInfo.goal_y = dot[j].y - 1;
					CarInfo.goal_x = dot[j].x;
					CarInfo.angle=0;
				}
				else 
				{
					CarInfo.goal_y = dot[j].y + 1;
					CarInfo.goal_x = dot[j].x;
					CarInfo.angle = 180;
				}
			}
			else 
			{
				Hx = Hx - 1;
				if(dot[j].x >= orignal_pos.x) 
				{
					CarInfo.goal_x = dot[j].x - 1;
					CarInfo.goal_y = dot[j].y;
					CarInfo.angle = -90;
				}
				else 
				{
					CarInfo.goal_x = dot[j].x + 1;
					CarInfo.goal_y = dot[j].y;
					 CarInfo.angle = 90;
				}
			}
			if(Hx == 0 && CarInfo.goal_y > orignal_pos.y)
			{
				CarInfo.AngleSet = 0;
			}
			else if(Hx == 0 && CarInfo.goal_y <= orignal_pos.y)
			{
				CarInfo.AngleSet = 180;
			}
			else 
			{
				if(CarInfo.goal_y <= orignal_pos.y && CarInfo.goal_x > orignal_pos.x)
				{
					CarInfo.AngleSet = -90-atan(Hy/Hx)*180/PI;
				}
				if(CarInfo.goal_y <= orignal_pos.y && CarInfo.goal_x < orignal_pos.x)
				{
					CarInfo.AngleSet = atan(Hy/Hx)*180/PI+90;
				}
				if(CarInfo.goal_y >= orignal_pos.y && CarInfo.goal_x > orignal_pos.x)
				{
					CarInfo.AngleSet = atan(Hy/Hx)*180/PI-90;
				}
				if(CarInfo.goal_y >= orignal_pos.y && CarInfo.goal_x < orignal_pos.x)
				{
					CarInfo.AngleSet = 90-atan(Hy/Hx)*180/PI;
				}
			}
			CarInfo.s = sqrt(Hx*Hx+Hy*Hy);
		}
		if(CarInfo.num == CarInfo.dotnum)            //点寻完，开始回归出发点
		{
			CarInfo.goal_x = 0;
			CarInfo.goal_y = 0;
			Hy = DetAbs(CarInfo.goal_y,orignal_pos.y);
			Hx = DetAbs(CarInfo.goal_x,orignal_pos.x);
			if(Hx == 0 && CarInfo.goal_y > orignal_pos.y)
			{
				CarInfo.AngleSet = 0;
			}
			else if(Hx == 0 && CarInfo.goal_y <= orignal_pos.y)
			{
				CarInfo.AngleSet = 180;
			}
			else 
			{
				if(CarInfo.goal_y <= orignal_pos.y && CarInfo.goal_x > orignal_pos.x)
				{
					CarInfo.AngleSet = -90-atan(Hy/Hx)*180/PI;
				}
				if(CarInfo.goal_y <= orignal_pos.y && CarInfo.goal_x < orignal_pos.x)
				{
					CarInfo.AngleSet = atan(Hy/Hx)*180/PI+90;
				}
				if(CarInfo.goal_y >= orignal_pos.y && CarInfo.goal_x > orignal_pos.x)
				{
					CarInfo.AngleSet = atan(Hy/Hx)*180/PI-90;
				}
				if(CarInfo.goal_y >= orignal_pos.y && CarInfo.goal_x < orignal_pos.x)
				{
					CarInfo.AngleSet = 90-atan(Hy/Hx)*180/PI;
				}
			}
			CarInfo.s = sqrt(Hx*Hx+Hy*Hy);
		}
		rt_sem_release(server2_sem);
//		SystemSettings.Is_arrival = 'F';		         //关闭到达标志位
  }
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
	int flag = 0;
	while(1)
	{
		rt_sem_take(server2_sem, RT_WAITING_FOREVER);
		SystemSettings.Is_arrival = 'F';		         //关闭到达标志位
		SystemSettings.Is_control = 'T';			
		if(CarInfo.num < CarInfo.dotnum) //巡点中
		{
			if(DetAbs(CarInfo.AngleSet, CarInfo.yaw) <= 1) //判断转到对应角度
			{
				position.x = CarInfo.s;
				if(CarInfo.white_proportion > 15) //判断到达点位置
			  {
					CarInfo.AngleSet = CarInfo.angle;			
					flag = 1;
			  }
				if(CarInfo.white_proportion < 15 && DetAbs(coordinate.y, position.x * 200) < 10)
				{
					position.x = CarInfo.s + 1;
				}
			}
			if(DetAbs(CarInfo.AngleSet, CarInfo.yaw) <= 1 && flag == 1) //判断转到对应角度
				  {
						rt_mb_send(buzzer_mailbox, 500);
				    SystemSettings.Is_arrival = 'T'; //到达标志位开启
				    SystemSettings.Iscorrect = 'T';  //开启矫正
						flag = 0;
          }		
		}
		else                                 //巡点完成回归出发点
		{
			if(DetAbs(CarInfo.AngleSet, CarInfo.yaw) <= 1) //判断转到对应角度
			{
			   position.x = CarInfo.s;
			}
			if(DetAbs(coordinate.y, CarInfo.s * 200) == 0) //判断回到出发点
			{
				rt_mb_send(buzzer_mailbox, 500);
				SystemSettings.Is_arrival = 'T'; //到达标志位开启
				SystemSettings.Is_control = 'F'; //控制关闭
				MotorStopped();                  //电机停止
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
static inline void alter_flag()
{
	SystemSettings.Iscorrect = 'F';   //关矫正
	renew_Carpulse();
	position.x = 0;	
	MotorStopped();                   //停止电机
	SystemSettings.IsAiOn = 'T';			//开启识别
}
void visual_pursit(void *parameter)
{
	rt_thread_mdelay(10); //延迟等待Finsh组件初始化
	rt_uint32_t e;
	int condition = 4;
	while(1)
	{
		rt_sem_take(server3_sem, RT_WAITING_FOREVER);
		if(CarInfo.white_proportion <= 5)
		{
			CarInfo.distance1 = -40;
			CarInfo.distance2 = 0;
		}
		else
		{
			CarInfo.distance2 = barycentre[0] - 75;                    //x轴偏差
			CarInfo.distance1 = 40 - barycentre[1];                     //y轴偏差	
		}
		if (Abs(CarInfo.distance1) < 4 && Abs(CarInfo.distance2) < 4) //矫正成功
		{
			rt_mb_send(buzzer_mailbox, 500);
			orignal_pos.x = CarInfo.goal_x;                           //更新车的绝对位置坐标
			orignal_pos.y = CarInfo.goal_y;	
			alter_flag();                                             //变更标志位
		}				
	}
}
/***********************************************************
 * @brief 搬运图片
 * @param
 * @return
 ***********************************************************/
void carry_image(void *parameter)
{
	rt_thread_mdelay(10); //延迟等待Finsh组件初始化
	rt_uint32_t e;
  while (1)
  {
    //等待搬运事件的接收
    if(rt_event_recv(run_event, (1|2|3),
           RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
           RT_WAITING_FOREVER, &e) == RT_EOK)
     {
			 //放下舵机摆臂
			 smotor_down();
			 //检测是否落下
			 rt_thread_mdelay(500);
			 //给电磁铁通电
			 gpio_set(DIAN_PIN, 1);		 
			 //吸起来图片
			 smotor_up();
			 rt_thread_mdelay(500);
			 switch(e)
			 {
				 case 1: CarInfo.goal_x = 0; CarInfo.goal_y = orignal_pos.y;  CarInfo.AngleSet=90; CarInfo.s = orignal_pos.x - CarInfo.goal_x; break;  //动物在地图的左边
				 case 2: CarInfo.goal_x = orignal_pos.x; CarInfo.goal_y = 20;  CarInfo.AngleSet=0; CarInfo.s = CarInfo.goal_y - orignal_pos.y; break;  //交通工具在地图的上边
				 case 3: CarInfo.goal_x = 20; CarInfo.goal_y = orignal_pos.y;  CarInfo.AngleSet=-90; CarInfo.s = CarInfo.goal_x - orignal_pos.x; break;  //水果在地图的右边
			 }
			 //里程计初始化
			 renew_Carpulse();                
			 //开启搬运
			 SystemSettings.IsCarry = 'T';
		 }
		 rt_sem_take(server4_sem, RT_WAITING_FOREVER);
		 //放下图片
//		 smotor_down();
//		 rt_thread_mdelay(500);
//		 gpio_set(DIAN_PIN, 0);
//		 smotor_up();
		 //开始下一个巡点
		 //放下图片
		 switch(DetectResult)
		 {	
			 case 1:gpio_set(DIAN_PIN, 0);rt_thread_mdelay(500);break;
			 case 2:gpio_set(DIAN_PIN, 0);rt_thread_mdelay(500);break;
			 case 3:gpio_set(DIAN_PIN, 0);rt_thread_mdelay(500);break;
		 }
		 position.x = 0;
		 SystemSettings.IsFound_dot = 'T';
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
 * @brief	运动线程初始化
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
 * @brief 视觉追踪线程初始化
 * @param
 * @return
 ***********************************************************/
void server3_thread_init(void)
{
  rt_thread_t server3;
  
  server3 = rt_thread_create("server3_thread", //线程名称
                             carry_image,      //线程入口函数
                             RT_NULL,          //入口参数
                             1024,             //线程栈
                             11,               //优先级
                             10);              //时间片
  if (RT_NULL != server3)
  {
    rt_thread_startup(server3);
  }
}
/***********************************************************
 * @brief 搬运线程初始化
 * @param
 * @return
 ***********************************************************/
void server4_thread_init(void)
{
  rt_thread_t server4;
  
  server4 = rt_thread_create("server4_thread", //线程名称
                             visual_pursit,      //线程入口函数
                             RT_NULL,          //入口参数
                             1024,             //线程栈
                             11,               //优先级
                             10);              //时间片
  if (RT_NULL != server4)
  {
    rt_thread_startup(server4);
  }
}
