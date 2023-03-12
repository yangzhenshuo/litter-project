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
	rt_enter_critical();
	CarInfo.RunDistance1 = 0;                    
	CarInfo.RunDistance2 = 0;
	rt_exit_critical();	
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
 * @brief 智能车开启进行目标点的搜寻并规划路线
 * @param
 * @return
 * *********************************************/
void server_find_dot(void *parameter)
{
  rt_thread_mdelay(10); //延迟等待Finsh组件初始化
	int i, j;
  while (1)
  {
    rt_sem_take(server1_sem, RT_WAITING_FOREVER);
		renew_Carpulse();
		if(CarInfo.num < CarInfo.dotnum)            //没有找完所有图片
		{
			j = sort_dot(dot, orignal_pos);            //找到最近点的索引号
			dot[j].flag = 0;                           //状态变更为已被寻找
			CarInfo.goal_x = dot[j].x;                 //把点下一格子坐标赋值给目标位置
			CarInfo.goal_y = dot[j].y - 1;
		}
		if(CarInfo.num == CarInfo.dotnum)            //点寻完，开始回归出发点
		{
				CarInfo.goal_x = 0;
				CarInfo.goal_y = 0;
		}
		position.x = CarInfo.goal_y - orignal_pos.y; //位置控制的设定值
		position.y = CarInfo.goal_x - orignal_pos.x;
		SystemSettings.Is_arrival = 'F';		         //关闭到达标志位
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
	while(1)
	{
		rt_sem_take(server2_sem, RT_WAITING_FOREVER);
		SystemSettings.Is_control = 'T';			
		if(CarInfo.num < CarInfo.dotnum) //巡点中
		{
			if(DetAbs(coordinate.x, CarInfo.goal_x * 200) < 20 && DetAbs(CarInfo.goal_y * 200, coordinate.y) < 20) //判断到达点位置
			{
				rt_mb_send(buzzer_mailbox, 500);
				SystemSettings.Is_arrival = 'T'; //到达标志位开启
				MotorStopped();                  //电机停止
				SystemSettings.Iscorrect = 'T';  //开启矫正
			}
		}
		else                                 //巡点完成回归出发点
		{
			if(DetAbs(coordinate.x, CarInfo.goal_x * 200) == 0 && DetAbs(coordinate.y, CarInfo.goal_y * 200) == 0) //判断回到出发点
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
	SystemSettings.Is_control = 'F';	//关控制
	renew_Carpulse();                 //里程计初始化
	MotorStopped();                   //停止电机
	SystemSettings.IsAiOn = 'T';			//开启识别
}
static inline int status(coordinateTypedef pos, int x, int y)
{
	int s;
	if(pos.x <= x && pos.y <= y)  //目标在车的右上方
	{
		s = 1;
	}
	if(pos.x <= x && pos.y > y)   //目标在车的右下方
	{
		s = 2;
	}
	if(pos.x > x && pos.y <= y)  //目标在车的左上方
	{
		s = 3;
	}
	if(pos.x > x && pos.y > y)   //目标在车的左下方
	{
		s = 4;
	}
	return s;
}
void visual_pursit(void *parameter)
{
	rt_thread_mdelay(10); //延迟等待Finsh组件初始化
	rt_uint32_t e;
	int condition = 4;
	while(1)
	{
		rt_sem_take(server3_sem, RT_WAITING_FOREVER);
		condition = status(orignal_pos, CarInfo.goal_x, CarInfo.goal_y);
		if(CarInfo.white_proportion <= 5)
		{
			switch(condition)
			{
				case 1:CarInfo.distance1 = 40;CarInfo.distance2 = 40;break;
				case 2:CarInfo.distance1 = -40;CarInfo.distance2 = 40;break;
				case 3:CarInfo.distance1 = 40;CarInfo.distance2 = -40;break;
				case 4:CarInfo.distance1 = -40;CarInfo.distance2 = -40;break;
			}
		}
		else
		{
			CarInfo.distance2 = barycentre[0] - 75;                    //x轴偏差
			CarInfo.distance1 = 40 - barycentre[1];                     //y轴偏差	
		}	
		if (Abs(CarInfo.distance1)<5 && Abs(CarInfo.distance2) < 5) //矫正成功
		{
			rt_mb_send(buzzer_mailbox, 100);
			orignal_pos.x = CarInfo.goal_x;                           //更新车的绝对位置坐标
			orignal_pos.y = CarInfo.goal_y;	
			alter_flag();
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
			 switch(e)
			 {
				 case 1: CarInfo.goal_x = -1; CarInfo.goal_y = orignal_pos.y;  position.x = 0; position.y = CarInfo.goal_x - orignal_pos.x; break;  //动物在地图的左边
				 case 2: CarInfo.goal_x = orignal_pos.x; CarInfo.goal_y = 19; position.x = CarInfo.goal_y - orignal_pos.y; position.y = 0; break;  //交通工具在地图的上边
				 case 3: CarInfo.goal_x = 21; CarInfo.goal_y = orignal_pos.y; position.x = 0; position.y = CarInfo.goal_x - orignal_pos.x; break;  //水果在地图的右边
			 }
			 //里程计初始化
			 renew_Carpulse();                
			 //开启控制和搬运
			 SystemSettings.IsCarry = 'T';
			 SystemSettings.Is_control = 'T';
		 }
		 rt_sem_take(server4_sem, RT_WAITING_FOREVER);
		 //放下图片
		 switch(DetectResult)
		 {	
			 case 1:smotor_down1(); rt_thread_mdelay(400); gpio_set(DIAN_PIN, 0); smotor_up();break;
			 case 2:gpio_set(DIAN_PIN, 0);break;
			 case 3:smotor_down1(); rt_thread_mdelay(400); gpio_set(DIAN_PIN, 0); smotor_up();break;
		 }
		 //开始下一个巡点
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
