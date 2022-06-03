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

rt_sem_t server1_sem;
rt_sem_t server2_sem;
extern rt_mailbox_t buzzer_mailbox;
/***********************************************
 * @brief 智能车开启进行目标点的搜寻并规划路线
 * @param
 * @return
 * *********************************************/
void server_find_dot(void *parameter)
{
  rt_thread_mdelay(10); //延迟等待Finsh组件初始化
  char test[] = "enter1\n";
	int n;
	int i;
	double s;
  while (1)
  {
    rt_sem_take(server1_sem, RT_WAITING_FOREVER);
		for(i=0; i < CarInfo.dotnum; i++)
		{
			if(dot[i].flag == 1)
			{
				if((dot[i].x-1-coordinate.x)==0)
				{
					CarInfo.AngleSet=0;
				}
				else 
				{
					if(atan2((dot[i].y-coordinate.y),(dot[i].x-coordinate.x))*180/PI<=-90&&atan2((dot[i].y-coordinate.y),(dot[i].x-coordinate.x))*180/PI>=-180)
				  {
					   CarInfo.AngleSet=atan2((dot[i].y-coordinate.y),(dot[i].x-coordinate.x))*180/PI+270;
					}
					else CarInfo.AngleSet=atan2((dot[i].y-coordinate.y),(dot[i].x-coordinate.x))*180/PI-90;
				}
				while(CarInfo.yaw!=CarInfo.AngleSet){}
				position.x=sqrt((dot[i].x-1-coordinate.x)*(dot[i].x-1-coordinate.x)+(dot[i].y-coordinate.y)*(dot[i].y-coordinate.y));
				s=(CarInfo.RunDistance1 / 210) * 5.5 * PI;
				while(s<position.x){}
				//找到下一个点关闭巡点标志
				SystemSettings.IsFound_dot = 'F';
				dot[i].flag =0;
				n++;
				coordinate.x=dot[i].x-1;
				coordinate.y=dot[i].y;
				break;
			}
		}
		if(n == CarInfo.dotnum)
		{
			//所有图片识别完成把原点位置给设定值
			 if((1-coordinate.x)==0)
				{
					CarInfo.AngleSet=0;
				}
				else 
				{
					if(atan2((1-coordinate.y),(1-coordinate.x))*180/PI<=-90&&atan2((1-coordinate.y),(1-coordinate.x))*180/PI>=-180)
				  {
					   CarInfo.AngleSet=atan2((1-coordinate.y),(1-coordinate.x))*180/PI+270;
					}
					else CarInfo.AngleSet=atan2((1-coordinate.y),(1-coordinate.x))*180/PI-90;
				}
				while(CarInfo.yaw!=CarInfo.AngleSet){}
				position.x=sqrt((1-coordinate.x)*(1-coordinate.x)+(1-coordinate.y)*(1-coordinate.y));
				while(s<position.x){}
				MotorStopped();
				SystemSettings.IsControl = 'F';                    
			  SystemSettings.IsFound_dot = 'F';
		}
    seekfree_wireless_send_buff((uint8 *)test, sizeof(test) - 1);
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
  //创建服务一信号量
  server1_sem = rt_sem_create("server1", 0, RT_IPC_FLAG_FIFO);
  server1 = rt_thread_create("server1_thread", //线程名称
                             server_find_dot,  //线程入口函数
                             RT_NULL,          //入口参数
                             1024,             //线程栈
                             11,               //优先级
                             30);              //时间片
  if (RT_NULL != server1)
  {
    rt_thread_startup(server1);
  }
}
/***********************************************************
 * @brief 姿态矫正
 * @param
 * @return
 ***********************************************************/
void pos_rectify(void *parameter)
{
  char test[] = "rectify\n";
  //寻找顶点
  while (1)
  {
    rt_sem_take(server2_sem, RT_WAITING_FOREVER);
    Computing_angle();
        //更新设置角度
    CarInfo.angle = CarInfo.current_angle;
    //判断是否旋转完成
    if (CarInfo.current_angle <= 2)
     {
    find_midpoint();
    //给出中点x轴像素坐标（控制到94）
    CarInfo.distance2 = 94-midpoint[0]; //x轴偏差
    CarInfo.distance1 = midpoint[1] - 117;//y轴偏差
    if (CarInfo.distance1<3 && CarInfo.distance1>-3 && CarInfo.distance2 == 0)
    {
      CarInfo.Iscorrect = 'F';
			MotorStopped();
			SystemSettings.IsControl = 'F';            
			rt_mb_send(buzzer_mailbox, 100);
    }
    }
    //seekfree_wireless_send_buff((uint8 *)test, sizeof(test) - 1);
  }
}
/***********************************************************
 * @brief 矫正线程初始化
 * @param
 * @return
 ***********************************************************/
void server2_thread_init(void)
{
  rt_thread_t server2;
  //创建服务一信号量
  server2_sem = rt_sem_create("server2", 0, RT_IPC_FLAG_FIFO);
  server2 = rt_thread_create("server2_thread", //线程名称
                             pos_rectify,      //线程入口函数
                             RT_NULL,          //入口参数
                             1024,             //线程栈
                             11,               //优先级
                             30);              //时间片
  if (RT_NULL != server2)
  {
    rt_thread_startup(server2);
  }
}