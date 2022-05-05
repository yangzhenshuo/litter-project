/*
 * serve.c
 * 服务管理
 * Author: 杨镇硕
 */
#include "System.h"
#include "server.h"
#include "image.h"
#include "TSP.h"

rt_sem_t server1_sem;

/***********************************************
 * @brief 智能车开启进行目标点的搜寻并规划路线
 * @param
 * @return
 * *********************************************/
void server_find_dot(void *parameter)
{
  rt_thread_mdelay(10); //延迟等待Finsh组件初始化
  char test[] = "enter1\n";
	while(1)
	{
  rt_sem_take(server1_sem, RT_WAITING_FOREVER);
  // Found_dot_info();
  Scan_line();
  seekfree_wireless_send_buff((uint8 *)test, sizeof(test) - 1);
	}
}
/***********************************************************
 * @brief 姿态矫正
 * @param
 * @return
 ***********************************************************/
void posture_rectify(void *parameter)
{
  char test[] = "enter2\n";
  //寻找顶点
  Scan_line();
  seekfree_wireless_send_buff((uint8 *)test, sizeof(test) - 1);
  // find_top_angle();
}
/***********************************************************
 * @brief 路径规划线程初始化
 * @param
 * @return
 ***********************************************************/
void server_thread_init(void)
{
  rt_thread_t server1;
  //创建服务一信号量
  server1_sem = rt_sem_create("server1", 0, RT_IPC_FLAG_FIFO);
  server1 = rt_thread_create("server1_thread", //线程名称
                             server_find_dot,  //线程入口函数
                             RT_NULL,          //入口参数
                             1024,             //线程栈
                             11,                //优先级
                             30);              //时间片
  if (RT_NULL != server1)
  {
    rt_thread_startup(server1);
  }
}