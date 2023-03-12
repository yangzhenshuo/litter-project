/*
 * display.c
 * ips1.14屏幕显示
 * Author: 杨镇硕
 */
#include "headfile.h"
#include "System.h"
#include "display.h"
#include "encoder.h"
#include "image.h"

/***********************************************************
 * @brief 显示线程入口函数
 * @param
 * @return
***********************************************************/
void display_entry(void *parameter)
{
	  //ips114_clear(WHITE);
	  rt_thread_mdelay(10);										// 延时等待finsh初始化完毕
    while(1)
    {
			ips114_showchar(rec[0][0],rec[0][1],'X');
			ips114_showchar(rec[1][0],rec[1][1],'X');
			ips114_showchar(rec[2][0],rec[2][1],'X');
			ips114_showchar(rec[3][0],rec[3][1],'X');
			//ips114_displayimage032_zoom(BinaryImage[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 240, 135);	//显示摄像头图像 
    }
    
}
/***********************************************************
 * @brief 屏幕初始化
 * @param
 * @return
***********************************************************/
void display_thread_init(void)
{
    rt_thread_t tid;
    
    //初始化屏幕
    ips114_init();
	  ips114_showstr(0,0,"Initializing... ");
	  ips114_showstr(0,0,"      OK...     ");
    //创建显示线程 优先级设置为31
    tid = rt_thread_create("display", //线程名称
														display_entry, //线程入口函数
														RT_NULL, //入口参数
														1024, //线程栈
														17, //优先级
														30);//时间片
    
    //启动显示线程
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}