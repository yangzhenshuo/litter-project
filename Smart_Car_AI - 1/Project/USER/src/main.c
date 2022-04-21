/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		IAR 8.3 or MDK 5.28
 * @Target core		NXP RT1064DVL6A
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 ********************************************************************************************************************/


//整套推荐IO查看Projecct文件夹下的TXT文本


//打开新的工程或者工程移动了位置务必执行以下操作
//第一步 关闭上面所有打开的文件
//第二步 project  clean  等待下方进度条走完

//下载代码前请根据自己使用的下载器在工程里设置下载器为自己所使用的

#include "headfile.h"

#include "System.h"
#include "image.h"
#include "display.h"
//#include "timer_pit.h"
//#include "encoder.h"
//#include "buzzer.h"
//#include "button.h"
//#include "motor.h"
//#include "openart_mini.h"
//#include "smotor.h"

rt_sem_t camera_sem;
uint32 use_time;
extern uint8 BinaryImage[IMAGE_H][IMAGE_W]; //存放二值图

INIT_APP_EXPORT(software_init);
INIT_APP_EXPORT(hardware_init);
int main(void)
{
		  
	  camera_sem = rt_sem_create("camera", 0, RT_IPC_FLAG_FIFO);
		//初始化pit定时器用来计时
		pit_init();
	  //CarInfoInit();
	  //SystemSettingsInit();
    EnableGlobalIRQ(0);
    while (1)
    {
		    //等待摄像头采集完毕
        rt_sem_take(camera_sem, RT_WAITING_FOREVER);
        //开始处理摄像头图像
				pit_start(PIT_CH0);
        BinaryImageConvert(2);
				use_time = pit_get_ms(PIT_CH0);
				pit_close(PIT_CH0);
				//发送图片数据到上位机
				//csi_seekfree_sendimg_03x(USART_1,BinaryImage[0],MT9V03X_CSI_W,MT9V03X_CSI_H);
        //根据图像计算出车模与赛道之间的位置偏差
        
        
        
        //根据偏差进行PD计算
        
        
        //PD计算之后的值用于寻迹舵机的控制
			
			
			// 进入临界区 避免打印的时候被其他线程打断
			//rt_kprintf("main_end:%d\n",use_time);
			
			rt_thread_mdelay(100);
    }
}

  



