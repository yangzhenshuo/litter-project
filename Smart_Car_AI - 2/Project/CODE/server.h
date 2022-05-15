#ifndef _SERVER_H
#define _SERVER_H

#include "headfile.h"

void server1_thread_init(void);
void server2_thread_init(void);

extern rt_sem_t server1_sem; //路径规划信号量
extern rt_sem_t server2_sem;

#endif