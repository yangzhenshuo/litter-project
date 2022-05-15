#ifndef _SERVER_H
#define _SERVER_H

#include "headfile.h"

void server_thread_init(void);

extern rt_sem_t server1_sem, server2_sem; //路径规划信号量

#endif