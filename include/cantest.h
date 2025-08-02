
#ifndef CANTEST_H
#define CANTEST_H

#include <stdio.h>
#include <pthread.h>
#include "data.h"
extern DB1 *db;
extern pthread_mutex_t mutex_db;
int can1_send_msgs(int,int,int);
int can2_send_msgs(int,int,int);
int can2_receive_msgs(void);
int init_can(void);
void* test_can(void *arg);

#endif



