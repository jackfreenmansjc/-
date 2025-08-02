#ifndef __BOATCONTROL_H
#define __BOATCONTROL_H
#include <pthread.h>
#include <math.h>
#include "data.h"
extern pthread_mutex_t mutex;  //定义一个全局互斥锁
extern DB1 *db;					//定义一个全局数据表

#define CONTROL_FREQUENCY 1   //定义控制频率 1HZ
// 定义结构体来存储解析后的 GPS 数据
struct GPSData {
    float latitude;
    float longitude;
    float speed;
    char time[10];
};

double data_process_GRO(unsigned char *data,int length);
void data_process_PLC(unsigned char *data,int data_length);
void data_process_IPC(unsigned char *data);
void data_process_PLC_wei(unsigned char *data,int data_length);
void intToChars(int num, char *chars);
void floatToChars(float num_float, char *chars);
int charsToInt(char *chars);
float charsToFloat(char *chars);
float linearInterpolation(float x, float x0, float x1, float y0, float y1);
float get_board_ang(float x_point,float xx_point);
// 函数原型
void parseNMEA(char *sentence, struct GPSData *data);

float hexToFloat(unsigned char *bytes);

float LOS_main(DB1 *db_record_s,ONOFF onoff);
#endif
