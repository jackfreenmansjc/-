
/********************************************************************************
                                    测试编译说明
将CAN1,CAN2数据线并联,测试数据收发。(实际使用的时候可以只开启配置一个CAN通道)
将  libcanbus.tar 拷贝到目录  /usr/local/lib 下面解压
配置环境变量:                export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
编译方法                     gcc cantest.cpp -lcanbus -lusb-1.0 -o can2test
静态编译方法                 gcc cantest.cpp -lrt -Wl,-Bstatic -lcanbus -lusb-1.0 -Wl,-Bdynamic -Wl,-ludev -o can2test
运行                       ./can2test
如果提示ludev错误，请安装libudev-dev：   sudo apt-get install libudev-dev
如果编译过程中找不到cc1plus 请安装              sudo apt-get install --reinstall build-essential
*********************************************************************************/



#include "canbus.h"
#include <string.h>
#include "cantest.h"
#include <pthread.h>
#include <stdint.h>
#include "data.h"
#include <time.h>
int dev = 0;
int cpot0 = 0;
int cpot1 = 1;
Can_Msg  txmsg[100];
Can_Msg  rxmsg[100];

uint8_t calculate_checksum(uint8_t *data, uint8_t counter) {
    //counter ：计数器的值
    // 步骤1：累加前7个字节，计数器，和固定值0x0B和0x0C

    uint16_t checksum = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6];
    checksum += (counter & 0x0F) + 0x0B + 0x0C;

    // 步骤2：根据给定公式计算校验和
    checksum = (((checksum >> 6) & 0x03) + (checksum >> 3) + checksum) & 0x07;

    // 返回结果（只使用低3位，符合Bit4-7存放Checksum的要求）
    return checksum;
}

int verify_checksum(uint8_t *data) {
    // 提取计数器（低4位）
    uint8_t counter = data[7] & 0x0F;

    // 提取校验和（高4位）
    uint8_t received_checksum = (data[7] >> 4) & 0x07;

    // 计算校验和
    uint8_t calculated_checksum = calculate_checksum(data, counter);

    // 比较接收到的校验和与计算的校验和
    return (received_checksum == calculated_checksum);
}

uint16_t get_engine_speed(uint8_t *data) {
    // 整合data[1]和data[2]为一个16位的整数
    uint16_t speed = data[1] | (data[2] << 8);

    // 返回整合后的转速值
    return speed;
}

uint16_t get_rudder_angle(uint8_t *data) {
    // 整合data[1]和data[2]为一个16位的整数
    uint16_t speed = data[3] | (data[4] << 8);

    // 返回整合后的转速值
    return speed;
}

int get_engine_speed1(uint8_t *data) {
    // 整合data[1]和data[2]为一个16位的整数
    uint16_t speed = data[3] | (data[4] << 8);
    speed=speed/8;
    // 返回整合后的转速值
    int speed_rpm;
    speed_rpm=(int)speed;
    return speed_rpm;
}

int get_water_temp(uint8_t *data) {
    // 整合data[1]和data[2]为一个16位的整数
    uint16_t water = data[0] + 40;
    // 返回整合后的转速值
    int waterT;
    waterT=(int)water;
    return waterT;
}

float get_oil_temp(uint8_t *data) {
    // 整合data[1]和data[2]为一个16位的整数
    uint16_t oil = data[2] | (data[3] << 8);
    // 返回整合后的转速值
    float oil1;
    oil1=(float)oil;
    oil1=oil1*0.03125-273;
    return oil1;
}

int get_oil_pressure(uint8_t *data) {
    // 整合data[1]和data[2]为一个16位的整数
    uint16_t oilp_ = data[3];
    // 返回整合后的转速值
    int oilp;
    oilp=(int)oilp_*4;
    return oilp;
}



int can1_send_msgs(int rpm_onoff,int value,int rudder)         //CAN1发送数据  value:rpm
{
    //control rpm
    //100ms 周期
    int ret;
    uint8_t checksum;
    uint8_t counter = 0x06;
    //value=value*8;
    switch (rpm_onoff) {
        case 1:
            //engine on
            memset(&txmsg[0],0,sizeof(txmsg[0]));
            txmsg[0].ID = 0x18F0010B;
            txmsg[0].Data[0] = 0x01;
            txmsg[0].Data[1] = 0xFF;
            txmsg[0].Data[2] = 0xFF;
            txmsg[0].Data[3] = 0xFF;
            txmsg[0].Data[4] = 0xFF;
            txmsg[0].Data[5] = 0xFF;
            txmsg[0].Data[6] = 0xFF;
            checksum = calculate_checksum(txmsg[0].Data, counter);
            txmsg[0].Data[7] =  (counter & 0x0F) | (checksum << 4);;   //校验shecksum
            txmsg[0].DataLen = 8;
            txmsg[0].ExternFlag = 1;  //拓展帧 29位帧id
            ret = CAN_Transmit(dev,cpot0,txmsg,1,100);
            break;
        case 2:
            //engine off
            memset(&txmsg[0],0,sizeof(txmsg[0]));
            txmsg[0].ID = 0x18F0010B;
            txmsg[0].Data[0] = 0x04;
            txmsg[0].Data[1] = 0xFF;
            txmsg[0].Data[2] = 0xFF;
            txmsg[0].Data[3] = 0xFF;
            txmsg[0].Data[4] = 0xFF;
            txmsg[0].Data[5] = 0xFF;
            txmsg[0].Data[6] = 0xFF;
            checksum = calculate_checksum(txmsg[0].Data, counter);
            txmsg[0].Data[7] =  (counter & 0x0F) | (checksum << 4);;   //校验shecksum
            txmsg[0].DataLen = 8;
            txmsg[0].ExternFlag = 1;  //拓展帧 29位帧id
            ret = CAN_Transmit(dev,cpot0,txmsg,1,100);
            break;
        case 3:
            //send rpm
            memset(&txmsg[0],0,sizeof(txmsg[0]));
            txmsg[0].ID = 0x0C00000B;
            txmsg[0].Data[0] = 0x01;
            txmsg[0].Data[1] = (uint8_t)(value&0xFF);   //转速低字节
            txmsg[0].Data[2] = (uint8_t)((value>>8)&0xFF);   //转速高字节  0x1F40 =8000*0.125=1000rpm
            txmsg[0].Data[3] = (uint8_t)(rudder&0xFF);   //目标舵角
            txmsg[0].Data[4] = (uint8_t)((rudder>>8)&0xFF);
            txmsg[0].Data[5] = 0xFF;
            txmsg[0].Data[6] = 0xFF;
            checksum = calculate_checksum(txmsg[0].Data, counter);
            txmsg[0].Data[7] =  (counter & 0x0F) | (checksum << 4);  //校验shecksum
            txmsg[0].DataLen = 8;
            txmsg[0].ExternFlag = 1;  //拓展帧 29位帧id
            ret = CAN_Transmit(dev,cpot0,txmsg,1,100);
            txmsg[0].ID = 0x0C00001B;
            ret = CAN_Transmit(dev,cpot1,txmsg,1,100);
            break;
        default:
            printf("Unknown engine order\n");
        break;
    }
    printf("CAN_Transmit items: %d\r\n",ret);
    return ret;
}

int can2_send_msgs(int rpm_onoff,int value,int rudder)         //CAN1发送数据  value:rpm
{
    //control rpm
    //100ms 周期
    int ret;
    uint8_t checksum;
    uint8_t counter = 0x06;
    value=value*8;
    switch (rpm_onoff) {
        case 1:
            //engine on
            memset(&txmsg[0],0,sizeof(txmsg[0]));
            txmsg[0].ID = 0x18F0010B;
            txmsg[0].Data[0] = 0x01;
            txmsg[0].Data[1] = 0xFF;
            txmsg[0].Data[2] = 0xFF;
            txmsg[0].Data[3] = 0xFF;
            txmsg[0].Data[4] = 0xFF;
            txmsg[0].Data[5] = 0xFF;
            txmsg[0].Data[6] = 0xFF;
            checksum = calculate_checksum(txmsg[0].Data, counter);
            txmsg[0].Data[7] =  (counter & 0x0F) | (checksum << 4);  //校验shecksum
            txmsg[0].DataLen = 8;
            txmsg[0].ExternFlag = 1;  //拓展帧 29位帧id
            ret = CAN_Transmit(dev,cpot1,txmsg,1,100);
            break;
        case 2:
            //engine off
            memset(&txmsg[0],0,sizeof(txmsg[0]));
            txmsg[0].ID = 0x18F0010B;
            txmsg[0].Data[0] = 0x04;
            txmsg[0].Data[1] = 0xFF;
            txmsg[0].Data[2] = 0xFF;
            txmsg[0].Data[3] = 0xFF;
            txmsg[0].Data[4] = 0xFF;
            txmsg[0].Data[5] = 0xFF;
            txmsg[0].Data[6] = 0xFF;
            checksum = calculate_checksum(txmsg[0].Data, counter);
            txmsg[0].Data[7] =  (counter & 0x0F) | (checksum << 4);   //校验shecksum
            txmsg[0].DataLen = 8;
            txmsg[0].ExternFlag = 1;  //拓展帧 29位帧id
            ret = CAN_Transmit(dev,cpot1,txmsg,1,100);
            break;
        case 3:
            //send rpm
            memset(&txmsg[0],0,sizeof(txmsg[0]));
            txmsg[0].ID = 0x0C00000B;
            txmsg[0].Data[0] = 0x01;
            txmsg[0].Data[1] = (uint8_t)(value&0xFF);   //转速低字节  缩放比0.125rpm/bit
            txmsg[0].Data[2] = (uint8_t)((value>>8)&0xFF);   //转速高字节  0x1F40 =8000*0.125=1000rpm
            txmsg[0].Data[3] = (uint8_t)(rudder&0xFF);    //目标扭矩 比例 1nm/bit
            txmsg[0].Data[4] = (uint8_t)((rudder>>8)&0xFF);
            txmsg[0].Data[5] = 0xFF;
            txmsg[0].Data[6] = 0xFF;
            checksum = calculate_checksum(txmsg[0].Data, counter);
            txmsg[0].Data[7] =  (counter & 0x0F) | (checksum << 4);  //校验shecksum
            txmsg[0].DataLen = 8;
            txmsg[0].ExternFlag = 1;  //拓展帧 29位帧id
            ret = CAN_Transmit(dev,cpot1,txmsg,1,100);
            break;
        default:
            printf("Unknown engine order\n");
        break;
    }
    printf("CAN_Transmit items2: %d\r\n",ret);
    return ret;
}

int can2_receive_msgs(void)         //CAN2接收数据
{
    int ret,i;
    memset(&rxmsg[0],0,sizeof(rxmsg[0]));
    ret = CAN_Receive(dev,cpot1,rxmsg,100,100);
    if(ret>0)
    {
        //printf("get can messages ID:%X\n",rxmsg[0].ID);
        switch (rxmsg[0].ID) {
            case 0x18F0010B:
                if(rxmsg[0].Data[0]==0x01)
                {
                    printf("engine ON\n");

                }
                if(rxmsg[0].Data[0]==0x04)
                {
                    printf("engine OFF\n");
                }
                printf("engine onoff get\n");
                break;
            case 0x0C00000B:
                uint16_t rpm;
                uint16_t rudder;
                rpm=get_engine_speed(rxmsg[0].Data);
                rudder=get_rudder_angle(rxmsg[0].Data);
                printf("rpm control get:%d\n",rpm);
                printf("rudder control get:%d\n",rudder);
                break;
            case 0x18FEF200:
                //water temprature//oil tempratrue
                int water_temp;
                float oil_temp;
                water_temp=get_water_temp(rxmsg[0].Data);
                oil_temp=get_oil_temp(rxmsg[0].Data);
                //printf("Temprature: water %d and oil %f \n",water_temp,oil_temp);
                break;
            case 0x0CF00400:
                //current rpm

                int rpm_=0;
                rpm_= get_engine_speed1(rxmsg[0].Data);
                //printf("rpm get:%d\n",rpm_);
                pthread_mutex_lock(&mutex_db);
                db->current_rpm=rpm_;
                //get_engine_speed1(rxmsg[0].Data);
                pthread_mutex_unlock(&mutex_db);
                break;
            case 0x18FECE00:
                //oil pressure
                int oiltemp;
                oiltemp=get_oil_pressure(rxmsg[0].Data);
                //printf("Pressure oil: %d \n",oiltemp);
                break;
            default:
                //printf("Unknown engine order\n");
                break;
        }
        //printf("CAN_Receive items: %d\r\n",ret);

        //printf("Receive msg: id = %x ,data: ",rxmsg[0].ID);
        /*
        for(i = 0 ; i < rxmsg[0].DataLen; i++)
        {
            printf("%02x ",(unsigned char)rxmsg[0].Data[i]);
        }
        printf("\r\n");
        */
    }

    return ret;
}

int init_can(void)
{
    int devs,ret;
    Can_Config cancfg;
    devs = CAN_ScanDevice();                 //扫描CAN设备
    printf("CAN_ScanDevice = %d\r\n",devs);
    if(devs <= 0) return -1;

    CAN_Reset(dev,cpot0 );
    CAN_Reset(dev,cpot1 );
    ret = CAN_OpenDevice(dev,cpot0);        //打开通道0(CAN1)
    printf("CAN_OpenDevice0 = %d\r\n",ret);
    if(ret != 0) return -1;
    printf("CAN_OpenDevice0 succeed!!\r\n");
    ret = CAN_OpenDevice(dev,cpot1);        //打开通道1(CAN2)
    printf("CAN_OpenDevice1 = %d\r\n",ret);
    if(ret != 0) return -1;
    printf("CAN_OpenDevice1 succeed!!\r\n");
    cancfg.model = 0;
    cancfg.configs = 0;
    cancfg.baudrate = 250000;  //设置波特率250k(250*1000)
    cancfg.configs |= 0x0001;  //接通内部匹配电阻
    cancfg.configs |= 0x0002;  //开启离线唤醒模式
    cancfg.configs |= 0x0004;  //开启自动重传

    ret = CAN_Init(dev,0,&cancfg);         //初始化CAN1
    printf("CAN_Init0 = %d\r\n",ret);
    if(ret != 0) return -1;
    printf("CAN_Init0 succeed!!\r\n");
    CAN_SetFilter(dev,cpot0,0,0,0,0,1);    //设置接收所有数据

    ret = CAN_Init(dev,cpot1,&cancfg);     //初始化CAN2
    printf("CAN_Init1 = %d\r\n",ret);
    if(ret != 0) return -1;
    printf("CAN_Init1 succeed!!\r\n");
    CAN_SetFilter(dev,cpot1,0,0,0,0,1);    //设置接收所有数据
    if(ret != 0) return -1;

}
void* test_can(void *arg)
{

    while(1)
    {
        can2_receive_msgs();                   //CAN2接收数据

    }

    //can1_send_msgs();                      //CAN1发送数据
    //can2_receive_msgs();                   //CAN2接收数据

    CAN_CloseDevice(dev,cpot0);            //关闭CAN1
    CAN_CloseDevice(dev,cpot1);            //关闭CAN2

    pthread_exit(NULL);
}




