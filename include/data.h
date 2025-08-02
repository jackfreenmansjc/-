#ifndef __DATA_H
#define __DATA_H
typedef  unsigned char u8;
typedef unsigned int u32;
//定义网络参数 端口号为6666  当前为服务器模式
#define MAXCONN 5
#define ERRORCODE -1
#define BUFFSIZE 1024
#define PORT 6666
#define IPLENGTH 25
#define MAX_LOS_POINTS 15
#define ERROR_PLC_FRONT_L 0x00000001
#define ERROR_PLC_FRONT_R 0x00000010
#define ERROR_PLC_BACK_L 0x00000100
#define ERROR_PLC_BACK_R 0x00001000
#define ERROR_PLC_RUDDER_L 0x00010000
#define ERROR_PLC_RUDDER_R 0x00100000
#define ERROR_PLC_GATE_L 0x01000000
#define ERROR_PLC_GATE_R 0x10000000
#define ERROR_PLC_ENGINE 0x00000001
#define HEARTBEAT_INTERVAL 5    // 心跳间隔，单位秒
#define CONNECTION_TIMEOUT 5   // 连接超时时间，单位秒
#define CRITICAL_SPEED 26 //临界速度
#define TAIL_ANG 4.0   //尾部滑板角度
#define MAX_BUFFER_SIZE 256  //存储GPS数据的字符串长度
#define DAISU 1000 //怠速转速
#define SAFE_BOARD 22//安全滑板角度

#define uint8_t u_int8_t
#define uint16_t u_int16_t
#define uint32_t u_int32_t

#define QUEUE_SIZE 5
//define data base
typedef enum{
	STANDBY_MODE,
    FAILURE_MODE,
    MANUAL_MODE,
    PID_MODE,
    LOS_MODE,
    Z_MODE,
    TURN_MODE,
    ZHIDONG_MODE
}MODE;

// 事件结构体
typedef struct {
    MODE mode;   //控制模式切换事件
    int emergency;  //紧急状态码事件
    int manual_control;  //进程遥控器介入事件
} Event;

typedef enum{
    NOTGOOD,
    GOOD
}GOODorNOT;

typedef enum{
    ON,
    OFF
}ONOFF;

typedef enum{

	STANDBY_,	//待机
	RUNNING_,  	//运行中
	ERROR_,		//故障
	OFF_		//离线
}PLC;

typedef enum{
    HAND,
    AUTO
}Hand_Auto;

//

typedef struct {
    float data[QUEUE_SIZE];  // 存储数据的数组
    int head;                 // 队列头部索引
    int size;                 // 当前数据数量
} Queue;
// 环形缓冲区结构体
typedef struct {
    float buffer[1024];
    int index;
    int count;
} CircularBuffer;

//向左回转还是向右回转
typedef enum{
    LEFT,
    RIGHT
}LEFTorRIGHT;

//control控制信号来自上位机的控制数据
typedef struct bc{
    MODE mode;
    ONOFF engine;   //全局发动机启停代码
    Hand_Auto HA;   //手动控制油门 还是 航速自动控制
    GOODorNOT PID_self_test;
    float Tar_yaw;                                 //目标首向角
	float Tar_speed;                               //感知域目标车速
	//rudder PID
	float kp_r;
	float ki_r;
	float kd_r;
	float umin_r;
	float umax_r;
	float error_max_r;
	float output_min_r;
	float output_max_r;
	//engine PID
	float kp_e;
	float ki_e;
	float kd_e;
	float umin_e;
	float umax_e;
	float error_max_e;
	float output_min_e;
	float output_max_e;
	int Tar_rpm;
    float Tar_slider;
	int rpmorspeed;  //1rpm,2speed
	//1.2MANUAL MODE
	GOODorNOT manualOK;
	int rpm;
	float rudder_left;
	float rudder_right;
	float front_board;
	float tail_board;
    int acc_point;   //km/h,放大了十倍
    int dec_point;   //km/h，放大了十倍
	ONOFF gate_left;
	ONOFF gate_right;
    //1.3LOS MODE
    double GPS_X[15];
    double GPS_Y[15];
    int num_points;
    ONOFF LOS_ONOFF;
	//1.3 Z MODE
	float rudder_turn;
	int rudder_turn_speed;
	float yaw_delta;
	ONOFF rudder_left_first;
    ONOFF start_Z;
	//1.4Turn MODE
	float t_delta;   //操舵角
	int t_turn_speed;	//转舵速度
	LEFTorRIGHT turn_LR;  //逆时针还是顺势针回转
	ONOFF back_to_straight;  //舵角回直
    ONOFF start_T;
	//1.5standby mode
	int standby;//standby mode
}Boatcontrol;
//来自GPS的数据
typedef struct gps{
    float yaw;
    float speed;
}BoatGPS;
//发送到PLC的输出数据
typedef struct output{
    int rpm;
    ONOFF onoff;
    float rudder_left;
    float rudder_right;
    float front_board;
    float tail_board;
    ONOFF gate_left;
    ONOFF gate_right;
}Boatoutput;
//基本数据表
typedef struct Data_base
{
	//self PARAMETERS
	GOODorNOT SYSTEM_self_test;
	GOODorNOT PLC_self_test;   //plc正常
	GOODorNOT IPC_self_test;   //上位机正常
	GOODorNOT GYRO_self_test;  //陀螺仪正常
	GOODorNOT GPS_self_test;   //gps正常
	GOODorNOT PID_self_test;   //PID参数正常
	//input
	//1.control data
	MODE mode;		//五种控制模式
	ONOFF engine;   //全局发动机启停代码
	Hand_Auto HA;   //手动控制油门 还是 航速自动控制
	//1.1PID MODE
	float Tar_yaw;
	float Tar_speed;
	//rudder PID
	float kp_r;
	float ki_r;
	float kd_r;
	float umin_r;
	float umax_r;
	float error_max_r;
	float output_min_r;
	float output_max_r;
	//engine PID
	float kp_e;
	float ki_e;
	float kd_e;
	float umin_e;
	float umax_e;
	float error_max_e;
	float output_min_e;
	float output_max_e;
	int Tar_rpm;
	float Tar_slider;
	int rpmorspeed;  //1rpm,2speed
	//1.2MANUAL MODE
	GOODorNOT manualOK;
	int rpm;
	float rudder_left;
	float rudder_right;
	float front_board;
	float tail_board;
    int acc_point;   //km/h,放大了十倍
    int dec_point;   //km/h，放大了十倍
	ONOFF gate_left;
	ONOFF gate_right;
	//1.3LOS MODE
	double GPS_X[MAX_LOS_POINTS];
	double GPS_Y[MAX_LOS_POINTS];
	int num_points;
	ONOFF LOS_ONOFF;
	//1.4 Z MODE
	float z_rudder_turn;//舵角
	int z_rudder_trun_speed;//转舵速度
	float z_yaw_delta;//超越角度
	ONOFF z_rudder_left_first;//先左转还是右转 on:left first; off:right;
    ONOFF start_Z;
	//1.4 TURN MODE
	float t_delta;   //操舵角
	int t_turn_speed;	//转舵速度
	LEFTorRIGHT turn_LR;  //逆时针还是顺势针回转
	ONOFF back_to_straight;  //舵角回直
    ONOFF start_T;
	//2 sensors
	//2.1sensor: GPS data
	float gps_time;
	double latitude;
	double longitude;
	float gps_speed;
	//2.2sensor: GRO data
	float yaw;
	//2.3 PLC status
	PLC plc_front_left;
	PLC plc_front_right;
	PLC plc_back_left;
	PLC plc_back_right;
	PLC plc_rudder_left;
	PLC plc_rudder_right;
	PLC plc_gate_left;
	PLC plc_gate_right;
	PLC plc_engine;
	//output
	//3.1 PLC control
	int out_rpm;
	ONOFF out_engine;   //控制发动机启动停止
	float out_rudder_left;
	float out_rudder_right;
	float out_front_board;
	float out_tail_board;
	ONOFF out_gate_left;
	ONOFF out_gate_right;
	int out_turn_speed;  //控制转舵速度
	//3.2 call back to control
	float current_speed;  //当前航速
	float current_yaw;		//当前航向
	double current_latitude;  //当前纬度
	double current_longitude;  //当前经度
	float front_left;			//首滑板左
	float front_right;		//首滑板右
	float back_left;		//尾滑板左
	float back_right;		//尾滑板右
	float callrudder_left;		//left rudder
	float callrudder_right; 	//right rudder
	int   current_rpm;		//当前发动机转速
	int   error_sign1;		//故障代码
	int   error_sign2;		//故障代码
	//1.5 standby mode
	int standby;
}DB1;



#endif
