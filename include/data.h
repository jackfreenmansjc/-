#ifndef __DATA_H
#define __DATA_H
typedef  unsigned char u8;
typedef unsigned int u32;
//����������� �˿ں�Ϊ6666  ��ǰΪ������ģʽ
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
#define HEARTBEAT_INTERVAL 5    // �����������λ��
#define CONNECTION_TIMEOUT 5   // ���ӳ�ʱʱ�䣬��λ��
#define CRITICAL_SPEED 26 //�ٽ��ٶ�
#define TAIL_ANG 4.0   //β������Ƕ�
#define MAX_BUFFER_SIZE 256  //�洢GPS���ݵ��ַ�������
#define DAISU 1000 //����ת��
#define SAFE_BOARD 22//��ȫ����Ƕ�

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

// �¼��ṹ��
typedef struct {
    MODE mode;   //����ģʽ�л��¼�
    int emergency;  //����״̬���¼�
    int manual_control;  //����ң���������¼�
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

	STANDBY_,	//����
	RUNNING_,  	//������
	ERROR_,		//����
	OFF_		//����
}PLC;

typedef enum{
    HAND,
    AUTO
}Hand_Auto;

//

typedef struct {
    float data[QUEUE_SIZE];  // �洢���ݵ�����
    int head;                 // ����ͷ������
    int size;                 // ��ǰ��������
} Queue;
// ���λ������ṹ��
typedef struct {
    float buffer[1024];
    int index;
    int count;
} CircularBuffer;

//�����ת�������һ�ת
typedef enum{
    LEFT,
    RIGHT
}LEFTorRIGHT;

//control�����ź�������λ���Ŀ�������
typedef struct bc{
    MODE mode;
    ONOFF engine;   //ȫ�ַ�������ͣ����
    Hand_Auto HA;   //�ֶ��������� ���� �����Զ�����
    GOODorNOT PID_self_test;
    float Tar_yaw;                                 //Ŀ�������
	float Tar_speed;                               //��֪��Ŀ�공��
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
    int acc_point;   //km/h,�Ŵ���ʮ��
    int dec_point;   //km/h���Ŵ���ʮ��
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
	float t_delta;   //�ٶ��
	int t_turn_speed;	//ת���ٶ�
	LEFTorRIGHT turn_LR;  //��ʱ�뻹��˳�����ת
	ONOFF back_to_straight;  //��ǻ�ֱ
    ONOFF start_T;
	//1.5standby mode
	int standby;//standby mode
}Boatcontrol;
//����GPS������
typedef struct gps{
    float yaw;
    float speed;
}BoatGPS;
//���͵�PLC���������
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
//�������ݱ�
typedef struct Data_base
{
	//self PARAMETERS
	GOODorNOT SYSTEM_self_test;
	GOODorNOT PLC_self_test;   //plc����
	GOODorNOT IPC_self_test;   //��λ������
	GOODorNOT GYRO_self_test;  //����������
	GOODorNOT GPS_self_test;   //gps����
	GOODorNOT PID_self_test;   //PID��������
	//input
	//1.control data
	MODE mode;		//���ֿ���ģʽ
	ONOFF engine;   //ȫ�ַ�������ͣ����
	Hand_Auto HA;   //�ֶ��������� ���� �����Զ�����
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
    int acc_point;   //km/h,�Ŵ���ʮ��
    int dec_point;   //km/h���Ŵ���ʮ��
	ONOFF gate_left;
	ONOFF gate_right;
	//1.3LOS MODE
	double GPS_X[MAX_LOS_POINTS];
	double GPS_Y[MAX_LOS_POINTS];
	int num_points;
	ONOFF LOS_ONOFF;
	//1.4 Z MODE
	float z_rudder_turn;//���
	int z_rudder_trun_speed;//ת���ٶ�
	float z_yaw_delta;//��Խ�Ƕ�
	ONOFF z_rudder_left_first;//����ת������ת on:left first; off:right;
    ONOFF start_Z;
	//1.4 TURN MODE
	float t_delta;   //�ٶ��
	int t_turn_speed;	//ת���ٶ�
	LEFTorRIGHT turn_LR;  //��ʱ�뻹��˳�����ת
	ONOFF back_to_straight;  //��ǻ�ֱ
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
	ONOFF out_engine;   //���Ʒ���������ֹͣ
	float out_rudder_left;
	float out_rudder_right;
	float out_front_board;
	float out_tail_board;
	ONOFF out_gate_left;
	ONOFF out_gate_right;
	int out_turn_speed;  //����ת���ٶ�
	//3.2 call back to control
	float current_speed;  //��ǰ����
	float current_yaw;		//��ǰ����
	double current_latitude;  //��ǰγ��
	double current_longitude;  //��ǰ����
	float front_left;			//�׻�����
	float front_right;		//�׻�����
	float back_left;		//β������
	float back_right;		//β������
	float callrudder_left;		//left rudder
	float callrudder_right; 	//right rudder
	int   current_rpm;		//��ǰ������ת��
	int   error_sign1;		//���ϴ���
	int   error_sign2;		//���ϴ���
	//1.5 standby mode
	int standby;
}DB1;



#endif
