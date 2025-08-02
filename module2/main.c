#define CONFIGURE_INIT
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/time.h>

#include <math.h>
#include <fcntl.h>
#include <termios.h>

#include <signal.h>
#include <sys/ioctl.h>
#include <sys/stat.h>  // 用于获取文件大小
#include "boat_control.h"
#include "data.h"
#include "cantest.h"

#define SERIAL_PORT "/dev/ttyUSB0" // 根据你的实际串口设备路径修改
#define BAUDRATE B38400
#define BUFFER_SIZE 256

#define MAX_FILE_SIZE 100 * 1024 * 1024  // 100MB 的字节数
#define CHECK_INTERVAL 1000  // 每1000次写入后检查文件大小

#include "cantest.h"
#include "canbus.h"


#define IBUS_LENGTH 0x20
#define IBUS_COMMAND40 0x40
#define IBUS_MAX_CHANNLES 14  // 最大通道数
const char* serial_ports[] = { "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2" };
DB1 *db=NULL;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t mutex_db = PTHREAD_MUTEX_INITIALIZER;


//z mode onoff
int zmode;

int turnmode;

int zhidongmode;

int pidcad ;

int summ;

// 校验变量
uint16_t checksum_cal, checksum_ibus;
uint16_t channel[IBUS_MAX_CHANNLES];
uint16_t channel_buffer[IBUS_MAX_CHANNLES];  // 存储解析后的通道数据
int user_channels = IBUS_MAX_CHANNLES;  // 使用的通道数
ONOFF channel_onoff=OFF;//手动操作启停开关

//zmode
int step_zmode=0;   //定义Zmode进行的阶段
int left_right=-1;  //定义小船的转动朝向 -1朝左， 1朝右边


int get5msg;
float PreSpeed_suisu=0;//记录上一次的水速仪速度
float PrePreSpeed_suisu=0;//记录上上时刻的水速仪速度
Queue speed_msg; //保存拿到的速度数据
Queue yaw_msg;  //保存拿到的航向角数据
float speed_median;  //计算得到的速度平均中值
float yaw_median;   //计算得到的航向角平均中值

//
//舵角控制算法PID参数
float SetSpeedc=0;            //定义设定值
float ActualSpeedc=0;        //定义实际值
float errc=0;                //定义偏差值
float err_lastc=0;            //定义上一个偏差值
float Kpc,Kic,Kdc;            //定义比例、积分、微分系数

float voltagec=0;            //定义电压值（控制执行器的变量）
float integralc=0;            //定义积分值
float umaxc=400;                //积分量不能超过这个值，抗积分饱和
float uminc=-400;                //负积分量不能小于这个值，抗积分饱和
float error_maxc=60;        //误差饱和量，用于积分分离过程，如果单次计算的误差超过这个值，那么这个值不计入积分项，防止数据抖动造成的积分过大
float output_maxc=60;        //输出限幅  执行机构输出限幅
float output_minc=-60;       //输出限幅
int indexc=0;
float Biasc=0;
//发动机控制参数
float SetSpeede=0;            //定义设定值
float ActualSpeede=0;        //定义实际值
float erre=0;                //定义偏差值
float err_laste=0;            //定义上一个偏差值
float Kpe,Kie,Kde;            //定义比例、积分、微分系数

float voltagee=900;            //定义电压值（控制执行器的变量）怠速油门量
float integrale=0;            //定义积分值
float umaxe=20;                //积分量不能超过这个值，抗积分饱和
float umine=20;                //负积分量不能小于这个值，抗积分饱和
float error_maxe=3;        //误差饱和量，用于积分分离过程，如果单次计算的误差超过这个值，那么这个值不计入积分项，防止数据抖动造成的积分过大
float output_maxe=1000;        //输出限幅  执行机构输出限幅
float output_mine=0;       //输出限幅
int indexe=0;
float Biase=0;

float temp_engine=0;
float SetSpeede_pre=0; //保存上一时刻的期望速度，当期望速度发生变化时，积分内容需要清零，防止稳态误差。
float danci_youmen=1000; //单次油门控制量 每秒最多能调节多少转速rpm
float pre_Tar_yaw=0; //用于保存之前的目标偏转角
float pre_Tar_speed=0; // 用于保存之前的目标速度

float acc_point=20;  //km/h 滑板自适应控制默认值20km/h触发
float dec_point=20;  //km/h


ONOFF new_data_IPC=OFF;   //地面站数据收到指示
ONOFF new_data_GPS=OFF;   //gps数据收到指示
ONOFF new_data_IMU=OFF;   //惯导数据收到指示
ONOFF new_data_HAND=OFF;  //进程遥控器收到指示

ONOFF PLC1_output=OFF;  //PLC1执行指示
ONOFF PLC2_output=OFF;  //PLC2执行指示
MODE control_mode = STANDBY_MODE;
Event control_event = {0, 0, 0}; // 设置 mode = 0 (standby 模式)
//signal for shut down socketthread

int server_socket = -1; // Placeholder for the server socket
int thread_count = 0;
pthread_t threads[10]; // Placeholder for threads

//
//中值计算平均当前速度和当前曲率
// 初始化队列
void init_queue(Queue *q) {
    q->head = 0;
    q->size = 0;
}
// 向队列中添加新数据
void enqueue(Queue *q, float value) {
    q->data[q->head] = value;
    q->head = (q->head + 1) % QUEUE_SIZE;
    if (q->size < QUEUE_SIZE) {
        q->size++;
    }
}

// 复制队列中的数据到一个数组中
void copy_queue_data(Queue *q, float dest[]) {
    int index = (q->head - q->size + QUEUE_SIZE) % QUEUE_SIZE;
    for (int i = 0; i < q->size; i++) {
        dest[i] = q->data[(index + i) % QUEUE_SIZE];
    }
}
// 比较函数，用于qsort排序
int compare(const void *a, const void *b) {
    if (*(double*)a > *(double*)b) return 1;
    else if (*(float*)a < *(float*)b) return -1;
    else return 0;
}
// 中值滤波函数
float median_filter(Queue *q) {
    float sorted_data[QUEUE_SIZE];
    copy_queue_data(q, sorted_data);

    // 使用qsort对数据进行排序
    qsort(sorted_data, q->size, sizeof(float), compare);

    // 计算中值
    if (q->size % 2 == 0) {
        return (sorted_data[q->size/2 - 1] + sorted_data[q->size/2]) / 2.0;
    } else {
        return sorted_data[q->size/2];
    }
}



void handle_signal(int signal) {
    printf("Caught signal %d, exiting and cleaning up resources...\n", signal);

    // Close socket if open
    if (server_socket != -1) {
        close(server_socket);
        printf("Closed server socket.\n");
    }

    // Cancel and join threads to ensure proper cleanup
    for (int i = 0; i < thread_count; i++) {
        if (pthread_cancel(threads[i]) == 0) {
            printf("Cancelled thread %d\n", i);
        }
        pthread_join(threads[i], NULL);
        printf("Joined thread %d\n", i);
    }


    CAN_CloseDevice(0,0);            //关闭CAN1
    CAN_CloseDevice(0,1);            //关闭CAN2

    // Final exit
    exit(0);
}

//calman filter realize speed calculate

// 卡尔曼滤波器状态
typedef struct {
    double velocity_estimate; // 速度估计值
    double error_covariance;  // 误差协方差
    double process_variance;  // 过程噪声
    double measurement_variance; // 测量噪声（GPS的不确定性）
} KalmanFilter;

double imu_acceleration = 0.1; // 假设IMU给出的加速度
double gps_velocity = 5.0; // GPS提供的速度（低频数据）
double prev_time ;// get_timestamp(); // 获取初始时间戳
double curr_time, delta_t;
//calman
KalmanFilter kf;
// 获取当前的高精度时间戳（秒为单位）
double get_timestamp() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;  // 将秒和纳秒转换为浮点数的秒
}
// 初始化卡尔曼滤波器
void kalman_init(KalmanFilter *kf, double process_variance, double measurement_variance) {
    kf->velocity_estimate = 0.0; // 初始速度
    kf->error_covariance = 1.0;  // 初始误差协方差
    //error_covariance：如果对初始速度估计不确定，可以设置为较大值（如 100），否则可以设为 1.0。
    kf->process_variance = process_variance; // 过程噪声协方差
    kf->measurement_variance = measurement_variance;  // 测量噪声协方差
}

// 预测步骤：根据IMU数据更新速度
void kalman_predict(KalmanFilter *kf, double acceleration, double delta_t) {
    // 用IMU的加速度预测速度
    acceleration = floor(acceleration * 100) / 100;
    kf->velocity_estimate += acceleration * delta_t;

    // 误差协方差更新
    kf->error_covariance += kf->process_variance;
}

// 校正步骤：根据GPS数据校正速度
void kalman_update(KalmanFilter *kf, double gps_velocity) {
    // 卡尔曼增益计算
    double kalman_gain = kf->error_covariance / (kf->error_covariance + kf->measurement_variance);

    // 用GPS数据更新速度估计
    kf->velocity_estimate += kalman_gain * (gps_velocity - kf->velocity_estimate);

    // 更新误差协方差
    kf->error_covariance *= (1 - kalman_gain);
    pthread_mutex_lock(&mutex);
    db->current_speed=(float)kf->velocity_estimate*3.6;
    pthread_mutex_unlock(&mutex);
}

//******************************************
int count_connect = 0;
struct pthread_socket
{
	int socket_d;
	pthread_t thrd;
	char ip[IPLENGTH];
};
//save ip address and socket
typedef struct {
    int client_socket;
    char ip_address[IPLENGTH];
} ClientInfo;

static void *thread_send(void *arg)
{
	ClientInfo clientinfo= *(ClientInfo *)arg;
	//int sd = *(int *) arg;
	int sd = clientinfo.client_socket;
	char ipaddr[IPLENGTH];
	memset(ipaddr, 0, IPLENGTH);
	strcpy(ipaddr,clientinfo.ip_address);

	while (1)
	{

		if(strncmp(ipaddr,"192.168.1.205",20)==0)
		{
			//printf("send to IPC: 192.168.1.205 !\n");
			if (send(sd, (char*)db, sizeof(DB1), 0) == -1)
			{
				printf("send error:%s \n", strerror(errno));
				break;
			}
			usleep(100000);
		}

		if(strncmp(ipaddr,"192.168.1.80",20)==0)
		{
		    if(PLC1_output==ON)
            {
                char buf[20];
                pthread_mutex_lock(&mutex_db);

                buf[0]=0xff;
                buf[1]=0xff;
                intToChars(db->out_engine, &buf[2]);
                float r_left;
                float r_right;
                r_left=db->out_rudder_left;
                r_left=r_left/0.6;

                r_right=db->out_rudder_right;
                r_right=r_right/0.6;
                //floatToChars(db->out_rudder_left,&buf[4]);
                //floatToChars(db->out_rudder_right,&buf[6]);
                floatToChars(r_left,&buf[4]);
                floatToChars(r_right,&buf[6]);
                floatToChars(db->out_front_board,&buf[8]);
                floatToChars(db->out_tail_board,&buf[10]);
                //db->gate_left = ON;
                //db->gate_right =ON;
                intToChars(db->out_gate_left,&buf[12]);
                intToChars(db->out_gate_right,&buf[14]);
                intToChars(db->out_rpm,&buf[16]);
                buf[18]=0xff;
                buf[19]=0xff;
                PLC1_output=OFF;

                pthread_mutex_unlock(&mutex_db);


                printf("left gate:%d \t",db->out_gate_left);
                printf("right gate:%d \t",db->out_gate_right);
                printf("left rudder:%f \t",db->out_rudder_left);
                printf("right rudder:%f \n",db->out_rudder_right);
                if (send(sd, (char*)&buf, sizeof(buf), 0) == -1)
                {
                    printf("send error:%s \n", strerror(errno));
                    break;
                }
                if(db->mode==Z_MODE && db->start_Z==ON)
                {
                    if(step_zmode==1)
                    {
                        step_zmode=2;
                    }else if (step_zmode==2)
                    {
                        left_right=left_right*(-1);
                    }
                }

            }
            else
            {
                //usleep(120000);
            }

		}


        if(strncmp(ipaddr,"192.168.1.88",20)==0)
		{
		    // to master WEI
		    if(PLC2_output==ON)
            {
                float temp_tail=0;
                float temp_front=0;
                char buf[20];
                pthread_mutex_lock(&mutex_db);

                buf[0]=0xff;
                buf[1]=0xaa;
                intToChars(db->out_engine, &buf[2]);
                floatToChars(db->out_rudder_left,&buf[4]);
                floatToChars(db->out_rudder_right,&buf[6]);
                temp_front=22-db->out_front_board;
                floatToChars(temp_front,&buf[8]);
                temp_tail=-db->out_tail_board+10;
                if(temp_tail<0)
                {
                    temp_tail=0;
                }
                floatToChars(temp_tail,&buf[10]);
                //db->gate_left = ON;
                //db->gate_right =ON;
                intToChars(db->out_gate_left,&buf[12]);
                intToChars(db->out_gate_right,&buf[14]);
                intToChars(db->out_rpm,&buf[16]);
                buf[18]=0xff;
                buf[19]=0xbb;
                PLC2_output=OFF;
                pthread_mutex_unlock(&mutex_db);

                printf("front slide: %f",db->out_front_board);
                printf("back slide: %f",db->out_tail_board);
                if (send(sd, (char*)&buf, sizeof(buf), 0) == -1)
                {
                    printf("send error:%s \n", strerror(errno));
                    break;
                }

            }
            else
            {
                //usleep(120000);
            }
		   //usleep(9000);
        }
	}
	return NULL;
}


static void* thread_recv(void *arg)
{
    static int speedtime_get=0;
	unsigned char buf[BUFFSIZE];
	struct pthread_socket *pt = (struct pthread_socket *) arg;
	int sd = pt->socket_d;
	pthread_t thrd = pt->thrd;
	char ipaddr[IPLENGTH];
	memset(ipaddr, 0, IPLENGTH);
	strcpy(ipaddr,pt->ip);

	while (1)
	{
		int suma;
		memset(buf, 0, sizeof(buf));
		int rv = recv(sd, (char*)&buf, sizeof(buf),0);
		if (rv < 0)
		{
			printf("recv error:%c \n", strerror(errno));
			break;
		}
		if (rv == 0)
		{
			break;
		}
		if(strncmp(ipaddr,"192.168.1.200",20)==0)
		{
		    if(speedtime_get==0)
            {
                speedtime_get=1;
                prev_time = get_timestamp();
            }
			//data from GRO  gaundao
			double imu_acceleration=data_process_GRO(buf,rv);
			// 计算实际的采样时间 delta_t
			curr_time = get_timestamp();
            delta_t = curr_time - prev_time;
            // 预测步骤：使用IMU加速度更新速度
            kalman_predict(&kf, imu_acceleration, delta_t);
            // 更新 prev_time 为当前时间
            //printf("delta time: %lf current speed: %lf m/s\n",delta_t,kf.velocity_estimate*9.794);
            prev_time = curr_time;
            new_data_IMU=ON;
			continue;
		}
		else if(strncmp(ipaddr,"192.168.1.203",20)==0)
		{
			//data from gps
			continue;
		}
		else if(strncmp(ipaddr,"192.168.1.80",20)==0)
		{
			//data from PLC
			data_process_PLC(buf,rv);
			continue;
		}
        else if(strncmp(ipaddr,"192.168.1.88",20)==0)
		{
			//data from PLC
			data_process_PLC_wei(buf,rv);
			continue;
		}

		else if(strncmp(ipaddr,"192.168.1.205",20)==0)
		{
			//data from IPC ground control
			data_process_IPC(buf);
            //printf("Longitude: %f\n", lon);
            pthread_mutex_lock(&mutex);
            control_event.mode=db->mode;
            new_data_IPC=ON;
            pthread_mutex_unlock(&mutex);

			continue;
		}
		else
		{
			//
            continue;
		}


	}
	pthread_cancel(thrd);
	pthread_mutex_lock(&mutex);
	count_connect--;
	pthread_mutex_unlock(&mutex);
	close(sd);
	return NULL;
}

static int create_listen(int port)
{

    	int listen_st;
    	struct sockaddr_in sockaddr; //¶¨ÒåIPµØÖ·½á¹¹
    	struct linger sl;  // 定义linger结构
    	int on = 1;
    	listen_st = socket(AF_INET, SOCK_STREAM, 0); //³õÊ¼»¯socket

    	if (listen_st == -1)
    	{
        	printf("socket create error:%s \n", strerror(errno));
        	return ERRORCODE;
    	}
    	server_socket = listen_st; // 将 socket 记录到全局变量中
    	if (setsockopt(listen_st, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) == -1) //ÉèÖÃipµØÖ·¿ÉÖØÓÃ
    	{
        	printf("setsockopt error:%s \n", strerror(errno));
        	return ERRORCODE;
    	}

    	    // 设置SO_LINGER选项，确保socket关闭时立即释放
    sl.l_onoff = 1;   // 启用linger
    sl.l_linger = 0;  // 立即关闭
    if (setsockopt(listen_st, SOL_SOCKET, SO_LINGER, &sl, sizeof(sl)) < 0)
    {
        printf("setsockopt SO_LINGER error: %s\n", strerror(errno));
        return ERRORCODE;
    }
    	sockaddr.sin_port = htons(port);
    	sockaddr.sin_family = AF_INET;
    	sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);


    	/*
    	if (bind(listen_st, (struct sockaddr *) &sockaddr, sizeof(sockaddr)) == -1)
    	{
       		printf("bind error:%s \n", strerror(errno));
        	return ERRORCODE;
    	}
    	*/
    	for (int attempt = 0; attempt < 5; ++attempt) {
            if (bind(listen_st, (struct sockaddr *) &sockaddr, sizeof(sockaddr)) == 0) {
                break;
            }
            printf("bind attempt %d failed: %s\n", attempt + 1, strerror(errno));
            sleep(1);  // 等待1秒钟再重试
        }

    	if (listen(listen_st, 5) == -1) //     ·þÎñ¶Ë¿ªÊ¼¼àÌý
    	{
        	printf("listen error:%s \n", strerror(errno));
        	return ERRORCODE;
    	}
    	return listen_st;
}

ClientInfo accept_socket(int listen_st)
{
    	int accept_st;
    	struct sockaddr_in accept_sockaddr;
    	socklen_t addrlen = sizeof(accept_sockaddr);
    	memset(&accept_sockaddr, 0, addrlen);
    	ClientInfo clientinfo;
    	accept_st = accept(listen_st, (struct sockaddr*)&accept_sockaddr,&addrlen);

    	if (accept_st == -1)
    	{
        	printf("accept error:%s \n", strerror(errno));
        	clientinfo.client_socket=ERRORCODE;
        	return clientinfo;
    	}
    	printf("accpet ip:%s \n", inet_ntoa(accept_sockaddr.sin_addr));
    	clientinfo.client_socket = accept_st;
    	memset(clientinfo.ip_address,0,sizeof(IPLENGTH));
    	strcpy(clientinfo.ip_address,inet_ntoa(accept_sockaddr.sin_addr));
    	return clientinfo;
}
int GP_INIT(int port)
{
    	int listen_st = create_listen(port);    //´´½¨¼àÌýsocket
    	pthread_t send_thrd, recv_thrd;
    	struct pthread_socket ps;
    	int accept_st;
    	ClientInfo clientinfo;
    	if (listen_st == -1)
    	{
        	return ERRORCODE;
    	}
    	printf("server start \n");
    	while (1)
    	{

    		clientinfo = accept_socket(listen_st); //»ñÈ¡Á¬½ÓµÄµÄsocket
    		accept_st=clientinfo.client_socket;
        	if (accept_st == -1)
        	{
            		return ERRORCODE;
        	}
        	if (count_connect >= MAXCONN)
        	{
            		printf("connect have already be full! \n");
            		close(accept_st);
            		continue;
        	}
        	pthread_mutex_lock(&mutex);
        	count_connect++;
        	pthread_mutex_unlock(&mutex);

        	if (pthread_create(&send_thrd, NULL, thread_send, &clientinfo) != 0) //´´½¨·¢ËÍÐÅÏ¢Ïß³Ì
        	{
            		printf("create thread error:%s \n", strerror(errno));
            		break;

        	}
        	threads[thread_count++] = send_thrd; // 记录创建的线程
        	pthread_detach(send_thrd);        //ÉèÖÃÏß³Ì¿É·ÖÀëÐÔ£¬ÕâÑùµÄ»°Ö÷Ïß³Ì¾Í²»ÓÃjoin
        	ps.socket_d = accept_st;
        	ps.thrd = send_thrd;
        	memset(ps.ip,0,sizeof(IPLENGTH));
			strcpy(ps.ip,clientinfo.ip_address);
        	if (pthread_create(&recv_thrd, NULL, thread_recv, &ps) != 0)//´´½¨½ÓÊÕÐÅÏ¢Ïß³Ì
        	{
            		printf("create thread error:%s \n", strerror(errno));
            		break;
        	}
        	threads[thread_count++] = recv_thrd; // 记录创建的线程
        	pthread_detach(recv_thrd); //ÉèÖÃÏß³ÌÎª¿É·ÖÀë£¬ÕâÑùµÄ»°£¬¾Í²»ÓÃpthread_join
    	}
    shutdown(accept_st,SHUT_RDWR);
    close(accept_st);
    close(listen_st);
    return 0;
}

//将gps坐标改成标准十进制格式
// 将度分格式转换为十进制度格式的函数
double convert_to_decimal(char coord[19]) {
    int degrees = 0;
    double minutes = 0.0;

    // 使用 sscanf 从字符数组中解析出度、分和方向
    sscanf(coord, "%2d%lf", &degrees, &minutes);

    // 将分转换为度
    double decimal = degrees + (minutes / 60.0);


    return decimal;
}

double convert_to_decimal_lon(char coord[19]) {
    int degrees = 0;
    double minutes = 0.0;

    // 使用 sscanf 从字符数组中解析出度、分和方向
    sscanf(coord, "%3d%lf", &degrees, &minutes);

    // 将分转换为度
    double decimal = degrees + (minutes / 60.0);


    return decimal;
}
//gps message process

// 提取经纬度函数
void extract_lat_lon(const char *gps_data) {
    char buffer[BUFFER_SIZE];
    char *token;
    const char *delimiter = "\n";
    char *line;

    // 使用 strtok_r 进行线程安全的分割
    char *saveptr1;
    char *saveptr2;

    // 复制数据以避免修改原始数据
    strncpy(buffer, gps_data, BUFFER_SIZE);
    buffer[BUFFER_SIZE - 1] = '\0'; // 确保字符串结尾

    // 处理每一行
    line = strtok_r(buffer, delimiter, &saveptr1);
    while (line != NULL) {
    if (strncmp(line, "$GNRMC", 6) == 0) {
        // 处理 GNRMC 语句
        token = strtok_r(line, ",", &saveptr2);
            if (token != NULL) {
                token = strtok_r(NULL, ",", &saveptr2); // 时间
                token = strtok_r(NULL, ",", &saveptr2); // 状态
                if (token != NULL && strcmp(token, "A") == 0) { // 检查状态是否为 A
                    token = strtok_r(NULL, ",", &saveptr2); // 纬度
                    if (token != NULL && strlen(token) > 0) {
                        char latitude[20];
                        strncpy(latitude, token, sizeof(latitude));
                        latitude[19] = '\0'; // 确保字符串结尾
                        double lat = convert_to_decimal(latitude);
                        pthread_mutex_lock(&mutex);
                        db->current_latitude = lat;
                        pthread_mutex_unlock(&mutex);
                    }
                    token = strtok_r(NULL, ",", &saveptr2); // 纬度方向
                    token = strtok_r(NULL, ",", &saveptr2); // 经度
                    if (token != NULL && strlen(token) > 0) {
                        char longitude[20];
                        strncpy(longitude, token, sizeof(longitude));
                        longitude[19] = '\0'; // 确保字符串结尾
                        double lon = convert_to_decimal_lon(longitude);
                        pthread_mutex_lock(&mutex);
                        db->current_longitude = lon;
                        pthread_mutex_unlock(&mutex);
                    }
                    token = strtok_r(NULL, ",", &saveptr2); // 经度方向
                    token = strtok_r(NULL, ",", &saveptr2); // GPS速度 (节，knots)
                    if (token != NULL && strlen(token) > 0) {
                        double speed_knots = atof(token);  // 将字符串转换为浮点数
                        gps_velocity = speed_knots * 0.514444;  // 将速度从节 (knots) 转换为米/秒 (m/s)

                        // 更新卡尔曼滤波器中的速度
                        kalman_update(&kf, gps_velocity);
                    }
                } else {
                    //printf("Invalid or no data in $GNRMC sentence.\n");
                }
            }
        }else if (strncmp(line, "$GNGGA", 6) == 0) {
            // 处理 GNGGA 语句
            token = strtok_r(line, ",", &saveptr2);
            if (token != NULL) {
                token = strtok_r(NULL, ",", &saveptr2); // 时间
                token = strtok_r(NULL, ",", &saveptr2); // 纬度
                if (token != NULL && strlen(token) > 0) {
                    char latitude[20];
                    strncpy(latitude, token, sizeof(latitude));
                    latitude[19] = '\0'; // 确保字符串结尾
                    double lat;
                    lat=convert_to_decimal(latitude);
                    pthread_mutex_lock(&mutex);
                    db->current_latitude=lat;
                    pthread_mutex_unlock(&mutex);
                    //printf("Latitude: %f\t", lat);
                }
                token = strtok_r(NULL, ",", &saveptr2); // 纬度方向
                token = strtok_r(NULL, ",", &saveptr2); // 经度
                if (token != NULL && strlen(token) > 0) {
                    char longitude[20];
                    strncpy(longitude, token, sizeof(longitude));
                    longitude[19] = '\0'; // 确保字符串结尾
                    double lon;
                    lon=convert_to_decimal_lon(longitude);
                    pthread_mutex_lock(&mutex);
                    db->current_longitude=lon;
                    pthread_mutex_unlock(&mutex);
                    //printf("Longitude: %f\n", lon);
                }
            } else {
                printf("Invalid or incomplete $GNGGA sentence.\n");
            }
        }
        line = strtok_r(NULL, delimiter, &saveptr1);
    }
}
// 自动检测串口设备
int detect_serial_port() {
    int fd;
    for (int i = 0; i < sizeof(serial_ports) / sizeof(serial_ports[0]); i++) {
        fd = open(serial_ports[i], O_RDWR | O_NOCTTY );
        if (fd != -1) {
            printf("Opened serial port: %s\n", serial_ports[i]);
            return fd;  // 返回打开的文件描述符
        } else {
            perror("Failed to open serial port");
        }
    }
    return -1;  // 没有找到可用的串口设备
}

// 检查设备状态的函数，返回1表示设备在线，返回0表示设备已断开
int check_device_status(int fd) {
    int status;
    if (ioctl(fd, TIOCMGET, &status) < 0) {
        perror("Error getting serial port status");
        return 0;
    }

    // 检查DTR (Data Terminal Ready) 或者 DSR (Data Set Ready) 状态
    if (status & TIOCM_DSR) {
        return 1; // 设备仍然在线
    } else {
        printf("GPS device disconnected.\n");
        return 0; // 设备断开
    }
}

//serial get message GPS
void *serial_read_thread(void *arg) {
    int fd;
    struct termios options;
    char buffer[256];
    int n;

    // 检测并打开可用串口
    fd = detect_serial_port();
    //fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("No available serial port found.\n");
        pthread_exit(NULL);
    }

    // 清空串口缓存，避免读取到旧数据
    tcflush(fd, TCIOFLUSH);
    // 获取当前串口设置
    if (tcgetattr(fd, &options) < 0) {
        perror("tcgetattr");
        close(fd);
        pthread_exit(NULL);
    }

    // 设置波特率
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);

    // 设置数据位、停止位、校验位等
    options.c_cflag |= (CLOCAL | CREAD); // 启动接收器，不要管调制解调器状态线
    options.c_cflag &= ~PARENB;          // 无校验位
    options.c_cflag &= ~CSTOPB;          // 1个停止位
    options.c_cflag &= ~CSIZE;           // 清除数据位设置
    options.c_cflag |= CS8;              // 8个数据位
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控制
    options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 禁用规范模式、回显、回显擦除和信号处理


    // 设置读取超时
    options.c_cc[VMIN] = 0;    // 最小读取字节数
    options.c_cc[VTIME] = 1;   // 读取超时设置为 100ms（VTIME 单位是 100ms）
    // 设置新的串口设置
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        perror("tcsetattr");
        close(fd);
        pthread_exit(NULL);
    }

    // 读取数据
    while (1) {
        memset(buffer, 0, sizeof(buffer));
        n = read(fd, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0'; // 确保字符串结束
            //printf("Received: %s\n", buffer);
                // 打开字符串数据作为文件流
            // 提取经纬度
            extract_lat_lon(buffer);


        }else if (n == 0) {
            // 没有接收到数据，可能是设备断开，检查GPS设备状态
            printf("No data received, checking device...\n");
        if (!check_device_status(fd)) {
                // 设备断开，尝试重新打开设备
                close(fd);
                while (1) {
                    printf("Trying to reconnect GPS device...\n");
                    usleep(500000); // 等待500ms再重新尝试
                    fd = detect_serial_port();
                    if (fd != -1) {
                        printf("GPS device reconnected.\n");
                        break;
                    }
                }

                // 重新配置串口
                tcflush(fd, TCIOFLUSH);
                if (tcgetattr(fd, &options) < 0) {
                    perror("tcgetattr");
                    close(fd);
                    pthread_exit(NULL);
                }
                cfsetispeed(&options, BAUDRATE);
                cfsetospeed(&options, BAUDRATE);
                options.c_cflag |= (CLOCAL | CREAD);
                options.c_cflag &= ~PARENB;
                options.c_cflag &= ~CSTOPB;
                options.c_cflag &= ~CSIZE;
                options.c_cflag |= CS8;
                options.c_iflag &= ~(IXON | IXOFF | IXANY);
                options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
                options.c_cc[VMIN] = 0;
                options.c_cc[VTIME] = 1;
                if (tcsetattr(fd, TCSANOW, &options) < 0) {
                    perror("tcsetattr");
                    close(fd);
                    pthread_exit(NULL);
                }
            }
            // 这里可以增加对设备状态的检测，如果设备关闭可以退出循环
            // 例如通过检查串口状态或尝试重新打开设备
        } else {
            // 读取错误
            perror("Read error");
            break;
        }
        usleep(10000); // 10ms 延时
    }

    // 关闭串口
    close(fd);
    pthread_exit(NULL);
}

//采集数据
// 获取文件大小的函数
long get_file_size(const char* filename) {
    struct stat st;
    if (stat(filename, &st) == 0) {
        return st.st_size;  // 返回文件大小（字节）
    } else {
        perror("获取文件大小失败");
        return -1;
    }
}
// 采集数据并写入文件的线程函数
void* collect_data_and_write_to_file(void* arg) {
    const char* filename = (const char*)arg;
    struct timespec sleep_time;
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = 150 * 1000000;  // 150ms = 150,000,000 纳秒

    int write_count = 0;  // 记录写入次数
    time_t now;
    struct timeval tv;
    struct tm* local;
    int data=0;
    DB1 db_record;//日志记录副本
    while (1) {
        // 打开文件，追加模式写入数据
        FILE* file = fopen(filename, "a");
        if (file == NULL) {
            perror("文件打开失败");
            pthread_exit(NULL);
        }

        // 模拟生成数据
        data++;
        if(db==NULL)
        {
            printf("failed to malloc memomry/n");
        }
        else{
            pthread_mutex_lock(&mutex_db);
        	db_record=*db;
        	pthread_mutex_unlock(&mutex_db);
        }
        // 获取当前时间
        //time(&now);
        //local = localtime(&now);
        gettimeofday(&tv,NULL);
        local=localtime(&tv.tv_sec);
        // 将数据和时间写入文件
        fprintf(file, "时间: %02d:%02d:%02d.%03ld 数据: %d\n",
                local->tm_hour, local->tm_min, local->tm_sec,tv.tv_usec/1000, data);

        // 强制将缓冲区中的数据写入文件
        int counter=0;
        fflush(file);
        fprintf(file, "Data %d:HEART BEAT \n", counter);
        fflush(file); // Ensure data is written to file immediately
        fprintf(file, " OP CONTROL: %d \t", db_record.mode);
        fflush(file);
        fprintf(file, " OP MODE: %d \n", db_record.mode);
        fflush(file);
        fprintf(file, " Current Curve: %f \t", db_record.current_yaw);
        fflush(file);
        fprintf(file, " Target Curve: %f \n", db_record.Tar_yaw);
        fflush(file);
        fprintf(file, " Current Speed: %f \t", db_record.current_speed);
        fflush(file);
        fprintf(file, " Target Speed: %f \n", db_record.Tar_speed);
        fflush(file);
        fprintf(file, " Current Head Down: %f \t", db_record.front_board);
        fflush(file);
        fprintf(file, " Current Tail: %f \t", db_record.tail_board);
        fflush(file);
        fprintf(file, " Current Left Rudder: %f \t", db_record.rudder_left);
        fflush(file);
        fprintf(file, " Current Right Rudder: %f \t", db_record.rudder_right);
        fflush(file);
        fprintf(file, " Current Left Door: %d \t", db_record.gate_left);
        fflush(file);
        fprintf(file, " Current Right Door: %d \t", db_record.gate_right);
        fflush(file);
        fprintf(file, " Current Heart beat in: 1\n");//;
        fflush(file);
        fprintf(file, " ********Emergency: 0 \n");
        fflush(file);
        fprintf(file, " OUTPUT Target engine: %d \t", db_record.out_rpm);
        fflush(file);
        fprintf(file, " OUTPUT Head Down: %f \t", db_record.out_front_board);
        fflush(file);
        fprintf(file, " OUTPUT Tail: %f \t", db_record.out_tail_board);
        fflush(file);
        fprintf(file, " OUTPUT Left Rudder: %f \t", db_record.out_rudder_left);
        fflush(file);
        fprintf(file, " OUTPUT Right Rudder: %f \t", db_record.out_rudder_right);
        fflush(file);
        fprintf(file, " OUTPUT Left Door: %d \t", db_record.out_gate_left);
        fflush(file);
        fprintf(file, " OUTPUT Right Door: %d \t", db_record.out_gate_right);
        fflush(file);
        fprintf(file, " OUTPUT Heart_Beat_out: 1 \t");//, db.Heart_Beat_out);
        fflush(file);
        fprintf(file, " \n");
        fflush(file);

        fclose(file);  // 关闭文件

        // 每1000次写入后检查文件大小
        if (++write_count >= CHECK_INTERVAL) {
            write_count = 0;  // 重置计数器
            long file_size = get_file_size(filename);
            if (file_size == -1) {
                pthread_exit(NULL);  // 如果获取文件大小失败，退出线程
            }

            // 如果文件大小超过100MB，清空文件
            if (file_size >= MAX_FILE_SIZE) {
                file = fopen(filename, "w");  // "w" 模式清空文件
                if (file == NULL) {
                    perror("文件清空失败");
                    pthread_exit(NULL);  // 如果文件打开失败，退出线程
                }
                fclose(file);
                printf("文件大小超过100MB，已清空文件内容。\n");
            }
        }

        // 休眠150ms
        nanosleep(&sleep_time, NULL);
    }

    pthread_exit(NULL);  // 线程结束
}
// Setup signal handlers to catch termination signals

// 每个状态下的处理函数
void standby_handler() {
    DB1 db_record_s;//记录副本
    //printf("Entering Standby Mode: Initializing systems...\n");
    //保存数据副本
    pthread_mutex_lock(&mutex_db);
    db_record_s=*db;
    pthread_mutex_unlock(&mutex_db);
    if(new_data_IPC==ON)
    {
        //如果收到了来自地面站的数据
        pthread_mutex_lock(&mutex_db);
        new_data_IPC=OFF;
        db->PID_self_test=NOTGOOD;
        pthread_mutex_unlock(&mutex_db);
        switch (db_record_s.standby) {
            case 0:
                //do nothing
                printf("Entering Standby Mode: 0 nothing\n");
                break;
            case 1:
                printf("Entering Standby Mode: 1 \n");
                //if(can_is_ok)
                //if(engine_is_OFF)
                //sendtoengine(start_engine_order);
                //else(//do nothing)//engine already on
                pthread_mutex_lock(&mutex_db);
                db->out_front_board=22;
                db->out_rudder_left=0;
                db->out_rudder_right=0;
                db->out_gate_left=ON; //左右水门打开
                db->out_gate_right=ON;
                PLC1_output=ON;
                PLC2_output=ON;
                pthread_mutex_unlock(&mutex_db);
                break;
            case 2:
                can1_send_msgs(1,0,0);
                printf("Entering Standby Mode: 2 \n");
                pthread_mutex_lock(&mutex_db);
                db->out_rpm=DAISU;
                db->out_front_board=22;
                db->out_rudder_left=0;
                db->out_rudder_right=0;
                db->out_gate_left=ON; //左右水门打开
                db->out_gate_right=ON;
                PLC1_output=ON;
                PLC2_output=ON;
                db->out_engine=ON;
                pthread_mutex_unlock(&mutex_db);
                break;
            case 3:
                printf("Entering Standby Mode: 3 \n");
                pthread_mutex_lock(&mutex_db);
                db_record_s.out_rpm=DAISU;
                pthread_mutex_unlock(&mutex_db);
                //sendtoengine(daisu_order);
                can1_send_msgs(3,900,0);
                break;
            case 4:
                //打开两侧边舵并怠速
                //sendtoengine(daisu_order);
                can1_send_msgs(3,900,0);
                printf("Entering Standby Mode: 4 \n");
                pthread_mutex_lock(&mutex_db);
                db->out_rpm=DAISU;
                db->out_front_board=22;
                db->out_rudder_left=0;
                db->out_rudder_right=0;
                PLC1_output=ON;
                PLC2_output=ON;
                pthread_mutex_unlock(&mutex_db);
                break;
            case 5:
                //紧急停车，关闭油门
                //sendtoengine(shutdown_engine_order);
                can1_send_msgs(2,0,0);
                printf("Entering Standby Mode: 5 \n");
                pthread_mutex_lock(&mutex_db);
                db->out_rpm=0;
                db->out_front_board=22;
                db->out_rudder_left=0;
                db->out_rudder_right=0;
                PLC1_output=ON;
                PLC2_output=ON;
                db->out_engine=OFF;
                pthread_mutex_unlock(&mutex_db);
                break;
            default:
                printf("Unknown manual mode\n");
                break;
        }
    }

}

void failure_handler() {
    printf("Entering Failure Mode: Shutting down systems...\n");
}

void manual_handler() {
    //printf("Entering Manual Mode: Enabling manual control...\n");
    DB1 db_record_s;//记录副本
    //printf("Entering Standby Mode: Initializing systems...\n");
    //保存数据副本
    pthread_mutex_lock(&mutex_db);
    db_record_s=*db;
    pthread_mutex_unlock(&mutex_db);
    if(new_data_IPC==ON)
    {
        //如果收到了来自地面站的数据
        if(db_record_s.engine==OFF)
        {
            //如果指令要求关闭发动机
            can1_send_msgs(2,900,0);
        }else{
            //发送发动机转速指令
            can1_send_msgs(3,db_record_s.rpm,0);
        }
        pthread_mutex_lock(&mutex_db);
        new_data_IPC=OFF;
        db->PID_self_test=NOTGOOD;
        db->out_rpm=db_record_s.rpm;
        db->out_front_board=db_record_s.front_board;
        db->out_tail_board=db_record_s.tail_board;
        db->out_rudder_left=db_record_s.rudder_left;
        db->out_rudder_right=db_record_s.rudder_right;
        db->out_gate_left=db_record_s.gate_left; //左右水门打开
        db->out_gate_right=db_record_s.gate_right;
        PLC1_output=ON;
        PLC2_output=ON;
        pthread_mutex_unlock(&mutex_db);

        printf("Entering Manual Mode \n");

    }
}
int checkForAnomaly(float a, float b, float c)
{
	float sum;
	sum=a+b+c;
	sum=sum/3;
	if (abs(a - sum) > 5)
	{
		 //printf("Anomaly detected!\n");
		return 0;

	}else
	{
		return 1;
	}
}

void PID_realize_engine(DB1 db_reco)
{
	//航速控制
	//pthread_mutex_lock(&mutex);
	SetSpeede=db_reco.Tar_speed;
	ActualSpeede=speed_median;
	Biase = SetSpeede-ActualSpeede;
	erre=Biase;//误差量
	if(SetSpeede==pre_Tar_speed){


	}
	else
	{
		pre_Tar_speed=SetSpeede;
		integrale=0;
	}
	if(integrale > umaxe)  //灰色底色表示抗积分饱和的实现,积分限幅,防止积分量过高
	{
	   if(abs(erre)>error_maxe)      //蓝色标注为积分分离过程    ,在误差很大的时候，关闭积分参数，防止累计误差过大
		{
		   //积分分离，当误差很大的时候，将误差加入积分量容易引入很大的控制偏差。
			indexe=0;
		}else{
				indexe=1;
				if(erre<0)
				{
					//抗饱和积分，当积分量超过了设置的上限，那么只允许负的误差被累积到积分量上，也就是防止积分量无限增大
					integrale+=erre;
				}
		}
	}else if(integrale < umine){
		if(abs(erre)>error_maxe)      //积分分离过程
		{
			indexe=0;
		}else{
				indexe=1;
				if(erre>0)
				{
					integrale+=erre;
				}
		}
	}else{
		//如果积分量没有超过上下限，此时只要误差值不要太大，就可以把误差加入到积分中去
		if(abs(erre)>error_maxe)                    //积分分离过程
		{
			indexe=0;
		}else{
				indexe=1;
				integrale+=erre;
		}
	}
	//kp+梯形积分+kd
	temp_engine=Kpe*erre+indexe*Kie*integrale/2+Kde*(erre-err_laste);//执行机构的输出
	//单次转速量输出不能超过xxxrpm

	if(temp_engine>danci_youmen)
	{
		voltagee+=danci_youmen;
	}else if(temp_engine<-danci_youmen)
	{
		voltagee-=danci_youmen;
	}
	else
	{
		voltagee+=temp_engine;
	}
	//printf("kp  kd  err : %f  %f  %f \n",Kpe,Kde,erre);
	//voltagee=500+Kpe*erre+Kde*(erre-err_laste);//执行机构的输出
	//printf("voltagee:is %f \n",voltagee);
	//pid输出限制幅    防止执行机构的输出过高，超出机构的能力限制
	//
	if(voltagee > 3800)
	{
		voltagee = 3800;
	}
	else if(voltagee < 900)
	{
		voltagee = 900;
	}
	err_laste=erre;
	//printf("voltage: %f \n",voltagee);
	db_reco.Tar_rpm = (int)(voltagee);
	//执行到输出端
    can1_send_msgs(3,db_reco.Tar_rpm,0);

}

void set_pid_para(DB1 db_reco)
{
        //舵角控制算法PID参数
        Kpc=db_reco.kp_r;
        Kic=db_reco.ki_r;
        Kdc=db_reco.kd_r;
        umaxc=db_reco.umax_r;                //积分量不能超过这个值，抗积分饱和
        uminc=db_reco.umin_r;                //负积分量不能小于这个值，抗积分饱和
        error_maxc=db_reco.error_max_r;        //误差饱和量，用于积分分离过程，如果单次计算的误差超过这个值，那么这个值不计入积分项，防止数据抖动造成的积分过大
        output_maxc=db_reco.output_max_r;        //输出限幅  执行机构输出限幅
        output_minc=db_reco.output_min_r;       //输出限幅
        //发动机控制参数
        Kpe=db_reco.kp_e;
        Kie=db_reco.ki_e;
        Kde=db_reco.kd_e;
        umaxe=db_reco.umax_e;                //积分量不能超过这个值，抗积分饱和
        umine=db_reco.umin_e;                //负积分量不能小于这个值，抗积分饱和
        error_maxe=db_reco.error_max_e;        //误差饱和量，用于积分分离过程，如果单次计算的误差超过这个值，那么这个值不计入积分项，防止数据抖动造成的积分过大
        output_maxe=db_reco.output_max_e;        //输出限幅  执行机构输出限幅
        output_mine=db_reco.output_min_e;       //输出限幅
}

//舵航向角控制
//
void PID_realize_rudder(DB1 *db_reco)
{
	//pthread_mutex_lock(&mutex);
	//目标角度
	SetSpeedc=db_reco->Tar_yaw; //目标航向角
	//当前角度
	ActualSpeedc=yaw_median;//当前艏向角
	//偏差值
	Biasc = SetSpeedc-ActualSpeedc;
		//陀螺仪范围0~360，防止不合理的超限
	if(Biasc >= 180)
	{
		Biasc = -360 + Biasc;
	}
	else if (Biasc <= -180)
	{
		Biasc = 360 + Biasc;
	}
	if(SetSpeedc==pre_Tar_yaw){


	}
	else
	{
		pre_Tar_yaw=SetSpeedc;
		integralc=0;
	}
	errc=Biasc;//边界控制

    if(integralc > umaxc)  //灰色底色表示抗积分饱和的实现,积分限幅,防止积分量过高
    {
       if(abs(errc)>error_maxc)      //蓝色标注为积分分离过程    ,在误差很大的时候，关闭积分参数，防止累计误差过大
        {
           //积分分离，当误差很大的时候，将误差加入积分量容易引入很大的控制偏差。
            indexc=0;
        }else{
                indexc=1;
                if(errc<0)
                {
                    //抗饱和积分，当积分量超过了设置的上限，那么只允许负的误差被累积到积分量上，也就是防止积分量无限增大
                    integralc+=errc;
                }
        }
    }else if(integralc < uminc){
        if(abs(errc)>error_maxc)      //积分分离过程
        {
            indexc=0;
        }else{
                indexc=1;
                if(errc>0)
                {
                    integralc+=errc;
                }
        }
    }else{
        //如果积分量没有超过上下限，此时只要误差值不要太大，就可以把误差加入到积分中去
        if(abs(errc)>error_maxc)                    //积分分离过程
        {
            indexc=0;
        }else{
                indexc=1;
                integralc+=errc;
        }
    }
    if(pre_Tar_yaw != ActualSpeedc)
    {
        integralc = 0;
        pre_Tar_yaw = ActualSpeedc;
    }
    //kp+梯形积分+kd
    voltagec=Kpc*errc+indexc*Kic*integralc/2+Kdc*(errc-err_lastc);//执行机构的输出
    //pid输出限制幅    防止执行机构的输出过高，超出机构的能力限制
    //这里可以理解为最大舵角
    //机构限制幅度
    if(voltagec > output_maxc)
    {
        voltagec = output_maxc;
    }
    else if(voltagec < output_minc)
    {
        voltagec = output_minc;
    }
    err_lastc=errc;
    voltagec=round(voltagec*10)/10;
    if(voltagec >= 0)
    {
        db_reco->out_rudder_left=0;
        db_reco->out_rudder_right=(int)(voltagec);

    }else
    {
        db_reco->out_rudder_left=-(int)(voltagec);
        db_reco->out_rudder_right=0;
    }

	//模拟物理上转舵的效果
	//db->current_yaw+=0.2*voltage;
	db_reco->out_gate_left= ON ;//水门保持打开
	db_reco->out_gate_right = ON; //水门保持打开
	//pthread_mutex_unlock(&mutex);
}

void Slider_control(DB1 *db_reco)
{
    float tar=0;
    float cur=10;
    float accp=15;
    float decp=20;
    if(db_reco!=NULL)
    {
        tar=db_reco->Tar_speed;
        cur=speed_median;
        accp=((float)db_reco->acc_point)/10;
        decp=((float)db_reco->dec_point)/10;
    }
    if(accp<=decp)
    {
        //right logic
        if(tar>=decp)
        {
            if(cur<=accp)
            {
                db_reco->out_front_board=22;
                db_reco->out_tail_board=-4;
            }
            else if(cur>=decp)
            {
                db_reco->out_front_board=10;
                db_reco->out_tail_board=-4;
            }
            else
            {
                db_reco->out_front_board=16;
                db_reco->out_tail_board=-4;
            }
        }
        else if(tar<=accp)
        {

            db_reco->out_front_board=22;
            db_reco->out_tail_board=-4;


        }
        else
        {
            db_reco->out_front_board=20;
            db_reco->out_tail_board=-4;
        }

    }
    else{
        //wrong logic
        db_reco->out_front_board=22;
        db_reco->out_tail_board=-4;
    }
}
void pid_handler() {
    //printf("Entering PID Mode: Adjusting with PID controller...\n");
        //printf("Entering Manual Mode: Enabling manual control...\n");
    DB1 db_record_s;//记录副本
    //printf("Entering Standby Mode: Initializing systems...\n");
    //保存数据副本
    static int error_overtime=0;
    pthread_mutex_lock(&mutex_db);
    db_record_s=*db;
    pthread_mutex_unlock(&mutex_db);
    if(new_data_IPC==ON)
    {
        //如果收到了来自地面站的数据
        pthread_mutex_lock(&mutex_db);
        new_data_IPC=OFF;
        pthread_mutex_unlock(&mutex_db);
        set_pid_para(db_record_s);
    }
    if(db_record_s.PID_self_test==GOOD)
    {
        if(new_data_IMU==ON)
        {
            error_overtime=0;
            //数据处理，滤波平滑处理
            enqueue(&speed_msg, db_record_s.current_speed);
			enqueue(&yaw_msg, db_record_s.current_yaw);
			speed_median = median_filter(&speed_msg);
			yaw_median = median_filter(&yaw_msg);
            if(get5msg>=7)
            {
                get5msg=0;
                //如果pid参数已经写入，执行航速航向控制运算
                if(db_record_s.rpmorspeed==1 && db_record_s.HA==HAND)
                {
                    //rpm 控制
                    //执行到输出端
                    can1_send_msgs(3,db_record_s.Tar_rpm,0);
                    db_record_s.out_front_board=db_record_s.Tar_slider;
                    printf("front board:%f \n",db_record_s.out_front_board);
                }
                else if(db_record_s.rpmorspeed==2 && db_record_s.HA==AUTO)
                {
                    //航速控制
                    int check_suisu;
                    float current_suisu;
                    current_suisu=speed_median;
                    check_suisu=checkForAnomaly(current_suisu,PreSpeed_suisu,PrePreSpeed_suisu);
                    if(check_suisu)
                    {
                        //航速控制任务
                        PID_realize_engine(db_record_s);
                        //首尾滑板控制函数
                        Slider_control(&db_record_s);
                    }
                    PrePreSpeed_suisu=PreSpeed_suisu;
					PreSpeed_suisu=speed_median;

                }
                //航向控制函数
                PID_realize_rudder(&db_record_s);

                //执行到输出端
                //can1_send_msgs(3,db_record_s.rpm);
                pthread_mutex_lock(&mutex_db);
                db->out_rpm=db_record_s.out_rpm;
                db->out_front_board=db_record_s.out_front_board;
                db->out_tail_board=db_record_s.out_tail_board;
                db->out_rudder_left=db_record_s.out_rudder_left;
                db->out_rudder_right=db_record_s.out_rudder_right;
                db->out_gate_left=ON; //左右水门打开
                db->out_gate_right=ON;
                PLC1_output=ON;
                PLC2_output=ON;
                pthread_mutex_unlock(&mutex_db);

            }
            else
            {
                get5msg++;
            }
            pthread_mutex_lock(&mutex_db);
            new_data_IMU=OFF;
            pthread_mutex_unlock(&mutex_db);
        }
        else
        {
            //惯导数据超时处理
            error_overtime++;
            if(error_overtime>=100)
            {
                pthread_mutex_lock(&mutex_db);
                db->error_sign1=1;
                //control_event.emergency=1;
                pthread_mutex_unlock(&mutex_db);
                error_overtime=0;
            }

        }

    }
}

//标准的PID控制速度和航向的程序
void PID_speed_yaw(DB1 db_record_s){
    //当target_rpm不为0时，启动转速控制，反之启用航速控制
    static int error_overtime_pid=0;
    if(db_record_s.kp_r>0)
    {
        if(new_data_IMU==ON)
        {
            //数据处理，滤波平滑处理
            enqueue(&speed_msg, db_record_s.current_speed);
            enqueue(&yaw_msg, db_record_s.current_yaw);
            speed_median = median_filter(&speed_msg);
            yaw_median = median_filter(&yaw_msg);
            if(get5msg>=7)
            {
                get5msg=0;
                //如果pid参数已经写入，执行航速航向控制运算
                if(db_record_s.Tar_rpm>500)
                {
                    //rpm 控制
                    //执行到输出端
                    can1_send_msgs(3,db_record_s.Tar_rpm,0);
                    db_record_s.out_rpm=db_record_s.Tar_rpm;
                    db_record_s.out_front_board=22;
                    printf("front board:22 \n");
                }
                else
                {
                    //航速控制
                    int check_suisu;
                    float current_suisu;
                    current_suisu=speed_median;
                    check_suisu=checkForAnomaly(current_suisu,PreSpeed_suisu,PrePreSpeed_suisu);
                    if(check_suisu)
                    {
                        //航速控制任务
                        PID_realize_engine(db_record_s);
                        //首尾滑板控制函数
                        Slider_control(&db_record_s);
                    }
                    PrePreSpeed_suisu=PreSpeed_suisu;
                    PreSpeed_suisu=speed_median;

                }
                //航向控制函数
                PID_realize_rudder(&db_record_s);

                //执行到输出端
                //can1_send_msgs(3,db_record_s.rpm);
                pthread_mutex_lock(&mutex_db);
                db->out_rpm=db_record_s.out_rpm;
                db->out_front_board=db_record_s.out_front_board;
                db->out_tail_board=db_record_s.out_tail_board;
                db->out_rudder_left=db_record_s.out_rudder_left;
                db->out_rudder_right=db_record_s.out_rudder_right;
                db->out_gate_left=ON; //左右水门打开
                db->out_gate_right=ON;
                PLC1_output=ON;
                PLC2_output=ON;
                pthread_mutex_unlock(&mutex_db);

            }
            else
            {
                get5msg++;
            }
            pthread_mutex_lock(&mutex_db);
            new_data_IMU=OFF;
            pthread_mutex_unlock(&mutex_db);
        }
    }
    else{
        printf("wrong PID parameters\n");
    }


}


void los_handler() {
    //printf("Entering LOS Mode: Line-of-sight control...\n");
    DB1 db_record_s;//记录副本
    ONOFF los_onoff=OFF;
    //printf("Entering Standby Mode: Initializing systems...\n");
    //保存数据副本
    pthread_mutex_lock(&mutex_db);
    db_record_s=*db;
    pthread_mutex_unlock(&mutex_db);
    /*
    printf("\n\n");
    for(int m=0;m<15;m++)
    {
        printf("db_record lat:%lf  lon %lf \n",db_record_s.GPS_X[m],db_record_s.GPS_Y[m]);
    }
    */
    if(new_data_IPC==ON)
    {
        //printf("new ipc construction got \n");
        los_onoff=ON;
        pthread_mutex_lock(&mutex_db);
        new_data_IPC=OFF;
        pthread_mutex_unlock(&mutex_db);

    }
    if(db_record_s.LOS_ONOFF==ON)
    {
        float yaw_los=LOS_main(&db_record_s,los_onoff);
        los_onoff=OFF;
        //printf("target yaw for los: %f",yaw_los);
        pthread_mutex_lock(&mutex_db);
        db->Tar_yaw=yaw_los;
        db_record_s.Tar_yaw=yaw_los;
        pthread_mutex_unlock(&mutex_db);
        PID_speed_yaw(db_record_s);

    }


}

void z_mode_handler() {
    //printf("Entering Z Mode: Z control is active...\n");
    DB1 db_record_s;//记录副本
    //printf("Entering Standby Mode: Initializing systems...\n");
    //保存数据副本
    static int error_overtime=0;
    pthread_mutex_lock(&mutex_db);
    db_record_s=*db;
    pthread_mutex_unlock(&mutex_db);
    if(new_data_IPC==ON)
    {
        //如果收到了来自地面站的数据
        pthread_mutex_lock(&mutex_db);
        new_data_IPC=OFF;
        pthread_mutex_unlock(&mutex_db);
        set_pid_para(db_record_s);
        step_zmode=1;
    }
    if(db_record_s.start_Z==OFF)
    {
        if(1)//new_data_IMU==ON)
        {
            error_overtime=0;
            //数据处理，滤波平滑处理
            enqueue(&speed_msg, db_record_s.current_speed);
			enqueue(&yaw_msg, db_record_s.current_yaw);
			speed_median = median_filter(&speed_msg);
			yaw_median = median_filter(&yaw_msg);
            if(get5msg>=7)
            {
                get5msg=0;
                //如果pid参数已经写入，执行航速航向控制运算
                if(db_record_s.rpmorspeed==1)
                {
                    //rpm 控制
                    //执行到输出端
                    can1_send_msgs(3,db_record_s.Tar_rpm,0);
                    db_record_s.out_front_board=db_record_s.Tar_slider;
                    printf("front board:%f \n",db_record_s.out_front_board);
                }
                else if(db_record_s.rpmorspeed==2)
                {
                    //航速控制
                    int check_suisu;
                    float current_suisu;
                    current_suisu=speed_median;
                    check_suisu=checkForAnomaly(current_suisu,PreSpeed_suisu,PrePreSpeed_suisu);
                    if(check_suisu)
                    {
                        //航速控制任务
                        PID_realize_engine(db_record_s);
                        //首尾滑板控制函数
                        Slider_control(&db_record_s);
                    }
                    PrePreSpeed_suisu=PreSpeed_suisu;
					PreSpeed_suisu=speed_median;

                }
                //航向控制函数
                PID_realize_rudder(&db_record_s);

                //执行到输出端
                //can1_send_msgs(3,db_record_s.rpm);
                pthread_mutex_lock(&mutex_db);
                db->out_rpm=db_record_s.out_rpm;
                db->out_front_board=db_record_s.out_front_board;
                db->out_tail_board=db_record_s.out_tail_board;
                db->out_rudder_left=db_record_s.out_rudder_left;
                db->out_rudder_right=db_record_s.out_rudder_right;
                db->out_gate_left=ON; //左右水门打开
                db->out_gate_right=ON;
                PLC1_output=ON;
                PLC2_output=ON;
                pthread_mutex_unlock(&mutex_db);

            }
            else
            {
                get5msg++;
            }
            pthread_mutex_lock(&mutex_db);
            new_data_IMU=OFF;
            pthread_mutex_unlock(&mutex_db);
        }
        else
        {
            error_overtime++;
            if(error_overtime>=100)
            {
                pthread_mutex_lock(&mutex_db);
                db->error_sign1=1;
                control_event.emergency=1;
                pthread_mutex_unlock(&mutex_db);
                error_overtime=0;
            }

        }

    }
    else
    {
        //start turn
        switch(step_zmode)
        {
            case 1:
                static float taryaw;
                taryaw=db_record_s.current_yaw;
                if(db_record_s.z_rudder_left_first==ON)
                {
                    pthread_mutex_lock(&mutex_db);
                    db->out_rpm=db_record_s.out_rpm;
                    db->out_front_board=22;
                    db->out_tail_board=-4;
                    db->out_rudder_left=db_record_s.z_rudder_turn;
                    db->out_rudder_right=0;
                    db->out_gate_left=ON; //左右水门打开
                    db->out_gate_right=ON;
                    PLC1_output=ON;
                    PLC2_output=ON;
                    pthread_mutex_unlock(&mutex_db);
                    left_right=-1;
                }else
                {
                    pthread_mutex_lock(&mutex_db);
                    db->out_rpm=db_record_s.out_rpm;
                    db->out_front_board=22;
                    db->out_tail_board=-4;
                    db->out_rudder_left=0;
                    db->out_rudder_right=db_record_s.z_rudder_turn;
                    db->out_gate_left=ON; //左右水门打开
                    db->out_gate_right=ON;
                    PLC1_output=ON;
                    PLC2_output=ON;
                    pthread_mutex_unlock(&mutex_db);
                    left_right=1;
                }
                step_zmode=2;

                break;
            case 2:
                static int status_z=0;  //定义边界状态，-1：左边界<0;0:正常边界 1：右边界>360
                float boarder_left; //定义左边界
                float boarder_right; //定义右边界
                boarder_left=taryaw-db_record_s.z_yaw_delta;
                boarder_right=taryaw+db_record_s.z_yaw_delta;
                if(boarder_left<0)
                {
                    boarder_left+=360;
                    status_z=-1;
                }
                else if(boarder_right>360)
                {
                    boarder_right-=360;
                    status_z=1;
                }else
                {
                    status_z=0;

                }
                if(left_right==-1)
                {
                    //如果向左打舵
                    if(status_z>=0)
                    {
                        if(db_record_s.current_yaw<=boarder_left)
                        {
                            pthread_mutex_lock(&mutex_db);
                            db->out_rpm=db_record_s.out_rpm;
                            db->out_front_board=22;
                            db->out_tail_board=-4;
                            db->out_rudder_left=0;
                            db->out_rudder_right=db_record_s.z_rudder_turn;
                            db->out_gate_left=ON; //左右水门打开
                            db->out_gate_right=ON;
                            PLC1_output=ON;
                            PLC2_output=ON;
                            pthread_mutex_unlock(&mutex_db);
                        }
                    }else if (status_z==-1)
                    {
                        //左边界情况
                        if(db_record_s.current_yaw<=boarder_left && db_record_s.current_yaw>180)
                        {
                            pthread_mutex_lock(&mutex_db);
                            db->out_rpm=db_record_s.out_rpm;
                            db->out_front_board=22;
                            db->out_tail_board=-4;
                            db->out_rudder_left=0;
                            db->out_rudder_right=db_record_s.z_rudder_turn;
                            db->out_gate_left=ON; //左右水门打开
                            db->out_gate_right=ON;
                            PLC1_output=ON;
                            PLC2_output=ON;
                            pthread_mutex_unlock(&mutex_db);
                        }
                    }

                }
                else
                {
                    //如果向右打舵
                    if(status_z<=0)
                    {
                        if(db_record_s.current_yaw>=boarder_right)
                        {
                            pthread_mutex_lock(&mutex_db);
                            db->out_rpm=db_record_s.out_rpm;
                            db->out_front_board=22;
                            db->out_tail_board=-4;
                            db->out_rudder_left=db_record_s.z_rudder_turn;
                            db->out_rudder_right=0;
                            db->out_gate_left=ON; //左右水门打开
                            db->out_gate_right=ON;
                            PLC1_output=ON;
                            PLC2_output=ON;
                            pthread_mutex_unlock(&mutex_db);
                        }
                    }else if (status_z==-1)
                    {
                        //左边界情况
                        if(db_record_s.current_yaw>=boarder_right && db_record_s.current_yaw<180)
                        {
                            pthread_mutex_lock(&mutex_db);
                            db->out_rpm=db_record_s.out_rpm;
                            db->out_front_board=22;
                            db->out_tail_board=-4;
                            db->out_rudder_left=db_record_s.z_rudder_turn;
                            db->out_rudder_right=0;
                            db->out_gate_left=ON; //左右水门打开
                            db->out_gate_right=ON;
                            PLC1_output=ON;
                            PLC2_output=ON;
                            pthread_mutex_unlock(&mutex_db);
                        }
                    }

                }
                break;
            default:
                printf("Unknown Z mode\n");
                break;
        }
    }
}

void turn_mode_handler() {
    //printf("Entering Turn Mode: Executing turning maneuver...\n");
    //printf("Entering Z Mode: Z control is active...\n");
    DB1 db_record_s;//记录副本
    //printf("Entering Standby Mode: Initializing systems...\n");
    //保存数据副本
    static int error_overtime=0;
    pthread_mutex_lock(&mutex_db);
    db_record_s=*db;
    pthread_mutex_unlock(&mutex_db);
    if(new_data_IPC==ON)
    {
        //如果收到了来自地面站的数据
        pthread_mutex_lock(&mutex_db);
        new_data_IPC=OFF;
        pthread_mutex_unlock(&mutex_db);
        set_pid_para(db_record_s);
    }
    if(db_record_s.start_T==OFF)
    {
        if(1)//new_data_IMU==ON)
        {
            error_overtime=0;
            //数据处理，滤波平滑处理
            enqueue(&speed_msg, db_record_s.current_speed);
			enqueue(&yaw_msg, db_record_s.current_yaw);
			speed_median = median_filter(&speed_msg);
			yaw_median = median_filter(&yaw_msg);
            if(get5msg>=7)
            {
                get5msg=0;
                //如果pid参数已经写入，执行航速航向控制运算
                if(db_record_s.rpmorspeed==1)
                {
                    //rpm 控制
                    //执行到输出端
                    can1_send_msgs(3,db_record_s.Tar_rpm,0);
                    db_record_s.out_front_board=db_record_s.Tar_slider;
                    printf("front board:%f \n",db_record_s.out_front_board);
                }
                else if(db_record_s.rpmorspeed==2)
                {
                    //航速控制
                    int check_suisu;
                    float current_suisu;
                    current_suisu=speed_median;
                    check_suisu=checkForAnomaly(current_suisu,PreSpeed_suisu,PrePreSpeed_suisu);
                    if(check_suisu)
                    {
                        //航速控制任务
                        PID_realize_engine(db_record_s);
                        //首尾滑板控制函数
                        Slider_control(&db_record_s);
                    }
                    PrePreSpeed_suisu=PreSpeed_suisu;
					PreSpeed_suisu=speed_median;

                }
                //航向控制函数
                PID_realize_rudder(&db_record_s);

                //执行到输出端
                //can1_send_msgs(3,db_record_s.rpm);
                pthread_mutex_lock(&mutex_db);
                db->out_rpm=db_record_s.out_rpm;
                db->out_front_board=db_record_s.out_front_board;
                db->out_tail_board=db_record_s.out_tail_board;
                db->out_rudder_left=db_record_s.out_rudder_left;
                db->out_rudder_right=db_record_s.out_rudder_right;
                db->out_gate_left=ON; //左右水门打开
                db->out_gate_right=ON;
                PLC1_output=ON;
                PLC2_output=ON;
                pthread_mutex_unlock(&mutex_db);

            }
            else
            {
                get5msg++;
            }
            pthread_mutex_lock(&mutex_db);
            new_data_IMU=OFF;
            pthread_mutex_unlock(&mutex_db);
        }
        else
        {
            error_overtime++;
            if(error_overtime>=100)
            {
                pthread_mutex_lock(&mutex_db);
                db->error_sign1=1;
                //control_event.emergency=1;
                pthread_mutex_unlock(&mutex_db);
                error_overtime=0;
            }

        }

    }
    else if(db_record_s.back_to_straight==OFF)
    {
        //start to turn
        if(db_record_s.turn_LR==LEFT)
        {
            pthread_mutex_lock(&mutex_db);
            db->out_rpm=db_record_s.out_rpm;
            db->out_front_board=22;
            db->out_tail_board=-3;
            db->out_rudder_left=db_record_s.t_delta;
            db->out_rudder_right=0;
            db->out_gate_left=ON; //左右水门打开
            db->out_gate_right=ON;
            PLC1_output=ON;
            PLC2_output=ON;
            pthread_mutex_unlock(&mutex_db);
        }else
        {
            pthread_mutex_lock(&mutex_db);
            db->out_rpm=db_record_s.out_rpm;
            db->out_front_board=22;
            db->out_tail_board=-3;
            db->out_rudder_left=0;
            db->out_rudder_right=db_record_s.t_delta;
            db->out_gate_left=ON; //左右水门打开
            db->out_gate_right=ON;
            PLC1_output=ON;
            PLC2_output=ON;
            pthread_mutex_unlock(&mutex_db);
        }

    }else
    {
        //back to straght
        pthread_mutex_lock(&mutex_db);
        db->out_rpm=db_record_s.out_rpm;
        db->out_front_board=22;
        db->out_tail_board=-4;
        db->out_rudder_left=0;
        db->out_rudder_right=0;
        db->out_gate_left=ON; //左右水门打开
        db->out_gate_right=ON;
        PLC1_output=ON;
        PLC2_output=ON;
        pthread_mutex_unlock(&mutex_db);
    }
}

void zhidong_handler() {
    printf("Entering Zhidong Mode: Activating braking system...\n");
    pthread_mutex_lock(&mutex_db);
    db->out_rpm=900;
    db->out_front_board=22;
    db->out_tail_board=-3;
    db->out_rudder_left=0;
    db->out_rudder_right=0;
    db->out_gate_left=ON; //左右水门打开
    db->out_gate_right=ON;
    PLC1_output=ON;
    PLC2_output=ON;
    pthread_mutex_unlock(&mutex_db);
}


void maunal_control()
{
    DB1 db_record_s;//记录副本
    //printf("Entering Standby Mode: Initializing systems...\n");
    pthread_mutex_lock(&mutex_db);
    db_record_s=*db;
    pthread_mutex_unlock(&mutex_db);
    if(channel_onoff==ON)
    {
        channel_onoff=OFF;
        //发动机开关
        if(channel[7]==55248)
        {
            //启动发动机
            can1_send_msgs(1,0,0);
            pthread_mutex_lock(&mutex_db);
            db->out_engine=ON;
            pthread_mutex_unlock(&mutex_db);

        }
        else if(channel[7]==54248)
        {
            //关闭发动机
            can1_send_msgs(2,0,0);
            pthread_mutex_lock(&mutex_db);
            db->out_engine=OFF;
            pthread_mutex_unlock(&mutex_db);
        }
        //油门输出
        if(channel[0]>=50152 && channel[0]<=51152)
        {
            //油门范围正确
            db_record_s.Tar_rpm=(channel[0]);

            //db_record_s.out_front_board=db_record_s.Tar_slider;
            //printf("engine out rpm:%d \n",db_record_s.Tar_rpm);

            pthread_mutex_lock(&mutex_db);
            db->out_rpm=db_record_s.out_rpm;
            pthread_mutex_unlock(&mutex_db);
            //侧边舵
            if(channel[1]>=54248 && channel[1]<=55248)
            {
                if(channel[1]<=54748)
                {
                    db_record_s.t_delta=(float)(54748-channel[1])/500*60;
                    pthread_mutex_lock(&mutex_db);
                    db->out_rudder_left=db_record_s.t_delta;
                    db->out_rudder_right=0.0;
                    pthread_mutex_unlock(&mutex_db);
                    //printf("left rudder:%f \t",db_record_s.t_delta);
                    //printf("right rudder:0 \n");
                }
                else
                {
                    db_record_s.t_delta=(float)(channel[1]-54748)/500*60;
                    pthread_mutex_lock(&mutex_db);
                    db->out_rudder_left=0;
                    db->out_rudder_right=db_record_s.t_delta;
                    pthread_mutex_unlock(&mutex_db);
                    //printf("left rudder:0 \t");
                    //printf("right rudder:%f \n",db_record_s.t_delta);
                }
                can1_send_msgs(3,channel[0],channel[1]);
                //can2_send_msgs(3,db_record_s.Tar_rpm,db_record_s.t_delta);
            }

        }

        //水门左边
        if(channel[2]==22480)
        {
            //
            //printf("left gate:ON \n");
            pthread_mutex_lock(&mutex_db);
            db->out_gate_left=ON; //左右水门打开
            //db->out_gate_right=ON;
            pthread_mutex_unlock(&mutex_db);

        }else if(channel[2]==21480)
        {
            //printf("left gate:OFF \n");
            pthread_mutex_lock(&mutex_db);
            db->out_gate_left=OFF; //左右水门关闭
            //db->out_gate_right=ON;
            pthread_mutex_unlock(&mutex_db);
        }
        //水门右
        if(channel[3]==51152)
        {
            //右边按钮弹起
            //printf("right gate:ON \n");
            pthread_mutex_lock(&mutex_db);
            //db->out_gate_left=ON; //左右水门打开
            db->out_gate_right=ON;
            pthread_mutex_unlock(&mutex_db);

        }else if(channel[3]==50152)
        {
            //printf("right gate:OFF \n");
            pthread_mutex_lock(&mutex_db);
            //db->out_gate_left=OFF; //左右水门关闭
            db->out_gate_right=OFF;
            pthread_mutex_unlock(&mutex_db);
        }

        //首滑板
        if(channel[4]>=54748 && channel[4]<=55248)
        {

            float outbd=0;
            outbd=SAFE_BOARD-(float)(channel[4]-54748)/500*12;
            pthread_mutex_lock(&mutex_db);
            db->out_front_board=outbd;
            pthread_mutex_unlock(&mutex_db);
            //printf("front board:%f \n",outbd);

        }
        //尾滑板
        if(channel[5]>=21980 && channel[5]<=22480)
        {

            float tailbd=0;
            tailbd=-(float)(channel[5]-21980)/500*10;
            pthread_mutex_lock(&mutex_db);
            db->out_tail_board=tailbd;
            pthread_mutex_unlock(&mutex_db);
            //printf("tail board:%f \n",tailbd);

        }
        pthread_mutex_lock(&mutex_db);
        PLC1_output=ON;
        PLC2_output=ON;
        pthread_mutex_unlock(&mutex_db);
    }
}
// 状态机函数，确定状态并调用对应处理函数
MODE state_machine(MODE current_mode, Event event) {
    if (event.emergency) {
        current_mode = FAILURE_MODE;
    } else if (event.manual_control) {
        current_mode = FAILURE_MODE;
        maunal_control();
    } else {
        switch (event.mode) {
            case 0:
                current_mode = STANDBY_MODE;
                standby_handler();
                break;
            case 2:
                current_mode = MANUAL_MODE;
                manual_handler();
                break;
            case 3:
                current_mode = PID_MODE;
                pid_handler();
                break;
            case 4:
                current_mode = LOS_MODE;
                los_handler();

                break;
            case 5:
                current_mode = Z_MODE;
                z_mode_handler();
                break;
            case 6:
                current_mode = TURN_MODE;
                turn_mode_handler();
                break;
            case 7:
                current_mode = ZHIDONG_MODE;
                zhidong_handler();
                break;
            default:
                printf("Unknown mode\n");
                break;
        }
    }


    return current_mode;
}
//main control task
void *boat_control_task(void *arg)
{
    MODE cmode = STANDBY_MODE;
    Event cevent = {0, 0, 0};
    struct timespec sleep_time1;
    sleep_time1.tv_sec = 0;
    sleep_time1.tv_nsec = 40 * 1000000;  // 40ms = 100,000,000 纳秒
    while(1)
    {
        pthread_mutex_lock(&mutex_db);
        cmode=control_mode;
        cevent=control_event;
        pthread_mutex_unlock(&mutex_db);
        cmode=state_machine(cmode, cevent);
        nanosleep(&sleep_time1, NULL);

    }
    pthread_exit(NULL);

}

//
int configure_uart(int fd) {
    struct termios options;
    tcgetattr(fd, &options);

    // 设置波特率为 115200
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // 设置8个数据位，无校验，1个停止位
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // 原始模式（不处理特殊字符）
    options.c_lflag = 0;

    // 应用设置
    tcsetattr(fd, TCSANOW, &options);

    return 0;
}
// 解析iBus数据包函数
void parse_ibus_packet(unsigned char* rx_buffer) {
    if(rx_buffer[0] == IBUS_LENGTH && rx_buffer[1] == IBUS_COMMAND40) {
        checksum_cal = 0xffff - rx_buffer[0] - rx_buffer[1];

        for (int i = 0; i < IBUS_MAX_CHANNLES; i++) {
            channel_buffer[i] = (uint16_t)(rx_buffer[i * 2 + 3] << 8 | rx_buffer[i * 2 + 2]);
            checksum_cal = checksum_cal - rx_buffer[i * 2 + 3] - rx_buffer[i * 2 + 2];
        }

        checksum_ibus = rx_buffer[31] << 8 | rx_buffer[30];

        if (checksum_cal == checksum_ibus) {
            for (int j = 0; j < user_channels; j++) {
                channel[j] = channel_buffer[j];
                //printf("Channel %d: %d\n", j + 1, channel[j]);

            }
            channel_onoff=ON;
            if(channel[6]==51152)
            {
                //当通道6+1下按后，swe=100，表示手动遥控器启动

                pthread_mutex_lock(&mutex_db);
                control_event.manual_control=1;
                pthread_mutex_unlock(&mutex_db);

            }
            else// if(channel[6]==50652)
            {
                pthread_mutex_lock(&mutex_db);
                control_event.manual_control=0;
                pthread_mutex_unlock(&mutex_db);
            }
            /*
            else
            {
                //遥控器sw7上挑起，表明只控制油门和首滑板
                if(channel[6]==50152)
                {
                    pthread_mutex_lock(&mutex_db);
                    control_event.manual_control=0;
                    pthread_mutex_unlock(&mutex_db);

                            //油门输出
                    if(channel[0]>=50652 && channel[0]<=51152)
                    {
                        //油门范围正确
                        int Tar_rpm_=(channel[0]-50652)*5.8+900;
                        can1_send_msgs(3,Tar_rpm_);
                        //printf("engine out rpm:%d \n",db_record_s.Tar_rpm);

                        pthread_mutex_lock(&mutex_db);
                        db->out_rpm=Tar_rpm_;
                        pthread_mutex_unlock(&mutex_db);

                    }
                            //首滑板
                    if(channel[4]>=54748 && channel[4]<=55248)
                    {

                        float outbd_=0;
                        outbd_=SAFE_BOARD-(float)(channel[4]-54748)/500*12;
                        pthread_mutex_lock(&mutex_db);
                        db->out_front_board=outbd_;
                        pthread_mutex_unlock(&mutex_db);
                        //printf("front board:%f \n",outbd);

                    }
                    pthread_mutex_lock(&mutex_db);
                    PLC1_output=ON;
                    PLC2_output=ON;
                    pthread_mutex_unlock(&mutex_db);

                }

            }*/
        }
    }
}


void* PL18(void *arg)
{
    unsigned char buffer[64];  // 存储完整的64字节数据包
    int buffer_index = 0;  // 追踪当前缓冲区中数据的字节数
    int uart_fd;  // 串口文件描述符


    // 打开串口设备
    uart_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        perror("Unable to open UART");
        //return 1;
    }

    // 配置 UART
    configure_uart(uart_fd);

    // 设置为非阻塞模式
    fcntl(uart_fd, F_SETFL, FNDELAY);

    unsigned char ibus_data[64];
    memset(ibus_data, 0, sizeof(ibus_data));

    fd_set read_fds;
    struct timeval timeout;

    while (1) {
        // 清除 fd 集合并将 UART 文件描述符加入
        FD_ZERO(&read_fds);
        FD_SET(uart_fd, &read_fds);

        // 设置超时时间，非阻塞等待数据 (timeout为0,非阻塞)
        timeout.tv_sec = 0;
        timeout.tv_usec = 5000;  // 每5ms检查一次

        // 使用 select 检查是否有数据可读
        int select_result = select(uart_fd + 1, &read_fds, NULL, NULL, &timeout);

        if (select_result > 0 && FD_ISSET(uart_fd, &read_fds)) {
            // 读取 iBus 数据
            int bytes_read = read(uart_fd, ibus_data, sizeof(ibus_data));
            if (bytes_read > 0) {
                for (int i = 0; i < bytes_read; i++) {
                    buffer[buffer_index++] = ibus_data[i];

                    // 如果缓冲区有32字节以上数据，检查包头
                    if (buffer_index >= 32) {
                        for (int j = 0; j <= buffer_index - 32; j++) {
                            // 检查是否找到了有效的包头
                            if (buffer[j] == 0x20 && buffer[j + 1] == 0x40) {
                                // 如果找到包头，从该位置开始解析
                                parse_ibus_packet(&buffer[j]);

                                // 将剩余的字节移到缓冲区开头
                                memmove(buffer, &buffer[j + 32], buffer_index - (j + 32));
                                buffer_index -= (j + 32);
                                break;
                            }
                        }
                    }

                    // 防止缓冲区溢出
                    if (buffer_index >= sizeof(buffer)) {
                        buffer_index = 0;
                    }
                }

            }
        }

        // 继续循环，保持高频率读取
    }

    // 关闭 UART
    close(uart_fd);
}
int main(void)
{
    zmode=0;
    turnmode=0;
    zhidongmode = 0;
    pidcad = 0;
    summ=0;
    get5msg=0;
    pthread_t thread_id;
    pthread_t data_thread;
    pthread_t control_task;
    pthread_t can_task;

    pthread_t pl8_task;
    Kpc=0.4;
    Kic=0.1;
    Kdc=0.1;
    Kpe=5;
    Kie=0.1;
    Kde=15;
    const char* filename = "data.txt";
    //global struct
    db = (DB1 *)malloc(sizeof(DB1));
    memset(db, 0, sizeof(DB1));
    db->PID_self_test=NOTGOOD;
    db->LOS_ONOFF=OFF;
    init_queue(&speed_msg);
	init_queue(&yaw_msg);
	speed_median=0;
	yaw_median=0;
	sleep(10);
    if (db==NULL)
    {
        return 0;
    }
    kalman_init(&kf,0.35,0.003);//初始化kalman积分函数
    //过程噪声协方差 如果系统本身比较稳定，可以设置较小的值（如 0.001）；如果系统有较大变化，设为较大的值（如 1.0或者更高）
    //测量噪声协方差   如果 GPS 数据的噪声较小，设为较小的值（如 0.01）；如果测量噪声较大，可以设置为较大的值（如 1.0或者更高）
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    //db->mode=3;
    //gps from serial
    // 创建串口读取线程

    if (pthread_create(&thread_id, NULL, serial_read_thread, NULL) != 0) {
        perror("pthread_create");
        return 1;
    }

        // 创建线程运行数据采集函数
    if (pthread_create(&data_thread, NULL, collect_data_and_write_to_file, (void*)filename)) {
        fprintf(stderr, "线程创建失败\n");
        return 1;
    }

    // 主线程可以执行其他任务
    // 在这里我们只是简单地等待串口线程完成
    //***************************

    if (pthread_create(&control_task, NULL, boat_control_task, NULL)!=0) {
        fprintf(stderr, "main主线程创建失败\n");
        return 1;
    }
    //PL18
    if (pthread_create(&pl8_task, NULL, PL18, NULL)!=0) {
        fprintf(stderr, "PL18控制创建失败\n");
        return 1;
    }

    //can bus for engine
    int can=init_can();
    if(can == -1)
    {
        fprintf(stderr, "can总线初始化失败\n");

    }else{
            if (pthread_create(&can_task, NULL, test_can, NULL)!=0) {
                fprintf(stderr, "can线程创建失败\n");
                return 1;
        }
    }

    GP_INIT(PORT);

    while(1)
    {
        //printf("main programme\n");
        sleep(1);
    }
    pthread_join(thread_id, NULL);
    pthread_join(data_thread, NULL);
    pthread_join(control_task, NULL);
    pthread_join(pl8_task, NULL);
    pthread_join(can_task, NULL);
    free(db);
    pthread_mutex_destroy(&mutex);
    return 0;
}
