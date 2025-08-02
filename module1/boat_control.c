#include "boat_control.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "data.h"


//LOS
#define PI 3.14159265358979323846
#define EARTH_RADIUS 6371000.0
#define BASE_LOOKAHEAD_DISTANCE 2.0  // ����ǰ�Ӿ���
//����NMEA��ʽ��gps����
void parseNMEA(char *sentence, struct GPSData *data) {
    char *token;
    int index = 0;
    token = strtok(sentence, ",");

    while (token != NULL) {
        switch (index) {
            case 3: // Latitude
                sscanf(token, "%f", &(data->latitude));
                break;
            case 5: // Longitude
                sscanf(token, "%f", &(data->longitude));
                break;
            case 7: // Speed
                sscanf(token, "%f", &(data->speed));
                break;
            case 1: // Time
                strncpy(data->time, token, 6); // Assuming time format is HHMMSS
                data->time[6] = '\0';
                break;
            default:
                break;
        }
        index++;
        token = strtok(NULL, ",");
    }
}
float hexToFloat(unsigned char *bytes) {
    float value;
    unsigned char p[4];
    p[0] = bytes[3];
    p[1] = bytes[2];
    p[2] = bytes[1];
    p[3] = bytes[0];
    memcpy(&value,p,4);
    return value;
}

//
float hexToFloat1(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
    float value1;
    uint32_t combined_bytes = ((uint32_t)byte4 << 24) | ((uint32_t)byte3 << 16) | ((uint32_t)byte2 << 8) | (uint32_t)byte1;
    memcpy(&value1,&combined_bytes,sizeof(uint32_t));
    return value1;
}
//
double data_process_GRO(unsigned char *received_data,int data_length)
{
	//
	//size_t data_length = sizeof(received_data);
	//printf("received data length: %d \n",data_length);
	int i;
	//int j;
	for (i = 0; i < data_length - 1; i++) {
		if (received_data[i] == 0xc0 && received_data[i + 1] == 0xc0) {
			//printf("yes got 0xc0 0xc0\n");
			if((i+55)<= data_length)
			{
				if(received_data[i+55] == 0x00)
				{
					// �ҵ�����ͷ��β����ȡ����Ƕ�ֵ
					//unsigned char heading_bytes[] = {received_data[i + 23], received_data[i + 24], received_data[i + 25], received_data[i + 26]};
					float heading = hexToFloat1(received_data[i + 22], received_data[i + 23], received_data[i + 24], received_data[i + 25]);
					// ��ӡ����Ƕ�ֵ
					/*
					for (j=19;j<=27;j++)
					{
						printf(" N%d :0x%02x,",(j),received_data[j + i]);
					}
					*/
					//printf("0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",received_data[i + 22],received_data[i + 23],received_data[i + 24],received_data[i + 25],received_data[i + 26]);

					//printf("current yaw: %f\n", heading);
					//printf("\n");
					//float acc_x = hexToFloat1(received_data[i + 38], received_data[i + 39], received_data[i + 40], received_data[i + 41]);
					float acc_y = hexToFloat1(received_data[i + 42], received_data[i + 43], received_data[i + 44], received_data[i + 45]);
					double acc_Y;
					acc_Y=acc_y;
					//float acc_z = hexToFloat1(received_data[i + 46], received_data[i + 47], received_data[i + 48], received_data[i + 49]);
                    //printf("current X: %f\t", acc_x);

                    //printf("current Y: %f\t", acc_y);

                    //printf("current Z: %f\n", acc_z);
					//printf("\n");
					if(db==NULL)
					{

					}else
					{
						pthread_mutex_lock(&mutex);
						db->current_yaw = heading;
						db->yaw = heading;
						pthread_mutex_unlock(&mutex);
						return acc_Y;
					}
					break;
				}
			}


		}
	}
	return 0;
}

void data_process_GPS(char *data)
{
	//��������gps������
}

void data_process_PLC(unsigned char *data,int data_length)
{
	char buff_plc[20];

	if(db==NULL)
	{

	}
	else
	{
		//
		if(data_length == 20 )
		{
			if(data[18]==0xff && data[19] == 0xff)
			{
				//
				if(data[0] == 0xff & data[1] == 0xff)
				{
					pthread_mutex_lock(&mutex);
					//db->out_engine = charsToInt(&data[2]);
					db->out_rudder_left = charsToFloat(&data[4]);
					db->out_rudder_right = charsToFloat(&data[6]);
					//db->out_front_board = charsToFloat(&data[8]);
					//db->out_tail_board = charsToFloat(&data[10]);
					db->out_gate_left = charsToInt(&data[12]);
					db->out_gate_right = charsToInt(&data[14]);
					//db->out_rpm = charsToInt(&data[16]);
					pthread_mutex_unlock(&mutex);
					//printf("get message from PLC\n");
				}

			}
		}

	}
}

void data_process_PLC_wei(unsigned char *data,int data_length)
{
	char buff_plc[20];

	if(db==NULL)
	{

	}
	else
	{
		//
		if(data_length == 20 )
		{
			if(data[18]==0xff && data[19] == 0xbb)
			{
				//
				if(data[0] == 0xff & data[1] == 0xaa)
				{
					pthread_mutex_lock(&mutex);
					//db->out_engine = charsToInt(&data[2]);
					//db->out_rudder_left = charsToFloat(&data[4]);
					//db->out_rudder_right = charsToFloat(&data[6]);
					db->out_front_board = charsToFloat(&data[8]);
					db->out_tail_board = charsToFloat(&data[10]);
					//db->out_gate_left = charsToInt(&data[12]);
					//db->out_gate_right = charsToInt(&data[14]);
					//db->out_rpm = charsToInt(&data[16]);
					pthread_mutex_unlock(&mutex);
					//printf("get message from PLC\n");
				}

			}
		}

	}
}
void data_process_IPC(unsigned char *data)
{
	Boatcontrol *ipc =(Boatcontrol *)data;
	if(db==NULL)
	{

	}
	else
	{
		pthread_mutex_lock(&mutex);
		//pid parameters
		db->mode=ipc->mode;
		db->engine=ipc->engine;
		db->HA=ipc->HA;
		db->PID_self_test=ipc->PID_self_test;
		db->Tar_yaw=ipc->Tar_yaw;
		db->Tar_speed=ipc->Tar_speed;
		db->kp_r=ipc->kp_r;
		db->ki_r=ipc->ki_r;
		db->kd_r=ipc->kd_r;
		db->umin_r=ipc->umin_r;
		db->umax_r=ipc->umax_r;
		db->error_max_r=ipc->error_max_r;
		db->output_min_r=ipc->output_min_r;
		db->output_max_r=ipc->output_max_r;

		db->kp_e=ipc->kp_e;
		db->ki_e=ipc->ki_e;
		db->kd_e=ipc->kd_e;
		db->umin_e=ipc->umin_e;
		db->umax_e=ipc->umax_e;
		db->error_max_e=ipc->error_max_e;
		db->output_min_e=ipc->output_min_e;
		db->output_max_e=ipc->output_max_e;
		db->Tar_rpm=ipc->Tar_rpm;
		db->Tar_slider=ipc->Tar_slider/10;
		db->rpmorspeed=ipc->rpmorspeed;
		//manual mode
		db->manualOK = ipc->manualOK;
		db->rpm=ipc->rpm;
		db->rudder_left=ipc->rudder_left;
		db->rudder_right=ipc->rudder_right;
		db->front_board=ipc->front_board/10;
		db->tail_board=ipc->tail_board/10;
		db->gate_left=ipc->gate_left;
		db->gate_right=ipc->gate_right;
		db->acc_point=ipc->acc_point;
		db->dec_point=ipc->dec_point;
		//db->IPC_self_test=GOOD;
		//los mode
		memcpy(db->GPS_X, ipc->GPS_X, 15 * sizeof(double));  //lat
		memcpy(db->GPS_Y, ipc->GPS_Y, 15 * sizeof(double));  //lon
		db->num_points=ipc->num_points;
		db->LOS_ONOFF=ipc->LOS_ONOFF;
		//z mode
		db->z_rudder_turn=ipc->rudder_turn;
		db->z_rudder_trun_speed=ipc->rudder_turn_speed;
		db->z_yaw_delta=ipc->yaw_delta;
		db->z_rudder_left_first=ipc->rudder_left_first;
		db->start_Z=ipc->start_Z;
		//turn mode
		db->t_delta=ipc->t_delta;   //�ٶ��
		db->t_turn_speed=ipc->t_turn_speed;	//ת���ٶ�
		db->turn_LR=ipc->turn_LR;
		db->back_to_straight = ipc->back_to_straight;
		db->standby=ipc->standby;
		db->start_T=ipc->start_T;

		pthread_mutex_unlock(&mutex);

		/*
		printf("mode is: %d \n",ipc->mode);
		printf("engine is: %d \n",ipc->engine);
		printf("selfPIDtest is: %d \n",ipc->PID_self_test);
		printf("tar speed is: %f \n",ipc->Tar_speed);
		printf("tar yaw is: %f \n",ipc->Tar_yaw);
		printf("PID kp_r is: %f \t",ipc->kp_r);
		printf("PID ki_r is: %f \t",ipc->ki_r);
		printf("PID kd_r is: %f \t",ipc->kd_r);
		printf("PID umin_r is: %f \t",ipc->umin_r);
		printf("PID umax_r is: %f \t",ipc->umax_r);
		printf("PID kp_e is: %f \t",ipc->kp_e);
		printf("PID ki_e is: %f \t",ipc->ki_e);
		printf("PID kd_e is: %f \t",ipc->kd_e);
		printf("PID umin_e is: %f \t",ipc->umin_e);
		printf("PID umax_e is: %f \n",ipc->umax_e);
		printf("PID rudder errormax is: %f \n",ipc->error_max_r);
		printf("PID rudder ouput min is: %f \n",ipc->output_min_r);
		printf("PID rudder ouput max is: %f \n",ipc->output_max_r);
		printf("PID engine errormax is: %f \n",ipc->error_max_e);
		printf("PID engint ouput min is: %f \n",ipc->output_min_e);
		printf("PID engint ouput max is: %f \n",ipc->output_max_e);
		printf("manual rpm  is: %d \n",ipc->rpm);
		printf("manual rudder left  is: %f \n",ipc->rudder_left);
		printf("manual rudder right  is: %f \n",ipc->rudder_right);
		printf("manual front slide  is: %f \n",ipc->front_board);
		printf("manual tail slide  is: %f \n",ipc->tail_board);
		printf("manual gate left:%d \n",ipc->gate_left);
		printf("manual gate right:%d \n",ipc->gate_right);
		printf("zmode  rudder_turn:%f \n",ipc->rudder_turn);
		printf("turn mode  t delta:%f \n",ipc->t_delta);
		printf("turn mode  back to straight:%d \n",ipc->back_to_straight);


		printf("mode is: %d \n",db->mode);
		printf("selfPIDtest is: %d \n",db->PID_self_test);
		printf("tar speed is: %f \n",db->Tar_speed);
		printf("tar yaw is: %f \n",db->Tar_yaw);
		printf("PID kp_r is: %f \n",db->kp_r);
		printf("PID ki_r is: %f \n",db->ki_r);
		printf("PID kp_e is: %f \n",db->kp_e);
		printf("PID ki_e is: %f \n",db->ki_e);
		printf("rpm  is: %d \n",db->rpm);
		printf("rudder left  is: %f \n",db->rudder_left);
		printf("rudder right  is: %f \n",db->rudder_right);
        printf("front slide  is: %f \n",db->front_board);
		printf("tail slide  is: %f \n",db->tail_board);
		printf("gate left:%d \n",db->gate_left);
		printf("gate right:%d \n",db->gate_right);
		printf("standy mode:%d \n\n",db->standby);
        */
		//printf("rudder  is: left:%f ,right:%f \n",db->rudder_left,db->rudder_right);
		//printf("board  is: front:%f ,tail:%f \n",db->front_board,db->tail_board);
		//printf("gate  is: left:%d ,right:%d \n",db->gate_left,db->gate_right);


	}
}
//����ת���������ַ���
void intToChars(int num, char *chars) {
    chars[1] = (num >> 8) & 0xFF;  // ȡ����8λ�洢��chars[0]
    chars[0] = num & 0xFF;          // ȡ����8λ�洢��chars[1]
}
//������ת���������ַ��У�����������10��������һλС��
void floatToChars(float num_float, char *chars) {
	int num;
	num = (int)(num_float * 10.0f);
    chars[1] = (num >> 8) & 0xFF;  // ȡ����8λ�洢��chars[0]
    chars[0] = num & 0xFF;          // ȡ����8λ�洢��chars[1]
}
//�����ַ�ת����һ������
int charsToInt(char *chars) {
    int num = (chars[0] & 0xFF) | ((chars[1] & 0xFF) << 8); // �ϲ�chars[1]��chars[0]�õ���������
    return num;
}
//�����ַ�ת����һ��������
float charsToFloat(char *chars) {
    int num = (chars[0] & 0xFF) | ((chars[1] & 0xFF) << 8); // �ϲ�chars[1]��chars[0]�õ���������
    return (float)num / 10.0f; // ���������ֳ���10�õ�������
}
// ���Բ�ֵ����
float linearInterpolation(float x, float x0, float x1, float y0, float y1)
{
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

float get_board_ang(float x_point,float xx_point) {
    // ��֪��һ������
	//��ǰ����
    float x_data_acc[] = {0, 25, 25.1, 27.5, 27.6, 29.5, 29.6, 34, 34.1, 40 };  //��Ϊ���ٹ���
    float x_data_dec[] = {0, 26, 26.1, 28, 28.1, 30.2, 30.3, 35, 35.1, 40 };   //��Ϊ���ٹ���
    float y_data[] = {22, 22, 18, 18, 14, 14, 12, 12, 10, 10};   //��Ӧ�׻���Ƕ�ӳ��


    // Ҫ��ȡ�����ݵ�
    if (x_point < 0)
    {
    	return 29;
    }
    // ����֪���ݵ����ҵ����ڵ�������
    int i;

    //��ǰ�ٶ�Ŀ���ٶȿ�
    float result;
    if(x_point >= xx_point)
    {
    	for (i = 0; i < sizeof(x_data_dec) / sizeof(x_data_dec[0]) - 1; i++) {
			if (x_point >= x_data_dec[i] && x_point <= x_data_dec[i + 1]) {
				break;
			}
		}
    	// �������Բ�ֵ
		result = linearInterpolation(x_point, x_data_dec[i], x_data_dec[i + 1], y_data[i], y_data[i + 1]);
    }
    else
    {
    	for (i = 0; i < sizeof(x_data_acc) / sizeof(x_data_acc[0]) - 1; i++) {
			if (x_point >= x_data_acc[i] && x_point <= x_data_acc[i + 1]) {
				break;
			}
		}
    	// �������Բ�ֵ
		result = linearInterpolation(x_point, x_data_acc[i], x_data_acc[i + 1], y_data[i], y_data[i + 1]);
    }


    // ������
    //printf("��Ӧ���ݵ� %.2lf �Ĺ���ֵΪ %.1f\n", x_point, result);

    return round(result * 10.0) / 10.0;;
}


//LOS���߷�ʵ��С��·������




// ��γ��תƽ������
void latlon_to_xy(double lat1, double lon1, double lat2, double lon2, double *dx, double *dy) {
    double lat1_rad = lat1 * PI / 180.0;
    double lat2_rad = lat2 * PI / 180.0;
    double lon1_rad = lon1 * PI / 180.0;
    double lon2_rad = lon2 * PI / 180.0;

    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;
    double avg_lat = (lat1_rad + lat2_rad) / 2.0;

    *dx = EARTH_RADIUS * dlon * cos(avg_lat);
    *dy = EARTH_RADIUS * dlat;
}

// ��������ͧ��Ŀ����ƫ����
double calculate_LOS_yaw(double target_x, double target_y, double current_x, double current_y) {
    double delta_x = target_x - current_x;
    double delta_y = target_y - current_y;

    double los_yaw = atan2(delta_y, delta_x);
    if (los_yaw < 0) {
        los_yaw += 2 * PI;
    }
    return los_yaw;
}

// ��������֮��ľ���
double calculate_distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

// ���������ڻ��������ж��Ƿ�Խ��Ŀ��㣩
double calculate_dot_product(double vx1, double vy1, double vx2, double vy2) {
    return vx1 * vx2 + vy1 * vy2;
}

// ��̬ǰ�Ӿ���� LOS �����㷨������������Խ���ж�
float LOS_control_dynamic(double GPS_X[], double GPS_Y[], int num_points, double current_lat, double current_lon, float current_yaw, float speed, ONOFF onoff) {
    static int current_target_index = 0;
    if(onoff==ON)
    {
        current_target_index=0;
    }
    if (current_target_index >= num_points) {
        printf("num_points:%d\n",num_points);
        return current_yaw;
    }

    // ��ȡ��ǰĿ������һ����ľ�γ��
    double target_lat = GPS_X[current_target_index];
    double target_lon = GPS_Y[current_target_index];

    // ����ǰ���Ŀ���ת��Ϊƽ������
    double target_x, target_y, current_x = 0, current_y = 0;
    latlon_to_xy(current_lat, current_lon, target_lat, target_lon, &target_x, &target_y);
    latlon_to_xy(current_lat, current_lon, current_lat, current_lon, &current_x, &current_y);

    // �ж��Ƿ�Խ��Ŀ��㣨ʹ����������
    if (current_target_index < num_points - 1) {
        // ��ȡ��һ��Ŀ���
        double next_target_lat = GPS_X[current_target_index + 1];
        double next_target_lon = GPS_Y[current_target_index + 1];
        double next_target_x, next_target_y;
        latlon_to_xy(current_lat, current_lon, next_target_lat, next_target_lon, &next_target_x, &next_target_y);

        // ����ӵ�ǰ�㵽Ŀ��������
        double vec_target_x = target_x - current_x;
        double vec_target_y = target_y - current_y;

        // ����ӵ�ǰ�㵽��һ���������
        double vec_next_target_x = next_target_x - target_x;
        double vec_next_target_y = next_target_y - target_y;

        // �����ڻ����ж��Ƿ�Խ��Ŀ���
        double dot_product = calculate_dot_product(vec_target_x, vec_target_y, vec_next_target_x, vec_next_target_y);
        if (dot_product < 0) {
            // ��Խ��Ŀ��㣬�л�����һ��Ŀ���
            current_target_index++;
            if (current_target_index >= num_points) {
                return current_yaw;  // �Ѿ��������һ��Ŀ���
            }
            target_lat = GPS_X[current_target_index];
            target_lon = GPS_Y[current_target_index];
            latlon_to_xy(current_lat, current_lon, target_lat, target_lon, &target_x, &target_y);
        }
    }

    // �����ٶȵ���ǰ�Ӿ���
    double lookahead_distance = BASE_LOOKAHEAD_DISTANCE + speed * 0.1;

    // ��������ͧ��Ŀ���ľ���
    double distance_to_target = calculate_distance(current_x, current_y, target_x, target_y);

    // �ж��Ƿ����ǰ�Ӿ��뷶Χ
    if (distance_to_target < lookahead_distance) {
        current_target_index++;
        if (current_target_index >= num_points) {
            return current_yaw;  // �ѵ������һ��Ŀ���
        }
        target_lat = GPS_X[current_target_index];
        target_lon = GPS_Y[current_target_index];
        latlon_to_xy(current_lat, current_lon, target_lat, target_lon, &target_x, &target_y);
    }

    // ���� LOS ƫ����
    float desired_yaw = (float)calculate_LOS_yaw(target_x, target_y, current_x, current_y)*180/PI;

    desired_yaw-=90;
    if(desired_yaw<0)
    {
        desired_yaw+=360;
    }
    desired_yaw=360-desired_yaw;

    /*
    // ���㵱ǰ������������������
    float yaw_error = desired_yaw - current_yaw*PI/180;
    if (yaw_error > PI) {
        yaw_error -= 2 * PI;
    } else if (yaw_error < -PI) {
        yaw_error += 2 * PI;
    }
    */
    return desired_yaw;  // ���غ������
}

float LOS_main(DB1 *db_record_s,ONOFF onoff) {
    // ʾ�����룺����켣��ľ�γ��
    double GPS_X[15]={0};//= {30.9886000, 30.9886050};  // γ��
    double GPS_Y[15]={0};//= {121.3612950, 121.3613000};  // ����
    int num_points;
    /*
    if(db_record_s->GPS_X[0]!=0)
    {
        memcpy(GPS_X, db_record_s->GPS_X, 15 * sizeof(double));
        memcpy(GPS_Y, db_record_s->GPS_Y, 15 * sizeof(double));
        num_points=db_record_s->num_points;
    }
    */
    // ��ǰ����ͧ�ľ�γ�Ⱥͺ���
    double current_lat = db_record_s->current_latitude;
    double current_lon = db_record_s->current_longitude;
    //printf("current latitude: %lf , longitude: %lf",current_lat,current_lon);
    float current_yaw = db_record_s->current_yaw;  // ��ǰ���� �Ƕ�
    float speed = db_record_s->current_speed;  // �����ٶ�Ϊ km/h

    // ���ö�̬ǰ�Ӿ���� LOS �����㷨
    float yaw_error = LOS_control_dynamic(GPS_X, GPS_Y, num_points, current_lat, current_lon, current_yaw, speed,  onoff);

    // ����������
    //printf("Target yaw for LOS: %f\n", yaw_error);

    return yaw_error;
}
