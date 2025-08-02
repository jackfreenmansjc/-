#include "socket_6666.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <arpa/inet.h>
int count_connect = 0;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
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
