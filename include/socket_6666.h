#ifndef SOCKET_UTILS_H
#define SOCKET_UTILS_H

#include <pthread.h>
#include <netinet/in.h>
#include <sys/socket.h>
// 定义常量和结构体
#define IPLENGTH 20
#define BUFFSIZE 1024
#define MAXCONN 10
#define ERRORCODE -1
extern count_connect;
extern mutex;
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
static void *thread_send(void *arg);
static void* thread_recv(void *arg);
static int create_listen(int port);
ClientInfo accept_socket(int listen_st);
int GP_INIT(int port);
#endif
