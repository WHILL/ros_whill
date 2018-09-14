/*
 * Functions to use UART
 * author : Kazumichi Shirai
 */

#include <string> 

#define PROTOCOL_SIGN (0xAF)

int sendCmdUART(int fd, char cmd[], int len);
int recvDataUART(int fd, char recv_buf[]);
int initializeUART(int *fd,std::string port);
void closeUART(int fd);
