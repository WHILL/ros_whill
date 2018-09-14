/*
 * Functions to use UART
 * author : Kazumichi Shirai
 */
#define PROTOCOL_SIGN (0xAF)

int sendCmdUART(int fd, char cmd[], int len);
int recvDataUART(int fd, char recv_buf[]);
int initializeUART(int *fd);
void closeUART(int fd);
