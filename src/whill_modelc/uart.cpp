/*
 * Functions to use UART
 * author : Kazumichi Shirai
 */

#include<stdio.h>
#include<stdlib.h>
#include<strings.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include "./uart.h"

#include "ros/ros.h"

//#define BAUDRATE B19200 //Model A
#define BAUDRATE B38400 //Model C


static struct termios oldtio;

int sendCmdUART(int fd, char cmd[], int len)
{
     if(write(fd, cmd, len) != len) 
     {
	  fprintf(stderr, "fail to write command %02x \n", cmd[2]);
	  return -1;
     }
     return 1;
}

int recvDataUART(int fd, char recv_buf[])
{
     int size;
     char prtcl;
     char len, remain_len;
     char tmp_recv_buf[128];
     int i, j,idx;
     int retry = 3;
     int check_sum = 0;
   
     //** Recv Protocol Sign **//
     size = read(fd, tmp_recv_buf, 1);

     if((tmp_recv_buf[0] & 0xff) != PROTOCOL_SIGN) return -1;
     //fprintf(stdout, "protocl sign = %d\n", tmp_recv_buf[0]);
     check_sum ^= tmp_recv_buf[0];

     //** Recv Data length **//
     size = read(fd, tmp_recv_buf, 1);
     //fprintf(stdout, "data length = %d\n", tmp_recv_buf[0]);
     check_sum ^= tmp_recv_buf[0];
     len = tmp_recv_buf[0];
     remain_len =len;
     idx = 0;

     //** Recv Data **//
     while(remain_len){
	  size = read(fd, tmp_recv_buf, remain_len);//recv data
	  //fprintf(stdout, "remain_len = %d, size = %d\n", remain_len, size);
	  if(size < 0)
	  {
	       fprintf(stderr, "fail to read\n");
	  }
	  for(i=0;i<size;i++)
	  {
	       recv_buf[idx] = tmp_recv_buf[i];
	       idx++;
	  }
	  remain_len = remain_len - size;
     }

     //** Check check_sum **//
     for(i=0;i<len-1;i++){
	     check_sum ^= recv_buf[i];
     }
     if(check_sum != recv_buf[len-1]){
       if(check_sum != recv_buf[len-1]){
	 fprintf(stderr, "checksum err, check_sum = %d, recv_buf = %d\n",check_sum, recv_buf[len-1]);
	 fprintf(stderr, "len = %d\n", len);
	 int debug_idx = 0;
	 fprintf(stderr, "0xAF,");
	 fprintf(stderr, "0x%x,", len);
	 for(debug_idx=0;debug_idx<len;debug_idx++){
	   fprintf(stderr, "0x%x,",recv_buf[debug_idx]);
	 }
	 //fprintf(stderr, "\n",recv_buf[debug_idx]);
	 return -1;
       }
     }
     return len;
}
	  
int initializeUART(int *fd,std::string port)
{
     /* fd :  シリアル通信ファイルディスクリプタ */
     struct termios newtio;    /* シリアル通信設定 */
         
     if(!(*fd = open(port.c_str(), O_RDWR))){
       fprintf(stderr, "Can't open UART device\n");
       return -1; /* デバイスをオープンする */
     }
     ioctl(*fd, TCGETS, &oldtio);       /* 現在のシリアルポートの設定を待避 */
 
     bzero(&newtio, sizeof(newtio));
     newtio = oldtio;                  /* ポートの設定をコピー */
     newtio.c_cflag = (BAUDRATE | CS8 | CLOCAL | CREAD | CSTOPB);
 
     newtio.c_iflag = (IGNPAR); //Ignore frame error and parity error
     newtio.c_oflag = 0;
     //newtio.c_lflag = ICANON; /* カノニカルモード */
     newtio.c_lflag = 0; /* 非カノニカルモード */
 
     ioctl(*fd, TCSETS, &newtio);       /* ポートの設定を有効にする */

     return 1;
}

void closeUART(int fd)
{
     ioctl(fd, TCSETS, &oldtio);       //ターミナルの設定を元に戻しておく
     close(fd);                        //シリアルポートを閉じる
}
