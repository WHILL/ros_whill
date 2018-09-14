#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <time.h>
#include <sys/time.h>
#include "./com_whill.h"

#define RESPONSE_DATA_IDX (0)
#define RESPONSE_DATA (0x52)

void main( void )
{
     int whill_fd; // file descriptor for UART to communicate with WHILL
     char recv_buf[128];
     int len, idx;
     int i, counter;


     initializeComWHILL(&whill_fd);

     /** send PowerOn command */
     sendPowerOn(whill_fd);
     usleep(2000);
     len = recvDataWHILL(whill_fd, recv_buf);
     for(idx=0;idx<len;idx++){
       fprintf(stdout, "%d, %d\n", idx, recv_buf[idx]);
     }
     if(recv_buf[RESPONSE_DATA_IDX] == RESPONSE_DATA) fprintf(stdout, "WHILL wakes up\n");
     else fprintf(stdout, "WHILL didn't wake up\n");
     closeComWHILL(whill_fd);
}
