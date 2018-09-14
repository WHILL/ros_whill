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
#include <unistd.h>

#include "./com_whill.h"

#define INIT_S1  0
#define INIT_FM1 10
#define INIT_FA1 16
#define INIT_FD1 48
#define INIT_RM1 10
#define INIT_RA1 16
#define INIT_RD1 40
#define INIT_TM1 8
#define INIT_TA1 56
#define INIT_TD1 72

void main( int argc, char *argv[] )
{
     int whill_fd; // file descriptor for UART to communicate with WHILL

     initializeComWHILL(&whill_fd);
     int s1  = INIT_S1;
     int fm1 = INIT_FM1;
     int fa1 = INIT_FA1;
     int fd1 = INIT_FD1;
     int rm1 = INIT_RM1;
     int ra1 = INIT_RA1;
     int rd1 = INIT_RD1;
     int tm1 = INIT_TM1;
     int ta1 = INIT_TA1;
     int td1 = INIT_TD1;

     sendSetSpeed(whill_fd, s1, fm1, fa1, fd1, rm1, ra1, rd1, tm1, ta1, td1);
     //sendSetSpeed(whill_fd, 0, 10, 16, 48, 10, 16, 40, 8, 56, 72);
     
     fprintf(stdout, "s1:%d\n"
             "fm1:%d\n"
             "fa1:%d\n"
             "fd1:%d\n"
             "rm1:%d\n"
             "ra1:%d\n"
             "rd1:%d\n"
             "tm1:%d\n"
             "ta1:%d\n"
             "td1:%d\n"
             , s1, fm1, fa1, fd1, rm1, ra1, rd1, tm1, ta1, td1
     );
     usleep(100000); //wait 100ms
     
     closeComWHILL(whill_fd);
}






