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

void main( int argc, char *argv[] )
{
     int whill_fd; // file descriptor for UART to communicate with WHILL
     int bo;

     if(argc < 2){
	  fprintf(stderr,"Usage %s [Battery out value 0/1 (disable/enable)]\n", argv[0]);
	  return;
     }
     bo = atoi(argv[1]);
     if(bo != 0 && bo != 1) bo = 0;

     initializeComWHILL(&whill_fd);

     /** Send ctrl command to WHILL **/
     sendSetBatteryOut(whill_fd, bo);  
     
     closeComWHILL(whill_fd);
}
