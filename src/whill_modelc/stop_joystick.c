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
#define COUNT (50)
void main( int argc, char *argv[] )
{
     int whill_fd; // file descriptor for UART to communicate with WHILL
     int i, counter;

     initializeComWHILL(&whill_fd);

     /** Send ctrl command to WHILL **/
     //Joststick test
     counter = COUNT;
     while(counter){
	  sendJoystick(whill_fd, 0, 0);
	  usleep(100000); //wait 100ms
	  counter--;
     }
     
     closeComWHILL(whill_fd);
}
