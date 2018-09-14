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
#define COUNT (100)
void main( int argc, char *argv[] )
{
     int whill_fd; // file descriptor for UART to communicate with WHILL
     int fb,lr;
     int i, counter;

     if(argc < 3){
	  fprintf(stderr,"Usage %s [Forward(Reverse) joystick value] [Right(Left) joystick value]\n", argv[0]);
	  return;
     }
     fb = atoi(argv[1]);
     lr = atoi(argv[2]);
     if(fb > 100) fb = 100;
     if(fb < -100) fb = -100;
     if(lr > 100) lr = 100;
     if(lr < -100) lr = -100;

     initializeComWHILL(&whill_fd);

     /** Send ctrl command to WHILL **/
     //Joststick test
     counter = COUNT;
     while(counter){
	  sendJoystick(whill_fd, fb, lr);
	  usleep(100000); //wait 100ms
	  counter--;
     }
     
     closeComWHILL(whill_fd);
}
