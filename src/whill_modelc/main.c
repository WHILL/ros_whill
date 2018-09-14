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
#define MAX_EVENTS (10)

int registerFdToEpoll(struct epoll_event *ev, int epollfd, int fd)
{
     /* register socket to epoll */
     ev->events = EPOLLIN;
     ev->data.fd = fd;
     if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, ev) == -1) {
	  fprintf(stderr, "fail epoll_ctl\n");
	  return -1;
     }
     return 1;
}

void main( void )
{
     struct epoll_event ev;
     struct epoll_event events[MAX_EVENTS];
     int epollfd, nfds;
     int whill_fd; // file descriptor for UART to communicate with WHILL
     char recv_buf[128];
     int len, idx;
     int i, counter;


     initializeComWHILL(&whill_fd);
     if((epollfd = epoll_create1(0)) < 0)
     {
	  fprintf(stderr, "can't creare epoll\n");
	  return;
     }


     /** send PowerOn command */
     sendPowerOn(whill_fd);
     usleep(2000);
     len = recvDataWHILL(whill_fd, recv_buf);
     fprintf(stdout, "recv data via UART\n");
     for(idx=0;idx<len;idx++){
       fprintf(stdout, "%d, %d\n", idx, recv_buf[idx]);
     }

     /* register Fd to epoll */
     registerFdToEpoll(&ev, epollfd, whill_fd);

     /** send setForward command **/
     /* removed on Model C
     sendSetForward(whill_fd, 10, 50, 10);
     usleep(1000000); //wait 1s
     sendSetReverse(whill_fd, 70, 50, 10);
     usleep(1000000); //wait 1s
     sendSetTurn(whill_fd, 70, 50, 10);
     usleep(1000000); //wait 1s
     */
     //sendSetSpeed(whill_fd, 0, 8, 1, 1, 8, 1, 1, 8, 1, 1); // added on Model C
     usleep(1000000); //wait 1s

     /** send StopSendingData command **/
     sendStopSendingData(whill_fd);
     usleep(2000);
     /** Send StartSendingData command **/
     //sendStartSendingData(whill_fd, 1000, 1);
     sendStartSendingData(whill_fd, 1000, 1, 0); // modified on Model C
     usleep(2000);
     while(1){
	  /** Process epoll **/
	  nfds = epoll_wait(epollfd, events, MAX_EVENTS, 20); //wait 100msec
	  if (nfds == -1) {
	       fprintf(stderr, "err : epoll wait");
	       return;
	  }

	  for (i = 0; i < nfds; i++) {
               /** Receive Data From WHILL **/
	       if(events[i].data.fd == whill_fd) {
		    len = recvDataWHILL(whill_fd, recv_buf);
		    fprintf(stdout, "recv data via UART\n");
		    for(idx=0;idx<len;idx++){
		      fprintf(stdout, "%d, %d\n", idx, recv_buf[idx]);
		    }
	       }
	  }
	  
	  /** Send ctrl command to WHILL **/
	  //Joststick test
	  sendJoystick(whill_fd, -50, 40);
	  //Seat move test
	  //sendSeatMove(whill_fd, 1);
     }
     closeComWHILL(whill_fd);
}
