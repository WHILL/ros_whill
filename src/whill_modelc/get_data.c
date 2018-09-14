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
#include <unistd.h>
#include "./com_whill.h"
#define MAX_EVENTS (10)
#define REPEAT_COUNT (10)

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

void main( int argc, char *argv[] )
{
     int opt, data_set_num = 0;
     int speed_mode = 0; // added on Model C
     struct epoll_event ev;
     struct epoll_event events[MAX_EVENTS];
     int epollfd, nfds;
     int whill_fd; // file descriptor for UART to communicate with WHILL
     char recv_buf[128];
     int len, idx;
     int i, counter = 0;

     while((opt = getopt(argc, argv, "d:")) != -1){
	  switch(opt){
	  case 'd':
	       if(atoi(optarg) < 2){
		    data_set_num = atoi(optarg);
	       }else{
		    fprintf(stderr, "Data set number is 0 or 1\n");
		    return;
	       }
	       break;
	  default:
	       fprintf(stderr, "Usage %s [-d data_set_number]\n", argv[0]);
	       return;
	       break;
	  }
     }

     // added on Model C
     while((opt = getopt(argc, argv, "s:")) != -1){
	  switch(opt){
	  case 's':
	       if(atoi(optarg) < 6){
		    speed_mode = atoi(optarg);
	       }else{
		    fprintf(stderr, "Speed mode is 0 - 5\n");
		    return;
	       }
	       break;
	  default:
	       fprintf(stderr, "Usage %s [-s speed_mode]\n", argv[0]);
	       return;
	       break;
	  }
     }

     initializeComWHILL(&whill_fd);
     if((epollfd = epoll_create1(0)) < 0)
     {
	  fprintf(stderr, "can't creare epoll\n");
	  return;
     }


     /* register Fd to epoll */
     registerFdToEpoll(&ev, epollfd, whill_fd);

     /** send StopSendingData command **/
     sendStopSendingData(whill_fd);
     usleep(2000);
     /** Send StartSendingData command **/
     //sendStartSendingData(whill_fd, 1000, data_set_num); //sending data inverval is  1s
     sendStartSendingData(whill_fd, 1000, data_set_num, speed_mode); // modified on Model C
     usleep(2000);
     while(1){
	  /** Process epoll **/
	  nfds = epoll_wait(epollfd, events, MAX_EVENTS, -1);
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
	  if(counter >= REPEAT_COUNT) break;
	  counter++;
     }
     /** send StopSendingData command **/
     sendStopSendingData(whill_fd);
     usleep(2000);

     closeComWHILL(whill_fd);
}
