#include "ros/ros.h"
#include "ros_whill/msgWhillModelC.h"
#include "ros_whill/msgWhillSpeedProfile.h"
#include "whill_modelc/com_whill.h"

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
#include <math.h>

#define MAX_EVENTS (10)
#define DATASET_NUM_ZERO (0)
#define DATASET_NUM_ONE (1)
#define SPEED_MODE (0)
#define SEND_INTERVAL (20)

#define ACC_CONST (0.000122)
#define GYR_CONST (0.004375)
#define MANGLE_CONST (0.001)
#define MSPEED_CONST (0.004)

int registerFdToEpoll(struct epoll_event *ev, int epollfd, int fd)
{
	/* register socket to epoll */
	ev->events = EPOLLIN;
	ev->data.fd = fd;
	if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, ev) == -1) {
		ROS_INFO("fail epoll_ctl\n");
		return -1;
	}
	return 1;
}

float calc_16bit_signed_data(char d1, char d2)
{
	float d = 0;
	d = float(((d1 & 0xff) << 8) | (d2 & 0xff));
	if(d > pow(2, 15)){
		d = d - pow(2, 16);
	}

	return d;
}

float calc_8bit_signed_data(char d1)
{
	float d = 0;
	d = float(d1 & 0xff);
	if(d > pow(2, 7)){
		d = d - pow(2, 8);
	}

	return d;
}

int main(int argc, char **argv)
{
	// ROS setup
	ros::init(argc, argv, "whill_modelc_publisher");
	ros::NodeHandle nh;
	ros::Publisher whill_modelc_pub = nh.advertise<ros_whill::msgWhillModelC>("whill_modelc_msg", 100);
	ros::Publisher whill_speed_profile_pub = nh.advertise<ros_whill::msgWhillSpeedProfile>("whill_speed_profile_msg", 10);

	// Node Param
	int send_interval = SEND_INTERVAL;
	nh.getParam("/whill_modelc_publisher/send_interval", send_interval);
	if(send_interval < 20){
		ROS_WARN("Too short interval. Set interval > 20");
		nh.setParam("/whill_modelc_publisher/send_interval", 20);
		send_interval = 20;
	}
	ROS_INFO("param: send_interval=%d", send_interval);

	// WHILL setup
	struct epoll_event ev;
	struct epoll_event events[MAX_EVENTS];
	int epollfd, nfds;
	int whill_fd; // file descriptor for UART to communicate with WHILL
	char recv_buf[128];
	int len, idx;
	int i;

	initializeComWHILL(&whill_fd);
	if((epollfd = epoll_create1(0)) < 0)
	{
		ROS_INFO("can't creare epoll\n");
		return -1;
	}

	// register Fd to epoll
	registerFdToEpoll(&ev, epollfd, whill_fd);

	// Send StartSendingData command: dataset 0 for all speed mode
	for(int i = 0; i < 6; i ++)
	{
		sendStopSendingData(whill_fd);
		usleep(2000);
		sendStartSendingData(whill_fd, 25, DATASET_NUM_ZERO, i);
		usleep(2000);
		len = recvDataWHILL(whill_fd, recv_buf);
		if(recv_buf[0] == DATASET_NUM_ZERO && len == 12)
		{
			ros_whill::msgWhillSpeedProfile msg_sp;
			msg_sp.s1  = int(recv_buf[1] & 0xff);
			msg_sp.fm1 = int(recv_buf[2] & 0xff);
			msg_sp.fa1 = int(recv_buf[3] & 0xff);
			msg_sp.fd1 = int(recv_buf[4] & 0xff);
			msg_sp.rm1 = int(recv_buf[5] & 0xff);
			msg_sp.ra1 = int(recv_buf[6] & 0xff);
			msg_sp.rd1 = int(recv_buf[7] & 0xff);
			msg_sp.tm1 = int(recv_buf[8] & 0xff);
			msg_sp.ta1 = int(recv_buf[9] & 0xff);
			msg_sp.td1 = int(recv_buf[10] & 0xff);
			whill_speed_profile_pub.publish(msg_sp);
			ROS_INFO("Speed profile %ld is published", msg_sp.s1);
		}
	}


	// send StopSendingData command
	sendStopSendingData(whill_fd);
	usleep(2000);
	// Send StartSendingData command: dataset 1
	sendStartSendingData(whill_fd, send_interval, DATASET_NUM_ONE, SPEED_MODE);
	usleep(2000);

	// loop
	int msg_cnt = 0;
	ROS_INFO("WHILL message sending start");
	while(ros::ok())
 	{
		ROS_INFO("WHILL message %d is published", msg_cnt ++);
		ros_whill::msgWhillModelC msg;

		// WHILL data
		// Process epoll
		nfds = epoll_wait(epollfd, events, MAX_EVENTS, -1);
		if (nfds == -1) {
			ROS_INFO("err : epoll wait");
			break;
		}
		
		for (i = 0; i < nfds; i++) {
			// Receive Data From WHILL
			if(events[i].data.fd == whill_fd) {
				len = recvDataWHILL(whill_fd, recv_buf);
				if(recv_buf[0] == DATASET_NUM_ONE && len == 30)
				{
					msg.acc_x = calc_16bit_signed_data(recv_buf[1], recv_buf[2]) * ACC_CONST;
					msg.acc_y = calc_16bit_signed_data(recv_buf[3], recv_buf[4]) * ACC_CONST;
					msg.acc_z = calc_16bit_signed_data(recv_buf[5], recv_buf[6]) * ACC_CONST;
					msg.gyr_x = calc_16bit_signed_data(recv_buf[7], recv_buf[8]) * GYR_CONST;
					msg.gyr_y = calc_16bit_signed_data(recv_buf[9], recv_buf[10]) * GYR_CONST;
					msg.gyr_z = calc_16bit_signed_data(recv_buf[11], recv_buf[12]) * GYR_CONST;
					msg.joy_front = (int)calc_8bit_signed_data(recv_buf[13]);
					msg.joy_side = (int)calc_8bit_signed_data(recv_buf[14]);
					msg.battery_power = int(recv_buf[15] & 0xff);
					msg.battery_current = calc_16bit_signed_data(recv_buf[16], recv_buf[17]); //TODO -> fixed point
					msg.right_motor_angle = calc_16bit_signed_data(recv_buf[18], recv_buf[19]) * MANGLE_CONST;
					msg.left_motor_angle = calc_16bit_signed_data(recv_buf[20], recv_buf[21]) * MANGLE_CONST;
					msg.right_motor_speed = calc_16bit_signed_data(recv_buf[22], recv_buf[23]) * MSPEED_CONST;
					msg.left_motor_speed = calc_16bit_signed_data(recv_buf[24], recv_buf[25]) * MSPEED_CONST;
					msg.power_on = int(recv_buf[26] & 0xff);
					msg.speed_mode_indicator = int(recv_buf[27] & 0xff);
					msg.error = int(recv_buf[28] & 0xff);
					if(msg.error != 0)
					{
						ROS_WARN("WHILL sends error message. error id: %d", msg.error);
					}
				}
			}
		}
		
		// publish
		whill_modelc_pub.publish(msg);
		
	}

	// send StopSendingData command
	sendStopSendingData(whill_fd);
	usleep(2000);
	closeComWHILL(whill_fd);
	return 0;
}


