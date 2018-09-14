#include "ros/ros.h"
#include "ros_whill/srvSetSpeedProfile.h"
#include "ros_whill/srvSetPower.h"
#include "ros_whill/srvSetBatteryVoltageOut.h"
#include "ros_whill/msgWhillSetJoystick.h"
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
#include <time.h>
#include <sys/time.h>
#include <math.h>

#define POWERON_RESPONSE_DATA_IDX (0)
#define POWERON_RESPONSE_DATA (0x52)

int whill_fd; // file descriptor for UART to communicate with WHILL

// Set Speed Profile
bool set_speed_profile_srv(ros_whill::srvSetSpeedProfile::Request &req, ros_whill::srvSetSpeedProfile::Response &res)
{

	// value check
	if(0 <= req.s1 && req.s1 <= 5
		&& 8 <= req.fm1 && req.fm1 <= 60
		&& 10 <= req.fa1 && req.fa1 <= 90
		&& 10 <= req.fd1 && req.fd1 <= 160
		&& 8 <= req.rm1 && req.rm1 <= 60
		&& 10 <= req.ra1 && req.ra1 <= 90
		&& 10 <= req.rd1 && req.rd1 <= 160
		&& 8 <= req.tm1 && req.tm1 <= 60
		&& 10 <= req.ta1 && req.ta1 <= 90
		&& 10 <= req.td1 && req.td1 <= 160)
	{
		ROS_INFO("Speed profile is set");
		sendSetSpeed(whill_fd, req.s1, req.fm1, req.fa1, req.fd1, req.rm1, req.ra1, req.rd1, req.tm1, req.ta1, req.td1);
		
		res.result = 1;
		return true;
	}
	else
	{
		ROS_WARN("wrong parameter is assigned.");
		ROS_WARN("s1 must be assingned between 0 - 5");
		ROS_WARN("fm1 must be assingned between 8 - 60");
		ROS_WARN("fa1 must be assingned between 10 - 90");
		ROS_WARN("fd1 must be assingned between 10 - 160");
		ROS_WARN("rm1 must be assingned between 8 - 60");
		ROS_WARN("ra1 must be assingned between 10 - 90");
		ROS_WARN("rd1 must be assingned between 10 - 160");
		ROS_WARN("tm1 must be assingned between 8 - 60");
		ROS_WARN("ta1 must be assingned between 10 - 90");
		ROS_WARN("td1 must be assingned between 10 - 160");
		res.result = -1;
		return false;
	}	
}


// Set Power
bool set_power_srv(ros_whill::srvSetPower::Request &req, ros_whill::srvSetPower::Response &res)
{
	// power off
	if(req.p0 == 0)
	{
		sendPowerOff(whill_fd);
		ROS_INFO("WHILL wakes down\n");
		res.result = 1;
		return true;
	}
	// power on
	else if (req.p0 == 1)
	{
		char recv_buf[128];
		int len;

		sendPowerOn(whill_fd);
		usleep(2000);
		len = recvDataWHILL(whill_fd, recv_buf);
		
		// success
		if(recv_buf[POWERON_RESPONSE_DATA_IDX] == POWERON_RESPONSE_DATA)
		{
			ROS_INFO("WHILL wakes up");
			res.result = 1;
			return true;
		}
		// fail
		else
		{
			ROS_WARN("WHILL didn't wake up");
			res.result = -1;
			return false;
		}
		
	}
	else
	{
		ROS_WARN("p0 must be assinged 0 or 1");
		res.result = -1;
		return false;
	}

}


// Set Battery Voltage out
bool set_battery_voltage_out_srv(ros_whill::srvSetBatteryVoltageOut::Request &req, ros_whill::srvSetBatteryVoltageOut::Response &res)
{
	if(req.v0 == 0 || req.v0 == 1)
	{
		sendSetBatteryOut(whill_fd, req.v0);
		if(req.v0 == 0) ROS_INFO("battery voltage out: disable");
		if(req.v0 == 1) ROS_INFO("battery voltage out: enable");
		res.result = 1;
		return true;
	}
	else
	{
		ROS_INFO("v0 must be assigned 0 or 1");
		res.result = -1;
		return false;
	}

}


// Set Joystick
void whillSetjoystickMsgCallback(const ros_whill::msgWhillSetJoystick::ConstPtr& msg)
{
	int joy_front = msg->front;
	int joy_side = msg->side;

	// value check
	if(joy_front < -100) joy_front = -100;
	if(joy_front > 100)  joy_front = 100;
	if(joy_side < -100) joy_side = -100;
	if(joy_side > 100)  joy_side = 100;

	sendJoystick(whill_fd, joy_front, joy_side);

}


int main(int argc, char **argv)
{
	// ROS setup
	ros::init(argc, argv, "whill_modelc_controller");
	ros::NodeHandle nh;

	ros::ServiceServer set_speed_profile_svr = nh.advertiseService("set_speed_profile_srv", set_speed_profile_srv);
	ros::ServiceServer set_power_svr = nh.advertiseService("set_power_srv", set_power_srv);
	ros::ServiceServer set_battery_voltage_out_svr = nh.advertiseService("set_battery_voltage_out_srv", set_battery_voltage_out_srv);

	ros::Subscriber whill_setjoystick_sub = nh.subscribe("whill_setjoystick", 100, whillSetjoystickMsgCallback);

	initializeComWHILL(&whill_fd);

	ros::spin();

	closeComWHILL(whill_fd);

	return 0;
}


