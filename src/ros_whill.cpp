/*
MIT License

Copyright (c) 2018 WHILL inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <string.h>
#include <stdint.h>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "nav_msgs/Odometry.h"

#include "libwhill/WHILL.h"   
#include "serial/serial.h"  // wwjwood/Serial (ros-melodic-serial)

// #include "./includes/subscriber.hpp"
// #include "./includes/services.hpp"



//
//  UART Interface
//
serial::Serial *ser = NULL;

int serialRead(std::vector<uint8_t> &data)
{
    if (ser)
    {
        return ser->read(data, 30); // How many bytes read in one time.
    }
    return 0;
}

int serialWrite(std::vector<uint8_t> &data)
{
    if (ser)
    {
        return ser->write(data);
    }
    return 0;
}


//
// WHILL
//
WHILL *whill = NULL;

void callback_data1(WHILL *caller)
{
    // This function is called when receive Joy/Accelerometer/Gyro,etc.
    ROS_INFO("Updated");
    ROS_INFO("%d",caller->joy.x);
    ROS_INFO("%d",caller->joy.y);
}

void callback_powered_on(WHILL *caller)
{
    // This function is called when powered on via setPower()
    ROS_INFO("power_on");
}


//
// Main
//
int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "whill");
    ros::NodeHandle nh("~");

    std::string serialport;
    nh.param<std::string>("serialport", serialport, "/dev/ttyUSB0");

    bool activate_experimental;
    nh.param<bool>("activate_experimental", activate_experimental, false);

    // Node Param
    int send_interval = 10;
    nh.getParam("send_interval", send_interval);
    if (send_interval < 10)
    {
        ROS_WARN("Too short interval. Set interval > 10");
        nh.setParam("/whill_modelc_publisher/send_interval", 10);
        send_interval = 10;
    }
    ROS_INFO("param: send_interval=%d", send_interval);

    std::string port = "/dev/ttyUSB0";
    unsigned long baud = 38400;

    ser = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(0));

    whill = new WHILL(serialRead, serialWrite);
    whill->register_callback(callback_data1, WHILL::EVENT::CALLBACK_DATA1);
    whill->register_callback(callback_powered_on, WHILL::EVENT::CALLBACK_POWER_ON);
    whill->begin(10);

    // // Services
    // ros::ServiceServer set_power_service_service = nh.advertiseService("power/on", set_power_service_callback);
    // //ros::ServiceServer service             = nh.advertiseService("odom/clear", &clearOdom);

    // // SubscriberstransferPacket
    // ros::Subscriber control_joystick_subscriber = nh.subscribe("controller/joy", 100, control_joystick_callback); // Defined in subscriber.cpp

    // // Publishers
    // ros::Publisher joystick_state_publisher = nh.advertise<sensor_msgs::Joy>("states/joy", 100);
    // ros::Publisher jointstate_publisher = nh.advertise<sensor_msgs::JointState>("states/jointState", 100);
    // ros::Publisher imu_publisher = nh.advertise<sensor_msgs::Imu>("states/imu", 100);
    // ros::Publisher battery_state_publisher = nh.advertise<sensor_msgs::BatteryState>("states/batteryState", 100);
    // ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>("odom", 100);

    if (activate_experimental)
    {
        //Services

        //Subscribers
        //ros::Subscriber control_cmd_vel_subscriber = nh.subscribe("controller/cmd_vel", 100, control_cmd_vel_callback);

        // Publishers
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(10);

    while (ros::ok())
    {
        //whill->refresh();
        whill->setJoystick(50, 0);
        // std::vector<uint8_t> received;
        // //size_t length = my_serial.read(received,20);
        // std::cout << length << ", String read: " << std::endl;
        // for (std::vector<uint8_t>::iterator i = begin(received); i != end(received); ++i)
        // {
        //     printf("%02x,", (unsigned int)*i);
        // }
        //ROS_INFO("sleep()");

        rate.sleep();

    }

    spinner.stop();

    return 0;
}
