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
#include <iostream>
#include <stdint.h>
#include <vector>

#include "unistd.h"

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/BatteryState.h"
#include "nav_msgs/Odometry.h"

#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"

#include "ros_whill/SpeedPack.h"
#include "ros_whill/SetSpeedProfile.h"

#include "tf/transform_broadcaster.h"

#include "whill/WHILL.h"
#include "serial/serial.h" // wjwood/Serial (ros-melodic-serial)

#include "utils/rotation_tools.h"
#include "utils/unit_convert.h"
#include "odom.h"

// #include "./includes/subscriber.hpp"
// #include "./includes/services.hpp"

WHILL *whill = nullptr;
Odometry odom;

// Global Parameters
int interval = 0;       // WHILL Data frequency
bool publish_tf = true; // Enable publishing Odometry TF
ros::Time last_received;

template <typename T>
void safeDelete(T *&p)
{
    if (p != NULL)
    {
        delete (p);
        (p) = NULL;
    }
}
//
// ROS Objects
//

// Publishers
ros::Publisher ros_joystick_state_publisher;
ros::Publisher ros_jointstate_publisher;
ros::Publisher ros_battery_state_publisher;
ros::Publisher ros_odom_publisher;

// TF Broadcaster
tf::TransformBroadcaster *odom_broadcaster = nullptr;

//
// ROS Callbacks
//
void ros_joystick_callback(const sensor_msgs::Joy::ConstPtr &joy)
{
    // Transform [-1.0,1.0] to [-100,100]
    int joy_x = -joy->axes[0] * 100.0f;
    int joy_y = joy->axes[1] * 100.0f;

    // value check
    if (joy_y < -100)
        joy_y = -100;
    if (joy_y > 100)
        joy_y = 100;
    if (joy_x < -100)
        joy_x = -100;
    if (joy_x > 100)
        joy_x = 100;

    if (whill)
    {
        whill->setJoystick(joy_x, joy_y);
    }
}

void ros_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    if (whill)
    {
        whill->setSpeed(cmd_vel->linear.x, cmd_vel->angular.z);
    }
}

bool ros_srv_set_power(std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res){
    if (whill == nullptr)
    {
        res.success = false;
        res.message = "whill instance is not initialzied.";
        return true;
    }

    whill->setPower(req.data);
    res.success = true;
    return true;
}

bool ros_srv_odom_clear_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Clear Odometry");
    odom.reset();
    return true;
}

bool ros_srv_set_speed_profile(ros_whill::SetSpeedProfile::Request &req, ros_whill::SetSpeedProfile::Response &res)
{

    if (whill == nullptr)
    {
        res.success = false;
        res.status_message = "whill instance is not initialzied.";
        return true;
    }

    WHILL::SpeedProfile profile;

    profile.forward.speed = convert_mps_to_whill_speed(req.forward.speed);
    profile.forward.acc = convert_mpss_to_whill_acc(req.forward.acc);
    profile.forward.dec = convert_mpss_to_whill_acc(req.forward.dec);
    profile.backward.speed = convert_mps_to_whill_speed(req.backward.speed);
    profile.backward.acc = convert_mpss_to_whill_acc(req.backward.acc);
    profile.backward.dec = convert_mpss_to_whill_acc(req.backward.dec);
    profile.turn.speed = convert_radps_to_whill_speed(whill->tread, req.turn.speed);
    profile.turn.acc = convert_radpss_to_whill_acc(whill->tread, req.turn.acc);
    profile.turn.dec = convert_radpss_to_whill_acc(whill->tread, req.turn.dec);

    ROS_INFO("Setting Speed Profile");
    ROS_INFO("Forward\tSpeed:%d,Acc:%d,Dec:%d", profile.forward.speed, profile.forward.acc, profile.forward.dec);
    ROS_INFO("Bacward\tSpeed:%d,Acc:%d,Dec:%d", profile.backward.speed, profile.backward.acc, profile.backward.dec);
    ROS_INFO("Turn\tSpeed:%d,Acc:%d,Dec:%d\n", profile.turn.speed, profile.turn.acc, profile.turn.dec);

    // Validate Value
    bool is_valid = false;
    auto error = profile.check();
    switch (error)
    {
    case WHILL::SpeedProfile::Error::InvalidForwardSpeed:
        res.status_message = "Invalid Forward Speed";
        break;
    case WHILL::SpeedProfile::Error::InvalidBackwardSpeed:
        res.status_message = "Invalid Backward Speed";
        break;
    case WHILL::SpeedProfile::Error::InvalidTurnSpeed:
        res.status_message = "Invalid Turn Speed";
        break;
    case WHILL::SpeedProfile::Error::InvalidForwardAcc:
        res.status_message = "Invalid Forward Acc";
        break;
    case WHILL::SpeedProfile::Error::InvalidBackwardAcc:
        res.status_message = "Invalid Backward Acc";
        break;
    case WHILL::SpeedProfile::Error::InvalidTurnAcc:
        res.status_message = "Invalid Turn Acc";
        break;
    case WHILL::SpeedProfile::Error::InvalidForwardDec:
        res.status_message = "Invalid Forward Dec";
        break;
    case WHILL::SpeedProfile::Error::InvalidBackwardDec:
        res.status_message = "Invalid Backward Dec";
        break;
    case WHILL::SpeedProfile::Error::InvalidTurnDec:
        res.status_message = "Invalid Turn Dec";
        break;
    default:
        is_valid = true;
    }

    if (!is_valid)
    {
        ROS_WARN("SpeedProfile Service has been called with invalid speed profile parameters.");
        res.success = false;
        return true;
    }

    if (whill->setSpeedProfile(profile, 4))
    {
        res.success = true;
        res.status_message = "Set Speed Profile command has been sent.";
    }
    else
    {
        res.success = false;
        res.status_message = "Invalid Value.";
    }

    return true;
}

//
//  UART Interface
//
serial::Serial *ser = nullptr;

int serialRead(std::vector<uint8_t> &data)
{
    try{
        if (ser && ser->isOpen())
        {
            int ret = ser->read(data, 30);
            return ret; // How many bytes read in one time.
        }
    }catch(...){
        if(ser){
            ser->close();
            ROS_WARN("Port closed due to exception.");
        }
    }
    return 0;
}

int serialWrite(std::vector<uint8_t> &data)
{
    try{
        if (ser && ser->isOpen())
        {
            return ser->write(data);
        }
    }catch(...){
        if(ser){
            ser->close();
            ROS_WARN("Port closed due to exception.");
        }
    }
    return 0;
}

void sleep_ms(uint32_t ms){
    usleep(ms * 1000);
    return;
}

//
// WHILL
//

// Enable cmd_vel Topic
bool enable_cmd_vel_topic = false;
ros::Subscriber cmd_vel_subscriber;
void activate_cmd_vel_topic(ros::NodeHandle &nh)
{
    static bool activated = false;
    if (!activated && enable_cmd_vel_topic)
    {
        cmd_vel_subscriber = nh.subscribe("controller/cmd_vel", 100, ros_cmd_vel_callback);
        activated = true;
    }
}

void whill_callback_data1(WHILL *caller)
{

    // This function is called when receive Joy/Accelerometer/Gyro,etc.

    ros::Time currentTime = ros::Time::now();

    // Joy
    sensor_msgs::Joy joy;
    joy.header.stamp = currentTime;
    joy.axes.resize(2);
    joy.axes[0] = -caller->joy.x / 100.0f; //X
    joy.axes[1] = caller->joy.y / 100.0f;  //Y
    ros_joystick_state_publisher.publish(joy);

    // Battery
    sensor_msgs::BatteryState batteryState;
    batteryState.header.stamp = currentTime;
    batteryState.voltage = 25.2;                               //[V] Spec voltage, since raw voltage is not provided.
    batteryState.current = -caller->battery.current / 1000.0f; // mA -> A
    batteryState.charge = std::numeric_limits<float>::quiet_NaN();
    batteryState.design_capacity = 10.04;                     //[Ah]
    batteryState.percentage = caller->battery.level / 100.0f; // Percentage
    batteryState.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batteryState.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batteryState.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batteryState.present = true;
    batteryState.location = "0";
    ros_battery_state_publisher.publish(batteryState);

    // JointState
    sensor_msgs::JointState jointState;
    jointState.header.stamp = currentTime;
    jointState.name.resize(2);
    jointState.position.resize(2);
    jointState.velocity.resize(2);

    jointState.name[0] = "leftWheel";
    jointState.position[0] = caller->left_motor.angle; //Rad
    jointState.name[1] = "rightWheel";
    jointState.position[1] = caller->right_motor.angle; //Rad

    static double joint_past[2] = {0.0f, 0.0f};
    if (caller->_interval == -1)
    {
        // Standard, Constant specified time intervel
        jointState.velocity[0] = rad_diff(joint_past[0], jointState.position[0]) / (double(interval) / 1000.0f); // Rad/sec
        jointState.velocity[1] = rad_diff(joint_past[1], jointState.position[1]) / (double(interval) / 1000.0f); // Rad/sec
    }
    else if (caller->_interval == 0)
    {
        // Experimental, Motor Control Disabled (= Brake Locked)
        jointState.velocity[0] = 0.0f;
        jointState.velocity[1] = 0.0f;
    }
    else
    {
        // Experimental, Under motor controlling
        jointState.velocity[0] = rad_diff(joint_past[0], jointState.position[0]) / (double(caller->_interval) / 1000.0f); // Rad/sec
        jointState.velocity[1] = rad_diff(joint_past[1], jointState.position[1]) / (double(caller->_interval) / 1000.0f); // Rad/sec
    }
    joint_past[0] = jointState.position[0];
    joint_past[1] = jointState.position[1];

    ros_jointstate_publisher.publish(jointState);

    // Odometory
    if (caller->_interval == -1)
    {
        // Standard
        odom.update(jointState, interval / 1000.0f);
    }
    else if (caller->_interval >= 0)
    {
        enable_cmd_vel_topic = true;
        // Experimental
        if(caller->_interval == 0){
            odom.zeroVelocity();
        }else{
            odom.update(jointState, caller->_interval / 1000.0f);
        }
    }

    nav_msgs::Odometry odom_msg = odom.getROSOdometry();
    odom_msg.header.stamp = currentTime;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    ros_odom_publisher.publish(odom_msg);

    // Odometory TF
    if (publish_tf)
    {
        geometry_msgs::TransformStamped odom_trans = odom.getROSTransformStamped();
        odom_trans.header.stamp = currentTime;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        if (odom_broadcaster)
        {
            odom_broadcaster->sendTransform(odom_trans);
        }
    }

    last_received = ros::Time::now();
}

void whill_callback_powered_on(WHILL *caller)
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

    // Services
    //set_power_service_service = nh.advertiseService("power/on", set_power_service_callback);
    ros::ServiceServer clear_odom_service = nh.advertiseService("odom/clear", &ros_srv_odom_clear_callback);
    ros::ServiceServer set_speed_profile_service = nh.advertiseService("speedProfile/set", &ros_srv_set_speed_profile);
    ros::ServiceServer set_power_service = nh.advertiseService("power", &ros_srv_set_power);

    // Subscriber
    ros::Subscriber joystick_subscriber = nh.subscribe("controller/joy", 100, ros_joystick_callback);

    // Publishers
    ros_joystick_state_publisher = nh.advertise<sensor_msgs::Joy>("states/joy", 100);
    ros_jointstate_publisher = nh.advertise<sensor_msgs::JointState>("states/jointState", 100);
    ros_battery_state_publisher = nh.advertise<sensor_msgs::BatteryState>("states/batteryState", 100);
    ros_odom_publisher = nh.advertise<nav_msgs::Odometry>("odom", 100);

    // TF Broadcaster
    odom_broadcaster = new tf::TransformBroadcaster;

    // Parameters
    // WHILL Report Packet Interval
    nh.getParam("send_interval", interval);
    if (interval < 10)
    {
        ROS_WARN("Too short interval. Set interval > 10");
        nh.setParam("send_interval", 10);
        interval = 10;
    }
    ROS_INFO("param: send_interval=%d", interval);

    // Serial Port Device Name
    std::string serialport;
    nh.param<std::string>("serialport", serialport, "/dev/ttyUSB0");

    // Disable publishing odometry tf
    nh.param<bool>("publish_tf", publish_tf, true);


    bool keep_connected;
    nh.param<bool>("keep_connected", keep_connected, false);

    unsigned long baud = 38400;
    serial::Timeout timeout = serial::Timeout::simpleTimeout(0);
    timeout.write_timeout_multiplier = 5; // Wait 5ms every bytes

    ros::AsyncSpinner spinner(1);
    spinner.start();



    while (ros::ok())
    {
        last_received = ros::Time::now();
        std::cout << "\n Port Opening." << std::flush;

        while (ros::ok())
        {
            try
            {
                ser = new serial::Serial(serialport, baud, timeout);
                break;
            }
            catch (...)
            {
                safeDelete(ser);
                ser = nullptr;
                ros::Duration(1.0).sleep();
                std::cout << "." << std::flush;
            }
        }
        if(!ros::ok())break;

        ROS_INFO("Opened.");
        ser->flush();

        whill = new WHILL(serialRead, serialWrite, sleep_ms);
        whill->setPower(true);
        whill->stopSendingData();

        odom.reset();
        odom.setParameters(whill->wheel_radius, whill->tread);

        whill->register_callback(whill_callback_data1, WHILL::EVENT::CALLBACK_DATA1);
        whill->register_callback(whill_callback_powered_on, WHILL::EVENT::CALLBACK_POWER_ON);


        // Initial Speed Profile
        ros_whill::SetSpeedProfile::Request init_speed_req;
        if (nh.getParam("init_speed/forward/speed", init_speed_req.forward.speed) &&
            nh.getParam("init_speed/forward/acc", init_speed_req.forward.acc) &&
            nh.getParam("init_speed/forward/dec", init_speed_req.forward.dec) &&
            nh.getParam("init_speed/backward/speed", init_speed_req.backward.speed) &&
            nh.getParam("init_speed/backward/acc", init_speed_req.backward.acc) &&
            nh.getParam("init_speed/backward/dec", init_speed_req.backward.dec) &&
            nh.getParam("init_speed/turn/speed", init_speed_req.turn.speed) &&
            nh.getParam("init_speed/turn/acc", init_speed_req.turn.acc) &&
            nh.getParam("init_speed/turn/dec", init_speed_req.turn.dec)
        )
        {
            ros_whill::SetSpeedProfile::Response res;
            ros_srv_set_speed_profile(init_speed_req,res);
            if(res.success == false){
                ROS_INFO("Could not set Initial Profile.");
            }
        }

        sleep_ms(10);
        whill->begin(20); // ms

        ros::Rate rate(100);

        while (ros::ok())
        {
            whill->refresh();
            activate_cmd_vel_topic(nh);
            rate.sleep();
            if (keep_connected && (abs((last_received - ros::Time::now()).toSec()) > 2.0))
            {
                ROS_INFO("Disconnect due to no longer packets received.");
                ROS_WARN("Check your serial connection to WHILL.");
                break;
            }
        }

        ser->close();
        safeDelete(ser);
        safeDelete(whill);
    }

    spinner.stop();

    return 0;
}
