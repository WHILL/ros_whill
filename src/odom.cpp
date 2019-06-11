/*
MIT License
Copyright (c) 2018-2019 WHILL inc.
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

#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_broadcaster.h"

#include "./odom.h"

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
#include <limits>

const float base_link_height = 0.1325;

Odometry::Odometry()
{
    pose.x = pose.y = pose.theta = 0.0;
    velocity.x = velocity.y = velocity.theta = 0.0;
}

long double Odometry::confineRadian(long double rad)
{
    if (rad >= M_PI)
    {
        rad -= 2.0 * M_PI;
    }
    if (rad <= -M_PI)
    {
        rad += 2.0 * M_PI;
    }
    return rad;
}

void Odometry::setParameters(double _wheel_radius, double _wheel_tread){
    this->wheel_radius = _wheel_radius;
    this->wheel_tread = _wheel_tread;
}

void Odometry::update(sensor_msgs::JointState jointState, double dt)
{
    if (dt == 0)
        return;

    double angle_vel_r = jointState.velocity[1];
    double angle_vel_l = -jointState.velocity[0];

    long double vr = angle_vel_r * wheel_radius;
    long double vl = angle_vel_l * wheel_radius;

    long double delta_L = (vr + vl) / 2.0;
    long double delta_theta = (vr - vl) / (wheel_tread);

    pose.x += delta_L * dt * cosl(pose.theta + delta_theta * dt / 2.0);
    pose.y += delta_L * dt * sinl(pose.theta + delta_theta * dt / 2.0);

    velocity.x = delta_L;
    velocity.y = 0.0;
    velocity.theta = delta_theta;

    double theta = pose.theta + delta_theta * dt;
    pose.theta = confineRadian(theta);

    return;
}

void Odometry::zeroVelocity(void){
    velocity.x = 0;
    velocity.y = 0;
    velocity.theta = 0;
    return;
}

void Odometry::reset()
{
    Space2D poseZero = {0, 0, 0};
    set(poseZero);
    velocity = poseZero;
}

void Odometry::set(Space2D pose)
{
    this->pose = pose;
}

Odometry::Space2D Odometry::getOdom()
{
    return pose;
}

nav_msgs::Odometry Odometry::getROSOdometry()
{
    nav_msgs::Odometry odom;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.theta);

    // position
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = base_link_height;
    odom.pose.pose.orientation = odom_quat;

    //velocity
    odom.twist.twist.linear.x = velocity.x;
    odom.twist.twist.linear.y = velocity.y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = velocity.theta;

    return odom;
}

geometry_msgs::TransformStamped Odometry::getROSTransformStamped()
{

    geometry_msgs::TransformStamped odom_trans;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.theta);

    odom_trans.transform.translation.x = pose.x;
    odom_trans.transform.translation.y = pose.y;
    odom_trans.transform.translation.z = base_link_height;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pose.theta);

    return odom_trans;
}