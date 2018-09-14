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



Odometry::Odometry(){
    pose.x = pose.y = pose.theta = 0.0;
    velocity.x = velocity.y = velocity.theta = 0.0;
}


long double Odometry::confineRadian(long double rad){
    if(rad >= M_PI){
        rad -= 2.0 * M_PI;
    }
    if(rad <= -M_PI){
        rad += 2.0 * M_PI;
    }
    return rad;
}


void Odometry::update(sensor_msgs::JointState jointState,double dt)
{
    if(dt == 0)return;

    double angle_vel_r = jointState.velocity[1];
    double angle_vel_l = -jointState.velocity[0];

    long double vr = angle_vel_r * wheel_radius_;
    long double vl = angle_vel_l * wheel_radius_;

    long double delta_L  = (vr + vl) / 2.0;
    long double delta_theta = (vr - vl) / (2.0 * wheel_tread_);


    pose.x += delta_L * dt * cosl(pose.theta + delta_theta * dt / 2.0);
    pose.y += delta_L * dt * sinl(pose.theta + delta_theta * dt / 2.0);

    velocity.x = delta_L;
    velocity.y = 0.0;
    velocity.theta = delta_theta;

    double theta = pose.theta + delta_theta * dt;
    pose.theta = confineRadian(theta);

    return;
}


void Odometry::reset(){
    Space2D poseZero = {0,0,0};
    set(poseZero);
    velocity = poseZero;
}

void Odometry::set(Space2D pose){
    this->pose = pose;
}

Odometry::Space2D Odometry::getOdom(){
    return pose;
}

nav_msgs::Odometry Odometry::getROSOdometry(){
    nav_msgs::Odometry odom;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.theta);

    // position
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;
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



geometry_msgs::TransformStamped Odometry::getROSTransformStamped(){

    geometry_msgs::TransformStamped odom_trans;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.theta);

    odom_trans.transform.translation.x = pose.x;
    odom_trans.transform.translation.y = pose.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pose.theta);

    return  odom_trans;
}
