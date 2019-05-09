#pragma once

#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"

class Odometry
{
private:
    long double confineRadian(long double rad);

    typedef struct
    {
        long double x;
        long double y;
        long double theta;
    } Space2D;

    static constexpr double wheel_radius_ = 0.1325;
    static constexpr double wheel_tread_ = 0.248;

    Space2D pose;
    Space2D velocity;

public:
    Odometry();
    void update(sensor_msgs::JointState joint, double dt);
    void set(Space2D pose);
    void reset();

    nav_msgs::Odometry getROSOdometry();
    geometry_msgs::TransformStamped getROSTransformStamped();
    Space2D getOdom();
};
