#!/usr/bin/env python
import rospy
import time
import roslib
roslib.load_manifest('ros_whill_localization')
import tf
from std_msgs import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from ros_whill_localization.msg import msgWhillModelC 
import numpy as np

WHEEL_TREAD = 0.246
WHEEL_RADIUS = 0.135


INIT_POS_X = 0.0
INIT_POS_Y = 0.0
INIT_POS_TH = 0.0

# at 204
#INIT_POS_X = 16.0 #1.0
#INIT_POS_Y = 19.0 #1.0
#INIT_POS_TH = -np.pi * 1 / 2

#INIT_POS_X = 1.0
#INIT_POS_Y = 2.0 
#INIT_POS_TH = np.pi * 1 / 2

angle_r = 0
angle_l = 0
pre_angle_r = 0
pre_angle_l = 0

x = INIT_POS_X
y = INIT_POS_Y
th = INIT_POS_TH

def calcAngleDiff(angle, pre_angle):
    if angle > 0:
        if pre_angle > 0:
            return (angle - pre_angle)
        else:
            tmp = angle - pre_angle
            if tmp < np.pi:
                return tmp
            else:
                return (-1) * (2 * np.pi - tmp)
    else:
        if pre_angle > 0:
            tmp = pre_angle - angle
            if tmp < np.pi:
                return (-1) * tmp
            else:
                return (2 * np.pi - tmp)
        else:
            return (angle - pre_angle)

def PItoPI(angle):
    while angle >= np.pi:
        angle = angle - 2 * np.pi
    while angle <= -np.pi:
        angle = angle + 2 * np.pi
    return angle

def modelc_msg_callback(msg):
    #print "angle_r l %lf %lf"%(msg.right_motor_angle, msg.left_motor_angle)
     
    global angle_r
    global angle_l

    angle_r = msg.right_motor_angle
    angle_l = msg.left_motor_angle

def talker():
    global pre_angle_r
    global pre_angle_l
    global x
    global y
    global th

    rospy.init_node('odometry_publisher', anonymous=True)

    rospy.Subscriber("whill_modelc_msg", msgWhillModelC, modelc_msg_callback)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    
    vx = 0.0
    vy = 0.0
    vth = 0.0
    count = 0

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    
    r = rospy.Rate(100)

    while not rospy.is_shutdown():
        #rospy.spinonce()
	count = count + 1
        current_time = rospy.Time.now()
        
	#http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/wheelrobot.html
        if pre_angle_r == 0 and pre_angle_l == 0:
            pre_angle_r = angle_r
	    pre_angle_l = angle_l
	    continue
        
        dt = (current_time - last_time).to_sec()
	angle_vel_r = calcAngleDiff(angle_r, pre_angle_r) / dt * (-1)
	angle_vel_l = calcAngleDiff(angle_l, pre_angle_l) / dt
        vr = angle_vel_r * WHEEL_RADIUS
	vl = angle_vel_l * WHEEL_RADIUS

	delta_L = (vr + vl) / 2.0
        delta_th = (vr - vl) / (2.0 * WHEEL_TREAD)
         
        vx = delta_L
        vth = delta_th
 
	th += delta_th * dt
        th = PItoPI(th)
        x += delta_L * dt * np.cos(th)
        y += delta_L * dt * np.sin(th)
	
        if count % 1 == 0:
 	    #print "dt %f"%(dt)
 	    #print "dt a p v R[%02.2f %02.3f %02.3f %02.3f]"%(dt, angle_r, pre_angle_r, angle_vel_r )
            #print "a p v L[%f %f %f]"%(angle_l, pre_angle_l, angle_vel_l )
 	    print "vr vl x y th %f %f %f %f %f"%(vr, vl, x, y, th)

	pre_angle_r = angle_r
        pre_angle_l = angle_l

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    
        odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                #"base_link",
                "base_footprint",
                "odom"
                )

        odom = Odometry()
        odom.header.stamp =  current_time
        odom.header.frame_id = 'odom'

        #set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        #set the velocity
        odom.child_frame_id = "base_footprint";
        #odom.child_frame_id = "base_link";
        odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))

        #publish the message
        odom_pub.publish(odom);

        last_time = current_time
        r.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

