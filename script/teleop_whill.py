#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import readchar
from sensor_msgs.msg import Joy


def main():

    rospy.init_node('teleop_whill', anonymous=True)
    rospy.loginfo('teleop_whill: start')

    teleop_pub = rospy.Publisher('/whill/controller/joy', Joy, queue_size=10)

    rospy.loginfo("teleop command -> forward:'w', backward:'x', left:'a', right:'d', kill:'k'")
    front_val = 10 # TODO to parameter
    back_val = -10
    left_val = -20
    right_val = 20

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        msg = Joy()
        msg.axes[0] = 0
        msg.axes[1] = 0

        key = readchar.readchar()
        if key == 'w':
            msg.axes[1] = front_val
        elif key == 'x':
            msg.axes[1] = back_val
        elif key == 'a':
            msg.axes[0] = left_val
        elif key == 'd':
            msg.axes[0] = right_val
        elif key == 'k':
            break

        if msg.axes[0] != 0 or msg.axes[1] != 0:
            teleop_pub.publish(msg)
            
            r.sleep()

if __name__ == '__main__':
    main()
