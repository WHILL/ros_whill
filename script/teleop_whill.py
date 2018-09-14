#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import readchar
from whill.msg import msgWhillSetJoystick


def main():

    rospy.init_node('teleop_whill', anonymous=True)
    rospy.loginfo('teleop_whill: start')

    teleop_pub = rospy.Publisher('whill_setjoystick', msgWhillSetJoystick, queue_size=10)

    rospy.loginfo("teleop command -> forward:'w', backward:'x', left:'a', right:'d', kill:'k'")
    front_val = 10 # TODO to parameter
    back_val = -10
    left_val = -20
    right_val = 20

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        msg = msgWhillSetJoystick()
        msg.front = 0
        msg.side = 0

        key = readchar.readchar()
        if key == 'w':
            msg.front = front_val
        elif key == 'x':
            msg.front = back_val
        elif key == 'a':
            msg.side = left_val
        elif key == 'd':
            msg.side = right_val
        elif key == 'k':
            break

        if msg.front != 0 or msg.side != 0:
            teleop_pub.publish(msg)
            
            r.sleep()

if __name__ == '__main__':
    main()
