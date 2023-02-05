#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


class VelPub:
    def __init__(self):
        print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        self.velocity_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    def robutler_move(self, lx=0.0, az=0.0):
        # Starts a new node
        print("hjdghashjdkjhgsadfgashjgkdfgashjkd")
        vel_msg = Twist()

        # Receiveing the user's input
        # Since we are moving just in x-axis
        vel_msg.linear.x = lx
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = az

        self.velocity_publisher.publish(vel_msg)
