#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import math

from geometry_msgs.msg import Twist
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class RunBot():
    def __init__(self):
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.speed = 0.1


    def calcTwist(self,linear_x, angular_z):            
        twist = Twist()
        twist.linear.x =  linear_x ; twist.linear.y = 0 ; twist.linear.z = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angular_z;
        self.vel_pub.publish(twist)
        return twist

    def strategy(self):
        r = rospy.Rate(10) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        while not rospy.is_shutdown():
            twist = self.calcTwist()
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('random_rulo')
    bot = RandomBot('Random')
    bot.strategy()

