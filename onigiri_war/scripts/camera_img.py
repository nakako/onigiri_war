#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
#import random

#from geometry_msgs.msg import Twist
#from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class Get_camera_img:
      def __init__(self, bot_name):
            print("-- " + self.name + ": Get_camera_img object init --")   #debag print
            print("-- " + self.name + ": Publisher(/camera/image_raw) set --")   #debag print            
            pub = rospy.Publisher('/camera/image_raw', Image, queue_size = 10) #output
            cycle = rospy.Rate(10)  #10hz 10[回/s]

      def image_get(self):
            while not rospy.is_shutdown():



if __name__ == '__main__':
    rospy.init_node('GetCameraImage')     #make node
    bot = Get_camera_img('ume_bot')       #make object
    bot.image_get()
    
