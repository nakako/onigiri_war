#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
#import random

#from geometry_msgs.msg import Twist
#from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from time import sleep
from Run import *


class Color_tracking:
  def __init__(self, bot_name):
    #bot nameprint
    self.name = bot_name
    print("-- " + self.name + ": Color_tracking object init --")   #debag print
            
    #走行用
    self.run = RunBot()

    #カメラで取った画像(ROS Message)を、CvBridgeを通して、OpenCVの値に変更する
    self.bridge = CvBridge()  
    self.image_sub = rospy.Subscriber('camera/image_raw', Image, self.image_callback) #input setting
    print("-- " + self.name + ": Subscriber set --")   #debag print
    


  #カメラ値取得    
  def image_callback(self, msg):
    print("-- " + self.name + ": into image_callback --" )   #debag print

    #ROS Message -> BRG(OpenCVで扱える形)
    try:
      bgr_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
      print("e")

    #BGR -> HSV
    hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

    #色を抽出するための閾値を設定
    green_lower = np.array([60, 50, 0]) #enemy
    green_upper = np.array([170, 100, 100])
    #blue_lower = np.array([110,50,50])
    #blue_upper = np.array([130,255,255])
    blue_lower = np.array([180,50,0]) #food
    blue_upper = np.array([260,100,100])
         
    #色抽出
    blue_msk = cv2.inRange(hsv_img, blue_lower, blue_upper)
             
    #マスク画像の抽出
    blue_res = cv2.bitwise_and(bgr_img, bgr_img, mask = blue_msk)

    #2値化
    blue_binary = cv2.threshold(blue_msk, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

    #ラベリング処理
    blue_label = cv2.connectedComponentsWithStats(blue_binary)

    #ラベリング情報を項目別に抽出
    n = blue_label[0] - 1                    #個数
    data = np.delete(blue_label[2], 0, 0)    #座標, 幅, 高さ
    center = np.delete(blue_label[3], 0, 0)  #中心座標

    #結果表示
    #cv2.imshow('bgr_img', bgr_img)
    #cv2.imshow('mask', msk)
    #cv2.imshow('res', res)
    #cv2.waitKey(0)
    linear_x = 0
    angular_z = 0

    if n > 0:
      print("ブロブの個数:")
      print(n)
      print("各ブロブの外接矩形の左上x座標 :")
      print(data[:,0])
      print(data[0,0])
      print("各ブロブの外接矩形の左上y座標 :")
      print(data[:,1])
      print(data[0,1])
      print("各ブロブの外接矩形の幅 :")
      print(data[:,2])
      print("各ブロブの外接矩形の高さ :")
      print(data[:,3])
      print("各ブロブの面積 :")
      print(data[:,4])
      print("各ブロブの中心座標: ")
      print(center)
              
    #一番近くのターゲットを探す
    #データ無し：バック、旋回
    if len(data) <= 0:
      linear_x = -1
      angular_z = 5
      print("データ無し状態-------------")
    else:
      #データあり
      max_index = np.argmax(data[:,4])#面積最大の検索
      print("max----------------------------")
      print(data[max_index,4]) 
      if data[max_index,4] < 3500 : #マーカーとの距離が近すぎないことを確認
        x_offset = 200
        linear_x = 1
        angular_z = 1
        #曲がる方向の確認
        if (data[max_index,0] - x_offset) > 0:
          self.angular_z = (-1) * angular_z 
      else:
        #マーカーとの距離が近い時
        linear_x = -1
        angular_z = 5  
    #run
    print(linear_x, 0, angular_z)
    self.run.calcTwist(linear_x, angular_z)
    print("-- " + self.name + ": end image_callback --" )   #debag print


      
#      def tracking(self):
            

if __name__ == '__main__':
      rospy.init_node('ColorTracking')    #make node
      bot = Color_tracking('ume_bot')     #make object
      while not rospy.is_shutdown():
            sleep(1)
            #run
      cv2.destroyAllWindows()
