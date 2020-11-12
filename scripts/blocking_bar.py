#!/usr/bin/env python
# -*-coding: utf-8 -*-
#차단바 인식
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rospy, cv2, cv_bridge, numpy
from camera_center import Follower_center
from Turtle_move import Turtle_move


class blocking_bar:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.camera_center = Follower_center()
        self.move = Turtle_move()

    def image_callback(self,msg):
        image = self.camera_center.image
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blocking_bar = cv2.imread('/home/shin/catkin_ws/src/deu_car/world/models/blocking_bar/materials/blocking.png')  #매칭시킬 이미지      blocking.png
        blocking_bar_gray = cv2.cvtColor(blocking_bar, cv2.COLOR_BGR2GRAY)
        sift = cv2.xfeatures2d.SIFT_create()

        kp1, des1 = sift.detectAndCompute(image_gray, None)
        kp2, des2 = sift.detectAndCompute(blocking_bar_gray, None)
        bf = cv2.BFMatcher()

        matches = bf.match(des1, des2)  #이미지 매칭
        res = cv2.drawMatches(image, kp1, blocking_bar, kp2, matches[:30], None, flags=2) #얼마나 매칭되었는지 이미지에 출력
        if matches[0].distance > 500 and self.blocking == 0 :  #distance : 이미지끼리의 인식률  숫자가 높을수록 인식률이 높다.  ## 인식률이 500이상이면 차단바라고 인식
            self.move.stop2()
        else :
            self.move.move(1.0) #더이상 차단바가 보이지 않을때 속도를 1.0으로 움직임
            self.blocking = 1  #더이상 차단바를 인식하지 않음