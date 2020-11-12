#!/usr/bin/env python
# -*-coding: utf-8 -*-
#정지 표지판 인식
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rospy, cv2, cv_bridge, numpy
from camera_center import Follower_center
from Turtle_move import Turtle_move


class stop_sign:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.camera_center = Follower_center()
        self.move = Turtle_move()

    def image_callback(self,msg):
        image = self.camera_center.image
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        stop_sign = cv2.imread('/home/shin/catkin_ws/src/deu_car/world/models/stop_sign_resize/materials/textures/StopSign_Diffuse.png')
        stop_sign_gray = cv2.cvtColor(stop_sign, cv2.COLOR_BGR2GRAY)
        sift = cv2.xfeatures2d.SIFT_create()

        kp1, des1 = sift.detectAndCompute(image_gray, None)
        kp2, des2 = sift.detectAndCompute(stop_sign_gray, None)
        bf = cv2.BFMatcher()

        matches = bf.match(des1, des2)  #이미지 매칭
        if matches[0].distance > 340 :  #distance : 이미지끼리의 인식률  숫자가 높을수록 인식률이 높다.  ## 인식률이 500이상이면 차단바라고 인식
            self.move.stop()
        else :
            self.move.move(1.0)