#!/usr/bin/env python
# -*-coding: utf-8 -*-
#정지선 인식

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rospy, cv2, cv_bridge, numpy
from camera_center import Follower_center
from Turtle_move import Turtle_move

class stop_line:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.camera_center = Follower_center()
        self.move = Turtle_move()

    def image_callback(self,msg):
        image = self.camera_center.image
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([180, 15, 255])
        mask_w = cv2.inRange(hsv, lower_white, upper_white)  #정지선 흰색만 인식
        h, w, d = self.image.shape
        search_top = 3 * h / 4
        search_bot = search_top + 20

        mask_w[0:search_top, 0:w] = 0
        mask_w[search_bot:h, 0:w] = 0
        M = cv2.moments(mask_w)

        self.move.move(1.0) #1.0의 속도로 움직임
        if M['m00'] > 0:  # 하얀선이 검출되면 객체의 면적을 계산함
            et, img_binary = cv2.threshold(mask_w, 127, 255, 0)
            _, contours, hierarchy = cv2.findContours(img_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                self.area = cv2.contourArea(cnt)   #면적 계산

            if self.area > 6000 :
                print("stop line")
                self.move.stop()