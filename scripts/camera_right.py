#!/usr/bin/env python
# -*-coding: utf-8 -*-

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rospy, cv2, cv_bridge, numpy
from std_msgs.msg import Bool
from camera_center import Follower_center


class Follower_right():
    def __init__(self, topic_name):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub_center = rospy.Subscriber(topic_name, Image, self.image_callback)
        self.camera_center = Follower_center()
        self.tmp = 0
        self.line = 0

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hue,saturation,value = cv2.split(hsv)
        self.line = cv2.inRange(value, 217, 219) #hsv의 v = 명도 명도가 219이면 노란색과 흰색 둘다 인식 가능
        h, w, d = image.shape

        search_top = 3 * h / 4
        search_bot = search_top + 20

        self.line[0:search_top, 0:w] = 0
        self.line[search_bot:h, 0:w] = 0
        self.line[0:h,0:w/2] = 0
        L = cv2.moments(self.line)

        if L['m00'] > 0 and self.camera_center.area < 6000:
            cx = int(L['m10'] / L['m00'])
            self.tmp = cx