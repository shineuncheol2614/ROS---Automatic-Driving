#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist
from Tutle_move import Turtle_move


class ObstacleDetection:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.callback, queue_size=1)
        self.move = Turtle_move()

    def doesImageContainObject(self, img):
        return (~np.isnan(img).any(axis=0).all())

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        cv_image_process = cv_image[:280, :]

        if (self.doesImageContainObject(cv_image_process) == True):
            print("I see something!")
            self.move.stop2()
        else:
            print("The coast is clear.")
            self.move.move(1.0)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "32FC1"))
        except CvBridgeError as e:
            print(e)