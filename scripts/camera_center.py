#!/usr/bin/env python
# -*-coding: utf-8 -*-

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rospy, cv2, cv_bridge, numpy
from std_msgs.msg import Bool
from Turtle_move import Turtle_move
from stop_sign import stop_sign
from blocking_bar import blocking_bar
from stop_line import stop_line



class Follower_center:
    def __init__(self, topic_name):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(topic_name, Image, self.image_callback)
        self.stop_line_pub = rospy.Publisher('stop_detect',Bool,queue_size=1)
        self.move = Turtle_move()
        self.stop_sign = stop_sign()
        self.blocking_bar = blocking_bar()
        self.stop_line = stop_line()
        self.image = 0

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') #보고있는 원본 화면 불러오기


