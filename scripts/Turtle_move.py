#!/usr/bin/env python
# -*-coding: utf-8 -*-

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rospy, cv2, cv_bridge, numpy
from std_msgs.msg import Bool
from camera_left import Follower_left
from camera_right import Follower_right

class Turtle_move:
    def __init__(self) :
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.follower_left = Follower_left()
        self.follower_right = Follower_right()


    def move(self,speed):
        err = (self.follower_left.tmp + self.follower_right.tmp)/2
        self.twist.linear.x = speed
        self.twist.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twist)

    def stop(self):
        now_time = rospy.Time.now().to_sec()
        target_time = now_time + 50
        rospy.sleep(rospy.Duration(3))
        while (target_time < int(rospy.Time.now().to_sec())):
            self.twist.linear.x = 0.7
        self.cmd_vel_pub.publish(self.twist)

    def stop2(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
