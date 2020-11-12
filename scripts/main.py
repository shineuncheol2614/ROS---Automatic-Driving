# !/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy, cv2, cv_bridge, numpy
from camera_left import Follower_left
from camera_right import Follower_right
from camera_center import Follower_center
from obstacle_avoid import ObstacleDetection

if __name__ == "__main__":
    rospy.init_node('auto drive')
    follower = Follower_center("camera/rgb/image_raw")
    follower_left = Follower_left("rrbot/camera_left/image_raw")
    follower_right = Follower_right("rrbot/camera_right/image_raw")
    obstacle_avoid = ObstacleDetection()

    rospy.spin()