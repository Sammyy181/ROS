#! /usr/bin/env python3

import rospy
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError 
import cv2.aruco as aruco
from sensor_msgs.msg import Image
import numpy as np
from nav_msgs.msg import Odometry
import math
import json

def Video(data):
    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
    gray = cv.cvtColor(cv_image,cv.COLOR_BGR2GRAY)
    cv.imshow("Video", gray)
    cv.waitKey(10)

if __name__ == '__main__':
    rospy.init_node("image.py",anonymous=True)
    rospy.Subscriber("/camera/color/image_raw",Image,Video)
    rospy.spin()

