#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError 
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
import cv2.aruco as aruco
from sensor_msgs.msg import Image

bridge = CvBridge()
arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
arucoParams = aruco.DetectorParameters_create()

def detectMarker(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
        rospy.loginfo("CvBridge Error: {0}".format(e))
        return
    
    gray = cv.cvtColor(cv_image,cv.COLOR_BGR2GRAY)
    

    corners, ids, rejected = aruco.detectMarkers(gray, arucoDict,parameters = arucoParams)

    if ids is not None:
        rospy.loginfo("Detected Markers with IDs: %s", ids.flatten())
        cv.imshow("Marker", gray)
        cv.waitKey(5)

if __name__ == '__main__':
    rospy.init_node('trial',anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, detectMarker)
    rospy.spin()
