#! /usr/bin/env python3

import rospy
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError 
from aruco_msgs.msg import MarkerArray
import cv2.aruco as aruco
from sensor_msgs.msg import Image
import numpy as np
from nav_msgs.msg import Odometry
import math

"""camera_matrix = np.array([[462.138,0,320],[0,462.138,240],[0,0,1]])
dist_coeffs = np.zeros([0.0,0.0,0.0,0.0,0.0])"""


class markers:

    def __init__(self):
        self.bridge = CvBridge()
        self.arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        self.arucoParams = aruco.DetectorParameters_create()
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.findPos)
        self.image_sub = rospy.Subscriber("/camera/color/image",Image,self.detectMarker)
        self.detectedIDs = []
        self.poses = []
        
    def findPos(self,data):
        self.x = round(data.pose.pose.position.x,3)
        self.y = round(data.pose.pose.position.y,3)
        self.z = round(data.pose.pose.position.z,3)

    def detectMarker(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")

        except CvBridgeError as e:
            rospy.loginfo("CvBridge Error: {0}".format(e))
            return
        
        gray = cv.cvtColor(cv_image,cv.COLOR_BGR2GRAY) 
        
        corners, ids, rejected = aruco.detectMarkers(gray, self.arucoDict,parameters = self.arucoParams)

        if self.ids is not None:
            marker_array = MarkerArray()
            
            #cv.imshow("Marker", gray)
            #cv.waitKey(5) 

            for id,corner in zip(ids,corners):
                if id[0] not in self.detectedIDs:
                    rospy.loginfo("Detected new Marker with ID: %d", id[0])
                    self.detectedIDs.append(id[0])
                    
                    self.poseDetect(self,corner[0],id[0])


        else:
            rospy.loginfo("No Markers Detected")

    def poseDetect(self,corners,id):
        
            marker_x = marker_y = 0

            for point in corners:
                marker_x += point[0]
                marker_y += point[1]
            
            marker_x = marker_x/4
            marker_y = marker_y/4

            marker_distance = math.sqrt((marker_x - self.x) ** 2 + (marker_y - self.y) ** 2 + self.z ** 2)

            rospy.loginfo("Pose of AruCo ID %d stored\n",id)
            rospy.loginfo("Pose: x = %d, y = %d, z = %d",marker_x,marker_y,self.z)

            self.waypoints[id] = {
                'AruCo_ID' : id,
                'Waypoint' : {
                    'x' : float(self.x),
                    'y' : float(self.y),
                    'z' : float(self.z)
                },
                'Pose' : {
                    'x' : float(marker_x),
                    'y' : float(marker_y),
                    'z' : float(self.z)
                }
            }
             

if __name__ == '__main__':
    rospy.init_node('AruCo',anonymous=True)

    ms = markers()

    rospy.spin()


