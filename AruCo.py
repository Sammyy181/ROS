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
import json

"""camera_matrix = np.array([[462.138,0,320],[0,462.138,240],[0,0,1]])
dist_coeffs = np.zeros([0.0,0.0,0.0,0.0,0.0])"""


class Markers:

    def __init__(self):
        self.bridge = CvBridge()
        self.arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        self.arucoParams = aruco.DetectorParameters_create()
        #self.odom_sub = rospy.Subscriber("/odom",Odometry,self.findPos)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.detectMarker)
        self.detectedIDs = []
        self.poses = []
        self.waypoints = {}
        
    def findPos(self,data,corners,id,subscriber):
        odomx = data.pose.pose.position.x
        odomy = data.pose.pose.position.y
        odomz = data.pose.pose.position.z
        
        marker_x = marker_y = 0
        
        for point in corners:
            marker_x += point[0]
            marker_y += point[1]
            
        marker_x /= 4
        marker_y /= 4
        
        marker_distance = math.sqrt((marker_x - x) ** 2 + (marker_y - y) ** 2 + z ** 2)
        
        rospy.loginfo("Pose of AruCo %d stored\n",id)
        rospy.loginfo("Pose: x = %lf, y = %lf, z = %lf\n",marker_x,marker_y,z)
        rospy.loginfo("Waypoint: x = %d, y = %d, z = %d", odomx, odomy, odomz)
        
        self.waypoints[id] = {
            'AruCo_ID' : id,
            'Waypoint' : {
                'x' : float(odomx),
                'y' : float(odomy),
                'z' : float(odomz)
            },
            'Pose' : {
                'x' : float(marker_x),
                'y' : float(marker_y),
                'z' : float(z)
            }
        }

        with open("data.json","w") as outfile:
            json.dump(waypoints[id],outfile)
        
        subscriber.unregister()

    def detectMarker(self,img_data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")

        except CvBridgeError as e:
            rospy.loginfo("CvBridge Error: {0}".format(e))
            return
        
        gray = cv.cvtColor(cv_image,cv.COLOR_BGR2GRAY) 
        
        corners, ids, rejected = aruco.detectMarkers(gray, self.arucoDict,parameters = self.arucoParams)

        if ids is not None:
            marker_array = MarkerArray()
            
            #cv.imshow("Marker", gray)
            #cv.waitKey(5) 

            for id,corner in zip(ids,corners):
                if id[0] not in self.detectedIDs:
                    rospy.loginfo("Detected new Marker with ID: %d", id[0])
                    self.detectedIDs.append(id[0])
                    
                    subscriber = rospy.Subscriber("/odom", Odometry, lambda data : self.findPos(data,corner[0],id[0],subscriber))


        else:
            rospy.loginfo("No Markers Detected")          

if __name__ == '__main__':
    rospy.init_node('AruCo',anonymous=True)

    ms = Markers()

    rospy.spin()


