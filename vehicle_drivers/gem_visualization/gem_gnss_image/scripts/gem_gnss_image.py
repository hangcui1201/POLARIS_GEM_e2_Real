#!/usr/bin/env python3

#================================================================
# File name          : gem_gnss_image.py                                                                  
# Description        : show vehicle's heading and position in an image                                                                
# Author             : Hang Cui (hangcui3@illinois.edu)                                                                     
# Date created       : 08/13/2022                                                                
# Date last modified : 01/25/2023                                                            
# Version            : 0.2                                                                    
# Usage              : rosrun gem_gnss gem_gnss_image.py                                                                      
# Python version     : 3.8                                                             
#================================================================

from __future__ import print_function

# Python Headers
import os
import cv2 
import csv
import math
import numpy as np
from numpy import linalg as la

# ROS Headers
import tf
import rospy
import rospkg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# GEM Sensor Headers
from std_msgs.msg import Float64
from gps_common.msg import GPSFix
from sensor_msgs.msg import Imu, NavSatFix
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva, NovatelCorrectedImuData

image_file  = 'gnss_map.png'
curr_path = os.path.abspath(__file__) 
image_path = curr_path.split('scripts')[0] + 'images/' + image_file

class GNSSImage(object):

    global image_path
    
    def __init__(self):

        self.rate = rospy.Rate(15)

        # Read image in BGR format
        self.map_image = cv2.imread(image_path)

        # Create the cv_bridge object
        self.bridge  = CvBridge()
        self.map_image_pub = rospy.Publisher("/motion_image", Image, queue_size=1) 

        # Subscribe information from sensors
        self.lat      = 0
        self.lon      = 0
        self.heading  = 0
        self.gnss_sub = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)

        self.lat_start_bt = 40.092722  # 40.09269  
        self.lon_start_l  = -88.236365 # -88.23628
        self.lat_scale    = 0.00062    # 0.0007    
        self.lon_scale    = 0.00136    # 0.00131   

        self.arrow        = 40 
        self.img_width    = 2107
        self.img_height   = 1313


    def inspva_callback(self, inspva_msg):
        self.lat     = inspva_msg.latitude
        self.lon     = inspva_msg.longitude
        self.heading = inspva_msg.azimuth 


    def image_heading(self, lon_x, lat_y, heading):
        
        if(heading >=0 and heading < 90):
            angle  = np.radians(90-heading)
            lon_xd = lon_x + int(self.arrow * np.cos(angle))
            lat_yd = lat_y - int(self.arrow * np.sin(angle))

        elif(heading >= 90 and heading < 180):
            angle  = np.radians(heading-90)
            lon_xd = lon_x + int(self.arrow * np.cos(angle))
            lat_yd = lat_y + int(self.arrow * np.sin(angle))  

        elif(heading >= 180 and heading < 270):
            angle = np.radians(270-heading)
            lon_xd = lon_x - int(self.arrow * np.cos(angle))
            lat_yd = lat_y + int(self.arrow * np.sin(angle))

        else:
            angle = np.radians(heading-270)
            lon_xd = lon_x - int(self.arrow * np.cos(angle))
            lat_yd = lat_y - int(self.arrow * np.sin(angle)) 

        return lon_xd, lat_yd         


    def start_gi(self):
        
        while not rospy.is_shutdown():

            lon_x = int(self.img_width*(self.lon-self.lon_start_l)/self.lon_scale)
            lat_y = int(self.img_height-self.img_height*(self.lat-self.lat_start_bt)/self.lat_scale)
            lon_xd, lat_yd = self.image_heading(lon_x, lat_y, self.heading)

            pub_image = np.copy(self.map_image)
            cv2.arrowedLine(pub_image, (lon_x, lat_y), (lon_xd, lat_yd), (0, 0, 255), 2)
            cv2.circle(pub_image, (lon_x, lat_y), 12, (0,0,255), 2)

            try:
                # Convert OpenCV image to ROS image and publish
                self.map_image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))

            self.rate.sleep()


def main():

    rospy.init_node('gem_gnss_image_node', anonymous=True)

    gi = GNSSImage()

    try:
    	gi.start_gi()
    except KeyboardInterrupt:
        print ("Shutting down gnss image node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

