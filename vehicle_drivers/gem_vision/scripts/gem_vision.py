#!/usr/bin/env python3

#================================================================
# File name: gem_vision.py                                                                  
# Description: show image from ZED camera                                                          
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 05/20/2021                                                                
# Date last modified: 08/24/2022                                                        
# Version: 0.1                                                                    
# Usage: rosrun gem_vision gem_vision.py                                                                      
# Python version: 3.8                                                             
#================================================================

from __future__ import print_function

import sys
import copy
import time
import rospy
import rospkg

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


class ImageConverter:

    def __init__(self):

        self.node_name = "gem_vision"
        
        rospy.init_node(self.node_name)
        
        rospy.on_shutdown(self.cleanup)
        
        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/zed2/zed_node/left/image_rect_color", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/front_camera/image_processed", Image, queue_size=1)


    def image_callback(self, ros_image):

        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # ----------------- Imaging processing code starts here ----------------

        pub_image = np.copy(frame)




        # ----------------------------------------------------------------------

        try:
            # Convert OpenCV image to ROS image and publish
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(pub_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))


    def cleanup(self):
        print ("Shutting down vision node.")
        cv2.destroyAllWindows()   


def main(args):       

    try:
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down vision node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
