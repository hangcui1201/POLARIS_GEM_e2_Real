#!/usr/bin/env python3

#================================================================
# File name: gem_rviz_text.py                                                                  
# Description: show sensor info in Rviz                                                              
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 07/10/2021                                                                
# Date last modified: 07/30/2022                                                          
# Version: 0.1                                                                    
# Usage: rosrun gem_gnss gem_rviz_text.py                                                                     
# Python version: 3.8                                                             
#================================================================

import rospy
import math
import random
import numpy as np

from geometry_msgs.msg import Twist
from novatel_gps_msgs.msg import Inspva, NovatelPosition, NovatelExtendedSolutionStatus
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA, Float32, Float64

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt


class GEMOverlay(object):

    def __init__(self):
          
        self.text_pub     = rospy.Publisher("/gem_rviz_text", OverlayText, queue_size=5)
        self.gps_sub      = rospy.Subscriber("/novatel/inspva", Inspva, self.gps_callback)
        self.rtk_sub      = rospy.Subscriber("/novatel/bestpos", NovatelPosition, self.rtk_callback)
        self.speed_sub    = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        self.steer_sub    = rospy.Subscriber("/pacmod/parsed_tx/steer_rpt", SystemRptFloat, self.steer_callback)
        self.overlaytext  = self.update_overlaytext()
        self.gps_update   = False
        self.lat          = 0.0
        self.lon          = 0.0
        self.yaw          = 0.0
        self.speed        = 0.0 # m/s
        self.steer        = 0.0 # degrees
        self.rtk          = "Disabled"
        self.stopSign     = 0.0
        self.rate         = rospy.Rate(20)

    def speed_callback(self, msg):
        self.speed = round(msg.vehicle_speed, 2)

    def steer_callback(self, msg):
        self.steer = round(np.degrees(msg.output),1)

    def gps_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)
        self.yaw = round(msg.azimuth, 6)

        print(self.yaw)

    def rtk_callback(self, msg):
        if msg.extended_solution_status.advance_rtk_verified:
            self.rtk = "Enabled"
        else:
            self.rtk = "Disabled"

    def update_overlaytext(self, rtk="Disabled", lat=0.0, lon=0.0, yaw=0.0, speed=0.0, steer=0.0):
        text            = OverlayText()
        text.width      = 400
        text.height     = 250
        text.left       = 10
        text.top        = 10
        text.text_size  = 12
        text.line_width = 2
        text.font       = "DejaVu Sans Mono"
        text.text       = """RTK      = %s
                             Lat      = %s
                             Lon      = %s
                             Yaw      = %s
                             Speed [m/s] = %s 
                             Steer [deg] = %s 
                          """ % (rtk, str(lat), str(lon), str(yaw), str(speed), str(steer))
        text.fg_color   = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        text.bg_color   = ColorRGBA(0.0, 0.0, 0.0, 0.2)
        return text
    
    def update_overlay_textonly(self, new_text):
        self.overlaytext.text = new_text
    
    def start_demo(self):
        
        while not rospy.is_shutdown():

            if(self.gps_update):
                gps_text = """RTK      = %s
                              Lat      = %s
                              Lon      = %s
                              Yaw      = %s
                              Speed [m/s] = %s 
                              Steer [deg] = %s
                           """ % (self.rtk, str(self.lat), str(self.lon), str(self.yaw), str(self.speed), str(self.steer))
                self.update_overlay_textonly(gps_text)
            else:
                self.overlaytext = self.update_overlaytext()
                self.gps_update  = True

            self.text_pub.publish(self.overlaytext)
            self.rate.sleep()

  
def gem_overlay():

    rospy.init_node('gem_rviz_markers', anonymous=True)

    gem_overlay_object = GEMOverlay()

    try:
        gem_overlay_object.start_demo()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    gem_overlay()