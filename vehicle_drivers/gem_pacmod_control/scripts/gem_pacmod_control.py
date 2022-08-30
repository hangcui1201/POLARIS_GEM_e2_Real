#!/usr/bin/env python3

#==============================================================================
# File name          : gem_pacmod_control.py                                                                  
# Description        : pacmod interface                                                             
# Author             : Hang Cui
# Email              : hangcui3@illinois.edu                                                                     
# Date created       : 08/08/2022                                                                 
# Date last modified : 08/18/2022                                                          
# Version            : 1.0                                                                    
# Usage              : rosrun gem_pacmod_control gem_pacmod_control.py                                                                   
# Python version     : 3.8                                                             
#==============================================================================

from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

# ROS Headers
import rospy
import alvinxy.alvinxy as axy 
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt


class PACMod(object):
    
    def __init__(self):

        self.rate = rospy.Rate(25)

        self.stanley_sub = rospy.Subscriber('/gem/stanley_gnss_cmd', AckermannDrive, self.stanley_gnss_callback)

        self.ackermann_msg_gnss                         = AckermannDrive()
        self.ackermann_msg_gnss.steering_angle_velocity = 0.0
        self.ackermann_msg_gnss.acceleration            = 0.0
        self.ackermann_msg_gnss.jerk                    = 0.0
        self.ackermann_msg_gnss.speed                   = 0.0 
        self.ackermann_msg_gnss.steering_angle          = 0.0

        # -------------------- PACMod setup --------------------

        self.gem_enable    = False
        self.pacmod_enable = False

        # GEM vehicle enable
        self.enable_sub = rospy.Subscriber('/pacmod/as_rx/enable', Bool, self.pacmod_enable_callback)
        # self.enable_cmd = Bool()
        # self.enable_cmd.data = False

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL

        # GEM vehilce brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear  = True
        self.brake_cmd.ignore = True

        # GEM vechile forward motion control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = False
        self.accel_cmd.clear  = True
        self.accel_cmd.ignore = True

        # GEM vechile turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None

        # GEM vechile steering wheel control
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 2.0 # radians/second


    # Get outputs of Stanley controller based on GNSS
    def stanley_gnss_callback(self, msg):
        self.ackermann_msg_gnss.acceleration = round(msg.acceleration ,2)
        self.ackermann_msg_gnss.steering_angle = round(msg.steering_angle ,2)


    # PACMod enable callback function
    def pacmod_enable_callback(self, msg):
        self.pacmod_enable = msg.data


    # Start PACMod interface
    def start_pacmod(self):
        
        while not rospy.is_shutdown():

            if(self.pacmod_enable == True):


                if (self.gem_enable == False):

                    # ---------- Enable PACMod ----------

                    # enable forward gear
                    self.gear_cmd.ui16_cmd = 3

                    # enable brake
                    self.brake_cmd.enable  = True
                    self.brake_cmd.clear   = False
                    self.brake_cmd.ignore  = False
                    self.brake_cmd.f64_cmd = 0.0

                    # enable gas 
                    self.accel_cmd.enable  = True
                    self.accel_cmd.clear   = False
                    self.accel_cmd.ignore  = False
                    self.accel_cmd.f64_cmd = 0.0

                    self.gear_pub.publish(self.gear_cmd)
                    print("Foward Engaged!")

                    self.turn_pub.publish(self.turn_cmd)
                    print("Turn Signal Ready!")
                    
                    self.brake_pub.publish(self.brake_cmd)
                    print("Brake Engaged!")

                    self.accel_pub.publish(self.accel_cmd)
                    print("Gas Engaged!")

                    self.gem_enable = True

                else: 

                    if (self.ackermann_msg_gnss.steering_angle <= 45 and self.ackermann_msg_gnss.steering_angle >= -45):
                        self.turn_cmd.ui16_cmd = 1
                    elif(self.ackermann_msg_gnss.steering_angle > 45):
                        self.turn_cmd.ui16_cmd = 2 # turn left
                    else:
                        self.turn_cmd.ui16_cmd = 0 # turn right

                    self.accel_cmd.f64_cmd = self.ackermann_msg_gnss.acceleration
                    self.steer_cmd.angular_position = np.radians(self.ackermann_msg_gnss.steering_angle)
  
                    self.accel_pub.publish(self.accel_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    

            self.rate.sleep()


def pacmod_run():

    rospy.init_node('pacmod_control_node', anonymous=True)
    pacmod = PACMod()

    try:
        pacmod.start_pacmod()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pacmod_run()


