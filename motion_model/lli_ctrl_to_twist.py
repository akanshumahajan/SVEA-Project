#!/usr/bin/env python  
import sys
import os
import math
import rospy
import numpy as np

from svea_arduino.msg import lli_ctrl
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry


class Republish():

    def __init__(self):
        ## Pull necessary ROS parameters from launch file:
        # Read control message topic
        param1 = rospy.search_param("ctrl_message_topic")
        self.ctrl_msg_top = rospy.get_param(param1)
        # Read twist message topic
        param2 = rospy.search_param("twist_message_topic")
        self.twist_msg_top = rospy.get_param(param2)
        # Read vehicle frame id topic
        param3 = rospy.search_param("est_frame_id")
        self.veh_frame_id = rospy.get_param(param3)
     
        # Initialize callback variables
        self.ctrl_msg = None
        # Initialize class variables
        self.twist_msg = None

        # Establish subscription to control message
        rospy.Subscriber(self.ctrl_msg_top, lli_ctrl, self.ctrl_msg_callback)
        # Delay briefly for subscriber to find message
        rospy.sleep(2)

        # Establish publisher of converted Twist message
        self.pub = rospy.Publisher(self.twist_msg_top, TwistWithCovarianceStamped, queue_size=10)

    def ctrl_calc_and_pub(self):
        # initialize message
        self.twist_msg = TwistWithCovarianceStamped()
        self.twist_msg.header.frame_id = self.veh_frame_id
        rate = rospy.Rate(100) # 100hz

        while not rospy.is_shutdown():
            if self.ctrl_msg != None:
                # Unpack ctrl message
                # Steering range (radians) - assumed linear across range
                # msg [-127,127] | actual [-pi/4,pi/4] | direction (right,left)
                c_ang = self.ctrl_msg.steering*(math.pi/4)/127
                # Velocity range 1st gear (m/s) - assumed linear across range with deadzone [-20,20] 
                # msg [-127,127] | actual [-1.5,1.5] | direction (back,forward)
                dz = [-20,20]
                max_speed = 2.5
                if self.ctrl_msg.velocity < dz[0]:
                    c_vel = (self.ctrl_msg.velocity - dz[0])*-1*max_speed/(-127 - dz[0])
                elif self.ctrl_msg.velocity > dz[1]:
                    c_vel = (self.ctrl_msg.velocity - dz[1])*max_speed/(127 - dz[1])
                else:
                    c_vel = 0

                # Apply Bicycyle Model
                wheelbase = .32 # in meters
                B = math.atan2(math.tan(c_ang),2)
                x_vel = c_vel*math.cos(B)
                y_vel = c_vel*math.sin(B)
                ang_vel = (c_vel/(wheelbase/2))*math.sin(B)

                # Build Header for current time stamp
                self.twist_msg.header.seq += 1
                self.twist_msg.header.stamp = rospy.Time.now()
                
                # Build Twist using bicycle model
                self.twist_msg.twist.twist.linear.x = x_vel
                self.twist_msg.twist.twist.linear.y = y_vel
                self.twist_msg.twist.twist.angular.z = ang_vel

                # Publish message
                self.pub.publish(self.twist_msg)
                rate.sleep()


    # Callback function for fiducial pose subscription (from aruco_detect)
    def ctrl_msg_callback(self, ctrl_msg):
        self.ctrl_msg = ctrl_msg

if __name__ == '__main__':

    rospy.init_node('ctrl_to_twist', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    ctt = Republish()
    ctt.ctrl_calc_and_pub()
