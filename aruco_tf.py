#!/usr/bin/env python  
import sys
import os
import math
import rospy
import numpy as np
import tf2_ros
import tf_conversions

#import geometry_msgs.msg
#import fiducial_msgs.msg

from fiducial_msgs.msg import FiducialTransformArray, FiducialArray
from geometry_msgs.msg import Pose, PoseWithCovariance
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class ArucoTF():

    def __init__(self):
        ## Pull necessary ROS parameters from launch file:
        # Read camera frame id
        param1 = rospy.search_param("camera_frame")
        self.cam_frame = rospy.get_param(param1)
        # Read fiducial pose topic parameter
        param2 = rospy.search_param("fiducial_pose_topic")
        self.fid_pose_topic = rospy.get_param(param2)
        # Read fiducial id value
        param3 = rospy.search_param("fiducial_id")
        self.fid_id = rospy.get_param(param3)
     
        # Initialize callback variables
        self.fid_pose = None

        # Initialize class variables
        #self.rel_fid_pose = None

        # Establish subscription to Fiducial Pose
        rospy.Subscriber(self.fid_pose_topic, FiducialTransformArray, self.fid_pose_callback)
        # Delay briefly for subscriber to find message
        rospy.sleep(2)

    def tf_calc(self):
        # initialize old_time for comparison
        old_time = 0

        while not rospy.is_shutdown():
            # Use a temp value so self.fid_pose.transforms 
            # does not update before calculations complete
            temp = self.fid_pose.transforms
            # Track time to limit measurements to camera frequency (30 hz)
            time = self.fid_pose.header.stamp.secs + self.fid_pose.header.stamp.nsecs*10**-9
            if time > old_time + .033 : # .033 sec = 30 hz
                for i in range(len(temp)): # number of detected markers
                    if temp[i].fiducial_id == self.fid_id: # if we see desired marker
                        #self.rel_fid_pose = temp[i]
                        print(temp[i])
            
            old_time = time # update time


        

    # Callback function for fiducial pose subscription (from aruco_detect)
    def fid_pose_callback(self, fid_pose_msg):
        self.fid_pose = fid_pose_msg
        if self.fid_pose.transforms != []:
            self.fid_pose.header.frame_id = self.cam_frame







if __name__ == '__main__':

    rospy.init_node('aruco_tf', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    tf_ar = ArucoTF()
    tf_ar.tf_calc()
