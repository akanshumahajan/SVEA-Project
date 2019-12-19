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
from geometry_msgs.msg import Pose, PoseWithCovariance, TransformStamped
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
        # Read estimated vehicle pose frame_id and topic
        param4 = rospy.search_param("est_vehicle_frame")
        self.est_veh_pose_frame = rospy.get_param(param4)
        param5 = rospy.search_param("map_pose_estimate_topic")
        self.est_veh_pose_top = rospy.get_param(param5)
        # Read map frame   
        param6 = rospy.search_param("map_frame")
        self.map_frame = rospy.get_param(param6)
        # Read covariance values
        param7 = rospy.search_param("lin_cov_aruco")
        self.lin_cov = rospy.get_param(param7)
        param8 = rospy.search_param("ang_cov_aruco")
        self.ang_cov = rospy.get_param(param8)

        # Initialize callback variables
        self.fid_pose = None

        # Initialize class variables
        

        # Establish subscription to Fiducial Pose
        rospy.Subscriber(self.fid_pose_topic, FiducialTransformArray, self.fid_pose_callback)
        # Delay briefly for subscriber to find message
        rospy.sleep(0.5)

        # Initilize tf2 broadcaster and transform message
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = self.cam_frame
        self.t.child_frame_id = "fid"+str(self.fid_id)

        # Initialize listener for estimated pose of vehicle in map frame
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Initialize publisher for estimated pose of vehicle in map frame
        self.posepub = rospy.Publisher(self.est_veh_pose_top, PoseWithCovarianceStamped, queue_size=10)
        self.pub = PoseWithCovarianceStamped()
        self.pub.header.frame_id = self.map_frame
        self.pub.pose.covariance = self.cov_matrix_build()


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
                        self.t.header.stamp = rospy.Time.now()
                        self.t.transform = temp[i].transform
                        self.br.sendTransform(self.t)
                        # Call publish function so estimated base_link can be read in map frame
                        self.map_pose_pub()

                old_time = time # update time for comparison on next iter

    def map_pose_pub(self):
        trans = None
        try:
            trans = self.tfBuffer.lookup_transform(self.map_frame, self.est_veh_pose_frame, rospy.Time(0), rospy.Duration(1.0))
        except:
            #trans = self.tfBuffer.lookup_transform(self.map_frame, self.est_veh_pose_frame, rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo('Failure of lookup transfrom from estimated vehicle pose to map')
        if trans:
            self.pub.header.stamp = rospy.Time.now()
            self.pub.pose.pose.position = trans.transform.translation
            self.pub.pose.pose.orientation = trans.transform.rotation
            self.posepub.publish(self.pub)            

    # Callback function for fiducial pose subscription (from aruco_detect)
    def fid_pose_callback(self, fid_pose_msg):
        self.fid_pose = fid_pose_msg
        if self.fid_pose.transforms != []:
            self.fid_pose.header.frame_id = self.cam_frame

    def cov_matrix_build(self):
        self.cov_matrix = [self.lin_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, self.lin_cov, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, self.lin_cov, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, self.ang_cov, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, self.ang_cov, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, self.ang_cov]
        return self.cov_matrix


if __name__ == '__main__':

    rospy.init_node('aruco_tf', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    tf_ar = ArucoTF()
    tf_ar.tf_calc()

