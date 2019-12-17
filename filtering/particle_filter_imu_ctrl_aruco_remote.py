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

#from fiducial_msgs.msg import FiducialTransformArray, FiducialArray
#from geometry_msgs.msg import Pose, PoseWithCovariance, TransformStamped, PoseStamped 
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
#from tf.transformations import quaternion_from_euler, euler_from_quaternion


class ArucoParticleFilterRemote():

    def __init__(self):
        ## Pull necessary ROS parameters from launch file:
        # Read pose observation topic (pose estimated from aruco detection) 
        
        
        # param = rospy.search_param("pose_observation_topic")
        # self.pose_obs_top = rospy.get_param(param)
        # # Read prediction update topic (EKF filtered odometry from IMU and ctrl inputs) 
        # param = rospy.search_param("prediction_update_topic")
        # self.pred_up_top = rospy.get_param(param)
        

        ## Read Sensory Input Starts ## 
        
        param = rospy.search_param("pose_observation_topic")
        self.pose_obs_top = rospy.get_param(param)    
        param = rospy.search_param("imu0") # Reading IMU data
        self.imu0_top = rospy.get_param(param)
        param = rospy.search_param("twist0") # Reading control input data
        self.twist0_top = rospy.get_param(param)
        
        ## Read Sensory Imput Ends ##
        
        # Read particle count
        param = rospy.search_param("particle_count")
        self.pc = rospy.get_param(param)
        
        # Read covariance values 
        param = rospy.search_param("initial_estimate_covariance")
        self.init_cov = rospy.get_param(param)
        param = rospy.search_param("linear_process_covariance")
        self.pl_cov = rospy.get_param(param)
        param = rospy.search_param("angular_process_covariance")
        self.pa_cov = rospy.get_param(param)
        param = rospy.search_param("linear_observation_covariance")
        self.ol_cov = rospy.get_param(param)
        param = rospy.search_param("angular_observation_covariance")
        self.oa_cov = rospy.get_param(param)

        # Initialize callback variables
        self.imu0 = None
        self.twist0 = None
        self.obs_pose = None
        #self.pred_odom = None

        # Initialize class variables
        self.time = None
        self.old_time = None
    
        # Establish subscription to observation pose
        rospy.Subscriber(self.pose_obs_top, PoseWithCovarianceStamped, self.obs_pose_callback)
        # Establish subscription to IMU
        rospy.Subscriber(self.imu0_top, Imu, self.imu0_callback)
        # Establish subscription to control input (message twist0)
        rospy.Subscriber(self.twist0_top, TwistWithCovarianceStamped, self.twist0_callback)

        # Delay briefly to allow subscribers to find messages
        rospy.sleep(0.5)


        # Build the process and observation covariance matrices
        self.cov_matrices_build()

        # Initialize array of particle states | # particles x 4 [x, y, theta_z, weight]
        self.particles = (np.random.rand(self.pc,4)-0.5)*(2*self.init_cov)
        self.pred_particles = self.particles
        print(self.particles)
        print(self.particles + self.particles[:,0])

        # Initialize publisher for estimated pose of vehicle in map frame
        # self.posepub = rospy.Publisher(self.est_veh_pose_top, PoseWithCovarianceStamped, queue_size=10)
        # self.pub = PoseWithCovarianceStamped()
        # self.pub.header.frame_id = self.map_frame
        # self.pub.pose.covariance = self.cov_matrix_build()

    # Function to call all functions and run particle filter
    def run_pf(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.5)
            
            # Only predict when (IMU and ctrl) comes in
            if self.imu0 != None and self.twist0 != None:
                # Track prediction message timestamp
                self.time = self.imu0.header.stamp.secs + self.imu0.header.stamp.nsecs*10**-9
                if self.time > self.old_time:
                    self.predict()
                # Update previous timestamp for comparison to next
                self.old_time = self.time

            # Only update observation when an aruco-based pose measurement comes in
            if self.obs_pose != None:
                self.obs_update()
                self.resample()
                self.weight()




    ##### Primary particle filter functions #####

    # Function for process/prediction step 
    def predict(self):
        # Use covariance to calculate gaussian noise for prediction
        pnoise = self.gaussian_noise(self.pcov_matrix)
        
        # Unpack IMU message
        quaternion = (
        self.imu0.orientation.x,
        self.imu0.orientation.y,
        self.imu0.orientation.z,
        self.imu0.orientation.w)
        
        #euler = tf_conversions.transformations.euler_from_quaternion(quaternion)
        euler = tf2_ros.euler_from_quaternion(quaternion)
        
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        vroll= self.imu0.angular_velocity.x
        vpitch= self.imu0.angular_velocity.y
        vyaw= self.imu0.angular_velocity.z
        ax= self.imu0.linear_acceleration.x
        ay= self.imu0.linear_acceleration.y
        az= self.imu0.linear_acceleration.z
        
        # Unpack odometry message
        xvel = self.twist0.twist.twist.linear.x
        yvel = self.twist0.twist.twist.linear.y
        omega = self.twist0.twist.twist.angular.z
        # Calculate timestep from last prediction update
        dt = self.time - self.old_time
        # Update particle pose estimates based on velocity and angle from odometry
        #self.pred_particles = 


    # Function for observation update
    def obs_update(self):
            # Unpack observation pose estimates
            x_obs  = self.obs_pose.pose.position.x
            y_obs  = self.obs_pose.pose.position.y
            #theta_z = tf2_ros.euler_from_quaternion(self.obs_pose.pose.orientation)[2]
            _, _, ang_z_obs = tf2_ros.euler_from_quaternion(self.obs_pose.pose.orientation)

            # Now do things ..


    # Function to resample particles
    def resample(self):
        print('resample time')

    # Function to reassign weights to particles
    def weight(self):
        print('weight time')

    ##### Support Functions #####

    # Function to build 3x3 process and observation covariance matrices
    def cov_matrices_build(self):
        # Build initial estimate covariance matrix
        # self.icov_matrix = np.array([[self.init_cov, 0.0, 0.0],
        #                             [0.0, self.init_cov, 0.0],
        #                             [0.0, 0.0, self.init_cov]])
        # Build process covariance matrix
        self.pcov_matrix = np.array([[self.pl_cov, 0.0, 0.0],
                                    [0.0, self.pl_cov, 0.0],
                                    [0.0, 0.0, self.pa_cov]])
        # Build observation covariance matrix
        self.ocov_matrix = np.array([[self.ol_cov, 0.0, 0.0],
                                    [0.0, self.ol_cov, 0.0],
                                    [0.0, 0.0, self.oa_cov]])

    # Function to assign gaussian noise from diagonalcovariance matrix
    def gaussian_noise(self, cov_mat):
        var = np.diagonal(cov_mat)
        noise = np.sqrt(var)*np.random.randn(self.pc, 3)
        return noise

    # Callback function for observation pose subscription (from aruco_detect)
    def obs_pose_callback(self, obs_pose_msg):
        self.obs_pose = obs_pose_msg

    # Callback function for IMU subscription (from IMU)
    def imu0_callback(self, Imu_msg):
        self.imu0 = Imu_msg

    # Callback function for control message subscription (from robot's control)
    def twist0_callback(self, twist0_msg):
        self.twist0 = twist0_msg




if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('particle_filter_aruco_remote', anonymous=True)
    rospy.loginfo("Successful initilization of node")
    
    # Create particle filter class
    pf = ArucoParticleFilterRemote()
    rospy.loginfo("ArucoParticleFilterRemote class successfully created")
    
    # Run particle filter
    pf.run_pf()
    

"""
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


                ### Should old_time only be in the if statement?
                # Seemed to work before when it was outside, but logically I think should be in...
                old_time = time # update time for comparison on next iter
            
            #old_time = time # update time for comparison on next iter

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
"""