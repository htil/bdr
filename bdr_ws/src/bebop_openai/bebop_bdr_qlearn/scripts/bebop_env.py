import numpy as np
import rospy
import time

import robot_ros_env

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty

import cv2
from cv_bridge import CvBridge, CvBridgeError
from gym import spaces
import pandas as pd

class Bebop2Env(robot_ros_env.RobotRosEnv):

    def __init__(self):
	self.image = None
        self.speed = 0.0
        self.error = 0.0
        self.last_error = 0.0

        # Define action and observation space
        self.actions = 89
        self.observations = 5

        self.kp = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
        self.kd = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]

        # Launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(Bebop2Env, self).__init__()

        # Start all the ROS related components
        self._image_sub = rospy.Subscriber("/bebop/image_raw", Image, self._camera_image_raw_callback)
        self._cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self._takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self._land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)

        self._check_all_sensors_ready()
        self._check_all_publishers_ready()
        rospy.logwarn("Bebop2 Environment initialized")

    def _check_all_sensors_ready(self):
        self._check_camera_image_raw_ready()
        
    def _check_camera_image_raw_ready(self):
        self.camera_image_raw = None
        rospy.logerr("Waiting for /bebop/image_raw to be ready...")
        while self.camera_image_raw is None and not rospy.is_shutdown():
            try:
                self.camera_image_raw = rospy.wait_for_message("/bebop/image_raw", Image, timeout=5.0)
                rospy.logdebug("/bebop/image_raw ready =>")
            except:
                rospy.logerr("/bebop/image_raw not ready yet, retrying...")
        return self.camera_image_raw
        
    def _camera_image_raw_callback(self, data):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, "bgr8")

        # filter by black
        gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        lower = np.array([0], dtype=np.uint8)
        upper = np.array([60], dtype=np.uint8)
        filtered = cv2.inRange(gray, lower, upper)

        self.image = filtered
    
    def _check_all_publishers_ready(self):
        self._check_cmd_vel_pub_connection()
        self._check_takeoff_pub_connection()
        self._check_land_pub_connection()

    def _check_cmd_vel_pub_connection(self):
        rate = rospy.Rate(10)
        while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logerr("/bebop/cmd_vel not ready yet, retrying...")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        rospy.logdebug("/bebop/cmd_vel publisher ready =>")
        
    def _check_takeoff_pub_connection(self):
        rate = rospy.Rate(10)
        while self._takeoff_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logerr("/bebop/takeoff not ready yet, retrying...")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        rospy.logdebug("/bebop/takeoff publisher ready =>")
        
    def _check_land_pub_connection(self):
        rate = rospy.Rate(10)
        while self._land_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logerr("/bebop/land not ready yet, retrying...")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        rospy.logdebug("/bebop/land publisher ready =>")

    def _compute_reward(self, observations, done):
        raise NotImplementedError()

    def _set_action(self, action):
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        raise NotImplementedError()
        
    def takeoff(self):
        takeoff_cmd = Empty()
        print("takeoff")

        self._check_takeoff_pub_connection()  
        self._takeoff_pub.publish(takeoff_cmd)
        self.wait_time_for_execute_movement()
        
    def land(self):
        land_cmd = Empty()
        print("land")
        
        self._check_land_pub_connection()
        self._land_pub.publish(land_cmd)
        self.wait_time_for_execute_movement()

    def move(self, action): 
        kpi, kdi = [int(x) for x in str(action).zfill(2)]
        print(str(kpi), str(kdi), str(self.error))
        y = -0.5*(self.kp[kpi] * self.error + self.kd[kdi] * (self.error - self.last_error));
        z = -0.75*(self.kp[kpi] * self.error + self.kd[kdi] * (self.error - self.last_error));

        print("Action Taken: ", str(y))

        velocity_cmd = Twist()
        velocity_cmd.linear.x  = self.speed
        velocity_cmd.linear.y  = y
        velocity_cmd.angular.z = z

        self._check_cmd_vel_pub_connection()
        self._cmd_vel_pub.publish(velocity_cmd)
        self.wait_time_for_execute_movement()
                                        
    def wait_time_for_execute_movement(self):
        time.sleep(0.1)
    
    def get_camera_image_raw(self):
        return self.camera_image_raw
