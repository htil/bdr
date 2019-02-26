import gym 
import rospy
import os
import signal
import subprocess
import time
from os import path
from std_srvs.srv import Empty
import random

class RealEnv(gym.Env):
    """
    Superclass for all Gazebo environments.
    """
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.port = "11311"
        print ("Roscore launched!")

        # Launch the simulation with the given launchfile name
        rospy.init_node('gym', anonymous=True)

    def set_ros_master_uri(self):
        os.environ["ROS_MASTER_URI"] = self.ros_master_uri

    def step(self, action):
        # implement this method in every subclass
        raise NotImplementedError

    def reset(self):
        # implement this method in every subclass
        raise NotImplementedError

    def render(self, mode=None,  close=False):
        pass

    def _render(self, mode=None,  close=False):
        self._close()

    def _close(self):
        output1 = subprocess.check_call(["cat" ,"/tmp/myroslaunch_" + self.port + ".pid"])
        output2 = subprocess.check_call(["cat" ,"/home/erle/.ros/roscore-" + self.port + ".pid"])
        subprocess.Popen(["kill", "-INT", str(output1)])
        subprocess.Popen(["kill", "-INT", str(output2)])

    def close(self):
        pass

    def _configure(self):
        pass

    def _seed(self):
        pass

# completed: lcb 02/26/19 11:54:00 AM