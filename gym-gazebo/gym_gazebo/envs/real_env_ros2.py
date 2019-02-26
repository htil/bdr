import gym
import rclpy
import os
import signal
import subprocess
import time
from os import path
from std_srvs.srv import Empty
import random

class RealEnvROS2(gym.Env):
    """
    Superclass for all Gazebo environments.
    """
    metadata = {'render.modes': ['human']}

    def __init__(self):
        # launch the simulation with the given launchfile name
        rclpy.init(args=None)
        self.node = rclpy.create_node('real_env_ros2')

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

# completed: lcb 02/26/19 11:58:00 AM