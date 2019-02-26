import gym
import rospy
import sys
import os
import signal
import subprocess
import time
from std_srvs.srv import Empty
import random
from rosgraph_msgs.msg import Clock

class GazeboEnv(gym.Env):
    """
    Superclass for all Gazebo environments.
    """
    metadata = {'render.modes': ['human']}

    def __init__(self, launchfile):
        self.last_clock_msg = Clock()

        random_number = random.randint(10000, 15000)
        self.port = str(random_number)
        self.port_gazebo = str(random_number+1)

        os.environ["ROS_MASTER_URI"] = "http://localhost:"+self.port
        os.environ["GAZEBO_MASTER_URI"] = "http://localhost:"+self.port_gazebo

        print("ROS_MASTER_URI=http://localhost:"+self.port + "\n")
        print("GAZEBO_MASTER_URI=http://localhost:"+self.port_gazebo + "\n")

        ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))

        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets", "launch", launchfile)
        if not os.path.exists(fullpath):
            raise IOError("File "+fullpath+" does not exist")

        self._roslaunch = subprocess.Popen([sys.executable, os.path.join(ros_path, b"roslaunch"), "-p", self.port, fullpath])
        print ("Gazebo launched!")

        self.gzclient_pid = 0

        # Launch the simulation with the given launchfile name
        rospy.init_node('gym', anonymous=True)

    def step(self, action):
        # implement this method in every subclass
        raise NotImplementedError

    def reset(self):
         # implement this method in every subclass
        raise NotImplementedError

    def _render(self, mode="human", close=False):
        if close:
            tmp = os.popen("ps -Af").read()
            proccount = tmp.count('gzclient')
            if proccount > 0:
                if self.gzclient_pid != 0:
                    os.kill(self.gzclient_pid, signal.SIGTERM)
                    os.wait()
            return

        tmp = os.popen("ps -Af").read()
        proccount = tmp.count('gzclient')
        if proccount < 1:
            subprocess.Popen("gzclient")
            self.gzclient_pid = int(subprocess.check_output(["pidof","-s","gzclient"]))
        else:
            self.gzclient_pid = 0

    def _close(self):
        # kill gzclient, gzserver and roscore
        tmp = os.popen("ps -Af").read()
        gzclient_count = tmp.count('gzclient')
        gzserver_count = tmp.count('gzserver')
        roscore_count = tmp.count('roscore')
        rosmaster_count = tmp.count('rosmaster')

        if gzclient_count > 0:
            os.system("killall -9 gzclient")
        if gzserver_count > 0:
            os.system("killall -9 gzserver")
        if rosmaster_count > 0:
            os.system("killall -9 rosmaster")
        if roscore_count > 0:
            os.system("killall -9 roscore")

        if (gzclient_count or gzserver_count or roscore_count or rosmaster_count >0):
            os.wait()

    def _configure(self):
        pass

    def _seed(self):    
        pass

# completed: lcb 02/26/19 11:47:00 AM