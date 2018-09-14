#!/usr/bin/env python

#This is what will determine the bebop drone's direction and velocity from line error and EEG scans.
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

class Controller():
    def __init__(self):
        rospy.init_node("controller", anonymous=True)
        self.takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=10)

    def takeoff(self):
        print "takeoff"
        message = Empty()
        self.takeoff_pub.publish(message)

    def land(self):
        print "land"
        message = Empty()
        self.land_pub.publish(message)

if __name__ == "__main__":
    bebop_controller = Controller()
    usr_input = ""

    while usr_input != "q":
        usr_input = raw_input("Takoff(t), Land(l), or Quit(q): ")
        
        if usr_input.lower() == "t":
            bebop_controller.takeoff()
        elif usr_input.lower() == "l":
            bebop_controller.land()
        elif usr_input.lower() == "q":
            bebop_controller.land()
            break
        else:
            break