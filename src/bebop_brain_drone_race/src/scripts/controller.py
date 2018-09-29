#!/usr/bin/env python

#This is what will determine the bebop drone's direction and velocity from line error and EEG scans.
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class Controller():
    def __init__(self):
        rospy.init_node("controller", anonymous=True)
        rospy.Subscriber("error", Float32, self.move_y)
        self.takeoff_pub = rospy.Publisher("takeoff", Empty, queue_size=10)
        self.land_pub = rospy.Publisher("land", Empty, queue_size=10)
        self.velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.velocity = Twist()

    def takeoff(self):
        print "takeoff"
        message = Empty()
        self.takeoff_pub.publish(message)

    def land(self):
        print "land"
        message = Empty()
        self.land_pub.publish(message)

    def move_y(self, data):
    if (data.data < 0):
        self.velocity.linear.y = 0.25
    else:
        self.velocity.linear.y = -0.25
        self.velocity_pub.publish(self.velocity)
    
    def move_x(self):
        self.velocity.linear.x = 1.0
        self.velocity_pub.publish(self.velocity)
        time.sleep(1)
        self.velocity.linear.x = 0.0
        self.velcity_pub.publish(self.velocity)

if __name__ == "__main__":
    bebop_controller = Controller()
    usr_input = ""

    while usr_input != "q":
        usr_input = raw_input("Takoff(t), Land(l), Move Forward(f), or Quit(q): ")
        
        if usr_input.lower() == "t":
            bebop_controller.takeoff()
        elif usr_input.lower() == "l":
            bebop_controller.land()
        elif usr_input.lower() == "f":
            bebop_controller.move_x()
        elif usr_input.lower() == "q":
            bebop_controller.land()
            break
        else:
            break
