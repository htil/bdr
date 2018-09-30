#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time 

class Controller():
    def __init__(self):
        rospy.init_node("controller", anonymous=True)
        rospy.Subscriber("/bebop/error", Float32, self.move_y)
        self.takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        self.velocity_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
        self.velocity = Twist()
        self.in_air = False

    def takeoff(self):
	self.in_air = True
        message = Empty()
        self.takeoff_pub.publish(message)

    def land(self):
	self.in_air = False
        message = Empty()
        self.land_pub.publish(message)

    def move_y(self, data):
        if self.in_air:
            if data.data < 0:
                self.velocity.linear.y = -0.10
                self.velocity_pub.publish(self.velocity)
            elif data.data > 0:
                self.velocity.linear.y = 0.10
                self.velocity_pub.publish(self.velocity)
            else:
                self.velocity.linear.y = 0
                self.velocity_pub.publish(self.velocity)
    
    def move_x(self):
        self.velocity.linear.x = 1.0
        self.velocity_pub.publish(self.velocity)
        time.sleep(1)
        self.velocity.linear.x = 0.0
        self.velocity_pub.publish(self.velocity)

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
