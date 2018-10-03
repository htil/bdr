#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from pynput import keyboard

class Controller():
    def __init__(self):
        rospy.init_node("controller", anonymous=True)
        rospy.Subscriber("/bebop/error", Float32, self.fix_error)
        self.takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        self.velocity_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
        self.velocity = Twist()
        self.in_air = False
        self.Kp_y = 0.0002
        self.Kp_z = 0.05

    def takeoff(self):
        self.in_air = True
        message = Empty()
        self.takeoff_pub.publish(message)

    def land(self):
        self.in_air = False
        message = Empty()
        self.land_pub.publish(message)

    def fix_error(self, data):
        if self.in_air:
                # linear
                self.velocity.linear.y = -(self.Kp_y * data.data[0])
                if self.velocity.linear.y > 0.1:
                    self.velocity.linear.y = 0.1
                elif self.velocity.linear.y < -0.1:
                    self.velocity.linear.y = -0.1

                # yaw
                self.velocity.angular.z = -(self.Kp_z * data.data[1])
                print(self.velocity.angular.z)
                if self.velocity.angular.z > 0.1:
                    self.velocity.angular.z = 0.1
                elif self.velocity.angular.z < -0.1:
                    self.velocity.angular.z = -0.1

                self.velocity_pub.publish(self.velocity)

    def on_press(self, key):
        try:
            if key.char == "w":
                self.velocity.linear.x = 0.1
                self.velocity_pub.publish(self.velocity)
            elif key.char == "s":
                self.velocity.linear.x = -0.1
                self.velocity_pub.publish(self.velocity)
            elif key.char == "a":
                self.velocity.angular.z = -0.1
                self.velocity_pub.publish(self.velocity)
            elif key.char == "d":
                self.velocity.angular.z = 0.1
                self.velocity_pub.publish(self.velocity)
            elif key.char == "t":
                self.takeoff()
            elif key.char == "l":
                self.land()

        except AttributeError:
            pass

    def on_release(self, key):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.velocity_pub.publish(self.velocity)

        if key == keyboard.Key.esc:
            # Stop listener and land drone
            self.land()
            return False

if __name__ == "__main__":
    bebop_controller = Controller()

    # Collect keyboard events until released
    with keyboard.Listener(
        on_press=bebop_controller.on_press,
        on_release=bebop_controller.on_release) as listener:
        listener.join()
