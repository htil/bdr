#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from LineError import LineError

class Sensor():
    def __init__(self):
        rospy.init_node("sensor", anonymous=True)
        rospy.Subscriber("image_raw", Image, self.calculate_error)
        self.error_publisher = rospy.Publisher("error", Float32) 
        self.error = 0
    
    def calculate_error(self, image):
        self.error = LineError.GetError(image.data)[1]

    def run(self):	
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.error_publisher.publish(self.error)
            rate.sleep()

if __name__ == "__main__":	
    sensor = Sensor()
    sensor.run()
