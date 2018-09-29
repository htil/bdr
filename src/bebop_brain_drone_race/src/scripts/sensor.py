#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from LineError import LineError
from cv_bridge import CvBridge, CvBridgeError

class Sensor():
    def __init__(self):
        rospy.init_node("sensor", anonymous=True)
        rospy.Subscriber("/bebop/image_raw", Image, self.calculate_error)
        self.error_publisher = rospy.Publisher("/bebop/error", Float32, queue_size=10) 
        self.error = 0
	self.line_error = LineError()
    
    def calculate_error(self, image):
        # Convert bebop image to compatible OpenCv image type
        bridge = CvBridge()
        try:
            img = bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError, e:
            print(e)

        self.error = self.line_error.GetError(img)

    def run(self):  
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.error_publisher.publish(self.error)
            print self.error
            rate.sleep()

if __name__ == "__main__":  
    sensor = Sensor()
    sensor.run()
