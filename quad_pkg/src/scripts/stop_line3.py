#!/usr/bin/env python
import rospy
import numpy as np
import cv2, cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class StopLine(object):
    def __init__(self):
        rospy.init_node("StopLine")
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("Predator", 1)

        rospy.Subscriber('camera2/usb_cam2/image_raw', Image, self._latestImage)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        
        self.max_linear_velocity = 0.5
        self.last_cmd_linear_x = 0
        self.last_state = 0
        self.line_detected = 0
        self.time_start = rospy.Time.now()

        rospy.loginfo("Ready to get out there and stop at the horizontal lines!")
        rospy.spin()

    def _latestImage(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        dark_blue = np.array([150,90,50])
        bright_blue = np.array([255,150,100])
        mask = cv2.inRange(image, dark_blue, bright_blue)
        mask[0:400, 0:width] = 0
        mask[460:height, 0:width] = 0
        mask[400:460, 0:220] = 0
        mask[400:460, 420:width] = 0
        (height, width, depth) = image.shape
        top_level = height / 2
        mask[0:top_level, 0:width] = 0

        Moments = cv2.moments(mask)
        if Moments['m00'] > 0:
            self.line_detected = 1
        else:
            self.line_detected = 0    
        cv2.imshow("The Good Samaritan", image)
        cv2.imshow("What he views", mask)
        cv2.waitKey(2)
        
        if abs(self.last_state - self.line_detected) > 0:
            self.time_start = rospy.Time.now()
        rospy.loginfo(self.last_state - self.line_detected)
        cmd = Twist()
        time_elapsed = rospy.Time.now() - self.time_start
        # Adjust thresholds 3,1 to get as close to line as we can
        if (time_elapsed < rospy.Duration.from_sec(3)) and (time_elapsed > rospy.Duration.from_sec(0.8)):
            cmd.linear.x = 0
        else:
            cmd.linear.x = self.max_linear_velocity
        #self.pub.publish(cmd)
        self.last_state = self.line_detected

if __name__ == "__main__":
    try:
        run = StopLine()
    except rospy.ROSInterruptException:
        pass
