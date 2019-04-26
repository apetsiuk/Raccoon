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
        kernelOpen=np.ones((5,5))
        kernelClose=np.ones((20,20))
        font=cv2.cv.InitFont(cv2.cv.CV_FONT_HERSHEY_SIMPLEX,2,0.5,0,3,1)
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        dark_blue = np.array([150,90,50])
        bright_blue = np.array([255,150,100])
        mask = cv2.inRange(image, dark_blue, bright_blue)

        (height, width, depth) = image.shape

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(image, dark_blue, bright_blue)
        mask[0:height/2, 0:width] = 0
        mask[height/2 + 1: -1, 0:width/3] = 0
        mask[height/2 + 1: -1, 2*width/3:width] = 0
        
        #Morphology
        maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

        conts,h=cv2.findContours(maskClose.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

        for i in range(len(conts)):
            x,y,w,h=cv2.boundingRect(conts[i])
            if w*h > 1500:
                cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,255), 2)
                self.line_detected = 1
            else:
                self.line_detected = 0
        cv2.imshow("The Good Samaritan", image)
        cv2.imshow("What he views", mask)
        cv2.waitKey(2)
        
        if abs(self.last_state - self.line_detected) > 0:
            self.time_start = rospy.Time.now()
        rospy.loginfo(len(conts))
        cmd = Twist()
        time_elapsed = rospy.Time.now() - self.time_start
        # Adjust thresholds 3,1 to get as close to line as we can
        if (time_elapsed < rospy.Duration.from_sec(2.5)) and (time_elapsed > rospy.Duration.from_sec(0.5)):
            cmd.linear.x = 0
        else:
            cmd.linear.x = self.max_linear_velocity
        self.pub.publish(cmd)
        self.last_state = self.line_detected

if __name__ == "__main__":
    try:
        run = StopLine()
    except rospy.ROSInterruptException:
        pass
