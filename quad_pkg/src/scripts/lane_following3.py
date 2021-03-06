#!/usr/bin/env python
import rospy
import numpy as np
import cv2, cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class LaneFollow(object):
    def __init__(self):
        rospy.init_node("LaneFollow")
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("Predator", 1)

        rospy.Subscriber('camera1/usb_cam1/image_raw', Image, self._latestImage)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.last_cmd_linear_x = 0

        rospy.loginfo("Ready to get out there and follow the lane!")
        rospy.spin()

    def _latestImage(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        dark_blue = np.array([150,90,50])
        bright_blue = np.array([255,150,100])
        mask = cv2.inRange(image, dark_blue, bright_blue)

        (height, width, depth) = image.shape
        mask[0:-40, 0:width] = 0
        Moments = cv2.moments(mask)
        leftMoments = cv2.moments(mask[:,0:width/2]
        rightMoments = cv2.moments(mask[:,width/2+1:-1])
 
        if Moments['m00'] > 0:
            centroid_x = int(Moments['m10'] / Moments['m00'])
            centroid_y = int(Moments['m01'] / Moments['m00'])
        if leftMoments['m00'] > 0:
            left_centroid_x = int(Moments['m10'] / Moments['m00'])
        if rightMoments['m00'] > 0:
            right_centroid_x = int(Moments['m10'] / Moments['m00'])

        cmd = Twist()

        detected_left = 1 if leftMoments['m00']>0 else 0
        detected_right = 1 if rightMoments['m00']>0 else 0

        pattern = str(detected_left) + str(detected_right)
        rospy.loginfo(pattern)

        if pattern == '11':
            focus = 0.01* (width/2 - centroid_x - 10)
            cmd.linear.x = np.clip(self.last_cmd_linear_x+0.01,0,1)
            cmd.angular.z = np.clip((focus),-2, 2)

        elif pattern == '01':
            focus = 0.01* (0.25*width - right_centroid_x -10)
            cmd.linear.x = np.clip(self.last_cmd_linear_x+0.01,0,1)
            cmd.angular.z = np.clip((focus),-2, 2)

        elif pattern == '10':
            focus = 0.01* (0.75*width - left_centroid_x -10)
            cmd.linear.x = np.clip(self.last_cmd_linear_x+0.01,0,1)
            cmd.angular.z = np.clip((focus),-2, 2)

        else:
            cmd.linear.x = 0
            cmd.angular.z = -0.2

        self.last_cmd_linear_x = cmd.linear.x

        rospy.loginfo(cmd)
        self.pub.publish(cmd)
        cv2.imshow("Predator", image[-80:-1,:,:])
        cv2.waitKey(2)   

if __name__ == "__main__":
    try:
        run = LaneFollow()
    except rospy.ROSInterruptException:
        pass
