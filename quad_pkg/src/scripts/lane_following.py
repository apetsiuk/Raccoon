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

        self.person_distance = 1.1
        self.last_cmd_linear_x = 0
        self.last_cmd_angular_z = 0

        rospy.loginfo("Ready to get out there and follow the lane!")
        rospy.spin()

    def _latestImage(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        dark_blue = np.array([150,90,50])
        bright_blue = np.array([255,150,100])
        mask = cv2.inRange(image, dark_blue, bright_blue)

        (height, width, depth) = image.shape
        mask[0:-40, 0:width] = 0

        cmd = Twist()

        Moments = cv2.moments(mask)
        if Moments['m00'] > 0:
            centroid_x = int(Moments['m10'] / Moments['m00'])
            centroid_y = int(Moments['m01'] / Moments['m00'])
            #Create Predator like representation of where the focus is
            cv2.circle(image, (centroid_x-5, centroid_y+3), 3, (10,10,255), -1)
            cv2.circle(image, (centroid_x+5, centroid_y+3), 3, (10,10,255), -1)
            cv2.circle(image, (centroid_x, centroid_y-3), 3, (10,10,255), -1)
            cmd.linear.x = np.clip(self.last_cmd_linear_x+0.008,0,1.2)
            cmd.angular.z = np.clip((0.01 * (width / 2 - centroid_x)-0.02),-1, 1)
        else:
            cmd.linear.x = 0
            cmd.angular.z = np.clip(self.last_cmd_angular_z + 0.08, 0, 1)
        self.last_cmd_linear_x = cmd.linear.x
        self.last_cmd_angular_z = cmd.angular.z
        rospy.loginfo(cmd)
        self.pub.publish(cmd)
        cv2.imshow("Predator", image[-80:-1,:,:])
        cv2.waitKey(2)   

if __name__ == "__main__":
    try:
        run = LaneFollow()
    except rospy.ROSInterruptException:
        pass
