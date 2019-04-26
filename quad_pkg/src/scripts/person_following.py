#!/usr/bin/env python
import rospy
import numpy as np
import cv2, cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class PersonFollow(object):
    def __init__(self):
        rospy.init_node("PersonFollow")
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("Predator", 1)

        rospy.Subscriber('camera2/usb_cam2/image_raw', Image, self._latestImage)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.person_distance = 1.1
        self.last_cmd_linear_x = 0
        self.last_cmd_angular_z = 0

        rospy.loginfo("Ready to get out there and follow Alex or Narendra or any one of the awesome guys!")
        rospy.spin()

    def _latestImage(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        dark_blue = np.array([150,90,50])
        bright_blue = np.array([255,150,100])
        mask = cv2.inRange(image, dark_blue, bright_blue)

        (height, width, depth) = image.shape
        top_level = height / 2
        mask[0:top_level, 0:width] = 0

        cmd = Twist()

        Moments = cv2.moments(mask)
        if Moments['m00'] > 0:
            centroid_x = int(Moments['m10'] / Moments['m00'])
            centroid_y = int(Moments['m01'] / Moments['m00'])
            #Create Predator like representation of where the focus is
            cv2.circle(image, (centroid_x-5, centroid_y+3), 3, (10,10,255), -1)
            cv2.circle(image, (centroid_x+5, centroid_y+3), 3, (10,10,255), -1)
            cv2.circle(image, (centroid_x, centroid_y-3), 3, (10,10,255), -1)
            cmd.linear.x = np.clip(self.last_cmd_linear_x+0.004,0,1.2)
            cmd.angular.z = np.clip((0.01 * (width / 2 - centroid_x)-0.03),-1.2, 1.2)
        else:
            cmd.linear.x = np.clip(self.last_cmd_linear_x - 0.004, 0,1.2)
            cmd.angular.z = np.clip(self.last_cmd_angular_z + 0.1, 0, 1.2)
        self.last_cmd_linear_x = cmd.linear.x
        self.last_cmd_angular_z = cmd.angular
        rospy.loginfo(cmd)
        self.pub.publish(cmd)
        cv2.imshow("Predator", image)
        cv2.waitKey(2)   

if __name__ == "__main__":
    try:
        run = PersonFollow()
    except rospy.ROSInterruptException:
        pass
