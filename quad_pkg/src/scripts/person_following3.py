#!/usr/bin/env python
import rospy
import numpy as np
import cv2, cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image,LaserScan

class PersonFollow(object):
    def __init__(self):
        rospy.init_node("PersonFollow")
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("The Silent Stalker", 1)

        sub_camera = rospy.Subscriber('camera1/usb_cam1/image_raw', Image, self._latestCommonCallback)
        sub_lidar = rospy.Subscriber('/scan', LaserScan, self._latestCommonCallback)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.person_distance = 0.2
        self.last_cmd_linear_x = 0
        self.last_cmd_angular_z = 0
        self.detected_by_camera = 0
        self.detected_by_lidar = 0
        self.centroid_x = 320
        self.angular_movement = 0
        self.cmd = Twist()
        self.too_close = 0
        rospy.loginfo("Ready to get out there and follow Alex or Narendra or any one of the awesome guys!")
        rospy.spin()

    def _latestCommonCallback(self,data):
        input_message_type = str(data._type)
        if input_message_type == "sensor_msgs/Image":
            image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            dark_blue = np.array([150,90,50])
            bright_blue = np.array([255,150,100])
            mask = cv2.inRange(image, dark_blue, bright_blue)

            (height, width, depth) = image.shape
            top_level = height / 2
            mask[0:top_level, 0:width] = 0

            Moments = cv2.moments(mask)
            if Moments['m00'] > 0:
                self.detected_by_camera = 1
                self.centroid_x = int(Moments['m10'] / Moments['m00'])
                centroid_y = int(Moments['m01'] / Moments['m00'])
                self.angular_movement = np.clip((0.01 * (width / 2 - self.centroid_x)-0.2),-1, 1)
            else:
                self.detected_by_camera = 0
                self.angular_movement = 0.2
            cv2.imshow("The Silent Stalker", image)
            cv2.waitKey(2)
        if input_message_type == "sensor_msgs/LaserScan":
            front_range_cone = np.asarray(data.ranges[200:500])

            if np.min(front_range_cone) < 1.5:
                self.detected_by_lidar = 1
            else:
                self.detected_by_lidar = 0

            if np.min(front_range_cone) < 0.5:
                self.too_close = 1
            else:
                self.too_close = 0
        pattern = str(self.detected_by_camera) + str(self.detected_by_lidar) + str(self.too_close)
        if pattern == '110':
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = self.angular_movement
        else:
            self.cmd.linear.x = 0
            self.cmd.angular.z = 0.3
        rospy.loginfo(self.cmd.angular.z)
        self.pub.publish(self.cmd)
    

if __name__ == "__main__":
    try:
        run = PersonFollow()
    except rospy.ROSInterruptException:
        pass
