#!/usr/bin/env python

# Use joystick input to launch behavioral nodes in jackal
#
# Intro to Robotics - EE5531 - Spring 2018
#          Final Project
#
#             Group #1
#             Narendra
#              Alex
#             Shivam
#
# version: v1.5

# define imports
import rospy
import roslaunch
import sys
import time
import os
import numpy as np
from sensor_msgs.msg import Joy, LaserScan

# class to read joystick messages and launch node
class joy_control(object):

    # define self routine
    def __init__(self):

        # define subscriber
        rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.joy_callback)
        rospy.Subscriber("/scan",LaserScan, self.laser_callback)
        rate = rospy.Rate(5)

        rospy.loginfo('started joystick routine..')

        # define and init variables
        self.person_following_start = False
        self.person_following_stop  = False
        self.wall_following_start = False
        self.wall_following_stop  = False
        self.lane_following_start = False
        self.lane_following_stop  = False
        self.stop_line_start = False
        self.stop_line_stop  = False

        # configure node roslaunch api
        package    = 'quad_pkg'
        executable_person_following = 'person_following3.py'
        node_person_following = roslaunch.core.Node(package, executable_person_following)
        executable_wall_following = 'wall_following.py'
        node_wall_following = roslaunch.core.Node(package, executable_wall_following)
        executable_stop_line = 'stop_line2.py'
        node_stop_line = roslaunch.core.Node(package, executable_stop_line)
        executable_lane_following = 'lane_following2.py'
        node_lane_following = roslaunch.core.Node(package, executable_lane_following)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        while not rospy.is_shutdown():
            # if start flag set: launch main launch-file
            if self.person_following_start:
                person_following_process = launch.launch(node_person_following)

            # if stop flag set: shutdown main launch-file
            if self.person_following_stop:
                if 'person_following_process' in locals():
                    person_following_process.stop()

            # if start flag set: launch main launch-file 
            if self.wall_following_start:
                wall_following_process = launch.launch(node_wall_following)

            # if stop flag set: shutdown main launch-file
            if self.wall_following_stop:
                if 'wall_following_process' in locals():
                    wall_following_process.stop()

            # if start flag set: launch main launch-file
            if self.lane_following_start:
                lane_following_process = launch.launch(node_lane_following)

            # if stop flag set: shutdown main launch-file
            if self.lane_following_stop:
                if 'lane_following_process' in locals():
                    lane_following_process.stop()

            # if start flag set: launch main launch-file
            if self.stop_line_start:
                stop_line_process = launch.launch(node_stop_line)

            # if stop flag set: shutdown main launch-file
            if self.stop_line_stop:
                if 'stop_line_process' in locals():
                    stop_line_process.stop()
            
            # reset trigger
            self.person_following_start = False
            self.person_following_stop  = False
            self.wall_following_start = False
            self.wall_following_stop  = False
            self.lane_following_start = False
            self.lane_following_stop  = False
            self.stop_line_start = False
            self.stop_line_stop  = False
            rate.sleep()


    # joystick callback routine
    def joy_callback(self, data):

        # define joystick buttons
        x, circ, sq, tri, L1, R1, share, options, p4, L3, R3, DL, DR, DU, DD = data.buttons
        llr, lud, L2, rlr, rud, R2 = data.axes

        # Start person following
        if (tri == 1) and (self.person_following_start == False):
            rospy.loginfo("Starting the person following routine...")
            # set the start flag
            self.person_following_start = True

        # Start wall following
        if (sq == 1) and (self.wall_following_start == False):
            rospy.loginfo("Starting the wall following routine...")
            # set the start flag
            self.wall_following_start = True

        # Start lane following
        if (circ == 1) and (self.lane_following_start == False):
            rospy.loginfo("Starting the lane following routine...")
            # set the start flag
            self.lane_following_start = True

        # Start stop line
        if (x == 1) and (self.stop_line_start == False):
            rospy.loginfo("Starting the stop line routine...")
            # set the start flag
            self.stop_line_start = True

        # Terminate everything running and return to manual
        if (R1 == 1):
            rospy.loginfo("Terminating the predator routine...")
            # set stop flag
            self.person_following_stop  = True
            self.wall_following_stop  = True
            self.lane_following_stop  = True
            self.stop_line_stop  = True

    def laser_callback(self,data):
        if (self.stop_line_stop ==False) and (np.min(np.asarray(data.ranges[250:450])) <=0.5):
            rospy.loginfo("Terminating stop line")
            self.stop_line_stop = True

if __name__ == "__main__":
    try:
        rospy.init_node("joy_start", anonymous=False)
        run = joy_control()
    except rospy.ROSInterruptException:
        rospy.loginfo("joy_start node terminated.")
