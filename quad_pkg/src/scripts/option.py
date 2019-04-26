#!/usr/bin/env python

# Use joystick input to launch object-tracking nodes in jackal
#
# Intro to Robotics - EE5900 - Spring 2017
#          Assignment #6
#
#       Project #6 Group #2
#             Prithvi
#              Aswin
#         Akhil (Team Lead)
#
# version: v1.3

# define imports
import rospy
import roslaunch
import sys
import time
import os

from   sensor_msgs.msg import Joy

# class to read joystick messages and launch node
class joy_control(object):

    # define self routine
    def __init__(self):

        # define subscriber
        rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.joy_callback)
        rate = rospy.Rate(5)

        rospy.loginfo('started joystick routine..')

        # define and init variables
        self.person_following_start = False
        self.person_following_stop  = False
        self.wall_following_start = False
        self.wall_following_stop  = False

        # configure node roslaunch api
        package    = 'quad_pkg'
        executable_person_following = 'person_following.py'
        node_person_following = roslaunch.core.Node(package, executable_person_following)
        executable_wall_following = 'person_following.py'
        node_wall_following = roslaunch.core.Node(package, executable_wall_following)
        

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        while not rospy.is_shutdown():
            # if start flag set: launch main launch-file
            if self.person_following_start:
                #launch = roslaunch.scriptapi.ROSLaunch()
                #launch.start()
                person_following_process = launch.launch(node_person_following)

            # if stop flag set: shutdown main launch-file
            if self.person_following_stop:
                person_following_process.stop()

            if self.wall_following_start:
                #launch = roslaunch.scriptapi.ROSLaunch()
                #launch.start()
                wall_following_process = launch.launch(node_wall_following)

            # if stop flag set: shutdown main launch-file
            if self.wall_following_stop:
                wall_following_process.stop()

            # reset trigger
            self.person_following_start = False
            self.person_following_stop  = False
            self.wall_following_start = False
            self.wall_following_stop  = False
            rate.sleep()


    # joystick callback routine
    def joy_callback(self, data):

        # define joystick buttons
        x, circ, sq, tri, L1, R1, share, options, p4, L3, R3, DL, DR, DU, DD = data.buttons
        llr, lud, L2, rlr, rud, R2 = data.axes

        # Start object tracking
        if (circ == 1) and (self.person_following_start == False):
            rospy.loginfo("Starting the predator routine...")
            # set the start flag
            self.person_following_start = True

        # Stop tracking
        if (x == 1):
            rospy.loginfo("Terminating the predator routine...")
            # set stop flag
            self.person_following_stop  = True

        # Start object tracking
        if (sq == 1) and (self.wall_following_start == False):
            rospy.loginfo("Starting the predator routine 2...")
            # set the start flag
            self.wall_following_start = True

        # Stop tracking
        if (tri == 1):
            rospy.loginfo("Terminating the predator routine 2...")
            # set stop flag
            self.wall_following_stop  = True


# standard boilerplate
if __name__ == "__main__":
    try:
        rospy.init_node("joy_start", anonymous=False)
        #read in joystick input
        run = joy_control()
    except rospy.ROSInterruptException:
        rospy.loginfo("joy_start node terminated.")
