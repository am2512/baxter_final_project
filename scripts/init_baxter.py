#!/usr/bin/env python


import baxter_interface
import rospy

from baxter_interface import CHECK_VERSION
from baxter_interface.camera import CameraController

import std_srvs.srv


class BaxterCtrls():

    def __init__(self):

        # Initialize actions of note
        self.cmds = baxter_interface.RobotEnable(CHECK_VERSION)

        # Reset the cameras
        self.reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
        rospy.wait_for_service('cameras/reset', timeout=10)
        self.reset_srv()

        self.head_cam = CameraController("head_camera")
        self.lh_cam = CameraController("left_hand_camera")


    def enableBaxter(self):

        # Print Info message detailing that Baxter is now enabled (unless there are other issues)
        rospy.loginfo("INIT - Issuing 'Enable' command to Baxter.")

        # Issue enabling command to Baxter
        # Kick out error message if an exception is encountered
        try:
            self.cmds.enable()
        except Exception, e:
            rospy.logerr(e.strerror)

        return


    def disableBaxter(self):

        # Print Info message detailing that Baxter is now disabled
        rospy.loginfo("INIT - Issuing 'Disable' command to Baxter.")

        # Issue disabling command to Baxter
        # Kick out error message if an exception is encountered
        try:
            self.cmds.disable()
        except Exception, e:
            rospy.logerr(e.strerror)

        return


    def cameraSetupHeadLH(self):

        # Print Info message describing Camera actions
        rospy.loginfo("INIT - Enabling 'head' and 'left_hand' cameras w/ output image resolution of 1280x800.")

        rospy.wait_for_service('cameras/list', timeout=10)

        # Open "head" camera with 1280x800 resolution
        self.head_cam.resolution = (1280,800)
        self.head_cam.open()

        # Open "left_hand" camera with 1280x800 resolution
        self.lh_cam.resolution = (1280,800)
        self.lh_cam.open()

        return


    def setScreenImage(self):

        return