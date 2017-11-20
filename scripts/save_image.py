#!/usr/bin/env python


import cv2
import rospy

from cv_bridge import  CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ArmCameraHandler():

    def __init__(self):
        self.lh_image_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.callback)
        # self.rh_image_sub = rospy.Subscriber("/cameras/right_hand_camera/image", Image, self.callback)
        self.bridge = CvBridge()


    def callback(self, imageData):

        try:
            cv_img = self.bridge.imgmsg_to_cv2(imageData, "bgr8")
        except CvBridgeError as err:
            print err

        image_path = "image_test.png"

        rospy.loginfo("Saving Image from Baxter to {0}".format(image_path))
        cv2.imwrite(image_path, cv_img)


def main():

    ArmCameraHandler()
    rospy.init_node('RnD_image_siphon')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':

    main()
