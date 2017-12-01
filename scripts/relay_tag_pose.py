#!/usr/bin/env python


import rospy

from ar_track_alvar.msg import (
    AlvarMarkers,
    AlvarMarker
)
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
)
from std_msgs.msg import (
    Bool,
    Header,
    String
)


class poseHandler():

    def __init__(self):

        # Publishers and Subscribers
        self.capture_pose = rospy.Subscriber("update_pose", Bool, self.cbCapturePoseAndUpdate)
        self.pose_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.cbRegisterObjPose)

        self.last_pose = rospy.Publisher("object_pose", PoseStamped, queue_size=10)

        # Local data structures
        self.obj_pose_head = Header
        self.last_obj_pose_head = Header

        self.obj_pose = PoseStamped
        self.last_obj_pose = PoseStamped

    def cbCapturePoseAndUpdate(self, update):

        # Based on state of "update_pose" bool, trigger publishing of new pose for detected objects in the world
        # "update_pose" bool will de-assert after subscribing function receives data
        if (update):

            self.obj_pose_head = self.pose_sub.header
            self.obj_pose = self.pose_sub.markers[0]

            last_pose.publish(object_pose)

        else:
            pass

    def cbRegisterObjPose(self, data):

        self.last_obj_pose_head = data.header
        self.last_obj_pose = data.markers[0]


def main():

    rospy.init_node("pose_relay")

    poseHandler()

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()