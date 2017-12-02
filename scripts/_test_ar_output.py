#!/usr/bin/env python


import rospy

from ar_track_alvar_msgs.msg import (
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
        self.pose_sub = rospy.Subscriber("/ar_trackers/ar_pose_marker", AlvarMarkers, self.cbRegisterObjPose)
        self.last_pose = rospy.Publisher("object_pose", Pose, queue_size=10)

        self.last_obj_pose_head = Header
        self.last_obj_pose = PoseStamped


    def cbRegisterObjPose(self, data):

        self.last_obj_pose_head = data.header
        self.last_obj_pose = data.markers[0].pose.pose

        rospy.loginfo("TEST")

        self.last_pose.publish(self.last_obj_pose)

        return


def main():

    rospy.init_node("pose_relay")

    poseHandler()

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()