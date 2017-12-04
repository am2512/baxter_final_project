#!/usr/bin/env python


import rospy

from ar_track_alvar_msgs.msg import (AlvarMarkers, AlvarMarker)
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)

from std_srvs.srv import Trigger


class poseHandler():

    def __init__(self):

        # Publishers and Subscribers
        self.pose_sub = rospy.Subscriber('/z_ar_trackers/ar_pose_marker', AlvarMarkers, self.cbRegisterObjPose)
        self.obj_pose = rospy.Publisher('/z_controls/object_pose', Pose, queue_size=1)
        # self.obj_ID = rospy.Publisher('object_ID', UInt32, queue_size=1)  REMOVED FOR NOW

        # Services
        self.update_obj_pose = rospy.Service('update_obj_pose', Trigger, self.srvUpdatePublishedPose)

        # Static configuration variables
        self.update_rate = rospy.Rate(5)

        # Variables for latest subscribed poses and actively published poses
        self.last_obj_pose = []
        self.pub_obj_pose = []


    def cbRegisterObjPose(self, data):

        if (data.markers == []):
            pass
        else:
            self.last_obj_pose = data.markers

        return


    def srvUpdatePublishedPose(self, data):

        if (self.last_obj_pose == []):
            return (False, "POSE HANDLER - No AR Tags in View")
        else:
            self.pub_obj_pose = []

            for index in range(len(self.last_obj_pose)):
                self.pub_obj_pose.append(self.last_obj_pose[index].pose.pose)

            return (True, "POSE HANDLER - Update Complete")


def main():

    # Node initialization
    rospy.init_node('pose_relay')

    # Class initialization
    pose_relay = poseHandler()

    while not rospy.is_shutdown():
        if len(pose_relay.pub_obj_pose) != 0:
            for tag in range(len(pose_relay.pub_obj_pose)):
                pose_relay.obj_pose.publish(pose_relay.pub_obj_pose[tag])
        else:
            pass

        # Publish new pose values at 5 Hz (provided poses have been 'captured')
        pose_relay.update_rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass