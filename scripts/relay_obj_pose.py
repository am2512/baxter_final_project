#!/usr/bin/env python


import rospy

from ar_track_alvar_msgs.msg import (AlvarMarkers, AlvarMarker)
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)
from std_msgs.msg import (Bool, Header, String, UInt32)

from std_srvs.srv import Trigger


class poseHandler():

    def __init__(self):

        # Static configuration variables
        self.update_rate = rospy.Rate(5)

        # Publishers and Subscribers
        self.pose_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.cbRegisterObjPose)
        self.obj_pose = rospy.Publisher('object_pose', Pose, queue_size=1)
        # self.obj_ID = rospy.Publisher('object_ID', UInt32, queue_size=1)  REMOVED FOR NOW

        # Services
        self.update_obj_pose = rospy.Service('update_obj_pose', Trigger, self.cbUpdatePublishedPose)

        # Variables for latest subscribed poses and actively published poses
        self.last_obj_id = UInt32[]
        self.last_obj_pose = Pose[]

        self.pub_obj_id = UInt32[]
        self.pub_obj_pose = Pose[]


    def cbRegisterObjPose(self, data):

        for index in range(len(data.markers)):
            # self.last_obj_id[index] = data.markers[index].id  REMOVED FOR NOW
            self.last_obj_pose[index] = data.markers[index].pose.pose

        return


    def cbUpdatePublishedPose(self):

        for index in range(len(self.last_obj_pose)):
            # self.pub_obj_id[index] = self.last_obj_id[index]  REMOVED FOR NOW
            self.pub_obj_pose[index] = self.last_obj_pose[index]

        return True


def main():

    # Node initialization
    rospy.init_node('pose_relay')

    # Class initialization
    pose_relay = poseHandler()

    while not rospy.is_shutdown():
        if len(pose_relay.pub_obj_pose) != 0:
            for tag in range(len(pose_relay.pub_obj_pose)):
                # pose_relay.obj_ID.publish(pose_relay.pub_obj_id[tag])  REMOVED FOR NOW
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