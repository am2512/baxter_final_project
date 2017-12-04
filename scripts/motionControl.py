#!/usr/bin/env python


import rospy
import baxter_interface

from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)
from std_msgs.msg import Header

from std_srvs.srv import Trigger
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)


class motionControl():

    def __init__(self):

        # Publishers and Subscribers
        self.obj_pose_sub = rospy.Subscriber("/z_controls/object_pose", Pose, self.cbSetTagPosition)

        # Services
        self.move_to_AR_tag = rospy.Service('move_to_AR_tag', Trigger, self.svcMoveToARTag)
        self.move_to_offset_pos = rospy.Service('move_to_offsetpos', Trigger, self.svcMoveToOffsetPos)

        # Static configuration variables
        self.limb = 'left'


    def cbSetTagPosition(self, data):

        # New values that are obtained from the published object pose
        self.x_tag = data.position.x
        self.y_tag = data.position.y
        self.z_tag = data.position.z

        return


    def svcMoveToARTag(self, data):

        # Establish connection to specific limb's IKSolver service
        ns = "ExternalTools/" + self.limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Update header information based on current time and with reference to base frame
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        poses = {
            'left': PoseStamped(
                header = hdr,
                pose = Pose(
                    position = Point(
                        x = self.x_tag,
                        y = self.y_tag,
                        z = self.z_tag,
                    ),
                    orientation = Quaternion(
                        x = -0.159821886749,
                        y = 0.984127886766,
                        z = -0.00293647198525,
                        w = 0.0770755742012,
                    ),
                ),
            ),
            'right': PoseStamped(
                header = hdr,
                pose = Pose(
                    position = Point(
                        x = self.x_tag,
                        y = self.y_tag,
                        z = self.z_tag,
                    ),
                    orientation = Quaternion(
                        x = 0.367048116303,
                        y = 0.885911751787,
                        z = -0.108908281936,
                        w = 0.261868353356,
                    )
                )
            )
        }


        ikreq.pose_stamp.append(poses[self.limb])

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        if (resp.isValid[0]):
            print("SUCCESS - Valid Joint Solution Found:")
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            # print limb_joints

            if (self.limb == 'left'):
                left = baxter_interface.Limb('left')
                left.move_to_joint_positions(limb_joints)
            elif (self.limb == 'right'):
                right = baxter_interface.Limb('right')
                right.move_to_joint_positions(limb_joints)
            else:
                pass
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")

        return (True, "Complete")


    def svcMoveToOffsetPos(self, data):

        return


def main():

    rospy.init_node("motion_controller")

    motionControl()

    while not rospy.is_shutdown():
        rospy.spin()

    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
