#!/usr/bin/env python


import rospy
import baxter_interface

from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
from me495_baxter_jar.srv import OffsetMove
from std_srvs.srv import Trigger




class motionControl():

    def __init__(self):

        # Publishers and Subscribers
        self.obj_pose_sub = rospy.Subscriber('/z_controls/object_pose', Pose, self.cbSetTagPosition)

        # Services
        self.move_to_AR_tag = rospy.Service('move_to_AR_tag', Trigger, self.svcMoveToARTag)
        self.move_to_offset_pos = rospy.Service('move_to_offset_pos', OffsetMove, self.svcMoveToOffsetPos)

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
        ns = 'ExternalTools/' + self.limb + '/PositionKinematicsNode/IKService'
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

        # Set the desired pose in the service request message to pose information pulled from the object pose topic
        ikreq.pose_stamp.append(poses[self.limb])


        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("MOTION CTRL - Service call failed: %s" % (e,))
            return (False, "Unknown exception occurred.")

        if (resp.isValid[0]):
            rospy.loginfo("IKSVC - Success! Valid joint solution found.")

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

            # Print Info message to alert users of impending motion
            rospy.loginfo("MOTION CTRL - WARNING! Moving Baxter's " + self.limb + " arm to pounce position.")

            if (self.limb == 'left'):
                left = baxter_interface.Limb('left')
                left.move_to_joint_positions(limb_joints)
            elif (self.limb == 'right'):
                right = baxter_interface.Limb('right')
                right.move_to_joint_positions(limb_joints)

            else:
                pass
        else:
            rospy.loginfo("IK SOLVER - Invalid pose - No valid joint solution found.")

        return (True, "MOTION CTRL - Move to AR pounce point complete.")


    def svcMoveToOffsetPos(self, data):

        # Establish connection to specific limb's IKSolver service
        ns = 'ExternalTools/' + self.limb + '/PositionKinematicsNode/IKService'
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Update header information based on current time and with reference to base frame
        hdr = Header(stamp = rospy.Time.now(), frame_id='base')

        poses = {
            'left': PoseStamped(
                header = hdr,
                pose = Pose(
                    position = Point(
                        x = CURPOS.position.x + data.position.x,
                        y = CURPOS.position.y + data.position.y,
                        z = CURPOS.position.z + data.position.z,
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
                        x = CURPOS.position.x + data.position.x,
                        y = CURPOS.position.y + data.position.y,
                        z = CURPOS.position.z + data.position.z,
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

        # Set the desired pose in the service request message to pose information pulled from the object pose topic
        ikreq.pose_stamp.append(poses[self.limb])

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("MOTION CTRL - Service call failed: %s" % (e,))
            return (False, "Unknown exception occurred.")

        if (resp.isValid[0]):
            rospy.loginfo("IKSVC - Success! Valid joint solution found.")

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

            # Print Info message to alert users of impending motion
            rospy.loginfo("MOTION CTRL - WARNING! Moving Baxter's " + self.limb + " arm to offset position.")

            if (self.limb == 'left'):
                left = baxter_interface.Limb('left')
                left.move_to_joint_positions(limb_joints)
            elif (self.limb == 'right'):
                right = baxter_interface.Limb('right')
                right.move_to_joint_positions(limb_joints)

            else:
                pass
        else:
            rospy.loginfo("IK SOLVER - Invalid pose - No valid joint solution found.")

        return (True, "MOTION CTRL - Move to offset position complete.")


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
