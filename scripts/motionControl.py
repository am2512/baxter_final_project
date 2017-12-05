#!/usr/bin/env python


import rospy
import baxter_interface

from baxter_core_msgs.msg import EndpointState
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
from me495_baxter_jar.srv import OffsetMove
from std_srvs.srv import Trigger


class motionControl():

    def __init__(self):

        # Publishers and Subscribers
        self.cur_left_ee_state = rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self.cbSetLeftEEPosition)
        self.cur_right_ee_state = rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.cbSetRightEEPosition)
        self.obj_pose_sub = rospy.Subscriber('object_pose', Pose, self.cbSetTagPosition)

        # Services
        self.move_to_AR_tag = rospy.Service('move_to_AR_tag', Trigger, self.svcMoveToARTag)
        self.move_to_offset_pos = rospy.Service('move_to_offset_pos', OffsetMove, self.svcMoveToOffsetPos)

        # Static configuration variables
        self.limb = 'left'      # Hardcoded for now


    def cbSetTagPosition(self, data):

        # New values that are obtained from the published object pose
        self.tag_point = data.position

        return


    def cbSetLeftEEPosition(self, data):

        # New values obtained from the left end-effector's published pose
        self.left_ee_point = data.pose.position
        self.left_ee_orientation = data.pose.orientation

        return


    def cbSetRightEEPosition(self, data):

        # New values obtained from the right end-effector's published pose
        self.right_ee_point = data.pose.position
        self.right_ee_orientation = data.pose.orientation

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
                    position = self.tag_point,
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
                    position = self.tag_point,
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
            rospy.loginfo("IK SOLVER - Success! Valid joint solution found.")

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
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        # Add input offsets to currently stored end-effector positions
        # Left offset
        left_offset = PoseStamped
        left_offset.header = hdr

        left_offset.pose.position.x = data.position.x + self.left_ee_point.x
        left_offset.pose.position.y = data.position.y + self.left_ee_point.y
        left_offset.pose.position.z = data.position.z + self.left_ee_point.z

        left_offset.pose.orientation.x = data.orientation.x + self.left_ee_orientation.x
        left_offset.pose.orientation.y = data.orientation.y + self.left_ee_orientation.y
        left_offset.pose.orientation.z = data.orientation.z + self.left_ee_orientation.z
        left_offset.pose.orientation.w = data.orientation.w + self.left_ee_orientation.w

        # Right offset
        right_offset = PoseStamped
        right_offset.header = hdr

        right_offset.pose.position.x = data.position.x + self.right_ee_point.x
        right_offset.pose.position.y = data.position.y + self.right_ee_point.y
        right_offset.pose.position.z = data.position.z + self.right_ee_point.z

        right_offset.pose.orientation.x = data.orientation.x + self.right_ee_orientation.x
        right_offset.pose.orientation.y = data.orientation.y + self.right_ee_orientation.y
        right_offset.pose.orientation.z = data.orientation.z + self.right_ee_orientation.z
        right_offset.pose.orientation.w = data.orientation.w + self.right_ee_orientation.w


        poses = { 'left' : left_offset,
                  'right': right_offset }

        # Set the desired pose in the service request message to pose information pulled from the object pose topic
        ikreq.pose_stamp.append(poses[self.limb])

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("MOTION CTRL - Service call failed: %s" % (e,))
            return (False, "Unknown exception occurred.")

        if (resp.isValid[0]):
            rospy.loginfo("IK SOLVER - Success! Valid joint solution found.")

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

    # Node initialization
    rospy.init_node("motion_controller")

    # Class initialization
    motionControl()

    while not rospy.is_shutdown():
        rospy.spin()

    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
