#!/usr/bin/env python


import rospy
import baxter_interface
import math
import tf

from baxter_core_msgs.msg import EndpointState
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
from me495_baxter_jar.srv import OffsetMove
from std_srvs.srv import Trigger


class motionControls():

    def __init__(self):

        # Publishers and Subscribers
        self.left_ee_sub = rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self.cb_set_left_ee_position)
        self.right_ee_sub = rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.cb_set_right_ee_position)
        self.obj_pose_sub = rospy.Subscriber('object_pose', Pose, self.cb_set_tag_position)

        #self.copy = rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self.copy_bottle)


        # Services
        self.move_AR_svc = rospy.Service('motion_controller/move_to_AR_tag', Trigger, self.svc_move_to_AR_tag)
        self.move_offset_svc = rospy.Service('motion_controller/move_to_offset_pos', OffsetMove, self.svc_move_to_offset_pos)
        self.move_to_bottle_svc = rospy.Service('motion_controller/move_to_bottle', Trigger, self.svc_move_to_bottle)
        self.store_bottle_ar_svc = rospy.Service('motion_controller/store_bottle_ar', Trigger, self.svc_copy_bottle_ar)

        # Static configuration variables
        self.limb = 'left'      # Hardcoded for now

        # Class variables for later use
        self.tag_x = 0
        self.tag_y = 0
        self.tag_z = 0

        self.tag_qx = 0
        self.tag_qy = 0
        self.tag_qz = 0
        self.tag_qw = 0

        self.bottle_x = 0
        self.bottle_y = 0
        self.bottle_z = 0

        self.bottle_qx = 0
        self.bottle_qy = 0
        self.bottle_qz = 0
        self.bottle_qw = 0




    def cb_set_tag_position(self, data):

        # New values that are obtained from the published object pose
        self.tag_x = data.position.x
        self.tag_y = data.position.y
        self.tag_z = data.position.z

        self.tag_qx = data.orientation.x
        self.tag_qy = data.orientation.y
        self.tag_qz = data.orientation.z
        self.tag_qw = data.orientation.w

        return


    def cb_set_left_ee_position(self, data):

        # New values obtained from the left end-effector's published pose
        self.left_ee_point = data.pose.position
        self.left_ee_orientation = data.pose.orientation

        return


    def cb_set_right_ee_position(self, data):

        # New values obtained from the right end-effector's published pose
        self.right_ee_point = data.pose.position
        self.right_ee_orientation = data.pose.orientation

        return


    def svc_move_to_AR_tag(self, data):

        # Establish connection to specific limb's IKSolver service
        ns = '/ExternalTools/' + self.limb + '/PositionKinematicsNode/IKService'
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Update header information based on current time and with reference to base frame
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        # Update orientation of gripper based on rotation of AR tag frame to a frame compatible with Baxter's EE state
        q_old = [self.tag_qx, self.tag_qy, self.tag_qz, self.tag_qw]
        q_rot = tf.transformations.quaternion_from_euler(0, math.pi, 0)
        q_new = tf.transformations.quaternion_multiply(q_rot, q_old)



        poses = {
            'left': PoseStamped(
                header = hdr,
                pose = Pose(
                    position = Point(
                        x = self.tag_x,
                        y = self.tag_y,
                        z = self.tag_z

                    ),
                    orientation = Quaternion(
                        x = q_new[0],
                        y = q_new[1],
                        z = q_new[2],
                        w = q_new[3]
                    )
                )
            ),
            'right': PoseStamped(
                header = hdr,
                pose = Pose(
                    position = Point(
                        x = self.tag_x,
                        y = self.tag_y,
                        z = self.tag_z
                    ),
                    orientation = Quaternion(
                        x = q_new[0],
                        y = q_new[1],
                        z = q_new[2],
                        w = q_new[3]
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


    def svc_copy_bottle_ar(self,data):
        try:

            bq_old = [self.tag_qx, self.tag_qy, self.tag_qz, self.tag_qw]
            bq_rot = tf.transformations.quaternion_from_euler(0, math.pi, 0)
            bq_new = tf.transformations.quaternion_multiply(bq_rot, bq_old)

            self.bottle_x = self.tag_x
            self.bottle_y = self.tag_y
            self.bottle_z = self.tag_z

            self.bottle_qx = bq_new[0]
            self.bottle_qy = bq_new[1]
            self.bottle_qz = bq_new[2]
            self.bottle_qw = bq_new[3]


            rospy.loginfo("COPY - Success!")
            return (True, " COPY COMPLETE ")

        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("COPY BOTTLE AR - Service call failed: %s" % (e,))
            return (False, "Unknown exception occurred.")


    def svc_move_to_bottle(self, data):

        # Establish connection to specific limb's IKSolver service
        ns = '/ExternalTools/' + self.limb + '/PositionKinematicsNode/IKService'
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Update header information based on current time and with reference to base frame
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        # Update orientation of gripper based on rotation of AR tag frame to a frame compatible with Baxter's EE state

        poses = {
            'left': PoseStamped(
                header = hdr,
                pose = Pose(
                    position = Point(
                        x = self.bottle_x,
                        y = self.bottle_y,
                        z = self.bottle_z
                    ),
                    orientation = Quaternion(
                        x = self.bottle_qx,
                        y = self.bottle_qy,
                        z = self.bottle_qz,
                        w = self.bottle_qz
                    )
                )
            ),
            'right': PoseStamped(
                header = hdr,
                pose = Pose(
                    position = Point(
                        x = self.tag_x,
                        y = self.tag_y,
                        z = self.tag_z
                    ),
                    orientation = Quaternion(
                        x = self.bottle_qx,
                        y = self.bottle_qy,
                        z = self.bottle_qz,
                        w = self.bottle_qz
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



    def svc_move_to_offset_pos(self, data):

        # Establish connection to specific limb's IKSolver service
        ns = '/ExternalTools/' + self.limb + '/PositionKinematicsNode/IKService'
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Update header information based on current time and with reference to base frame
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        # Add input offsets to currently stored end-effector positions
        # Left offset
        left_offset = PoseStamped()
        left_offset.header = hdr

        left_offset.pose.position.x = data.offset.position.x + self.left_ee_point.x
        left_offset.pose.position.y = data.offset.position.y + self.left_ee_point.y
        left_offset.pose.position.z = data.offset.position.z + self.left_ee_point.z

        left_offset.pose.orientation.x = data.offset.orientation.x + self.left_ee_orientation.x
        left_offset.pose.orientation.y = data.offset.orientation.y + self.left_ee_orientation.y
        left_offset.pose.orientation.z = data.offset.orientation.z + self.left_ee_orientation.z
        left_offset.pose.orientation.w = data.offset.orientation.w + self.left_ee_orientation.w

        # Right offset
        right_offset = PoseStamped()
        right_offset.header = hdr

        right_offset.pose.position.x = data.offset.position.x + self.right_ee_point.x
        right_offset.pose.position.y = data.offset.position.y + self.right_ee_point.y
        right_offset.pose.position.z = data.offset.position.z + self.right_ee_point.z

        right_offset.pose.orientation.x = data.offset.orientation.x + self.right_ee_orientation.x
        right_offset.pose.orientation.y = data.offset.orientation.y + self.right_ee_orientation.y
        right_offset.pose.orientation.z = data.offset.orientation.z + self.right_ee_orientation.z
        right_offset.pose.orientation.w = data.offset.orientation.w + self.right_ee_orientation.w


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

# ========== #


def main():

    # Node initialization
    rospy.init_node('motion_controller')

    # Class initialization
    motionControls()

    while not rospy.is_shutdown():
        rospy.spin()

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
