#!/usr/bin/env python

import argparse
import struct
import sys

import rospy
from math import pi

import baxter_interface

from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class IK_solver():
    def __init__(self):
        rospy.init_node("rsdk_ik_service_client")
        self.poses = {
            'left': PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base'),
                pose=Pose(
                    position=Point(
                        x=0.657579481614,
                        y=0.851981417433,
                        z=0.0388352386502,
                    ),
                    orientation=Quaternion(
                        x=-0.366894936773,
                        y=0.885980397775,
                        z=0.108155782462,
                        w=0.262162481772,
                    ),
                ),
            )
        }
        self.flag_done = 0
        self.flag_valid = 0

    def ik_test(self, limb):
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        self.ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')

        # example = PoseStamped(
        #     header=Header(stamp=rospy.Time.now(), frame_id='base'),
        #     pose=Pose(
        #         position=Point(
        #             x=0.657579481614,
        #             y=0.851981417433,
        #             z=0.0388352386502,
        #         ),
        #         orientation=Quaternion(
        #             x=-0.366894936773,
        #             y=0.885980397775,
        #             z=0.108155782462,
        #             w=0.262162481772,
        #         ),
        #     )
        # )
        print(self.poses[limb])
        self.ikreq.pose_stamp.append(self.poses[limb])
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(self.ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        #Check if result valid, and type of seed ultimately used to get solution
        #convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                   resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            # seed_str = {
            #             self.ikreq.SEED_USER: 'User Provided Seed',
            #             self.ikreq.SEED_CURRENT: 'Current Joint Angles',
            #             self.ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
            #            }.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found ")
            self.flag_valid = 1
            # Format solution into Limb API-compatible dictionary

            # self.left.move_to_joint_positions(limb_joints)

            # print "\nIK Joint Solution:\n", self.limb_joints
            # print "------------------"
            # print "Response Message:\n", resp
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
            self.flag_valid = 0

        self.limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        self.left_var = baxter_interface.Limb('left')
        # return 0

    def callback(self, data):
        q_rot = quaternion_from_euler(pi,0,0)
        q_desired = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        q_new = quaternion_multiply(q_rot, q_desired)
        new_pose = data
        new_pose.orientation.x = q_new[0]
        new_pose.orientation.y = q_new[1]
        new_pose.orientation.z = q_new[2]
        new_pose.orientation.w = q_new[3]
        self.poses = {
            'left': PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='base'),
                pose=new_pose
                )
        }
        print("I got a new pose. Yayy!")

        # find IK solution
        arg_fmt = argparse.RawDescriptionHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                         description=main.__doc__)
        parser.add_argument(
            '-l', '--limb', choices=['left'], required=False,
            help="the limb to test"
        )
        args = parser.parse_args(rospy.myargv()[1:])
        self.ik_test('left')
        if (self.flag_valid == 1):
            print(self.limb_joints)
            self.left_var.move_to_joint_positions(self.limb_joints)
            print("I moved to the desired pose. Yayy!")
            rospy.sleep(3)
            self.flag_done = 1
            return (self.pose)
        else:
            print("PLease try again.")
            # self.ikreq.seed_mode = 2
        return 0

def main():
    ik_class = IK_solver()

    #define desired pose
    rospy.Subscriber('move_to_position', Pose, ik_class.callback)
    rospy.spin()

    while(1):
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()
