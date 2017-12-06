#!/usr/bin/env python


import rospy
import baxter_interface
import math

from sensor_msgs.msg import (JointState, Range)

from std_srvs.srv import Trigger


class gripperControls():

    def __init__(self):

        # Publishers and Subscribers
        self.lh_range_sub = rospy.Subscriber('/robot/range/left_hand_range/state', Range, self.cb_rangefinder)
        self.joint_states_sub = rospy.Subscriber('/robot/ref_joint_states', JointState, self.cb_joint_states)

        self.left_arm = baxter_interface.Limb('left')
        self.left_wrist = self.left_arm.joint_names()[6]
        self.left_gripper = baxter_interface.Gripper('left')

        self.right_arm = baxter_interface.Limb('right')
        self.right_wrist = self.right_arm.joint_names()[6]
        self.right_gripper = baxter_interface.Gripper('right')

        # Services
        self.svc_unscrew = rospy.Service('gripper_controller/unscrew_lid', Trigger, self.srv_opening_sequence)
        self.svc_screw = rospy.Service('gripper_controller/screw_lid', Trigger, self.srv_closing_sequence)

        # Class-specific variables
        self.distance = 0
        self.current_states = JointState()


    def cb_rangefinder(self, data):

        self.distance = data.range

        return


    def cb_joint_states(self, data):

        self.current_states = data

        return


    def srv_opening_sequence(self, data):

        # Resetting 'left_w2' and 'left_gripper' position
        self.left_gripper.open()
        self.left_arm.move_to_joint_positions({self.left_wrist: math.pi/2})
        rospy.sleep(1)

        # Basic opening sequence
        self.left_gripper.close()
        rospy.sleep(1)
        self.left_arm.move_to_joint_positions({self.left_wrist: -math.pi / 2})
        rospy.sleep(1)
        self.left_gripper.open()
        rospy.sleep(1)
        self.left_arm.move_to_joint_positions({self.left_wrist: math.pi / 2})
        rospy.sleep(1)
        self.left_gripper.close()
        rospy.sleep(1)
        self.left_arm.move_to_joint_positions({self.left_wrist: -math.pi / 2})
        rospy.sleep(1)

        return (True, "GRIP - Opening Sequence Complete.")


    def srv_closing_sequence(self, data):

        # Resetting 'left_w2' and 'left_gripper' position
        self.left_arm.move_to_joint_positions({self.left_wrist: -math.pi / 2})
        rospy.sleep(1)

        # Basic opening sequence
        self.left_gripper.close()
        rospy.sleep(1)
        self.left_arm.move_to_joint_positions({self.left_wrist: math.pi / 2})
        rospy.sleep(1)
        self.left_gripper.open()
        rospy.sleep(1)
        self.left_arm.move_to_joint_positions({self.left_wrist: -math.pi / 2})
        rospy.sleep(1)
        self.left_gripper.close()
        rospy.sleep(1)
        self.left_arm.move_to_joint_positions({self.left_wrist: math.pi / 2})
        rospy.sleep(1)
        self.left_gripper.open()
        rospy.sleep(1)

        return (True, "GRIP - Closing Sequence Complete.")

# ========== #


def main():

    # Node initialization
    rospy.init_node('gripper_controller')

    # Class initialization
    gripperControls()

    while not rospy.is_shutdown():
        rospy.spin()

    return



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
