#!/usr/bin/env python


import rospy
import baxter_interface

from std_srvs.srv import Trigger


class gripperControls():

    def __init__(self):

        # Baxter Interface Components
        self.left_arm = baxter_interface.Limb('left')
        self.left_wrist = self.left_arm.joint_names()[6]
        self.left_gripper = baxter_interface.Gripper('left')

        self.right_arm = baxter_interface.Limb('right')
        self.right_wrist = self.right_arm.joint_names()[6]
        self.right_gripper = baxter_interface.Gripper('right')

        # Service Definitions
        rospy.Service('gripper_controller/unscrew_lid', Trigger, self.srv_opening_sequence)
        rospy.Service('gripper_controller/screw_lid', Trigger, self.srv_closing_sequence)

        rospy.Service('gripper_controller/close_grip', Trigger, self.srv_close_grip)
        rospy.Service('gripper_controller/open_grip', Trigger, self.srv_open_grip)


    def srv_open_grip(self, data):

        self.left_gripper.open()

        return (True, "GRIP - Gripper Open.")


    def srv_close_grip(self, data):

        self.left_gripper.close()

        return (True, "GRIP - Gripper Closed.")


    def srv_opening_sequence(self, data):
        '''
        Baxter should start near the lid of the object-to-be-opened before triggering this subsequence.
        '''

        # Resetting 'left_w2' and 'left_gripper' position
        self.left_gripper.open()
        self.left_arm.move_to_joint_positions({self.left_wrist: 3})
        rospy.sleep(1)

        # Basic opening sequence
        self.left_gripper.close()
        rospy.sleep(1)
        self.left_arm.move_to_joint_positions({self.left_wrist: -3})
        rospy.sleep(1)

        # Gripper is now holding something - DO NOT issue an "open_grip" command without considering the ramifications!

        return (True, "GRIP - Opening Sequence Complete.")


    def srv_closing_sequence(self, data):
        '''
        Baxter should start this subsequence while grasping the lid of the object-to-be-closed and while positioned
        over the object.
        '''

        # Resetting 'left_w2' and 'left_gripper' position
        self.left_arm.move_to_joint_positions({self.left_wrist: -3})
        rospy.sleep(1)

        # Basic closing sequence
        self.left_gripper.close()
        rospy.sleep(1)
        self.left_arm.move_to_joint_positions({self.left_wrist: 3})
        rospy.sleep(1)
        self.left_gripper.open()
        rospy.sleep(1)

        # Gripper should now have released object - now free to issue other commands to Baxter.

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
