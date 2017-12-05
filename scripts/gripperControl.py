#!/usr/bin/env python


import rospy

from baxter_core_msgs.msg import (EndEffectorCommand, JointCommand)
from sensor_msgs.msg import (JointState, Range)

from std_srvs.srv import Trigger


class gripperControls():

    def __init__(self):

        # Publishers and Subscribers
        rospy.Subscriber('robot/range/left_hand_range/state', Range, self.cb_rangefinder)
        rospy.Subscriber('robot/ref_joint_states', JointState, self.cb_joint_states)

        self.pub_grip = rospy.Publisher('robot/end_effector/left_gripper/command', EndEffectorCommand, queue_size = 10)
        self.pub_turn = rospy.Publisher('robot/limb/left/joint_command', JointCommand, queue_size = 10)

        # Services
        rospy.Service('unscrew_lid', Trigger, self.srv_opening_sequence)
        rospy.Service('screw_lid', Trigger, self.srv_closing_sequence)

        # Class-specific variables
        self.distance = 0
        self.current_states = JointState


    def cb_rangefinder(self, data):

        self.distance = data.range

        return


    def cb_joint_states(self, data):

        self.current_states = data

        return


    def close_gripper(self):

        # Grip command parameters
        close_grip = EndEffectorCommand(
            id = 65538,
            command = 'grip'
        )

        # Publish the 'grip' command to the left gripper topic
        self.pub_grip.publish(close_grip)

        # DEBUG
        rospy.loginfo("DEBUG - Gripper closed.")
        # END DEBUG

        return


    def open_gripper(self):

        # Grip command parameters
        open_grip = EndEffectorCommand(
            id = 65538,
            command = 'release'
        )

        # Publish the 'release' command to the left gripper topic
        self.pub_grip.publish(open_grip)

        # DEBUG
        rospy.loginfo("DEBUG - Gripper open.")
        # END DEBUG

        return


    def twist_CCW(self):

        # Command parameters needed for a counter-clockwise wrist turn
        CCW_turn = JointCommand(
            mode = 1,               # Position control
            names = ['left_w2'],    # Left wrist nearest EE
            command = [-3]          # Turn to position corresponding to -3
        )

        # Publish the 'twist CCW' command to the joint command topic
        self.pub_turn.publish(CCW_turn)

        # DEBUG
        rospy.loginfo("DEBUG - Turning Wrist 2 counter-clockwise to joint angle @ -3")
        # END DEBUG

        return


    def twist_CW(self):

        # Command parameters needed for a clockwise wrist turn
        CW_turn = JointCommand(
            mode = 1,               # Position control
            names = ['left_w2'],    # Left wrist nearest EE
            command = [3]           # Turn to position corresponding to -3
        )

        # Publish the 'twist CW' command to the joint command topic
        self.pub_turn.publish(CW_turn)

        # DEBUG
        rospy.loginfo("DEBUG - Turning Wrist 2 clockwise to joint angle @ 3")
        # END DEBUG

        return

    def srv_opening_sequence(self, data):

        count = 0

        while (count < 3):
            self.open_gripper()
            self.twist_CW()
            self.close_gripper()
            self.twist_CCW()
            count += 1

        return

    def srv_closing_sequence(self, data):

        count = 0

        while (count < 3):
            self.open_gripper()
            self.twist_CCW()
            self.close_gripper()
            self.twist_CW()
            count += 1

        self.open_gripper()

        return

# ========== #


def main():

    # Node initialization
    rospy.init_node('gripper_controller')

    gripperControls()

    while not rospy.is_shutdown():
        rospy.spin()

    return



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
