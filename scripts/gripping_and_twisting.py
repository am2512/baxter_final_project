#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Range
from baxter_core_msgs.msg import JointCommand
from baxter_core_msgs.msg import EndEffectorCommand
from std_srvs.srv import SetBool

class Gripping():
    def __init__(self):
        self.distance = 100
        rospy.Subscriber("robot/range/left_hand_range/state", Range, self.callback)

        self.pub_grip = rospy.Publisher('robot/end_effector/left_gripper/command', EndEffectorCommand, queue_size=10)
        self.pub_turn = rospy.Publisher('robot/limb/left/joint_command', JointCommand, queue_size=10)


    def callback(self, data):
        self.distance = data.range
        #rospy.loginfo("I'm %f above the lid!", self.distance)

    def grip_cup(self):
        self.grip = EndEffectorCommand()
        self.grip.id = 65538
        self.grip.command = 'grip'
        self.pub_grip.publish(self.grip)
        rospy.loginfo("I have gripped the cup.")
        rospy.sleep(1)

    def release_cup(self):
        self.grip = EndEffectorCommand()
        self.grip.id = 65538
        self.grip.command = 'release'
        self.pub_grip.publish(self.grip)
        rospy.loginfo("I have released the cup.")
        rospy.sleep(1)

    def twist_CCW(self):
        #unscrew cup
        self.turn = JointCommand()
        self.turn.mode = 3 # position control (1)
        self.turn.names = ['left_w2']
        self.turn.command = [-30]
        self.pub_turn.publish(self.turn)
        rospy.loginfo("Twisting")
        rospy.sleep(1)

    def twist_CW(self):
        #untwist gripper
        self.turn = JointCommand()
        self.turn.mode = 3 # position control (1)
        self.turn.names = ['left_w2']
        self.turn.command = [30]
        self.pub_turn.publish(self.turn)
        rospy.loginfo("Untwisting")
        rospy.sleep(1)

def main(input):
    if (input.data == 1):
        rospy.sleep(2)
        grip_class = Gripping()

        # while (distance> 0.1)
            # move arm

        # rospy.loginfo("DISTANCE: %f", grip_class.distance)
        # if (grip_class.distance < 0.15):

        count = 0
        while (count<4):
            grip_class.release_cup()
            grip_class.twist_CW()
            grip_class.grip_cup()
            grip_class.twist_CCW()
            count += 1


if __name__ == '__main__':
    try:
        rospy.init_node('gripping')
        rospy.Service('ready_to_grip', SetBool, main)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
