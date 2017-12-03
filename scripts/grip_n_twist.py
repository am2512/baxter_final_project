#!/usr/bin/env python
import time
import rospy
import baxter_interface


def unscrew():
    left = baxter_interface.Limb('left')
    g_left = baxter_interface.Gripper('left')
    wrist_left = left.joint_names()[6]

    # Resetting left_w2 and gripper position for maximum unscrewing
    if left.joint_angle(wrist_left) < 3:
        left.move_to_joint_positions({wrist_left: 3})
        time.sleep(1)
    if g_left.position <= 95:
        g_left.open()

    # Gripping
    g_left.close()
    time.sleep(1)
    if g_left.position() <= 10:
        g_left.open()
        print('Grip Failed: No object detected')
        quit()

    # Twisting
    else:
        left.move_to_joint_positions({wrist_left: -3})
        print('Unscrew complete!')
        quit()

def main():
    """Grip 'n' Twist: Command Baxter's left gripper to grab
    and twist an object. Created to unscrew a Tide bottle cap."""
    print("Initializing Node...")
    rospy.init_node('gripntwist')
    print("Unscrewing...")
    unscrew()




if __name__ == '__main__':
    main()
