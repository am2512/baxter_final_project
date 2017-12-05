#!/usr/bin/env python


import rospy

import _init_baxter

from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)

from me495_baxter_jar.srv import OffsetMove
from std_srvs.srv import Trigger


def main():
    
    '''
    ### BASELINE PLAN ###
    # Initialize code
    # Reset Baxter to nominal state
    # Configure cameras and set resolution to 1280x800
    # Execution Loop
        # Move to Pounce Position (above table)
        # Get Pose information from either AR tags or color segmentation
        # *** Trigger MotionController node to move to some offset position from tracked pose
        # *** Pick Position Loop
            # *** Move towards pick position
            # *** Check depth sensor
            # *** If close enough, grip, otherwise, return to head of loop
                # *** Exit Pick Position Loop when gripped to lid
        # *** Gripped to Lid Loop
            # *** Perform CCW turn
                # *** After a certain number of loops, check whether the lid is still on by applying an upwards motion (???)
                # *** If lid is removed, exit Gripped to Lid Loop
        # *** Set Lid down on Table w/ frame extracted from AR tag mounted on table
            # *** Exit Execution Loop if no "CONTINUE" keypress
    '''

    ##################
    # INITIALIZATION #
    ##################
    # Start master controller ('director') node
    rospy.init_node('director')

    # Service initializations (IMPORTANT!!!)
    update_obj_pose = rospy.ServiceProxy('pose_relay/update_obj_pose', Trigger)
    rospy.wait_for_service('pose_relay/update_obj_pose', 3.0)

    move_AR_tag = rospy.ServiceProxy('motion_controller/move_to_AR_tag', Trigger)
    move_offset = rospy.ServiceProxy('motion_controller/move_to_offset_pos', OffsetMove)
    rospy.wait_for_service('motion_controller/move_to_offset_pos', 3.0)

    unscrew_lid = rospy.ServiceProxy('gripper_controller/unscrew_lid', Trigger)
    screw_lid = rospy.ServiceProxy('gripper_controller/screw_lid', Trigger)
    rospy.wait_for_service('gripper_controller/screw_lid', 3.0)


    # Initialize Baxter to nominal state
    baxCtrl = _init_baxter.BaxterCtrls()

    baxCtrl.enable_baxter()
    baxCtrl.camera_setup_head_LH()

    # Calibrate each end effector gripper
    baxCtrl.calibrate_grippers()

    # Display info message communicating initialization
    rospy.loginfo("Baxter initialization complete.")

    ##################
    # EXECUTION LOOP #
    ##################
    while (True):

        # Move Baxter's arms to home
        baxCtrl.move_arm_to_home('right')
        baxCtrl.move_arm_to_home('left')

        ## If sequencer set to AR TRACK MODE, set trigger to update the published pose for an AR tag matching a specific ID value
        ## If sequencer set to COLOR SEGMENT MODE, set trigger to identify the pose of an object matching the material characteristics
        ##   of target object(s)
        # update_obj_pose()

        rospy.loginfo("Shiny dice, spinning, spinning...")
        rospy.sleep(1)

        # Update the published position of the lid
        update_obj_pose()

        # Move to the pounce position over the detected AR tag
        move_AR_tag()

        # DEBUG
        rospy.loginfo("Sleeping for 3 seconds.")
        rospy.sleep(3)

        rospy.loginfo("Testing for offset motion to work.")

        test = Pose(
                    position = Point(
                        x = 0.0,
                        y = 0.0,
                        z = 0.05
                    ),
                    orientation = Quaternion(
                        x = 0.0,
                        y = 0.0,
                        z = 0.0,
                        w = 0.0
                    )
               )

        print test

        move_offset(test)

        rospy.loginfo("Offset move complete.")
        # END DEBUG

        # DEBUG
        rospy.loginfo("End of Line")
        # END DEBUG

        break

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
