#!/usr/bin/env python


import rospy

import _init_baxter
import move2

from std_srvs.srv import Trigger


def main():
    
    '''
    ### BASELINE PLAN ###
    # Initialize code
    # Reset Baxter to nominal state
    # Configure cameras and set resolution to 1280x800
    # Execution Loop
        # Move to Pounce Position (above table)
        # *** Get Pose information from either AR tags or color segmentation
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

    # Service initializations
    update_obj_pose = rospy.ServiceProxy('update_obj_pose', Trigger)
    rospy.wait_for_service('update_obj_pose', 3.0)
    
    # Initialize Baxter to nominal state
    baxCtrl = _init_baxter.BaxterCtrls()

    baxCtrl.enableBaxter()
    baxCtrl.cameraSetupHeadLH()

    # Move Baxter's arms to home
    baxCtrl.moveArmToHome('right')
    baxCtrl.moveArmToHome('left')

    # Calibrate each end effector gripper
    baxCtrl.calibrateGrippers()

    # Display info message communicating initialization
    rospy.loginfo("Baxter initialization complete.")

    ##################
    # EXECUTION LOOP #
    ##################
    while (True):

        ## If sequencer set to AR TRACK MODE, set trigger to update the published pose for an AR tag matching a specific ID value
        ## If sequencer set to COLOR SEGMENT MODE, set trigger to identify the pose of an object matching the material characteristics
        ##   of target object(s)
        # update_obj_pose()

        rospy.loginfo("Shiny dice, spinning, spinning...")

        # Update the published position of the lid
        update_obj_pose()




        break

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
