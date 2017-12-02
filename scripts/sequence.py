#!/usr/bin/env python


import rospy
import init_baxter


def main():
    
    ### BASELINE ###
    # Initialize code
    # *** Reset Baxter to nominal state
    # Configure cameras and set resolution to 1280x800
    # *** Start of Execution Loop
        # *** Move to Pounce Position (above table)
        # *** Get Pose information from either AR tags or color segmentation
        # *** Call IK Solver to move to some offset position from seen pose
        # *** Pick Position Loop
            # *** Move towards pick position
            # *** Check depth sensor
            # *** If close enough, grip, otherwise, loop
                # *** Exit Pick Position Loop when gripped to lid
        # *** Gripped to Lid Loop
            # *** Perform half-turn CCW
            # *** Ungrip
            # *** Perform half-turn CW
            # *** Grip
            # *** Return to Head
                # *** After a certain number of loops, check whether the lid is still on by applying an upwards motion (???)
                # *** If lid is removed, exit Gripped to Lid Loop
        # *** Set Lid down on Table w/ frame extracted from AR tag mounted on table

    # Initialize master controller (director) node
    rospy.init_node('director')
    
    # Initialize Baxter to nominal state
    baxCtrl = init_baxter.BaxterCtrls()

    baxCtrl.enableBaxter()
    baxCtrl.cameraSetupHeadLH()

    Move(x,y,z,qx, +0.3,0,0,)


    return


if __name__ == '__main__':
    main()
