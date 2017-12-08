# **Baxter as a Laundry Assistant:** opening and closing a Tide bottle
*Group 1: Ahalya, Wanning, Bill, Ola, Mike*


## Overview of project

This package (package name: me495_baxter_jar) allows Baxter to open and close a Tide bottle, using his left arm.

![MarkView](https://github.com/am2512/baxter_final_project/blob/master/images/demo1.png)

## How to run

A closed Tide bottle with an AR tag on the cap needs to be firmly attached (duct tape will suffice) to a table well within Baxter's reachable area. Once communication with Baxter is established run command:

`roslaunch me495_baxter_jar startup.launch`

To visualize Baxter and the AR tag in rviz to monitor what Baxter is seeing, run the following command:

`roslaunch me495_baxter_jar debug_ar_track.launch`

## State machine: nodes, services, and their functionality

The package operates similarly to a state machine. Upon completion of each task, the next one is called using a service.

Key steps:

1. Initiate Baxter:

This is the initial stage in the sequence of events. Baxter is enabled, grippers are calibrated, and the left hand camera resolution is set to 1280x800.

2. Go to home position:

The left arm moves to position that gives the left hand camera a clear view of the table. 

3. Locate AR tag:

4. Move to AR tag and prepare to grip: 

This is done using the pose and orientation data that the AR tag provides. We used the Inverse Kinematics Solver Service to obtain the joint angles for a given pose and orientation. 

5. Untwist wrist (CW) and unscrew lid (CCW):

6. Retract and move to drop cap on table:

7. Go to home position:

8. Locate AR tag:

9. Move to AR tag and prepare to grip:

10. Untwist wrist (CCW) and screw in lid (CW):

11. Retract and go to home position:


## Additional ROS packages required

1. **ar_track_alvar** - a ROS wrapper for Alvar, an open-source AR-tag-tracking library.

2. is there more?

