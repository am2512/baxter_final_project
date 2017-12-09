# **Baxter as a Laundry Assistant:** opening and closing a Tide bottle
*Group 1: Ahalya, Wanning, Bill, Ola, Mike*


## Overview of project

This package (package name: me495_baxter_jar) allows Baxter to open and close a Tide bottle, using his left arm.

![MarkView](https://github.com/am2512/baxter_final_project/blob/master/images/demo1.png)

## How to run the files

A closed Tide bottle with an AR tag on the cap needs to be firmly attached (duct tape will suffice) to a table well within Baxter's reachable area. Once communication with Baxter is established run command:

`roslaunch me495_baxter_jar startup.launch`

To visualize Baxter and the AR tag in rviz to monitor what Baxter is seeing, run the following command:

`roslaunch me495_baxter_jar debug_ar_track.launch`

## State machine: nodes, services, and their functionality

The package operates similarly to a state machine. Upon completion of each task, the next one is called using a service. The full sequence of steps is controlled through a master node ([sequencer.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/sequencer.py) node.)

Key steps:

**1. Initialize Baxter:**

This is the initial stage in the sequence of events. Baxter is enabled, grippers are calibrated, and the left hand camera resolution is set to 1280x800. The exact code can be found here: [_init_baxter.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/gripperControl.py).

**2. Go to home position:**

The left arm moves to position that gives the left hand camera a clear view of the table. 

**3. Locate AR tag:**

This step uses the `ar_track_alvar` ROS wrapper, to detect AR tags that are fixed onto objects. In this case, there is one AR tag on the top of the Tide bottle lid. The AR tag provides accurate details about the pose and orientation and orientation of the lid. The service definitions can be found in this [relayObjPose.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/relayObjPose.py) node.

**4. Move to AR tag and prepare to grip:** 

This is done using the pose and orientation data that the AR tag provides. We used the Inverse Kinematics Solver Service to obtain the joint angles for a given pose and orientation. The service definitions can be found in this [motionControl.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/motionControl.py) node.

The AR tag data at this time is copied and saved so that it can be used later to return to the bottle.

**5. Untwist wrist (CW) and unscrew lid (CCW):**

In this step, the custom services `open_gripper`, `close_gripper` and `unscrew_lid` are utilised. The service definitions can be found in this [gripperControl.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/gripperControl.py) node. 

**6. Retract and move to drop cap on table:**

The left arm moves up and then opens its gripper to drop the cap beside the bottle.

**7. Go to home position:**

The left arm goes back to the home position and waits for an updated position of the lid.

**8. Locate AR tag:**

The AR tag is located and the pose and orientation are relayed to the IK Solver Service. 

**9. Move to AR tag and prepare to grip:**

The arm moves to the AR tag on the lid, aligns itself and grips the lid.

**10. Move to saved pose of bottle:**

The location of the bottle was saved in Step 4. The left arm now moves back to the bottle using this saved pose and orientation data. 

**11. Untwist wrist (CCW) and screw in lid (CW):**

The `unscrew_lid` and `screw_lid` services are called during this step.

**12. Retract and go to home position:**

The gripper releases the lid after completing the closing action and moves back to the home position.

Task = Complete!

## Additional ROS packages required

**ar_track_alvar** - a ROS wrapper for Alvar, an open-source AR-tag-tracking library.s

