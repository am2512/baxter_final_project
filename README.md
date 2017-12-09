# **Baxter as a Laundry Assistant:** opening and closing a Tide bottle
*Group 1: Ahalya, Wanning, Bill, Ola, Mike*


## Project goal

Our goal was to get Baxter to open and close the lid of a Tide bottle. 

## Overview of project

This package (package name: `me495_baxter_jar`) allows Baxter to open and close a Tide bottle, using his left arm.

![MarkView](https://github.com/am2512/baxter_final_project/blob/master/images/demo1.png) 

## How to run the files

A closed Tide bottle with an AR tag on the cap needs to be firmly attached (duct tape will suffice) to a table well within Baxter's reachable area. Once communication with Baxter is established, run this command:

`roslaunch me495_baxter_jar startup.launch`

To visualize Baxter and the AR tag in rviz to monitor what Baxter is seeing, run the following command:

`roslaunch me495_baxter_jar debug_ar_track.launch`

## State machine: nodes, services, and their functionality

<h4>List of Nodes:</h4>

1. [_init_baxter.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/_init_baxter.py)
2. [gripperControl.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/gripperControl.py)
3. [motionControl.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/motionControl.py)
4. [relayObjPose.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/relayObjPose.py)
5. [sequencer.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/sequencer.py)

The package operates similarly to a state machine. Upon completion of each task, the next one is called using a service. The full sequence of steps is controlled through a master node ([sequencer.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/sequencer.py)).

<h4>Key steps:</h4>

**1. Initialize Baxter:**

This is the initial stage in the sequence of events. Baxter is enabled, grippers are calibrated, and the left hand camera resolution is set to 1280x800. The exact code can be found here: [_init_baxter.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/_init_baxter.py).

**2. Go to home position:**

The left arm moves to position that gives the left hand camera a clear view of the table. For moving the arm, we referred to this [IK Service](http://sdk.rethinkrobotics.com/wiki/IK_Service_-_Code_Walkthrough) example.
 
**3. Locate AR tag:**

This step uses the `ar_track_alvar` ROS wrapper, to detect AR tags that are fixed onto objects. In this case, there is one AR tag on the top of the Tide bottle lid. The AR tag provides accurate details about the pose and orientation and orientation of the lid. The service definitions can be found in this [relayObjPose.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/relayObjPose.py) node. 

Explanation: The function [cd_register_obj_pose](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/relayObjPose.py#L31-L42) gets the tag data, and then uses [svc_update_published_pose](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/relayObjPose.py#L45-L61) service to store the pose and orientation from the tag data and then update.

**4. Move to AR tag and prepare to grip:** 

This is done using the pose and orientation data that the AR tag provides. We used the Inverse Kinematics Solver Service in our node to obtain the joint angles for a given pose and orientation. We imported `quaternion_from_euler` and `quaternion_multiply` from `tf.transformations`, and used these functions along with the quaternion values of the AR tag to make sure that the gripper was aligned directly above the lid. The service definitions that carry out these functions can be found in this [motionControl.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/motionControl.py) node.

Explanation: [cb_set_tag_position](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/motionControl.py#L50-L56) is used to obtain the new pose value of the AR tag. [cb_set_left_ee_position](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/motionControl.py#L59-L67) is useful for offset moves based on current position. [svc_move_to_AR_tag](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/motionControl.py#L81-L143) is the service controlling the arm to move to AR tag position. [svc_store_bottle_pose](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/motionControl.py#L146-L167) is the service to store the bottle position, which is useful when the lid needs to be closed. The AR tag data at this time is copied and saved so that it can be used later to return to the bottle. [svc_move_to bottle](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/motionControl.py#L170-L222) moves lid back to the bottle, and then arm is ready to close the lid. [svc_move_to_offset](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/motionControl.py#L226-L307) helps to adjust the position of the end effector.

**5. Untwist wrist (CW) and unscrew lid (CCW):**

In this step, the following custom services are used: [srv_open_grip](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/gripperControl.py#L31-L35) is used to open the gripper. [srv_close_grip](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/gripperControl.py#L38-L42) is used to close the gripper. [srv_opening_sequence](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/gripperControl.py#L45-L63) is used unscrew the lid. Lastly, [srv_closing_sequence](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/gripperControl.py#L66-L86) is used to screw the lid. The service definitions can be found in this [gripperControl.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/gripperControl.py) node.

![MarkView](https://github.com/am2512/baxter_final_project/blob/master/images/opening_lid.gif)

**6. Retract and move to drop cap on table:**

The left arm moves up, moves to the side of the bottle, moves down and then opens its gripper to drop the cap beside the bottle.

**7. Go to home position:**

The left arm goes back to the home position and waits for an updated position of the lid. At this point, the lid can be shifted around. Baxter will identify the new position of the lid.

**8. Locate AR tag:**

The AR tag of the lid is located and the pose and orientation of the lid are obtained, using the [relayObjPose.py](https://github.com/am2512/baxter_final_project/blob/master/scripts/relayObjPose.py) node. 

**9. Move to AR tag and prepare to grip:**

The arm moves to the AR tag on the lid, aligns itself and grips the lid.

**10. Move to saved pose of bottle:**

The location of the bottle was saved in Step 4, when this service was called: [svc_store_bottle_pose](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/motionControl.py#L146-L167). The left arm now moves back to the bottle using this saved pose and orientation data. 

![MarkView](https://github.com/am2512/baxter_final_project/blob/master/images/move_to_bottle.gif)

**11. Untwist wrist (CCW) and screw in lid (CW):**

The [srv_open_grip](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/gripperControl.py#L31-L35) and [srv_close_grip](https://github.com/am2512/baxter_final_project/blob/e5a648deb35b5c857654af809f4dbe646fdc7b7d/scripts/gripperControl.py#L38-L42) services are called during this step.

![MarkView](https://github.com/am2512/baxter_final_project/blob/master/images/close_lid.gif)

**12. Retract and go to home position:**

The gripper releases the lid after completing the closing action and moves back to the home position.

Task = Complete!

## Additional ROS packages required

**ar_track_alvar** - a ROS wrapper for Alvar, an open-source AR-tag-tracking library.

## Conclusion

We were able to implement our goal of making Baxter open and close the lid of a Tide bottle. The final video of our demo can be found [here](https://vimeo.com/246549829).



