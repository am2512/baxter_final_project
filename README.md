# Final project
#### 11.21 
`move.py` successfully move end effector to desired position.
Details: In`ik_test(limb)`: Firstly use ik service to transfer end effector position and rotation to joints angles which is stored in `limb_joints`, then use `baxter_interface.limb.move_to_joint_positions` send joints angles to the baxter and move.
#### 11.22 ---- working on simulation
`./baxter.sh sim` to get to the simulation environment.
`rosrun baxter_tools enable_robot.py -e` enable robot
then run my own node