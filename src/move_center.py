#!/usr/bin/python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from baxter_core_msgs.msg import EndpointState
import serial
import time

global upper_baxter_pos
global lower_baxter_pos

upper_baxter = 1
lower_baxter = 0
upper_baxter_pos = 1500
lower_baxter_pos = 1500



def callback(ball_pos):
    global upper_baxter_pos
    global lower_baxter_pos

    ball_x = ball_pos.x
    ball_y = ball_pos.y

    

    robot_poserospy.Subscriber('/robot/limb/left/endpoint_state',EndpointState.pose.position,queue_size=1)
    print robot_pose.pose.position


    if ball_x < 300:
        lower_baxter_pos -= 9
        if lower_baxter_pos < 1000:
            lower_baxter_pos = 1000
        l, h = pololu_set_target(lower_baxter_pos)
        
    if ball_x > 340:
        lower_baxter_pos += 9
        if lower_baxter_pos > 2000:
            lower_baxter_pos = 2000
        l, h = pololu_set_target(lower_baxter_pos)
        
    if ball_y > 260:
        upper_baxter_pos -= 9
        if upper_baxter_pos < 1000:
            upper_baxter_pos = 1000
        l, h = pololu_set_target(upper_baxter_pos)
        
    if ball_y < 220:
        upper_baxter_pos += 9
        if upper_baxter_pos > 2000:
            upper_baxter_pos = 2000
        l, h = pololu_set_target(upper_baxter_pos)


def ik_control(limb,p_x,p_y,p_z):
    #rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=p_x,
                    y=p_y,
                    z=p_z,
                ),
                orientation=Quaternion(
                    x=-0.159821886749,
                    y=0.984127886766,
                    z=-0.00293647198525,
                    w=0.0770755742012,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
                ),
            ),
        ),
    }
    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        #print limb_joints

        if(limb == 'left'):
            left = baxter_interface.Limb('left')
            left.move_to_joint_positions(limb_joints)
        else: 
            right = baxter_interface.Limb('right')
            right.move_to_joint_positions(limb_joints)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
 
    return 0


def listener():

    rospy.init_node('baxter_act', anonymous = True)
    rospy.Subscriber('/ball_center', Vector3, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()