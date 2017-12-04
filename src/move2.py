#!/usr/bin/env python
import time
import rospy
from math import pi
import baxter_interface
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import rospy
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply


#global variables 
xnew = 0
ynew = 0
znew = 0
qx=0
qy=0
qz=0
qw=0

def callback(data):
    
    global xnew_g
    global ynew_g
    global znew_g
    global qx_g
    global qy_g
    global qz_g
    global qw_g

    #new values that are obtained from the AR marker
    xnew_g = pose.position.x
    ynew_g = pose.position.y
    znew_g = pose.position.z
    
    qx_g = pose.orientation.x
    qy_g = pose.orientation.y
    qz_g = pose.orientation.z
    qw_g = pose.orientation.w
    # rospy.loginfo("x is: %f" % (data.markers[0].pose.pose.position.x))
    # rospy.loginfo("y is: %f" % (data.markers[0].pose.pose.position.y))
    # rospy.loginfo("z is: %f" % (data.markers[0].pose.pose.position.z))
 

def ik_control(limb):
    
    #rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    q_old=[qx,qy,qz,qw]
    print q_old
    # RPY to convert: 0deg, 180deg, 0deg
    q_rot = quaternion_from_euler(0, pi, 0)
    print q_rot
    q_new = quaternion_multiply(q_rot,q_old)
    print q_new
    # print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])
    
    # print(xnew)    
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=xnew,
                    y=ynew,
                    z=znew,
                ),
                orientation=Quaternion(
                    x=q_new[0],
                    y=q_new[1],
                    z=q_new[2],
                    w=q_new[3],
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
        rospy.sleep(5)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")


def main():
	while(1):
	    rospy.init_node("rsdk_ik_service_client")
	    rospy.Subscriber("/z_controls/object_pose", Pose, callback)
	    #hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	    time.sleep(1)
	    xnew = xnew_g
	    ynew = ynew_g
	    xnew = znew_g
	    qx = qx_g
	    qy = qy_g
	    qz = qz_g
	    qw = qw_g
	    ik_control('left')
	    rospy.spin()
   
if __name__ == '__main__':
    main()
