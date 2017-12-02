#!/usr/bin/env python
import time
import rospy
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

#global variables 
xnew = 0
ynew = 0
znew = 0

def callback(data):
    
    global xnew
    global ynew
    global znew

    #new values that are obtained from the AR marker
    xnew = data.markers[0].pose.pose.position.x
    ynew = data.markers[0].pose.pose.position.y
    znew = data.markers[0].pose.pose.position.z

   
    
    # rospy.loginfo("x is: %f" % (data.markers[0].pose.pose.position.x))
    # rospy.loginfo("y is: %f" % (data.markers[0].pose.pose.position.y))
    # rospy.loginfo("z is: %f" % (data.markers[0].pose.pose.position.z))
 

def ik_control(limb):
    
    #rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    
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

def main():
    rospy.init_node("rsdk_ik_service_client")
 

    rospy.Subscriber("/ar_trackers/ar_pose_marker", AlvarMarkers, callback)
    #hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    time.sleep(1)
    ik_control('left')
    
    rospy.spin()
   


    
if __name__ == '__main__':
    main()
