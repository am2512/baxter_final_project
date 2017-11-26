#!/usr/bin/python
 
import rospy
from baxter_core_msgs.msg import EndpointState
def main():
    rospy.init_node('test',anonymous = True)
    robot_pose=rospy.Subscriber('/robot/limb/left/endpoint_state',EndpointState,queue_size=10)
    rospy.spin()
    print robot_pose




if __name__ == '__main__':
    main()