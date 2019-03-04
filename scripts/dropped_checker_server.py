#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

rospy.init_node('dropped_checker_server')
holding = False
width = 0.12
def check_gripper_handler(data):
    global holding
    global width
    joints = data.position
    width = joints[0] + joints[1]
    # rospy.loginfo("Width: " + str(width))
    if width < 0.055:
        holding = False
    else:
        holding = True


def check_if_server(req):
    global holding
    global width

    rospy.loginfo("Width: " + str(width))
    rospy.loginfo(holding)
    if not holding:
        return TriggerResponse(success=True)
    else:
        return TriggerResponse(success=False)

rospy.Subscriber("/franka/joint_states", JointState, check_gripper_handler)
brick_manager_s = rospy.Service('check_if_dropped', Trigger, check_if_server)

rospy.spin()
