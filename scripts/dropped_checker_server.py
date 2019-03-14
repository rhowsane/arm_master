#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

# Functions to determine is gripper is still holding onto brick


def check_gripper_handler(data):
    """Short Function Summary

    Longer Function Description

    Args:
        data (list): ``[joint0, joint1, joint2,...]``. Where gripper are ``joint0`` and ``joint1``

    Returns:
        bool: Returns true if gripped is open. Return false if gripper is closer togther then a given threshhold.
        This indicates that it tried to close it hand and it didn't pick up a brick

    """
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
    """Service wrapper for checking if brick has been dropped"""
    global holding
    global width

    rospy.loginfo("Width: " + str(width))
    rospy.loginfo(holding)
    if not holding:
        return TriggerResponse(success=True)
    else:
        return TriggerResponse(success=False)

if __name__ == '__main__':
    rospy.init_node('dropped_checker_server')
    holding = False
    width = 0.12

    rospy.Subscriber("/franka/joint_states", JointState, check_gripper_handler)
    brick_manager_s = rospy.Service('check_if_dropped', Trigger, check_if_server)

    rospy.spin()
