#!/usr/bin/env python
from arm_utils import *
import time
import rospy
from de_msgs.srv import QueryNextPos, MoveArm
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerRequest
from arm_utils import *
import numpy as np
rospy.init_node('arm_master', anonymous=True)

# TODO FIX ISSUE WITH BAD INIT

pub_gripper = rospy.Publisher('/franka/gripper_width',
                          Float64, queue_size=1)

rospy.wait_for_service('move_arm')
move_arm_wrapper = rospy.ServiceProxy('move_arm', MoveArm)

rospy.wait_for_service('move_arm_curve')
move_arm_curve_wrapper = rospy.ServiceProxy('move_arm_curve', MoveArm)

rospy.wait_for_service('/gen_brick')
gen_brick_wrapper = rospy.ServiceProxy('/gen_brick', Trigger)

def gen_brick():
    gen_brick_wrapper(TriggerRequest())

def get_brick_pos():
    return [0.5, 0.5, 0.05 + 0.1, 3.14, 0, 3.14/4]

def get_goal_pos():
    return [-0.5, 0.5, 0.2,  3.14, 0, 0]

def get_home_pos():
    return [0, 0.5, 0.5, 3.14, 0, 0]

def get_num_bricks():
    return 10

def move_arm(pos):
    msg = MoveArm()
    success = move_arm_wrapper(x = pos[0], y =pos[1], z = pos[2],rot_x =pos[3],rot_y =pos[4],rot_z =pos[5])
    return success
def move_arm_curve(pos):
    success = move_arm_curve_wrapper(x = pos[0], y =pos[1], z = pos[2],rot_x =pos[3],rot_y =pos[4],rot_z =pos[5])
    return success

def pick_up(target, via_offset = 0.3):
    #First Move to point above the pick up location
    via_point = copy.deepcopy(target)
    via_point[2] += via_offset #Z offset
    move_arm(via_point)
    rospy.sleep(1) # Tune time

    #Make sure gripper is open
    open_gripper()
    rospy.sleep(1) # Tune time

    move_arm(target)
    close_gripper()
    rospy.sleep(2)

    move_arm(via_point)

def place_down(target, via_offset = 0.3):
        #First Move to point above the pick up location
    via_point = copy.deepcopy(target)
    via_point[2] += via_offset #Z offset
    move_arm(via_point)
    rospy.sleep(1) # Tune time
    move_arm(target)
    open_gripper()
    rospy.sleep(3)
    move_arm(via_point)

def go_to(pos):
    move_arm(pos)

def open_gripper():
    pub_gripper.publish(0.12)
    return True
def close_gripper():
    pub_gripper.publish(0.03)
    return True

rate = rospy.Rate(1)


num_bricks = get_num_bricks()
placed = 0
move_arm_curve(get_brick_pos()) #start in location so you can go back to where you came from
move_arm_curve(get_home_pos())

while not rospy.is_shutdown():
    if placed < num_bricks:
        placed += 1
        # place_brick()

        brick = get_brick_pos()
        goal = get_goal_pos()
        home = get_home_pos()
        gen_brick()
        pick_up(brick)
        place_down(goal)
        go_to(home)
        print("Placed")
        #Place another brick from stack onto wall
    else:

        rospy.loginfo("Done, " +str(placed) + " bricks placed")
    rate.sleep()
