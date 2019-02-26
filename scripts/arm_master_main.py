#!/usr/bin/env python
from arm_utils import *
import time
import rospy
from de_msgs.srv import QueryNextPos, MoveArm
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerRequest
from de_msgs.srv import QueryBrickLoc, QueryBrickLocRequest
from arm_utils import *
import numpy as np
rospy.init_node('arm_master', anonymous=True)

# TODO FIX ISSUE WITH BAD INIT

pub_gripper = rospy.Publisher('/franka/gripper_width',
                          Float64, queue_size=1)

#Get all Services

#Services for Moving arm
def connect_srv(name, msg_type):
    rospy.loginfo("Searching for " + name + " .... ")
    rospy.wait_for_service(name)
    srv_wrapper = rospy.ServiceProxy(name, msg_type)
    rospy.loginfo(name + " CONNECTED")
    return srv_wrapper

#Services for moving arm
move_arm_wrapper = connect_srv('move_arm', MoveArm)
move_arm_curve_wrapper = connect_srv('move_arm_curve', MoveArm)

#gen_brick generates a brick in Gazebo
gen_brick_wrapper = connect_srv('/gen_brick', Trigger)

#Services for querying pick and place locations
get_pick_loc_wrapper = connect_srv('get_pick_loc', QueryBrickLoc)
get_place_lock_wrapper = connect_srv('get_place_loc', QueryBrickLoc)

#Functinos to further wrap function calls
def gen_brick():
    gen_brick_wrapper(TriggerRequest())

def get_brick_pos():
    loc = get_pick_loc_wrapper(QueryBrickLocRequest())
    p = [loc.x, loc.y, loc.z, loc.wx, loc.wy, loc.wz]
    # return [0.5, 0.5, 0.05 + 0.1, 3.14, 0, 3.14/4]
    return p

def get_goal_pos():
    loc = get_place_lock_wrapper(QueryBrickLocRequest())
    p = [loc.x, loc.y, loc.z, loc.wx, loc.wy, loc.wz]
    # return [-0.5, 0.5, 0.2,  3.14, 0, 0]
    return p

def get_home_pos():
    return [0, 0.5, 0.5, 3.14, 0, 0]
def get_over_pos():
    return [0, 0, 1, 3.14*2, 0, 3.14/4]

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

###################################################################
#MAIN CODE
###################################################################
num_bricks = get_num_bricks()
placed = 0

#MOVE ARM TO STARTING LOCATION
move_arm_curve(get_brick_pos()) #start in location so you can go back to where you came from
move_arm_curve(get_home_pos())

while not rospy.is_shutdown(): #MAIN LOOP that does the control of the arm
    if placed < num_bricks: #Continue to loop until you have placed the correct number of bricks
        placed += 1

        #Query Positions
        brick = get_brick_pos()
        goal = get_goal_pos()
        home = get_home_pos()
        over_head = get_over_pos()

        #Issue trying to place brick directly behind you must go up first
        #generate Brick
        gen_brick()

        #Pick Place operation then return home
        pick_up(brick)
        #temp fix to move to goal position
        move_arm_curve(over_head)
        move_arm_curve(goal)
        open_gripper()
        move_arm_curve(home)
        move_arm_curve(over_head)

        # place_down(goal)
        # go_to(home)

        print("Placed")
        #Place another brick from stack onto wall
    else: #When done just wait
        rospy.loginfo("Done, " +str(placed) + " bricks placed")
    rate.sleep()
