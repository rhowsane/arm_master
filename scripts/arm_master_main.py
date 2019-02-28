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
from round_path import *
rospy.init_node('arm_master', anonymous=True)
from arm_master_functions import *
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
get_place_loc_wrapper = connect_srv('get_place_loc', QueryBrickLoc)

#Functinos to further wrap function calls
def gen_brick():
    gen_brick_wrapper(TriggerRequest())

def get_brick_pos(placed):
    loc = get_pick_loc_wrapper(QueryBrickLocRequest(placed))
    p = [loc.x, loc.y, loc.z, loc.wx, loc.wy, loc.wz]
    return [0.5, 0.5, 0.15, 3.14, 0, 3.14/4]
    return p

def get_goal_pos(placed):
    loc = get_place_loc_wrapper(QueryBrickLocRequest(placed))
    p = [loc.x, loc.y, loc.z, loc.wx, loc.wy, loc.wz]
    return [0.5, -0.5, 0.15,  3.14, 0,  3.14/4]
    return p

def get_home_pos():
    return [0.5, 0, 0.5, 3.14, 0, 0]
def get_over_pos():
    return [0.5, 0.5, 0.5, 3.14, 0, 0]

def get_num_bricks():
    return 10

def move_arm(pos):
    msg = MoveArm()
    rospy.loginfo(pos)
    success = move_arm_wrapper(x = pos[0], y =pos[1], z = pos[2],rot_x =pos[3],rot_y =pos[4],rot_z =pos[5])
    return success

def move_arm_curve(pos):
    success = move_arm_curve_wrapper(x = pos[0], y =pos[1], z = pos[2],rot_x =pos[3],rot_y =pos[4],rot_z =pos[5])
    return success

def pick_up(target, via_offset = 0.2):
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

def place_down(target, via_offset = 0.2):
        #First Move to point above the pick up location
    via_point = copy.deepcopy(target)
    via_point[2] += via_offset #Z offset
    move_arm(via_point)
    rospy.sleep(1) # Tune time
    move_arm(target)
    open_gripper()
    rospy.sleep(3)
    move_arm(via_point)

def get_round_points():
    round_path = dict()
    res = float(20)
    diameter = 0.75
    r = diameter/2
    height = 0.5

    x_c = 0
    y_c = 0

    for i in np.arange(20):
        theta = (2 * np.pi) * ((i + 1 )/res)
        left = i-1
        if (left < 0):
            left = 19

        right = i+1
        if right > 19:
            right = 0

        neighbour = (right,left)
        x = x_c + r * np.cos(theta)
        y = y_c + r * np.sin(theta)
        plt.plot(x,y,'+')
        pos = [x_c + r * np.cos(theta), y_c + r * np.sin(theta), height]
        round_path[i] = (pos,neighbour)

    return round_path
# plt.show()
# print(round_path)

round_way_points = get_round_points()

def move_towards(start, end):
    #find nearest point to pick
    min_start_dist = 10000
    min_start_ind = 0
    min_end_dist = 10000
    min_end_ind = 0

    for key, value in round_way_points.items():
        # print(key, value)
        p = value[0]
        dist_start = distance(start,p)
        dist_end = distance(end,p)

        if dist_start < min_start_dist:
            min_start_dist = dist_start
            min_start_ind = key

        if dist_end < min_end_dist:
            min_end_dist = dist_end
            min_end_ind = key

        print(p)

    curr_ind = min_start_ind

    selector = left_or_right(curr_ind, min_end_ind, round_way_points)
    while curr_ind != min_end_ind:
        #move arm to the curr node positon
        curr_node = round_way_points[curr_ind]
        move_arm([curr_node[0][0],curr_node[0][1],curr_node[0][2],3.14,0,0])
        curr_ind = curr_node[1][selector] #go one way around the circle

    #move toward location in a controlled maner without running into


def go_to(pos):
    move_arm(pos)

def open_gripper():
    pub_gripper.publish(0.16)
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
open_gripper()
# move_arm_curve(get_brick_pos()) #start in location so you can go back to where you came from
move_arm_curve(get_home_pos())

while not rospy.is_shutdown(): #MAIN LOOP that does the control of the arm
    if placed < num_bricks: #Continue to loop until you have placed the correct number of bricks
        placed += 1

        #Query Positions
        brick = get_brick_pos(placed)
        goal = get_goal_pos(placed)
        home = get_home_pos()
        over_head = get_over_pos()

        #Issue trying to place brick directly behind you must go up first
        #generate Brick
        gen_brick()
        move_towards(home, brick)

        #Pick Place operation then return home
        pick_up(brick)
        #temp fix to move to goal position
        # move_arm_curve(over_head)
        # move_arm_curve(goal)
        move_towards(brick, goal)
        # move_arm_curve(home)
        # move_arm_curve(over_head)
        # move_towards(goal)
        place_down(goal)
        move_towards(goal, home)

        # go_to(over_head)
        # go_to(home)

        print("Placed")
        #Place another brick from stack onto wall
    else: #When done just wait
        rospy.loginfo("Done, " +str(placed) + " bricks placed")
    rate.sleep()
