#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import tf
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float64
from moveit_commander.conversions import pose_to_list
from de_msgs.srv import QueryBrickLoc, QueryBrickLocResponse
from samrowan import SamRowan
# from keith_code import *

# Provides the goal location queries.

rospy.init_node('brick_manager_server')


#Create Classes to Manager Goal and Brick Stack Location

GoalManager = SamRowan(20,4)

def brick_manager_server(req):
    resp = QueryBrickLocResponse()
    p = [0.5, 0.5, 0.05 + 0.1, 3.14, 0, 3.14/4]
    resp.x = p[0]
    resp.y = p[1]
    resp.z = p[2]
    resp.wx = p[3]
    resp.wy = p[4]
    resp.wz = p[5]
    return p


def goal_manager_server(req):
    p = GoalManager.get_next_goal_loc()    #SAM DO CODE AND LOGIC IN HERE
    print("GoalManager P: ", p)
    resp = QueryBrickLocResponse()
    # p = [-0.5, 0.5, 0.2,  3.14, 0, 0]
    resp.x = p[0]
    resp.y = p[1]
    resp.z = p[2]
    resp.wx = p[3]
    resp.wy = p[4]
    resp.wz = p[5]
    return resp

brick_manager_s = rospy.Service('get_pick_loc', QueryBrickLoc, brick_manager_server)
goal_manager_s = rospy.Service('get_place_loc', QueryBrickLoc, goal_manager_server)

rospy.spin()
