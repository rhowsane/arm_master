#!/usr/bin/env python
from arm_utils import *
import time
import rospy
from de_msgs.srv import QueryNextPos, MoveArm
from std_msgs.msg import Float64
from arm_utils import *

rospy.init_node('arm_master', anonymous=True)



pub_gripper = rospy.Publisher('/franka/gripper_width',
                          Float64, queue_size=1)

rospy.wait_for_service('move_arm')
move_arm_wrapper = rospy.ServiceProxy('move_arm', MoveArm)

rospy.wait_for_service('place_brick')
place_brick_wrapper = rospy.ServiceProxy('place_brick', MoveArm)

def get_brick_pos():
    return [0.25, 0.25, 0, 3.14, 0, 0]

def get_goal_pos():
    return [0.5, 0.5, 0.5, 3.14, 0, 0]

def get_num_bricks():
    return 10

def move_arm(pos):
    msg = MoveArm()
    success = move_arm_wrapper(x = pos[0], y =pos[1], z = pos[2],rot_x =pos[3],rot_y =pos[4],rot_z =pos[5])
    return success
def place_brick():
    place_brick_wrapper(x = 0, y =0, z = 0, rot_x =0, rot_y =0, rot_z =0)

def build_wall(home, pick, place):

def open_gripper():
    pub_gripper.publish(0.1)
    return True
def close_gripper():
    pub_gripper.publish(0.03)
    return True

rate = rospy.Rate(1)


num_bricks = get_num_bricks()
placed = 0
while not rospy.is_shutdown():
    if placed < num_bricks:
        placed += 1
        # place_brick()

        brick = get_brick_pos()
        goal = get_goal_pos()

        open_gripper() #repeated right now
        move_arm(brick)
        close_gripper()
        time.sleep(1)

        goal = get_goal_pos()
        move_arm(goal)
        open_gripper()
        time.sleep(1)


        print("Placed")
        pass
        #Place another brick from stack onto wall
    else:

        rospy.loginfo("Done, " +str(placed) + " bricks placed")
    rate.sleep()
