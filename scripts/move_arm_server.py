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
from de_msgs.srv import QueryNextPos, MoveArm, QueryPPBrick
# rospy.init_node('arm_controller', anonymous=True)


rospy.init_node('arm_server')

publishers = [rospy.Publisher('/franka/joint{}_position_controller/command'.format(i), Float64, queue_size=1) for i in range(1, 8)]
moveit_commander.roscpp_initialize(sys.argv)

# rospy.init_node('move_group_python_interface_tutorial',
#                 anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

#variables for functions
box_name = "box"
grasping_group = 'hand'

def plan_cartesian_path(goal):
    # Go from where ever you are current to where ever you want to be in a straight line
    #Convert all arrays in 7 array quaterions
    waypoints = []
    wpose = group.get_current_pose().pose
    waypoints.append(copy.deepcopy(wpose))

    #all code needed
    wpose.position.x = goal[0]
    wpose.position.y = goal[1]  # First move up (z)
    wpose.position.z = goal[2]  # and sideways (y)
    quaternion = tf.transformations.quaternion_from_euler(goal[3], goal[4], goal[5]) #(roll, pitch, yaw)
    wpose.orientation.x = quaternion[0]
    wpose.orientation.y = quaternion[1]
    wpose.orientation.z = quaternion[2]
    wpose.orientation.w = quaternion[3]
    waypoints.append(copy.deepcopy(wpose))
    rospy.loginfo(goal)
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       15)         # jump_threshold
    return plan, fraction

def place_brick_handler(req):
    eef_link = group.get_end_effector_link()
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    scene.add_box(box_name, box_pose, size=(0.1, 0.9, 0.1))
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    rospy.loginfo("Placing Break")
    return wait_for_state_update(box_is_known=True, box_is_attached=False)

def attach_brick_handler():

    return wait_for_state_update(box_is_known=True, box_is_attached=True)

def attach_brick_handler():

    return wait_for_state_update(box_is_known=True, box_is_attached=False)
def move_arm_curve_handler(req):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = req.x
    pose_goal.position.y = req.y  # First move up (z)
    pose_goal.position.z = req.z # and sideways (y)
    quaternion = tf.transformations.quaternion_from_euler(req.rot_x, req.rot_y, req.rot_z) #(roll, pitch, yaw)
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    group.set_pose_target(pose_goal)
    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # print(plan)
    group.stop()
    group.clear_pose_targets()
    return True

def move_arm_handler(req):

    goal = [req.x,req.y,req.z,req.rot_x,req.rot_y,req.rot_z]
    # goal = [0.5,-0.5,0.5,0,3.14,0]

    group.set_goal_position_tolerance(0.001)
    group.set_goal_orientation_tolerance(0.1)
    plan, frac = plan_cartesian_path(goal)
    group.set_max_velocity_scaling_factor(0.005)
    group.set_max_acceleration_scaling_factor(0.005)
    group.execute(plan, wait=True)
    rospy.loginfo(plan)
    group.stop()
    group.clear_pose_targets()
    return True

def wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4):#talen from tutorial

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
    # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

# If we exited the while loop without returning then we timed out
    return False

move_arm_s = rospy.Service('move_arm', MoveArm, move_arm_handler)
move_arm_curve_s = rospy.Service('move_arm_curve', MoveArm, move_arm_curve_handler)

rospy.spin()
