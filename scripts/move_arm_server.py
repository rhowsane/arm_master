#!/usr/bin/env python

# Import standard libraries
import sys
import copy
import numpy as np
from math import pi

#Import ROS libraries
import rospy
import moveit_commander
import tf
from moveit_commander.conversions import pose_to_list

# Import standard messages
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Float64
from actionlib_msgs.msg import GoalStatusArray
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory

# Import custom messages
from de_msgs.srv import QueryNextPos, MoveArm, QueryPPBrick


# Import custom functions
from arm_server_functions import *


def plan_cartesian_path(goal,resolution = 1): #speed in m/s
    """Sample points between current pose and goal pos at set resolution

    Longer Function Description

    Args:
        param1 (int): The first parameter.
        param2 (str): The second parameter.

    Returns:
        bool: The return value. True for success, False otherwise.

    """

    # Go from where ever you are current to where ever you want to be in a straight line
    #Convert all arrays in 7 array quaterions
    waypoints = []
    wpose = group.get_current_pose().pose
    waypoints.append(copy.deepcopy(wpose))

    #Get current Position in in roll,pitch,yaw

    p = pose_q2array(wpose)
    curr_pos = quat2point(p)
    via_points = get_via_points(curr_pos,goal,res=resolution)
    #all code needed

    return via_points
def move_arm_a_to_b(goal): #move very short distance
    rospy.loginfo('goal')

    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x += 0.0001
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x = goal[0]
    wpose.position.y = goal[1]  # First move up (z)
    wpose.position.z = goal[2]  # and sideways (y)
    quaternion = tf.transformations.quaternion_from_euler(goal[3], goal[4], goal[5]) #(roll, pitch, yaw)
    wpose.orientation.x = quaternion[0]
    wpose.orientation.y = quaternion[1]
    wpose.orientation.z = quaternion[2]
    wpose.orientation.w = quaternion[3]
    waypoints.append(copy.deepcopy(wpose))
    # rospy.loginfo("waypoitns")
    # rospy.loginfo(waypoints)
    group.set_planning_time(4)
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.02,        # eef_step
                                       2)         # jump_threshold
    # rospy.loginfo(goal)

    return plan

def place_brick_handler(req):
    eef_link = group.get_end_effector_link()
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    scene.add_box(box_name, box_pose, size=(0.1, 0.9, 0.1))
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    return wait_for_state_update(box_is_known=True, box_is_attached=False)

def attach_brick_handler():

    return wait_for_state_update(box_is_known=True, box_is_attached=True)

def attach_brick_handler():

    return wait_for_state_update(box_is_known=True, box_is_attached=False)
def move_arm_curve_handler(req):
    # rospy.loginfo(robot.get_current_state())
    # point = [0.5, -0.5, 0, 3.14, 0, 0 ]
    # return True
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = req.x
    pose_goal.position.y = req.y  # First move up (z)
    pose_goal.position.z = req.z # and sideways (y)
    quaternion = tf.transformations.quaternion_from_euler(req.rot_x, req.rot_y, req.rot_z) #(roll, pitch, yaw)
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    group.set_max_acceleration_scaling_factor(0.2)
    group.set_max_velocity_scaling_factor(0.2)

    group.set_pose_target(pose_goal)
    plan = group.plan()
    # points = plan.joint_trajectory.points
    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return True

def move_panda_eef(pose_goal):
    group.set_pose_target(pose_goal)
    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

#accel =


def slow_down(traj):


    new_traj = RobotTrajectory()
    new_traj.joint_trajectory = traj.joint_trajectory
    n_joints = len(traj.joint_trajectory.joint_names)
    n_points = len(traj.joint_trajectory.points)

    spd = 0.8

    for i in range(n_points):
        new_traj.joint_trajectory.points[i].time_from_start = traj.joint_trajectory.points[i].time_from_start / spd

        # rospy.loginfo(type(traj.joint_trajectory.points[i]))
        v = list(new_traj.joint_trajectory.points[i].velocities)
        a = list(new_traj.joint_trajectory.points[i].accelerations)
        p = list(new_traj.joint_trajectory.points[i].positions)

        for j in range(n_joints):
            # rospy.loginfo(type(new_traj.joint_trajectory.points[i].velocities))
            v[j] = traj.joint_trajectory.points[i].velocities[j] * spd
            a[j] = traj.joint_trajectory.points[i].accelerations[j] * spd**2
            p[j] = traj.joint_trajectory.points[i].positions[j]

            # new_traj.joint_trajectory.points[i].accelerations[j] = traj.joint_trajectory.points[i].accelerations[j] * spd
            # new_traj.joint_trajectory.points[i].positions[j] = traj.joint_trajectory.points[i].positions[j]

        v = tuple(v)
        a = tuple(a)
        p = tuple(p)

        new_traj.joint_trajectory.points[i].velocities = v
        new_traj.joint_trajectory.points[i].accelerations = a
        new_traj.joint_trajectory.points[i].positions = p

        # rospy.loginfo( new_traj.joint_trajectory.points[i].velocities[j])
        # rospy.loginfo( new_traj.joint_trajectory.points[i].accelerations[j])
        # rospy.loginfo( new_traj.joint_trajectory.points[i].positions[j])
    return new_traj

def move_arm_handler(req):

    goal = [req.x,req.y,req.z,req.rot_x,req.rot_y,req.rot_z]
    # goal = [0.5,-0.5,0.5,0,3.14,0]

    group.set_goal_position_tolerance(0.001)
    group.set_goal_orientation_tolerance(0.01)

    via_points = plan_cartesian_path(goal,resolution = 1) #res can be changed

    for point in via_points:
        # COMENT THIS OUT
        # move_panda_eef(point)
        # ----------------------------------------------------
        plan = move_arm_a_to_b(point) #
        #Publish this plan at my own speed
        if not real_panda:
            group.execute(plan, wait=False)
            print("EXECUTING PLAN")

            execute(plan)
        else: #Running on real panada
            # plan = slow_down(plan)
            print("EXECUTING PLAN ON REAL ROBOT")

            group.execute(plan, wait=True)

        group.stop()
        group.clear_pose_targets()

    return True

def execute(plan, freq=100): #freq in hz
    # print(plan.joint_trajectory.points)
    override = [0, 0, 0, -0.5, 0, 0.5, 0.75]
    target_pos = plan.joint_trajectory.points
    rate = rospy.Rate(freq)
    for point in target_pos:
        joint_pos = point.positions
        for i in range(7):
            #if i < 6:
                #publishers[i].publish(override[i])
            #else:
            publishers[i].publish(joint_pos[i])
        rate.sleep()

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


if __name__ == '__main__':

    rospy.init_node('arm_server')

    box_name = "box"
    grasping_group = 'hand'

    # ----------------------------------------------
    real_panda = True  # Are you using the real robot?
    # ----------------------------------------------

    if not real_panda:  # If not using real_panda then you need to publish joint angles to gazebo. Create the publishers to do this
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        # To publish joint states directly
        publishers = [rospy.Publisher('/franka/joint{}_position_controller/command'.format(i), Float64, queue_size=1)
                      for i in range(1, 8)]


    else:  # Otherwise connect to the moveit move group which has control over the robot
        rospy.wait_for_message('move_group/status', GoalStatusArray)
        group = MoveGroupCommander('panda_arm')

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # Variables for functions

    move_arm_s = rospy.Service('move_arm', MoveArm, move_arm_handler)
    move_arm_curve_s = rospy.Service('move_arm_curve', MoveArm, move_arm_curve_handler)
    rospy.spin()
