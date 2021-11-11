#!/usr/bin/env python

# Import standard libs.
import rospy
import copy

# Import action lib.
import actionlib

# Import standard msgs/srvs
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerRequest
from franka_gripper.msg import GraspAction, GraspGoal, MoveGoal, MoveAction

# Import custom srv defined in de_msgs
from de_msgs.srv import QueryNextPos, MoveArm, QueryBrickLoc, QueryBrickLocRequest, QueryObjectplaced

# Import custom helper functions
from arm_utils import *
from arm_master_functions import *

# Services for Moving arm
def connect_srv(name, msg_type):
    """Function to connect to services

    Connects to service with queried name. Will wait for service to be available and print status updates

    Args:
        name (str): name of service.
        msg_type (str): msg_type of the service.

    Returns:
        func: returns function object which wraps requested service

    """
    rospy.loginfo("Searching for " + name + " .... ")
    rospy.wait_for_service(name)
    srv_wrapper = rospy.ServiceProxy(name, msg_type)
    rospy.loginfo(name + " CONNECTED")
    return srv_wrapper


# Functions to further wrap function calls. Allows for changing of underlying function
# without effecting the main loop



def gen_caisse():
    """Generates brick in Gazebo. See Spawn Manager Package for more details"""

    gen_caisse_wrapper(TriggerRequest())

def gen_mobile():
    """Generates brick in Gazebo. See Spawn Manager Package for more details"""

    gen_mobile_wrapper(TriggerRequest())

def gen_chargeur():
    """Generates brick in Gazebo. See Spawn Manager Package for more details"""

    gen_chargeur_wrapper(TriggerRequest())

def gen_couverture():
    """Generates brick in Gazebo. See Spawn Manager Package for more details"""

    gen_couverture_wrapper(TriggerRequest())


def check_dropped(placed):
    """Checks if brick is dropped

    Calls holding_brick service which checks the width of gripper and compares it to the expected width

    Returns:
        bool: True if gripper closer together then expect (means brick has fallen). False otherwise.
    """
    asn = holding_brick_wrapper(QueryObjectplacedRequest(placed))
    rospy.loginfo("ANSWERS: ")
    rospy.loginfo(asn)
    return asn.success


def get_brick_pos(placed):
    """Queries location to pick up brick"""

    loc = get_pick_loc_wrapper(QueryBrickLocRequest(placed))
    p = [loc.x, loc.y, loc.z, loc.wx, loc.wy, loc.wz]
    p = orientation_correct(p)
    # return [0.5, 0.5, 0.15, 3.14, 0, 3.14/4] #Override for Debugging
    return p


def get_goal_pos(placed):
    """Queries location to place brick"""

    loc = get_place_loc_wrapper(QueryBrickLocRequest(placed))
    p = [loc.x, loc.y, loc.z, loc.wx, loc.wy, loc.wz]
    p = orientation_correct(p)
    # return [0.5, -0.5, 0.15,  3.14, 0,  3.14/4] #Override for Debugging
    return p


def get_home_pos():
    """Queries home location to return too"""

    p = [0.5, 0, 0.5, 0, 0, 0]
    p = orientation_correct(p)
    return p


def get_over_pos():  # not used
    return [0.5, 0.5, 0.5, 3.14, 0, 0]


def get_num_objects():
    """Queries number of objects panda should place"""

    return 3


def orientation_correct(pose):
    """Corrections so that the end effector is aligned as expect. New default is facing downwards
     with gripper opening along y-axis"""

    pose[2] += 0.04
    pose[3] += 3.14
    pose[4] += 0
    pose[5] -= 3.14 / 4

    return pose


def move_arm(pos):
    """Move arm to queried position

    This function calls the move arm service defined in move_arm_server.py. Arm will move following a cartesian path between its current
    position and the desired position.

    Args:
        pos (list): ``[x, y, z, rot_x, rot_y, rot_z]``.

    Returns:
        bool: True when motion of arm completes, regardless of wether it was succesful enough.

    """

    msg = MoveArm()
    rospy.loginfo(pos)
    success = move_arm_wrapper(x=pos[0], y=pos[1], z=pos[2], rot_x=pos[3], rot_y=pos[4], rot_z=pos[5])

    return success


def move_arm_curve(pos):
    """Move end effector to queried position

    This function calls the move arm curved service defined in move_arm_server.py. The Arm will execute a RRT planner to
    determine a path from its current position to the desired location. Helpful when as it does not

    Args:
        pos (list): [x, y, z, rot_x, rot_y, rot_z].

    Returns:
        bool: True when motion of arm completes, regardless of whether it was successful or not.

    """
    success = move_arm_curve_wrapper(x=pos[0], y=pos[1], z=pos[2], rot_x=pos[3], rot_y=pos[4], rot_z=pos[5])
    return success


def pick_up(target, placed, via_offset=0.4):
    """Pick up object at target location

    First moves the arm to a way point a set offset above the target location (set by ``via_offset``). Then lowers end-effector
    to target and closes the gripper. Finally moves arm back up to offset location.

    Args:
        target (list): [x, y, z, rot_x, rot_y, rot_z]. End effector target position
        via_offset (float): z-offset for way point to reach before going to target location

    Returns:
        bool: True when motion of arm completes, regardless of whether it was successful or not.

    """

    global holding_object  # use global var

    # First Move to point above the pick up location
    via_point = copy.deepcopy(target)
    via_point[2] += via_offset  # Z offset


    move_arm(via_point)  # Move arm to just above goal
    move_arm(target)  # Lower arm down to goal
    # rospy.sleep(0.5) # Play with timming in here to get desired behaviour
    close_gripper(placed)  # Grasp around object

    holding_object = True
    move_arm(via_point)  # Move back to via point

    return True


def place_down(target, via_offset=0.4):
    """Place down object at target location

    First moves the arm to a way point a set offset above the target location (set by ``via_offset``). Then lowers end-effector
    to target and opens the gripper. Finally moves arm back up to offset location.

    Args:
        target (list): [x, y, z, rot_x, rot_y, rot_z]. End effector target position
        via_offset (float): z-offset for way point to reach before going to target location

    Returns:
        bool: True when motion of arm completes, regardless of whether it was successful or not.

    """

    global holding_object  # use global var

    # First Move to point above the pick up location
    via_point = copy.deepcopy(target)
    via_point[2] += via_offset  # Z offset
    move_arm(via_point)  # Move arm just above goal
    move_arm(target)  # Move arm down to goal
    open_gripper()  # Open hand
    # rospy.sleep(1) # Tune time, mabye wait for object to fully drop

    holding_object = False
    move_arm(via_point)

    return True


# Get circular path planning waypoints around the robot
def move_towards(start, end, round_way_points, placed, check=False):
    """Move arm towards end location following way points around a virtual circle centered on the robot

    Moves end effector towards a desired goal position by following circular way points along circle centered at panda base.
    Important, to avoid moving arm into the robot as moveit compute cartesian path does not account for joint limits.

    Args:
        start (list): [x, y, z, rot_x, rot_y, rot_z]. End effector way point close to the current position of robot.
        end (list): [x, y, z, rot_x, rot_y, rot_z]. Desired end effector pose.
        round_way_points (dict)**: dict[point_id] = (pos, neighbour). pos and neighbour are of the following form:
        pos = [x_pos, y_pos, z_pos], neighbour = [left_id, right_id]. Intermediate points to travel between start and end.
        check (bool): Set true to check if robot has dropped the object during movement.

    Returns:
        bool: The return value. True for success, False otherwise.

      """
    # find nearest point to pick
    min_start_dist = 10000
    min_start_ind = 0
    min_end_dist = 10000
    min_end_ind = 0

    for key, value in round_way_points.items():
        # print(key, value)
        p = value[0]
        dist_start = distance(start, p)
        dist_end = distance(end, p)

        if dist_start < min_start_dist:
            min_start_dist = dist_start
            min_start_ind = key

        if dist_end < min_end_dist:
            min_end_dist = dist_end
            min_end_ind = key

        #rospy.loginfo(p)

    curr_ind = min_start_ind

    selector = left_or_right(curr_ind, min_end_ind, round_way_points)
    while curr_ind != min_end_ind:

        if check:  # Check if dropped
            rospy.loginfo("CHECKING IF DROPPED")
            if check_dropped(placed):  # Exit and return failure
                rospy.loginfo("DROPPED OBJECT!")
                return False

        # move arm to the curr node positon
        curr_node = round_way_points[curr_ind]
        #print("MOVING ARM")
        #print("CURR NODE Z: ", curr_node[0][2])

        move_arm([curr_node[0][0], curr_node[0][1], curr_node[0][2]+0.1, 3.14, 0, 3.14 / 4])
        curr_ind = curr_node[1][selector]  # go one way around the circle
    return True
    # move toward location in a controlled manner without running into


def go_to(pos):
    """Move end effector to pos location in a straight line"""
    move_arm(pos)


# change these gripper functions to the correct topic for panda arm
def open_gripper():
    """Opens robot gripper

    If using ``real_panda`` then call move gripper action server. If using gazebo then publish desired
    grip width to */franka/gripper_width* topic.

    Returns:
        bool: returns True when done.

      """

    if real_panda:
        client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        client.wait_for_server()
        goal = MoveGoal(width = 0.08, speed = 0.04)
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(5.0))
    else:
        pub_gripper.publish(0.12)
        rospy.sleep(2)
    return True


def close_gripper(placed):
    """Closes robot gripper

    If using ``real_panda`` then call move gripper action server. If using gazebo then publish desired
    grip width to */franka/gripper_width* topic.

    Returns:
        bool: returns True when done.

      """
    if real_panda:#MODIFIER!!!
       client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
       rospy.loginfo("CONNECTING")
       client.wait_for_server()
       action = GraspGoal(width=0.5,speed=0.08,force=1)
       rospy.loginfo("SENDING ACTION")
       client.send_goal(action)
       client.wait_for_result(rospy.Duration.from_sec(5.0))
       rospy.loginfo("DONE")
    else:
	if placed==0: #pick mobile
		pub_gripper.publish(0.06)#0.06
		rospy.sleep(2)
	elif placed==1: #pick chargeur
		pub_gripper.publish(0.008)
		rospy.sleep(2)
	elif placed==2: #pick couverture
		pub_gripper.publish(0.080)
		rospy.sleep(2)
    return True


###################################################################
# MAIN CODE
###################################################################

if __name__ == '__main__':

    rospy.init_node('arm_master', anonymous=True)

    # Get round way points for circular path planning
    circle_points = get_round_points()

    # ----------------------------------------------
    global real_panda    
    real_panda = False
    # ----------------------------------------------

    # Set up connections with services and topics

    # Create publishers to control gripped width in gazebo simulation
    pub_gripper = rospy.Publisher('/franka/gripper_width',
                                  Float64, queue_size=1)

    # Get all Services

    # Services for moving arm
    move_arm_wrapper = connect_srv('move_arm', MoveArm)
    move_arm_curve_wrapper = connect_srv('move_arm_curve', MoveArm)

    # gen_object generates an object in Gazebo
    if not real_panda:
        gen_caisse_wrapper = connect_srv('/gen_caisse', Trigger)
	gen_mobile_wrapper = connect_srv('/gen_mobile', Trigger)
	gen_chargeur_wrapper = connect_srv('/gen_chargeur', Trigger)
	gen_couverture_wrapper = connect_srv('/gen_couverture', Trigger)

    # Services for querying pick and place locations
    get_pick_loc_wrapper = connect_srv('get_pick_loc', QueryBrickLoc)
    get_place_loc_wrapper = connect_srv('get_place_loc', QueryBrickLoc)

    # Service for checking if brick fell out of panda's hand
    holding_brick_wrapper = connect_srv('check_if_dropped', Trigger)

    rate = rospy.Rate(1)

    # Define deafult variables that will be used in main loop
    holding_object = False
    dropped_object = False
    closed_width = 0.5
    brick_width = 0.6
    dropped_thresh_width = 0.055

    num_bricks = get_num_objects()
    placed = 0

    # MOVE ARM TO STARTING LOCATION
    last_goal = None
    # move_arm_curve(get_brick_pos()) #start in location so you can go back to where you came from

    # make sure gripper is open to begin with
    go_to(get_home_pos())
    close_gripper(placed)
    open_gripper()


    while not rospy.is_shutdown():  # Main Control Loop for the arm
        if placed < num_bricks:  # Continue to loop until you have placed the correct number of bricks

            # Query Positions
            brick = get_brick_pos(placed)
            goal = get_goal_pos(placed)

            if goal == last_goal:  # same as last time, don't go back
                continue
            home = get_home_pos()
            over_head = get_over_pos()

            if not real_panda:
                gen_caisse()
		gen_mobile()
		gen_chargeur()
		gen_couverture()
	    #rospy.loginfo('Begginning of the first move_towards')
            #succ = move_towards(home, brick, circle_points,placed)

            # Pick Place operation then return home
	    rospy.loginfo('Begginning of the first pick_up')
            pick_up(brick,placed)
            '''succ = move_towards(brick, goal, circle_points, placed,check=False)

            if not real_panda: #Functionality to return to brick location if you dropped it.
                if not succ:
                    brick_via = brick
                    brick_via[2] += 0.2
                    go_to(brick_via)
                    continue #continue, don't increment placed'''

            place_down(goal)
	    rospy.loginfo("Finishing all movements")
            rate.sleep()
            succ = move_towards(goal, home, circle_points,placed)
            placed += 1
            last_goal = goal  # placed down now its a last brick

            rospy.loginfo("Placed")
            # Place another brick from stack onto wall

        else:  # When done just wait
            rospy.loginfo("Done, " + str(placed) + " bricks placed")
        rate.sleep()
