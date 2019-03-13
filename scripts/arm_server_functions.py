import tf
import geometry_msgs.msg
import numpy as np

###############################
#Functions to read in random joint angles and send them at the proper speed


def get_via_points(start, goal, res = 5):
    """Returns sampled points between two end effector positions

    Via points functions to further discretize a tracjectory. It first calculates the diplacement vector between
    points ``start`` and ``goal``. It then samples along that vector direction an amount defined by ``res``

    Args:
        start (list): ``[x, y, z, rot_x, rot_y, rot_z]``. Starting end effector position
        end (list): ``[x, y, z, rot_x, rot_y, rot_z]``. Goal end effector position

    Returns:
        list: Returns list of end effector poses between the two desired location. The number of entries in the list
        is given by ``res``.

    """
    via_points = []
    d_x = goal[0] - start[0]
    d_y = goal[1] - start[1]
    d_z = goal[2] - start[2]
    d_wx = goal[3] - start[3]
    d_wy = goal[4] - start[4]
    d_wz = goal[5] - start[5]

    path_length = np.sqrt(d_x**2 + d_y**2 + d_z**2)
    points = max(int(round(path_length * res)),1) #must always have at least 1 point
    for p in np.arange(points):
        scale = (p+1)/float(points) #full length at last point

        via_x = start[0] + scale * d_x
        via_y = start[1] + scale * d_y
        via_z = start[2] + scale * d_z

        via_wx = start[3] + scale * d_wx
        via_wy = start[4] + scale * d_wy
        via_wz = start[5] + scale * d_wz
        via_p = [via_x,via_y,via_z,via_wx,via_wy,via_wz]
        via_points.append(via_p)

    return via_points


def point2quat(p):
    """Converts end effector position in euler to quaterion domain"""
    quaternion = tf.transformations.quaternion_from_euler(p[3], p[4], p[5]) #(roll, pitch, yaw)
    q = [p[0],p[1],p[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]
    return q

def quat2point(q):
    """Converts end effector position in quaterion to euler domain"""

    orientation_list = [q[3],q[4],q[5],q[6]]
    (roll, pitch, yaw) =tf.transformations.euler_from_quaternion (orientation_list)
    p = [q[0],q[1],q[2],roll, pitch, yaw]
    return p

def pose_e2array(pose):
    """Converts euluer end effector position to a list"""
    p = [pose.positon.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,pose.orientation.z]
    return p
def pose_q2array(pose):
    """Converts quaterion end effector position to a list"""

    p = [pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
    return p

# start = [0.5, -0.5, 0.4,  3.14, 0,  3.14]
# goal = [0.5, 0, 0.5, 3.14, 0, 0]
# via_points = get_via_points(start,goal)
# print(via_points)
