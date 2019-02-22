#class for brick pose
class BrickPose():
    def __init__(self, x, y, z, theta):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta



import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
