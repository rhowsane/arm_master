#!/usr/bin/env python
import rospy, tf, random
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose

rospy.init_node('spawn_brick',log_level=rospy.INFO)

quaternion = tf.transformations.quaternion_from_euler(0,0,1.570796)

initial_pose = Pose()
initial_pose.position.x = 0.5
initial_pose.position.y = 0.5
initial_pose.position.z = 1
initial_pose.orientation.x = quaternion[0]
initial_pose.orientation.y = quaternion[1]
initial_pose.orientation.z = quaternion[2]
initial_pose.orientation.w = quaternion[3]

f = open("../../../.gazebo/models/Brick/model-1_4.sdf", "r")
sdff = f.read()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

i = random.randint(1,500)

spawn_model_prox("brick_"+str(i), sdff, "brick_"+str(i), initial_pose, "world")


#CODE FOR MAKING this node into a servicec

move_arm_curve_s = rospy.Service('move_arm_curve', MoveArm, move_arm_curve_handler)
