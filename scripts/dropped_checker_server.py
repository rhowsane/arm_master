import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

holding = False
def check_gripper_handler(data):
    global holding
    joints = data.position
    width = joints[0] + joints[1]
    if width < dropped_thresh_width:
        holding = False
    else:
        holding = True


def check_if_server(req):
    if holding:
        TriggerResponse(success=True)
    if not holding:
        TriggerResponse(success=True)

rospy.Subscriber("/franka/joint_states", JointState, check_gripper_handler)
brick_manager_s = rospy.Service('check_if_dropper', Trigger, check_if_server)
