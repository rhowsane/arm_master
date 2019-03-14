=========================================
Documentation on spawn_mnger ROS package
=========================================

The main purpose of the `spawn_mnger` *ROS package* is to interface with Gazebo, and spawn objects within
Gazebo during a simulation. Running this code would be more practical than using the 'drag and drop'
interface within Gazebo, as during simulations spawning can be automated at pre-determined locations,
which is especially applicable for having a repeatable 'brick-pickup' procedure in this project.

`spawn.py`_ contains the main code, and defines the ROS service that is called in the `arm_master_main.py`_ main loop.

Setup
----------

Importing necessary functions and services::

    #!/usr/bin/env python
    import rospy, tf, random
    import sys
    import os
    from gazebo_msgs.srv import DeleteModel, SpawnModel # for Gazebo
    from geometry_msgs.msg import Pose # for object orientation

    #initialising node in which the service resides
    rospy.init_node('spawn_brick',log_level=rospy.INFO)

ROS services that are required to spawn and delete objects need to be imported. From ``geometry_msgs``, the message type ``Pose()`` is also required: this is understood by Gazebo as to in what pose
the object needs to be spawned. Whilst it takes in normal (x,y,z) co-ordinates for translation, the
orientation values are different, which will be covered later.

We then initiate a node named ``spawn_brick``, which will be the node in which the latter defined service
resides on.

Defining object pose
---------------------

``Pose()`` takes in quaternion instead of *conventional* (roll, pitch, yaw) Euler angles. Therefore, a
conversion is required, which is conveniently provided by a function within ``tf``.::

    quaternion = tf.transformations.quaternion_from_euler(0,0,1.570796)
    #defining brick orientation, which will be translated into quarternion

    # defining pose of object to be spawned
    initial_pose = Pose()
    initial_pose.position.x = 0.5
    initial_pose.position.y = 0.5
    initial_pose.position.z = 0.2
    initial_pose.orientation.x = quaternion[0]
    initial_pose.orientation.y = quaternion[1]
    initial_pose.orientation.z = quaternion[2]
    initial_pose.orientation.w = quaternion[3]

    # Finding the model file to be spawned
    file = os.path.expanduser('~/.gazebo/models/Brick/model-1_4.sdf')
    f = open(file, "r")
    sdff = f.read()

Afterwards, ``initial_pose`` is defined as a ``Pose()`` type, and the translation and translated
orientation values are written in. **These values should be the same as that within the** ``get_pick_loc()``
**service defined within** ``brick_manager_server.py`` **inside the** ``arm_master`` **ROS package.**::

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

    # function that defines the service gen_brick
    def gen_brick_handler(req):
        i = random.randint(1,5000)
        spawn_model_prox("brick_"+str(i), sdff, "brick_"+str(i), initial_pose, "world")
        resp = TriggerResponse()
        return resp

    #CODE FOR MAKING this node into a service
    from std_srvs.srv import Trigger, TriggerResponse
    gen_brick_s = rospy.Service('gen_brick', Trigger, gen_brick_handler)

    19  # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

The function ``gen_brick_handler()`` takes all the pre-defined pose and spawn instructions and does the
actual spawning in Gazebo. Each object requires a unique ID, therefore a random integer is appended
to ``brick_``.

Finally, the function defined is referenced to be called as a ROS service named ``gen_brick``.