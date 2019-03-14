
Arm Master Package
========================


Arm Master is the *ROS Package* which controls the arm. At the most basic level, its purpose is to query goal
end effector poses, and interpolate their from the arms current position.

Code for this computation is located in python scripts. Many of which are exposes them selves as ROS node, but also some that simply
provide function calls.

File Structure
-----------------
The core of the package files are structure as follows::

    arm_master
    ├── docs
    └── launch
        ├── light.launch
        ├── panda_one_brick.launch
        └── sim.launch
    └── scripts
        ├── arm_master_main.py
        ├── arm_master_functions.py
        ├── move_arm_server.py
        ├── arm_server_functions.py
        ├── arm_utils.py
        └── dropped_checker_server.launch
    ├── CMakeLists.txt
    └── package.xml

Launch files `light.launch`_, `panda_one_brick.launch`_, `sim.launch`_ run all nessecary ros nodes.
Refer to *launch* for more information on running the code

The *scripts* are structured such that there is a script which defines the ROS related code, and a script which provides ROS free functions. To illustrate observe the difference between
``arm_master_main.py`` and ``arm_master_functions.py``

Once running, the panda arm loops through a control sequence defined in `arm_master_main.py`_,
which calls services and publishes to topics defined in ``move_arm_server.py``

I will explain this control squence in the remainder of this section by going line by line
through arm master main and explaining the neseecary conecepts required to understand what is going on


Real Panda
-----------------------------------

At the start of the runnable code, a variable ``real_panda`` is defined. It is essential this is set correctly
depending on wether you wish to run the code on gazebo or on the real franka panda.


Setting this variable to ``False``, ensures that the code doesn't wait for services from the real robot which will
not apear on the ROS network

.. warning::
   The ``real_panda`` variable also needs to be set in `move_arm_server.py`_

.. warning::
   If you find that the code is not running and is getting hanged up at launch time it could be beacuse the arm_master_main
   is waiting for all the required nodes to be launched. Double check ``real_panda`` is correct.
   Note that the code calls ``rospy.wait_for_service()`` each time it is required to connect to another service
   or action client
   service or action client.

   .. literalinclude:: ../scripts/arm_master_main.py
      :lines: 50-69


Loop
-----------------------------------
The main loop has the following structure::

  #Continue to loop until you have placed the correct number of bricks
  while not rospy.is_shutdown():
        if placed < num_bricks:

            #Query Positions
            brick = get_brick_pos()
            goal = get_goal_pos()

            #Move towards brick and pick up
            pick_up(brick)

            #Move from pick up positon to place down position
            move_towards(brick, goal)

            #Place down brick
            place_down(goal)

            # Increment num of brick placed
            placed += 1


If you wish to change how the arm moves, change the order in which the ``pick_up()``, ``place_down()``, ``place_down()``
functions are called. Additional motion functions also available in ``arm_master_main.py`` are ``go_to()`` and ``move_arm_curve()``. To illustrate, The main loop for our project implementation was implemented as follows:

.. literalinclude:: ../scripts/arm_master_main.py
  :lines: 50-69


Behind the Scences
-----------------------------------

I know will explain some more of the theoretical aspects of what happens when a motion function like ``pick_up()`` is called in
``arm_master_main.py``.

Pick Up
++++++++++++++++++++++


The pick up function in full is::

    def pick_up(target, via_offset=0.3):

        global holding_brick  # use global var

        # First Move to point above the pick up location
        via_point = copy.deepcopy(target)
        via_point[2] += via_offset  # Z offset

        move_arm(via_point)  # Move arm to just above goal
        move_arm(target)  # Lower arm down to goal
        # rospy.sleep(0.5) # Play with timming in here to get desired behaviour
        close_gripper()  # Grasp around brick

        holding_brick = True
        move_arm(via_point)  # Move back to via point

        return True


It is queried using a target end effector position of where the pick up will happen (defined by a
``[x, y, z, rot_x, rot_y, rot_z]`` list) and a ``via_offset`` parameter which determines how high the above the brick it travels before
lowering and picking it up.

Pictorially the function of ``pick_up()`` looks like ::

**INSERT IMAGE**

first ``move_arm(via_point)`` is called. This calls the function::

    def move_arm(pos):

    msg = MoveArm()
    rospy.loginfo(pos)
    success = move_arm_wrapper(x=pos[0], y=pos[1], z=pos[2], rot_x=pos[3], rot_y=pos[4], rot_z=pos[5])

    return success

which further provides a wrapper to the service `move_arm`::

    move_arm_wrapper = connect_srv('move_arm', MoveArm)

All arm movment services are defined in the ``move_arm_server.py``. When a requested is send to the ``move_arm`` service,
the ``move_arm_handler(req)`` function defined inside ``move_arm_server.py`` is called::

    def move_arm_handler(req):

        goal = [req.x,req.y,req.z,req.rot_x,req.rot_y,req.rot_z]
        # goal = [0.5,-0.5,0.5,0,3.14,0]

        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.01)

        via_points = plan_cartesian_path(goal,resolution = 1) #res can be changed

        for point in via_points:
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



Depdning on wether your running on the real robot or gazebo, how the plan is executed changes. But the fundamental planning of the path doesn't.

First a set of end_effector via_points are determined between the current robot position and the goal position. This is done by calling
`` plan_cartesian_path(goal,resolution = 1)`` which then calls ``get_via_points(curr_pos,goal,res=resolution)``. ``get_via_points()`` is function
defined in the ``arm_server_functions.py`` files. ``get_via_points()`` essentially determines the dispalcment vector between the start and goal
position and then samples vectors along the same direction at various resolutions. Pictorally the operation is as follows::

**INSERT IMAGE**


While much of this sampling computation can be accomplished using the the ``compute_cartesian_path()``, it gives up addtional flexibility and control over the positon
of waypoints, and always use to break up the movment into smaller chunkcs. Once ``via_points`` have been obtained, it time to plan a path through the determined points.
This is done using the ``move_arm_a_to_b()`` function::

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

        group.set_planning_time(4)
        (plan, fraction) = group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.02,        # eef_step
                                           2)         # jump_threshold
        # rospy.loginfo(goal)

        return plan


The ``move_arm_a_to_b()`` function utilises moveit to solve IK along the desired path. First it gets the current positon of the robot from the move group interface::

 wpose = group.get_current_pose().pose

It then gets the desired end effector position (``[x, y, z, roll, pitch, yaw]``) defined with euler angles and changes it to a quaternion representation
(``[x, y, z, X, Y, Z, W]``). The quaternion representation is equivalent to the euler angles, but rather then represent a rotation with 3 seperate rotations around
linearly indepdent axis (like with eulers desccription), a 4D vector is used. This 4D vector has advantages in that it doesn't degengrate in certain rotation squences and
thus can be seen as more general. That said, it is not intutive to work with. All poses in our code base are encoded with the Euler description and trasnformed to
a quanterion at the last moment using the ``tf.transformations`` function.

Now that the goal pose is describded in the same vector space as the current position, a linear interpolation can be calculated between the two. For this purpose,
the ``compute_cartesian_path`` function is used. This function first samples points along the straight line between the waypoints, the distance between the points is given by the
``eef_step`` parameter. It then solves IK for each of thoose sampled points. As the robot is redudant (7 DOF for a task which requires at most 6 DOF), it is able to find many solutions to
the IK problem. Mathetically this means that that null space of the jacobean contains vectors other than the zero vector. Redundancy resolution is specified such that IK solution minimises the distance from the
previous olution. Specifically we specificy in the ``jump_threshold`` parameter that the difference in joint angles in neighbouring IK solutions can be no greater than 2 radians.

The resulting output is a series of joint angles which describe an arm trajectory along the line between the current and goal position.


.. note::

    Because we always specify the end effector of the robot to be pointing downwards, it remains pointing downwards during the interpolation between the current and
    goal position. Just the x, y, z position of the end effector changes.


Focusing back on the ``move_arm_handler(req)`` function. The next step is to exectue that path. Regardless of wether the robot is running on gazebo or on the real robot.
The desired joint angles are used to update the set point on the robot's PID controller, virtual or real. This results in a error between current and desired joint angles,
which results in a proportional gain to be applied to the motors and ultimetly arm movement. Sending all the joint angles in
succession and the arm will track the desired end effector movment. Key parameters here are the frequency at which the joint angles are published and the distance between the joint angles.
As only a feedback is being used to control the robot, extremly large steps in joint angles will lead to un antural arm behaviour.

This summarizes the main computation and consideratoins behind moving the panda arm. We now focus back on the ``pick_up()`` function, you will see that picking up the brick is simply
a matter of asking the robot arm to move first from its current position to a via point a set z-offset above the brick. lowering down to just above the brick,
closing the gripper around the brick, and then returning to the via point.

Motion happens as in the paragraphs described above. While there are slight differences between controlling the gripper in gazebo vs on the real robot, the essence is to publish
a desired gripper width to a topic that is being read by a controller on the franka gripper.


Move towards
+++++++++++++++

Move towards is the other main motion function called in ``arm_master_main.py``. The mechanics through which it moves remain the same as described previously. After a few layer of functions,
it calls the exact same ``move_arm()`` function. The difference in ``move_towards()`` is how the way points are selected. Pictorall what happens when you call the function is as follows::

**INSERT DIAGRAM OF MOVE TOWARDS**

Stepping through the function you will see exactly this behaviour implemented. First the closest points on the circular
to the goal and start location are determined. These will become the entry and exit points to the circle::

    def move_towards(start, end, round_way_points, check=False):

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

            print(p)


Once the starting point is determined, one must then descide wether to go left or right around the circle. This computation is done the ``left_or_right()`` function::


        curr_ind = min_start_ind

        selector = left_or_right(curr_ind, min_end_ind, round_way_points)


        while curr_ind != min_end_ind:

            if check:  # Check if dropped
                rospy.loginfo("CHECKING IF DROPPED")
                if check_dropped():  # Exit and return failure
                    rospy.loginfo("DROPPED BRICK!")
                    return False

            # move arm to the curr node positon
            curr_node = round_way_points[curr_ind]
            print("MOVING ARM")
            print("CURR NODE Z: ", curr_node[0][2])


Finally the arm is moved to the selected way point untill it have reached the way point with the exit id::


            move_arm([curr_node[0][0], curr_node[0][1], curr_node[0][2]+0.1, 3.14, 0, 3.14 / 4])
            curr_ind = curr_node[1][selector]  # go one way around the circle
        return True


Here is more detailed information for the specific motion functions. Vist the :doc:`../api_reference.rst` API Reference section of the documentation for more information

arm_master_main Functions
-----------------------------------

.. automodule:: scripts.arm_master_main
  :members:

move_arm_server Functions
-----------------------------------

.. automodule:: scripts.move_arm_server
  :members:


arm_server_functions Functions
-----------------------------------

.. automodule:: scripts.arm_server_functions
  :members:

.. _arm_master_main.py: https://github.com/de3-robo/arm_master/blob/master/scripts/arm_master_main.py
.. _move_arm_server.py: https://github.com/de3-robo/arm_master/blob/master/scripts/move_arm_server.py
.. _brick_manager_server.py: https://github.com/de3-robo/arm_master/blob/master/scripts/brick_manager_server.py
.. _light.launch: https://github.com/de3-robo/arm_master/blob/master/scripts/arm_master_main.py
.. _panda_one_brick.launch: https://github.com/de3-robo/arm_master/blob/master/scripts/arm_master_main.py
.. _sim.launch: https://github.com/de3-robo/arm_master/blob/master/scripts/arm_master_main.py


In python scripts:


Installation


testing a random code block::

  if x:
    z = 1
  else:
    pass
  x = True


arm_master is a ros package which provides the nessecary control flow functions for controlling the panada arm.
Let me see how this deals with spaces. I am interested
to be honest
Very Interested

At the command line:

easy_install crawler

Or, if you have pip installed:

pip install crawler

*italics*
**boldface**

``code sample``


* bullet 1.
* bullet 2.

1. num List
2. num list

#. One List

  #. nested


.. _reference-name:

Cool section
-------------

:ref:`reference-name`
