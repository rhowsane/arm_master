Running on Real Panda
========================

*"Pure logical thinking cannot yield us any knowledge of the empirical world; all knowledge of reality starts from
experience and ends in it."*

*- Albert Einstein*

The ultimate goal of the robotics project should be to run on the real panda robot. By doing so you take full advantage
of the resources Imperial has, and get the awesome feeling of making a real robot move: a robotic arm that you have
programmed yourself.

The aim of this section of the documentation is to bridge that gap between simulation and the real robot. In the ideal case,
no change is needed, your code can simply continue to publish to the same ROS topics, which now are subscribed to by a real
rather then simulated robot. Unfortunately this is not the case with the Panda platform.

.. note::

    For this part of the project you will need to run on the computer in the robotics lab. It is connected to a *real time
    kernal* which allows it to communicate with the panda arm at an extremely high frequency.

You may find that a workspace is already setup. In that case simply pull the ROS packages mentioned in the **Setup** section
of the the documentation into the ``src``. directory and build using what ever convention is used. (could potentially be ``catkin_make``).

In the case that there is no existing workspace, follow the steps in **Set Up** to get started.

Once you have the workspace set up you will need to install 2 additional ROS packages.

* `franka_ros`_
* `lib_franka`_

Installation instructions for these two packages are described on the `Franka documentation site`_

.. note::

    While the newest version of ``franka_ros`` is readily available online, we have made small adjustments in the source codes of our version.
    If you want to have the identical setup to that used in this project, we recommend you download the versions
    of ``franka_ros`` available on our repo. Changes made to the source code will be further described bellow.


As described in the `Franka documentation site`_ there are a number of options for interfacing with the real Panda. Ultimately what worked
for us was doing the control through *MoveIt*. Included in the ``franka_ros`` package is already the code for making this work,
which greatly simplifies the process and means that you do not need to write a custom c++ controller or deal directly with
``lib_franka``.


Changing Code
---------------------------------------


Our source code is written such that only two variables need to be changed to switch from running on *Gazebo* to running on the real robot.
In ``arm__master_main.py`` and ``move_arm_server.py`` changed::

    real_panda = True

This has a number of knock on effects which are automatically controlled by ``if`` statements in the code.

In ``arm__master_main.py``, the node no longer waits for the ``/gen_brick`` service which is only relevant to *Gazebo*::

     if not real_panda:
            gen_brick_wrapper = connect_srv('/gen_brick', Trigger)


and behaviour in the ``open_gripper()`` and ``close_gripper()`` function changes from publishing to a topic which is read by the simulated Gazebo robot,
to calling a action server which is exposed by the actually Panda robot::

   def open_gripper():

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


In ``move_arm_server.py``, you no longer create the joint angle publishers for the Gazebo robot and instead you connect directly to the *MoveIt* move_group which is
in sync with the real robot::

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

The most important difference is in the ``move_arm_handler()`` function. When running on the real panda, you execute trajectories on *MoveIt* and wait until it completes. This is because
*MoveIt* deals with the control of the robot. As the robot in *MoveIt* moves, so does the real robot. On the *Gazebo* robot, there where separate virtual PID controllers for moving the robot. Thus we simply run the code on moveit for consistency between the
Gazebo and Moveit robot, but we do not wait for the Moveit robot to finish its path before then moving the simulated robot::

     if not real_panda:
            group.execute(plan, wait=False)
            print("EXECUTING PLAN")

            execute(plan)
        else: #Running on real panada
            # plan = slow_down(plan)
            print("EXECUTING PLAN ON REAL ROBOT")

            group.execute(plan, wait=True)

Getting our code to run on the real robot was a trial and error process. Here we outline some of the changes and learnings we made in the hope that you will not
have to make the same mistakes.

More tweaks in franka_ros
---------------------------------------

A number of tweaks where made in order enable to panda arm to move successfully. Straight out of the box, the robot was performing extremely sub-optimally.
It's motion was extremely slow, yet it was still returning ``acceleration discontinuity`` errors because supposedly it was moving to fast. This initial testing was done using the provided
``joint_position_example_controller.cpp``.

We then switched to using the ``joint_velocity_example_controller.cpp`` and made small modifications to the file. The robot moved noticeably faster. This gave us hope that the current robot limitations where programmatic
and not hardware related.

Ultimately the most meaningful change was in ``franka_hw/src/franka_hw.cpp``. Here we::

    joint_limits.max_acceleration = franka::kMaxJointAcceleration[i]*5;
    joint_limits.max_jerk = franka::kMaxJointJerk[i]*5;

 Increased the max allowable joint acceleration and jerk. We also increased the parameters on the low pass filter for real time control on the robot::

    auto limit_rate = get_limit_rate_()*100;
    auto cutoff_frequency = get_cutoff_frequency_()*10;


During initial testing, this seemed to actually let the panda robot to move without the controllers dying. In final testing, we noticed the Panda PID would often begin to randomly diverge.
While we kept these changes in our deployed code, in hindsight, the PID problem was probably caused by these changes.

.. warning::
    For future scope I would suggest reverting these changes. For us, it made it work, but I personally believe changes made to the panda_moveit_config are what allowed the robot
    to successfully move.


Tweaks in panda_moveit_config
---------------------------------------

One error we noticed that was causing the robot controllers to crash was that there was to large a difference between the internal moveit representation of the robot and the actual robot.
To this effect we made one extremely important change which I believe enabled control of the real panda robot through moveit to work.

From the ``launch/trajectory_exectuion.launch.xml`` file in the ``panda_moveit_config`` we increased the allowable time for the trajectory to finish and the joint tolerance::

      <!-- When determining the expected duration of a trajectory, this multiplicative factor is applied to get the allowed duration of execution -->
      <param name="trajectory_execution/allowed_execution_duration_scaling" value="1.2*5"/> <!-- default 1.2 -->
      <!-- Allow more than the expected execution time before triggering a trajectory cancel (applied after scaling) -->
      <param name="trajectory_execution/allowed_goal_duration_margin" value="0.5*5"/> <!-- default 0.5 -->
      <!-- Allowed joint-value tolerance for validation that trajectory's first point matches current robot state -->
      <param name="trajectory_execution/allowed_start_tolerance" value="0.01*5"/> <!-- default 0.01 -->


we also edited ``config/joint_limits.yaml`` to increase ``max_velocity`` and  ``max_acceleration`` limits::

    joint_limits:
      panda_joint1:
        has_velocity_limits: true
        max_velocity: 2.1750 * 1.5
        has_acceleration_limits: true
        max_acceleration: 3.75 * 1.5


While the sum of these changes enabled control of the Panda robot through the moveit interface, we are unable to determine whether some of these changes where meaningless. Our intuition certainly
told us that the change to the ``trajectory_exectuion.launch.xml`` files was important and that the changes in ``ranka_hw.cpp`` can potentially be reverted.


Getting the Gripper to work
---------------------------------------

During testing, we were also having a lot of trouble with the panda gripper. By launching the panda gripper node, a number of action services are exposed. (refer to `franka_ros documentation`_ for more
information.)

Initially, we utilized the ``Move`` server to grasp the brick::

        client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)

However the tolerance on reaching the goal position seemed to be to small, so the action would fail as the robot wouldn't be able to release brick. The action server would attempt to close
the gripper to a certain width, but because of the brick in between not being aligned properly, it wouldn't be able to close fully and 'complete' the action. We attempted to fix this by first better tuning the goal
widths to better match the actual brick width. During the subsequent testing this seemed to work, but the gripper remained extremely sensitive to off-angle pick ups and would have a gripper failure after around placing 6 bricks.

Other fixes we tried included making two separate clients for opening and closing, such that opening would continue to work even if the closing action got hung up.

We increased the wait time for the action to reach the goal significantly, but this did not solve the root problem either::

        client.wait_for_result(rospy.Duration.from_sec(10.0))

Finally, we also found a configuration file for the action where we could edit the goal tolerances directly. From the ``franka_ros`` package, inside the file ``franka_gripper/config/franka_gripper_node.yaml``::

    default_grasp_epsilon:
      inner: 0.02 # [m]
      outer: 0.02 # [m]

Where epsilon is a parameter which determines the goal tolerance for the action. Ultimately, none of these fixes could over come to the problem of the gripper getting stuck holding a brick. We kept the move action
server for opening the gripper, but we switched to using the the Grasp action server provided by the ``franka_gripper`` package for pick up. The grasp action server is specifically designed for pick and place operation and accounts for the
fact that the gripper will not be able to fully close::

    client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    client.wait_for_server()
    action = GraspGoal(width=0.5,speed=0.08,force=1)

You now also have the functionality to define how hard the gripper will grasp the brick.

Further slowing down the arm
---------------------------------------

Even with all the changes mentioned above we were still having issues with the controller dying to position and acceleration discontinuity errors. Initially to fix this issue we attempted to both increase the resolution
of our trajectory, such that the difference in joint angles per step in trajectory was extremely small. In package ``arm_master``, file ``move_arm_server.py``, we increased the resolution of the cartesian path planner and decreased
the jump threshold, which is the maximum allowable difference in joint angles for IK solutions along the trajectory::

    (plan, fraction) = group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.001,        # eef_step
                                           1.5)         # jump_threshold


We also wrote a function to take the planned path and slow down the trajectory further. While this *slow down* functionality is provided for joint angle and end effector position goal planning in moveit, it is not available for the
``compute_cartesian_path()`` function, therefore we needed to implement it ourselves. This function essentially iterated through all the points in the current plan, increased ``time_from_start`` and decreased ``velocities``,
``accelerations``::

   spd = 0.2

   new_traj.joint_trajectory.points[i].time_from_start = traj.joint_trajectory.points[i].time_from_start / spd


   for j in range(n_joints):
            # rospy.loginfo(type(new_traj.joint_trajectory.points[i].velocities))
            v[j] = traj.joint_trajectory.points[i].velocities[j] * spd
            a[j] = traj.joint_trajectory.points[i].accelerations[j] * spd**2
            p[j] = traj.joint_trajectory.points[i].positions[j]

 We would The adjusted trajectory plan was then passed to moveit for execution. Changing the ``spd`` parameter gave complete control over the real robot speed.

Ultimetly However, in testing we realized that the opposite approach worked better. Providing such finely discretised trajectories overly constrained the motion, and in practice this just resulted in jittery motion, controller divergence and failure.

What we found to work best was setting the ``spd`` near 1 and decreasing the resolution by a factor of 10, only solving for a way point every 1 cm::

    (plan, fraction) = group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.02,        # eef_step
                                           2.5)         # jump_threshold

::

   spd = 0.8


Running The code on the robot
---------------------------------------

Provided that you have followed the steps in the documentation till now, the learnings noted above are already reflected in the code
which you have downloaded from our repository. You are now in a position to run the robot.

.. note::

    Please refer to the GTA's for setting up and connecting to Panda. Also double check
    the real time kernal is running (it's an option that must be selected from the boot menu).

On the computer open 3 terminal windows. Launch the following ros packages in order

.. code-block:: bash

    roslaunch franka_example_controllers move_to_start.launch robot_ip:=192.168.0.88

Starts up moveit, rviz and connects to real robot.

.. code-block:: bash

    roslaunch franka_gripper franka_gripper.launch robot_ip:=192.168.0.88

Launches interface to panda gripper

.. code-block:: bash

    roslaunch arm_master light.launch

Runs the project main code


.. _franka_ros: https://github.com/de3-robo/franka_ros
.. _lib_franka: https://github.com/frankaemika/libfranka
.. _Franka documentation site: https://frankaemika.github.io/docs/installation.html
.. _franka_ros documentation: https://frankaemika.github.io/docs/franka_ros.html




