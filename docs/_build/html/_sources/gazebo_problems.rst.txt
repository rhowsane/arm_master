Running on Gazebo
===============================

Before testing our code on the real Panda robot, we must be able to run it within a digital simulation first, proving
that it works. Otherwise, having a physically moving robot with unknown risks / movements that have not been tested
would be a major risk. Here were the main problems that were encountered by the team when attempting to perform simulations in Gazebo.

Arm experiencing vibration / 'jittering'
----------------------------------------
This was mostly likely due to the fact that the PID settings for the robot's joints were not well tuned. Therefore,
this was investigated alongside the Gazebo simulations.

The 'default' PID values for the robot arm can be configured in ``franka_gazebo/config/default.yaml`` within the
``franka_gazebo`` ROS package.

During the simulation, a convenient graphical interface with sliders can be utilised to finetune PID values, by
running the following in another terminal:
``rosrun rqt_reconfigure rqt_reconfigure``

Once the ``rqt_reconfigure`` window is open, the slider interfaces can be found within ``gazebo_ros_control/pid_gains``
in the menu on the left hand side.

.. figure:: _static/gazebo_pid_interface.png
    :align: center
    :figclass: align-center

Brick model friction issues
--------------------------------------
This was a major issue in simulations, as otherwise the Panda's gripper would fail to pick up a brick and carry it in
its grip effectively.

The solution was to edit the brick object file and change its parameters regarding surface friction.

The file could be located in the following filepath: ``~/.gazebo/models/Brick/model-1_4.sdf``

With the model file ``model-1_4.sdf``, the following were changed::

        <surface>
          <friction>
            <ode>
              <mu>100</mu>
              <mu2>50</mu2>
              <fdir1>0 0 1</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>100000.000000</kp>
              <kd>10.00000</kd>
              <max_vel>2.000000</max_vel>
              <min_depth>0.0001</min_depth>
            </ode>
          </contact>
        </surface>

These improved parameters allowed the brick to be grasped more effectively, and were less likely to slip from Panda's
gripper.

