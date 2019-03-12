

# Explain step by step how the arm move to pick up a brick
# Tidy up code along the way

# Go and do doc strings

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
        ├── index.rst
        ├── support.rst
        ├── install.rst
    ├── CMakeLists.txt
    └── package.xml

Launch files `light.launch`_, `panda_one_brick.launch`_, `sim.launch`_ run all nessecary ros nodes.
Refer to *launch* for more information on running the code

Once running, the panda arm loops through a control sequence defined in `arm_master_main.py`_,
which calls services and publishes to topics defined in:

* move_arm_server.py
* brick_manager_server.py

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
            placed +=


If you wish to change how the arm moves, change the order in which the ``pick_up()``, ``place_down()``, ``place_down()``
functions are called. Additional motion functions also available in ``arm_master_main.py`` are ``go_to()`` and ``move_arm_curve()``. To illustrate, The main loop for our project implementation was implemented as follows:

.. literalinclude:: ../scripts/arm_master_main.py
  :lines: 50-69

See API calls

Testing AutoDOc
---------------------
.. automodule:: scripts.arm_master_main
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
