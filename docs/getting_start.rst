Getting Started
========================

At a high level, to install and run this projects code on your computer involves:

1. Pulling the 3 main packages (*arm_master*,*brick_manager*,*spawn_manager*) from github and building them
2. Pulling relevant packages for interfacing with Panda robot (*franka_ros*, *franka_gazebo*)
3. Downloading relevant support files (*brick.stl*)

We will now walk step by step to go from an empty workspace to a robot that can build a wall in Gazebo

Set Up
-------------------------

Before installing the projects packages we need to make sure everyone has set up their enviroment correctly and are using the correct tools.

.. note::

    The project was developed using **ROS Kinetic running on a native Ubuntu 16.04**. It was also tested on **Ubuntu 18.04 VM's
    running ROS Melodic**. Given ROS's backwards compatability it is likely that it should work on your setup as well, but having
    a similar setup is likely to minimize possible problems.


First you will need to install catkin tools. Catkin tools is the perfed build system used by the ASL lab at ETHz.
It provides additonal functionaly (``catkin clean``) and greater control over build configs.

Please refer to `documentation`_ for details on installation and usage

.. warning::

    the deafult ``$ catkin_make`` and catkin tools build systems are not comptabile. Its is not recommended to mix the two.
    Please make sure you only use the ``catkin build`` command to build your src.

.. note::

    If for some reason you end up mixing the two or the project generally begins to break down, you can manually reset it by
    deleteing the ``build``, ``devel`` and ``logs`` folders. Running ``catkin clean`` from command line provides similar functionality.

We will also add some code to your ``.bashrc``, to automate some of the background installation work. From your home directory, open
bashrc with your favorite text editor

.. code-block:: bash

    gedit ~/.bashrc


Copy paste the following two lines at the bottom.

.. code-block:: bash

    source /opt/ros/kinetic/setup.bash
    source ~/de3_ws/devel/setup.bash

Then close and save.

Assuming you now have ``catkin tools`` installed and your bashrc edited, we can begin the setup process. First create
we create a workspace and intialize it. From within your desired directory:

.. code-block:: bash

    mkdir de3_workspace
    cd de3_workspace
    mkdir src
    catkin init

Build the workspace


.. code-block:: bash

    catkin build

You should now see in addition to your ``src`` folder, ``build``, ``devel`` and ``logs`` folders.


.. warning::

    Remember to ``source`` your workspace after building to let ROS know you have made changes. From your workspace directory

    .. code-block:: bash

        source devel/setup.bash

    We list it here once and assume you will do it after each ``catkin build`` in this tutorial. Note that generally during
    development you will not need to resources your workspace, especially when writing python scripts. It is however required
    when creating new packages or messages.

    As a rule of thumb, if ros commands like ``roscd`` aren't working, and your getting ``module not found`` errors when you try to import custom messages: also
    re-source your workspace.

.. note::
    In the setup we added the following code to your ``.bashrc``

    .. code-block:: bash

        source /opt/ros/kinetic/setup.bash
        source ~/de3_ws/devel/setup.bash

    The ``.bashrc`` is run everytime you start a new terminal session. That means instead of manually resourcing your workspace, you can simply close and open a new terminal,
    and ROS will update it self. If you decide to continue working in the same terminal session, you will need to run the aformentioned code, however.


.. warning::



Congrats you have created and built your ROS workspace. In the ``src`` folder is where you will install all relevant ROS packages.

.. _documentation: https://catkin-tools.readthedocs.io/en/latest/installing.html

Installing Project Packages
---------------------------

First we install the custom packages written for this project. Find links to the three packages and download using git

* `arm_master`_
* `brick_manager`_
* `spawn_manager`_
* `de_msgs`_

.. _arm_master: https://github.com/de3-robo/arm_master
.. _brick_manager: https://github.com/de3-robo/brick_manager
.. _spawn_manager: https://github.com/de3-robo/spawn_mnger
.. _de_msgs: https://github.com/de3-robo/de_msgs

Example for arm master. First move into src directory

.. code-block:: bash

    cd de3_workspace/src

Then clone

.. code-block:: bash

     git clone https://github.com/de3-robo/arm_master.git

.. note::

    Use *ctrl-shift-v* to paste the github link into terminalhttps://github.com/de3-robo/de_msgs.git

At this point you should now have all 3 packages installed in your ``workspace/src`` folder. Lets build what we have
to make sure its working

.. code-block:: bash

     catkin build

If it worked, Great job!


Installing 3rd Party Packages
-----------------------------

Now we will install the 3rd party packages required for path planning and interfacing with Panda. These may or may not already be installed in your computer.

.. note::

    Here we will only install nessecary packages required to run Panda in Gazebo. refer to the **Getting Real Panda Working** section to see setup for
    running on real Panda


First we download moveit. Please refer to `moveit documentation`_ for installation guide. We can also highly recommend walking
through the `moveit python tutorial`_.

You will now need to install

* `franka_gazebo`_
* `panda_moveit_config`_

as before use the git command line calls and then build the package

.. code-block:: bash

     git clone https://github.com/de3-robo/franka_gazebo.git
     ...

     catkin build

Great job! your almost there.

Installing additional files
-----------------------------

First we will install the brick which is used to place in gazebo.

.. note::

    Note the brick provided by on the module box folder needs to have its friction and mass parameters tunned to actually work in simulation.
    The brick we provided is not perfect, but should be sufficient as a starting point.

Navigate to your ``models`` folder in your ``.gazebo`` installation. On my computer ``.gazebo`` is accesible from the home directory.


.. code-block:: bash

    cd .gazebo/models/

Then clone the brick into the `brick model directory`_

.. code-block:: bash

     git clone https://github.com/de3-robo/Brick.git

Finally we need to install a new gazebo world and launch file with tables like those found on level 3 in the Dyson school setup

Download the code from the following `table repo`_ into your computer:

Then move file ``wall_table_world.launch`` into ``/opt/ros/kinetic/share/gazebo_ros/launch`` directory. Make sure to enter in your
correct ROS version.

Then move file ``wall_table.world`` into ``/usr/share/gazebo-9/worlds`` directory.

If you made it this far your doing great! Lets run the code now

Running the Code
-------------------

All the code can be run by 3 launch files located in the ``arm_master`` package. Make sure to run the code in order listed.
In 3 seperate terminals copy paste the following commands:

.. code-block:: bash

     roslaunch arm_master panda_one_brick.launch

.. code-block:: bash

     roslaunch arm_master sim.launch

.. code-block:: bash

     roslaunch arm_master light.launch


.. _franka_gazebo: https://github.com/de3-robo/franka_gazebo
.. _panda_moveit_config: https://github.com/de3-robo/panda_moveit_config
.. _brick model directory: https://github.com/de3-robo/Brick
.. _table repo: https://github.com/de3-robo/walltableworldlaunch


.. _moveit documentation: https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html
.. _moveit python tutorial: https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html



Addtional Resources
-----------------------------

Great intro to ROS by ETHz Automous Systems Lab:
API reference for python movegroup interface, (has all the useful command you need for using moveit with python):
