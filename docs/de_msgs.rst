de_msgs Package
========================

Arm_Master, Spawn_Manager and Brick_Manager are all nodes that communicate using de_msgs services. Spawn Manager was built
 by Keith Li and Justice Duruanyanwu, Arm Master was built by Zachary Gen and Brick Manager was built by Rowan Dixon and Samuel Willis


Inside the de_msgs package
--------------------------

``de_msgs`` is a package that contains 4 custom ``.srv`` files that are imported into ``arm_master_main.py``:

* QueryNextPos
* MoveArm,
* QueryBrickLoc
* QueryBricklocRequest

ROS uses a simplified service description language for describing ROS service types.
This builds on the ROS msg format to enable a request-response communication between nodes. Service descriptions are stored
in the srv directory of a package. srv files consist of a request and a response msg type separated by a three dash line.

.. note::

    Code to import message types::

        from de_msgs.srv import QueryNextPos, MoveArm, QueryBrickLoc, QueryBrickLocRequest

QueryNextPos requests the brick number as "num" in the format of float64. Float64 is a 64 bit number that can only have
four kinds of values: real numbers, positive infinity, negative infinity and NaN. The service responds with 6 float64
values as "x", "y", "z", "wx", "wy", "wz" which correspond to the pose of the brick specified by "num".

Euler angles were chosen over the quaternion to communicate the orientation of the robot between nodes as it is more
human understandable which is very important when there are multiple people working on different parts of the project.
In addition to this, Euler angles only require 3 values to completely express all rotations whereas the quaternion require 4 values
to express all rotations. This means messages using Euler angles are smaller, and save on memory space::


  float64 num
  ---
  float64 x
  float64 y
  float64 z
  float64 wx
  float64 wy
  float64 wz


MoveArm requests the pose of the end effector as 6 float64 values as "x", "y", "z", "wx", "wy", "wz". The service responds with bool success which indicates successful run of the triggered service.::

  float64 x
  float64 y
  float64 z
  float64 rot_x
  float64 rot_y
  float64 rot_z
  ---
  bool success


QueryBrickLoc requests the brick number as "num" in the format of float64. The service responds with 6 float64 values as "x", "y", "z", "wx", "wy", "wz" which correspond to the pose of the brick specified by "num".::

  float64 num
  ---
  float64 x
  float64 y
  float64 z
  float64 wx
  float64 wy
  float64 wz

ROS automatically generates the QueryBrickLocRequest and QueryBrickLocResponse from the QueryBrickLoc service.

Updating CMakeList.txt
======================

The CMakeList.txt must be updated so that the srv files can be called as services in the arm_master_main loop.
The custom service names must be added to the add_service_files function. Message_generation must also be added to the find_package function.::

	find_package(catkin REQUIRED
	COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation)


	add_service_files(
	FILES
	QueryNextPos.srv
	QueryPPBrick.srv
	QueryBrickLoc.srv
	MoveArm.srv
	)



Updating package.xml
===================

Ensure the following two lines are uncommented from the code to allow the sending of messages
Line 40::

  <build_depend>message_generation</build_depend>

Line 46::

  <exec_depend>message_runtime</exec_depend>