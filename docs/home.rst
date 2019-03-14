
DE3 Robotics Project
========================

.. figure::  imgs/arm_with_brick.jpg
   :align:   center

This Robotics project took place at the Dyson School of Engineering at the start of 2019.

ROS was used to program an industrial Franka Panda Robot to perform pick and place task of foam bricks.

All code for project is available in the `team git repo`_

Development focused on 3 main packages and 1 custom message package:

* `arm_master`_
* `brick_manager`_
* `spawn_manager`_
* `de_msgs`_

Setup documentation and overall comments have been grouped here.


Videos
--------

`Project overview`_

The aim of this project was to build a foam brick wall that was both tall and aesthetic. We accomplished this by
by programming a Franka Panda robot to pick up and place bricks into the desired locations. Further we were highly interested in the
possibility of commanding Panda and designing the wall using augment reality technology. For this a mirror environment, with an identical workspace to
the real setup, was created in Unity 3D. Using this software were able manipulate virtual bricks. The program then save the position
of the bricks in the resulting design into a text file for processing on the robot.

While the entire workflow was never executed, what we demonstrate in this video is a "Wizard of Oz" concept. Certainly in the
future, advanced robot programming techniques such as using AR will become more accessible.


`Robot building wall in simulation`_

`Robot building wall in real-life`_

`Setting up and running code`_

Presentation
-------------

`Robot demo presentation slides`_



.. _team git repo: https://github.com/de3-robo
.. _arm_master: https://github.com/de3-robo/arm_master
.. _brick_manager: https://github.com/de3-robo/brick_manager
.. _spawn_manager: https://github.com/de3-robo/spawn_mnger
.. _de_msgs: https://github.com/de3-robo/de_msgs

.. _Project overview: https://drive.google.com/file/d/1upAYPv9WAtRqW-wK1cnZig8cDrmZvin2/view?fbclid=IwAR2OWkxUuuH4r3dMeiGpXatqs_VzAjbUqYJ-8Y4pmy0s-TFVt2B1EIfaAgg
.. _Robot building wall in simulation: https://drive.google.com/open?id=1E517xbq1VebSo29f8bf77n7HyZpDgVL6
.. _Robot building wall in real-life: https://drive.google.com/open?id=1hC8p6dtVZZ6CVmGSD1spYmxh6AhzPmxO
.. _Setting up and running code: https://drive.google.com/open?id=164bEFaRacpHIMV_tlWwU3duqRCKpdJvD
.. _Robot demo presentation slides: https://www.dropbox.com/s/uztyttssk7mjkrf/robotics%20presentation.pdf?dl=0