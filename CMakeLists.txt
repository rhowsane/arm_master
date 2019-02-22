cmake_minimum_required(VERSION 2.8.3)

#project name, must be same as package.xml
project(arm_master)

#Todo define to compile with c++ configs
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

#List package dependices, must also be listed in package.xml
find_package(catkin REQUIRED
COMPONENTS
roscpp
std_msgs
moveit_ros_planning_interface
moveit_visual_tools
moveit_core
moveit_ros_planning
message_generation
de_msgs
    )

#EXPORT build information, defining env variables...
#DEPENDS are passed down through pkgs, ie eigen boost ect..)
add_service_files(
  FILES
  MoveArm.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arm_master
   CATKIN_DEPENDS roscpp std_msgs moveit_visual_tools moveit_ros_planning_interface moveit_core moveit_ros_planning de_msgs

   DEPENDS EIGEN3
)

include_directories(include	${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

#Define exectuable to build, todo... multiple cmack
add_executable(${PROJECT_NAME}_talker_node src/talker.cpp)
target_link_libraries(${PROJECT_NAME}_talker_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_hello_move_it src/hello_world.cpp)
target_link_libraries(${PROJECT_NAME}_hello_move_it ${catkin_LIBRARIES})

add_executable(arm_control_fine_server src/arm_control_fine_server.cpp)
add_dependencies(arm_control_fine_server ${PROJECT_NAME}_generate_messages_cpp )
target_link_libraries(arm_control_fine_server ${catkin_LIBRARIES})
