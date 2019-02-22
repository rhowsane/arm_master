/*
 * hello_world.cpp
 *
 *  Created on: 19 Feb 2019
 *      Author: zy2016
 */


#include <ros/ros.h>
#include <Eigen/Geometry>
//Move It Includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>



int main(int argc, char** argv)
{
	ros::init(argc,argv,"hello_move_it");
	ros::NodeHandle nodeHandle;

//	need spiiner to run
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//Set Up
	static const std::string PLANNING_GROUP = "panda_arm";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	const robot_state::JointModelGroup* joint_model_group =
	    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
	visual_tools.deleteAllMarkers();


	  // Remote control is an introspection tool that allows users to step through a high level script
	  // via buttons and keyboard shortcuts in RViz
	  visual_tools.loadRemoteControl();

	  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
	  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	  text_pose.translation().z() = 1.75;
	  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

	  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
	  visual_tools.trigger();

	  // Getting Basic Information
	  // ^^^^^^^^^^^^^^^^^^^^^^^^^
	  //
	  // We can print the name of the reference frame for this robot.
	  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

	  // We can also print the name of the end-effector link for this group.
	  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

	  // Start the demo
	  // ^^^^^^^^^^^^^^^^^^^^^^^^^
	  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	  // Planning to a Pose goal
	  // ^^^^^^^^^^^^^^^^^^^^^^^
	  // We can plan a motion for this group to a desired pose for the
	  // end-effector.
	  geometry_msgs::Pose target_pose1;
	  target_pose1.orientation.w = 1.0;
	  target_pose1.position.x = 0.28;
	  target_pose1.position.y = -0.2;
	  target_pose1.position.z = 0.5;
	  move_group.setPoseTarget(target_pose1);

	  // Now, we call the planner to compute the plan and visualize it.
	  // Note that we are just planning, not asking move_group
	  // to actually move the robot.
	  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	  visual_tools.publishAxisLabeled(target_pose1, "pose1");
	  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	  visual_tools.trigger();
	  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


	  moveit_msgs::OrientationConstraint ocm;
	  ocm.link_name = "panda_link7";
	  ocm.header.frame_id = "panda_link0";
	  ocm.orientation.w = 1.0;
	  ocm.absolute_x_axis_tolerance = 0.1;
	  ocm.absolute_y_axis_tolerance = 0.1;
	  ocm.absolute_z_axis_tolerance = 0.1;
	  ocm.weight = 1.0;

	  // Now, set it as the path constraint for the group.
	  moveit_msgs::Constraints test_constraints;
	  test_constraints.orientation_constraints.push_back(ocm);
	  move_group.setPathConstraints(test_constraints);

	  // We will reuse the old goal that we had and plan to it.
	  // Note that this will only work if the current state already
	  // satisfies the path constraints. So, we need to set the start
	  // state to a new pose.
	  robot_state::RobotState start_state(*move_group.getCurrentState());
	  geometry_msgs::Pose start_pose2;
	  start_pose2.orientation.w = 1.0;
	  start_pose2.position.x = 0.55;
	  start_pose2.position.y = -0.05;
	  start_pose2.position.z = 0.8;
	  start_state.setFromIK(joint_model_group, start_pose2);
	  move_group.setStartState(start_state);

	  // Now we will plan to the earlier pose target from the new
	  // start state that we have just created.
	  move_group.setPoseTarget(target_pose1);

	  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
	  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
	  move_group.setPlanningTime(10.0);

	  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

	  // Visualize the plan in RViz
	  visual_tools.deleteAllMarkers();
	  visual_tools.publishAxisLabeled(start_pose2, "start");
	  visual_tools.publishAxisLabeled(target_pose1, "goal");
	  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
	  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	  visual_tools.trigger();
	  visual_tools.prompt("next step");

	  // When done with the path constraint be sure to clear it.
	  move_group.clearPathConstraints();

	  // Since we set the start state we have to clear it before planning other paths
	  move_group.setStartStateToCurrentState();

	return 0;
}

