/*********************************************************************

 *********************************************************************/

/* Author:*/

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>	
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "origins");

  ros::AsyncSpinner spinner(1);
  spinner.start();


  // Setting up to start using the RobotModel class is very easy. In
  // general, you will find that most higher-level components will
  // return a shared pointer to the RobotModel. You should always use
  // that when possible. In this example, we will start with such a
  // shared pointer and discuss only the basic API. You can have a
  // look at the actual code API for these classes to get more
  // information about how to use more features provided by these
  // classes.
  //
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  //
  // .. _RobotModelLoader:
  //     http://docs.ros.org/melodic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html

  // Assumes that a robot has been loaded
  // One approach is to use a launch file

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group, e.g. the "pr_arm" of the robot
  // robot.

  // Set the robot kinematic state
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  kinematic_state->setToDefaultValues();


  //MoveIt operates on sets of joints called “planning groups” and stores them in an object called the JointModelGroup.
  //Throughout MoveIt the terms “planning group” and “joint model group” are used interchangably.
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the PR arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Forward Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // Now, we can compute forward kinematics for a set of random joint
  // values. Note that we would like to find the pose of the
  // "link4" which is the most distal link in the
  // "panda_arm" group of the robot.

  joint_values[0] = 3.14/6;
  joint_values[1] = 3.14/6;
  joint_values[2] = 0.5;

  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Set joint %s to: %f", joint_names[i].c_str(), joint_values[i]);
  }
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("sag_ince");
  /* Print end-effector transformation matrix. Remember that this is in the model frame */
  ROS_INFO_STREAM("Transformation Matrix: \n" << end_effector_state.matrix() << "\n");
  const Eigen::Vector4d origin(0,0,0,1); 
  ROS_INFO_STREAM("Transformed Coordinates: \n" << end_effector_state.matrix()*origin << "\n");

  // Inverse Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // We can now solve inverse kinematics (IK) for the PR robot.
  // To solve IK, we will need the following:
  //
  //  * The desired pose of the end-effector (by default, this is the last link in the "pr_arm" chain):
  //    end_effector_state that we computed in the step above.
  //  * The timeout: 0.1 s
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Inverse Kinematics result: Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  // Forward Kinematics 2
  std::vector<double> joint_values2;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values2);
  joint_values[0] = 3.14/6;
  joint_values[1] = 3.14/6;
  joint_values[2] = 0.5;

  kinematic_state->setJointGroupPositions(joint_model_group, joint_values2);

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Set joint %s to: %f", joint_names[i].c_str(), joint_values2[i]);
  }

  const Eigen::Isometry3d& end_effector_state2 = kinematic_state->getGlobalLinkTransform("sag_ince");

  /* Print end-effector transformation matrix. Remember that this is in the model frame */
  ROS_INFO_STREAM("Transformation Matrix: \n" << end_effector_state2.matrix() << "\n");
  ROS_INFO_STREAM("Transformed Coordinates: \n" << end_effector_state2.matrix()*origin << "\n");

  // Inverse Kinematics 2
  found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state2, timeout);
  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values2);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Inverse Kinematics result: Joint %s: %f", joint_names[i].c_str(), joint_values2[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  static const std::string PLANNING_GROUP = "pr_move";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);


  
  ros::shutdown();
  return 0;
}
