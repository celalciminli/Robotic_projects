// ROS
#include <ros/ros.h>

#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

using namespace std;



int main(int argc, char** argv)
{
// initialiizng ros node
  ros::init(argc, argv, "function_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

//initializing moveit interfaces, planning_scene_interface and move_group_interface
  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("body");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group.setPlanningTime(60.0);


  static const std::string PLANNING_GROUP = "body";
 moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
 const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());


  // Set the robot kinematic state
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();

  // Get joint names
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  const robot_model::JointModel* joint_model1 = kinematic_model->getJointModel("base_to_link_1");
  const robot_model::JointModel* joint_model2 = kinematic_model->getJointModel("link_1_to_link_2");
  const robot_model::JointModel* joint_model3 = kinematic_model->getJointModel("link2_to_link3");

  const robot_model::RevoluteJointModel* R1_joint_model = dynamic_cast<const robot_model::RevoluteJointModel*>(joint_model1);
  const robot_model::RevoluteJointModel* R2_joint_model = dynamic_cast<const robot_model::RevoluteJointModel*>(joint_model2);
  const robot_model::PrismaticJointModel* P_joint_model = dynamic_cast<const robot_model::PrismaticJointModel*>(joint_model3);
  std::vector<double> joint_group_positions; 
  kinematic_state->setToRandomPositions(joint_model_group);   // ignore this line it is just for initialization
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


  int N;		// number of objects to be transported
  nh.getParam("/N", N);	// getting the number of objects from the rosparam server
  double object[N][4];	// array where holds the information about object locations and dimensions
  double basket[4]; // array where holds the information about basket locations and dimensions

  std::vector<moveit_msgs::CollisionObject> collision_objects;	// creating a collision object object
  collision_objects.resize(N+1);

	for(int i=0; i<N; i++)	// loop where we get information about objects from rosparam server
	{
		string param;
		string num,x,y;
		num = to_string(i+1);

		// getting information from rosparam server
		param = "/p" + num + "x";
		nh.getParam(param, object[i][0]);
		param = "/p" + num + "y";
		nh.getParam(param, object[i][1]);
		param = "/p" + num;
		nh.getParam(param, object[i][2]);
		param = "/h" + num;
		nh.getParam(param, object[i][3]);

		//x = to_string(object[i][0]);
		//y = to_string(object[i][1]);
		//param = "rosrun gazebo_ros spawn_model -file ~/final_project_ws/URDF_Files/cylinder.urdf -urdf -x "
		// + x + " -y " + y + " -z 0 -model cylinder" + num;

		//const char *cstr = param.c_str();
		//system(cstr);

		/* spawning objects in rviz as collision objects */
		collision_objects[i].id = "cylinder" + num;
		collision_objects[i].header.frame_id = "base_link";

		/* Define the primitive and its dimensions. */
		collision_objects[i].primitives.resize(1);
		collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].CYLINDER;
		collision_objects[i].primitives[0].dimensions.resize(3);
		collision_objects[i].primitives[0].dimensions[0] = object[i][3];
		collision_objects[i].primitives[0].dimensions[1] = object[i][2];

		/* Define the pose of the cylinder. */
		collision_objects[i].primitive_poses.resize(1);
		collision_objects[i].primitive_poses[0].position.x = object[i][0];
		collision_objects[i].primitive_poses[0].position.y = object[i][1];
		collision_objects[i].primitive_poses[0].position.z = object[i][3] / 2;
    ROS_INFO_STREAM("object coordinates:" << collision_objects[i]);
		collision_objects[i].operation = collision_objects[i].ADD;
	}
    string param;
    param = "/gx";
		nh.getParam(param, basket[0]);
		param = "/gy";
		nh.getParam(param, basket[1]);
		param = "/gz";
		nh.getParam(param, basket[2]);
		param = "/hb";
		nh.getParam(param, basket[3]);

    /* spawning objects in rviz as collision objects */
		collision_objects[N].id = "BASKET";
		collision_objects[N].header.frame_id = "base_link";

		/* Define the primitive and its dimensions. */
		collision_objects[N].primitives.resize(1);
		collision_objects[N].primitives[0].type = collision_objects[N].primitives[0].CYLINDER;
		collision_objects[N].primitives[0].dimensions.resize(3);
		collision_objects[N].primitives[0].dimensions[0] = basket[3];
		collision_objects[N].primitives[0].dimensions[1] = basket[2];

		/* Define the pose of the cylinder. */
		collision_objects[N].primitive_poses.resize(1);
		collision_objects[N].primitive_poses[0].position.x = -basket[0];
		collision_objects[N].primitive_poses[0].position.y = -basket[1];
		collision_objects[N].primitive_poses[0].position.z = -basket[3] / 2;
    ROS_INFO_STREAM("basket coordinates:" << collision_objects[N]);
		collision_objects[N].operation = collision_objects[N].ADD;



  planning_scene_interface.applyCollisionObjects(collision_objects);	// apply collision objects to the scene

//---------------------------------------------------------------------


  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  geometry_msgs::Pose target_pose1;	// create an object for position targets
  group.setGoalTolerance(0.02);

double l1,l2,q1,q2,d3,s,r;
l1=1.5;
l2=0.9;

double q22,q12,d32,s2,r2;
double h= 0;

for(int i=0;i<N;i++){
s=object[i][3] - l1;
r=sqrt(object[i][0]*object[i][0] + object[i][1]*object[i][1]);
q1=atan(object[i][1]/object[i][0]);
d3=sqrt(s*s+r*r)-l2;
q2=atan(s/r);

ROS_INFO("Joints %d : %f, %f, %f",i,q1,q2,d3);
joint_group_positions[0] = q1;
joint_group_positions[1] = q2;
joint_group_positions[2] = d3;
/*for(int i=0;i<3;i++){
  ROS_INFO("Joint %d: %f",i, joint_group_positions[i]);
}*/
//kinematic_state->setJointGroupPositions(joint_model_group, joint_group_positions);

//move_group.setJointValueTarget(joint_group_positions);
//move_group.move();

group.setJointValueTarget(joint_group_positions);
group.move();

group.attachObject(collision_objects[i].id);

r2= sqrt(basket[0]*basket[0] + basket[1]*basket[1]);
s2= h - l1;
q12=atan(basket[1]/basket[0]);
d32=sqrt(s2*s2+r2*r2)-l2;
q22=atan(s2/r2);

ROS_INFO("Joints to_basket: %f, %f, %f",q12,q22,d32);
joint_group_positions[0] = q12;
joint_group_positions[1] = q22;
joint_group_positions[2] = d32;
group.setJointValueTarget(joint_group_positions);
group.move();
group.detachObject(collision_objects[i].id);
h += object[i][3];
}


joint_group_positions[0] = 0;
joint_group_positions[1] = 0;
joint_group_positions[2] = 0;
group.setJointValueTarget(joint_group_positions);
group.move();

/*for(int i=0;i<N;i++){
  // go to the object position
  target_pose1.position.x = object[i][0];
  target_pose1.position.y = object[i][1];
  target_pose1.position.z = object[i][2];
  group.setJointValueTarget(target_pose1);
    ROS_INFO_STREAM("target: " << target_pose1);
  group.setJointValueTarget(target_pose1);
// attach to the object
  group.attachObject(collision_objects[i].id);
// go to the goal position
  target_pose1.position.x = basket[0];
  target_pose1.position.y = basket[1];
  target_pose1.position.z = object[i][2];
  group.setJointValueTarget(target_pose1);
  group.move();
// detach the object
  group.detachObject(collision_objects[i].id);
  ROS_INFO("Strawberry to basket");

}
/*
// go to the some position
  target_pose1.position.x = -0.5;
  target_pose1.position.y = -0.5;
  group.setJointValueTarget(target_pose1);  
  group.move();
  */
  ROS_INFO("finish");

  ros::waitForShutdown();
  return 0;
}