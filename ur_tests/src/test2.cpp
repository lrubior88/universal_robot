#include <ros/ros.h>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <math.h>       /* atan2 */

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

//Constants
static const double PI = 3.14159265;
static const std::string COLLISION_TOPIC = "/collision_object";

ros::Publisher pub_collision_obj_; // for MoveIt collision objects
boost::scoped_ptr<move_group_interface::MoveGroup> group_;

int main(int argc, char* argv[]){
	
	    //Startup ros
    ros::init(argc, argv, "test2");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Get the node handle for this node
    ros::NodeHandle node("~");
    
  
	group_.reset(new move_group_interface::MoveGroup("manipulator"));
	group_->setPlanningTime(45.0);
	
	
	moveit_msgs::CollisionObject collision_obj;

	//Configure publisher
	pub_collision_obj_ = node.advertise<moveit_msgs::CollisionObject>(COLLISION_TOPIC, 0);
	sleep(2.0);		
		/* The id of the object is used to identify it. */
	collision_obj.id = "table";

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose table_pose;
	table_pose.orientation.w = 1;
	table_pose.position.x = 0.0;
	table_pose.position.y = 0.0;
	table_pose.position.z = -0.2;

	/* Define a box to add to the world. */
	collision_obj.header.stamp = ros::Time::now();
	collision_obj.header.frame_id = "world";
	collision_obj.operation = moveit_msgs::CollisionObject::ADD;
	collision_obj.primitives.resize(1);
	collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	collision_obj.primitives[0].dimensions.resize(3);
	collision_obj.primitives[0].dimensions[0] = 2;
	collision_obj.primitives[0].dimensions[1] = 2;
	collision_obj.primitives[0].dimensions[2] = 0.05;
	collision_obj.primitive_poses.resize(1);
	collision_obj.primitive_poses[0] = table_pose;


	pub_collision_obj_.publish(collision_obj);
	ros::spinOnce();
	
		
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, 0.0);
		
	ROS_INFO("Published collision object table");
	geometry_msgs::Pose target_pose1;
	target_pose1.position.x = 0.5;
	target_pose1.position.y = 0.5;
	target_pose1.position.z = 0.5;
	target_pose1.orientation.x = q.x();
	target_pose1.orientation.y = q.y();
	target_pose1.orientation.z = q.z();
	target_pose1.orientation.w = q.w();

	group_->setPoseTarget(target_pose1);
	
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group_->plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group_->move();
		sleep(5.0);
	}
	
	
	
	std::vector<geometry_msgs::Pose> waypoints;

	waypoints.push_back(target_pose1);
	
	
	geometry_msgs::Pose target_pose3 = target_pose1;
	target_pose3.position.x -= 0.2;
	waypoints.push_back(target_pose3);  // up and out

	target_pose3.position.y -= 0.2;
	waypoints.push_back(target_pose3);  // left

	target_pose3.position.y += 0.2;
	target_pose3.position.x += 0.2;
	waypoints.push_back(target_pose3);  // down and right (back to start)
	
	moveit_msgs::RobotTrajectory trajectory;
	double fraction = group_->computeCartesianPath(waypoints,
                                             0.01,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);

	ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
      fraction * 100.0);
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(15.0);
	
	moveit::planning_interface::MoveGroup::Plan plan;
	plan.trajectory_ = trajectory;
	group_->execute(plan);
	
	

  //~ moveit_msgs::RobotTrajectory trajectory_msg;
  //~ group.setPlanningTime(10.0);
 //~ 
  //~ double fraction = group.computeCartesianPath(waypoints,
                                               //~ 0.01,  // eef_step
                                               //~ 0.0,   // jump_threshold
                                               //~ trajectory_msg, false);
  //~ // The trajectory needs to be modified so it will include velocities as well.
  //~ // First to create a RobotTrajectory object
  //~ robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm_group");
//~ 
  //~ // Second get a RobotTrajectory from trajectory
  //~ rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);
 //~ 
  //~ // Thrid create a IterativeParabolicTimeParameterization object
  //~ trajectory_processing::IterativeParabolicTimeParameterization iptp;
//~ 
  //~ // Fourth compute computeTimeStamps
  //~ success = iptp.computeTimeStamps(rt);
  //~ ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
//~ 
  //~ // Get RobotTrajectory_msg from RobotTrajectory
  //~ rt.getRobotTrajectoryMsg(trajectory_msg);
//~ 
  //~ // Finally plan and execute the trajectory
  //~ plan.trajectory_ = trajectory_msg;
  //~ ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
  //~ sleep(5.0);
  //~ group.execute(plan);
    
    return 0;
}
