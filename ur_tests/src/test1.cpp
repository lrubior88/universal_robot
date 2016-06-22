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

#include "std_srvs/Trigger.h"

//Constants
static const double PI = 3.14159265;
static const std::string COLLISION_TOPIC = "/collision_object";


ros::Publisher pub_collision_obj_; // for MoveIt collision objects
boost::scoped_ptr<move_group_interface::MoveGroup> group_;
ros::ServiceClient client;

// Function names
void move_back_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end);
void move_forward_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end);
void move_right_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end);
void move_left_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end);
void turn_right_trajectory(geometry_msgs::Point axes_pos, double normal_angle, double radius);
int turn_right_trajectory2(geometry_msgs::Point axes_pos, double normal_angle, double radius);
bool saveScan();
bool go2Pose(geometry_msgs::Point position, tf::Quaternion q, int attempts);
void scanning_z(geometry_msgs::Point position, tf::Quaternion q, double z_init, double z_end, double interval);

/// -   -   -   -   - MAIN -    -   -   -   - ///
int main(int argc, char* argv[]){
	
	    //Startup ros
    ros::init(argc, argv, "test1");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Get the node handle for this node
    ros::NodeHandle node("~");
    
  
	group_.reset(new move_group_interface::MoveGroup("manipulator"));
	group_->setPlanningTime(10.0);
	//~ group_->setPlannerId("SBLkConfigDefault");
    //~ moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    client = node.serviceClient<std_srvs::Trigger>("/laser2pcl_srv");
    
	ROS_INFO("Reference frame: %s", group_->getPlanningFrame().c_str());

	ROS_INFO("Reference frame: %s", group_->getEndEffectorLink().c_str());
	
	
	double m = 0.0;
	double n = 0.8;
	double x_plane = 0.0;
	double y_plane = 0.8;
	
	double x_handle;
	double y_handle;
	

	
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	double m_normal = -1/m;
	
	int cuad = 0;
	if (m > 0.0){
		if (n > 0) cuad = 2;
		else cuad = 4;
	}
	else if(m < 0.0){
		if (n > 0) cuad = 1;
		else cuad = 3;
	}
	else{
		if (n > 0) cuad = 0;
		else cuad = -1;
	}
	
	
	if (cuad == 1)			yaw = atan2(m_normal,1);
	else if (cuad == 2)		yaw = atan2(-m_normal,-1);
	else if (cuad == 3)		yaw = atan2(-m_normal,-1);
	else if (cuad == 4)		yaw = atan2(m_normal,1);
	else if (cuad == 0)		yaw = PI/2;
	else if (cuad == 1)		yaw = 3*PI/2;
	
	double yaw_degree = yaw* 180 / PI;
	ROS_INFO("cuad= %d | yaw = %f",cuad, yaw_degree);
	
	move_left_plane(x_plane, y_plane, yaw, 0.2, x_handle, y_handle);
	move_back_plane(x_handle, y_handle, yaw, 0.035, x_handle, y_handle);
    
    geometry_msgs::Point axes_pos;
    axes_pos.x = x_handle;
    axes_pos.y = y_handle;
    axes_pos.z = 0.3;		
	
		
	tf::Quaternion q;
	q.setRPY(roll, pitch, yaw);

	double d = 0.4;
	double x_target;
	double y_target;
	
	
	
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
	table_pose.position.z = -0.1;

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
	ROS_INFO("Published collision object table");


	/* The id of the object is used to identify it. */
	collision_obj.id = "box1";

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose;
	box_pose.orientation.x = q.x();
	box_pose.orientation.y = q.y();
	box_pose.orientation.z = q.z();
	box_pose.orientation.w = q.w();
	box_pose.position.x = x_plane;
	box_pose.position.y = y_plane;
	box_pose.position.z = 0.5;

	/* Define a box to add to the world. */
	collision_obj.header.stamp = ros::Time::now();
	collision_obj.header.frame_id = "world";
	collision_obj.operation = moveit_msgs::CollisionObject::ADD;
	collision_obj.primitives.resize(1);
	collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	collision_obj.primitives[0].dimensions.resize(3);
	collision_obj.primitives[0].dimensions[0] = 0.02;
	collision_obj.primitives[0].dimensions[1] = 1.5;
	collision_obj.primitives[0].dimensions[2] = 1.0;
	collision_obj.primitive_poses.resize(1);
	collision_obj.primitive_poses[0] = box_pose;


	pub_collision_obj_.publish(collision_obj);
	ros::spinOnce();
	ROS_INFO("Published collision object box1");


	/* Sleep so we have time to see the object in RViz */
	sleep(5.0);


	//~ /* The id of the object is used to identify it. */
	//~ collision_obj.id = "box2";
//~ 
	//~ /* A pose for the box (specified relative to frame_id) */
	//~ box_pose.orientation.x = q.x();
	//~ box_pose.orientation.y = q.y();
	//~ box_pose.orientation.z = q.z();
	//~ box_pose.orientation.w = q.w();
	//~ box_pose.position.x = axes_pos.x;
	//~ box_pose.position.y = axes_pos.y;
	//~ box_pose.position.z = 0.3;
//~ 
	//~ /* Define a box to add to the world. */
	//~ collision_obj.header.stamp = ros::Time::now();
	//~ collision_obj.header.frame_id = "world";
	//~ collision_obj.operation = moveit_msgs::CollisionObject::ADD;
	//~ collision_obj.primitives.resize(1);
	//~ collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	//~ collision_obj.primitives[0].dimensions.resize(3);
	//~ collision_obj.primitives[0].dimensions[0] = 0.05;
	//~ collision_obj.primitives[0].dimensions[1] = 0.01;
	//~ collision_obj.primitives[0].dimensions[2] = 0.01;
	//~ collision_obj.primitive_poses.resize(1);
	//~ collision_obj.primitive_poses[0] = box_pose;
    //~ 
	//~ pub_collision_obj_.publish(collision_obj);
	//~ ros::spinOnce();
	//~ ROS_INFO("Published collision object box2");
    //~ sleep(5.0);	
    
	geometry_msgs::Point target_pose1;
	 
    move_back_plane(x_plane, y_plane, yaw, d, x_target, y_target);
	ROS_INFO("x:%f,y:%f",x_target,y_target);

	target_pose1.x = x_target;
	target_pose1.y = y_target;
	target_pose1.z = 0.34;
    scanning_z(target_pose1, q, 0.34, 0.4, 0.005);

    move_back_plane(x_plane, y_plane, yaw, d, x_target, y_target);
    move_left_plane(x_target, y_target, yaw, 0.2, x_target, y_target);
	ROS_INFO("x:%f,y:%f",x_target,y_target);

	target_pose1.x = x_target;
	target_pose1.y = y_target;
	target_pose1.z = 0.34;
    scanning_z(target_pose1, q, 0.34, 0.4, 0.005);
	
    move_back_plane(x_plane, y_plane, yaw, d, x_target, y_target);
    move_left_plane(x_target, y_target, yaw, 0.4, x_target, y_target);
	ROS_INFO("x:%f,y:%f",x_target,y_target);

	target_pose1.x = x_target;
	target_pose1.y = y_target;
	target_pose1.z = 0.34;
    scanning_z(target_pose1, q, 0.34, 0.4, 0.005);
	
	

	//~ move_back_plane(axes_pos.x, axes_pos.y, yaw, 0.1, axes_pos.x, axes_pos.y); 
    
    //~ double radius = 0.1;
	//~ q.setRPY(PI, 0.0, yaw);
	//~ double s;
	//~ geometry_msgs::Pose tool_pose;
	//~ tool_pose.position.x = axes_pos.x;
	//~ tool_pose.position.y = axes_pos.y;
	//~ tool_pose.position.z = axes_pos.z +radius+0.05;
	//~ tool_pose.orientation.x = q.x();
	//~ tool_pose.orientation.y = q.y();
	//~ tool_pose.orientation.z = q.z();
	//~ tool_pose.orientation.w = q.w();
	//~ 
	//~ for(int i=0;i<20;i++){
	//~ 
		//~ group_->setPoseTarget(tool_pose);
		//~ success = group_->plan(my_plan);
//~ 
		//~ ROS_INFO("Visualizing plan x (pose goal) %s",success?"":"FAILED");
		//~ ROS_INFO("Try:%d",i);
		//~ /* Sleep to give Rviz time to visualize the plan. */
	//~ 
		//~ if (success){
			//~ group_->move();
			//~ sleep(5.0);
			//~ ROS_INFO("Start position");
			//~ s = turn_right_trajectory2(axes_pos, yaw, radius);
			//~ ROS_INFO("Finish trajectory");
			//~ if(!s) break;
			//~ sleep(5.0);
		//~ }
	//~ }
    

    /*Remove object*/
    collision_obj.id = "box1";
	collision_obj.header.stamp = ros::Time::now();
	collision_obj.operation = moveit_msgs::CollisionObject::REMOVE;
	pub_collision_obj_.publish(collision_obj);
	ros::spinOnce();
	ROS_INFO("Removed object box1");
	
	collision_obj.id = "box2";
	collision_obj.header.stamp = ros::Time::now();
	collision_obj.operation = moveit_msgs::CollisionObject::REMOVE;
	pub_collision_obj_.publish(collision_obj);
	ros::spinOnce();
	ROS_INFO("Removed object box2");
    
    
    
    return 0;
}

void 
move_back_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end)
{
	x_end = x_ini - d*cos(normal_angle);
	y_end = y_ini - d*sin(normal_angle);
	
	return;
}
void 
move_forward_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end)
{
	x_end = x_ini + d*cos(normal_angle);
	y_end = y_ini + d*sin(normal_angle);
	
	return;
}
void 
move_right_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end)
{
	x_end = x_ini + d*sin(normal_angle);
	y_end = y_ini - d*cos(normal_angle);
	
	return;
}
void 
move_left_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end)
{
	x_end = x_ini - d*sin(normal_angle);
	y_end = y_ini + d*cos(normal_angle);
	
	return;
}

void 
turn_right_trajectory(geometry_msgs::Point axes_pos, double normal_angle, double radius)
{

	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose tool_pose;
	tf::Quaternion q;
	bool success;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	
	q.setRPY(-PI, 0.0, normal_angle);		
		
	tool_pose.position.x = axes_pos.x;
	tool_pose.position.y = axes_pos.y;
	tool_pose.position.z = axes_pos.z + radius + 0.05;
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	
	group_->setPoseTarget(tool_pose);
	success = group_->plan(my_plan);

	ROS_INFO("Visualizing plan t (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group_->move();
	}
	
	tool_pose.position.z = axes_pos.z + radius;
	
	group_->setPoseTarget(tool_pose);
	success = group_->plan(my_plan);

	ROS_INFO("Visualizing plan t (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group_->move();
	}
	
	double x_aux;
	double y_aux;
	double z_aux;
	q.setRPY(-5*PI/6.0, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(PI/3.0), x_aux, y_aux);
	tool_pose.position.x = x_aux;
	tool_pose.position.y = y_aux;
	tool_pose.position.z = axes_pos.z + (radius*sin(PI/3.0));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	
	group_->setPoseTarget(tool_pose);
	success = group_->plan(my_plan);

	ROS_INFO("Visualizing plan t (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group_->move();
	}
	
	q.setRPY(-2*PI/3.0, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(PI/6.0), x_aux, y_aux);
	tool_pose.position.x = x_aux;
	tool_pose.position.y = y_aux;
	tool_pose.position.z = axes_pos.z + (radius*sin(PI/6.0));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	
	group_->setPoseTarget(tool_pose);
	success = group_->plan(my_plan);

	ROS_INFO("Visualizing plan t (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group_->move();
	}
	
	q.setRPY(-PI/2.0, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius, x_aux, y_aux);
	tool_pose.position.x = x_aux;
	tool_pose.position.y = y_aux;
	tool_pose.position.z = axes_pos.z;
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	
	group_->setPoseTarget(tool_pose);
	success = group_->plan(my_plan);

	ROS_INFO("Visualizing plan t (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group_->move();
	}
	
	q.setRPY(-PI/3.0, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(-PI/6.0), x_aux, y_aux);
	tool_pose.position.x = x_aux;
	tool_pose.position.y = y_aux;
	tool_pose.position.z = axes_pos.z + (radius*sin(-PI/6.0));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	
	group_->setPoseTarget(tool_pose);
	success = group_->plan(my_plan);

	ROS_INFO("Visualizing plan t (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group_->move();
	}
	
	q.setRPY(-PI/6, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(-PI/3.0), x_aux, y_aux);
	tool_pose.position.x = x_aux;
	tool_pose.position.y = y_aux;
	tool_pose.position.z = axes_pos.z + (radius*sin(-PI/3.0));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	
	group_->setPoseTarget(tool_pose);
	success = group_->plan(my_plan);

	ROS_INFO("Visualizing plan t (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group_->move();
	}
	
	q.setRPY(0.0, 0.0, normal_angle);	
	tool_pose.position.x = axes_pos.x;
	tool_pose.position.y = axes_pos.y;
	tool_pose.position.z = axes_pos.z - radius;
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	
	group_->setPoseTarget(tool_pose);
	success = group_->plan(my_plan);

	ROS_INFO("Visualizing plan t (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group_->move();
	}
	

	tool_pose.position.z = axes_pos.z - radius -0.05;
	
	group_->setPoseTarget(tool_pose);
	success = group_->plan(my_plan);

	ROS_INFO("Visualizing plan t (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group_->move();
	}

	
	
	return;
}
int  
turn_right_trajectory2(geometry_msgs::Point axes_pos, double normal_angle, double radius)
{
                                          
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose tool_pose;
	tf::Quaternion q;
	geometry_msgs::Point aux;

	// Initial pose
	q.setRPY(-PI, 0.0, normal_angle);			
	tool_pose.position.x = axes_pos.x;
	tool_pose.position.y = axes_pos.y;
	tool_pose.position.z = axes_pos.z + radius + 0.05;
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	// Connect with handle	
	tool_pose.position.z = axes_pos.z + radius;
	waypoints.push_back(tool_pose);

	// Start to turn
	q.setRPY(-5*PI/6.0, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(PI/3.0), aux.x, aux.y);
	tool_pose.position.x = aux.x;
	tool_pose.position.y = aux.y;
	tool_pose.position.z = axes_pos.z + (radius*sin(PI/3.0));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	q.setRPY(-2*PI/3.0, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(PI/6.0), aux.x, aux.y);
	tool_pose.position.x = aux.x;
	tool_pose.position.y = aux.y;
	tool_pose.position.z = axes_pos.z + (radius*sin(PI/6.0));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	q.setRPY(-PI/2.0, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius, aux.x, aux.y);
	tool_pose.position.x = aux.x;
	tool_pose.position.y = aux.y;
	tool_pose.position.z = axes_pos.z;
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	q.setRPY(-PI/3.0, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(-PI/6.0), aux.x, aux.y);
	tool_pose.position.x = aux.x;
	tool_pose.position.y = aux.y;
	tool_pose.position.z = axes_pos.z + (radius*sin(-PI/6.0));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	q.setRPY(-PI/6, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(-PI/3.0), aux.x, aux.y);
	tool_pose.position.x = aux.x;
	tool_pose.position.y = aux.y;
	tool_pose.position.z = axes_pos.z + (radius*sin(-PI/3.0));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	q.setRPY(0.0, 0.0, normal_angle);	
	tool_pose.position.x = axes_pos.x;
	tool_pose.position.y = axes_pos.y;
	tool_pose.position.z = axes_pos.z - radius;
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	q.setRPY(PI/6, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(-2*PI/3.0), aux.x, aux.y);
	tool_pose.position.x = aux.x;
	tool_pose.position.y = aux.y;
	tool_pose.position.z = axes_pos.z + (radius*sin(-2*PI/3.0));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	q.setRPY(PI/3, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(-5*PI/6.0), aux.x, aux.y);
	tool_pose.position.x = aux.x;
	tool_pose.position.y = aux.y;
	tool_pose.position.z = axes_pos.z + (radius*sin(-5*PI/6.0));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	q.setRPY(PI/2, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(-PI), aux.x, aux.y);
	tool_pose.position.x = aux.x;
	tool_pose.position.y = aux.y;
	tool_pose.position.z = axes_pos.z + (radius*sin(-PI));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	q.setRPY(2*PI/3, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(5*PI/6.0), aux.x, aux.y);
	tool_pose.position.x = aux.x;
	tool_pose.position.y = aux.y;
	tool_pose.position.z = axes_pos.z + (radius*sin(5*PI/6.0));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	q.setRPY(5*PI/6.0, 0.0, normal_angle);	
	move_right_plane(axes_pos.x, axes_pos.y, normal_angle, radius*cos(2*PI/3), aux.x, aux.y);
	tool_pose.position.x = aux.x;
	tool_pose.position.y = aux.y;
	tool_pose.position.z = axes_pos.z + (radius*sin(2*PI/3));
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	q.setRPY(-PI, 0.0, normal_angle);			
	tool_pose.position.x = axes_pos.x;
	tool_pose.position.y = axes_pos.y;
	tool_pose.position.z = axes_pos.z + radius;
	tool_pose.orientation.x = q.x();
	tool_pose.orientation.y = q.y();
	tool_pose.orientation.z = q.z();
	tool_pose.orientation.w = q.w();
	waypoints.push_back(tool_pose);
	
	// Disconnect from handle
	tool_pose.position.z = axes_pos.z + radius +0.05;
	waypoints.push_back(tool_pose);
	
	moveit_msgs::RobotTrajectory trajectory;
	double fraction;
	moveit::planning_interface::MoveGroup::Plan plan;
	
	bool success_trajectory = false;

		fraction = group_->computeCartesianPath(waypoints,
                                             0.02,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory, false);

		ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
		fraction * 100.0);
		if(fraction == 1.0)
		{
			plan.trajectory_ = trajectory;
			group_->execute(plan);
			success_trajectory = true;	
		}

	
	if(!success_trajectory)
	{
		ROS_INFO("Planning failed, random planning");
		group_->setRandomTarget();
		group_->move();
		return 1;
	}
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(5.0);
                              
	return 0;
}

bool 
saveScan()
{
	std_srvs::Trigger srv;
	if (client.call(srv))
	{
		if(srv.response.success)
		{
			ROS_INFO("Response: %s", srv.response.message.c_str());
		}
		else
		{
			ROS_WARN("Service problem");
		}
	}
	else
	{
		ROS_ERROR("Failed to call service");
		return false;
	}
	return true;
}
bool
go2Pose(geometry_msgs::Point position, tf::Quaternion q, int attempts)
{
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success;
	geometry_msgs::Pose target_pose;
	
	target_pose.position.x = position.x;
	target_pose.position.y = position.y;
	target_pose.position.z = position.z;
	target_pose.orientation.x = q.x();
	target_pose.orientation.y = q.y();
	target_pose.orientation.z = q.z();
	target_pose.orientation.w = q.w();
	

	
	for(int i=0;i<attempts;i++){
		group_->setPoseTarget(target_pose);
		success = group_->plan(my_plan);

		ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
		ROS_INFO("Try:%d",i);
		/* Sleep to give Rviz time to visualize the plan. */
	
		if (success){
			group_->execute(my_plan);
			sleep(1.0);
			return true;
		}
	}
	return false;
}

void scanning_z(geometry_msgs::Point position, tf::Quaternion q, double z_init, double z_end, double interval)
{
	ROS_INFO("SCANNING_Z --> z_init:%f", z_init);
	position.z = z_init;
	while(position.z<z_end)
	{
		ROS_INFO("SCANNING_Z ---> x:%f,y:%f,z:%f",position.x,position.y,position.z);
		go2Pose(position, q, 20);
		if (saveScan()) 	position.z += interval;
	}
	
	return;
}
