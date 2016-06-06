#include <ros/ros.h>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <math.h>       /* atan2 */

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


#include <geometry_msgs/Pose.h>

//Constants
static const double PI = 3.14159265;
static const std::string COLLISION_TOPIC = "/collision_object";
ros::Publisher pub_collision_obj_; // for MoveIt collision objects

// Function names
void move_back_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end);
void move_forward_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end);
void move_right_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end);
void move_left_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end);



/// -   -   -   -   - MAIN -    -   -   -   - ///
int main(int argc, char* argv[]){
	
	    //Startup ros
    ros::init(argc, argv, "test1");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Get the node handle for this node
    ros::NodeHandle node("~");
    
  
    moveit::planning_interface::MoveGroup group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
	
	
	double m = -3.0/2.0;
	double n = 3.0;
	double x_plane = 0.5;
	double y_plane = 0.5;
	
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
	
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	double m_normal = -1/m;
	
	if (cuad == 1)			yaw = atan2(m_normal,1);
	else if (cuad == 2)		yaw = atan2(-m_normal,-1);
	else if (cuad == 3)		yaw = atan2(-m_normal,-1);
	else if (cuad == 4)		yaw = atan2(m_normal,1);
	else if (cuad == 0)		yaw = PI/2;
	else if (cuad == 1)		yaw = 3*PI/2;
	
	double yaw_degree = yaw* 180 / PI;
	ROS_INFO("cuad= %d | yaw = %f",cuad, yaw_degree);		
	
		
	tf::Quaternion q;
	q.setRPY(roll, pitch, yaw);

	double d = 0.1;
	double x_target;
	double y_target;
	
	move_back_plane(x_plane, y_plane, yaw, d, x_target, y_target);
	//~ x_target = x_plane - d*cos(yaw);
	//~ y_target = y_plane - d*sin(yaw);
	
	ROS_INFO("x:%f,y:%f",x_target,y_target);


	geometry_msgs::Pose target_pose1;
	target_pose1.position.x = x_target;
	target_pose1.position.y = y_target;
	target_pose1.position.z = 0.5;
	target_pose1.orientation.x = q.x();
	target_pose1.orientation.y = q.y();
	target_pose1.orientation.z = q.z();
	target_pose1.orientation.w = q.w();

	group.setPoseTarget(target_pose1);
	
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group.move();
		sleep(5.0);
	}
	
	
	moveit_msgs::CollisionObject collision_obj;

	//Configure publisher
	pub_collision_obj_ = node.advertise<moveit_msgs::CollisionObject>(COLLISION_TOPIC, 0);
	sleep(2.0);	

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
	ROS_INFO("Published collision object ");


	/* Sleep so we have time to see the object in RViz */
	sleep(5.0);
	group.setPlanningTime(10.0);
	
	/*Remove object*/
	//~ collision_obj.header.stamp = ros::Time::now();
	//~ collision_obj.operation = moveit_msgs::CollisionObject::REMOVE;
	//~ pub_collision_obj_.publish(collision_obj);
	//~ ros::spinOnce();
	//~ ROS_INFO("Removed object ");
	//~ 
	//~ sleep(10.0);

    //~ ros::Rate loop_rate(1);   
    //~ while (ros::ok())
	//~ {
		//~ ros::spinOnce();
		//~ loop_rate.sleep();
    //~ }
    
    
    
    double x_aux;
    double y_aux;
    
    x_aux = x_target;
    y_aux = y_target;
    move_back_plane(x_aux, y_aux, yaw, d, x_target, y_target);

	target_pose1.position.x = x_target;
	target_pose1.position.y = y_target;
	target_pose1.position.z = 0.5;
	target_pose1.orientation.x = q.x();
	target_pose1.orientation.y = q.y();
	target_pose1.orientation.z = q.z();
	target_pose1.orientation.w = q.w();

	group.setPoseTarget(target_pose1);
	success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group.move();
		sleep(5.0);
	}
	
	x_aux = x_target;
    y_aux = y_target;
    move_forward_plane(x_aux, y_aux, yaw, d, x_target, y_target);

	target_pose1.position.x = x_target;
	target_pose1.position.y = y_target;
	target_pose1.position.z = 0.5;
	target_pose1.orientation.x = q.x();
	target_pose1.orientation.y = q.y();
	target_pose1.orientation.z = q.z();
	target_pose1.orientation.w = q.w();

	group.setPoseTarget(target_pose1);
	success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group.move();
		sleep(5.0);
	}
	
	x_aux = x_target;
    y_aux = y_target;
    move_right_plane(x_aux, y_aux, yaw, d, x_target, y_target);

	target_pose1.position.x = x_target;
	target_pose1.position.y = y_target;
	target_pose1.position.z = 0.5;
	target_pose1.orientation.x = q.x();
	target_pose1.orientation.y = q.y();
	target_pose1.orientation.z = q.z();
	target_pose1.orientation.w = q.w();

	group.setPoseTarget(target_pose1);
	success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group.move();
		sleep(5.0);
	}
	
	x_aux = x_target;
    y_aux = y_target;
    move_left_plane(x_aux, y_aux, yaw, d, x_target, y_target);

	target_pose1.position.x = x_target;
	target_pose1.position.y = y_target;
	target_pose1.position.z = 0.5;
	target_pose1.orientation.x = q.x();
	target_pose1.orientation.y = q.y();
	target_pose1.orientation.z = q.z();
	target_pose1.orientation.w = q.w();
	
	group.setPoseTarget(target_pose1);
	success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	
	if (success){
		group.move();
		sleep(5.0);
	}
    
    
    
    
    return 0;
}

void move_back_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end)
{
	x_end = x_ini - d*cos(normal_angle);
	y_end = y_ini - d*sin(normal_angle);
	
	return;
}
void move_forward_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end)
{
	x_end = x_ini + d*cos(normal_angle);
	y_end = y_ini + d*sin(normal_angle);
	
	return;
}
void move_right_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end)
{
	x_end = x_ini + d*sin(normal_angle);
	y_end = y_ini - d*cos(normal_angle);
	
	return;
}
void move_left_plane(double x_ini, double y_ini, double normal_angle, double d, double &x_end, double &y_end)
{
	x_end = x_ini - d*sin(normal_angle);
	y_end = y_ini + d*cos(normal_angle);
	
	return;
}
