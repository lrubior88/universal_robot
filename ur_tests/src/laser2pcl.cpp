#include "ros/ros.h"

#include "std_srvs/Trigger.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

// PCL Includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>


sensor_msgs::LaserScan current_scan;
sensor_msgs::PointCloud2::Ptr lidar_cloud_ros(new sensor_msgs::PointCloud2);
pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr complete_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher point_cloud_publisher_;
tf::TransformListener *tf_listener;


bool laser2pcl_callback(std_srvs::Trigger::Request  &req,
						std_srvs::Trigger::Response &res)
{
  ROS_INFO("laser2pcl_callback");

  laser_geometry::LaserProjection projector_;

  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud("world", current_scan, *lidar_cloud_ros, *tf_listener);

  lidar_cloud_pcl->points.clear();
  pcl::fromROSMsg(*lidar_cloud_ros, *lidar_cloud_pcl);

  *complete_cloud_pcl += *lidar_cloud_pcl;  
  
  res.success = true;
  res.message = "Todo bien";
  return true;
}

void getScanInfo(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  current_scan = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser2pcl");
  ros::NodeHandle n;
  
  
  ros::Subscriber sub = n.subscribe("/hokuyo/laser/scan", 1, getScanInfo);
  ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hokuyo/laser/scan");
  
  tf_listener = new tf::TransformListener();
  tf_listener->setExtrapolationLimit(ros::Duration(0.1));

  point_cloud_publisher_ = n.advertise<sensor_msgs::PointCloud2> ("/hokuyo/pcl", 1, false);
  ros::ServiceServer service = n.advertiseService("/laser2pcl_srv", laser2pcl_callback);
  ROS_INFO("Laser2pcl:  Service ready");
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    sensor_msgs::PointCloud2 output;  
    pcl::toROSMsg(*complete_cloud_pcl, output);
    output.header.frame_id = "world";
    point_cloud_publisher_.publish(output);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
