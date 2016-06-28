#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(input, cloud);
    std::string pcd_path;
    ros::param::param<std::string>("/pcl_write/pcd_path", pcd_path, "aux_pcd.pcd");
    ROS_INFO("Saving pcd at %s", pcd_path.c_str());
    pcl::io::savePCDFileASCII (pcd_path.c_str(), cloud);
}

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_write");

    ROS_INFO("Started PCL write node");

    ros::NodeHandle nh;
    ros::Subscriber bat_sub = nh.subscribe("/hokuyo/pcl", 10, cloudCB);

    ros::spin();

    return 0;
}
