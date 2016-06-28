#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//~ #include <pcl/filters/model_outlier_removal.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZ PointT;

main(int argc, char **argv)
{
    ros::init (argc, argv, "pcl_read");

    ROS_INFO("Started PCL read node");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    
    std::string pcd_path;
    ros::param::param<std::string>("/pcl_read/pcd_path", pcd_path, "aux_pcd.pcd");
    ROS_INFO("Reading from %s", pcd_path.c_str());
    pcl::io::loadPCDFile (pcd_path.c_str(), *cloud);
    
    
    
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-1.0, 1.0);
  //~ pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0.0, 2.0);
  //~ pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  
    //~ // Create the filtering object
  //~ pcl::VoxelGrid<pcl::PointXYZ> sor;
  //~ sor.setInputCloud (cloud_filtered->makeShared());
  //~ sor.setLeafSize (0.01f, 0.01f, 0.01f);
  //~ sor.filter (*cloud_filtered);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filtered);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.5);
  sor.filter (*cloud_filtered);    


    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.037);
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
  
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
  
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (true); // Set false for the other part of the cloud
    extract.filter(*cloud_filtered);

  
    //~ pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    //~ // build the filter
    //~ outrem.setInputCloud(cloud_filtered);
    //~ outrem.setRadiusSearch(0.003);
    //~ outrem.setMinNeighborsInRadius (3);
    //~ // apply filter
    //~ outrem.filter (*cloud_filtered);
    
    
    
    
    //~ // Detect max and min of the clouds
    //~ pcl::PointXYZ minPt, maxPt;
    //~ pcl::getMinMax3D(*cloud_filtered, minPt, maxPt);
    //~ double delta_x = maxPt.x - minPt.x;
    //~ double delta_y = maxPt.y - minPt.y;
    //~ double delta_z = maxPt.z - minPt.z;
    //~ ROS_INFO("Handle dimensions x:%f y:%f z:%f", delta_x, delta_y, delta_z);
    
    
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg2;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    
    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);
    
    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg2.setOptimizeCoefficients (true);
    seg2.setModelType (pcl::SACMODEL_CYLINDER);
    seg2.setMethodType (pcl::SAC_RANSAC);
    seg2.setNormalDistanceWeight (0.1);
    seg2.setMaxIterations (10000);
    seg2.setDistanceThreshold (0.05);
    seg2.setRadiusLimits (0, 0.1);
    seg2.setInputCloud (cloud_filtered);
    seg2.setInputNormals (cloud_normals);
    














    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = "world";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
