#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/passthrough.h>

#include "std_msgs/String.h"

#include <sstream>
#include <vector>

ros::Publisher pub;
ros::Publisher chatter_pub;

std_msgs::String msg;

std::stringstream ss;
std::ostringstream convert;

int Number = 0;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segment (new pcl::PointCloud<pcl::PointXYZ>);

   // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform downsampling
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.02, 0.02, 0.02);
  sor.filter (cloud_filtered);

  // convert PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_filtered,*temp_cloud);

  // segmenting area
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (temp_cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0.0, 10.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_segment);

  convert << Number << " test middle " << Number + 1;
  ss << convert.str() << std::endl << std::endl;
  msg.data = ss.str();
  
  convert.str("");
  convert.clear();

  
  // Find centroid

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  //pcl_conversions::fromPCL(cloud_filtered, output);
  pcl::toROSMsg(*cloud_segment, output);

  // Publish the data
  pub.publish (output);
  //chatter_pub.publish(msg);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_tracking");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  chatter_pub = nh.advertise<std_msgs::String>("chatter", 1);

  // Spin
  ros::spin();
}
