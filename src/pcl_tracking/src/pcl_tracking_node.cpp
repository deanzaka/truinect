
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

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"

#include <sstream>
#include <vector>

ros::Publisher pub;
ros::Publisher chatter_pub;
ros::Publisher cock_pose_pub;
ros::Publisher marker_pub;

visualization_msgs::Marker marker, line, points;

std_msgs::String msg;

std::stringstream ss;
std::ostringstream convert;

void
marker_init() {
 // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
	uint32_t line_strip = visualization_msgs::Marker::LINE_STRIP;
	uint32_t points_mark =  visualization_msgs::Marker::POINTS;

  points.header.frame_id = line.header.frame_id = marker.header.frame_id = "camera_link";
  points.header.stamp = line.header.stamp = marker.header.stamp = ros::Time::now();
  marker.type = shape;
  line.type = line_strip;
  points.type = points_mark;
  points.action = line.action = marker.action = visualization_msgs::Marker::ADD;

  line.id = 0;
  points.id = 1;
  marker.id = 2;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  points.scale.x = 0.1;
  points.scale.y = 0.1;

  points.color.r = 0.0f;
  points.color.g = 0.0f;
  points.color.b = 1.0f;
  points.color.a = 1.0;

  line.scale.x = 0.05;	
  line.color.r = 1.0f;
  line.color.g = 0.0f;
  line.color.b = 0.0f;
  line.color.a = 1.0;
}

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
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);

  // convert PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_filtered,*temp_cloud);

  // segmenting area
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (temp_cloud);
  // left - right
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-2, 2);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_segment);

  pass.setInputCloud (cloud_segment);
  // up - down
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0, 2);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_segment);

  pass.setInputCloud (cloud_segment);
  // backward - forward
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.8, 3);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_segment);

  // Find centroid
  float sum_x = 0;
  float sum_y = 0;
  float sum_z = 0;
  if(cloud_segment->points.size() > 50) {
	  for(size_t i = 0; i < cloud_segment->points.size(); ++i) {
	  	sum_x += cloud_segment->points[i].x;
	  	sum_y += cloud_segment->points[i].y;
	  	sum_z += cloud_segment->points[i].z;
	  }

	  sum_x = sum_x / cloud_segment->points.size();
	  sum_y = sum_y / cloud_segment->points.size();
	  sum_z = sum_z / cloud_segment->points.size();

	  geometry_msgs::PoseStamped sPose;
    sPose.header.stamp = ros::Time::now();

    marker.lifetime = ros::Duration();
    
    sPose.pose.position.x = sum_z; // equals to z in pcl // backward - forward
    sPose.pose.position.y = -(sum_x); // equals to -x in pcl // right - left
    sPose.pose.position.z = -(sum_y); // equals to -y in pcl // down - up

    sPose.pose.orientation.x = 0.0;
    sPose.pose.orientation.y = 0.0;
    sPose.pose.orientation.z = 0.0;
    sPose.pose.orientation.w = 1.0;

    cock_pose_pub.publish(sPose);
    marker.pose = sPose.pose;

    geometry_msgs::Point p;
    p.x = sum_z;
    p.y = -(sum_x);
    p.z = -(sum_y);


    // line.lifetime = ros::Duration(5);
    // points.lifetime = ros::Duration(5);

    line.pose.orientation.w = 1.0;
    points.pose.orientation.w = 1.0;
    line.points.push_back(p);
    points.points.push_back(p);

    marker_pub.publish(marker);
    marker_pub.publish(line);
    marker_pub.publish(points);

	  convert << "Test: " << sum_x << "  " << sum_y << "  " << sum_z;
	  ss << convert.str() << std::endl << std::endl;
	  msg.data = ss.str();
	  
	  convert.str("");
	  convert.clear();
	  
	  chatter_pub.publish(msg);
	}

  

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  //pcl_conversions::fromPCL(cloud_filtered, output);
  pcl::toROSMsg(*cloud_segment, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_tracking");
  ros::NodeHandle nh;

  // Initialize marker
  marker_init();

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  chatter_pub = nh.advertise<std_msgs::String>("chatter", 1);
  cock_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cock_pose", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Spin
  ros::spin();
}
