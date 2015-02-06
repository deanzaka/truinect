#ifndef TRUINECT_PCL_H_
#define TRUINECT_PCL_H_

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace truinect_pcl {

typedef void (*voidFuncPtr)(void);

class PCL_Tutor {
 public:
  PCL_Tutor(ros::NodeHandle nh);
  ~PCL_Tutor();

  void run(ros::Rate loop_rate);
 
 private:
  ros::NodeHandle nh_;

};

}

#endif