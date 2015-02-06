#include <truinect_pcl/pcl_tutor.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "PCL_Tutor");
  ros::NodeHandle nh;
  
  truinect_pcl::PCL_Tutor pcl_tutor(nh);

  ROS_INFO("Testing PCL Tutorial");
  ros::Duration(5.0).sleep();

	pcl_tutor.run(ros::Rate(10));

  return(0);
}