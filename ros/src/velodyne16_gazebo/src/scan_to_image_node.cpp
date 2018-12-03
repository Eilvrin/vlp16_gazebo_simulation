#include <ros/ros.h>
#include "velodyne16_gazebo/scan_to_image.h"

int main (int argc, char **argv)
{
  ros::init(argc,argv,"scan_to_image");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  velodyne16_gazebo::ScanToImage scan_to_img (n, private_nh);
  
  // handle callbacks until shut down
  ros::spin();
  return 0;
}
