#include <ros/ros.h>
#include <velodyne16_gazebo/scan_to_image.h>
#include <boost/filesystem.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

namespace fs = boost::filesystem;

int main (int argc, char **argv)
{
  ros::init(argc,argv,"pcd_to_image");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  velodyne16_gazebo::ScanToImage scan_to_img (n, private_nh);

  std::string pcd_filepath;
  // Read parameters
  private_nh.param("path_to_clouds", pcd_filepath, std::string("~/"));
  std::cout << "path " << pcd_filepath << std::endl;
  // Check that folder with PCD files exist
  if(!fs::exists(pcd_filepath)){
    ROS_ERROR("Folder %s does not exist!", pcd_filepath.c_str());
    n.shutdown();
  }

  // Go through the folder with PCD files
  std::set<std::string> cloud_ext = {".pcd"};
  std::set<std::string> cloud_names;
  {
     fs::directory_iterator iter(pcd_filepath);
     fs::directory_iterator endit;
     while(iter != endit)
     {
       if(fs::is_regular_file(*iter) && cloud_ext.find(iter->path().extension().string()) != cloud_ext.end()){
         // Add PCD filename to set
         cloud_names.insert(iter->path().string());
       }
       ++iter;
      }
  }

  for(const auto &cloudname : cloud_names){
    // Read PCD file
    ROS_INFO("We will be reading %s", cloudname.c_str());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::io::loadPCDFile(cloudname, *cloud);
    std::cout << cloud->header.frame_id<<  std::endl;
    scan_to_img.reproject(cloud);
  }
  return 0;
}
