#include <ros/ros.h>
#include <velodyne16_gazebo/scan_to_image.h>
#include <boost/filesystem.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>

namespace fs = boost::filesystem;

int main (int argc, char **argv)
{
  ros::init(argc,argv,"image_to_pcd");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  velodyne16_gazebo::ScanToImage scan_to_img (n, private_nh);

  std::string pcdlist_filepath;
  std::string h5_filepath;
  std::string outpcd_filepath;
  // Read parameters
  private_nh.param("path_to_pcdlist", pcdlist_filepath, std::string("~/"));
  std::cout << "path pcdlist " << pcdlist_filepath << std::endl;
  private_nh.param("path_to_h5", h5_filepath, std::string("~/"));
  std::cout << "path h5 " << h5_filepath << std::endl;
  private_nh.param("out_path_pcd", outpcd_filepath, std::string("~/"));
  std::cout << "out path pcd " << outpcd_filepath << std::endl;
  std::ifstream input_file(pcdlist_filepath);
  std::vector<std::string> cloud_names;
  std::string line;
  if(!input_file) {
    std::cerr << "File list could not be opened\n";
    return 1;
  }
  while(std::getline(input_file, line)) {
    cloud_names.push_back(line);
  }
  std::cout << "Number of files to be analyzed: " << cloud_names.size() << "\n"; 
  
  for(const auto &cloudname : cloud_names){
  // Read PCD file
  std::vector<std::string> strs;
  boost::split(strs,cloudname,boost::is_any_of("/")); 
  for (int i = 0; i< strs.size(); ++i)
    std::cout << strs[i]  <<std::endl;
  std::vector<std::string> strs_name;
  boost::split(strs_name,strs[strs.size()-1],boost::is_any_of("_")); 
  strs_name[strs_name.size()-1]= boost::filesystem::change_extension(strs_name[strs_name.size()-1], "").string();
  for (int i = 0; i< strs_name.size(); ++i)
    std::cout << strs_name[i]  <<std::endl;
  
  std::stringstream h5_name;
  int num = stoi(strs_name[strs_name.size()-1]); 
  num = num/5;
  if (strs[strs.size()-3] == "house")
  {
    h5_name << strs[strs.size()-3] << "_" << strs[strs.size()-2] << "_training_" << num/6 << "_" << num%6 << "_pred.h5"; 
  }else{
    h5_name << strs[strs.size()-2] << "_training_" << num/6 << "_" << num%6 << "_pred.h5"; 
  } 
  std::cout << h5_filepath << "/" << h5_name.str() <<std::endl;
  std::string full_h5_path = h5_filepath + "/" + h5_name.str();
  if(!boost::filesystem::exists(full_h5_path)){
    std::cerr << full_h5_path << " doesn't exist.\n";
    return 1;
  }
  
  std::string out_pcd_name;
  out_pcd_name = outpcd_filepath + "/" + strs[strs.size()-2] + "_" + strs[strs.size()-1];
  std::cout << out_pcd_name <<std::endl;
  if(!boost::filesystem::exists(outpcd_filepath)){
    boost::filesystem::create_directories(outpcd_filepath);
  }
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::io::loadPCDFile(cloudname, *cloud);
  scan_to_img.reproject_h5(cloud, full_h5_path, out_pcd_name);
  }
  return 0;
}
