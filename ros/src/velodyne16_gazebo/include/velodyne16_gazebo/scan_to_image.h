#ifndef SCAN_TO_IMAGE_H
#define SCAN_TO_IMAGE_H

#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <velodyne16_gazebo/h5wrapper.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace velodyne16_gazebo {

// Max range for Velodyne
static const float MAX_RANGE_ = 100.0f;

class ScanToImage
{
public:

  typedef pcl::PointXYZINormal PclPoint;
  typedef pcl::PointCloud<PclPoint> PclCloud;
  typedef PclCloud::Ptr PclCloudPtr;

  ScanToImage(ros::NodeHandle node, ros::NodeHandle private_nh);  
  ~ScanToImage(){};
  void reproject(const PclCloudPtr &transformed_cloud);
  void reproject_h5(const PclCloudPtr &transformed_cloud, std::string h5_name, std::string out_pcd_name);
private:
  void processScan(const PclCloudPtr &point_cloud);
  Eigen::Vector2i getProjectedPointIndex(const Eigen::Vector3f& point);
  Eigen::Vector2i getProjectedIndexVelodyne(const Eigen::Vector3f& point);
  void saveNormalsToPng(std::string filename);
  void saveLabelToPng(std::string filename);
  void saveDepthImageToPng(std::string filename);
  float normalizeAngle(float angle);
  bool saveTransform (const PclCloud &cloud_in, const std::string& out_file_name);

  ros::Subscriber assembled_cloud_;
  H5Wrapper writer;

  int width_;
  int height_;

  bool planar_projection_; // If false the spherical projection is used

  // Parameters for planar projection
  float fx_;
  float fy_;
  float px_;
  float py_;
  // Parameters for spherical projection
  float xAngularResolution_;
  float xAngleOffset_;
  float yAngleOffset_;
  float yAngularResolution_;
  float xAngleRange_;

  std::vector<float> depth_;
  std::vector<float> normals_xyz_;
  std::vector<int> labels_;
  std::vector<int> idx_;

  tf::TransformListener listener_;
  std::string target_frame_id_;

  std::unordered_map<int, int> labelmap_;
  std::string out_folder_path_;

};

} //namespace


#endif // SCAN_TO_IMAGE_H
