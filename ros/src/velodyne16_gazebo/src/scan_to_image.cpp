#include "velodyne16_gazebo/scan_to_image.h"

namespace velodyne16_gazebo {

ScanToImage::ScanToImage(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  private_nh.param("target_frame_id", target_frame_id_, std::string("velodyne_optical"));
  private_nh.param("width", width_, 255);
  private_nh.param("height", height_, 255);

  private_nh.param("planar_projection", planar_projection_, true);

  // Parameters for planar projection
  private_nh.param("focal_length_x", fx_, 125.5f);
  private_nh.param("focal_length_y", fy_, 125.5f);
  px_ = static_cast<float>(width_)/2;
  py_ = static_cast<float>(height_)/2;

  // Parameters for spherical projection
  private_nh.param("x_angle_range", xAngleRange_, static_cast<float>(2.0*M_PI));
  xAngularResolution_ = xAngleRange_ / width_;
  private_nh.param("y_angular_resolution", yAngularResolution_, 0.02321288f);
  private_nh.param("x_angle_offset", xAngleOffset_, 0.0f);
  private_nh.param("y_angle_offset", yAngleOffset_, 4.45059f);


  // For suncg out folder is the out folder path + name of house
  std::string world_name;

  private_nh.param("out_folder_path", out_folder_path_, std::string(""));

  if (private_nh.getParam("/world_name", world_name)){
    boost::filesystem::path p (world_name);
    out_folder_path_ = out_folder_path_ + "/" + p.parent_path().filename().c_str();
  } else
    ROS_WARN("Failed to get param 'world_name'.");

  if (out_folder_path_ != "" ) out_folder_path_+= "/";
  if ((out_folder_path_ != "") && !(boost::filesystem::exists(out_folder_path_)))
    boost::filesystem::create_directories(out_folder_path_);

  ROS_INFO ("Output folder: %s ", out_folder_path_.c_str());

  // Resize vectors
  depth_.resize(width_*height_, 0);
  labels_.resize(width_*height_, 0);
  normals_xyz_.resize(width_*height_*3, 0);

  assembled_cloud_ =
      node.subscribe("assembled_cloud", 10, &ScanToImage::processScan, this);

  writer = H5Wrapper(out_folder_path_ + "training_", width_, height_, 10);

  bool suncg_house;
  private_nh.param("suncg_house", suncg_house, false);
  // For laboratory building
  // stairs -1, doors -2, person -3, chairs -4, tables -5;
  if (!suncg_house){
    labelmap_ = { {50, 1}, {150, 2}, {20, 3}, {130, 4}, {80, 5}, {120, 5}};
  } else {
    // For suncg dataset
    labelmap_ = { {3, 5}, {12, 4}, {37, 5}, {40, 5}, {93, 5}, {101, 4}, {106, 5}, {121, 2}, {129, 1}, {170, 4}, {179, 2}, {186, 3}, {13, 255}, {79, 255}, {110, 255}, {157, 255}, {172, 255}, {184, 255} };
  }
  
}

void ScanToImage::processScan(const PclCloudPtr &point_cloud)
{
  PclCloudPtr transformed_cloud (new PclCloud);
  std::fill(depth_.begin(), depth_.end(), std::numeric_limits<float>::quiet_NaN());
  std::fill(labels_.begin(), labels_.end(), 255);
  std::fill(normals_xyz_.begin(), normals_xyz_.end(),  std::numeric_limits<float>::quiet_NaN());

  if (target_frame_id_ == "") target_frame_id_ = point_cloud->header.frame_id;
  pcl_ros::transformPointCloudWithNormals (target_frame_id_, *point_cloud, *transformed_cloud, listener_);
  for(int i=0; i<transformed_cloud->size(); ++i){
    // Skip nan points
    if (std::isnan(transformed_cloud->points[i].x) || std::isnan(transformed_cloud->points[i].y) || std::isnan(transformed_cloud->points[i].z)) continue;

    int intensity = static_cast<int>(transformed_cloud->points[i].intensity);
    if(labelmap_.find(intensity) != labelmap_.end())
    {
      transformed_cloud->points[i].intensity = labelmap_[intensity];
    } else {
      transformed_cloud->points[i].intensity = 0;
    }

    if (planar_projection_){
      // Skip if point is behind the camera
      if(transformed_cloud->points[i].getVector3fMap()[2]<=0 ) continue;
    }

    Eigen::Vector2i projected;
    if (planar_projection_){
      projected = getProjectedPointIndex(transformed_cloud->points[i].getVector3fMap());
    } else {
      projected = getProjectedIndexVelodyne(transformed_cloud->points[i].getVector3fMap());
    }
    // Check boundary conditions
    if( projected(1) < 0 || projected(1) >= height_ ) continue;
    if( projected(0) < 0 || projected(0) >= width_  ) continue;

    float distance = transformed_cloud->points[i].getVector3fMap().norm();

    uint32_t index;
    if (planar_projection_){
      index = projected(0)+projected(1)*width_;
    } else {
      index = (width_ - 1 - projected(0))+(height_ - 1 - projected(1))*width_;
    }

    if (std::isnan(depth_[index]) || depth_[index] > distance ){
      // Depth image
      depth_[index] = distance;
      // Label image
      labels_[index] = transformed_cloud->points[i].intensity;
      // Normals image
      normals_xyz_[index+0*(width_*height_)] = transformed_cloud->points[i].normal_z; // B
      normals_xyz_[index+1*(width_*height_)] = transformed_cloud->points[i].normal_y; // G
      normals_xyz_[index+2*(width_*height_)] = transformed_cloud->points[i].normal_x; // R
    }
  }

  // Generate filenames
  writer.add_normals(normals_xyz_);
  writer.add_depth(depth_);
  writer.add_labels(labels_);

  std::stringstream ss;
  ss << std::setfill('0') << std::setw(6) << transformed_cloud->header.seq;

  std::string transform_name;
  transform_name = out_folder_path_ +"transform_" + ss.str() + ".txt";
  saveTransform(*point_cloud, transform_name);

  std::string pcd_name;
  pcd_name = out_folder_path_ +"assembled_cloud_" + ss.str() + ".pcd";
  pcl::io::savePCDFileBinaryCompressed(pcd_name, *transformed_cloud);
}

void ScanToImage::reproject(const PclCloudPtr &transformed_cloud)
{
  std::fill(depth_.begin(), depth_.end(), std::numeric_limits<float>::quiet_NaN());
  std::fill(labels_.begin(), labels_.end(), 255);
  std::fill(normals_xyz_.begin(), normals_xyz_.end(),  std::numeric_limits<float>::quiet_NaN());

  for(int i=0; i<transformed_cloud->size(); ++i){
    // Skip nan points
    if (std::isnan(transformed_cloud->points[i].x) || std::isnan(transformed_cloud->points[i].y) || std::isnan(transformed_cloud->points[i].z)) continue;

    int intensity = static_cast<int>(transformed_cloud->points[i].intensity);

    if (planar_projection_){
      // Skip if point is behind the camera
      if(transformed_cloud->points[i].getVector3fMap()[2]<=0 ) continue;
    }

    Eigen::Vector2i projected;
    if (planar_projection_){
      projected = getProjectedPointIndex(transformed_cloud->points[i].getVector3fMap());
    } else {
      projected = getProjectedIndexVelodyne(transformed_cloud->points[i].getVector3fMap());
    }
    // Check boundary conditions
    if( projected(1) < 0 || projected(1) >= height_ ) continue;
    if( projected(0) < 0 || projected(0) >= width_  ) continue;

    float distance = transformed_cloud->points[i].getVector3fMap().norm();

    uint32_t index;
    if (planar_projection_){
      index = projected(0)+projected(1)*width_;
    } else {
      index = (width_ - 1 - projected(0))+(height_ - 1 - projected(1))*width_;
    }

    if (std::isnan(depth_[index]) || depth_[index] > distance ){
      // Depth image
      depth_[index] = distance;
      // Label image
      labels_[index] = transformed_cloud->points[i].intensity;
      // Normals image
      normals_xyz_[index+0*(width_*height_)] = transformed_cloud->points[i].normal_z; // B
      normals_xyz_[index+1*(width_*height_)] = transformed_cloud->points[i].normal_y; // G
      normals_xyz_[index+2*(width_*height_)] = transformed_cloud->points[i].normal_x; // R
    }
  }

  // Generate filenames
  writer.add_normals(normals_xyz_);
  writer.add_depth(depth_);
  writer.add_labels(labels_);
}

void ScanToImage::reproject_h5(const PclCloudPtr &transformed_cloud, std::string h5_name, std::string out_pcd_name)
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud (new pcl::PointCloud<pcl::PointXYZL>);
  labeled_cloud->header   = transformed_cloud->header;
  labeled_cloud->width    = transformed_cloud->width;
  labeled_cloud->height   = transformed_cloud->height;
  labeled_cloud->is_dense = transformed_cloud->is_dense;
  labeled_cloud->sensor_orientation_ = transformed_cloud->sensor_orientation_;
  labeled_cloud->sensor_origin_ = transformed_cloud->sensor_origin_;
  labeled_cloud->points.resize (transformed_cloud->points.size ());
  
  std::fill(depth_.begin(), depth_.end(), std::numeric_limits<float>::quiet_NaN());
  
  std::vector<int> idx;
  idx.resize(width_*height_, 0);
  std::fill(idx.begin(), idx.end(),  std::numeric_limits<float>::quiet_NaN());
  
  std::vector<int> labels (width_*height_);

  H5::H5File file(h5_name, H5F_ACC_RDONLY);
  H5::DataSet dataset = file.openDataSet("labels");
  H5::DataSpace filespace = dataset.getSpace();
  int rank = filespace.getSimpleExtentNdims();
  hsize_t dims[2];    // dataset dimensions
  rank = filespace.getSimpleExtentDims( dims );
  H5::DataSpace mspace(rank, dims);
  dataset.read( labels.data(), H5::PredType::NATIVE_INT, mspace, filespace);

  for(int i=0; i<transformed_cloud->size(); ++i){
    labeled_cloud->points[i].x = transformed_cloud->points[i].x;
    labeled_cloud->points[i].y = transformed_cloud->points[i].y;
    labeled_cloud->points[i].z = transformed_cloud->points[i].z;
    labeled_cloud->points[i].label = 255;
    // Skip nan points
    if (std::isnan(transformed_cloud->points[i].x) || std::isnan(transformed_cloud->points[i].y) || std::isnan(transformed_cloud->points[i].z)) continue;

    if (planar_projection_){
      // Skip if point is behind the camera
      if(transformed_cloud->points[i].getVector3fMap()[2]<=0 ) continue;
    }

    Eigen::Vector2i projected;
    if (planar_projection_){
      projected = getProjectedPointIndex(transformed_cloud->points[i].getVector3fMap());
    } else {
      projected = getProjectedIndexVelodyne(transformed_cloud->points[i].getVector3fMap());
    }
    // Check boundary conditions
    if( projected(1) < 0 || projected(1) >= height_ ) continue;
    if( projected(0) < 0 || projected(0) >= width_  ) continue;

    float distance = transformed_cloud->points[i].getVector3fMap().norm();

    uint32_t index;
    if (planar_projection_){
      index = projected(0)+projected(1)*width_;
    } else {
      index = (width_ - 1 - projected(0))+(height_ - 1 - projected(1))*width_;
    }

    if (std::isnan(depth_[index]) || depth_[index] > distance ){
      idx[index] = i;
      depth_[index] = distance;
    }
  }
  
   for(int i=0; i< idx.size(); ++i){
    if (!std::isnan(idx[i]))
       labeled_cloud->points[idx[i]].label = labels[i];
   }

  pcl::io::savePCDFileBinaryCompressed(out_pcd_name, *labeled_cloud);

}


bool  ScanToImage::saveTransform (const PclCloud &cloud_in, const std::string& out_file_name)
{
  tf::StampedTransform transform;
  try
  {
    listener_.lookupTransform (target_frame_id_, cloud_in.header.frame_id, pcl_conversions::fromPCL(cloud_in.header).stamp, transform);
  }
  catch (tf::LookupException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }
  catch (tf::ExtrapolationException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }
  tf::Quaternion q = transform.getRotation();
  tf::Vector3 v = transform.getOrigin();
  std::ofstream out_file;
  out_file.open (out_file_name);
  out_file << std::setprecision(17) << v.getX() << " " << v.getY() << " " << v.getZ() << " " <<  q.getX() << " " << q.getY() << " " << q.getZ() << " " << q.getW();

  out_file.close();
  return (true);
}

Eigen::Vector2i ScanToImage::getProjectedPointIndex(const Eigen::Vector3f &point)
{
  // x' = x/z
  // y' = y/z
  // u = f_x * x' + c_x
  // v = f_y * y' + c_y
  Eigen::Vector2i index;
  index(0) = std::floor( (point.x() / point.z() ) * fx_ + px_);
  index(1) = std::floor( (point.y() / point.z() ) * fy_ + py_);
  return index;
}

void ScanToImage::saveNormalsToPng(std::string filename)
{
  std::vector<float> normalized_norm;
  // From 0 to 255
  std::transform(normals_xyz_.begin(), normals_xyz_.end(), std::back_inserter(normalized_norm), std::bind1st(std::multiplies<float>(),127.5f));
  std::transform(normalized_norm.begin(), normalized_norm.end(), normalized_norm.begin(), std::bind1st(std::plus<float>(),127.5f));
  // Nan is converted to 0
  std::vector<uchar> normals_data(normalized_norm.begin(), normalized_norm.end());
  cv::Mat normals(height_, width_, CV_8UC3, normals_data.data());
  cv::imwrite(filename.c_str(), normals);
}

Eigen::Vector2i ScanToImage::getProjectedIndexVelodyne(const Eigen::Vector3f& point){
  Eigen::Vector2i index;
  float angleX = atan2(point.y(), point.x());
  float angleY = asin(point.z() / point.norm());

  index(1) = std::floor(normalizeAngle(angleY - yAngleOffset_) / yAngularResolution_);
  index(0) = std::floor(normalizeAngle(angleX + xAngleOffset_) / xAngularResolution_);
  return index;
}

float ScanToImage::normalizeAngle(float angle)
{
  static float TwoPi = 2.0*M_PI;
  angle = std::fmod(angle, TwoPi);
  if(angle < 0.0){
    angle += TwoPi;
  }
  return angle;
}

void ScanToImage::saveLabelToPng(std::string filename)
{
  cv::Mat labels(height_, width_, CV_8UC1, labels_.data());
  cv::imwrite(filename.c_str(), labels);
}


void ScanToImage::saveDepthImageToPng(std::string filename)
{
  float multiplier = (256.0/MAX_RANGE_);
  std::vector<float> normalized_depth;
  std::transform(depth_.begin(), depth_.end(), std::back_inserter(normalized_depth), std::bind1st(std::multiplies<float>(), multiplier));

  std::vector<uchar> udepth(normalized_depth.begin(), normalized_depth.end());
  cv::Mat depth(height_, width_, CV_8UC1, udepth.data());
  cv::imwrite(filename.c_str(), depth);
}

} // namespace velodyne16_gazebo
