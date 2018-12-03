#include <velodyne16_gazebo/h5wrapper.h>


H5Wrapper::H5Wrapper(){}
H5Wrapper::H5Wrapper(std::string filename_pattern, int width, int height, int n_images) : 
        filename_pattern_(filename_pattern),
	width_(width), height_(height), n_images_(n_images){
  normals_accumulate_.reserve(n_images_*3*height_*width_);
  depth_accumulate_.reserve(n_images_*1*height_*width_);
  labels_accumulate_.reserve(n_images_*1*height_*width_);
}

H5Wrapper::~H5Wrapper () {}

void H5Wrapper::check_h5_file(){
  // If current file is fully filled
  if(normals_counter_ == n_images_){
    save_normals_dataset();
    normals_accumulate_.clear(); 
    normals_counter_ = 0;
  }
  if(depth_counter_ == n_images_){
    save_depth_dataset();
    depth_accumulate_.clear(); 
    depth_counter_ = 0;
  }
  if(labels_counter_ == n_images_){
    save_labels_dataset();
    labels_accumulate_.clear(); 
    labels_counter_ = 0;
  }
  if(normals_counter_ == 0 && labels_counter_ == 0 && depth_counter_ == 0) {
    // H5F_ACC_TRUNC - will overwrite existing h5 file
    file.reset(new H5::H5File( file_pattern_to_filename(), H5F_ACC_TRUNC) );
    std::cout << "Created new file training_" << h5_counter_ << ".h5" << std::endl;
    h5_counter_++;
  }
}

void H5Wrapper::save_normals_dataset(){
  hsize_t fdim[] = {n_images_, 3, height_, width_}; // dim sizes of ds (on disk)
  H5::DataSpace fspace( 4, fdim );
  H5::DataSet ds_normals = file->createDataSet(
        "normals", H5::PredType::NATIVE_FLOAT, fspace);
  ds_normals.write(normals_accumulate_.data(), H5::PredType::NATIVE_FLOAT);
}
void H5Wrapper::save_depth_dataset(){
  hsize_t fdim[] = {n_images_, 1, height_, width_}; // dim sizes of ds (on disk)
  H5::DataSpace fspace( 4, fdim );
  H5::DataSet ds_normals = file->createDataSet(
        "depth", H5::PredType::NATIVE_FLOAT, fspace);
  ds_normals.write(depth_accumulate_.data(), H5::PredType::NATIVE_FLOAT);
}
void H5Wrapper::save_labels_dataset(){
  hsize_t fdim[] = {n_images_, 1, height_, width_}; // dim sizes of ds (on disk)
  H5::DataSpace fspace( 4, fdim );
  H5::DataSet ds_normals = file->createDataSet(
        "labels", H5::PredType::NATIVE_INT, fspace);
  ds_normals.write(labels_accumulate_.data(), H5::PredType::NATIVE_INT);
}
void H5Wrapper::add_normals(const std::vector<float> &normals){
  check_h5_file();
  normals_accumulate_.insert(normals_accumulate_.end(),normals.begin(),normals.end());
  normals_counter_++;
}
void H5Wrapper::add_depth(const std::vector<float> &depth){
  check_h5_file();
  depth_accumulate_.insert(depth_accumulate_.end(),depth.begin(),depth.end());
  depth_counter_++;
}
void H5Wrapper::add_labels(const std::vector<int> &labels){
  check_h5_file();
  labels_accumulate_.insert(labels_accumulate_.end(),labels.begin(),labels.end());
  labels_counter_++;
}

std::string H5Wrapper::file_pattern_to_filename() {
  return filename_pattern_+std::to_string(h5_counter_) + ".h5";
}
