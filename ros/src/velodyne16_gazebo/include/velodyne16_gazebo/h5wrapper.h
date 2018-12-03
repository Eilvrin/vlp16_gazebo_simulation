#pragma once
#include <H5Cpp.h>
#include <vector>
#include <string>
#include <iostream>
#include <memory>

/**
 * H5Wrapper class writes three datasets
 *  normals
 *  depth
 *  labels
 *
 */
 
class H5Wrapper {
public:
  H5Wrapper(); // default constructor
  H5Wrapper(const std::string filename_pattern, int width, int height, int number_of_files);
  // Copy constructors
  H5Wrapper(const H5Wrapper &obj) = default;
  // Copy assignments
  H5Wrapper& operator=(const H5Wrapper& obj) = default;
  ~H5Wrapper();
  
  void add_normals(const std::vector<float> &normals);
  void add_depth(const std::vector<float> &depth);
  void add_labels(const std::vector<int> &labels);

private:
  std::string filename_pattern_;
  hsize_t width_ = 0;
  hsize_t height_ = 0;
  hsize_t n_images_ = 0; //! total number of images in one h5 file
  int normals_counter_ = 0; //! counts how many images has been written
  int depth_counter_ = 0; //! counts how many images has been written
  int labels_counter_ = 0; //! counts how many images has been written
  int h5_counter_ = 0; //! counts how many h5 files has been written
  std::shared_ptr<H5::H5File> file;
  std::vector<float> normals_accumulate_; 
  std::vector<float> depth_accumulate_; 
  std::vector<int>   labels_accumulate_; 

  std::string file_pattern_to_filename(); //! produces filename out of filename pattern
  void check_h5_file(); // creates new h5 file if previous was filled
  void save_normals_dataset();
  void save_depth_dataset();
  void save_labels_dataset();
  
};
