cmake_minimum_required(VERSION 2.8.3)
project(velodyne16_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_plugins
  tf
  cmake_modules
  geometry_msgs
  sensor_msgs
  laser_assembler
  pcl_ros
  pcl_tools
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)
find_package(HDF5 REQUIRED)
find_package(MPI REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Enable optimization for eigen
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse2 -march=native")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse2 -march=native")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_ros roscpp tf geometry_msgs sensor_msgs laser_assembler pcl_ros pcl_tools
  DEPENDS gazebo Eigen
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${HDF5_INCLUDE_DIR} ${MPI_INCLUDE_PATH})

add_library(disable_physics_plugin src/disable_physics_plugin.cpp)
target_link_libraries(disable_physics_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_pose_controller  src/gazebo_ros_pose_controller_plugin.cpp)
target_link_libraries(gazebo_ros_pose_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(scan_to_image_node src/scan_to_image_node.cpp src/scan_to_image.cpp src/h5wrapper.cpp)
target_link_libraries(scan_to_image_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES} hdf5 hdf5_cpp ${MPI_LIBRARIES})

add_executable(pcd_to_image_node src/pcd_to_image_node.cpp src/scan_to_image.cpp src/h5wrapper.cpp)
target_link_libraries(pcd_to_image_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES} hdf5 hdf5_cpp ${MPI_LIBRARIES})

add_executable(image_to_pcd_node src/image_to_pcd_node.cpp src/scan_to_image.cpp src/h5wrapper.cpp)
target_link_libraries(image_to_pcd_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES} hdf5 hdf5_cpp ${MPI_LIBRARIES})

#Installation 
install(TARGETS gazebo_ros_pose_controller disable_physics_plugin scan_to_image_node pcd_to_image_node image_to_pcd_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
