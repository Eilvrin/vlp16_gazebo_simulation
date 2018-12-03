#ifndef GAZEBO_ROS_POSE_CONTROLLER_H
#define GAZEBO_ROS_POSE_CONTROLLER_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_tools/AssembleClouds.h>

#include <cmath>
#include <cstring>
#include <string>
#include <limits>
#include <random>

#include <Eigen/Core>

namespace gazebo
{
class GazeboRosPoseController : public ModelPlugin
{
public:
  
  GazeboRosPoseController();
  ~GazeboRosPoseController(){};

  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  void OnUpdate();

private:
  bool initializeJoint();
  bool initializeSensors();
  double triangle(double x);
  Eigen::Vector3f sampleTranslation(float offset_z, float stdz);
  bool readBboxFile (std::string& filename, std::vector<std::array<double, 6> > &bboxs_);
  float sampleRotation(float min=0, float max=360);
  float normalizeAngle(float angle);
  float randFloat(float low, float high);
  float randFloatNormalDist(float low, float high);
  void samplePose ();

  bool poseIsInsideBbox(const math::Pose &p, const std::array<double, 6> &b, double offset);
  bool poseCollidesWithObjectsBboxes(double offset);
  // \brief Reorders min and max point such that
  // min is smaller than max for all dimensions
  // e.g. min.x < max.x && min.y < max.y && min.z < max.z
  void reorderMinMax(std::array<double, 6> &b);

  GazeboRosPtr gazebo_ros_;
  std::string node_name_;
  tf::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped model_tf_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  sdf::ElementPtr sdf_;

  // Joint
  physics::JointPtr joint_;
  std::string joint_name_;
  ros::Publisher joint_state_pub_;
  sensor_msgs::JointState joint_state_;

  // Contact
  std::string contact_sensor_name_;
  sensors::ContactSensorPtr contact_sensor_;

  // Velodyne
  std::string velodyne_sensor_name_;
  sensors::RaySensorPtr velodyne_sensor_;

  // Cloud assemble
  ros::Publisher assembled_cloud_pub_;
  ros::ServiceClient assemble_client_;

  event::ConnectionPtr update_connection_;
  common::Time prev_time_;

  double diff_limits_;  // [rad] Half of a difference between upper and lower angle limits 
  double offset_; // [rad] offset of angles

  double joint_speed_;

  math::Pose model_pose_;

  ros::Time sweep_begin_;
  ros::Time sweep_end_;

  bool collision_checked_;
  bool use_default_bbox_;
  std::vector<std::array<double, 6> > bboxs_rooms_;
  std::vector<std::array<double, 6> > bboxs_objects_;
};

} // namespace gazebo

#endif /* GAZEBO_ROS_POSE_CONTROLLER_H */