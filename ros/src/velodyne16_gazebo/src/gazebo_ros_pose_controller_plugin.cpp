#include "velodyne16_gazebo/gazebo_ros_pose_controller_plugin.h"

namespace gazebo
{

GazeboRosPoseController::GazeboRosPoseController(): joint_speed_(5.0), use_default_bbox_(false)
{
}


void GazeboRosPoseController::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  model_ = parent;
  if (!model_)
  {
    ROS_ERROR_STREAM("Invalid model pointer.");
    return;
  }

  gazebo_ros_ = GazeboRosPtr(new GazeboRos(model_, sdf, "PoseController"));
  sdf_ = sdf;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Get then name of the parent model and use it as node name
  std::string model_name = sdf->GetParent()->Get<std::string>("name");
  gzdbg << "Plugin model name: " << model_name << "\n";
  node_name_ = model_name;
  world_ = parent->GetWorld();

  // Initialize sensors
  if(initializeSensors() == false)
    return;

  // Initialize joint
  if(initializeJoint() == false)
    return;

  std::string world_name = "";
  gazebo_ros_->node()->getParam("/world_name", world_name);
  if (world_name != ""){
    std::string world_folder = world_name.substr(0, world_name.find_last_of("\\/"));
    std::string file_name = world_name.substr(world_name.find_last_of("\\/")+1, world_name.size());
    std::cout << file_name << std::endl;
    if (file_name == "laboratory.world") {
      use_default_bbox_ = true;
    }else{
      std::string bbox_name = world_folder + "/bbox_room.txt";
      if (!readBboxFile(bbox_name, bboxs_rooms_))
      {
        ROS_WARN("Couldn't read box file. Using default bbox for pose sampling.");
        use_default_bbox_ = true;
      }

      std::string bbox_objects_name = world_folder + "/bbox_objects.txt";
      if (!readBboxFile(bbox_objects_name, bboxs_objects_))
      {
        ROS_WARN("Couldn't read box file for objects. ");
      }
    }
  } else {
    ROS_WARN ("Couldn't get parameter 'world_name'. Using default bbox for pose sampling.");
    use_default_bbox_ = true;
  }

  // Create the service client for calling the assembler
  assemble_client_ = gazebo_ros_->node()->serviceClient<pcl_tools::AssembleClouds>("assemble_clouds");
  
  // Create a publisher for the joint state
  joint_state_pub_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1);
  
  samplePose();

  double angle;
  angle = -triangle(0.0)*diff_limits_ + offset_;
#if GAZEBO_MAJOR_VERSION >= 7
  joint_->SetPosition(0, angle);
#else
  joint_->SetAngle(0, angle);
#endif
  prev_time_ = world_->GetSimTime();
  collision_checked_ = false;
  velodyne_sensor_->SetActive(false);
  update_connection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&GazeboRosPoseController::OnUpdate, this));
}


void GazeboRosPoseController::OnUpdate()
{
  ros::spinOnce(); 

  common::Time curr_time = world_->GetSimTime();
  common::Time t_diff = curr_time - prev_time_;

  double rate = 1.0;
  double angle;

  // Publish joint state
  std::string base_frame = gazebo_ros_->resolveTF("base_footprint");
  joint_state_.header.stamp.sec = curr_time.sec;
  joint_state_.header.stamp.nsec = curr_time.nsec;
  joint_state_.header.frame_id = base_frame;
  joint_state_.position[0] = joint_->GetAngle(0).Radian();
  joint_state_pub_.publish(joint_state_);

  // Publish model transform
  std::string world_frame = gazebo_ros_->resolveTF("world");

  math::Pose curr_pose = model_->GetWorldPose();
  model_tf_.header.stamp = joint_state_.header.stamp;
  model_tf_.header.frame_id = world_frame;
  model_tf_.child_frame_id = base_frame;
  model_tf_.transform.translation.x = curr_pose.pos.x;
  model_tf_.transform.translation.y = curr_pose.pos.y;
  model_tf_.transform.translation.z = curr_pose.pos.z;
  model_tf_.transform.rotation.w = curr_pose.rot.w;
  model_tf_.transform.rotation.x = curr_pose.rot.x;
  model_tf_.transform.rotation.y = curr_pose.rot.y;
  model_tf_.transform.rotation.z = curr_pose.rot.z;
  tf_broadcaster_.sendTransform(model_tf_);

  if (collision_checked_)
  {
    
    if (t_diff.Double()/rate*joint_speed_ > 2.0)
    {
      sweep_begin_.sec = prev_time_.sec;
      sweep_begin_.nsec = prev_time_.nsec;
      sweep_end_.sec = curr_time.sec;
      sweep_end_.nsec = curr_time.nsec;

      pcl_tools::AssembleClouds srv;
      srv.request.begin = sweep_begin_;
      srv.request.end = sweep_end_ - ros::Duration(0.02);

      // Make the service call
      if (assemble_client_.call(srv))
      {
        std::cout << "response: " << (bool) srv.response.accepted << std::endl;
      }
      else
      {
        ROS_ERROR("Error making service call\n") ;
      }

      velodyne_sensor_->SetActive(false);
      contact_sensor_->SetActive(true);
      samplePose();
      collision_checked_ = false;
      prev_time_ = world_->GetSimTime();
      t_diff = 0.0;
    }

    angle = -triangle(t_diff.Double()/rate*joint_speed_)*diff_limits_ + offset_;
#if GAZEBO_MAJOR_VERSION >= 7
    joint_->SetPosition(0, angle);
#else
    joint_->SetAngle(0, angle);
#endif

  } else {
    prev_time_ = world_->GetSimTime();
    // Get all the contacts
    msgs::Contacts contacts;
#if GAZEBO_MAJOR_VERSION >= 7
    contacts = contact_sensor_->Contacts();
    if (contact_sensor_->LastMeasurementTime() == world_->GetSimTime())
#else
    contacts = contact_sensor_->GetContacts();
    if (contact_sensor_->GetLastMeasurementTime() == world_->GetSimTime())
#endif
    {
      if (contacts.contact_size() > 0 || poseCollidesWithObjectsBboxes(0.3)) {
        collision_checked_ = false;
        velodyne_sensor_->SetActive(false);
        samplePose();
        angle = -triangle(0.0)*diff_limits_ + offset_;
#if GAZEBO_MAJOR_VERSION >= 7
        joint_->SetPosition(0, angle);
#else
        joint_->SetAngle(0, angle);
#endif
      } else {
        collision_checked_ = true;
        velodyne_sensor_->SetActive(true);
        contact_sensor_->SetActive(false);
      }
    }
  }
}

bool GazeboRosPoseController::initializeJoint()
{
  if (sdf_->HasElement("joint_name")){
    joint_name_ = sdf_->GetElement("joint_name")->Get<std::string>();
  } else {
    ROS_ERROR_STREAM("Couldn't find joint in the model description." );
    return false;
  }
  
  joint_ = model_->GetJoint(joint_name_);
  if (!joint_)
  {
    ROS_ERROR_STREAM("Couldn't find the joint in the model.");
    return false;
  }

  joint_state_.header.frame_id = "Joint States";
  joint_state_.name.push_back(joint_name_);
  joint_state_.position.push_back(0);
  joint_state_.velocity.push_back(0);
  joint_state_.effort.push_back(0);

  math::Angle upper_limit = joint_->GetUpperLimit(0);
  math::Angle lower_limit = joint_->GetLowerLimit(0);
  diff_limits_ = (upper_limit.Radian() - lower_limit.Radian())/2;
  offset_ = upper_limit.Radian() - diff_limits_;

  if (sdf_->HasElement("speed")){
    joint_speed_ = sdf_->GetElement("speed")->Get<double>();
  } else {
    ROS_WARN_STREAM("speed of joint is not specified. Set to default: " <<  joint_speed_);
    joint_speed_ = 5.0;
  }
  joint_speed_ = joint_speed_/diff_limits_;
  return true;
}

bool GazeboRosPoseController::initializeSensors()
{
  // Load the contact sensor
  if (sdf_->HasElement("contact_sensor_name")){
    contact_sensor_name_ = sdf_->GetElement("contact_sensor_name")->Get<std::string>();
  } else {
    ROS_ERROR_STREAM("Couldn't find contact sensor in the model description." );
    return false;
  }

  sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor(contact_sensor_name_);
  if (!sensor)
  {
    ROS_ERROR_STREAM("Couldn't load the sensor: " << contact_sensor_name_);
    return false;
  }

#if GAZEBO_MAJOR_VERSION >= 7
  contact_sensor_ = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
#else
  contact_sensor_ = boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
#endif
  if (!contact_sensor_)
  {
    ROS_ERROR_STREAM("Sensor is NULL: " << contact_sensor_name_);
    return false;
  }
  contact_sensor_->SetActive(true);

  // Load the velodyne sensor
  if (sdf_->HasElement("velodyne_sensor_name")){
    velodyne_sensor_name_ = sdf_->GetElement("velodyne_sensor_name")->Get<std::string>();
  } else {
    ROS_ERROR_STREAM("Couldn't find velodyne laser sensor in the model description." );
    return false;
  }

  sensors::SensorPtr sensor_velodyne = sensors::SensorManager::Instance()->GetSensor(velodyne_sensor_name_);
  if (!sensor_velodyne)
  {
    ROS_ERROR_STREAM("Couldn't load the sensor: " << velodyne_sensor_name_);
    return false;
  }

#if GAZEBO_MAJOR_VERSION >= 7
  velodyne_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(sensor_velodyne);
#else
  velodyne_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(sensor_velodyne);
#endif
  if (!velodyne_sensor_)
  {
    ROS_ERROR_STREAM("Sensor is NULL: " << velodyne_sensor_name_);
    return false;
  }

  return true;
}

double GazeboRosPoseController::triangle(double x) {
  return fabs(fmod(x,4.f)-2)-1;
}

void GazeboRosPoseController::samplePose () {
  Eigen::Vector3f translation;
  translation = sampleTranslation(0.628, 0.02);

  model_pose_.pos.x = translation[0];
  model_pose_.pos.y = translation[1];
  model_pose_.pos.z = translation[2];

  // Sample rotation
  double roll = 0;//normalizeAngle(sampleRotation(-10,10));
  double pitch = 0;//normalizeAngle(sampleRotation(-10,10));
  double yaw = sampleRotation();
  model_pose_.rot.SetFromEuler(roll, pitch, yaw);
  model_->SetWorldPose(model_pose_);

}


Eigen::Vector3f GazeboRosPoseController::sampleTranslation(float offset_z, float stdz)
{
  Eigen::Vector3f translation;
  if (!use_default_bbox_)
  {
    float idx_bbox = randFloat(0, bboxs_rooms_.size()-1);
    translation[0] = randFloat(bboxs_rooms_[idx_bbox][0], bboxs_rooms_[idx_bbox][3]);
    translation[1] = randFloat(bboxs_rooms_[idx_bbox][1], bboxs_rooms_[idx_bbox][4]);
    translation[2] = bboxs_rooms_[idx_bbox][2] + offset_z + randFloatNormalDist(0.0, stdz);
  } else
  {
    translation[0] = randFloat(-22.64, 22.75);
    translation[1] = randFloat(-7.58, 7.63);
    translation[2] = offset_z + randFloatNormalDist(0.0, stdz);
  }

  return translation;
}

float GazeboRosPoseController::sampleRotation(float min, float max)
{
  float minrad, maxrad;
  minrad = min*M_PI/180.0;
  maxrad = max*M_PI/180.0;
  return randFloat(minrad, maxrad);
}

bool GazeboRosPoseController::readBboxFile (std::string& filename, std::vector<std::array<double, 6> > &bboxs_){
  std::ifstream bboxFile(filename.c_str());
  if(!bboxFile.is_open()){
    ROS_WARN( "Cannot open bbox file: %s", filename.c_str());
    return 0;
  }

  std::string line;
  while (std::getline(bboxFile, line)){
    std::istringstream iss(line);
    double x1, y1, z1, x2, y2, z2;
    if (!(iss >> x1 >> y1 >> z1 >> x2 >> y2 >> z2)) {
      ROS_ERROR( "Error reading lines from file: %s", filename.c_str());
      return 0;
    }
    std::array<double, 6> bbox = {x1, y1, z1, x2, y2, z2};
    reorderMinMax(bbox);
    bboxs_.push_back(bbox);
  }

  if (bboxs_.size() == 0){
    ROS_ERROR( "Error reading file: %s", filename.c_str());
    return 0;
  }

  return 1;
}


float GazeboRosPoseController::normalizeAngle(float angle)
{
  static float TwoPi = 2.0*M_PI;
  angle = std::fmod(angle, TwoPi);
  if(angle < 0.0){
    angle += TwoPi;   
  }
  return angle;
}

float GazeboRosPoseController::randFloat(float low, float high) {
  thread_local static std::mt19937 gen {std::random_device{}()};
  std::uniform_real_distribution <float> dist (low, high);
  return dist(gen);
}

float GazeboRosPoseController::randFloatNormalDist(float low, float high) {
  thread_local static std::mt19937 gen {std::random_device{}()};
  std::normal_distribution <float> dist (low, high);
  return dist(gen);
}

bool GazeboRosPoseController::poseIsInsideBbox(const math::Pose &p, const std::array<double, 6> &b, double offset){
  double minx = b[0] - offset;
  double maxx = b[3] + offset;

  double miny = b[1] - offset;
  double maxy = b[4] + offset;

  double minz = b[2] - offset;
  double maxz = b[5] + offset;

  return (p.pos.x >= minx && p.pos.x <= maxx &&
          p.pos.y >= miny && p.pos.y <= maxy &&
          p.pos.z >= minz && p.pos.z <= maxz );
}

bool GazeboRosPoseController::poseCollidesWithObjectsBboxes(double offset){
  for(const auto &bbox : bboxs_objects_){
    if( poseIsInsideBbox(model_pose_, bbox, offset) )
      return true;
  }
  return false;
}

void GazeboRosPoseController::reorderMinMax(std::array<double, 6> &b){
  if(b[0] > b[3]) std::swap(b[0],b[3]);
  if(b[1] > b[4]) std::swap(b[1],b[4]);
  if(b[2] > b[5]) std::swap(b[2],b[5]);
}


GZ_REGISTER_MODEL_PLUGIN(GazeboRosPoseController)
}
