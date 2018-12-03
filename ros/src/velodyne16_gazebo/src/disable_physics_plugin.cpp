#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include <gazebo/common/Plugin.hh>
#include "gazebo/gazebo.hh"

namespace gazebo
{
class DisablePhysics : public WorldPlugin
{

public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  if (_parent->GetEnablePhysicsEngine())
    _parent->EnablePhysicsEngine(false);
}  
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(DisablePhysics)
}

