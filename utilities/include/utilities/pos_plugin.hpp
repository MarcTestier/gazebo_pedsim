#ifndef POS_PLUGIN_HPP
#define POS_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
class PosPlugin : public WorldPlugin
{
public:
    /**
     * Constructor
     */
    PosPlugin();

    /**
     * Destructor, delete the ped scene and its pointers on obstacles, waypoints and agents
     */
    ~PosPlugin();

    /**
     * Called when Gazebo finish initializing
     * @param _world [description]
     * @param _sdf   [description]
     */
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /**
     * Called on every update step of the physics
     */
    void OnUpdate();

private:
    /// Gazebo variables
    physics::WorldPtr world;
    event::ConnectionPtr update_connection;
};

GZ_REGISTER_WORLD_PLUGIN(PosPlugin)
} // namespace gazebo

#endif  // POS_PLUGIN_HPP
