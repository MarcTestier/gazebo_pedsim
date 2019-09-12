#ifndef PEDSIM_ROS_PLUGIN_HPP
#define PEDSIM_ROS_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>

#include "pedsim/ped_includes.h"


namespace gazebo
{
class PedSimPlugin : public WorldPlugin
{
public:
    PedSimPlugin();
    ~PedSimPlugin();

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    void OnUpdate();

private:
    void InitPedSim();

private:
    physics::WorldPtr model;
    event::ConnectionPtr updateConnection;

    /// PedSim variables
    std::shared_ptr<Ped::Tscene> pedscene;
    std::shared_ptr<Ped::Twaypoint> w1;
    std::shared_ptr<Ped::Twaypoint> w2;
    std::shared_ptr<Ped::Tobstacle> obstacle;
    std::vector<std::shared_ptr<Ped::Tagent>> agent_array;
};

GZ_REGISTER_WORLD_PLUGIN(PedSimPlugin)
} // namespace gazebo

#endif  // PEDSIM_ROS_PLUGIN_HPP




