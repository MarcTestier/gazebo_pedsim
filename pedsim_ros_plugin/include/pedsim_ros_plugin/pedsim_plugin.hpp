#ifndef PEDSIM_ROS_PLUGIN_HPP
#define PEDSIM_ROS_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>

#include "pedsim/ped_includes.h"

#include "pedsim_ros_plugin/PedSimInit.h"


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
    void initROSNode();

    void initPedSim();

    void createAgentModel(int i, Ped::Tvector pos);
    void createObstacleModel(std::string name, double ax, double ay, double bx, double by);

    bool PedSimInitService(
        pedsim_ros_plugin::PedSimInit::Request &req,
        pedsim_ros_plugin::PedSimInit::Response &res
    );

private:
    /// Gazebo variables
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;
    std::vector<physics::ModelPtr> agent_model_array;

    /// ROS variables
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::ServiceServer pedSimInitService;

    /// PedSim variables
    std::shared_ptr<Ped::Tscene> pedscene;
    std::shared_ptr<Ped::Twaypoint> w1;
    std::shared_ptr<Ped::Twaypoint> w2;
    std::shared_ptr<Ped::Tobstacle> obstacle;
    std::vector<std::shared_ptr<Ped::Tagent>> agent_array;

    /// Flags
    bool isPedSimInit;
};

GZ_REGISTER_WORLD_PLUGIN(PedSimPlugin)
} // namespace gazebo

#endif  // PEDSIM_ROS_PLUGIN_HPP




