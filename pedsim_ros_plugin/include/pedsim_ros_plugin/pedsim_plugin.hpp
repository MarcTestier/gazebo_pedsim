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
#include "std_srvs/Empty.h"


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
    void createObstacleModel(int i, double ax, double ay, double bx, double by);

    bool pedSimInitServiceCb(
        pedsim_ros_plugin::PedSimInit::Request &req,
        pedsim_ros_plugin::PedSimInit::Response &res
    );

    bool pedSimResetServiceCb(
        std_srvs::Empty::Request &req, 
        std_srvs::Empty::Response &res
    );

    void pedsimCleanup();

private:
    /// Gazebo variables
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;
    std::vector<physics::ModelPtr> agent_model_array;

    /// ROS variables
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::ServiceServer pedSimInitService;
    ros::ServiceServer pedSimResetService;

    /// PedSim variables
    Ped::Tscene* pedscene;
    std::vector<Ped::Tagent*> agent_array;

    /// Flags
    bool isPedSimInit;
};

GZ_REGISTER_WORLD_PLUGIN(PedSimPlugin)
} // namespace gazebo

#endif  // PEDSIM_ROS_PLUGIN_HPP




