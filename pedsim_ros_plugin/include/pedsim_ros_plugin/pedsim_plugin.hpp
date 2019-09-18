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

// ROS messages
#include "pedsim_ros_plugin/PedSimInit.h"
#include "std_srvs/Empty.h"

#include "pedsim_ros_plugin/waypoints.hpp"
#include "pedsim_ros_plugin/obstacles.hpp"
#include "pedsim_ros_plugin/agents.hpp"

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
    event::ConnectionPtr update_connection;

    /// ROS variables
    std::shared_ptr<ros::NodeHandle> ros_node;
    ros::ServiceServer pedsim_init_service;
    ros::ServiceServer pedsim_reset_service;
    int pub_rate;

    /// PedSim variables
    Ped::Tscene* ped_scene;
    float factor_social_force;
    float factor_obstacle_force;
    float factor_lookahead_force;
    float factor_desired_force;
    int agent_number;

    Agents agents;
    Waypoints waypoints;
    Obstacles obstacles;

    /// Flags
    bool is_pedsim_init;
    bool reset_pedsim;
};

GZ_REGISTER_WORLD_PLUGIN(PedSimPlugin)
} // namespace gazebo

#endif  // PEDSIM_ROS_PLUGIN_HPP
