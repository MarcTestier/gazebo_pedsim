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
    /**
     * Constructor
     */
    PedSimPlugin();

    /**
     * Destructor, delete the ped scene and its pointers on obstacles, waypoints and agents
     */
    ~PedSimPlugin();

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
    /**
     * Initialize the ROS node and the ROS services
     */
    void initROSNode();

    /**
     * Initialize the pedsim simulator
     */
    void initPedSim();

    /**
     * ROS service to initialize the pedsim simulator
     * @param  req [description]
     * @param  res [description]
     * @return     [description]
     */
    bool pedSimInitServiceCb(
        pedsim_ros_plugin::PedSimInit::Request &req,
        pedsim_ros_plugin::PedSimInit::Response &res
    );

    /**
     * ROS service to reset the pedsim simulator
     * @param  req [description]
     * @param  res [description]
     * @return     [description]
     */
    bool pedSimResetServiceCb(
        std_srvs::Empty::Request &req, 
        std_srvs::Empty::Response &res
    );

    /**
     * Clear the ped scene and its waypoints, agents and obstacles
     */
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
