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
#include "std_srvs/SetBool.h"

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

    /**
     * @brief Display the points from the yaml files
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool displayPointsModelsServiceCb(
        std_srvs::SetBool::Request &req, 
        std_srvs::SetBool::Response &res
    );

    void spawnModels(std::string param_name, float col_r, float col_g, float col_b);

    void createPointModel(std::string name, double pos_x, double pos_y, double pos_z, double col_r, double col_g, double col_b);

    void deleteModels();

private:
    /// Gazebo variables
    physics::WorldPtr world;
    event::ConnectionPtr update_connection;
    std::vector<std::string> point_name_array;
    
    /// ROS variables
    std::shared_ptr<ros::NodeHandle> ros_node;
    ros::ServiceServer display_points_models_service;

    // Flags
    bool spawn_models;
    bool delete_models;
};

GZ_REGISTER_WORLD_PLUGIN(PosPlugin)
} // namespace gazebo

#endif  // POS_PLUGIN_HPP
