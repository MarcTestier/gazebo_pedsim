#ifndef AGENTS_HPP
#define AGENTS_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include "pedsim/ped_includes.h"
#include "visualization_msgs/Marker.h"

#include "pedsim_ros_plugin/waypoints.hpp"

namespace gazebo
{
class Agents {

public:
    // TODO: clean or remove this constructor
    /**
     * Constructor
     */
    Agents();

    /**
     * Constructor
     * @param ros_node_ [description]
     * @param world_    [description]
     */
    Agents(std::shared_ptr<ros::NodeHandle> ros_node_, physics::WorldPtr world_);

    /**
     * Create *agent_number* agents for the given *ped_scene* and in Gazebo.
     * The agents will follow the waypoints given in *waypoint_name_array* using the forces given in parameters
     * @param agent_number           [description]
     * @param ped_scene              [description]
     * @param waypoint_name_array    [description]
     * @param waypoints              [description]
     * @param factor_social_force    [description]
     * @param factor_obstacle_force  [description]
     * @param factor_lookahead_force [description]
     * @param factor_desired_force   [description]
     * @param spawn_point            [description]
     */
    void addAgents(
        int agent_number,
        Ped::Tscene* ped_scene,
        std::vector<std::string> waypoint_name_array,
        Waypoints waypoints,
        float factor_social_force,
        float factor_obstacle_force,
        float factor_lookahead_force,
        float factor_desired_force,
        std::string spawn_point);

    /**
     * Wait for all the Gazebo models to spawn and put them in a vector to make it easier to update their position later
     */
    void finishAgents();

    /**
     * Update the position of the Gazebo models based on the position in the ped scene
     */
    void updateModelPos();

    /**
     * Publish the position of the agents in ROS as a Marker for Rviz to display
     */
    void publishRvizPos();

    /**
     * Clear all the obstacles from Gazebo, the ped scene will take care of clearing its own obstacles
     */
    void clear();

    /**
     * Create the Gazebo model of the given agent
     * @param i   [description]
     * @param pos [description]
     */
    void createAgentModel(int i, Ped::Tvector pos);

private:
    /**
     * Create the visualization_msgs::Marker message to be sent when publishing the position of the agents for Rviz
     * @return [description]
     */
    visualization_msgs::Marker createAgentMarkerMsg();

    /**
     * @brief Get the spawn position for an agent
     * 
     * @param spawn_point 
     * @return Ped::Tvector 
     */
    Ped::Tvector getSpawnPose(std::string spawn_point);

    std::string checkSpawnPoint(std::string spawn_point);

private:
    physics::WorldPtr world;
    std::vector<Ped::Tagent*> agent_array;
    std::vector<physics::ModelPtr> agent_model_array;
    std::shared_ptr<ros::NodeHandle> ros_node;
    int total_agent_number;
    ros::Publisher agent_pos_pub;
    std::map<std::string, ignition::math::Vector3d> spawn_point_map;
    std::map<std::string, int> spawn_point_count_map;

    static int const agents_per_sp = 9;
};
} // namespace gazebo

#endif  // AGENTS_HPP
