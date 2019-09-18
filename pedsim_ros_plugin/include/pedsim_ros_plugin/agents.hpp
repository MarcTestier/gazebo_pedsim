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
    Agents();
    Agents(std::shared_ptr<ros::NodeHandle> ros_node_, physics::WorldPtr world_);

    void addAgents(
        int agent_number,
        Ped::Tscene* ped_scene,
        std::vector<std::string> waypoint_name_array,
        Waypoints waypoints,
        float factor_social_force,
        float factor_obstacle_force,
        float factor_lookahead_force,
        float factor_desired_force);

    void updatePos();

    void publishPos();

    void finishAgents();

    void clear();

    std::vector<Ped::Tagent*> getAgentArray();

    void createAgentModel(int i, Ped::Tvector pos);

private:
    visualization_msgs::Marker createAgentMarkerMsg();

private:
    physics::WorldPtr world;
    std::vector<Ped::Tagent*> agent_array;
    std::vector<physics::ModelPtr> agent_model_array;
    std::shared_ptr<ros::NodeHandle> ros_node;
    int total_agent_number;
    ros::Publisher pedsim_pos_pub;
};
} // namespace gazebo

#endif  // AGENTS_HPP
