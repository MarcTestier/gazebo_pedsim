#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include "pedsim/ped_includes.h"


namespace gazebo
{
class Obstacles {

public:
    Obstacles();
    Obstacles(std::shared_ptr<ros::NodeHandle> node);

    void addObstacle(std::string name, double ax, double ay, double bx, double by);

    void clear();

    std::vector<Ped::Tobstacle*> getObstacleArray();

private:
    std::vector<Ped::Tobstacle*> obstacle_array;
    std::vector<physics::ModelPtr> obstacle_model_array;
    std::shared_ptr<ros::NodeHandle> ros_node;
};
} // namespace gazebo

#endif  // OBSTACLES_HPP
