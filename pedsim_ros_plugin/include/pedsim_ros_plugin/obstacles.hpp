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
    Obstacles(std::shared_ptr<ros::NodeHandle> ros_node_, physics::WorldPtr world_);

    void addObstacle(std::string name, double ax, double ay, double bx, double by);

    void finishObstacles(Ped::Tscene* ped_scene);

    void clear();

    std::vector<Ped::Tobstacle*> getObstacleArray();

    void createObstacleModel(std::string name, double ax, double ay, double bx, double by);


private:
    physics::WorldPtr world;
    std::vector<Ped::Tobstacle*> obstacle_array;
    std::vector<std::string> obstacle_name_array;
    std::shared_ptr<ros::NodeHandle> ros_node;
};
} // namespace gazebo

#endif  // OBSTACLES_HPP
