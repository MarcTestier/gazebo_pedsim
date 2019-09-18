#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include "pedsim/ped_includes.h"


namespace gazebo
{
/**
 * Obstacles
 */
class Obstacles {

public:
    // TODO: clean or remove this constructor
    /**
     * Constructor
     */
    Obstacles();

    /**
     * Constructor
     * @param ros_node_ [description]
     * @param world_    [description]
     */
    Obstacles(std::shared_ptr<ros::NodeHandle> ros_node_, physics::WorldPtr world_);

    /**
     * Create an obstacle for the pedsim and for gazebo
     * @param name [description]
     * @param ax   [description]
     * @param ay   [description]
     * @param bx   [description]
     * @param by   [description]
     */
    void addObstacle(std::string name, double ax, double ay, double bx, double by);

    /**
     * Add all the obstacles to the ped scene, to be used after adding all the obstacles using the function addObstacle
     * @param ped_scene [description]
     */
    void finishObstacles(Ped::Tscene* ped_scene);

    /**
     * Clear all the obstacles from Gazebo, the ped scene will take care of clearing its own obstacles
     */
    void clear();

    /**
     * Create the Gazebo model of the given obstacle
     * @param name [description]
     * @param ax   [description]
     * @param ay   [description]
     * @param bx   [description]
     * @param by   [description]
     */
    void createObstacleModel(std::string name, double ax, double ay, double bx, double by);


private:
    physics::WorldPtr world;
    std::vector<Ped::Tobstacle*> obstacle_array;
    std::vector<std::string> obstacle_name_array;
    std::shared_ptr<ros::NodeHandle> ros_node;
};
} // namespace gazebo

#endif  // OBSTACLES_HPP
