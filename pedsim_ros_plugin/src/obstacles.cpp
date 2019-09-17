#include "pedsim_ros_plugin/obstacles.hpp"

namespace gazebo
{
    Obstacles::Obstacles()
    {
    }

    Obstacles::Obstacles(std::shared_ptr<ros::NodeHandle> node):ros_node(node)
    {
    }

    void Obstacles::addObstacle(std::string name, double ax, double ay, double bx, double by) {
        this->obstacle_array.push_back(new Ped::Tobstacle(ax, ay, bx, by));
    }

    void Obstacles::clear() {
        this->obstacle_array.clear();
    }

    std::vector<Ped::Tobstacle*> Obstacles::getObstacleArray() {
        return this->obstacle_array;
    }
} // namespace gazebo