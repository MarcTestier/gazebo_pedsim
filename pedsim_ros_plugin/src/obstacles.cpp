#include "pedsim_ros_plugin/obstacles.hpp"

namespace gazebo
{
    Obstacles::Obstacles()
    {
    }

    Obstacles::Obstacles(std::shared_ptr<ros::NodeHandle> ros_node_, physics::WorldPtr world_):ros_node(ros_node_), world(world_)
    {
    }

    void Obstacles::addObstacle(std::string name, double ax, double ay, double bx, double by) {
        this->obstacle_array.push_back(new Ped::Tobstacle(ax, ay, bx, by));
        this->createObstacleModel(name, ax, ay, bx, by);

        this->obstacle_name_array.push_back(name);
    }

    void Obstacles::finishObstacles(Ped::Tscene* ped_scene) {
        for (int i = 0; i < this->obstacle_array.size(); i++) {
            ped_scene->addObstacle(this->obstacle_array[i]);
        }
    }

    void Obstacles::clear() {
        // No need to delete the Tobstacle* as they are being deleted by the Tscene
        this->obstacle_array.clear();

        ROS_INFO_STREAM("pedsimCleanup going to delete obstacle models");
        for (int i = 0; i < this->obstacle_name_array.size(); i++) {
            this->world->RemoveModel(this->obstacle_name_array[i]);
        }
        this->obstacle_name_array.clear();
    }

    void Obstacles::createObstacleModel(std::string name, double ax, double ay, double bx, double by)
    {
        double posX = (ax+bx)/2;
        double posY = (ay+by)/2;

        float angle = std::atan2(ay - by, ax - bx);
        double length = std::sqrt(std::pow(bx-ax, 2) + std::pow(by-ay, 2));

        sdf::SDF obstacleSDF;
        obstacleSDF.SetFromString(
        "<sdf version ='1.6'>\
            <model name ='" + name + "'>\
                <static>true</static>\
                <pose>" + std::to_string(posX) + " " + std::to_string(posY) + " 0.5 0 0 " + std::to_string(angle) + "</pose>\
                <link name ='link'>\
                    <pose>0 0 0 0 0 0</pose>\
                    <collision name ='collision'>\
                        <geometry>\
                            <box><size>" + std::to_string(length) + " 0.1 1</size></box>\
                        </geometry>\
                    </collision>\
                    <visual name='visual'>\
                        <geometry>\
                            <box><size>" + std::to_string(length) + " 0.1 1</size></box>\
                        </geometry>\
                    </visual>\
                </link>\
            </model>\
            </sdf>");
        this->world->InsertModelSDF(obstacleSDF);
    }
} // namespace gazebo