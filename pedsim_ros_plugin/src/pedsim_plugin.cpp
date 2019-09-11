#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>

namespace gazebo
{
class PedSimPlugin : public WorldPlugin
{
public:
    PedSimPlugin() : WorldPlugin()
    {
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        ROS_INFO_STREAM("Hello World!");
        this->model = _world;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&PedSimPlugin::OnUpdate, this));
    }


    void OnUpdate()
    {
        ROS_INFO_STREAM("Hola!");
    }

private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
};
GZ_REGISTER_WORLD_PLUGIN(PedSimPlugin)
} // namespace gazebo