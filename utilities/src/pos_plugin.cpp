#include "utilities/pos_plugin.hpp"

namespace gazebo
{
    PosPlugin::PosPlugin() : WorldPlugin()
    {
    }

    PosPlugin::~PosPlugin()
    {
    }

    void PosPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        this->world = _world;
        this->update_connection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&PosPlugin::OnUpdate, this)
        );
    }

    void PosPlugin::OnUpdate()
    {
        // Display the position of default models every 1 sec
        if (this->world->Iterations() % 1000 == 0) {
            physics::Model_V model_vector = this->world->Models();
            for (int i = 0; i < model_vector.size(); i++) {
                std::string name = model_vector[i]->GetName();
                ignition::math::Pose3d pose = model_vector[i]->WorldPose();
                if (name.find("unit") != std::string::npos) {
                    std::cout << "Position of " << name << " (" << pose.Pos().X() << ", "  << pose.Pos().Y() << ", " << pose.Pos().Z() << ")" << std::endl;
                }
            }
            std::cout << std::endl;
        }
    }
} // namespace gazebo
