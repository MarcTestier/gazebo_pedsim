#include "utilities/pos_plugin.hpp"

namespace gazebo
{
    PosPlugin::PosPlugin() : WorldPlugin(), spawn_models(false)
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

        // Create ros node and services
        this->ros_node.reset(new ros::NodeHandle("pos_plugin_ros_node"));

        // Create the service to initialize pedsim
        this->display_points_models_service = this->ros_node->advertiseService(
            this->world->Name() + "/displayPointsModels",
            &PosPlugin::displayPointsModelsServiceCb,
            this
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

        if (this->spawn_models) {
            this->spawnModels("pedsim_spawn_points", 1.0, 0.0, 0.0);
            this->spawnModels("pedsim_waypoints", 0.0, 0.0, 1.0);
            this->spawn_models = false;
        }

        if (this->delete_models) {
            this->deleteModels();
            this->delete_models = false;
        }
    }

    bool PosPlugin::displayPointsModelsServiceCb(
        std_srvs::SetBool::Request &req,
        std_srvs::SetBool::Response &res)
    {
        this->spawn_models = req.data;
        this->delete_models = !req.data;
        res.success = true;
        return true;
    }

    void PosPlugin::spawnModels(std::string param_name, float col_r, float col_g, float col_b)
    {
        ROS_INFO_STREAM("Going to spawn " << param_name << " models");
        if (ros::param::has(param_name)) {
            XmlRpc::XmlRpcValue param_points;
            ros::param::get(param_name, param_points);
            ROS_ASSERT(param_points.getType() == XmlRpc::XmlRpcValue::TypeArray);

            for (int32_t i = 0; i < param_points.size(); ++i) {
                ROS_ASSERT(param_points[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
                ROS_ASSERT(param_points[i].size() == 4);

                this->point_name_array.push_back(param_points[i][0]);

                this->createPointModel(param_points[i][0], param_points[i][1], param_points[i][2], param_points[i][3], col_r, col_g, col_b);
            }

            ROS_INFO_STREAM(param_points.size() << " " << param_name << " added");
        } else {
            ROS_WARN_STREAM("Didn't find any spawn points, agents won't be able to spawn");
        }
    }

    void PosPlugin::createPointModel(std::string name, double pos_x, double pos_y, double pos_z, double col_r, double col_g, double col_b)
    {
        ROS_INFO_STREAM("Creating model for point " << name);

        sdf::SDF agentSDF;
        agentSDF.SetFromString(
        "<sdf version ='1.6'>\
            <model name ='agent" + name + "'>\
                <static>true</static>\
                <pose>" + std::to_string(pos_x) + " " + std::to_string(pos_y) + " " + std::to_string(pos_z) + " 0 0 0</pose>\
                <link name ='link'>\
                    <pose>0 0 0 0 0 0</pose>\
                    <collision name ='collision'>\
                        <geometry>\
                            <sphere>\
                                <radius>0.5</radius>\
                            </sphere>\
                        </geometry>\
                    </collision>\
                    <visual name='visual'>\
                        <geometry>\
                            <sphere>\
                                <radius>0.5</radius>\
                            </sphere>\
                        </geometry>\
                        <material>\
                            <ambient>0.1 0.1 0.1 1</ambient>\
                            <diffuse>" + std::to_string(col_r) + " " + std::to_string(col_g) + " " + std::to_string(col_b) +" 1</diffuse>\
                            <specular>0 0 0 0</specular>\
                            <emissive>0 0 0 1</emissive>\
                        </material>\
                    </visual>\
                </link>\
            </model>\
            </sdf>");

        this->world->InsertModelSDF(agentSDF);
    }

    void PosPlugin::deleteModels()
    {
        ROS_INFO_STREAM("Going to delete points models");
        for (int i = 0; i < this->point_name_array.size(); i++) {
            this->world->RemoveModel(this->point_name_array[i]);
        }
        this->point_name_array.clear();
    }
} // namespace gazebo
