#include "pedsim_ros_plugin/pedsim_plugin.hpp"

// TODO: README
// TODO: Comments and auto documentation
namespace gazebo
{
    PedSimPlugin::PedSimPlugin() : WorldPlugin(), is_pedsim_init(false), factor_social_force(2.1), factor_obstacle_force(1.0), factor_lookahead_force(1.0), factor_desired_force(1.0), agent_number(10), reset_pedsim(false), pub_rate(15)
    {
    }

    PedSimPlugin::~PedSimPlugin()
    {
        if (this->is_pedsim_init) {
            this->ped_scene->clear();
            delete this->ped_scene;
        }
    }

    void PedSimPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        this->world = _world;
        this->update_connection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&PedSimPlugin::OnUpdate, this)
        );

        this->initROSNode();
        this->waypoints = Waypoints(this->ros_node);
    }

    void PedSimPlugin::OnUpdate()
    {
        if (this->is_pedsim_init && !this->reset_pedsim) {
            // Move all agents
            this->ped_scene->moveAgents(0.05);

            // Update the position of the models
            for (int i = 0; i < this->agent_array.size(); i++) {
                if (this->agent_model_array.size() > i && this->agent_model_array[i]) {
                    Ped::Tvector pos = this->agent_array[i]->getPosition();
                    this->agent_model_array[i]->SetWorldPose(ignition::math::Pose3d(pos.x, pos.y, pos.z, 0, 0, 0));
                }
            }

            // Publish the position of agents at a given rate
            if ((this->world->Iterations() % (int) std::round(1.0/this->world->Physics()->GetMaxStepSize()/this->pub_rate)) == 0) {
                visualization_msgs::Marker marker_msg = this->createAgentMarkerMsg();
                std::vector<geometry_msgs::Point> point_array;

                for (int i = 0; i < this->agent_array.size(); i++) {
                    if (this->agent_model_array.size() > i) {
                        Ped::Tvector pos = this->agent_array[i]->getPosition();
                        geometry_msgs::Point point_msg;
                        point_msg.x = pos.x;
                        point_msg.y = pos.y;
                        point_msg.z = pos.z;
                        point_array.push_back(point_msg);
                    }
                }
                marker_msg.points = point_array;
                this->pedsim_pos_pub.publish(marker_msg);
            }

            if (this->world->Iterations() % 1000 == 0) {
                this->ped_scene->cleanup();
            }
        }

        // Deleting models in the main world thread to avoid race conflicts leading to double free and corruption errors
        if (this->reset_pedsim) {
            this->pedsimCleanup();
        }
    }

    void PedSimPlugin::initROSNode()
    {
        // Create ros node and publish stuff there!
        this->ros_node.reset(new ros::NodeHandle("pedsim_plugin_ros_node"));

        this->pedsim_init_service = this->ros_node->advertiseService(
            this->world->Name() + "/pedSimInitService",
            &PedSimPlugin::pedSimInitServiceCb,
            this
        );

        this->pedsim_reset_service = this->ros_node->advertiseService(
            this->world->Name() + "/pedSimResetService",
            &PedSimPlugin::pedSimResetServiceCb,
            this
        );

        this->pedsim_pos_pub = this->ros_node->advertise<visualization_msgs::Marker>("pedsim_pos", 0);
    }

    void PedSimPlugin::initPedSim()
    {
        ROS_INFO_STREAM("Initializing PedSim");

        this->world->SetPaused(true);

        // Setup the scene
        ROS_INFO_STREAM("Create pedscene");
        this->ped_scene = new Ped::Tscene(-200, -200, 400, 400);

        // Create waypoints
        ROS_INFO_STREAM("Create waypoints");
        this->waypoints.addWaypoint("w1", -100, 0, 24);
        this->waypoints.addWaypoint("w2", +100, 0, 12);

        // Create obstacles
        ROS_INFO_STREAM("Create obstacles");
        Ped::Tobstacle* obstacle = new Ped::Tobstacle(0, -5,  0, +5);
        this->createObstacleModel(0, 0, -5,  0.05, +5);
        this->ped_scene->addObstacle(obstacle);

        // Wait for the entity to spawn
        while (!this->world->ModelByName("obstacle0"))
            common::Time::MSleep(10);

        this->obstacle_model_array.push_back(this->world->ModelByName("obstacle0"));

        // Create agents
        ROS_INFO_STREAM("Create agents");
        for (int i = 0; i < this->agent_number; i++) {
            Ped::Tagent* agent = new Ped::Tagent();
            this->agent_array.push_back(agent);
            agent->addWaypoint(this->waypoints.getWaypoint("w1"));
            agent->addWaypoint(this->waypoints.getWaypoint("w2"));
            agent->setPosition(-50 + rand()/(RAND_MAX/80)-40, 0 + rand()/(RAND_MAX/20) -10, 0);
            agent->setfactorsocialforce(this->factor_social_force);
            agent->setfactorobstacleforce(this->factor_obstacle_force);
            agent->setfactorlookaheadforce(this->factor_lookahead_force);
            agent->setfactordesiredforce(this->factor_desired_force);
            this->createAgentModel(i, agent->getPosition());
            this->ped_scene->addAgent(agent);
        }

        for (int i = 0; i < this->agent_number; i++) {
            // Wait for the entity to spawn
            while (!this->world->ModelByName("agent" + std::to_string(i)))
                common::Time::MSleep(10);

            this->agent_model_array.push_back(this->world->ModelByName("agent" + std::to_string(i)));
        }


        this->world->SetPaused(false);

        ROS_INFO_STREAM("PedSim initialized");
    }


    void PedSimPlugin::createObstacleModel(int i, double ax, double ay, double bx, double by)
    {
        // Insert a sphere model from string
        double posX = (ax+bx)/2;
        double posY = (ay+by)/2;

        float angle = std::atan2(ay - by, ax - bx);
        double length = std::sqrt(std::pow(bx-ax, 2) + std::pow(by-ay, 2));

        sdf::SDF obstacleSDF;
        obstacleSDF.SetFromString(
        "<sdf version ='1.6'>\
            <model name ='obstacle" + std::to_string(i) + "'>\
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

    /// TODO: Add human model
    void PedSimPlugin::createAgentModel(int i, Ped::Tvector pos)
    {
        ROS_INFO_STREAM("Creating agent" << i);
       // Insert a sphere model from string
        sdf::SDF agentSDF;
        agentSDF.SetFromString(
        "<sdf version ='1.6'>\
            <model name ='agent" + std::to_string(i) + "'>\
                <static>true</static>\
                <pose>" + std::to_string(pos.x) + " " + std::to_string(pos.y) + " " + std::to_string(pos.z) + " 0 0 0</pose>\
                <link name ='link'>\
                    <pose>0 0 .2 0 0 0</pose>\
                    <collision name ='collision'>\
                        <geometry>\
                            <cylinder><radius>0.15</radius><length>1</length></cylinder>\
                        </geometry>\
                    </collision>\
                    <visual name='visual'>\
                        <geometry>\
                            <cylinder><radius>0.15</radius><length>1</length></cylinder>\
                        </geometry>\
                    </visual>\
                </link>\
            </model>\
            </sdf>");

        this->world->InsertModelSDF(agentSDF);
    }

    // TODO: add params to init service to set obstacles (?), waypoints, scene size
    bool PedSimPlugin::pedSimInitServiceCb(
        pedsim_ros_plugin::PedSimInit::Request &req,
        pedsim_ros_plugin::PedSimInit::Response &res)
    {
        ROS_INFO_STREAM("pedSimInitService called\n" << req);
        if (!this->is_pedsim_init) {
            if (req.factor_social_force >= 0)
                this->factor_social_force = req.factor_social_force;

            if (req.factor_obstacle_force >= 0)
                this->factor_obstacle_force = req.factor_obstacle_force;

            if (req.factor_lookahead_force >= 0)
                this->factor_lookahead_force = req.factor_lookahead_force;

            if (req.factor_desired_force >= 0)
                this->factor_desired_force = req.factor_desired_force;

            if (req.agent_number > 0)
                this->agent_number = req.agent_number;

            if (req.pub_rate > 0)
                this->pub_rate = req.pub_rate;

            this->initPedSim();
            this->is_pedsim_init = true;
        } else {
            ROS_INFO_STREAM("Please, reset PedSim before initializing");
        }
        return true;
    }

    bool PedSimPlugin::pedSimResetServiceCb(
        std_srvs::Empty::Request &req, 
        std_srvs::Empty::Response &res)
    {
        ROS_INFO_STREAM("pedSimResetService called");

        if (this->is_pedsim_init) {
            this->is_pedsim_init = false;
            this->reset_pedsim = true;
        } else {
            ROS_INFO_STREAM("Can't reset PedSim as it wasn't initialized");
        }

        return true;
    }

    void PedSimPlugin::pedsimCleanup()
    {
        this->world->SetPaused(true);

        this->agent_array.clear();

        ROS_INFO_STREAM("pedsimCleanup going to clear scene");
        this->ped_scene->clear();
        if (this->ped_scene) {
            delete this->ped_scene;
        }


        ROS_INFO_STREAM("pedsimCleanup going to delete agent models");
        for (int i = 0; i < this->agent_model_array.size(); i++) {
            this->world->RemoveModel(this->agent_model_array[i]);
        }
        this->agent_model_array.clear();

        ROS_INFO_STREAM("pedsimCleanup going to delete obstacle models");
        for (int i = 0; i < this->obstacle_model_array.size(); i++) {
            this->world->RemoveModel(this->obstacle_model_array[i]);
        }
        this->obstacle_model_array.clear();
        //this->world->Clear();

        ROS_INFO_STREAM("pedsimCleanup finished cleanup");

        this->reset_pedsim = false;
        this->is_pedsim_init = false;
        this->world->SetPaused(false);
    }

    visualization_msgs::Marker PedSimPlugin::createAgentMarkerMsg() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time();
        marker.ns = "pedsim";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        return marker;
    }

} // namespace gazebo