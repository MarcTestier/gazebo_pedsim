#include "pedsim_ros_plugin/pedsim_plugin.hpp"

// TODO: README
// TODO: Comments and auto documentation
namespace gazebo
{
    PedSimPlugin::PedSimPlugin() : WorldPlugin(), is_pedsim_init(false), factor_social_force(2.1), factor_obstacle_force(1.0), factor_lookahead_force(1.0), factor_desired_force(1.0), agent_number(10), reset_pedsim(false)
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
    }

    void PedSimPlugin::initROSNode(){
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
    }

    void PedSimPlugin::initPedSim(){
        ROS_INFO_STREAM("Initializing PedSim");

        this->world->SetPaused(true);

        // Setup the scene
        ROS_INFO_STREAM("Create pedscene");
        this->ped_scene = new Ped::Tscene(-200, -200, 400, 400);

        // Create waypoints
        ROS_INFO_STREAM("Create waypoints");
        Ped::Twaypoint* w1 = new Ped::Twaypoint(-100, 0, 24);
        Ped::Twaypoint* w2 = new Ped::Twaypoint(+100, 0, 12);

        // Create obstacles
        ROS_INFO_STREAM("Create obstacles");
        Ped::Tobstacle* obstacle = new Ped::Tobstacle(0, -5,  0, +5);
        this->createObstacleModel(0, 0, -5,  0.05, +5);
        this->ped_scene->addObstacle(obstacle);

        // Wait for the entity to spawn
        while (!this->world->ModelByName("obs0"))
            common::Time::MSleep(10);

        this->obs_model_array.push_back(this->world->ModelByName("obs0"));

        // Create agents
        ROS_INFO_STREAM("Create agents");
        for (int i = 0; i < this->agent_number; i++) {
            Ped::Tagent* agent = new Ped::Tagent();
            this->agent_array.push_back(agent);
            agent->addWaypoint(w1);
            agent->addWaypoint(w2);
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


    void PedSimPlugin::createObstacleModel(int i, double ax, double ay, double bx, double by){
        // Insert a sphere model from string
        double posX = (ax+bx)/2;
        double posY = (ay+by)/2;

        float angle = std::atan2(ay - by, ax - bx);
        double length = std::sqrt(std::pow(bx-ax, 2) + std::pow(by-ay, 2));

        sdf::SDF obsSDF;
        obsSDF.SetFromString(
        "<sdf version ='1.6'>\
            <model name ='obs" + std::to_string(i) + "'>\
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
        this->world->InsertModelSDF(obsSDF);
    }

    /// TODO: Add human model
    void PedSimPlugin::createAgentModel(int i, Ped::Tvector pos){
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

        /*
        // Wait for the entity to spawn
        while (!this->world->ModelByName("agent" + std::to_string(i)))
            common::Time::MSleep(10);

        agent_model_array.push_back(this->world->ModelByName("agent" + std::to_string(i)));
        */
    }

    void PedSimPlugin::OnUpdate(){
        if (this->is_pedsim_init && !this->reset_pedsim) {
            // Move all agents
            this->ped_scene->moveAgents(0.05);

            for (int j = 0; j < this->agent_array.size(); j++) {
                if (this->agent_model_array.size() > j && this->agent_model_array[j]) {
                    Ped::Tvector pos = this->agent_array[j]->getPosition();
                    this->agent_model_array[j]->SetWorldPose(ignition::math::Pose3d(pos.x, pos.y, pos.z, 0, 0, 0));
                }
            }

            if (this->world->Iterations() % 1000 == 0) {
                this->ped_scene->cleanup();
            }
        }

        // Deleting models here, in the main world thread to avoid race conflicts
        if (this->reset_pedsim) {
            this->pedsimCleanup();
        }
    }

    // TODO: add params to init service to set number of agents, obstacles (?), waypoints, scene size and forces on agent
    bool PedSimPlugin::pedSimInitServiceCb(
        pedsim_ros_plugin::PedSimInit::Request &req,
        pedsim_ros_plugin::PedSimInit::Response &res)
    {
        ROS_INFO_STREAM("pedSimInitService called\n" << req);
        if (!this->is_pedsim_init) {
            this->factor_social_force = req.factor_social_force;
            this->factor_obstacle_force = req.factor_obstacle_force;
            this->factor_lookahead_force = req.factor_lookahead_force;
            this->factor_desired_force = req.factor_desired_force;
            this->agent_number = req.agent_number;
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

    void PedSimPlugin::pedsimCleanup(){
        this->world->SetPaused(true);

        this->agent_array.clear();

        ROS_INFO_STREAM("pedsimCleanup going to clear scene");
        this->ped_scene->clear();
        delete this->ped_scene;


        ROS_INFO_STREAM("pedsimCleanup going to delete agent models");
        for (int i = 0; i < this->agent_model_array.size(); i++) {
            this->world->RemoveModel(this->agent_model_array[i]);
        }
        this->agent_model_array.clear();

        ROS_INFO_STREAM("pedsimCleanup going to delete obstacle models");
        for (int i = 0; i < this->obs_model_array.size(); i++) {
            this->world->RemoveModel(this->obs_model_array[i]);
        }
        this->obs_model_array.clear();
        //this->world->Clear();

        ROS_INFO_STREAM("pedsimCleanup finished cleanup");

        this->reset_pedsim = false;
        this->is_pedsim_init = false;
        this->world->SetPaused(false);
    }

} // namespace gazebo