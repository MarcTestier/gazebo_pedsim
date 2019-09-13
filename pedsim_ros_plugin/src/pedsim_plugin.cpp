#include "pedsim_ros_plugin/pedsim_plugin.hpp"

namespace gazebo
{
    PedSimPlugin::PedSimPlugin() : WorldPlugin(), isPedSimInit(false)
    {
    }

    PedSimPlugin::~PedSimPlugin()
    {
        this->pedscene->clear();
        delete this->pedscene;
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
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&PedSimPlugin::OnUpdate, this));

        this->initROSNode();
    }

    void PedSimPlugin::initROSNode(){
        // Create ros node and publish stuff there!
        this->rosNode.reset(new ros::NodeHandle("pedsim_plugin_ros_node"));

        this->pedSimInitService = this->rosNode->advertiseService(this->world->Name() + "/pedSimInitService", &PedSimPlugin::pedSimInitServiceCb, this);

        this->pedSimResetService = this->rosNode->advertiseService(this->world->Name() + "/pedSimResetService", &PedSimPlugin::pedSimResetServiceCb, this);
    }

    void PedSimPlugin::initPedSim(){
        ROS_INFO_STREAM("Initializing PedSim");

        this->world->SetPaused(true);

        // Setup the scene
        ROS_INFO_STREAM("Create pedscene");
        this->pedscene = new Ped::Tscene(-200, -200, 400, 400);

        // Create waypoints
        ROS_INFO_STREAM("Create waypoints");
        Ped::Twaypoint* w1 = new Ped::Twaypoint(-100, 0, 24);
        Ped::Twaypoint* w2 = new Ped::Twaypoint(+100, 0, 12);

        // Create obstacles
        ROS_INFO_STREAM("Create obstacles");
        Ped::Tobstacle* obstacle = new Ped::Tobstacle(0, -5,  0, +5);
        this->createObstacleModel(0, 0, -5,  0.05, +5);
        this->pedscene->addObstacle(obstacle);

        // Create agents
        ROS_INFO_STREAM("Create agents");
        for (int i = 0; i < 5; i++) {
            Ped::Tagent* agent = new Ped::Tagent();
            this->agent_array.push_back(agent);
            agent->addWaypoint(w1);
            agent->addWaypoint(w2);
            agent->setPosition(-50 + rand()/(RAND_MAX/80)-40, 0 + rand()/(RAND_MAX/20) -10, 0);
            agent->setfactorsocialforce(2.0);
            agent->setfactorobstacleforce(2.0);
            //agent->setfactorlookaheadforce(5.0);
            this->createAgentModel(i, agent->getPosition());
            this->pedscene->addAgent(agent);
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


        // Wait for the entity to spawn
        while (!this->world->ModelByName("agent" + std::to_string(i)))
            common::Time::MSleep(10);

        agent_model_array.push_back(this->world->ModelByName("agent" + std::to_string(i)));
    }

    void PedSimPlugin::OnUpdate(){
        if (this->isPedSimInit) {
            // Move all agents
            this->pedscene->moveAgents(0.05);

            for (int j = 0; j < this->agent_array.size(); j++) {
                if (this->agent_model_array.size() > j && this->agent_model_array[j]) {
                    Ped::Tvector pos = this->agent_array[j]->getPosition();
                    this->agent_model_array[j]->SetWorldPose(ignition::math::Pose3d(pos.x, pos.y, pos.z, 0, 0, 0));
                }
            }

            if (this->world->Iterations() % 1000 == 0) {
                this->pedscene->cleanup();
            }
        }
    }

    // TODO: add params to init service to set number of agents, obstacles (?), waypoints, scene size and forces on agent
    bool PedSimPlugin::pedSimInitServiceCb(
        pedsim_ros_plugin::PedSimInit::Request &req,
        pedsim_ros_plugin::PedSimInit::Response &res)
    {
        ROS_INFO_STREAM("pedSimInitService called: " << req.data);
        if (!this->isPedSimInit) {
            this->initPedSim();
            this->isPedSimInit = true;
        }
        return true;
    }

    bool PedSimPlugin::pedSimResetServiceCb(
        std_srvs::Empty::Request &req, 
        std_srvs::Empty::Response &res)
    {
        ROS_INFO_STREAM("pedSimResetService called");

        this->world->SetPaused(true);

        this->pedsimCleanup();

        this->isPedSimInit = false;

        this->world->SetPaused(false);

        return true;
    }

    void PedSimPlugin::pedsimCleanup(){
        ROS_INFO_STREAM("pedsimCleanup agent size: " << this->agent_array.size());

        ROS_INFO_STREAM("pedsimCleanup going to delete agent models");
        for (int i = 0; i < this->pedscene->getAllAgents().size(); i++) {
            this->world->RemoveModel("agent" + std::to_string(i));
        }
        this->agent_model_array.clear();
        this->agent_array.clear();

        ROS_INFO_STREAM("pedsimCleanup going to delete obstacle models");
        for (int i = 0; i < this->pedscene->getAllObstacles().size(); i++) {
            this->world->RemoveModel("obs" + std::to_string(i));
        }

        ROS_INFO_STREAM("pedsimCleanup going to clear scene");
        this->pedscene->clear();
        delete this->pedscene;

        ROS_INFO_STREAM("pedsimCleanup agent size: " << this->agent_array.size());

        ROS_INFO_STREAM("pedsimCleanup finished cleanup");
    }

} // namespace gazebo