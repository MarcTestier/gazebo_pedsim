#include "pedsim_ros_plugin/pedsim_plugin.hpp"

namespace gazebo
{
    PedSimPlugin::PedSimPlugin() : WorldPlugin(), isPedSimInit(false)
    {
    }

    PedSimPlugin::~PedSimPlugin()
    {
        // Cleanup
        // for (Ped::Tagent* agent : pedscene->getAllAgents()) delete agent;
        // delete pedscene;
        // delete w1;
        // delete w2;
        // delete o;
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

        this->pedSimInitService = this->rosNode->advertiseService(this->world->Name() + "/pedSimInitService", &PedSimPlugin::PedSimInitService, this);
    }

    void PedSimPlugin::initPedSim(){
        ROS_INFO_STREAM("Initializing PedSim");

        this->world->SetPaused(true);

        // Setup
        this->pedscene = std::make_shared<Ped::Tscene>(-200, -200, 400, 400);
        this->w1 = std::make_shared<Ped::Twaypoint>(-100, 0, 24);
        this->w2 = std::make_shared<Ped::Twaypoint>(+100, 0, 12);
        this->obstacle = std::make_shared<Ped::Tobstacle>(0, -5,  0.05, +5);
        this->createObstacleModel("obs1", 0, -5,  0.05, +5);
        pedscene->addObstacle(obstacle.get());

        for (int i = 0; i < 20; i++) {
            std::shared_ptr<Ped::Tagent> agent = std::make_shared<Ped::Tagent>();
            agent_array.push_back(agent);
            agent->addWaypoint(w1.get());
            agent->addWaypoint(w2.get());
            agent->setPosition(-50 + rand()/(RAND_MAX/80)-40, 0 + rand()/(RAND_MAX/20) -10, 0);
            agent->setfactorsocialforce(2.0);
            agent->setfactorobstacleforce(2.0);
            //agent->setfactorlookaheadforce(5.0);
            this->createAgentModel(i, agent->getPosition());
            this->pedscene->addAgent(agent.get());
        }

        this->world->SetPaused(false);

        ROS_INFO_STREAM("PedSim initialized");
    }


    void PedSimPlugin::createObstacleModel(std::string name, double ax, double ay, double bx, double by){
       // Insert a sphere model from string
        sdf::SDF obsSDF;
        obsSDF.SetFromString(
        "<sdf version ='1.4'>\
            <model name ='" + name + "'>\
                <static>true</static>\
                <pose>" + std::to_string(ax+bx) + " " + std::to_string(ay+by) + " 0.5 0 0 0</pose>\
                <link name ='link'>\
                    <pose>0 0 0 0 0 0</pose>\
                    <collision name ='collision'>\
                        <geometry>\
                            <box><size>" + std::to_string((std::abs(ax)+std::abs(bx))) + " " + std::to_string((std::abs(ay)+std::abs(by))) + " 1</size></box>\
                        </geometry>\
                    </collision>\
                    <visual name='visual'>\
                        <geometry>\
                            <box><size>" + std::to_string((std::abs(ax)+std::abs(bx))) + " " + std::to_string((std::abs(ay)+std::abs(by))) + " 1</size></box>\
                        </geometry>\
                    </visual>\
                </link>\
            </model>\
            </sdf>");
        this->world->InsertModelSDF(obsSDF);
    }

    void PedSimPlugin::createAgentModel(int i, Ped::Tvector pos){
       // Insert a sphere model from string
        sdf::SDF agentSDF;
        agentSDF.SetFromString(
        "<sdf version ='1.4'>\
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
            common::Time::MSleep(100);

        agent_model_array.push_back(this->world->ModelByName("agent" + std::to_string(i)));
    }

    void PedSimPlugin::OnUpdate(){

        if (this->isPedSimInit) {
            // Move all agents
            pedscene->moveAgents(0.05);

            for (int j = 0; j < this->agent_array.size(); j++) {
                Ped::Tvector pos = agent_array[j]->getPosition();
                agent_model_array[j]->SetWorldPose(ignition::math::Pose3d(pos.x, pos.y, pos.z, 0, 0, 0));
            }
        }
    }

    // TODO: add reset service
    // TODO: add params to init service to set number of agents, obstacles (?), waypoints, scene size and forces on agent

    bool PedSimPlugin::PedSimInitService(
        pedsim_ros_plugin::PedSimInit::Request &req, pedsim_ros_plugin::PedSimInit::Response &res)
    {
        ROS_INFO_STREAM("PedSimInitService called: " << req.data);
        if (!this->isPedSimInit) {
            this->initPedSim();
            this->isPedSimInit = true;
        }
        return true;
    }

} // namespace gazebo