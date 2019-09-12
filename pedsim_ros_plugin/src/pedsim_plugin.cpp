#include "pedsim_ros_plugin/pedsim_plugin.hpp"

namespace gazebo
{
    PedSimPlugin::PedSimPlugin() : WorldPlugin()
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

        this->initPedSim();
    }

    void PedSimPlugin::initPedSim(){
        ROS_INFO_STREAM("Initializing PedSim");
        // Setup
        this->pedscene = std::make_shared<Ped::Tscene>(-200, -200, 400, 400);
        this->w1 = std::make_shared<Ped::Twaypoint>(-100, 0, 24);
        this->w2 = std::make_shared<Ped::Twaypoint>(+100, 0, 12);
        this->obstacle = std::make_shared<Ped::Tobstacle>(0, -50,  0, +50);
        pedscene->addObstacle(obstacle.get());
        for (int i = 0; i < 10; i++) {
            std::shared_ptr<Ped::Tagent> agent = std::make_shared<Ped::Tagent>();
            agent_array.push_back(agent);
            agent->addWaypoint(w1.get());
            agent->addWaypoint(w2.get());
            agent->setPosition(-50 + rand()/(RAND_MAX/80)-40, 0 + rand()/(RAND_MAX/20) -10, 0);
            this->createAgentModel(i, agent->getPosition());
            this->pedscene->addAgent(agent.get());
        }

        ROS_INFO_STREAM("PedSim initialized");
    }

    void PedSimPlugin::createAgentModel(int i, Ped::Tvector pos){
        // Insert a sphere model from string
        sdf::SDF sphereSDF;
        sphereSDF.SetFromString(
        "<sdf version ='1.4'>\
            <model name ='agent'>\
                <pose>" + std::to_string(pos.x) + " " + std::to_string(pos.y) + " " + std::to_string(pos.z) + " 0 0 0</pose>\
                <link name ='link'>\
                <pose>0 0 .2 0 0 0</pose>\
                <collision name ='collision'>\
                    <geometry>\
                    <cylinder><radius>0.2</radius><length>1</length></cylinder>\
                    </geometry>\
                </collision>\
                <visual name='visual'>\
                    <geometry>\
                    <cylinder><radius>0.2</radius><length>1</length></cylinder>\
                    </geometry>\
                </visual>\
                </link>\
            </model>\
            </sdf>");
        // Demonstrate using a custom model name.
        sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
        model->GetAttribute("name")->SetFromString("agent " + std::to_string(i));
        this->world->InsertModelSDF(sphereSDF);
    }

    void PedSimPlugin::OnUpdate(){
        // Move all agents
        // pedscene->moveAgents(0.3);

        /*
        for (int j = 0; j < this->agent_array.size(); j++) {
            Ped::Tvector pos = agent_array[j]->getPosition();
            ROS_INFO("Position of agent %d : (%f, %f, %f)", j, pos.x, pos.y, pos.z);
        }
        */
    }

} // namespace gazebo