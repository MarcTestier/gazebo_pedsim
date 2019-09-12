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

        this->InitPedSim();

        this->model = _world;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&PedSimPlugin::OnUpdate, this));
        
    }

    void PedSimPlugin::InitPedSim(){
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
            this->pedscene->addAgent(agent.get());
        }

        ROS_INFO_STREAM("PedSim initialized");
    }

    void PedSimPlugin::OnUpdate()
    {
        // Move all agents
        pedscene->moveAgents(0.3);

        for (int j = 0; j < this->agent_array.size(); j++) {
            Ped::Tvector pos = agent_array[j]->getPosition();
            ROS_INFO("Position of agent %d : (%f, %f, %f)", j, pos.x, pos.y, pos.z);
        }
    }

} // namespace gazebo