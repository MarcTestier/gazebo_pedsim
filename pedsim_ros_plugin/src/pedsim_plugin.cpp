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
        this->obstacles = Obstacles(this->ros_node, this->world);
        this->agents = Agents(this->ros_node, this->world);
    }

    void PedSimPlugin::OnUpdate()
    {
        if (this->is_pedsim_init && !this->reset_pedsim) {
            // Move all agents
            this->ped_scene->moveAgents(0.05);

            // Update the position of the models
            this->agents.updateModelPos();

            // TODO: add some param to enable/disable this
            // Publish the position of agents at a given rate
            if ((this->world->Iterations() % (int) std::round(1.0/this->world->Physics()->GetMaxStepSize()/this->pub_rate)) == 0) {
                this->agents.publishRvizPos();
            }

            // Do some cleanup of the pedscene
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
        // Create ros node and services
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
        this->obstacles.addObstacle("obstacle0", 0, -5,  0, +5);
        this->obstacles.finishObstacles(this->ped_scene);

        // Create agents
        this->agents.addAgents(
            this->agent_number,
            this->ped_scene,
            //std::vector<std::string>{"w1", "w2"},
            {"w1", "w2"},
            this->waypoints,
            this->factor_social_force,
            this->factor_obstacle_force,
            this->factor_lookahead_force,
            this->factor_desired_force
        );
        this->agents.finishAgents();


        this->world->SetPaused(false);

        ROS_INFO_STREAM("PedSim initialized");
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

        ROS_INFO_STREAM("pedsimCleanup going to clear scene");
        this->ped_scene->clear();
        if (this->ped_scene) {
            delete this->ped_scene;
        }

        this->agents.clear();
        this->waypoints.clear();   
        this->obstacles.clear();

        ROS_INFO_STREAM("pedsimCleanup finished cleanup");

        this->reset_pedsim = false;
        this->is_pedsim_init = false;
        this->world->SetPaused(false);
    }
} // namespace gazebo