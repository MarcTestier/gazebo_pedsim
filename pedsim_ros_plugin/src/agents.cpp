#include "pedsim_ros_plugin/agents.hpp"

namespace gazebo
{
    Agents::Agents()
    {
    }

    Agents::Agents(
        std::shared_ptr<ros::NodeHandle> ros_node_,
        physics::WorldPtr world_
    ):ros_node(ros_node_), world(world_), total_agent_number(0)
    {
        this->pedsim_pos_pub = this->ros_node->advertise<visualization_msgs::Marker>("pedsim_pos", 0);
    }

    void Agents::addAgents(
        int agent_number,
        Ped::Tscene* ped_scene,
        std::vector<std::string> waypoint_name_array,
        Waypoints waypoints,
        float factor_social_force,
        float factor_obstacle_force,
        float factor_lookahead_force,
        float factor_desired_force
    ) {
        ROS_INFO_STREAM("Create agents");

        for (int i = this->total_agent_number; i < this->total_agent_number + agent_number; i++) {
            Ped::Tagent* agent = new Ped::Tagent();
            this->agent_array.push_back(agent);

            for (int j = 0; j < waypoint_name_array.size(); j++) {
                agent->addWaypoint(waypoints.getWaypoint(waypoint_name_array[j]));
            }
            agent->setPosition(-50 + rand()/(RAND_MAX/80)-40, 0 + rand()/(RAND_MAX/20) -10, 0);

            agent->setfactorsocialforce(factor_social_force);
            agent->setfactorobstacleforce(factor_obstacle_force);
            agent->setfactorlookaheadforce(factor_lookahead_force);
            agent->setfactordesiredforce(factor_desired_force);
            this->createAgentModel(i, agent->getPosition());
            ped_scene->addAgent(agent);
        }

        this->total_agent_number += agent_number;
    }

    void Agents::finishAgents() {
        for (int i = 0; i < this->total_agent_number; i++) {
            // Wait for the entity to spawn
            while (!this->world->ModelByName("agent" + std::to_string(i)))
                common::Time::MSleep(10);

            this->agent_model_array.push_back(this->world->ModelByName("agent" + std::to_string(i)));
        }
    }

    void Agents::clear() {
        this->agent_array.clear();

        ROS_INFO_STREAM("pedsimCleanup going to delete agent models");
        for (int i = 0; i < this->agent_model_array.size(); i++) {
            this->world->RemoveModel(this->agent_model_array[i]);
        }
        this->agent_model_array.clear();

        this->total_agent_number = 0;
    }

    std::vector<Ped::Tagent*> Agents::getAgentArray() {
        return this->agent_array;
    }

    /// TODO: Add human model
    void Agents::createAgentModel(int i, Ped::Tvector pos)
    {
        ROS_INFO_STREAM("Creating model for agent" << i);
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
                            <cylinder>\
                                <radius>0.15</radius>\
                                <length>1</length>\
                            </cylinder>\
                        </geometry>\
                    </collision>\
                    <visual name='visual'>\
                        <geometry>\
                            <cylinder>\
                                <radius>0.15</radius>\
                                <length>1</length>\
                            </cylinder>\
                        </geometry>\
                    </visual>\
                </link>\
            </model>\
            </sdf>");

        this->world->InsertModelSDF(agentSDF);
    }

    void Agents::updatePos() {
        for (int i = 0; i < this->agent_array.size(); i++) {
            if (this->agent_model_array.size() > i && this->agent_model_array[i]) {
                Ped::Tvector pos = this->agent_array[i]->getPosition();
                this->agent_model_array[i]->SetWorldPose(ignition::math::Pose3d(pos.x, pos.y, pos.z, 0, 0, 0));
            }
        }
    }

    void Agents::publishPos() {
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

    visualization_msgs::Marker Agents::createAgentMarkerMsg() {
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