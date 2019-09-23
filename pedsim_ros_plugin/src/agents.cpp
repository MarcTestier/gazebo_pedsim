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
        this->agent_pos_pub = this->ros_node->advertise<visualization_msgs::Marker>("pedsim_agent_pos", 0);
        
        if (ros::param::has("/pedsim_spawn_points")) {
            XmlRpc::XmlRpcValue spawn_points;
            ros::param::get("/pedsim_spawn_points", spawn_points);
            ROS_ASSERT(spawn_points.getType() == XmlRpc::XmlRpcValue::TypeArray);

            for (int32_t i = 0; i < spawn_points.size(); ++i) {
                ROS_ASSERT(spawn_points[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
                ROS_ASSERT(spawn_points[i].size() == 4);

                spawn_point_map.insert({
                    static_cast<std::string>(spawn_points[i][0]),
                    ignition::math::Vector3d(
                        static_cast<double>(spawn_points[i][1]),
                        static_cast<double>(spawn_points[i][2]),
                        static_cast<double>(spawn_points[i][3])
                    )
                });
                spawn_point_count_map.insert({static_cast<std::string>(spawn_points[i][0]), 0});
            }

            ROS_INFO_STREAM("Got " << spawn_points.size() << " spawn points, will be able to spawn up to " << spawn_points.size()*this->agents_per_sp << " agents");
        } else {
            ROS_WARN_STREAM("Didn't find any spawn points, agents won't be able to spawn");
        }
    }

    void Agents::addAgents(
        int agent_number,
        Ped::Tscene* ped_scene,
        std::vector<std::string> waypoint_name_array,
        Waypoints waypoints,
        float factor_social_force,
        float factor_obstacle_force,
        float factor_lookahead_force,
        float factor_desired_force,
        std::string spawn_point
    ) {
        ROS_INFO_STREAM("Create agents");
        int count = 0;
        for (int i = this->total_agent_number; i < this->total_agent_number + agent_number; i++) {

            ROS_INFO_STREAM("Creating agent " << i);
            std::string final_spawn_point = this->checkSpawnPoint(spawn_point);

            if (!final_spawn_point.empty()) {
                Ped::Tagent* agent = new Ped::Tagent();
                this->agent_array.push_back(agent);

                for (int j = 0; j < waypoint_name_array.size(); j++) {
                    agent->addWaypoint(waypoints.getWaypoint(waypoint_name_array[j]));
                }

                agent->setPosition(this->getSpawnPose(final_spawn_point));

                agent->setfactorsocialforce(factor_social_force);
                agent->setfactorobstacleforce(factor_obstacle_force);
                agent->setfactorlookaheadforce(factor_lookahead_force);
                agent->setfactordesiredforce(factor_desired_force);
                ROS_INFO_STREAM("Pedsim agent " << i << " pos: (" << agent->getPosition().x << ", " << agent->getPosition().y << ")");
                this->createAgentModel(i, agent->getPosition());
                ped_scene->addAgent(agent);
                count++;
            } else {
                ROS_WARN_STREAM("Can't spawn any more agents");
                break;
            }
        }

        this->total_agent_number += count;
    }

    void Agents::finishAgents() {
        // Wait for each entity to spawn to get their modelPtr to update later
        for (int i = 0; i < this->total_agent_number; i++) {
            while (!this->world->ModelByName("agent" + std::to_string(i)))
                common::Time::MSleep(10);

            this->agent_model_array.push_back(this->world->ModelByName("agent" + std::to_string(i)));
        }
    }

    void Agents::clear() {
        // No need to delete the Tagent* as they are being deleted by the Tscene
        this->agent_array.clear();

        ROS_INFO_STREAM("pedsimCleanup going to delete agent models");
        for (int i = 0; i < this->agent_model_array.size(); i++) {
            this->world->RemoveModel(this->agent_model_array[i]);
        }
        this->agent_model_array.clear();

        this->total_agent_number = 0;

        // Resetting the spawn point count
        for (auto& spawn_point_count : this->spawn_point_count_map) {
            spawn_point_count.second = 0;
        }
    }

    /// TODO: Add human model
    void Agents::createAgentModel(int i, Ped::Tvector pos)
    {
        ROS_INFO_STREAM("Creating model for agent " << i);
        sdf::SDF agentSDF;
        agentSDF.SetFromString(
        "<sdf version ='1.6'>\
            <model name ='agent" + std::to_string(i) + "'>\
                <static>true</static>\
                <pose>" + std::to_string(pos.x) + " " + std::to_string(pos.y) + " " + std::to_string(pos.z) + " 0 0 0</pose>\
                <link name ='link'>\
                    <pose>0 0 0 0 0 0</pose>\
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
                        <material>\
                            <ambient>0.1 0.1 0.1 1</ambient>\
                            <diffuse>0.1 0.7 0.1 1</diffuse>\
                            <specular>0 0 0 0</specular>\
                            <emissive>0 0 0 1</emissive>\
                        </material>\
                    </visual>\
                </link>\
            </model>\
            </sdf>");

        this->world->InsertModelSDF(agentSDF);
    }

    void Agents::updateModelPos() {
        for (int i = 0; i < this->agent_array.size(); i++) {
            if (this->agent_model_array.size() > i && this->agent_model_array[i]) {
                Ped::Tvector pos = this->agent_array[i]->getPosition();
                this->agent_model_array[i]->SetWorldPose(ignition::math::Pose3d(pos.x, pos.y, pos.z + 0.5, 0, 0, 0));
            }
        }
    }

    void Agents::publishRvizPos() {
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
        this->agent_pos_pub.publish(marker_msg);
    }

    visualization_msgs::Marker Agents::createAgentMarkerMsg() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time();
        marker.ns = "pedsim";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        return marker;
    }

    Ped::Tvector Agents::getSpawnPose(std::string spawn_point) {
        auto it1 = this->spawn_point_count_map.find(spawn_point);
        ROS_ASSERT(it1 != this->spawn_point_count_map.end());

        auto it2 = this->spawn_point_map.find(spawn_point);
        ROS_ASSERT(it2 != this->spawn_point_map.end());

        double posX = it2->second.X()+(it1->second%3-1);
        double posY = it2->second.Y()+(it1->second/3-1);
        double posZ = it2->second.Z();

        ROS_INFO_STREAM("Going to use spawn point " << spawn_point << " which has been used " << it1->second << " times and spawn in ("
        << it1->second%3-1 << ", "
        << it1->second/3-1 << ") + ("
        << it2->second.X() << ", "
        << it2->second.Y() << ") = ("
        << posX << ", "
        << posY << ")"
        );

        it1->second++;

        return Ped::Tvector(posX, posY, posZ);
    }

    std::string Agents::checkSpawnPoint(std::string spawn_point){
        if (spawn_point.empty()) {
            for (auto const& it : this->spawn_point_count_map) {
                if (it.second >= agents_per_sp) {
                    ROS_WARN_STREAM("Spawn point " << it.first << " is full, trying the next one");
                } else {
                    ROS_INFO_STREAM("Going to use spawn point " << it.first);
                    return it.first;
                }
            }
            ROS_WARN_STREAM("All the spawn points are full");
            spawn_point = "";
        } else {
            auto it = this->spawn_point_count_map.find(spawn_point);
            if (it != this->spawn_point_count_map.end()) {
                ROS_WARN_STREAM("Spawn point " << spawn_point << " could not be found");
                spawn_point = "";
            }
            if (it->second >= agents_per_sp) {
                ROS_WARN_STREAM("Spawn point " << spawn_point << " is full");
                spawn_point = "";
            }
        }

        return spawn_point;
    }
} // namespace gazebo