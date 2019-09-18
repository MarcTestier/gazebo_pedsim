#include "pedsim_ros_plugin/waypoints.hpp"

namespace gazebo
{
    Waypoints::Waypoints()
    {
    }

    Waypoints::Waypoints(std::shared_ptr<ros::NodeHandle> node):ros_node(node)
    {
        this->waypoint_pos_pub = this->ros_node->advertise<visualization_msgs::MarkerArray>("pedsim_waypoint_pos", 0);
    }

    void Waypoints::addWaypoint(std::string name, double x, double y, double radius) {
        this->waypoint_map.insert({name, new Ped::Twaypoint(x, y, radius)});
    }

    void Waypoints::clear() {
        // No need to delete the Twaypoint* as they are being deleted by the Tscene
        this->waypoint_map.clear();
    }

    // TODO: use null_ptr ??
    // TODO: some exception stuff
    Ped::Twaypoint* Waypoints::getWaypoint(std::string name) { 
        auto search = waypoint_map.find(name);
        if (search != waypoint_map.end()) {
            return search->second;
        } else {
            ROS_WARN_STREAM("Waypoint " << name << " not found");
        }
        return NULL;
    }

    void Waypoints::publishRvizPos() {
        visualization_msgs::MarkerArray marker_array_msg;
        std::vector<visualization_msgs::Marker> marker_array;
        
        int i = 0;
        for (auto const& waypoint : this->waypoint_map) {
            visualization_msgs::Marker marker_msg = this->createWaypointMarkerMsg();

            marker_msg.id = i;
            i++;
            marker_msg.pose.position.x = waypoint.second->getx();
            marker_msg.pose.position.y = waypoint.second->gety();
            marker_msg.pose.position.z = 0;
            marker_msg.scale.x = waypoint.second->getr()*2;
            marker_msg.scale.y = waypoint.second->getr()*2;
            marker_msg.scale.z = 0.5;
            marker_array.push_back(marker_msg);
        }
        marker_array_msg.markers = marker_array;
        this->waypoint_pos_pub.publish(marker_array_msg);
    }

    visualization_msgs::Marker Waypoints::createWaypointMarkerMsg() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time();
        marker.ns = "pedsim";
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.a = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        return marker;
    }    
}
