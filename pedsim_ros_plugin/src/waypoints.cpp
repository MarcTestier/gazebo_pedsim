#include "pedsim_ros_plugin/waypoints.hpp"

Waypoints::Waypoints()
{
}

Waypoints::Waypoints(std::shared_ptr<ros::NodeHandle> node):ros_node(node)
{
}

void Waypoints::addWaypoint(std::string name, double x, double y, double radius) {
    this->waypoint_map.insert({name, new Ped::Twaypoint(x, y, radius)});
}

void Waypoints::clear() {
    this->waypoint_map.clear();
}

// TODO some exception stuff
Ped::Twaypoint* Waypoints::getWaypoint(std::string name) { 
    auto search = waypoint_map.find(name);
    if (search != waypoint_map.end()) {
        return search->second;
    } else {
        ROS_WARN_STREAM("Waypoint " << name << " not found");
    }
    return NULL;
}

std::map<std::string, Ped::Twaypoint*> Waypoints::getWaypointMap() {
    return this->waypoint_map;
}