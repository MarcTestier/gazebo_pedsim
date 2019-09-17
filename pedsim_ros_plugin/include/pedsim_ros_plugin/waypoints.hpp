#ifndef WAYPOINTS_HPP
#define WAYPOINTS_HPP

#include <ros/ros.h>
#include "pedsim/ped_includes.h"


class Waypoints {

public:
    Waypoints();
    Waypoints(std::shared_ptr<ros::NodeHandle> node);

    void addWaypoint(std::string name, double x, double y, double radius);

    void clear();

    std::map<std::string, Ped::Twaypoint*> getWaypointMap();

    Ped::Twaypoint* getWaypoint(std::string name);

private:
    std::map<std::string, Ped::Twaypoint*> waypoint_map;
    std::shared_ptr<ros::NodeHandle> ros_node;
};

#endif  // WAYPOINTS_HPP
