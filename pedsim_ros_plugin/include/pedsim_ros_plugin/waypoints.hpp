#ifndef WAYPOINTS_HPP
#define WAYPOINTS_HPP

#include <ros/ros.h>
#include "pedsim/ped_includes.h"
#include "visualization_msgs/MarkerArray.h"


namespace gazebo
{
class Waypoints {

public:
    // TODO: clean or remove this constructor
    /**
     * Constructor
     */
    Waypoints();

    /**
     * Constructor
     * @param ros_node_ [description]
     */
    Waypoints(std::shared_ptr<ros::NodeHandle> node);

    /**
     * Create a pedsim waypoint
     * @param name   [description]
     * @param x      [description]
     * @param y      [description]
     * @param radius [description]
     */
    void addWaypoint(std::string name, double x, double y, double radius);

    /**
     * Clear the waypoint array, the ped scene will take care of clearing the waypoints
     */
    void clear();

    /**
     * Return the waypoint with the given name or NULL
     * @param  name [description]
     * @return      [description]
     */
    Ped::Twaypoint* getWaypoint(std::string name);

    /**
     * Publish the position of the waypoints in ROS as a Marker for Rviz to display
     */
    void publishRvizPos();

private:
    /**
     * Create the visualization_msgs::Marker message to be sent when publishing the position of the waypoints for Rviz
     * @return [description]
     */
    visualization_msgs::Marker createWaypointMarkerMsg();

private:
    std::map<std::string, Ped::Twaypoint*> waypoint_map;
    std::shared_ptr<ros::NodeHandle> ros_node;
    ros::Publisher waypoint_pos_pub;
};
}
#endif  // WAYPOINTS_HPP
