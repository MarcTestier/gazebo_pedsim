# Utilities
A set of tools I use to help using the PedSim ROS Gazebo plugin.
Right now, only contains 1 Gazebo world plugin to help getting the position of default models in Gazebo.

## Installing
Install [ROS melodic](http://wiki.ros.org/melodic) and like any other ROS package, compile this package:
```
cd <path_to_ros_ws>
source /opt/ros/melodic/setup.bash
catkin_make
````

## How to use
To use this package, add it as a dependency in your `package.xml` and `CMakeLists.txt`. Then include the plugin into your world:
```
<plugin name="pos_plugin" filename="libpos_plugin.so"/>
```

You can check the [CMakeLists.txt](https://github.com/MarcTestier/gazebo_pedsim/blob/master/pedsim_ros_plugin/CMakeLists.txt) and [package.xml](https://github.com/MarcTestier/gazebo_pedsim/blob/master/pedsim_ros_plugin/package.xml) from the `pedsim_ros_plugin` folder as example.


This package contains 2 functions, the first one is triggered as soon as you include the plugin into your world and will simply output in your terminal the position of all the default unit objects in Gazebo.

The second one is a service to display points models in Gazebo of spawn points and waypoints taken from ROS parameters:
```
rosservice call /pos_plugin_ros_node/default/displayPointsModels "true"
```

## TODO
More utilities coming soon.
