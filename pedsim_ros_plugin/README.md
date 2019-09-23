# WORK IN PROGRESS
# PedSim Gazebo plugin
A Gazebo world plugin integrating PedSim into Gazebo 9.

## Installing
Install [ROS melodic](http://wiki.ros.org/melodic) and Gazebo 9 and compile this package:
```
cd <path_to_ros_ws>
source /opt/ros/melodic/setup.bash
catkin_make
````

## Parameters
This package uses parameters as defined in the `config/points.yaml` file.
This file is used to generate spawn points and waypoints for PedSim.
Each point is described as follow:
```
[string name, double pos_x, double pos_y, double pos_z ]
```

## How to use
### Launch the Gazebo simulation
2 launch files are provided as examples in this package, one to launch Gazebo with its client and one without the client.

Launch with the Gazebo client:
```
roslaunch pedsim_ros_plugin gui.launch
```
Launch without the Gazebo client (server only + rviz):
```
roslaunch pedsim_ros_plugin no_gui.launch
```

### Start PedSim
2 services are available to start the PedSim simulator:
```
rosservice call /pedsim_plugin_ros_node/default/pedSimInitService "{
    factor_social_force: 2.1,
    factor_obstacle_force: 3.0,
    factor_lookahead_force: 1.0,
    factor_desired_force: 1.0,
    agent_number: 200,
    agent_pos_pub_rate: 15,
    pedsim_update_rate: 20
}"
```
Or to remove it:
```
rosservice call /pedsim_plugin_ros_node/default/pedSimResetService "{}"
```

## Limitations
Soon

## TODO
- More parameters
- Link dynamically against libpedsim.so ??
- Check which license to use 
