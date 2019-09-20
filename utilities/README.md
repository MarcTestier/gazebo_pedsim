# PedSim Gazebo plugin
Soon

## Installing
Soon

## Parameters
Soon

## How to use
### To launch the Gazebo simulation
With the Gazebo client:
```
roslaunch pedsim_ros_plugin gui.launch
```
Without the Gazebo client (server only + rviz):
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
Soon




