# PedSim
Some example of a ROS package using [pedsim](https://github.com/MarcTestier/gazebo_pedsim/tree/master/pedsim).

## Installing
Install [ROS melodic](http://wiki.ros.org/melodic) and compile this package:
```
cd <path_to_ros_ws>
source /opt/ros/melodic/setup.bash
catkin_make
````

## How to use
Simply source and use the roslaunch provided.
```
cd <path_to_ros_ws>
source devel/setup.bash
roslaunch pedsim_test pedsim_test.launch
```
