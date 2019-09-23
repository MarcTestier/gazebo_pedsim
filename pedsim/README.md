# PedSim
Pedsim is an open source Pedestrian Crowd Simulation available [here](http://pedsim.silmaril.org/).

## Installing
Install [ROS melodic](http://wiki.ros.org/melodic) and compile this package:
```
cd <path_to_ros_ws>
source /opt/ros/melodic/setup.bash
catkin_make
````

## How to use
To use this package, add it as a dependency in your `package.xml` and `CMakeLists.txt`.

You can check the [CMakeLists.txt](https://github.com/MarcTestier/gazebo_pedsim/blob/master/pedsim_test/CMakeLists.txt) and [package.xml](https://github.com/MarcTestier/gazebo_pedsim/blob/master/pedsim_test/package.xml) from the `pedsim_test` folder as example.

## Limitations
To check.
