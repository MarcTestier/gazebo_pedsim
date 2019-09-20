cmake_minimum_required(VERSION 2.8.3)
project(utilities)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  std_srvs
  rosconsole 
  gazebo_ros 
)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp gazebo_ros std_msgs std_srvs rosconsole
  DEPENDS
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(pos_plugin src/pos_plugin.cpp)
target_link_libraries(pos_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
