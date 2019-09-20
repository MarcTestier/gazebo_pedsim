cmake_minimum_required(VERSION 2.8.3)
project(pedsim_ros_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  std_srvs
  visualization_msgs
  rosconsole 
  gazebo_ros 
  pedsim
  message_generation
  utilities
)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

add_service_files(
  FILES
  PedSimInit.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp gazebo_ros std_msgs std_srvs visualization_msgs rosconsole pedsim message_runtime
  DEPENDS
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(agents src/agents.cpp)
target_link_libraries(agents ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(obstacles src/obstacles.cpp)
target_link_libraries(obstacles ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(waypoints src/waypoints.cpp)
target_link_libraries(waypoints ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(${PROJECT_NAME} src/pedsim_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} agents obstacles waypoints)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
