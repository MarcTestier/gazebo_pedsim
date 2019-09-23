cmake_minimum_required(VERSION 2.8.3)
project(pedsim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS  roscpp
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/
  LIBRARIES pedsim
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/pedsim/
  ${roscpp_INCLUDE_DIRS}
)

set(SOURCES
  src/ped_agent.cpp
  src/ped_obstacle.cpp
  src/ped_outputwriter.cpp
  src/ped_scene.cpp
  src/ped_tree.cpp
  src/ped_vector.cpp
  src/ped_waypoint.cpp
)

add_library(pedsim ${SOURCES})

target_link_libraries(pedsim
  ${BOOST_LIBRARIES}
)
