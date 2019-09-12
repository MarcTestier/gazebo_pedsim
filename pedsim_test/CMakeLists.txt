cmake_minimum_required(VERSION 2.8.3)
project(pedsim_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs rosconsole pedsim
)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs rosconsole pedsim
  DEPENDS
)


include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(pedsim_test src/pedsim_test.cpp)
target_link_libraries(pedsim_test pedsim ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS pedsim_test
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
