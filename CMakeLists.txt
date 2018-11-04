cmake_minimum_required(VERSION 2.8.3)
project(mavros_rover_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  ackermann_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

include(EnableCXX11)
include(MavrosMavlink)

catkin_package(
  CATKIN_DEPENDS mavros roscpp ackermann_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
)

add_library(mavros_rover_plugins
  src/plugins/ackermann_interface.cpp
)
target_link_libraries(mavros_rover_plugins
  ${mavros_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS mavros_rover_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  mavros_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)