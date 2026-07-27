cmake_minimum_required(VERSION 3.0.2)
project(fsm_ctrl)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  mavros_msgs
  std_msgs
  nav_msgs
  tf
  visualization_msgs
  ctrl_math
)
find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


generate_messages(
  DEPENDENCIES
  geometry_msgs
  mavros_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES fsm
#LIBRARIES motordriver
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

#add_executable(single_offboard src/single_offboard.cpp)
#target_link_libraries(single_offboard ${catkin_LIBRARIES} boost_system boost_filesystem)

add_library(dfbc
  src/dfbc.cpp
)



add_executable(single_offboard_fsm 
  src/single_offboard_fsm.cpp
)
target_link_libraries(single_offboard_fsm
  dfbc  ${catkin_LIBRARIES} boost_system boost_filesystem
)


add_executable(px4_estimator
  src/px4_estimator.cpp
)
target_link_libraries(px4_estimator
  ${catkin_LIBRARIES} boost_system boost_filesystem
)

add_executable(swarm_user_cmd
  src/swarm_user_cmd.cpp
)
target_link_libraries(swarm_user_cmd
  ${catkin_LIBRARIES} boost_system boost_filesystem
)

