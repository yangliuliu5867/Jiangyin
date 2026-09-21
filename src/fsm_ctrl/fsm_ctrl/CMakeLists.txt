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
find_package(casadi CONFIG REQUIRED)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  mavros_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(single_offboard_fsm
  src/single_offboard_fsm.cpp
  src/NMPC_Controller.cpp
)
target_link_libraries(single_offboard_fsm
  ${catkin_LIBRARIES}
  casadi::casadi
)

add_executable(px4_estimator
  src/px4_estimator.cpp
)
target_link_libraries(px4_estimator
  ${catkin_LIBRARIES}
)

add_executable(swarm_user_cmd
  src/swarm_user_cmd.cpp
)
target_link_libraries(swarm_user_cmd
  ${catkin_LIBRARIES}
)
