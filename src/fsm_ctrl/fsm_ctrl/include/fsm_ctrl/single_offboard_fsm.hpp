#ifndef FSM_CTRL_SINGLE_OFFBOARD_FSM_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_FSM_HPP_

#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <fsm_ctrl/NMPC_Controller.hpp>

#endif  // FSM_CTRL_SINGLE_OFFBOARD_FSM_HPP_
