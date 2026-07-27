/*
 * @Author: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditors: yuzhe-yang chn.yuzhe.yang@gmail.com
 * @LastEditTime: 2026-02-04 19
 * @FilePath: /src/fsm_ctrl/fsm_ctrl/include/fsm_ctrl/single_offboard_fsm.hpp
 * @Description: 
 * 
 * Copyright (c) 2026 by yuzhe-yang, All Rights Reserved. 
 */
#ifndef _SINGLE_OFFBOARD_FSM_HPP_
#define _SINGLE_OFFBOARD_FSM_HPP_



#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <vector>
#include <deque>
#include <mavros_msgs/PositionTarget.h>
#include <tf/tf.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Int64.h"
#include <mavros_msgs/AttitudeTarget.h>
#include <fsm_ctrl/dfbc.hpp>
#include <visualization_msgs/Marker.h>
// SYX FSM TEST
// #include <traj_utils/Flag.h>
// #include <traj_utils/FlagState.h>
#include <ctrl_math/ctrl_math.hpp>
// SYX FSM TEST DONE
#include <nav_msgs/Path.h>




struct TypePoint
{
    int id;
    int mode;
    int is_map;
    double x,y,z;
    double yaw;
    int perc_mode;
};



#endif