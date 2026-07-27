/**
 * @file    single_offboard_fsm
 * @brief   finite state machine for single drone
 * @author  FLAG Lab, BIT
 * @version 4.0
 * @date    2024-06-03
 */

#include <fsm_ctrl/single_offboard_fsm.hpp>

using namespace std;

#define RATE 50.0
#define INTERV 1.0/RATE
#define HEIGHT 1.0 



/*--------------------------- FSM & Control ---------------------------*/

static bool apland = false;

static bool is_udp_enable = true;         // 1: enable receiving udp command  
static bool is_ready_fly = false;         // 0: wait for EKF pose fusion converging
static int cmd = 0;
  
static bool is_quat_init = false;         // 0: wait to initialize quaternion fusion
static bool is_need_rot = false;          // 1: initial qw < 0, need to rotate 360 (deg) on yaw axis

static bool is_land = false;            // 0: wait to land near ground 
static bool is_takeoff = false;         // 0: wait for RC confirming

static bool is_get_planner_msgs = false;

static int takeoff_channel = 0;
static int emergency_channel = 0;

mavros_msgs::State current_state;
static double voltage;

static Eigen::Vector3d pos_fcu;    
static Eigen::Vector3d vel_fcu;     
static Eigen::Quaterniond quat_fcu;
static Eigen::Vector3d euler_fcu;

static Eigen::Vector3d acc_imu;     
static Eigen::Quaterniond quat_imu;    
static Eigen::Vector3d euler_imu;
static Eigen::Vector3d rate_imu;

static Eigen::Vector3d pos_odom;
static Eigen::Vector3d vel_odom;
static Eigen::Quaterniond quat_odom;
static Eigen::Vector3d euler_odom;

geometry_msgs::PoseStamped aim_pos;          // position xyz + yaw, DO NOT use for attitude control
geometry_msgs::TwistStamped aim_vel;         // velocity xyz + yaw rate, DO NOT use for attitude control   
mavros_msgs::PositionTarget target_pos;         // position & velocity & acceleration + yaw & yaw rate
mavros_msgs::AttitudeTarget target_att;         // attitude & body rate + thrust
geometry_msgs::PoseStamped april_pos; 



static bool ap_land = false;
static bool dynamic_ring = false;
static bool set_dynamic_ring = false;
static int task1 =0;
static int traj_time;

/*----------LXK----------*/
static int task; 
//1密林，2集装箱，3迷宫，4静环，5动环，6动平台 
static int task_print_count = 0;
static int state2task[] = {1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5};
static int ego_end_lock = 0;
std_msgs::Int64 perc_mode_msg;
static int perc_mode;
static int perc_mode_print_count = 0;
static Eigen::Quaterniond quat_yaw;
static double yaw_now;
static int yaw_print_count = 0;

static vector<TypePoint> Ego_traj;
static int Ego_traj_count = 0;                     // 当前飞往点的标号
static int Ego_traj_size = 46;
static bool need_GeneTraj = true;                 // 是否需要修改轨迹

//存放历史一段时间内的位姿
static vector<vector<double>> fsm_pos;                  //存放历史一段时间内的位姿
static int fsm_pos_num = 200;                           //存放历史位姿的容量
ros::Time ini_time;                                     //位姿初始时间
static bool is_ini_time = false;                        //是否初始化位姿时间

//tunnel
static Eigen::Vector3d tunnel_pose1 = {16.60, -0.44, 1.2};
static vector<Eigen::Vector3d> tunnel_filter1;
static Eigen::Vector3d tunnel_pose2 = {22.33, 1.147, 1.2};
static vector<Eigen::Vector3d> tunnel_filter2;
static Eigen::Vector3d tunnel_pose3 = {19.43, 0.1, 1.95};
static vector<Eigen::Vector3d> tunnel_filter3;
static int tunnel_filter_size = 9;

//maze
static Eigen::Vector3d maze_pose1 = {19.11, 11.59, 1.8};
static vector<Eigen::Vector3d> maze_filter1;
static Eigen::Vector3d maze_pose2 = {21.78, 14.16, 2.1};
static vector<Eigen::Vector3d> maze_filter2;
static Eigen::Vector3d maze_pose3 = {18.77, 16.40, 1.8};
static vector<Eigen::Vector3d> maze_filter3;
static int maze_filter_size = 5;

//ring1
static Eigen::Vector3d ring1_pose = {3.3, -0.2, 1.7};
static vector<Eigen::Vector3d> ring1_filter;
static int ring1_filter_size = 5;
//ring3
static Eigen::Vector3d ring3_pose = {7.2, -0.8, 1.0};
static vector<Eigen::Vector3d> ring3_filter;
static int ring3_filter_size = 5;
//ring2
static Eigen::Vector3d ring2_pose = {2.0, 0.0, 1.0};   
static vector<Eigen::Vector3d> ring2_filter;
static int ring2_filter_size = 7;
static bool is_arrive_ring2 = false;
static bool is_crossed_ring2 = false;
static int ring2_cross_count = 0;
static bool is_send_r2_postpoint = false;
static bool is_arrive_r2_postpoint = false;
static int ring2_approach_count = 0;
static int r2_postpoint_lock = 0;
static int r2_print_count = 0;

static double ring_start_time = 0;                  // 圆环感知开始时间
static double ring_end_time = 0;                    // 圆环感知结束时间

//double ring
static double doublering1_start_time = 0; 
static double doublering2_start_time = 0; 
static double doublering1_end_time = 0; 
static double doublering2_end_time = 0; 

//ap pre
static Eigen::Vector3d dynringpost_pose = {3.0, 0.0, 1.00};
static Eigen::Vector3d appre_pose = {4.8, 0.8, 1.00};

static Eigen::Vector3d double_pose1 = {-1.737, 3.69, 1.50};
static vector<Eigen::Vector3d> dou1_filter;
static int dou1_filter_size = 5;
static Eigen::Vector3d double_pose2 = {-1.687, 5.3, 1.50};
static vector<Eigen::Vector3d> dou2_filter;
static int dou2_filter_size = 5;

static bool getring1 = false;
static bool getring2 = false;


static int count1 = 0;
static int count2 = 0;


//apriltag
static Eigen::Vector3d ap_pose = {5.0, 1.1, 1.00};
static vector<Eigen::Vector3d> ap_filter;
static vector<Eigen::Vector3d> ap_his_pose;
static int ap_filter_size = 5;
static double ap_start_time = 0;
static double ap_end_time = 0;
static bool is_found_ap = false;
static int need_change_ap_target = false;
static int search_orient = 0;
static int ap_print_count = 0;
static bool ap_search_lock = false;
static int ap_search_lock_count = 0;
static int landing_count = 0;
static int predict_orient;
geometry_msgs::PoseStamped ap_gl_msg;
static Eigen::Vector3d ap_gl_pos;
static bool is_arrive_ap_prepoint = false;
static bool is_send_ap_prepoint = false;
static int ap_prepoint_lock = 0;
static int ctl_land_count = 0;
static bool land_success = false;
/*----------LXK----------*/


/**
 * @brief  set aim position
 * @param  _x_y_z position
 * @return NONE
 */
void Set_AimPos(double _x, double _y, double _z)
{
    aim_pos.pose.position.x = _x;
    aim_pos.pose.position.y = _y;
    aim_pos.pose.position.z = _z;
}

/**
 * @brief  set aim position
 * @param  _x_y_z position
 * @param  _yaw euler angle
 * @return NONE
 */
void Set_AimPos(double _x, double _y, double _z, double _yaw)
{
    aim_pos.pose.position.x = _x;
    aim_pos.pose.position.y = _y;
    aim_pos.pose.position.z = _z;
    Eigen::Quaterniond quat = EulerToQuat(0.0, 0.0, _yaw);
    aim_pos.pose.orientation.w = quat.w();
    aim_pos.pose.orientation.x = quat.x();
    aim_pos.pose.orientation.y = quat.y();
    aim_pos.pose.orientation.z = quat.z();
}


/**
 * @brief  set aim velocity
 * @param  _x_y_z velocity
 * @return NONE
 */
void Set_AimVel(double _x, double _y, double _z)
{
    aim_vel.twist.linear.x = _x;
    aim_vel.twist.linear.y = _y;
    aim_vel.twist.linear.z = _z;
}

/**
 * @brief  set aim velocity
 * @param  _x_y_z velocity
 * @param  _yaw body rate
 * @return NONE
 */
void Set_AimVel(double _x, double _y, double _z, double _yaw)
{
    aim_vel.twist.linear.x = _x;
    aim_vel.twist.linear.y = _y;
    aim_vel.twist.linear.z = _z;
    aim_vel.twist.angular.z = _yaw;
}


/**
 * @brief  set target according to type mask
 * @param  _x_y_z position / velocity / acceleration
 * @param  _yaw euler angle / body rate
 * @return NONE
 * @note   DO NOT use position + velocity + acceleration or yaw + yaw_rate together
 */
void Set_TargetPosition(double _x, double _y, double _z, double _yaw)
{
    target_pos.type_mask = // mavros_msgs::PositionTarget::IGNORE_PX |
                           // mavros_msgs::PositionTarget::IGNORE_PY |
                           // mavros_msgs::PositionTarget::IGNORE_PZ |
                           mavros_msgs::PositionTarget::IGNORE_VX |
                           mavros_msgs::PositionTarget::IGNORE_VY |
                           mavros_msgs::PositionTarget::IGNORE_VZ |
                           mavros_msgs::PositionTarget::IGNORE_AFX |
                           mavros_msgs::PositionTarget::IGNORE_AFY |
                           mavros_msgs::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::PositionTarget::FORCE |
                           // mavros_msgs::PositionTarget::IGNORE_YAW |
                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    target_pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // target_pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    
    target_pos.position.x = _x;
    target_pos.position.y = _y;
    target_pos.position.z = _z;
    // target_pos.velocity.x = _x;
    // target_pos.velocity.y = _y;
    // target_pos.velocity.z = _z;
    // target_pos.acceleration_or_force.x = _x;
    // target_pos.acceleration_or_force.y = _y;
    // target_pos.acceleration_or_force.z = _z;
    target_pos.yaw = _yaw;
    // target_pos.yaw_rate = _yaw;
}


/**
 * @brief  subscriber callback
 * @param  msg from estimator
 * @return NONE
 */
void Ready_Callback(const std_msgs::Bool::ConstPtr &msg)
{
    is_ready_fly = msg->data;
}


/**
 * @brief  subscriber callback
 * @param  msg from mavros
 * @return NONE
 */
void Voltage_Callback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    voltage = msg->voltage;
}


/**
 * @brief  subscriber callback
 * @param  msg from mavros
 * @return NONE
 */
void State_Callback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}


/**
 * @brief  subscriber callback
 * @param  msg from mavros
 * @return NONE
 * @note   if initial w < 0, need to rotate yaw 360 deg
 */
void Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_fcu = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    quat_fcu = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    if(!is_quat_init)
    {
        is_quat_init = true;
        if(quat_fcu.w() < 0.0) {is_need_rot = true;}
        else {is_need_rot = false;}
    }
    if(is_need_rot) {quat_fcu = quat_fcu * Eigen::Quaterniond(-1.0, 0.0, 0.0, 0.0);}
    
    euler_fcu = QuatToEuler(quat_fcu);

    /*----------LXK----------*/
    if (!is_ini_time)
    {
        ini_time = msg->header.stamp;
        is_ini_time = true;
    }
    vector<double> fsm_pos_now(7);
    fsm_pos_now[0] = (msg->header.stamp-ini_time).toSec();//存放时间
    fsm_pos_now[1] = pos_fcu[0];
    fsm_pos_now[2] = pos_fcu[1];
    fsm_pos_now[3] = pos_fcu[2];
    fsm_pos_now[4] = euler_fcu[0];
    fsm_pos_now[5] = euler_fcu[1];
    fsm_pos_now[6] = euler_fcu[2];
    if (fsm_pos.size() < fsm_pos_num)
    {
        fsm_pos.push_back(fsm_pos_now);
    }
    else
    {
        for (int i=0;i<fsm_pos_num-1;i++)
        {
            fsm_pos[i] = fsm_pos[i+1];
        }
        fsm_pos.pop_back();
        fsm_pos.push_back(fsm_pos_now);
    }
    /*----------LXK----------*/
}


/**
 * @brief  subscriber callback
 * @param  msg from mavros
 * @return NONE
 */
void Vel_Callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_fcu = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
}


/**
 @brief  subscriber callback
 @param  msg from mavros
 @return NONE
 */
void IMU_Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    acc_imu = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    quat_imu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    euler_imu = QuatToEuler(quat_imu);
    rate_imu = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);   
}


/**
 @brief  subscriber callback
 @param  msg from mavros
 @return NONE
 */
void RC_Callback(const mavros_msgs::RCIn::ConstPtr &msg)
{
    takeoff_channel = msg->channels[8];
    emergency_channel = msg->channels[9];
    // cout << "takeoff_channel: " << takeoff_channel << endl;
}



/*--------------------------- Perception & Planning ---------------------------*/

CtrlPt traj_pt; // 规划轨迹


// SYX FSM TEST
// traj_utils::Flag flag_traj_msg;//发布至ego，飞行点信息
// traj_utils::Flag flag_state;//订阅至ego,运行状态信息反馈

static bool dynamic_ready = false;
/*----------LXK----------*/
template <typename T>
bool MedianFilter(vector<T> &_filter, T _data, int _size, T &result)
{
    if (_filter.size() < _size-1)
    {
        _filter.push_back(_data);
        return false;
    }
    else if (_filter.size() < _size)
    {
        _filter.push_back(_data);
    }
    else
    {
        for (int i = 0; i < _size - 1; i++)
        {
            _filter[i] = _filter[i + 1];
        }
        _filter.pop_back();
        _filter.push_back(_data);
    }

    vector<T> sort_filter = _filter;
    sort(sort_filter.begin(), sort_filter.end(),
        [](Eigen::Vector3d a, Eigen::Vector3d b)
        { return a.x() > b.x(); });
    result[0] = sort_filter[_size / 2][0];
    sort(sort_filter.begin(), sort_filter.end(),
        [](Eigen::Vector3d a, Eigen::Vector3d b)
        { return a.y() > b.y(); });
    result[1] = sort_filter[_size / 2][1];
    sort(sort_filter.begin(), sort_filter.end(),
        [](Eigen::Vector3d a, Eigen::Vector3d b)
        { return a.z() > b.z(); });
    result[2] = sort_filter[_size / 2][2];
    return true;
}

template <typename T>
bool MedAveFilter(vector<T> &_filter, T _data, int _size, T &result)
{
    if (_filter.size() < _size-1)
    {
        _filter.push_back(_data);
        return false;
    }
    else if (_filter.size() < _size)
    {
        _filter.push_back(_data);
    }
    else
    {
        for (int i = 0; i < _size - 1; i++)
        {
            _filter[i] = _filter[i + 1];
        }
        _filter.pop_back();
        _filter.push_back(_data);
    }

    vector<T> sort_filter = _filter;
    double sum0=0, sum1=0, sum2=0;
    sort(sort_filter.begin(), sort_filter.end(),
        [](Eigen::Vector3d a, Eigen::Vector3d b)
        { return a.x() > b.x(); });
    for (int i=1;i<sort_filter.size()-1;i++)
    {
        sum0+=sort_filter[i][0];
    }
    result[0] = sum0/(_size-2);

    sort(sort_filter.begin(), sort_filter.end(),
        [](Eigen::Vector3d a, Eigen::Vector3d b)
        { return a.y() > b.y(); });
    for (int i=1;i<sort_filter.size()-1;i++)
    {
        sum1+=sort_filter[i][1];
    }
    result[1] = sum1/(_size-2);

    sort(sort_filter.begin(), sort_filter.end(),
        [](Eigen::Vector3d a, Eigen::Vector3d b)
        { return a.z() > b.z(); });
    for (int i=1;i<sort_filter.size()-1;i++)
    {
        sum2+=sort_filter[i][2];
    }
    result[2] = sum2/(_size-2);
    return true;
}

Eigen::Matrix4d LocalToGlobal(double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
{
    Eigen::Matrix4d _LTG;
    _LTG(0, 0) = (cos(_pitch) * cos(_yaw));
    _LTG(0, 1) = (sin(_roll) * sin(_pitch) * cos(_yaw) - cos(_roll) * sin(_yaw));
    _LTG(0, 2) = (cos(_roll) * sin(_pitch) * cos(_yaw) + sin(_roll) * sin(_yaw));
    _LTG(1, 0) = (cos(_pitch) * sin(_yaw));
    _LTG(1, 1) = (sin(_roll) * sin(_pitch) * sin(_yaw) + cos(_roll) * cos(_yaw));
    _LTG(1, 2) = (cos(_roll) * sin(_pitch) * sin(_yaw) - sin(_roll) * cos(_yaw));
    _LTG(2, 0) = sin(_pitch);
    _LTG(2, 1) = sin(_roll) * cos(_pitch);
    _LTG(2, 2) = cos(_roll) * cos(_pitch);
    _LTG(0, 3) = _x;
    _LTG(1, 3) = _y;
    _LTG(2, 3) = _z;
    _LTG(3, 0) = 0;
    _LTG(3, 1) = 0;
    _LTG(3, 2) = 0;
    _LTG(3, 3) = 1;
    return _LTG;
}

bool JudgeDis(Eigen::Vector3d now, Eigen::Vector3d orin, double dis)
{
    if (fabs(now(0)-orin(0)) < dis && fabs(now(1)-orin(1)) < dis && fabs(now(2)-orin(2)) < dis)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool JudgeDis(Eigen::Vector3d now, Eigen::Vector3d orin, double _x, double _y, double _z)
{
    if (fabs(now(0)-orin(0)) < _x && fabs(now(1)-orin(1)) < _y && fabs(now(2)-orin(2)) < _z)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void PrintInfo(int &_count, int _size, string _info)
{
    if (_count < _size)
    {
        _count++;
    }
    else
    {
        cout << _info << endl;
        _count = 0;
    }
}

template<typename T>
void PrintInfo(int &_count, int _size, string _info, T data)
{
    if (_count < _size)
    {
        _count++;
    }
    else
    {
        cout << _info << data << endl;
        _count = 0;
    }
}






/**
 @brief  subscriber callback
 @param  msg from odometry
 @return NONE
 */
void Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    pos_odom = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    vel_odom = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    quat_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    euler_odom = QuatToEuler(quat_odom);
}



// void YawSmooth(double &yaw_now, double yaw_des)
// {
//     if (yaw_now < yaw_des - 0.02)
//     {
//         yaw_now += 0.01;
//     }
//     else if (yaw_now > yaw_des + 0.02)
//     {
//         yaw_now -= 0.01;
//     }
// }

void EgoAddPoint(int _id, int _mode, int _is_map, double _x, double _y, double _z, double _yaw, double _perc_mode)
{
    TypePoint pose;
    pose.id = _id;
    pose.mode = _mode;
    pose.is_map = _is_map;
    pose.x = _x;
    pose.y = _y;
    pose.z = _z;
    pose.yaw = _yaw;
    pose.perc_mode = _perc_mode;
    Ego_traj.push_back(pose);
}

// 存放所有的轨迹点，0为id，1为mode，2为map building, 3为x，4为y，5为z，6为yaw，7为感知模式
void EgoGeneTraj()
{
    Ego_traj.clear();
    // EgoAddPoint(0, 2, 0, 2.36, 0.23, 1.25, 0.0, 7);
    // EgoAddPoint(1, 3, 0, 3.35, 3.2, 1.25, 1.57, 7);
    // EgoAddPoint(2, 3, 0, 1.57, 4.47, 1.25, 3.13, 7);
    // EgoAddPoint(3, 3, 0, double_pose2(0)+0.8, double_pose2(1), 1.50, 3.13, 0);
    // EgoAddPoint(4, 3, 0, double_pose2(0), double_pose2(1), 1.50, 3.13, 0);
    // EgoAddPoint(5, 3, 0, double_pose2(0)-0.8, double_pose2(1), 1.50, 3.13, 0);
    // EgoAddPoint(6, 3, 0, double_pose1(0)-0.8, double_pose1(1), 1.50, 3.13, 0);
    // EgoAddPoint(7, 3, 0, double_pose1(0), double_pose1(1), 1.50, 3.13, 0);
    // EgoAddPoint(8, 3, 0, double_pose1(0)+1.8, double_pose1(1), 1.50, 3.13, 0);
    // EgoAddPoint(9, 3, 0, double_pose1(0)+1.8, double_pose1(1)-1, 0.5, 3.13, 0);


    
    EgoAddPoint(0, 2, 0, 1.35, 0.0, 1.08, 0.0, 3);
    EgoAddPoint(1, 3, 0, 3.2, 0.09, 1.85, 0.0, 0);
    EgoAddPoint(2, 3, 0, 5.3, 0.195, 1.85, 0.0, 0);
    EgoAddPoint(3, 3, 0, 6.28, 0.902, 1.85, 0.0, 0);
    EgoAddPoint(4, 3, 0, 7.16, 0.95, 1.85, 0.0, 0);
    EgoAddPoint(5, 3, 0, 7.68, 0.16, 1.85, 0.0, 0);
    EgoAddPoint(6, 3, 0, 9.15, 0.35, 1.85, 0.0, 0);
    EgoAddPoint(7, 3, 0, 10.32, 0.53, 1.85, 0.0, 0);
    EgoAddPoint(8, 3, 0, 12.32, 0.53, 1.85, 0.0, 2);
    EgoAddPoint(9, 3, 0, 12.89, 0.05, 1.85, 0.0, 2);
    EgoAddPoint(10, 3, 0, 13.12, -0.48, 1.85, 0.0, 2);
    EgoAddPoint(11, 3, 0, tunnel_pose1(0)-0.8, tunnel_pose1(1), 1.75, 0.0, 0);
    EgoAddPoint(12, 3, 0, tunnel_pose1(0), tunnel_pose1(1), 1.75, 0.0, 0);
    EgoAddPoint(13, 3, 0, tunnel_pose1(0)+0.8, tunnel_pose1(1), 1.85, 0.0, 0);
    EgoAddPoint(14, 3, 0, tunnel_pose3(0)-0.5, tunnel_pose3(1), 1.85, 0.0, 0);
    EgoAddPoint(15, 3, 0, tunnel_pose3(0), tunnel_pose3(1), 1.85, 0.0, 0);
    EgoAddPoint(16, 3, 0, tunnel_pose3(0)+0.5, tunnel_pose3(1), 1.85, 0.0, 0);
    EgoAddPoint(17, 3, 0, tunnel_pose2(0)-1.0, tunnel_pose2(1), 1.85, 0.0, 0);
    EgoAddPoint(18, 3, 0, tunnel_pose2(0), tunnel_pose2(1), 1.85, 0.0, 0);
    EgoAddPoint(19, 3, 0, tunnel_pose2(0)+1.0, tunnel_pose2(1), 1.85, 0.0, 0);
    EgoAddPoint(20, 3, 0, 24.65, 0.85, 1.8, 0.0, 0);
    EgoAddPoint(21, 3, 0, 20.80, 7.55, 2.0, 1.57, 0);
    EgoAddPoint(22, 3, 0, 19.85, 8.34, 2.0, 1.57, 3);
    EgoAddPoint(23, 2, 0, maze_pose1(0), maze_pose1(1)-0.8, 2.0, 1.57, 0);
    EgoAddPoint(24, 2, 0, maze_pose1(0), maze_pose1(1), 2.0, 1.57, 0);
    EgoAddPoint(25, 2, 0, maze_pose1(0), maze_pose1(1)+0.8, 2.0, 1.57, 0);
    EgoAddPoint(26, 2, 0, maze_pose2(0), maze_pose2(1)-0.8, 2.0, 1.57, 0);
    EgoAddPoint(27, 2, 0, maze_pose2(0), maze_pose2(1), 2.0, 1.57, 0);
    EgoAddPoint(28, 2, 0, maze_pose2(0), maze_pose2(1)+0.8, 2.0, 1.57, 0);
    EgoAddPoint(29, 2, 0, maze_pose3(0), maze_pose3(1)-0.8, 2.0, 1.57, 0);
    EgoAddPoint(30, 2, 0, maze_pose3(0), maze_pose3(1), 2.0, 1.57, 0);
    EgoAddPoint(31, 2, 0, maze_pose3(0), maze_pose3(1)+0.8, 2.0, 1.57, 0);
    EgoAddPoint(32, 3, 0, 19.00, 18.00, 1.8, 1.57, 0);
    EgoAddPoint(33, 3, 0, 14.80, 15.42, 1.0, 3.13, 0);
    EgoAddPoint(34, 2, 0, 2.479, 0.0, 1.0, 0.0, 7);
    EgoAddPoint(34, 3, 0, 3.26, 4.0, 1.65, 1.57, 7);
    EgoAddPoint(36, 3, 0, 1.12, 4.37, 1.65, 3.13, 7);
    EgoAddPoint(37, 3, 0, double_pose2(0)+0.8, double_pose2(1), 1.65, 3.13, 0);
    EgoAddPoint(38, 3, 0, double_pose2(0), double_pose2(1), 1.65, 3.13, 0);
    EgoAddPoint(39, 3, 0, double_pose2(0)-0.8, double_pose2(1), 1.65, 3.13, 0);
    EgoAddPoint(40, 3, 0, double_pose1(0)-0.8, double_pose1(1), 1.65, 3.13, 0);
    EgoAddPoint(41, 3, 0, double_pose1(0), double_pose1(1), 1.65, 3.13, 0);
    EgoAddPoint(42, 3, 0, double_pose1(0)+1.8, double_pose1(1), 1.65, 3.13, 0);
    EgoAddPoint(43, 3, 0, double_pose1(0)+1.8, double_pose1(1)-1, 1.65, 3.13, 0);
    EgoAddPoint(44, 3, 0, -4.3, 2.12, 1.65, 3.13, 5);
    EgoAddPoint(45, 3, 0, ring2_pose(0)+1,ring2_pose(1), 1.65, 3.13, 5);
    
    // EgoAddPoint(8, 3, 0, double_pose2(0)+0.8, double_pose2(1), 1.65, 0.0, 0);
    // EgoAddPoint(9, 3, 0, double_pose2(0)-0.8, double_pose2(1), 1.65, 0.0, 0);
    // EgoAddPoint(9, 3, 0, double_pose2(0)+0.8, double_pose2(1), 1.65, 0.0, 0);
    // EgoAddPoint(10, 2, 0, ring2_pose(0)-1,ring2_pose(1), ring2_pose(2), 0.0, 5);
    // EgoAddPoint(2, 3, 0, 10.9, -0.12, 1.2, 0.0, 5);
    // EgoAddPoint(3, 3, 0, 11.2, -1.1, 1.2, -3.13, 5);
    // EgoAddPoint(4, 3, 0, 9.66, -1.35, 1.4, -3.13, 5);
    // EgoAddPoint(5, 3, 0, 8.77, -1.35, 1.4, -3.13, 5);
    // EgoAddPoint(6, 3, 0, 7.9, -4.7, 1.3, -3.13, 5);
    // EgoAddPoint(7, 3, 0, double_pose1(0)+1, double_pose1(1), 1.65, -3.13, 5);
    // EgoAddPoint(8, 3, 0, double_pose1(0)-1, double_pose1(1), 1.65, -3.13, 5);
    // EgoAddPoint(9, 3, 0, double_pose2(0)-1, double_pose2(1), 1.65, -3.13, 5);
    // EgoAddPoint(10, 3, 0, double_pose2(0)+1, double_pose2(1), 1.65, -3.13, 5);
    // EgoAddPoint(11, 3, 0, 7.49, -5.48, 0.5, -3.13, 5);

    // EgoAddPoint(0, 2, 0, 3, 0, 0.5, 0.0, 5);
    // EgoAddPoint(1, 2, 0, double_pose1(0)-0.5, double_pose1(1), double_pose1(2), 0.0, 5);
    // EgoAddPoint(2, 2, 0, double_pose1(0)+0.5, double_pose1(1), double_pose1(2), 0.0, 5);
    // EgoAddPoint(3, 2, 0, double_pose2(0)+0.5, double_pose2(1), double_pose2(2), 0.0, 5);  //forest
    // EgoAddPoint(4, 2, 0, double_pose2(0)-0.5, double_pose2(1), double_pose2(2), 0.0, 5);
    // EgoAddPoint(4, 2, 0, tunnel_pose1(0), tunnel_pose1(1), tunnel_pose1(2), 0.0, 5);
    // EgoAddPoint(5, 2, 0, tunnel_pose2(0), tunnel_pose2(1), tunnel_pose2(2), 0.0, 4);
    // EgoAddPoint(6, 2, 0, 6.41, -0.75, 1.44, 0.0, 5);  //tunnel
    // EgoAddPoint(7, 2, 0, 6.41, -0.75, 1.44, 0.0, 5);
    // EgoAddPoint(8, 2, 0, maze_pose1(0), maze_pose1(1), maze_pose1(2), 0.0, 6);
    // EgoAddPoint(9, 2, 0, maze_pose2(0)+1, maze_pose2(1), maze_pose2(2), 1.57, 6);
    // EgoAddPoint(10, 2, 0, maze_pose3(0)+1, maze_pose3(1)+1, maze_pose3(2), 3.14, 6);
    // EgoAddPoint(11, 2, 0, 6.41, -0.75, 1.44, 0.0, 5);  //maze
    // EgoAddPoint(12, 2, 0, 6.41, -0.75, 1.44, 0.0, 5);
    // EgoAddPoint(13, 2, 0, ring1_pose(0), ring1_pose(1), ring1_pose(2), 3.14, 4);
    // EgoAddPoint(14, 2, 0, ring1_pose(0)+2, ring1_pose(1), ring1_pose(2), 3.14, 5);
    // EgoAddPoint(15, 2, 0, ring2_pose(0)+2, ring2_pose(1), ring2_pose(2), 3.14, 4);
    // EgoAddPoint(16, 2, 0, ring2_pose(0), ring2_pose(1), ring2_pose(2), 3.14, 5);
    // EgoAddPoint(17, 2, 0, double_pose1(0), double_pose1(1), double_pose1(2), 3.14, 5);  //double ring
    // EgoAddPoint(18, 2, 0, double_pose2(0), double_pose2(1), double_pose2(2), 3.14, 5);  
    // EgoAddPoint(19, 2, 0, ring3_pose(0), ring3_pose(1), ring3_pose(2), 3.14, 5);  //dynamic ring
}

/*----------LXK----------*/




/**
 * @date 2024-04-11 03:10:00 
 * @author SHI YANGXI
 * @brief  New Ego_traj function , not include pub function!
 * @param id:   id of each waypoint, should start from 0 and increase by 1 each time
 * @param mode :    0 : init mode; 1: normal mode; 2: slow mode; 3: stop mode(not used)  // not finished and tested yet
 * @param x:    position x,double
 * @param y:    position y,double
 * @param z:    position z,double
 * @param  traj_utils::Flag , should be a ptr of flag msg
 * @param  super_msgs::Flag , should be a ptr of flag msg
 * @return none
 */
void EGO_flag_aimpos(TypePoint point)
{
    // flag_traj_msg.header.frame_id = "world";
    // flag_traj_msg.header.stamp = ros::Time::now();
    // flag_traj_msg.header.seq = 0;
    // flag_traj_msg.id = point.id;
    // flag_traj_msg.mode = point.mode;
    // flag_traj_msg.is_map = point.is_map;
    // flag_traj_msg.position.x = point.x;
    // flag_traj_msg.position.y = point.y;
    // flag_traj_msg.position.z = point.z;

    // flag_super_msg.header.frame_id = "world";
    // flag_super_msg.header.stamp = ros::Time::now();
    // flag_super_msg.header.seq = 0;
    // flag_super_msg.id = point.id;
    // flag_super_msg.mode = point.mode;
    // flag_super_msg.is_map = point.is_map;
    // flag_super_msg.position.x = point.x;
    // flag_super_msg.position.y = point.y;
    // flag_super_msg.position.z = point.z;
}
/**
 * @date 2024-04-11 03:10:00
 * @author SHI YANGXI
 * @brief  New Ego_traj positionCommand callback function
 * @param  quadrotor_msgs::PositionCommand::ConstPtr &msg : the ptr of control message
 * @return none
 */


/**
 * @date 2024-04-13 19:10:00
 * @author SHI YANGXI
 * @brief  New Ego_traj state callback function
 * @param  traj_utils::FlagConstPtr &msg : the ptr of ego state message
 * @param  super_msgs::FlagConstPtr &msg : the ptr of ego state message
 * @return none
 */
// void EGO_flag_state_cb(const traj_utils::FlagStateConstPtr &msg)
// {   
//     flag_state.now_id = msg->now_id; 
//     flag_state.touch_goal = msg->touch_goal;
    
// }

/*--------------------------- Main ---------------------------*/

void UdpListen(const uint16_t cport)
{
    ros::NodeHandle nh;
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
        ROS_ERROR("Network Error!!!");
        return;
    }

    /* 将套接字和IP、端口绑定 */
    struct sockaddr_in addr_lis;
    int len;
    memset(&addr_lis, 0, sizeof(struct sockaddr_in));
    addr_lis.sin_family = AF_INET;
    addr_lis.sin_port = htons(cport);
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    addr_lis.sin_addr.s_addr = htonl(INADDR_ANY); // 自动获取IP地址
    len = sizeof(addr_lis);

    /* 绑定socket */
    if (bind(sock_fd, (struct sockaddr *)&addr_lis, sizeof(addr_lis)) < 0)
    {
        perror("bind error:");
        exit(1);
    }

    int recv_num;
    char recv_buf[100];
    const char dot[2] = ",";
    struct sockaddr_in addr_client;

    while (ros::ok())
    {
        char *p;
        int ent = 0;
        int user_cmd;
        double px, py, pz;

        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        if (recv_num < 0 || abs(recv_num - 19) > 3)
        {
            ROS_ERROR("Receive Fail!!!");
            continue;
        }
        recv_buf[recv_num] = '\0';
        // ROS_INFO("Rec: %s, len = %d", recv_buf, recv_num);

        /* receive UDP data */
        p = strtok(recv_buf, dot);
        sscanf(p, "%d", &user_cmd);
        p = strtok(NULL, dot);
        sscanf(p, "%lf", &px);
        p = strtok(NULL, dot);
        sscanf(p, "%lf", &py);
        p = strtok(NULL, dot);
        sscanf(p, "%lf", &pz);
        
        if (is_udp_enable) {cmd = user_cmd;}

        /*    wait for pose EKF    */
        if (cmd == 0)
        {
            //if(is_ready_fly) {ROS_INFO("Ready Fly");}
            //else {ROS_WARN("Waiting Pose EKF!");}
        }
        /* low battery protection (land and disable UDP) */
        else
        {
            // if(voltage < 14.8)
            // {
            //     cmdd = 4;
            //     flag_udp = false;
            //     ROS_ERROR("Low Battery!!!");
            // }
            // if(emergency_channel < 1500)
            // {cmd = 9;}
        }

    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "single_offboard_fsm");
    ros::NodeHandle nh;

    /*--------- Publisher ---------*/
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
        ("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher local_target_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("/mavros/setpoint_raw/local", 10);
    ros::Publisher local_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>
        ("/mavros/setpoint_raw/attitude", 10);
    ros::Publisher vis_att_pub = nh.advertise<geometry_msgs::PoseStamped>("/vis", 100);
    ros::Publisher ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_goal", 10);
    // SYX FSM TEST
    // ros::Publisher ego_flag_pub = nh.advertise<traj_utils::Flag>("/ego_planner/flag_msg", 10);
    // SYX FSM TEST DONE

    /*----------LXK----------*/
    ros::Publisher perc_mode_pub = nh.advertise<std_msgs::Int64>("/perc_mode", 10);
    ros::Publisher ap_pub = nh.advertise<geometry_msgs::PoseStamped>("/ap_global", 10);
    // ros::Publisher ring_pose_pub1 = nh.advertise<geometry_msgs::PoseStamped>("/ring_pose1", 10);
    // ros::Publisher ring_pose_pub2 = nh.advertise<geometry_msgs::PoseStamped>("/ring_pose2", 10);
   


    /*--------- Subscriber ---------*/
    ros::Subscriber ready_sub = nh.subscribe<std_msgs::Bool>
        ("/fsm_ctrl/ekf_ready", 10, Ready_Callback);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("/mavros/state", 10, State_Callback);
    ros::Subscriber voltage_sub = nh.subscribe<sensor_msgs::BatteryState>
        ("/mavros/battery", 10, Voltage_Callback);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, Pose_Callback);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("/mavros/local_position/velocity_local", 10, Vel_Callback);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
        ("/mavros/imu/data", 3, IMU_Callback);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>
        ("/mavros/rc/in", 10, RC_Callback);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
        ("/camera/odom/sample", 3, Odom_Callback);
    // ros::Subscriber traj_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
    //     ("/drone_0_planning/pos_cmd", 10, Traj_Callback);
    // SYX FSM TEST
    // We can try new control using this topic 
   
 
    // ros::Subscriber ego_flag_state_sub = nh.subscribe<traj_utils::FlagState>("/ego_planner/flag_state", 10, EGO_flag_state_cb);
    // SYX FSM TEST DONE


    /*----------YYZ----------*/
    // ros::Subscriber planner_msgs_sub = nh.subscribe<super_msgs::Flag>("/super/flag_cmd", 10, PlannerCallback);
    // /*----------YYZ----------*/

    // /*----------LYX----------*/
    // ros::Subscriber planner_state_sub = nh.subscribe<super_msgs::Flag>("/super/flag_state", 10, SUPER_flag_state_cb);
    // ros::Publisher planner_cmd_pub = nh.advertise<super_msgs::Flag>("/super/flag_waypoint", 10);
    /*----------LYX----------*/
    

    ros::Publisher task_pub = nh.advertise<std_msgs::Int64>("/perc/task", 10);

    /*--------- Client ---------*/
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(RATE); // the setpoint publishing rate must be faster than 2Hz

    Eigen::Vector4d kp, kpi, kpd, kv, kvi, kvd;
   
    // parameters
    nh.param("single_offboard_fsm/kp_x", kp(0), 0.0);
    nh.param("single_offboard_fsm/kp_y", kp(1), 0.0);
    nh.param("single_offboard_fsm/kp_z", kp(2), 0.0);
    nh.param("single_offboard_fsm/kp_yaw", kp(3), 0.0);
    nh.param("single_offboard_fsm/kpi_x", kpi(0), 0.0);
    nh.param("single_offboard_fsm/kpi_y", kpi(1), 0.0);
    nh.param("single_offboard_fsm/kpi_z", kpi(2), 0.0);
    nh.param("single_offboard_fsm/kpi_yaw", kpi(3), 0.0);
    nh.param("single_offboard_fsm/kpd_x", kpd(0), 0.0);
    nh.param("single_offboard_fsm/kpd_y", kpd(1), 0.0);
    nh.param("single_offboard_fsm/kpd_z", kpd(2), 0.0);
    nh.param("single_offboard_fsm/kd_yaw", kpd(3), 0.0);
    nh.param("single_offboard_fsm/kv_x", kv(0), 0.0);
    nh.param("single_offboard_fsm/kv_y", kv(1), 0.0);
    nh.param("single_offboard_fsm/kv_z", kv(2), 0.0);
    nh.param("single_offboard_fsm/kv_yaw", kv(3), 0.0);
    nh.param("single_offboard_fsm/kvi_x", kvi(0), 0.0);
    nh.param("single_offboard_fsm/kvi_y", kvi(1), 0.0);
    nh.param("single_offboard_fsm/kvi_z", kvi(2), 0.0);
    nh.param("single_offboard_fsm/kvi_yaw", kvi(3), 0.0);
    nh.param("single_offboard_fsm/kvd_x", kvd(0), 0.0); 
    nh.param("single_offboard_fsm/kvd_y", kvd(1), 0.0);
    nh.param("single_offboard_fsm/kvd_z", kvd(2), 0.0);
    nh.param("single_offboard_fsm/kvd_yaw", kvd(3), 0.0);

    

    int test_yaw = 0;

    Set_AimPos(0.0, 0.0, HEIGHT);

    new std::thread(&UdpListen, 12001);
    // wait for FCU connection
    /*
        while(ros::ok() && !current_state.connected)
        {
            ros::spinOnce();
            rate.sleep();
        }
    */

    mavros_msgs::SetMode offboard_mode;
    offboard_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::SetMode land_mode;
    land_mode.request.custom_mode = "LAND";
    mavros_msgs::SetMode mode_cmd;
    mavros_msgs::CommandBool arm_cmd, disarm_cmd;
    arm_cmd.request.value = true;
    disarm_cmd.request.value = false;

    ros::Time last_request = ros::Time::now();
    ros::Time last_goal;
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(aim_pos);
        ros::spinOnce();
        rate.sleep();
    }

    // FLAGCtrller ctrller(RATE, 0.33, kp, kpi, kpd, kv, kvi, kvd);

    /*----------LXK----------*/
    EgoGeneTraj();
    /*----------LXK----------*/

    // SYX FSM TEST
    ros::Time flagFSM = ros::Time::now();
    ros::Time flagReplanFSM = ros::Time::now();
    bool flag_istimer_done = false;
    bool flag_issend_done = false;
    // BOOL FLAG_isreplan = false;
    // SYX FSM TEST DONE




   

    std::array<double, 2> _w_limit = {-3.14, 3.14};
    std::array<double, 2> _acc_z_limit = {0.0, 15.0};
    
    std::vector<double> current_states;
    std::vector<double> desired_states;

    double w_x_cmd = 0;
    double w_y_cmd = 0;
    double w_z_cmd = 0;

    traj_time = 0;



    while (ros::ok())
    {
        ros::spinOnce();
        


        /*--------- 解锁 ---------*/
        if (cmd == 1)
        {
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
                {
                    ROS_WARN("Mode Offboard!");
                }
                last_request = ros::Time::now();
            }
            else
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_WARN("Mode Armed!");
                    }
                    last_request = ros::Time::now();
                }
            }

            target_att.header.frame_id = std::string("FCU");
            target_att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

            target_att.body_rate.x = 0.0;
            target_att.body_rate.y = 0.0;
            target_att.body_rate.z = 0.0;

            target_att.thrust = 0.02;
            local_attitude_pub.publish(target_att);
        }

        // /*--------- 位置悬停 ---------*/
        if (cmd == 2)
        {
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
                {
                    ROS_WARN("Mode Offboard!");
                }
                last_request = ros::Time::now();
            }
            else
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_WARN("Mode Armed!");
                    }
                    last_request = ros::Time::now();
                }
            }

            Set_AimPos(0.0, 0.0, 1.0);
            local_pos_pub.publish(aim_pos);
        }

        // /*--------- 推力起飞 ---------*/
        if (cmd == 3)
        {
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
                {
                    ROS_WARN("Mode Offboard!");
                }
                last_request = ros::Time::now();
            }
            else
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_WARN("Mode Armed!");
                    }
                    last_request = ros::Time::now();
                }
            }

            if(takeoff_channel > 1500)
            {
                mavros_msgs::AttitudeTarget target_arm;
	            target_arm.header.frame_id = std::string("FCU");
                target_arm.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                                mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                                mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
                target_arm.orientation.x = 0.0;
                target_arm.orientation.y = 0.0;
                target_arm.orientation.z = 0.0;
                target_arm.orientation.w = 1.0;
                target_arm.thrust = 0.05;
                local_attitude_pub.publish(target_arm);
            }
            else
            {
                Set_AimPos(0.0, 0.0, 0.4);
                local_pos_pub.publish(aim_pos);
            } 

            // perc_mode = 7;
            // perc_mode_msg.data = perc_mode;
            // PrintInfo(perc_mode_print_count, 25, "perc mode: ", perc_mode);
            // perc_mode_pub.publish(perc_mode_msg);   

        }

        /*--------- 降落 ---------*/
        if (cmd == 4)
        {
            if (!is_land)
            {
                Set_AimPos(pos_fcu[0], pos_fcu[1], 0.005);
                local_pos_pub.publish(aim_pos);
                if (abs(pos_fcu[2] - 0.05) < 0.05)
                {
                    is_land = true;
                }
            }
            else
            {
                if (current_state.mode == "OFFBOARD")
                {
                    // mode_cmd.request.custom_mode = "Hold";
                    // set_mode_client.call(mode_cmd);
                }
                else
                {
                    if (current_state.armed)
                    {
                        arm_cmd.request.value = false;
                        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_WARN("Mode Disarm!");
                        }
                    }
                }
            }
        }

        if (cmd == 5)
        {
            double traj_x = 0.75 * std::sin(0.02 * traj_time);
            double traj_y = 0.75 * std::cos(0.02 * traj_time)- 0.75;
            Set_AimPos(traj_x, traj_y, 1.0);
            local_pos_pub.publish(aim_pos);
            traj_time++;
            if(traj_time > 314)
                traj_time = 0;
        }

        
        if (cmd == 8)
        {}

        if (cmd == 6)
        {
            if(traj_time >= 0 && traj_time < 200)
            {
                double traj_x = 1.5 * 0.005 * traj_time;
                double traj_y = 0.0;
                Set_AimPos(traj_x, traj_y, 1.0);
                local_pos_pub.publish(aim_pos);
                traj_time++;
            }
            else if(traj_time >= 200 && traj_time < 400)
            {
                double traj_x = 1.5;
                double traj_y = -1.5 * 0.005 * (traj_time - 200);
                Set_AimPos(traj_x, traj_y, 1.0);
                local_pos_pub.publish(aim_pos);
                traj_time++;
            }
            else if(traj_time >= 400 && traj_time < 600)
            {
                double traj_x = 1.5 - 1.5 * 0.005 * (traj_time - 400);
                double traj_y = -1.5;
                Set_AimPos(traj_x, traj_y, 1.0);
                local_pos_pub.publish(aim_pos);
                traj_time++;
            }
            else
            {
                double traj_x = 0.0;
                double traj_y = 1.5 * 0.005 * (traj_time - 600) - 1.5;
                Set_AimPos(traj_x, traj_y, 1.0);
                local_pos_pub.publish(aim_pos);
                traj_time++;
            }
            if(traj_time == 800)
            {
                traj_time = 0;
            }
            
        }

        if (cmd ==7)
        {}
        
        if(cmd == 9)
        {}

        
        rate.sleep();
    }
    return 0;
}
