/**
 * @file single_offboard_fsm.cpp
 * @brief Basic PX4 offboard control for a single drone.
 */

#include <fsm_ctrl/single_offboard_fsm.hpp>

namespace
{
constexpr double kControlRateHz = 50.0;
constexpr double kInitialHeight = 1.0;
constexpr uint16_t kUdpPort = 12001;
constexpr int kRcThreshold = 1500;

std::atomic<int> command{0};
bool is_landed = false;
int takeoff_channel = 0;

mavros_msgs::State current_state;
Eigen::Vector3d local_position = Eigen::Vector3d::Zero();
Eigen::Vector3d local_velocity = Eigen::Vector3d::Zero();
Eigen::Quaterniond local_attitude = Eigen::Quaterniond::Identity();
bool feedback_ready = false;
geometry_msgs::PoseStamped position_setpoint;
mavros_msgs::AttitudeTarget attitude_setpoint;

void SetPosition(double x, double y, double z)
{
    position_setpoint.pose.position.x = x;
    position_setpoint.pose.position.y = y;
    position_setpoint.pose.position.z = z;
}

void StateCallback(const mavros_msgs::State::ConstPtr &message)
{
    current_state = *message;
}

void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &message)
{
    local_position = Eigen::Vector3d(
        message->pose.position.x,
        message->pose.position.y,
        message->pose.position.z);
    local_attitude = Eigen::Quaterniond(message->pose.orientation.w, message->pose.orientation.x, message->pose.orientation.y, message->pose.orientation.z).normalized();
}

void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &message)
{
    local_velocity = Eigen::Vector3d(message->twist.linear.x, message->twist.linear.y, message->twist.linear.z);
    feedback_ready = true;
}

void RcCallback(const mavros_msgs::RCIn::ConstPtr &message)
{
    if (message->channels.size() <= 8)
    {
        ROS_WARN_THROTTLE(1.0, "RC input has no takeoff channel (channel 9)");
        return;
    }
    takeoff_channel = message->channels[8];
}

void ListenForUdpCommands(uint16_t port)
{
    const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0)
    {
        ROS_ERROR("Failed to create UDP command socket");
        return;
    }

    sockaddr_in server_address{};
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(port);
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(socket_fd,
             reinterpret_cast<sockaddr *>(&server_address),
             sizeof(server_address)) < 0)
    {
        ROS_ERROR(
            "Failed to bind UDP command socket on port %u",
            static_cast<unsigned int>(port));
        close(socket_fd);
        return;
    }

    while (ros::ok())
    {
        char receive_buffer[100]{};
        sockaddr_in client_address{};
        socklen_t client_address_length = sizeof(client_address);
        const ssize_t received_size = recvfrom(
            socket_fd,
            receive_buffer,
            sizeof(receive_buffer) - 1,
            0,
            reinterpret_cast<sockaddr *>(&client_address),
            &client_address_length);

        if (received_size < 0)
        {
            ROS_ERROR_THROTTLE(1.0, "Failed to receive UDP command");
            continue;
        }

        receive_buffer[received_size] = '\0';
        int received_command = 0;
        if (std::sscanf(receive_buffer, "%d", &received_command) != 1)
        {
            ROS_WARN_THROTTLE(1.0, "Ignored malformed UDP command");
            continue;
        }
        command.store(received_command, std::memory_order_relaxed);
    }

    close(socket_fd);
}

void RequestOffboardAndArm(
    ros::ServiceClient &set_mode_client,
    ros::ServiceClient &arming_client,
    mavros_msgs::SetMode &offboard_mode,
    mavros_msgs::CommandBool &arm_command,
    ros::Time &last_request)
{
    const ros::Time now = ros::Time::now();
    if (current_state.mode != "OFFBOARD" &&
        now - last_request > ros::Duration(5.0))
    {
        if (set_mode_client.call(offboard_mode) &&
            offboard_mode.response.mode_sent)
        {
            ROS_WARN("Mode Offboard!");
        }
        last_request = now;
    }
    else if (!current_state.armed &&
             now - last_request > ros::Duration(5.0))
    {
        if (arming_client.call(arm_command) && arm_command.response.success)
        {
            ROS_WARN("Mode Armed!");
        }
        last_request = now;
    }
}
}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_offboard_fsm");
    ros::NodeHandle node;

    const ros::Publisher position_publisher =
        node.advertise<geometry_msgs::PoseStamped>(
            "/mavros/setpoint_position/local", 10);
    const ros::Publisher attitude_publisher =
        node.advertise<mavros_msgs::AttitudeTarget>(
            "/mavros/setpoint_raw/attitude", 10);

    const ros::Subscriber state_subscriber =
        node.subscribe<mavros_msgs::State>(
            "/mavros/state", 10, StateCallback);
    const ros::Subscriber position_subscriber =
        node.subscribe<geometry_msgs::PoseStamped>(
            "/mavros/local_position/pose", 10, PoseCallback);
    const ros::Subscriber velocity_subscriber =
        node.subscribe<geometry_msgs::TwistStamped>(
            "/mavros/local_position/velocity_local", 10, VelocityCallback);
    const ros::Subscriber rc_subscriber =
        node.subscribe<mavros_msgs::RCIn>(
            "/mavros/rc/in", 10, RcCallback);

    ros::ServiceClient arming_client =
        node.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client =
        node.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode offboard_mode;
    offboard_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_command;
    arm_command.request.value = true;

    SetPosition(0.0, 0.0, kInitialHeight);
    position_setpoint.pose.orientation.w = 1.0;

    Eigen::Vector3f qpos(100.0f, 100.0f, 70.0f), qvel(4.0f, 4.0f, 2.0f);
    Eigen::Vector3f qquat(1.0f, 1.0f, 10.0f), rw(0.45f, 0.85f, 0.45f);
    NMPC_Ctrller_simple nmpc(0.02, {{0.0, 15.0}}, {{-3.14, 3.14}}, 8, 0.05,
        10, 4, qpos, qvel, qquat, rw, 0.15, 0.50);
    const auto nmpc_hover = [&](double height) {
        if (!feedback_ready) return false;
        std::vector<double> current{local_position.x(), local_position.y(), local_position.z(), local_velocity.x(), local_velocity.y(), local_velocity.z(), local_attitude.w(), local_attitude.x(), local_attitude.y(), local_attitude.z()};
        std::vector<double> desired;
        for (int i = 0; i < 9; ++i) desired.insert(desired.end(), {0.0, 0.0, height, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
        for (int i = 0; i < 8; ++i) desired.insert(desired.end(), {0.0, 0.0, 0.0, 9.8015});
        try { nmpc.optimal_solution(current, desired); }
        catch (const std::exception &e) { ROS_ERROR_THROTTLE(1.0, "NMPC failed: %s", e.what()); return false; }
        const Eigen::Vector3d rates = nmpc.getwCommand();
        attitude_setpoint.header.stamp = ros::Time::now();
        attitude_setpoint.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        attitude_setpoint.body_rate.x = rates.x(); attitude_setpoint.body_rate.y = rates.y(); attitude_setpoint.body_rate.z = rates.z();
        attitude_setpoint.thrust = std::max(0.0, std::min(1.0, nmpc.getAcc_zCommand()));
        attitude_publisher.publish(attitude_setpoint); return true;
    };

    std::thread(ListenForUdpCommands, kUdpPort).detach();

    ros::Rate rate(kControlRateHz);
    for (int i = 0; ros::ok() && i < 100; ++i)
    {
        position_publisher.publish(position_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();
    int trajectory_step = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        switch (command.load(std::memory_order_relaxed))
        {
        case 1:
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);

            attitude_setpoint.header.frame_id = "FCU";
            attitude_setpoint.type_mask =
                mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
            attitude_setpoint.body_rate.x = 0.0;
            attitude_setpoint.body_rate.y = 0.0;
            attitude_setpoint.body_rate.z = 0.0;
            attitude_setpoint.thrust = 0.02;
            attitude_publisher.publish(attitude_setpoint);
            break;

        case 2:
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);
            if (!nmpc_hover(1.0)) { SetPosition(0.0, 0.0, 1.0); position_publisher.publish(position_setpoint); }
            break;

        case 3:
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);
            if (!nmpc_hover(0.4)) { SetPosition(0.0, 0.0, 0.4); position_publisher.publish(position_setpoint); }
            break;

        case 4:
            if (!is_landed)
            {
                SetPosition(local_position.x(), local_position.y(), 0.005);
                position_publisher.publish(position_setpoint);
                is_landed = std::abs(local_position.z() - 0.05) < 0.05;
            }
            else if (current_state.mode != "OFFBOARD" && current_state.armed)
            {
                arm_command.request.value = false;
                if (arming_client.call(arm_command) &&
                    arm_command.response.success)
                {
                    ROS_WARN("Mode Disarm!");
                }
            }
            break;

        case 5:
        {
            const double trajectory_x =
                0.75 * std::sin(0.02 * trajectory_step);
            const double trajectory_y =
                0.75 * std::cos(0.02 * trajectory_step) - 0.75;
            SetPosition(trajectory_x, trajectory_y, 1.0);
            position_publisher.publish(position_setpoint);
            trajectory_step = (trajectory_step + 1) % 315;
            break;
        }

        case 6:
        {
            double trajectory_x = 0.0;
            double trajectory_y = 0.0;
            if (trajectory_step < 200)
            {
                trajectory_x = 1.5 * 0.005 * trajectory_step;
            }
            else if (trajectory_step < 400)
            {
                trajectory_x = 1.5;
                trajectory_y = -1.5 * 0.005 * (trajectory_step - 200);
            }
            else if (trajectory_step < 600)
            {
                trajectory_x =
                    1.5 - 1.5 * 0.005 * (trajectory_step - 400);
                trajectory_y = -1.5;
            }
            else
            {
                trajectory_y =
                    1.5 * 0.005 * (trajectory_step - 600) - 1.5;
            }

            SetPosition(trajectory_x, trajectory_y, 1.0);
            position_publisher.publish(position_setpoint);
            trajectory_step = (trajectory_step + 1) % 800;
            break;
        }

        default:
            break;
        }

        rate.sleep();
    }

    return 0;
}
