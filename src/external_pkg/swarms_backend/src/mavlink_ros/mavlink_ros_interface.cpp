#include "mavlink_ros/mavlink_ros_interface.h"
#include <iostream>
#include <random>

void *start_mavlink_ros_interface_read_thread(void *args) {
    MavlinkRosInterface *mavlink_ros_interface = (MavlinkRosInterface *) args;

    mavlink_ros_interface->start_read_thread();

    return nullptr;
}

void *start_mavlink_ros_interface_write_thread(void *args) {
    MavlinkRosInterface *mavlink_ros_interface = (MavlinkRosInterface *) args;

    mavlink_ros_interface->start_write_thread();

    return nullptr;
}

//-------------------------------------------------------------------------------
using namespace std::chrono_literals;
using std::placeholders::_1;

MavlinkRosInterface::MavlinkRosInterface(boost::asio::io_service &io_service, const std::string &curr_drone_id,
                                         const std::string &prefix,
                                         int vehicle_id,
                                         const std::string &udp_remote_url,
                                         int udp_local_port,
                                         const std::string &serial_url,
                                         int serial_baudrate,
                                         bool udp_use,
                                         bool debug,
                                         const std::string &node_name,
                                         const std::string &node_namespace, bool intra_process_comms)
        : Node(node_name, node_namespace,
               rclcpp::NodeOptions().use_intra_process_comms(
                       intra_process_comms)),
          qos_default(intra_process_comms
                      ? rclcpp::QoS(
                          rclcpp::QoSInitialization(
                                  rmw_qos_profile_sensor_data.history,
                                  10),
                          rmw_qos_profile_default)
                      : rclcpp::SystemDefaultsQoS()),
          qos_sensor(rclcpp::QoS(
                  rclcpp::QoSInitialization(
                          rmw_qos_profile_sensor_data.history,
                          10),
                  rmw_qos_profile_sensor_data)),
          curr_drone_id_(curr_drone_id),
          prefix_(prefix),
          vehicle_id_(vehicle_id),
          udp_remote_url_(udp_remote_url),
          udp_local_port_(udp_local_port),
          serial_url_(serial_url),
          serial_baudrate_(serial_baudrate),
          udp_use_(udp_use),
          debug_(debug),
          system_id(vehicle_id),
          companion_id(1) {
    reading_status = false; // whether the read thread is running
    writing_status = false; // whether the write thread is running
    time_to_exit = false;   // flag to signal thread exit

    read_tid = 0;  // read thread id
    write_tid = 0; // write thread id
    start_port(io_service);
    init();
}

MavlinkRosInterface::MavlinkRosInterface(boost::asio::io_service &io_service, const std::string &node_name,
                                         bool intra_process_comms) : Node(node_name,
                                                                          rclcpp::NodeOptions().use_intra_process_comms(
                                                                                  intra_process_comms)),
                                                                     qos_default(intra_process_comms
                                                                                 ? rclcpp::QoS(
                                                                                     rclcpp::QoSInitialization(
                                                                                             rmw_qos_profile_sensor_data.history,
                                                                                             10),
                                                                                     rmw_qos_profile_default)
                                                                                 : rclcpp::SystemDefaultsQoS()),
                                                                     qos_sensor(rclcpp::QoS(
                                                                             rclcpp::QoSInitialization(
                                                                                     rmw_qos_profile_sensor_data.history,
                                                                                     10),
                                                                             rmw_qos_profile_sensor_data)) {

    reading_status = false; // whether the read thread is running
    writing_status = false; // whether the write thread is running
    time_to_exit = false;   // flag to signal thread exit

    read_tid = 0;  // read thread id
    write_tid = 0; // write thread id

    // 声明并读取参数
    udp_remote_url_ = this->declare_parameter<std::string>("udp_remote_url", "10.88.105.142:13550");
    udp_local_port_ = this->declare_parameter<int>("udp_local_port", 14890);
    serial_url_ = this->declare_parameter<std::string>("serial_url", "/dev/ttyUSB0");
    serial_baudrate_ = this->declare_parameter<int>("serial_baudrate", 115200);
    udp_use_ = this->declare_parameter<bool>("udp_use", true);
    debug_ = this->declare_parameter<bool>("debug", true);
    prefix_ = this->declare_parameter<std::string>("prefix", "drone_");
    vehicle_id_ = this->declare_parameter<int>("vehicle_id", 0);
    // curr_drone_id_ = "/" + prefix_ + std::to_string(vehicle_id_);
    curr_drone_id_ = this->declare_parameter("curr_drone_id", "");
    system_id = vehicle_id_;
    start_port(io_service);

    init();
}

void MavlinkRosInterface::init() {
    std::string msg_set_msgs_path = ament_index_cpp::get_package_share_directory("msg_set_msgs");
    if (msg_set_msgs_path == "") {
        RCLCPP_ERROR_STREAM(get_logger(), "Can not find msg_set_msgs, exit...");
        exit(EXIT_FAILURE);
    }

    YAML::Node topic_config;
    try {
        std::string topic_config_path = msg_set_msgs_path + "/config/topic_key.yaml";
        topic_config = YAML::LoadFile(topic_config_path);
    }
    catch (const YAML::ParserException) {
        RCLCPP_ERROR_STREAM(get_logger(), "YAML FILE 'topic_key' parse error, exit...");
        exit(EXIT_FAILURE);
    }
    catch (const YAML::BadFile) {
        RCLCPP_ERROR_STREAM(get_logger(), "YAML FILE 'topic_key' cannot be loaded, exit...");
        exit(EXIT_FAILURE);
    }

    try {
        robotMajorState_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["uav_state_machine"]["usm_state_topic"].as<std::string>() +
                "/backend";
        usmData_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["uav_state_machine"]["usm_info_topic"].as<std::string>() +
                "/backend";
        globalPositionGps_sub_topic_ = curr_drone_id_ + "/mavros/global_position/global" + "/backend";
        statePX4_sub_topic_ = curr_drone_id_ + "/mavros/state" + "/backend";
        taskIndex_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_manage"]["task_index_topic"].as<std::string>() +
                "/backend";
        pastIndex_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_manage"]["cover_reach_index"].as<std::string>() +
                "/backend";
        canConfirmedFlag_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_manage"]["can_confirmed_flag"].as<std::string>() +
                "/backend";
        track_img_sub_topic_ = curr_drone_id_ + "/" + topic_config["perception"]["track_img_topic"].as<std::string>();
        targets_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_attack"]["target_info_topic"].as<std::string>() +
                "/backend";

        attack_confirm_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_fixedpoint"]["operation_confirm"].as<std::string>() +
                "/backend";

        satellite_sub_topic_ = curr_drone_id_ + "/mavros/global_position/raw/satellites/backend";
        battery_sub_topic_ = curr_drone_id_ + "/mavros/battery/backend";
        px4Pos_sub_topic_ = curr_drone_id_ + "/mavros/local_position/pose/backend";
        px4Vel_sub_topic_ = curr_drone_id_ + "/mavros/local_position/velocity_local/backend";

        videoRecordState_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["perception"]["video_record_state_topic"].as<std::string>() +
                "/backend";

        lightState_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["light"]["light_state_topic"].as<std::string>() + "/backend";

        ptzState_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["perception"]["ptz_flag_topic"].as<std::string>() + "/backend";

        shipRecored_sub_topic_ = curr_drone_id_ + "/isrecord/backend";
    }
    catch (const YAML::Exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "YAML FILE 'topic_key' find key error: " << std::string(e.what()) << ", exit...");
        exit(EXIT_FAILURE);
    }

    // ROS subscriber
    statePX4_sub_ = this->create_subscription<mavros_msgs::msg::State>(statePX4_sub_topic_, qos_sensor,
                                                                       std::bind(&MavlinkRosInterface::statePX4Callback,
                                                                                 this, _1));

    robotMajorState_sub_ = this->create_subscription<std_msgs::msg::Int32>(robotMajorState_sub_topic_, qos_sensor,
                                                                           std::bind(
                                                                                   &MavlinkRosInterface::robotMajorStateCallback,
                                                                                   this, _1));

    globalPosition_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(globalPositionGps_sub_topic_,
                                                                                 qos_sensor, std::bind(
                    &MavlinkRosInterface::currGlobalPositionCallback, this, _1));

    //    track_img_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(track_img_sub_topic_, qos_sensor, std::bind(&MqttNode::trackImgCallback, this, _1));
    //
    targets_sub_ = this->create_subscription<msg_set_msgs::msg::TargetsPose>(targets_sub_topic_, qos_sensor, std::bind(
            &MavlinkRosInterface::targetsCallback, this, _1));
    //
    //    // task state
    taskIndex_sub_ = this->create_subscription<std_msgs::msg::UInt8>(taskIndex_sub_topic_, qos_sensor,
                                                                     std::bind(&MavlinkRosInterface::taskIndexCallback,
                                                                               this, _1));

    pastIndex_sub_ = this->create_subscription<msg_set_msgs::msg::CoverIndex>(pastIndex_sub_topic_, qos_sensor,
                                                                              std::bind(
                                                                                      &MavlinkRosInterface::pastIndexCallback,
                                                                                      this, _1));

    canConfirmedFlag_sub_ = this->create_subscription<std_msgs::msg::Bool>(canConfirmedFlag_sub_topic_, qos_sensor,
                                                                           std::bind(
                                                                                   &MavlinkRosInterface::canConfirmedFlagCallback,
                                                                                   this, _1));

    attack_confirm_sub_ = this->create_subscription<std_msgs::msg::Bool>(attack_confirm_sub_topic_,
                                                                         qos_sensor,
                                                                         std::bind(
                                                                                 &MavlinkRosInterface::attackConfirmCallback,
                                                                                 this, ::_1));

    satellite_sub_ = this->create_subscription<std_msgs::msg::UInt32>(satellite_sub_topic_, qos_sensor,
                                                                      std::bind(&MavlinkRosInterface::satelliteCallback,
                                                                                this, _1));

    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(battery_sub_topic_, qos_sensor,
                                                                             std::bind(
                                                                                     &MavlinkRosInterface::batteryCallback,
                                                                                     this, _1));

    videoRecordState_sub_ = this->create_subscription<std_msgs::msg::Int8>(videoRecordState_sub_topic_, qos_sensor,
                                                                           std::bind(
                                                                                   &MavlinkRosInterface::videoRecordStateCallback,
                                                                                   this, _1));
    lightState_sub_ = this->create_subscription<std_msgs::msg::Int8>(lightState_sub_topic_, qos_sensor,
                                                                     std::bind(&MavlinkRosInterface::lightStateCallback,
                                                                               this, _1));
    ptzState_sub_ = this->create_subscription<std_msgs::msg::Int32>(ptzState_sub_topic_, qos_sensor,
                                                                    std::bind(&MavlinkRosInterface::ptzStateCallback,
                                                                              this, _1));

    px4Pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(px4Pos_sub_topic_, qos_sensor,
                                                                             std::bind(
                                                                                     &MavlinkRosInterface::px4PosCallback,
                                                                                     this, _1));

    px4Vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(px4Vel_sub_topic_, qos_sensor,
                                                                              std::bind(
                                                                                      &MavlinkRosInterface::px4VelCallback,
                                                                                      this, _1));
    usmData_sub_ = this->create_subscription<msg_set_msgs::msg::UsmDataInfo>(usmData_sub_topic_, qos_sensor,
                                                                             std::bind(
                                                                                     &MavlinkRosInterface::usmDataCallback,
                                                                                     this, _1));

    shipRecored_sub_ = this->create_subscription<std_msgs::msg::Bool>(shipRecored_sub_topic_,
                                                                      qos_sensor,
                                                                      std::bind(
                                                                              &MavlinkRosInterface::shipRecordCallback,
                                                                              this, ::_1));


    //    mavros_state_monitor_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MqttNode::mavrosStateMonitorTimerCallback, this));
    //    recv_px4_state_backend_time_ = this->now();
    //    is_recv_px4_state_backend_ = false;
    //
    //    ctl_heartbeat_monitor_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MqttNode::ctlHeartbeatMonitorTimerCallback, this));
    //    recv_ctl_heartbeat_time_ = this->now();
    //    is_recv_ctl_heartbeat_ = false;
    //

    std::random_device rd;  // 随机设备（种子生成器）
    std::mt19937 gen(rd()); // 使用 Mersenne Twister 随机数引擎
    std::uniform_real_distribution<> dist(0.2, 1.0);

    double seconds = dist(gen);
    std::cout << "seconds:    " << seconds << std::endl;
    uav_state_output_timer_ = this->create_wall_timer(std::chrono::duration<double>(seconds),
                                                      std::bind(&MavlinkRosInterface::uavStateOutputTimerCallback,
                                                                this));
}

MavlinkRosInterface::~MavlinkRosInterface() {
    stop();
}

void MavlinkRosInterface::add_message_to_buff(mavlink_message_t &msg) {
    std::lock_guard<std::mutex> lock_guard(mtx);
    buff.push(msg);
}

void MavlinkRosInterface::statePX4Callback(const mavros_msgs::msg::State::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in statePX4.");
        return;
    }

    try {
        std::lock_guard<std::mutex> lock(px4_state_mutex_);
        px4_state_.connected = msg->connected;
        px4_state_.armed = msg->armed;
        px4_state_.guided = msg->guided;
        px4_state_.manual_input = msg->manual_input;
        px4_state_.mode = msg->mode;
        px4_state_.isUpdated = true;
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in statePX4Callback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::robotMajorStateCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in robotMajorState.");
        return;
    }
    try {

        std::lock_guard<std::mutex> lock(robot_major_state_mutex_);
        robot_major_state_.state = msg->data;
        robot_major_state_.isUpdated = true;
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in robotMajorStateCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::currGlobalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in currGlobalPosition.");
        return;
    }
    try {

        std::lock_guard<std::mutex> lock(global_position_mutex_);
        global_position_.status = msg->status.status;
        global_position_.service = msg->status.service;
        global_position_.latitude = msg->latitude;
        global_position_.longitude = msg->longitude;
        global_position_.altitude = msg->altitude;
        global_position_.isUpdated = true;
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in currGlobalPositionCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::satelliteCallback(const std_msgs::msg::UInt32::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in satelliteCallback.");
        return;
    }
    try {
        std::lock_guard<std::mutex> lock(satellite_mutex_);
        satellite_.satellite_count = msg->data;
        satellite_.isUpdated = true;
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in satelliteCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in batteryCallback.");
        return;
    }
    try {
        std::lock_guard<std::mutex> lock(batteryState_mutex_);
        batteryState_.voltage = msg->voltage;
        batteryState_.isUpdated = true;
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in batteryCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::px4PosCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in px4PosCallback.");
        return;
    }
    try {
        std::lock_guard<std::mutex> lock(px4Pos_mutex_);
        px4Pos_.x = msg->pose.position.x;
        px4Pos_.y = msg->pose.position.y;
        px4Pos_.z = msg->pose.position.z;
        tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                          msg->pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(px4Pos_.roll, px4Pos_.pitch, px4Pos_.yaw);
        Eigen::Vector2d diff_pos(
                px4Pos_.x - usmDataInfo_.ref_pos[0],
                px4Pos_.y - usmDataInfo_.ref_pos[1]);
        px4Pos_.dis = diff_pos.norm();
        px4Pos_.high = px4Pos_.z - usmDataInfo_.ref_pos[2];
        px4Pos_.isUpdated = true;
    }

    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in px4PosCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::px4VelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in px4VelCallback.");
        return;
    }
    try {
        std::lock_guard<std::mutex> lock(px4Vel_mutex_);
        px4Vel_.vx = msg->twist.linear.x;
        px4Vel_.vy = msg->twist.linear.y;
        px4Vel_.vz = msg->twist.linear.z;
        px4Vel_.dis_vel = sqrt(pow(px4Vel_.vx, 2) + pow(px4Vel_.vy, 2));
        px4Vel_.isUpdated = true;
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in px4VelCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::usmDataCallback(const msg_set_msgs::msg::UsmDataInfo::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in usmDataCallback.");
        return;
    }
    try {
        std::lock_guard<std::mutex> lock(usmData_mutex_);
        usmDataInfo_.ref_gps[0] = msg->ref_gps[0];
        usmDataInfo_.ref_gps[1] = msg->ref_gps[1];
        usmDataInfo_.ref_gps[2] = msg->ref_gps[2];

        usmDataInfo_.ref_pos[0] = msg->ref_pos[0];
        usmDataInfo_.ref_pos[1] = msg->ref_pos[1];
        usmDataInfo_.ref_pos[2] = msg->ref_pos[2];
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in usmDataCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::uavStateOutputTimerCallback() {
    bool shouldSend = false;
    mavlink_uav_state_t uavStateMav; // 这里初始化消息结构体
    uint16_t field_flags = 0;
    {
        std::lock_guard<std::mutex> lock(robot_major_state_mutex_);
        if (robot_major_state_.isUpdated) {
            uav_state_.robotMajorState_ = robot_major_state_;
            robot_major_state_.isUpdated = false;
            shouldSend = true;
            uavStateMav.robotMajorState = uav_state_.robotMajorState_.state;
            field_flags |= 0x01;
        }
    }

    {
        std::lock_guard<std::mutex> lock(px4_state_mutex_);
        if (px4_state_.isUpdated) {
            uav_state_.px4State_ = px4_state_;
            px4_state_.isUpdated = false;
            shouldSend = true;
            uavStateMav.connected = static_cast<uint8_t>(uav_state_.px4State_.connected);
            uavStateMav.armed = static_cast<uint8_t>(uav_state_.px4State_.armed);
            uavStateMav.guided = static_cast<uint8_t>(uav_state_.px4State_.guided);
            uavStateMav.manual_input = static_cast<uint8_t>(uav_state_.px4State_.manual_input);
            strncpy(uavStateMav.mode, uav_state_.px4State_.mode.c_str(), sizeof(uavStateMav.mode));
            uavStateMav.mode_len = uav_state_.px4State_.mode.length();
            field_flags |= 0x02;
        }
    }

    {
        std::lock_guard<std::mutex> lock(global_position_mutex_);
        if (global_position_.isUpdated) {
            uav_state_.globalPosition_ = global_position_;
            global_position_.isUpdated = false;
            shouldSend = true;
            uavStateMav.status = uav_state_.globalPosition_.status;
            uavStateMav.service = uav_state_.globalPosition_.service;
            uavStateMav.longitude = static_cast<int32_t>(uav_state_.globalPosition_.longitude * 1e7);
            uavStateMav.latitude = static_cast<int32_t>(uav_state_.globalPosition_.latitude * 1e7);
            uavStateMav.altitude = static_cast<int32_t>(uav_state_.globalPosition_.altitude * 1e5);
            field_flags |= 0x04;
        }
    }

    {
        std::lock_guard<std::mutex> lock(satellite_mutex_);
        if (satellite_.isUpdated) {
            uav_state_.satellite_ = satellite_;
            satellite_.isUpdated = false;
            shouldSend = true;
            uavStateMav.satellite_count = uav_state_.satellite_.satellite_count;
            field_flags |= 0x08;
        }
    }

    {
        std::lock_guard<std::mutex> lock(batteryState_mutex_);
        if (batteryState_.isUpdated) {
            uav_state_.batteryState_ = batteryState_;
            batteryState_.isUpdated = false;
            shouldSend = true;
            uavStateMav.voltage = uav_state_.batteryState_.voltage;
            field_flags |= 0x10;
        }
    }

    {
        std::lock_guard<std::mutex> lock(px4Pos_mutex_);
        if (px4Pos_.isUpdated) {
            uav_state_.px4Pos_ = px4Pos_;
            px4Pos_.isUpdated = false;
            shouldSend = true;
            uavStateMav.dis = uav_state_.px4Pos_.dis;
            uavStateMav.high = uav_state_.px4Pos_.high;
            uavStateMav.yaw = uav_state_.px4Pos_.yaw;
            field_flags |= 0x20;
        }
    }
    {

        std::lock_guard<std::mutex> lock(px4Vel_mutex_);
        if (px4Vel_.isUpdated) {
            uav_state_.px4Vel_ = px4Vel_;
            px4Vel_.isUpdated = false;
            shouldSend = true;
            uavStateMav.dis_vel = uav_state_.px4Vel_.dis_vel;
            uavStateMav.high_vel = uav_state_.px4Vel_.vz;
            field_flags |= 0x40;
        }
    }

    if (shouldSend) {
        try {
            mavlink_message_t message;
            uavStateMav.field_flags = field_flags;
            mavlink_msg_uav_state_encode(system_id, companion_id, &message, &uavStateMav);
            add_message_to_buff(message);
            // RCLCPP_INFO_STREAM(get_logger(), "uavState update ");
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(get_logger(), "Error in uavStateOutputTimerCallback: " << std::string(e.what()));
        }
    }
}

void MavlinkRosInterface::targetsCallback(const msg_set_msgs::msg::TargetsPose::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in targetsCallback.");
        return;
    }

    try {
        const size_t targets_per_batch = 11;                                                // 每批发送11个目标
        size_t total_targets = msg->targets_pose.size();                                    // 总目标数量
        size_t total_batches = (total_targets + targets_per_batch - 1) / targets_per_batch; // 计算总批次

        for (size_t batch_index = 0; batch_index < total_batches; ++batch_index) {
            mavlink_targets_t targets;
            targets.total_batches = total_batches;
            targets.current_batch = batch_index + 1; // 当前批次

            int count = 0;
            // 计算本批次的有效目标数量
            size_t start_index = batch_index * targets_per_batch;
            size_t end_index = std::min((batch_index + 1) * targets_per_batch, total_targets);
            size_t actual_targets = end_index - start_index; // 实际发送的目标数量

            for (size_t i = start_index; i < end_index; ++i) {
                const auto &target = msg->targets_pose[i];
                // 将每个目标的经纬度、高度和标签字段转换并填充到 MAVLink 消息中
                targets.target[count++] = static_cast<int32_t>(target.lat * 1e7);
                targets.target[count++] = static_cast<int32_t>(target.lon * 1e7);
                targets.target[count++] = static_cast<int32_t>(target.alt * 1e5);
                targets.target[count++] = target.label;
                targets.target[count++] = target.id;
            }

            // 使用实际目标数量更新信息
            targets.batch_target_count = actual_targets; // 新增字段，表示实际目标数量
            targets.total_target_count = total_targets;
            // 构造并发送 MAVLink 消息
            mavlink_message_t message;
            mavlink_msg_targets_encode(system_id, companion_id, &message, &targets);
            add_message_to_buff(message); // 发送消息
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in targetsCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::taskIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in taskIndexCallback.");
        return;
    }
    try {
        // std::string jsonData = JsonUtils::jsonValue(msg);
        // auto pubmsg = mqtt::make_message(MqttTopic::mqtt_task_deploy_stream_feedback + std::to_string(vehicle_id_), jsonData);
        // pubmsg->set_qos(0);
        // client_->publish(pubmsg);

        mavlink_stream_feedback_t streamFeedback;
        streamFeedback.data = msg->data;

        mavlink_message_t message;
        mavlink_msg_stream_feedback_encode(system_id, companion_id, &message, &streamFeedback);
        add_message_to_buff(message);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in taskIndexCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::pastIndexCallback(const msg_set_msgs::msg::CoverIndex::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in pastIndexCallback.");
        return;
    }
    try {
        // std::string jsonData = JsonUtils::jsonValue(msg);
        // auto pubmsg = mqtt::make_message(MqttTopic::mqtt_follow_cover_past_index + std::to_string(vehicle_id_), jsonData);
        // pubmsg->set_qos(0);
        // client_->publish(pubmsg);

        mavlink_follow_cover_pass_waypoint_t flowConverPassWaypoint;
        flowConverPassWaypoint.groupId = msg->group_id;
        flowConverPassWaypoint.currentPointId = msg->point_index;

        mavlink_message_t message;
        mavlink_msg_follow_cover_pass_waypoint_encode(system_id, companion_id, &message, &flowConverPassWaypoint);
        add_message_to_buff(message);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in pastIndexCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::canConfirmedFlagCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in canConfirmedFlagCallback.");
        return;
    }
    // 检查客户端连接状态
    try {
        // std::string jsonData = JsonUtils::jsonValue(msg);
        // auto pubmsg = mqtt::make_message(MqttTopic::mqtt_can_confirmed_flag + std::to_string(vehicle_id_), jsonData);
        // pubmsg->set_qos(0);
        // client_->publish(pubmsg);
        mavlink_can_confirmed_flag_t confirmFlag;
        confirmFlag.data = static_cast<uint8_t>(msg->data);

        mavlink_message_t message;
        // mavlink_msg_can_confirmed_flag_encode(system_id,companion_id,&message, &confirmFlag);
        mavlink_msg_can_confirmed_flag_encode_chan(system_id, companion_id, 1, &message, &confirmFlag);
        add_message_to_buff(message);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in canConfirmedFlagCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::shipRecordCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in shipRecordCallback.");
        return;
    }
    // 检查客户端连接状态
    try {
        // std::string jsonData = JsonUtils::jsonValue(msg);
        // auto pubmsg = mqtt::make_message(MqttTopic::mqtt_can_confirmed_flag + std::to_string(vehicle_id_), jsonData);
        // pubmsg->set_qos(0);
        // client_->publish(pubmsg);
        mavlink_ship_record_state_t shipRecordState;
        shipRecordState.data = static_cast<int8_t>(msg->data);

        mavlink_message_t message;
        // mavlink_msg_can_confirmed_flag_encode(system_id,companion_id,&message, &confirmFlag);
        mavlink_msg_ship_record_state_encode_chan(system_id, companion_id, 1, &message, &shipRecordState);
        add_message_to_buff(message);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in shipRecordCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::attackConfirmCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in attackConfirmCallback.");
        return;
    }
    // 检查客户端连接状态
    try {
        // std::string jsonData = JsonUtils::jsonValue(msg);
        // auto pubmsg = mqtt::make_message(MqttTopic::mqtt_can_confirmed_flag + std::to_string(vehicle_id_), jsonData);
        // pubmsg->set_qos(0);
        // client_->publish(pubmsg);
        mavlink_attack_confirmed_flag_t attackConfirmedFlag;
        attackConfirmedFlag.data = static_cast<uint8_t>(msg->data);

        mavlink_message_t message;
        mavlink_msg_attack_confirmed_flag_encode(system_id, companion_id, &message, &attackConfirmedFlag);
        add_message_to_buff(message);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in attackConfirmCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::videoRecordStateCallback(const std_msgs::msg::Int8::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in videoRecordStateCallback.");
        return;
    }
    // 检查客户端连接状态
    try {
        mavlink_video_record_state_t videoRecordState;
        videoRecordState.data = msg->data;
        mavlink_message_t message;
        mavlink_msg_video_record_state_encode(system_id, companion_id, &message, &videoRecordState);
        add_message_to_buff(message);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in videoRecordStateCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::lightStateCallback(const std_msgs::msg::Int8::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in lightStateCallback.");
        return;
    }
    // 检查客户端连接状态
    try {
        std::cout << "*****************" << std::endl;
        mavlink_light_state_t lightState;
        lightState.data = msg->data;
        mavlink_message_t message;
        mavlink_msg_light_state_encode(system_id, companion_id, &message, &lightState);
        add_message_to_buff(message);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in lightStateCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::ptzStateCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in ptzStateCallback.");
        return;
    }
    try {
        std::cout << "-------------" << std::endl;
        mavlink_ptz_state_t ptzState;
        ptzState.data = msg->data;
        mavlink_message_t message;
        mavlink_msg_ptz_state_encode(system_id, companion_id, &message, &ptzState);
        add_message_to_buff(message);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in ptzStateCallback: " << std::string(e.what()));
    }
}

void MavlinkRosInterface::start() {
    int result;

    //    if (!port->is_running()) // PORT_OPEN
    //    {
    //        fprintf(stderr, "ERROR: port not open\n");
    //        throw 1;
    //    }

    printf("START READ THREAD \n");

    result = pthread_create(&read_tid, nullptr, &start_mavlink_ros_interface_read_thread, this);
    if (result)
        throw result;

    result = pthread_create(&write_tid, nullptr, &start_mavlink_ros_interface_write_thread, this);
    if (result)
        throw result;

    return;
}

void MavlinkRosInterface::stop() {
    printf("CLOSE THREADS\n");

    time_to_exit = true;
    stop_port();
    pthread_join(read_tid, NULL);
    pthread_join(write_tid, NULL);
}

void MavlinkRosInterface::read_messages() {
    // uint8_t seq_ack = 0; // record the sequence number of the ack

    // int success; // receive success flag

    // // Blocking wait for new data
    // while (!time_to_exit)
    // {
    //     mavlink_message_t message;
    //     success = port->read_message(message);
    //     if (success > 0)
    //     {
    //         // Handle Message ID
    //         switch (message.msgid)
    //         {
    //         case MAVLINK_MSG_ID_HEARTBEAT:
    //         {
    //             mavlink_heartbeat_t heartbeat_msg;
    //             mavlink_msg_heartbeat_decode(&message, &heartbeat_msg);
    //             std::cout << static_cast<int>(heartbeat_msg.custom_mode) << "========" << std::endl;
    //             // mavlink_to_rosmsg(heartbeat_msg);
    //             break;
    //         }

    //         case MAVLINK_MSG_ID_COMMAND_LONG:
    //         {
    //             mavlink_command_long_t cmd;
    //             mavlink_msg_command_long_decode(&message, &cmd);
    //             // mavlink_to_rosmsg(cmd);
    //             break;
    //         }
    //         default:
    //         {
    //             // printf("Warning, did not handle message id %i\n",message.msgid);
    //             break;
    //         }
    //         } // end: switch msgid
    //     } // end: if read message

    //     // give the write thread time to use the port
    //     if (writing_status == false)
    //     {
    //         usleep(100); // look for components of batches at 10kHz
    //     }
    // } // end: while not received all

    return;
}

void MavlinkRosInterface::read_thread() {
    // reading_status = true;

    // while (!time_to_exit)
    // {
    //     read_messages();
    //     usleep(1000); // Read batches at 1000Hz
    // }

    // reading_status = false;
    return;
}

int MavlinkRosInterface::write_message(mavlink_message_t message) {
    // do the write
    int len = port->write_message(message);

    // Done!
    return len;
}

void MavlinkRosInterface::write_thread(void) {
    writing_status = true;

    while (!time_to_exit) {
        mtx.lock();
        while (buff.size() > 0) {
            auto msg = buff.front();
            write_message(msg);
            buff.pop();
        }
        mtx.unlock();

        usleep(100);
    }

    writing_status = false;
    return;
}

void MavlinkRosInterface::start_read_thread() {
    if (reading_status != 0) {
        fprintf(stderr, "read thread already running\n");
        return;
    } else {
        read_thread();
        return;
    }
}

void MavlinkRosInterface::start_write_thread(void) {
    if (writing_status) {
        fprintf(stderr, "write thread already running\n");
        return;
    } else {
        write_thread();
        return;
    }
}

void MavlinkRosInterface::start_port(boost::asio::io_service &io_service) {

    if (udp_use_) {
        port_url_ = "udp://:" + std::to_string(udp_local_port_) + "@" + udp_remote_url_;
    } else {
        port_url_ = serial_url_ + ":" + std::to_string(serial_baudrate_);
    }

    // 根据 port_url 创建对应的 port
    if (port_url_.find("udp://:") == 0) {
        std::regex pattern("udp://:([0-9]+)@([0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+):([0-9]+)");
        std::smatch result;
        if (std::regex_match(port_url_, result, pattern)) {
            int rx_port = std::stoi(result[1].str());
            std::string ip = result[2];
            int tx_port = std::stoi(result[3].str());
            port = std::make_unique<UDP_Port>(io_service, ip.c_str(), rx_port, tx_port, debug_);
            std::cout << "UDP Port initialized: tx_port: " << tx_port << ", rx_port: " << rx_port << ", ip: " << ip
                      << std::endl;
        } else {
            throw std::runtime_error("UDP URL format error!");
        }
    } else if (port_url_.find("/dev/tty") == 0) {
        std::regex pattern("^(.*):([0-9]+)$");
        std::smatch result;
        if (std::regex_match(port_url_, result, pattern)) {
            std::string uart_name = result[1];
            int baudrate = std::stoi(result[2]);
            port = std::make_unique<Serial_Port>(uart_name.c_str(), baudrate, debug_);
        } else {
            throw std::runtime_error("Serial URL format error!");
        }
    } else {
        throw std::runtime_error("Port URL format error!");
    }

    // port->start(); // 打开端口
}

void MavlinkRosInterface::stop_port() {
    if (port) {
        port->stop();
    }
}
