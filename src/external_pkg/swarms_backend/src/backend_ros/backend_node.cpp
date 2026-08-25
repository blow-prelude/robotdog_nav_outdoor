#include "../include/backend_ros/backend_node.hpp"
#include "../include/backend_ros/backend_utils.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

BackendNode::BackendNode(const std::string &node_name, bool intra_process_comms) : Node(node_name, "",
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
                                                                                   timeout_duration_(2s) {

    prefix_ = this->declare_parameter("prefix", "");
    vehicle_id_ = this->declare_parameter("vehicle_id", 0);
    pub_rate_ = this->declare_parameter("pub_rate", 5.0);
    curr_drone_id_ = this->declare_parameter("curr_drone_id", "");

    backendnode_init();
}

BackendNode::BackendNode(const std::string &prefix, const std::string &curr_drone_id, int vehicle_id,
                         double pub_rate, std::string node_name = "backend_ros_foxy_node",
                         std::string node_namespace = "", bool intra_process_comms) : prefix_(prefix),
                                                                                      vehicle_id_(vehicle_id),
                                                                                      curr_drone_id_(curr_drone_id),
                                                                                      pub_rate_(pub_rate),
                                                                                      timeout_duration_(2s),
                                                                                      Node(node_name, node_namespace,
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

    backendnode_init();
}

void BackendNode::backendnode_init() {

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
    catch (const YAML::ParserException &e) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "YAML FILE 'topic_key' parse error: " << std::string(e.what()) << ", exit...");
        exit(EXIT_FAILURE);
    }
    catch (const YAML::BadFile &e) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "YAML FILE 'topic_key' cannot be loaded: " << std::string(e.what()) << ", exit...");
        exit(EXIT_FAILURE);
    }

    try {
        robotMajorState_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["uav_state_machine"]["usm_state_topic"].as<std::string>();
        usmData_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["uav_state_machine"]["usm_info_topic"].as<std::string>();
        globalPositionGps_sub_topic_ = curr_drone_id_ + "/mavros/global_position/global";
        mavrosState_sub_topic_ = curr_drone_id_ + "/mavros/state";
        taskIndex_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_manage"]["task_index_topic"].as<std::string>();
        pastIndex_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_manage"]["cover_reach_index"].as<std::string>();
        can_confirmed_flag_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_manage"]["can_confirmed_flag"].as<std::string>();
        attack_confirm_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_fixedpoint"]["operation_confirm"].as<std::string>();
        targetsGps_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["task"]["task_attack"]["target_info_topic"].as<std::string>();

        satellite_sub_topic_ = curr_drone_id_ + "/mavros/global_position/raw/satellites";
        // 订阅 /mavros/battery 话题
        battery_sub_topic_ = curr_drone_id_ + "/mavros/battery";

        videoRecordState_sub_topic_ =
                curr_drone_id_ + "/" + topic_config["perception"]["video_record_state_topic"].as<std::string>();

        lightState_sub_topic_ = curr_drone_id_ + "/" + topic_config["light"]["light_state_topic"].as<std::string>();

        ptzState_sub_topic_ = curr_drone_id_ + "/" + topic_config["perception"]["ptz_flag_topic"].as<std::string>();

        px4Pos_sub_topic_ = curr_drone_id_ + "/mavros/local_position/pose";

        px4Vel_sub_topic_ = curr_drone_id_ + "/mavros/local_position/velocity_local";

        shipRecord_sub_topic_ = curr_drone_id_+"/isrecord";
        shipRecord_pub_topic_ = shipRecord_sub_topic_+"/backend";

        robotMajorState_pub_topic_ = robotMajorState_sub_topic_ + "/backend";
        globalPositionGps_pub_topic_ = globalPositionGps_sub_topic_ + "/backend";
        mavrosState_pub_topic_ = mavrosState_sub_topic_ + "/backend";
        taskIndex_pub_topic_ = taskIndex_sub_topic_ + "/backend";
        pastIndex_pub_topic_ = pastIndex_sub_topic_ + "/backend";
        can_confirmed_flag_pub_topic_ = can_confirmed_flag_sub_topic_ + "/backend";
        attack_confirm_pub_topic_ = attack_confirm_sub_topic_ + "/backend";
        targetsGps_pub_topic_ = targetsGps_sub_topic_ + "/backend";
        satellite_pub_topic_ = satellite_sub_topic_ + "/backend";
        battery_pub_topic_ = battery_sub_topic_ + "/backend";
        videoRecordState_pub_topic_ = videoRecordState_sub_topic_ + "/backend";
        lightState_pub_topic_ = lightState_sub_topic_ + "/backend";
        ptzState_pub_topic_ = ptzState_sub_topic_ + "/backend";
        px4Pos_pub_topic_ = px4Pos_sub_topic_ + "/backend";
        px4Vel_pub_topic_ = px4Vel_sub_topic_ + "/backend";
        usmData_pub_topic_ = usmData_sub_topic_ + "/backend";
    }
    catch (const YAML::Exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "YAML FILE 'topic_key' find key error: " << std::string(e.what()) << ", exit...");
        exit(EXIT_FAILURE);
    }

    // queue_1
    queue_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_1 = rclcpp::SubscriptionOptions();
    sub_opt_1.callback_group = queue_1;
    robotMajorState_sub_ = this->create_subscription<std_msgs::msg::Int32>(robotMajorState_sub_topic_,
                                                                           qos_sensor,
                                                                           std::bind(
                                                                                   &BackendNode::robotMajorStateCallback,
                                                                                   this, ::_1),
                                                                           sub_opt_1);
    robotMajorState_pub_ = this->create_publisher<std_msgs::msg::Int32>(robotMajorState_pub_topic_, qos_default);

    // queue_2
    queue_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_2 = rclcpp::SubscriptionOptions();
    sub_opt_2.callback_group = queue_2;
    globalPositionGps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(globalPositionGps_sub_topic_,
                                                                                    qos_sensor,
                                                                                    std::bind(
                                                                                            &BackendNode::globalPositionGpsCallback,
                                                                                            this, ::_1),
                                                                                    sub_opt_2);
    globalPositionGps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(globalPositionGps_pub_topic_,
                                                                                 qos_default);

    // queue_3
    queue_3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_3 = rclcpp::SubscriptionOptions();
    sub_opt_3.callback_group = queue_3;
    mavrosState_sub_ = this->create_subscription<mavros_msgs::msg::State>(mavrosState_sub_topic_,
                                                                          qos_sensor,
                                                                          std::bind(&BackendNode::mavrosStateCallback,
                                                                                    this, ::_1),
                                                                          sub_opt_3);
    mavrosState_pub_ = this->create_publisher<mavros_msgs::msg::State>(mavrosState_pub_topic_, qos_default);

    // queue_4
    queue_4 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_4 = rclcpp::SubscriptionOptions();
    sub_opt_4.callback_group = queue_4;
    taskIndex_sub_ = this->create_subscription<std_msgs::msg::UInt8>(taskIndex_sub_topic_,
                                                                     qos_sensor,
                                                                     std::bind(&BackendNode::taskIndexCallback, this,
                                                                               ::_1),
                                                                     sub_opt_4);
    taskIndex_pub_ = this->create_publisher<std_msgs::msg::UInt8>(taskIndex_pub_topic_, qos_default);

    // queue_5
    queue_5 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_5 = rclcpp::SubscriptionOptions();
    sub_opt_5.callback_group = queue_5;
    pastIndex_sub_ = this->create_subscription<msg_set_msgs::msg::CoverIndex>(pastIndex_sub_topic_,
                                                                              qos_sensor,
                                                                              std::bind(&BackendNode::pastIndexCallback,
                                                                                        this, ::_1),
                                                                              sub_opt_5);
    pastIndex_pub_ = this->create_publisher<msg_set_msgs::msg::CoverIndex>(pastIndex_pub_topic_, qos_default);

    // queue_6
    queue_6 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_6 = rclcpp::SubscriptionOptions();
    sub_opt_6.callback_group = queue_6;
    can_confirmed_flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(can_confirmed_flag_sub_topic_,
                                                                             qos_sensor,
                                                                             std::bind(
                                                                                     &BackendNode::canConfirmedFlagCallback,
                                                                                     this, ::_1),
                                                                             sub_opt_6);
    can_confirmed_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>(can_confirmed_flag_pub_topic_, qos_default);

    // queue_7
    queue_7 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_7 = rclcpp::SubscriptionOptions();
    sub_opt_7.callback_group = queue_7;
    targetsGps_sub_ = this->create_subscription<msg_set_msgs::msg::TargetsPose>(targetsGps_sub_topic_,
                                                                                qos_sensor,
                                                                                std::bind(
                                                                                        &BackendNode::targetsGpsCallback,
                                                                                        this, ::_1),
                                                                                sub_opt_7);
    targetsGps_pub_ = this->create_publisher<msg_set_msgs::msg::TargetsPose>(targetsGps_pub_topic_, qos_default);

    queue_8 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_8 = rclcpp::SubscriptionOptions();
    sub_opt_8.callback_group = queue_8;
    attack_confirm_sub_ = this->create_subscription<std_msgs::msg::Bool>(attack_confirm_sub_topic_,
                                                                         qos_sensor,
                                                                         std::bind(
                                                                                 &BackendNode::attackConfirmCallback,
                                                                                 this, ::_1),
                                                                         sub_opt_8);
    attack_confirm_pub_ = this->create_publisher<std_msgs::msg::Bool>(attack_confirm_pub_topic_, qos_default);

    queue_9 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_9 = rclcpp::SubscriptionOptions();
    sub_opt_9.callback_group = queue_9;
    satellite_sub_ = this->create_subscription<std_msgs::msg::UInt32>(satellite_sub_topic_,
                                                                      qos_sensor,
                                                                      std::bind(
                                                                              &BackendNode::satelliteCallback,
                                                                              this, ::_1),
                                                                      sub_opt_9);

    satellite_pub_ = this->create_publisher<std_msgs::msg::UInt32>(satellite_pub_topic_, qos_default);

    queue_10 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_10 = rclcpp::SubscriptionOptions();
    sub_opt_10.callback_group = queue_10;
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(battery_sub_topic_,
                                                                             qos_sensor,
                                                                             std::bind(
                                                                                     &BackendNode::batteryCallback,
                                                                                     this, ::_1),
                                                                             sub_opt_10);
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(battery_pub_topic_, qos_default);

    queue_11 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_11 = rclcpp::SubscriptionOptions();
    sub_opt_11.callback_group = queue_11;
    videoRecordState_sub_ = this->create_subscription<std_msgs::msg::Int8>(videoRecordState_sub_topic_,
                                                                           qos_sensor,
                                                                           std::bind(
                                                                                   &BackendNode::videoRecordStateCallback,
                                                                                   this, ::_1),
                                                                           sub_opt_11);
    videoRecordState_pub_ = this->create_publisher<std_msgs::msg::Int8>(videoRecordState_pub_topic_, qos_default);

    queue_12 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_12 = rclcpp::SubscriptionOptions();
    sub_opt_12.callback_group = queue_12;
    lightState_sub_ = this->create_subscription<std_msgs::msg::Int8>(lightState_sub_topic_,
                                                                     qos_sensor,
                                                                     std::bind(
                                                                             &BackendNode::lightStateCallback,
                                                                             this, ::_1),
                                                                     sub_opt_12);
    lightState_pub_ = this->create_publisher<std_msgs::msg::Int8>(lightState_pub_topic_, qos_default);

    queue_13 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_13 = rclcpp::SubscriptionOptions();
    sub_opt_13.callback_group = queue_13;
    ptzState_sub_ = this->create_subscription<std_msgs::msg::Int32>(ptzState_sub_topic_,
                                                                    qos_sensor,
                                                                    std::bind(
                                                                            &BackendNode::ptzStateCallback,
                                                                            this, ::_1),
                                                                    sub_opt_13);
    ptzState_pub_ = this->create_publisher<std_msgs::msg::Int32>(ptzState_pub_topic_, qos_default);

    queue_14 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_14 = rclcpp::SubscriptionOptions();
    sub_opt_14.callback_group = queue_14;
    px4Pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(px4Pos_sub_topic_,
                                                                             qos_sensor,
                                                                             std::bind(
                                                                                     &BackendNode::px4PosCallback,
                                                                                     this, ::_1),
                                                                             sub_opt_14);
    px4Pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(px4Pos_pub_topic_, qos_default);

    queue_15 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_15 = rclcpp::SubscriptionOptions();
    sub_opt_15.callback_group = queue_15;
    px4Vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(px4Vel_sub_topic_,
                                                                              qos_sensor,
                                                                              std::bind(
                                                                                      &BackendNode::px4VelCallback,
                                                                                      this, ::_1),
                                                                              sub_opt_15);
    px4Vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(px4Vel_pub_topic_, qos_default);

    queue_16 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_16 = rclcpp::SubscriptionOptions();
    sub_opt_16.callback_group = queue_16;
    usmData_sub_ = this->create_subscription<msg_set_msgs::msg::UsmDataInfo>(usmData_sub_topic_,
                                                                             qos_sensor,
                                                                             std::bind(
                                                                                     &BackendNode::usmDataCallback,
                                                                                     this, ::_1),
                                                                             sub_opt_16);
    usmData_pub_ = this->create_publisher<msg_set_msgs::msg::UsmDataInfo>(usmData_pub_topic_, qos_default);

    queue_17 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_17 = rclcpp::SubscriptionOptions();
    sub_opt_17.callback_group = queue_17;
    shipRecord_sub_ = this->create_subscription<std_msgs::msg::Bool>(shipRecord_sub_topic_,
                                                                     qos_sensor,
                                                                     std::bind(
                                                                             &BackendNode::shipRecordCallback,
                                                                             this, ::_1),
                                                                     sub_opt_17);
    shipRecord_pub_ = this->create_publisher<std_msgs::msg::Bool>(shipRecord_pub_topic_, qos_default);


    my_Timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&BackendNode::timerCallback, this));
    last_message_time_ = this->now();
    is_recv_origin_px4_state_ = false;
}

void BackendNode::robotMajorStateCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in robotMajorStateCallback.");
        return;
    }
    try {
        lastRobotMajorState_ = msg->data;
        rclcpp::Rate rate(pub_rate_);
        std_msgs::msg::Int32 pub_msg;
        pub_msg.data = msg->data;
        robotMajorState_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in robotMajorStateCallback: " << std::string(e.what()));
    }
}

void BackendNode::mavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in mavrosStateCallback.");
        return;
    }
    try {
        last_message_time_ = this->now();
        rclcpp::Rate rate(pub_rate_);
        mavros_msgs::msg::State pub_msg;
        pub_msg.header = msg->header;
        pub_msg.armed = msg->armed;
        pub_msg.manual_input = msg->manual_input;
        pub_msg.guided = msg->guided;
        pub_msg.connected = msg->connected;
        pub_msg.mode = msg->mode;
        mavrosState_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in mavrosStateCallback: " << std::string(e.what()));
    }
}

void BackendNode::videoRecordStateCallback(const std_msgs::msg::Int8::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in videoRecordStateCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        std_msgs::msg::Int8 pub_msg;
        pub_msg.data = msg->data;
        videoRecordState_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in videoRecordStateCallback: " << std::string(e.what()));
    }
}

void BackendNode::lightStateCallback(const std_msgs::msg::Int8::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in lightStateCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        std_msgs::msg::Int8 pub_msg;
        pub_msg.data = msg->data;
        lightState_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in lightStateCallback: " << std::string(e.what()));
    }
}

void BackendNode::ptzStateCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in ptzStateCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        std_msgs::msg::Int32 pub_msg;
        pub_msg.data = msg->data;
        ptzState_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in ptzStateCallback: " << std::string(e.what()));
    }
}

void BackendNode::px4PosCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in px4PosCallback.");
        return;
    }

    try {
        rclcpp::Rate rate(pub_rate_);
        geometry_msgs::msg::PoseStamped pub_msg;
        pub_msg.pose = msg->pose;
        px4Pos_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in px4PosCallback: " << std::string(e.what()));
    }
}

void BackendNode::px4VelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in px4VelCallback.");
        return;
    }

    try {
        rclcpp::Rate rate(pub_rate_);
        geometry_msgs::msg::TwistStamped pub_msg;
        pub_msg.twist = msg->twist;
        px4Vel_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in px4VelCallback: " << std::string(e.what()));
    }
}

void BackendNode::usmDataCallback(const msg_set_msgs::msg::UsmDataInfo::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in usmDataCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        msg_set_msgs::msg::UsmDataInfo pub_msg;
        pub_msg.ref_pos = msg->ref_pos;
        pub_msg.ref_gps = msg->ref_gps;
        usmData_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in usmDataCallback: " << std::string(e.what()));
    }
}

void BackendNode::globalPositionGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in globalPositionGpsCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        sensor_msgs::msg::NavSatFix pub_msg;
        pub_msg.header = msg->header;
        pub_msg.altitude = msg->altitude;
        pub_msg.latitude = msg->latitude;
        pub_msg.longitude = msg->longitude;
        pub_msg.status = msg->status;
        globalPositionGps_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in globalPositionGpsCallback: " << std::string(e.what()));
    }
}

void BackendNode::taskIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in taskIndexCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        std_msgs::msg::UInt8 pub_msg;
        pub_msg.data = msg->data;
        taskIndex_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in taskIndexCallback: " << std::string(e.what()));
    }
}

void BackendNode::pastIndexCallback(const msg_set_msgs::msg::CoverIndex::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in pastIndexCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        msg_set_msgs::msg::CoverIndex pub_msg;
        pub_msg.group_id = msg->group_id;
        pub_msg.point_index = msg->point_index;
        pastIndex_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in pastIndexCallback: " << std::string(e.what()));
    }
}

void BackendNode::canConfirmedFlagCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in canConfirmedFlagCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        std_msgs::msg::Bool pub_msg;
        pub_msg.data = msg->data;
        can_confirmed_flag_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in canConfirmedFlagCallback: " << std::string(e.what()));
    }
}

void BackendNode::attackConfirmCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in canConfirmedFlagCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        std_msgs::msg::Bool pub_msg;
        pub_msg.data = msg->data;
        attack_confirm_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in attackConfirmCallback: " << std::string(e.what()));
    }
}

void BackendNode::shipRecordCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in shipRecordCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        std_msgs::msg::Bool pub_msg;
        pub_msg.data = msg->data;
        shipRecord_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in shipRecordCallback: " << std::string(e.what()));
    }
}

void BackendNode::targetsGpsCallback(const msg_set_msgs::msg::TargetsPose::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in targetsGpsCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        msg_set_msgs::msg::TargetsPose pub_msg;
        pub_msg.targets_pose = msg->targets_pose;
        targetsGps_pub_->publish(pub_msg);
        rate.sleep();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in targetsGpsCallback: " << std::string(e.what()));
    }
}

void BackendNode::satelliteCallback(const std_msgs::msg::UInt32::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in satelliteCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        std_msgs::msg::UInt32 pub_msg;
        pub_msg.data = msg->data;
        satellite_pub_->publish(pub_msg);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in satelliteCallback: " << std::string(e.what()));
    }
}

void BackendNode::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    // 检查消息是否为空
    if (!msg) {
        RCLCPP_WARN_STREAM(get_logger(), "Received a null message in batteryCallback.");
        return;
    }
    try {
        rclcpp::Rate rate(pub_rate_);
        sensor_msgs::msg::BatteryState pub_msg;
        pub_msg.header = msg->header;
        pub_msg.voltage = msg->voltage;
        pub_msg.current = msg->current;
        pub_msg.temperature = msg->temperature;
        pub_msg.charge = msg->charge;
        pub_msg.capacity = msg->capacity;
        pub_msg.design_capacity = msg->design_capacity;
        pub_msg.percentage = msg->percentage;
        pub_msg.power_supply_status = msg->power_supply_status;
        pub_msg.power_supply_health = msg->power_supply_health;
        pub_msg.power_supply_technology = msg->power_supply_technology;
        pub_msg.present = msg->present;
        pub_msg.location = msg->location;
        pub_msg.serial_number = msg->serial_number;
        battery_pub_->publish(pub_msg);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error in batteryCallback: " << std::string(e.what()));
    }
}

void BackendNode::timerCallback() {

    auto now = this->now();
    if ((now - last_message_time_) > timeout_duration_) {
        RCLCPP_WARN_STREAM(get_logger(), "origin px4 state not recv!");
        is_recv_origin_px4_state_ = false;
    } else {
        if (is_recv_origin_px4_state_ == false) {
            const std::string GREEN = "\033[32m"; // green
            const std::string RESET = "\033[0m";  // reset to default color
            RCLCPP_INFO_STREAM(get_logger(), GREEN << "origin px4 state recv!" << RESET);
        }
        is_recv_origin_px4_state_ = true;
    }
}

BackendNode::~BackendNode() {
    rclcpp::shutdown();
}

// int main(int argc, char **argv) {
//
//     rclcpp::init(argc, argv);
//
// #ifdef NDEBUG
//     std::cout << "release --------------------------------------- " << std::endl;
//     std::string config_file_path = "/etc/mqttnode.yaml";
//     YAML::Node config;
//     try {
//         config = YAML::LoadFile(config_file_path);
//     } catch (const YAML::BadFile&e) {
//         std::cerr << "Failed to load config file: " << config_file_path << std::endl;
//         return -1;
//     }
//     std::string prefix = config["prefix"].as<std::string>("drone_");
//     int vehicle_id = config["vehicle_id"].as<int>(0);
//     double pub_rate = config["pub_rate"].as<double>(5.0);
//
//     std::string curr_drone_id = "/" + prefix + std::to_string(vehicle_id);
//
//     std::string node_name = "backend_ros_foxy_node_" + std::to_string(vehicle_id);
//     std::string node_namespace = prefix + std::to_string(vehicle_id);
//
//     auto backend_node = std::make_shared<BackendNode>(prefix, curr_drone_id, vehicle_id, pub_rate, node_name, node_namespace);
//
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(backend_node);
//
//     executor.spin();
//
//     rclcpp::shutdown();
// #else
//     std::cout << "debug --------------------------------------- " << std::endl;
//     auto backend_node = std::make_shared<BackendNode>();
//
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(backend_node);
//
//     executor.spin();
//
//     rclcpp::shutdown();
// #endif
//     return 0;
// }