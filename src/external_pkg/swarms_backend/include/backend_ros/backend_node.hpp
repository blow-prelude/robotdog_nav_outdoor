#ifndef BACKEND_ROS_BACKEND_NODE_H
#define BACKEND_ROS_BACKEND_NODE_H

#include <map>
#include <thread>
#include <iostream>
#include <functional>
#include <unordered_set>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <msg_set_msgs/msg/cover_index.hpp>
#include <msg_set_msgs/msg/multi_task_confirm.hpp>
#include <msg_set_msgs/msg/cover_confirm.hpp>
#include <msg_set_msgs/msg/cover_param.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <msg_set_msgs/msg/targets_pose.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <msg_set_msgs/msg/usm_data_info.hpp>

struct TupleHash {
    size_t operator()(const std::tuple<float, float, float> &key) const {
        size_t h1 = std::hash<float>{}(std::get<0>(key));
        size_t h2 = std::hash<float>{}(std::get<1>(key));
        size_t h3 = std::hash<float>{}(std::get<2>(key));
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

class BackendNode : public rclcpp::Node {

private:
    rclcpp::CallbackGroup::SharedPtr queue_1;
    rclcpp::CallbackGroup::SharedPtr queue_2;
    rclcpp::CallbackGroup::SharedPtr queue_3;
    rclcpp::CallbackGroup::SharedPtr queue_4;
    rclcpp::CallbackGroup::SharedPtr queue_5;
    rclcpp::CallbackGroup::SharedPtr queue_6;
    rclcpp::CallbackGroup::SharedPtr queue_7;
    rclcpp::CallbackGroup::SharedPtr queue_8;
    rclcpp::CallbackGroup::SharedPtr queue_9;
    rclcpp::CallbackGroup::SharedPtr queue_10;

    rclcpp::CallbackGroup::SharedPtr queue_11;
    rclcpp::CallbackGroup::SharedPtr queue_12;
    rclcpp::CallbackGroup::SharedPtr queue_13;
    rclcpp::CallbackGroup::SharedPtr queue_14;
    rclcpp::CallbackGroup::SharedPtr queue_15;
    rclcpp::CallbackGroup::SharedPtr queue_16;
    rclcpp::CallbackGroup::SharedPtr queue_17;

    rclcpp::QoS qos_sensor;
    rclcpp::QoS qos_default;

    std::string curr_drone_id_;
    int vehicle_id_;
    std::string prefix_;
    double pub_rate_;
    int lastRobotMajorState_;

    /** ros sub **/

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robotMajorState_sub_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr globalPositionGps_sub_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavrosState_sub_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr taskIndex_sub_;

    rclcpp::Subscription<msg_set_msgs::msg::CoverIndex>::SharedPtr pastIndex_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr can_confirmed_flag_sub_;

    rclcpp::Subscription<msg_set_msgs::msg::TargetsPose>::SharedPtr targetsGps_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr attack_confirm_sub_;

    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr satellite_sub_;

    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr videoRecordState_sub_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr lightState_sub_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ptzState_sub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr px4Pos_sub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr px4Vel_sub_;

    rclcpp::Subscription<msg_set_msgs::msg::UsmDataInfo>::SharedPtr usmData_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shipRecord_sub_;


    /** ros sub topic **/
    std::string robotMajorState_sub_topic_;

    std::string globalPositionGps_sub_topic_;

    std::string mavrosState_sub_topic_;

    std::string taskIndex_sub_topic_;

    std::string pastIndex_sub_topic_;

    std::string can_confirmed_flag_sub_topic_;

    std::string targetsGps_sub_topic_;

    std::string attack_confirm_sub_topic_;

    std::string satellite_sub_topic_;

    std::string battery_sub_topic_;

    std::string videoRecordState_sub_topic_;

    std::string lightState_sub_topic_;

    std::string ptzState_sub_topic_;

    std::string px4Pos_sub_topic_;

    std::string px4Vel_sub_topic_;

    std::string usmData_sub_topic_;

    std::string shipRecord_sub_topic_;

    /** ros pub **/

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr robotMajorState_pub_;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr globalPositionGps_pub_;

    rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr mavrosState_pub_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr taskIndex_pub_;

    rclcpp::Publisher<msg_set_msgs::msg::CoverIndex>::SharedPtr pastIndex_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can_confirmed_flag_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr attack_confirm_pub_;

    rclcpp::Publisher<msg_set_msgs::msg::TargetsPose>::SharedPtr targetsGps_pub_;

    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr satellite_pub_;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr videoRecordState_pub_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr lightState_pub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ptzState_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr px4Pos_pub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr px4Vel_pub_;

    rclcpp::Publisher<msg_set_msgs::msg::UsmDataInfo>::SharedPtr usmData_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shipRecord_pub_;

    /** ros pub topic **/
    std::string robotMajorState_pub_topic_;

    std::string globalPositionGps_pub_topic_;

    std::string mavrosState_pub_topic_;

    std::string taskIndex_pub_topic_;

    std::string pastIndex_pub_topic_;

    std::string can_confirmed_flag_pub_topic_;

    std::string attack_confirm_pub_topic_;

    std::string targetsGps_pub_topic_;

    std::string satellite_pub_topic_;

    std::string battery_pub_topic_;

    std::string videoRecordState_pub_topic_;

    std::string lightState_pub_topic_;

    std::string ptzState_pub_topic_;

    std::string px4Pos_pub_topic_;

    std::string px4Vel_pub_topic_;

    std::string usmData_pub_topic_;

    std::string shipRecord_pub_topic_;

    rclcpp::TimerBase::SharedPtr my_Timer_;
    rclcpp::Time last_message_time_;
    rclcpp::Duration timeout_duration_;
    bool is_recv_origin_px4_state_;

public:
    void robotMajorStateCallback(const std_msgs::msg::Int32::SharedPtr msg);

    void mavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg);

    void globalPositionGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void taskIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg);

    void pastIndexCallback(const msg_set_msgs::msg::CoverIndex::SharedPtr msg);

    void canConfirmedFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void attackConfirmCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void targetsGpsCallback(const msg_set_msgs::msg::TargetsPose::SharedPtr msg);

    void satelliteCallback(const std_msgs::msg::UInt32::SharedPtr msg);

    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

    void videoRecordStateCallback(const std_msgs::msg::Int8::SharedPtr msg);

    void lightStateCallback(const std_msgs::msg::Int8::SharedPtr msg);

    void ptzStateCallback(const std_msgs::msg::Int32::SharedPtr msg);

    void px4PosCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void px4VelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    void usmDataCallback(const msg_set_msgs::msg::UsmDataInfo::SharedPtr msg);

    void shipRecordCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void timerCallback();

public:
    BackendNode(const std::string &prefix, const std::string &curr_drone_id, int vehicle_id,
                double pub_rate, std::string node_name, std::string node_namespace, bool intra_process_comms = false);

    BackendNode(const std::string &node_name, bool intra_process_comms = false);

    ~BackendNode();

    void backendnode_init();
};

#endif // BACKEND_ROS_BACKEND_NODE_H