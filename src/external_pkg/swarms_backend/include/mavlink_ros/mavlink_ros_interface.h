#ifndef MAVLINK_ROS_INTERFACE_H_
#define MAVLINK_ROS_INTERFACE_H_
struct BatteryState;
struct Satellite;

#include <sys/socket.h>
#include <netinet/in.h>
#include <net/if.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <sys/ioctl.h>
#include "generic_port.h"
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <msg_set_msgs/msg/multi_task.hpp>
#include <msg_set_msgs/msg/jump_task.hpp>
#include <msg_set_msgs/msg/cover_param.hpp>
#include <msg_set_msgs/msg/cover_task.hpp>
#include <msg_set_msgs/msg/cover_index.hpp>
#include <msg_set_msgs/msg/global_param.hpp>
#include <msg_set_msgs/msg/multi_task_confirm.hpp>
#include <msg_set_msgs/msg/cover_confirm.hpp>
#include <msg_set_msgs/msg/targets_pose.hpp>
#include <msg_set_msgs/msg/usm_data_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include "mavlink_v2/common/mavlink.h"
#include "mavlink_v2/minimal/mavlink.h"
#include "mavlink_v2/standard/mavlink.h"
#include <regex>
#include <sys/time.h>
#include "serial_port.h"
#include "udp_port.h"
#include "uav_state.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <Eigen/Core>

void *start_mavlink_ros_interface_read_thread(void *args);

void *start_mavlink_ros_interface_write_thread(void *args);

class MavlinkRosInterface : public rclcpp::Node {
public:
    MavlinkRosInterface(boost::asio::io_service &io_service,
                        const std::string &curr_drone_id,
                        const std::string &prefix,
                        int vehicle_id,
                        const std::string &udp_remote_url,
                        int udp_local_port,
                        const std::string &serial_url,
                        int serial_baudrate,
                        bool udp_use,
                        bool debug,
                        const std::string &node_name,
                        const std::string &node_namespace, bool intra_process_comms = false);

    MavlinkRosInterface(boost::asio::io_service &io_service, const std::string &node_name,
                        bool intra_process_comms = false);

    ~MavlinkRosInterface();

    void init();

    void start();

    void stop();

    void stop_port();

    void start_read_thread();

    void start_write_thread(void);

    void uavStateOutputTimerCallback();

private:
    void read_thread();

    void write_thread(void);

    void read_messages();

    int write_message(mavlink_message_t message);

    void add_message_to_buff(mavlink_message_t &msg);

    void start_port(boost::asio::io_service &io_service);

    void statePX4Callback(const mavros_msgs::msg::State::SharedPtr msg);

    void robotMajorStateCallback(const std_msgs::msg::Int32::SharedPtr msg);

    void currGlobalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void targetsCallback(const msg_set_msgs::msg::TargetsPose::SharedPtr msg);

    void taskIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg);

    void pastIndexCallback(const msg_set_msgs::msg::CoverIndex::SharedPtr msg);

    void canConfirmedFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void attackConfirmCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void shipRecordCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void satelliteCallback(const std_msgs::msg::UInt32::SharedPtr msg);

    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

    void videoRecordStateCallback(const std_msgs::msg::Int8::SharedPtr msg);

    void lightStateCallback(const std_msgs::msg::Int8::SharedPtr msg);

    void ptzStateCallback(const std_msgs::msg::Int32::SharedPtr msg);

    void px4PosCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void px4VelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    void usmDataCallback(const msg_set_msgs::msg::UsmDataInfo::SharedPtr msg);

private:
    int system_id;
    int companion_id;

    std::unique_ptr<Generic_Port> port;

    rclcpp::QoS qos_sensor;
    rclcpp::QoS qos_default;

    std::mutex mtx; // 互斥锁
    std::queue<mavlink_message_t> buff;
    bool time_to_exit;
    char reading_status;
    char writing_status;
    pthread_t read_tid;
    pthread_t write_tid;

    int vehicle_id_;
    std::string prefix_;
    std::string curr_drone_id_;

    std::string udp_remote_url_ = "10.88.105.142:13550";
    int udp_local_port_ = 14890;
    std::string serial_url_ = "/dev/ttyUSB0";
    int serial_baudrate_ = 115200;
    std::string port_url_;
    bool udp_use_ = true;
    bool debug_ = true; // 是否显示消息原始数据

    /** mqtt 无人机消息汇总 **/
    RobotMajorState robot_major_state_; // 无人机主状态消息
    GlobalPosition global_position_;    // 无人机全局位置消息
    PX4State px4_state_;                // 无人机PX4状态消息
    UAVState uav_state_;                // 无人机汇总消息
    Satellite satellite_;
    BatteryState batteryState_;
    PX4Pos px4Pos_;
    PX4Vel px4Vel_;
    UsmDataInfo usmDataInfo_;
    std::mutex px4Vel_mutex_;
    std::mutex px4Pos_mutex_;
    std::mutex usmData_mutex_;

    std::mutex batteryState_mutex_;
    std::mutex satellite_mutex_;
    std::mutex robot_major_state_mutex_;
    std::mutex px4_state_mutex_;
    std::mutex global_position_mutex_;

    /** ros sub topic **/
    std::string robotMajorState_sub_topic_;
    std::string globalPositionGps_sub_topic_;
    std::string statePX4_sub_topic_;
    std::string taskIndex_sub_topic_;
    std::string pastIndex_sub_topic_;
    std::string canConfirmedFlag_sub_topic_;
    std::string track_img_sub_topic_;
    std::string targets_sub_topic_;
    std::string attack_confirm_sub_topic_;
    std::string satellite_sub_topic_;
    std::string battery_sub_topic_;
    std::string videoRecordState_sub_topic_;
    std::string lightState_sub_topic_;
    std::string ptzState_sub_topic_;
    std::string px4Pos_sub_topic_;
    std::string px4Vel_sub_topic_;
    std::string usmData_sub_topic_;

    std::string shipRecored_sub_topic_;

    rclcpp::TimerBase::SharedPtr uav_state_output_timer_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robotMajorState_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr globalPosition_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr statePX4_sub_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr taskIndex_sub_;
    rclcpp::Subscription<msg_set_msgs::msg::CoverIndex>::SharedPtr pastIndex_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr canConfirmedFlag_sub_;
    rclcpp::Subscription<msg_set_msgs::msg::TargetsPose>::SharedPtr targets_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr attack_confirm_sub_;

    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr satellite_sub_;

    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr videoRecordState_sub_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr lightState_sub_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ptzState_sub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr px4Pos_sub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr px4Vel_sub_;

    rclcpp::Subscription<msg_set_msgs::msg::UsmDataInfo>::SharedPtr usmData_sub_;


    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shipRecored_sub_;
};

#endif
