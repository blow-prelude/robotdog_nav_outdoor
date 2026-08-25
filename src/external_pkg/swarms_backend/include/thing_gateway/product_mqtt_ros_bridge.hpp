// ROS 侧：订阅感知输出与 events_bridge，发布控制类话题；MQTT 侧 osd/events 经构造函数注入的 mqtt_publish 回调发出。
// payload_status 组包见 thing_json::build_payload_status（协议见项目文档及《无人船》载荷/OSD 章节）。
//

#ifndef THING_GATEWAY_PRODUCT_MQTT_ROS_BRIDGE_HPP
#define THING_GATEWAY_PRODUCT_MQTT_ROS_BRIDGE_HPP

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include "thing_gateway/thing_json/types.hpp"

namespace thing_gateway {

/// 可经 ROS 参数覆盖的完整话题名（感知 input/output；MQTT services 下行由网关映射到同前缀话题）。
struct ProductMqttRosTopics {
    std::string rtsp_flag;
    /// input：Float32MultiArray，8 元 [home,pos_flag,roll,pitch,yaw,zoom_flag,zoom,wiper]。
    /// MQTT 非回中：home=−1，pos_flag=−2，三槽为 roll_rate/pitch_rate/yaw_rate；MQTT `reset:true` 独占时 home=1，pos_flag 仍为 −2。
    std::string ptz_ctrl;
    std::string lamp_ctrl;
    /// output：Int8，灯状态回传，上送 payload_type=3（云台 OSD 不含 lamp，灯状态以此 topic 为准）。
    std::string lamp_status;
    /// input：Int8 喊话器控制，1=开喊、-1=停；话题 megaphone_ctrl（与 lamp_ctrl 独立）。MQTT payload_control payload_type=4。
    std::string megaphone_ctrl;
    /// output：Int8，喊话器状态回传，上送 payload_type=4 OSD。
    std::string megaphone_status;
    std::string video_record_ctrl;
    /// 发布 `std_msgs/Bool`：true=开启告警，false=关闭告警（MQTT `alarm_set` 下行映射；默认 `/{usv_sn}/gateway/from_mqtt/alarm_set`）。
    std::string from_mqtt_alarm_set;
    /// input：std_msgs/Int8MultiArray；**[-1]** 全检；否则为类别 ID 列表（与 MQTT `enabled_types` 映射一致）。
    std::string label_filter;
    /// output：Float32MultiArray，六元 [roll,pitch,yaw,zoom,wiper,lamp]（本地感知）；MQTT `payload_type=1` 仅上报前五维 + `state`，**不含 `lamp`**。
    std::string ptz_status;
    std::string video_record_state;
    /// input：std_msgs/Bool，经网关发 MQTT `thing/product/{sn}/events`，`method`=`alarm_state`；默认订阅 `/{usv_sn}/gateway/to_mqtt/events`。
    std::string events_bridge;
};

/// OSD 上云节流：秒；0 表示不节流（每次回调都尝试发 MQTT）。
struct ProductMqttRosBridgeOptions {
    std::chrono::seconds gimbal_osd_min_interval{2};
    std::chrono::seconds camera_osd_min_interval{5};
    std::chrono::seconds lamp_osd_min_interval{5};
    std::chrono::seconds megaphone_osd_min_interval{2};
};

class ProductMqttRosBridge {
public:
    ProductMqttRosBridge(rclcpp::Node &node, thing_json::DeviceIdentity device,
                         const ProductMqttRosTopics &ros_topics,
                         rclcpp::QoS qos_default, const std::string &mqtt_events_topic,
                         const std::string &mqtt_osd_topic,
                         std::function<bool(const std::string &topic, const std::string &payload, int qos)> mqtt_publish,
                         std::function<std::string()> get_task_bid, const ProductMqttRosBridgeOptions &io_opts = {});

    void publish_rtsp_flag(float v);
    /// std_msgs/Bool：true=开启告警，false=取消告警。
    void publish_alert_to_ros(bool alert_on);
    void publish_label_filter(const std::vector<int8_t> &types);
    void publish_ptz_ctrl(const std::vector<float> &data);
    void publish_lamp_ctrl(int8_t v);
    void publish_megaphone_ctrl(int8_t v);
    void publish_video_record_ctrl(int8_t v);

    /// 通知 bridge 即将析构，停止所有操作
    void shutdown();

private:
    void on_ptz(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void on_vrec(const std_msgs::msg::Int8::SharedPtr msg);
    void on_lamp(const std_msgs::msg::Int8::SharedPtr msg);
    void on_megaphone_status(const std_msgs::msg::Int8::SharedPtr msg);
    void on_events_bridge(const std_msgs::msg::Bool::SharedPtr msg);

    /// 组 payload_status JSON，经 mqtt_publish_ 发到构造时传入的 osd topic（一般为 thing/product/{sn}/osd）。
    void try_publish_gimbal_payload_status();
    void try_publish_camera_payload_status();
    void try_publish_lamp_payload_status();
    void try_publish_megaphone_payload_status();

    rclcpp::Node &node_;
    /// 与节点析构配合：为 true 时订阅回调与对外 publish 均短路返回
    std::atomic<bool> is_shutdown_{false};
    thing_json::DeviceIdentity device_;
    ProductMqttRosTopics ros_topics_;
    rclcpp::QoS qos_default_;
    std::string mqtt_events_topic_;
    std::string mqtt_osd_topic_;
    std::function<bool(const std::string &, const std::string &, int)> mqtt_publish_;
    std::function<std::string()> get_task_bid_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rtsp_flag_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alert_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr label_filter_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ptz_ctrl_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr lamp_ctrl_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr megaphone_ctrl_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr video_record_ctrl_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ptz_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr vrec_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr lamp_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr megaphone_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr events_bridge_sub_;

    std::mutex mu_;
    std::vector<float> ptz_cache_;
    bool has_ptz_{false};
    int8_t video_rec_state_{0};
    bool has_vrec_{false};
    int8_t lamp_state_{0};
    bool has_lamp_{false};
    int8_t megaphone_state_{0};
    bool has_megaphone_{false};

    std::mutex throttle_mu_;
    std::chrono::seconds gimbal_throttle_;
    std::chrono::seconds camera_throttle_;
    std::chrono::seconds lamp_throttle_;
    std::chrono::seconds megaphone_throttle_;
    std::chrono::steady_clock::time_point last_gimbal_pub_{};
    std::chrono::steady_clock::time_point last_camera_pub_{};
    std::chrono::steady_clock::time_point last_lamp_pub_{};
    std::chrono::steady_clock::time_point last_megaphone_pub_{};
};

} // namespace thing_gateway

#endif // THING_GATEWAY_PRODUCT_MQTT_ROS_BRIDGE_HPP
