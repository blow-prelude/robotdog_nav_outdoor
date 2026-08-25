#include "thing_gateway/product_mqtt_ros_bridge.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>

#include "thing_gateway/thing_json/serialize.hpp"
#include "thing_gateway/thing_json/types.hpp"

// ROS 订阅（ptz/录像/events）与发布（控制/告警等）；MQTT osd/events 经 mqtt_publish_ 回调
namespace thing_gateway {

using thing_json::json;

ProductMqttRosBridge::ProductMqttRosBridge(rclcpp::Node &node, thing_json::DeviceIdentity device,
                                           const ProductMqttRosTopics &ros_topics,
                                           rclcpp::QoS qos_default, const std::string &mqtt_events_topic,
                                           const std::string &mqtt_osd_topic,
                                           std::function<bool(const std::string &topic, const std::string &payload, int qos)>
                                                   mqtt_publish,
                                           std::function<std::string()> get_task_bid,
                                           const ProductMqttRosBridgeOptions &io_opts)
    : node_(node), device_(std::move(device)), ros_topics_(ros_topics), qos_default_(qos_default),
      mqtt_events_topic_(mqtt_events_topic), mqtt_osd_topic_(mqtt_osd_topic), mqtt_publish_(std::move(mqtt_publish)),
      get_task_bid_(std::move(get_task_bid)), gimbal_throttle_(io_opts.gimbal_osd_min_interval),
      camera_throttle_(io_opts.camera_osd_min_interval), lamp_throttle_(io_opts.lamp_osd_min_interval),
      megaphone_throttle_(io_opts.megaphone_osd_min_interval) {

    rtsp_flag_pub_ = node_.create_publisher<std_msgs::msg::Float32>(ros_topics_.rtsp_flag, qos_default_);

    ptz_ctrl_pub_ =
            node_.create_publisher<std_msgs::msg::Float32MultiArray>(ros_topics_.ptz_ctrl, qos_default_);
    lamp_ctrl_pub_ = node_.create_publisher<std_msgs::msg::Int8>(ros_topics_.lamp_ctrl, qos_default_);
    megaphone_ctrl_pub_ =
            node_.create_publisher<std_msgs::msg::Int8>(ros_topics_.megaphone_ctrl, qos_default_);
    video_record_ctrl_pub_ =
            node_.create_publisher<std_msgs::msg::Int8>(ros_topics_.video_record_ctrl, qos_default_);

    alert_pub_ =
            node_.create_publisher<std_msgs::msg::Bool>(ros_topics_.from_mqtt_alarm_set, qos_default_);

    label_filter_pub_ =
            node_.create_publisher<std_msgs::msg::Int8MultiArray>(ros_topics_.label_filter, qos_default_);

    ptz_sub_ = node_.create_subscription<std_msgs::msg::Float32MultiArray>(
            ros_topics_.ptz_status, qos_default_,
            std::bind(&ProductMqttRosBridge::on_ptz, this, std::placeholders::_1));

    vrec_sub_ = node_.create_subscription<std_msgs::msg::Int8>(
            ros_topics_.video_record_state, qos_default_,
            std::bind(&ProductMqttRosBridge::on_vrec, this, std::placeholders::_1));

    lamp_sub_ = node_.create_subscription<std_msgs::msg::Int8>(
            ros_topics_.lamp_status, qos_default_,
            std::bind(&ProductMqttRosBridge::on_lamp, this, std::placeholders::_1));

    megaphone_status_sub_ = node_.create_subscription<std_msgs::msg::Int8>(
            ros_topics_.megaphone_status, qos_default_,
            std::bind(&ProductMqttRosBridge::on_megaphone_status, this, std::placeholders::_1));

    events_bridge_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
            ros_topics_.events_bridge, qos_default_,
            std::bind(&ProductMqttRosBridge::on_events_bridge, this, std::placeholders::_1));
}

void ProductMqttRosBridge::publish_rtsp_flag(float v) {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    std_msgs::msg::Float32 m;
    m.data = v;
    rtsp_flag_pub_->publish(m);
}

void ProductMqttRosBridge::publish_alert_to_ros(bool alert_on) {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    std_msgs::msg::Bool m;
    m.data = alert_on;
    alert_pub_->publish(m);
}

void ProductMqttRosBridge::publish_label_filter(const std::vector<int8_t> &types) {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    std_msgs::msg::Int8MultiArray m;
    m.data.assign(types.begin(), types.end());
    label_filter_pub_->publish(m);
}

void ProductMqttRosBridge::publish_ptz_ctrl(const std::vector<float> &data) {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    std_msgs::msg::Float32MultiArray m;
    m.data.assign(data.begin(), data.end());
    ptz_ctrl_pub_->publish(m);
}

void ProductMqttRosBridge::publish_lamp_ctrl(int8_t v) {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    std_msgs::msg::Int8 m;
    m.data = v;
    lamp_ctrl_pub_->publish(m);
}

void ProductMqttRosBridge::publish_megaphone_ctrl(int8_t v) {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    std_msgs::msg::Int8 m;
    m.data = v;
    megaphone_ctrl_pub_->publish(m);
}

void ProductMqttRosBridge::publish_video_record_ctrl(int8_t v) {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    std_msgs::msg::Int8 m;
    m.data = v;
    video_record_ctrl_pub_->publish(m);
}

void ProductMqttRosBridge::shutdown() {
    is_shutdown_.store(true, std::memory_order_release);
}

namespace {
constexpr std::size_t kPtzStatusFloats = 6;
} // namespace

void ProductMqttRosBridge::on_ptz(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    if (!msg) {
        return;
    }
    if (msg->data.size() != kPtzStatusFloats) {
        RCLCPP_WARN_THROTTLE(
                node_.get_logger(), *node_.get_clock(), 5000,
                "[thing_gateway] ptz_status: need exactly %zu floats [roll,pitch,yaw,zoom,wiper,lamp] (lamp 仅本地，MQTT 云台 OSD 不含 lamp), got %zu",
                kPtzStatusFloats, msg->data.size());
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mu_);
        ptz_cache_.assign(msg->data.begin(), msg->data.end());
        has_ptz_ = true;
    }
    try_publish_gimbal_payload_status();
}

void ProductMqttRosBridge::on_vrec(const std_msgs::msg::Int8::SharedPtr msg) {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    if (!msg) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mu_);
        video_rec_state_ = msg->data;
        has_vrec_ = true;
    }
    try_publish_camera_payload_status();
}

void ProductMqttRosBridge::on_lamp(const std_msgs::msg::Int8::SharedPtr msg) {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    if (!msg) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mu_);
        lamp_state_ = msg->data;
        has_lamp_ = true;
    }
    try_publish_lamp_payload_status();
}

void ProductMqttRosBridge::on_megaphone_status(const std_msgs::msg::Int8::SharedPtr msg) {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    if (!msg) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mu_);
        megaphone_state_ = msg->data;
        has_megaphone_ = true;
    }
    try_publish_megaphone_payload_status();
}

void ProductMqttRosBridge::on_events_bridge(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg || !mqtt_publish_ || is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    const std::string bid = get_task_bid_ ? get_task_bid_() : std::string();
    json common;
    common["enabled"] = msg->data;
    const std::string body = thing_json::build_product_event(device_, bid, common);
    mqtt_publish_(mqtt_events_topic_, body, 1);
}

void ProductMqttRosBridge::try_publish_gimbal_payload_status() {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    if (!mqtt_publish_) {
        return;
    }

    std::vector<float> ptz;
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!has_ptz_) {
            return;
        }
        ptz = ptz_cache_;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto gimbal_min = std::chrono::duration_cast<std::chrono::steady_clock::duration>(gimbal_throttle_);
    {
        std::lock_guard<std::mutex> lk(throttle_mu_);
        if (gimbal_throttle_.count() > 0 && last_gimbal_pub_.time_since_epoch().count() != 0 &&
            now - last_gimbal_pub_ < gimbal_min) {
            return;
        }
    }

    if (ptz.size() != kPtzStatusFloats) {
        return;
    }

    json attrs;
    attrs["state"] = 1;
    // 六元顺序：[roll, pitch, yaw, zoom, wiper, lamp]
    enum : std::size_t { kRoll = 0, kPitch = 1, kYaw = 2, kZoom = 3, kWiper = 4, kLamp = 5 };
    attrs["roll"] = static_cast<double>(ptz[kRoll]);
    attrs["pitch"] = static_cast<double>(ptz[kPitch]);
    attrs["yaw"] = static_cast<double>(ptz[kYaw]);
    attrs["zoom"] = static_cast<double>(ptz[kZoom]);
    attrs["wiper"] = static_cast<double>(ptz[kWiper]);
    (void)ptz[kLamp]; // 本地六元保留 lamp，接口文档 payload_type=1 不上报 lamp

    const std::string bid = get_task_bid_ ? get_task_bid_() : std::string();
    const std::string body = thing_json::build_payload_status(device_, bid, 1, attrs);
    if (mqtt_publish_(mqtt_osd_topic_, body, 0)) {
        std::lock_guard<std::mutex> lk(throttle_mu_);
        last_gimbal_pub_ = now;
    }
}

void ProductMqttRosBridge::try_publish_camera_payload_status() {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    if (!mqtt_publish_) {
        return;
    }

    int8_t rec = 0;
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!has_vrec_) {
            return;
        }
        rec = video_rec_state_;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto camera_min = std::chrono::duration_cast<std::chrono::steady_clock::duration>(camera_throttle_);
    {
        std::lock_guard<std::mutex> lk(throttle_mu_);
        if (camera_throttle_.count() > 0 && last_camera_pub_.time_since_epoch().count() != 0 &&
            now - last_camera_pub_ < camera_min) {
            return;
        }
    }

    json attrs;
    // 接口文档：摄像机 attributes 仅 state，1=录制中，0=未录制
    attrs["state"] = (rec == 1) ? 1 : 0;

    const std::string bid = get_task_bid_ ? get_task_bid_() : std::string();
    const std::string body = thing_json::build_payload_status(device_, bid, 2, attrs);
    if (mqtt_publish_(mqtt_osd_topic_, body, 0)) {
        std::lock_guard<std::mutex> lk(throttle_mu_);
        last_camera_pub_ = now;
    }
}

void ProductMqttRosBridge::try_publish_lamp_payload_status() {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    if (!mqtt_publish_) {
        return;
    }

    int8_t lamp = 0;
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!has_lamp_) {
            return;
        }
        lamp = lamp_state_;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto lamp_min = std::chrono::duration_cast<std::chrono::steady_clock::duration>(lamp_throttle_);
    {
        std::lock_guard<std::mutex> lk(throttle_mu_);
        if (lamp_throttle_.count() > 0 && last_lamp_pub_.time_since_epoch().count() != 0 &&
            now - last_lamp_pub_ < lamp_min) {
            return;
        }
    }

    json attrs;
    // 附录 D.1.1：MQTT 上 state 仅 1=开、0=关；ROS 侧关常为 Int8 -1，此处归一为 0
    attrs["state"] = (lamp == static_cast<int8_t>(1)) ? 1 : 0;

    const std::string bid = get_task_bid_ ? get_task_bid_() : std::string();
    const std::string body = thing_json::build_payload_status(device_, bid, 3, attrs);
    if (mqtt_publish_(mqtt_osd_topic_, body, 0)) {
        std::lock_guard<std::mutex> lk(throttle_mu_);
        last_lamp_pub_ = now;
    }
}

void ProductMqttRosBridge::try_publish_megaphone_payload_status() {
    if (is_shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    if (!mqtt_publish_) {
        return;
    }

    int8_t spk = 0;
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!has_megaphone_) {
            return;
        }
        spk = megaphone_state_;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto spk_min = std::chrono::duration_cast<std::chrono::steady_clock::duration>(megaphone_throttle_);
    {
        std::lock_guard<std::mutex> lk(throttle_mu_);
        if (megaphone_throttle_.count() > 0 && last_megaphone_pub_.time_since_epoch().count() != 0 &&
            now - last_megaphone_pub_ < spk_min) {
            return;
        }
    }

    json attrs;
    attrs["state"] = (spk == static_cast<int8_t>(1)) ? 1 : 0;

    const std::string bid = get_task_bid_ ? get_task_bid_() : std::string();
    const std::string body = thing_json::build_payload_status(device_, bid, 4, attrs);
    if (mqtt_publish_(mqtt_osd_topic_, body, 0)) {
        std::lock_guard<std::mutex> lk(throttle_mu_);
        last_megaphone_pub_ = now;
    }
}

} // namespace thing_gateway
