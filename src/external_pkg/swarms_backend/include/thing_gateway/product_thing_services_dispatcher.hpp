// MQTT services 入站：按 method 分派到已注册 handler，经 ctx 回 MQTT / ROS。
//

#ifndef THING_GATEWAY_PRODUCT_THING_SERVICES_DISPATCHER_HPP
#define THING_GATEWAY_PRODUCT_THING_SERVICES_DISPATCHER_HPP

#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

#include "thing_gateway/thing_json/types.hpp"
#include "thing_gateway/product_mqtt_topics.hpp"

namespace thing_gateway {

class ProductThingServicesDispatcher;

/// MQTT services 解析后映射到 ROS 的委托（与 `ProductMessageDispatchContext::mqtt_publish` 分离，便于扩展与单测桩替换）。
struct ProductRosDownstream {
    /// stream_start/stop → perception/input/rtsp_flag（std_msgs/Float32）。
    std::function<void(float)> publish_rtsp_flag;
    /// 告警开关下发 ROS：`std_msgs/Bool`，**true**=开启告警，**false**=关闭（与 MQTT `alarm_set` 的 `data.common.enabled` 一致）。
    std::function<void(bool alert_on)> publish_alert_to_ros;
    /// 当前任务周期 bid（可为空）；OSD/events 组包时若为空则不写根级 `bid`。
    std::function<std::string()> get_active_task_bid;
    /// Int8MultiArray：{-1} 全检，否则为类别 ID 列表（见 perception label_filter）。
    std::function<void(const std::vector<int8_t> &)> publish_label_filter;
    /// payload_type=1 → ptz_ctrl（Float32MultiArray，8 元）。
    std::function<void(const std::vector<float> &)> publish_ptz_ctrl;
    /// payload_type=3 → lamp_ctrl（Int8，1/-1）。
    std::function<void(int8_t)> publish_lamp_ctrl;
    /// payload_type=2 → video_record/ctrl_flag（Int8，1/-1）。
    std::function<void(int8_t)> publish_video_record_ctrl;
    /// payload_type=4 → megaphone_ctrl（Int8：1=开喊 / -1=停，话题与 lamp_ctrl 不同）。
    std::function<void(int8_t)> publish_megaphone_ctrl;
};

struct ProductMessageDispatchContext {
    /// 由 ProductGatewayNode::fill_message_dispatch_context 赋值为节点 logger。
    rclcpp::Logger logger{rclcpp::get_logger("thing_gateway_dispatch")};
    thing_json::DeviceIdentity device;
    ProductMqttTopics topics;
    /// 为 true 时：无 device/sn 也接受（兼容旧云端）；默认 false 须带 device.sn。
    bool relax_device_sn{false};

    /// 与节点共生的 thing/product **services** 分派器；`payload_control` 等 handler 通过其查询子表。非拥有指针。
    ProductThingServicesDispatcher *thing_services{nullptr};

    std::function<bool(const std::string &topic, const std::string &payload, int qos)> mqtt_publish;
    ProductRosDownstream ros;

    std::function<void(int task_action, const std::string &bid)> apply_task_session;
};

using ReplyErr = std::function<void(int32_t code)>;
using ReplyOk = std::function<void()>;

/// services_raw_payload：MQTT 原始正文；内置 handler 解析后映射到 ROS（std_msgs 等），不整包 MQTT 上 ROS。
using ProductServiceHandler =
        std::function<void(const ProductMessageDispatchContext &ctx, const nlohmann::json &root, const std::string &tid,
                           const std::string &bid, const std::string &method, const std::string &services_raw_payload,
                           const ReplyErr &reply_err, const ReplyOk &reply_ok)>;

/// payload_control 子类型处理器（control_params 已解析，payload_type 已验证）
using PayloadControlHandler = std::function<void(const ProductMessageDispatchContext &ctx, const nlohmann::json &control_params,
                                                 const ReplyErr &reply_err, const ReplyOk &reply_ok)>;

/// thing/product **services** JSON：按 `method` 分派 + `payload_control` 的 `payload_type` 子表；线程安全。
class ProductThingServicesDispatcher {
public:
    ProductThingServicesDispatcher() = default;
    ProductThingServicesDispatcher(const ProductThingServicesDispatcher &) = delete;
    ProductThingServicesDispatcher &operator=(const ProductThingServicesDispatcher &) = delete;

    /// 后注册覆盖同名 method（便于测试或现场打补丁）。
    void registerServiceHandler(const std::string &method, ProductServiceHandler handler);
    /// 后注册覆盖同 payload_type。
    void registerPayloadControlHandler(int payload_type, PayloadControlHandler handler);

    /// 供 `payload_control` 总 handler 查询子处理器；未注册返回空 function。
    [[nodiscard]] PayloadControlHandler findPayloadControlHandler(int payload_type) const;

    /// 解析 JSON → 校验信封与 SN → 按 method 调用已注册 handler。
    void dispatch(const ProductMessageDispatchContext &ctx, const std::string &payload) const;

private:
    mutable std::mutex mu_;
    std::map<std::string, ProductServiceHandler> by_method_;
    std::map<int, PayloadControlHandler> by_payload_type_;
};

/// 内置 handler 注册到指定分派器（实现位于 handlers/*.cpp）。
void register_alert_task_handlers(ProductThingServicesDispatcher &d);
void register_perception_handlers(ProductThingServicesDispatcher &d);
void register_payload_control_handlers(ProductThingServicesDispatcher &d);
/// 依次调用上述三者；须在创建依赖分派的 `ProductGatewayNode` 之前调用一次。
void register_builtin_product_handlers(ProductThingServicesDispatcher &d);

} // namespace thing_gateway

#endif // THING_GATEWAY_PRODUCT_THING_SERVICES_DISPATCHER_HPP
