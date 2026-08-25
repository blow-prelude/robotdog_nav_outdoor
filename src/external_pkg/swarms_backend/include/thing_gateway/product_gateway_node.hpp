// ProductGatewayNode：MQTT thing/product/{sn}/… 与 ROS 感知侧桥接（话题前缀可配，见参数 thing_gateway.ros_topic_*）。
// 协议与话题约定见 docs/thing_gateway_业务与数据流.md 及《无人船》thing/product 相关章节。
//
// 线程：`message_arrived` 等 Paho 回调多在 MQTT 客户端线程，非 ROS executor；services 分派与经 ProductMqttRosBridge 的 ROS publish
// 可能发生在该路径。`active_task_bid_` 由 task_mu_ 保护。若要求「仅 executor 内碰 ROS」，需对入站消息做队列化。
//

#ifndef THING_GATEWAY_PRODUCT_GATEWAY_NODE_HPP
#define THING_GATEWAY_PRODUCT_GATEWAY_NODE_HPP

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <mqtt/async_client.h>
#include <rclcpp/rclcpp.hpp>

#include "thing_gateway/product_mqtt_topics.hpp"
#include "thing_gateway/product_mqtt_topic_router.hpp"
#include "thing_gateway/product_thing_services_dispatcher.hpp"
#include "thing_gateway/thing_json/types.hpp"

namespace thing_gateway {

// 前向声明，减少头文件依赖
class ProductMqttRosBridge;

class ProductGatewayNode : public rclcpp::Node, public virtual mqtt::callback, public virtual mqtt::iaction_listener {
public:
    using MessageHandler = ProductMqttTopicRouter::MessageHandler;
    using MessageHandlerWithProps = ProductMqttTopicRouter::MessageHandlerWithProps;

    explicit ProductGatewayNode(bool intra_process_comms, ProductThingServicesDispatcher &thing_services,
                                ProductMqttTopicRouter &mqtt_topic_router);
    ~ProductGatewayNode() override;

    /// 连接 Broker（失败仅打日志，不终止进程）。
    void connect();

    bool isConnected();

    /// 从配置文件读取设备 SN（如果文件不存在或读取失败，返回空字符串）
    static std::string read_sn_from_file(const std::string &file_path);

    void connected(const std::string &cause) override;
    void connection_lost(const std::string &cause) override;
    void message_arrived(mqtt::const_message_ptr msg) override;
    void delivery_complete(mqtt::delivery_token_ptr token) override;

    void on_success(const mqtt::token &tok) override;
    void on_failure(const mqtt::token &tok) override;

    /// 登记 topic → handler 并向 Broker 订阅；QoS 由参数指定（services 用 services_subscribe_qos_）。
    void subscribe(const std::string &topic, MessageHandler handler, int qos);

    /// MQTT 5：带 properties 的订阅；内部 subscribe QoS 固定为 1。
    void subscribeWithProps(const std::string &topic, MessageHandlerWithProps handler);

    /// 异步 publish，不阻塞等待 Paho token。
    bool publish_mqtt(const std::string &topic, const std::string &payload, int qos);

    /// 同 publish_mqtt，附加 MQTT 5 properties（如 User Property）。
    bool publish_mqtt_with_props(const std::string &topic, const std::string &payload, int qos, const mqtt::properties& props);

private:
    void product_mqtt_init();

    void register_product_service_handler_topic();

    /// 填充分派上下文（新增 ROS 出口时只改此处）。
    void fill_message_dispatch_context(ProductMessageDispatchContext &ctx);

    std::string get_active_task_bid() const;
    void apply_task_session(int task_action, const std::string &bid);

    std::atomic<bool> is_connected_{false};
    std::atomic<bool> is_shutting_down_{false};
    bool relax_device_sn_{false};

    thing_json::DeviceIdentity device_;
    ProductMqttTopics mqtt_topics_;

    std::string mqtt_address_;
    std::string mqtt_username_;
    std::string mqtt_passwd_;
    int mqtt_timeout_{60};
    int mqtt_keep_alive_interval_{100};
    int mqtt_max_inflight_{100};
    int vehicle_id_{0};
    std::string client_id_;

    mutable std::mutex task_mu_;
    std::string active_task_bid_;

    std::shared_ptr<mqtt::async_client> client_;

    ProductThingServicesDispatcher &thing_services_;
    ProductMqttTopicRouter &mqtt_topic_router_;
    MessageHandler cached_services_handler_;
    bool services_handler_ready_{false};

    /// 与 `mqtt_bridge_` 配套：MQTT/析构路径上对 bridge 的访问与 `reset` 互斥，避免 TOCTOU。
    mutable std::mutex bridge_mu_;
    std::unique_ptr<ProductMqttRosBridge> mqtt_bridge_;
    rclcpp::QoS qos_default_;
    int services_subscribe_qos_{1};
};

} // namespace thing_gateway

#endif // THING_GATEWAY_PRODUCT_GATEWAY_NODE_HPP
