#ifndef THING_GATEWAY_PRODUCT_MQTT_TOPIC_ROUTER_HPP
#define THING_GATEWAY_PRODUCT_MQTT_TOPIC_ROUTER_HPP

#include <atomic>
#include <functional>
#include <map>
#include <mutex>
#include <string>

#include <mqtt/async_client.h>
#include <rclcpp/logger.hpp>

namespace thing_gateway {

/// MQTT 入站：按 **topic** 把消息交给已订阅时登记的回调（与 `async_client::subscribe` 同步维护表）。
class ProductMqttTopicRouter {
public:
    using MessageHandler = std::function<void(const std::string &topic, const std::string &payload)>;
    using MessageHandlerWithProps =
            std::function<void(const std::string &topic, const std::string &payload, const mqtt::properties &props)>;

    void subscribePlain(mqtt::async_client &client, const rclcpp::Logger &logger, const std::string &topic,
                        MessageHandler handler, int qos, const std::atomic<bool> &is_connected);

    void subscribeWithProps(mqtt::async_client &client, const rclcpp::Logger &logger, const std::string &topic,
                            MessageHandlerWithProps handler, const std::atomic<bool> &is_connected);

    /// Paho 回调路径：按 topic / MQTT5 properties 选择 handler 并调用。
    void deliver(mqtt::const_message_ptr mqtt_msg, const rclcpp::Logger &logger,
                 const std::atomic<bool> &is_shutting_down) const;

private:
    mutable std::mutex mu_;
    std::map<std::string, MessageHandler> plain_;
    std::map<std::string, MessageHandlerWithProps> with_props_;
};

} // namespace thing_gateway

#endif // THING_GATEWAY_PRODUCT_MQTT_TOPIC_ROUTER_HPP
