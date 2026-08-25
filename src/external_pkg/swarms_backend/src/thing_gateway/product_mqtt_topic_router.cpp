#include "thing_gateway/product_mqtt_topic_router.hpp"

#include <rclcpp/rclcpp.hpp>

namespace thing_gateway {

void ProductMqttTopicRouter::subscribePlain(mqtt::async_client &client, const rclcpp::Logger &logger,
                                                  const std::string &topic, MessageHandler handler, int qos,
                                                  const std::atomic<bool> &is_connected) {
    if (topic.empty()) {
        RCLCPP_ERROR_STREAM(logger, "[thing_gateway] subscribe: empty topic");
        return;
    }
    if (!is_connected.load(std::memory_order_acquire)) {
        RCLCPP_ERROR_STREAM(logger,
                            "[thing_gateway] MQTT client is not connected. Cannot subscribe to topic: " << topic);
        return;
    }
    try {
        {
            std::lock_guard<std::mutex> lk(mu_);
            plain_[topic] = std::move(handler);
        }
        mqtt::subscribe_options opts;
        opts.set_no_local(true);
        client.subscribe(topic, qos, opts);
        RCLCPP_INFO_STREAM(logger, "[thing_gateway] Subscribed to topic: " << topic << " (QoS=" << qos << ")");
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(logger, "[thing_gateway] MQTT exception during subscribe: " << e.what());
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(logger, "[thing_gateway] Exception during subscribe: " << e.what());
    }
}

void ProductMqttTopicRouter::subscribeWithProps(mqtt::async_client &client, const rclcpp::Logger &logger,
                                                      const std::string &topic, MessageHandlerWithProps handler,
                                                      const std::atomic<bool> &is_connected) {
    if (topic.empty()) {
        RCLCPP_ERROR_STREAM(logger, "[thing_gateway] subscribeWithProps: empty topic");
        return;
    }
    if (!is_connected.load(std::memory_order_acquire)) {
        RCLCPP_ERROR_STREAM(logger,
                            "[thing_gateway] MQTT client is not connected. Cannot subscribe to topic: " << topic);
        return;
    }
    try {
        {
            std::lock_guard<std::mutex> lk(mu_);
            with_props_[topic] = std::move(handler);
        }
        mqtt::subscribe_options opts;
        opts.set_no_local(true);
        mqtt::properties props;
        client.subscribe(topic, 1, opts, props);
        RCLCPP_INFO_STREAM(logger, "[thing_gateway] Subscribed to topic with props: " << topic);
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(logger, "[thing_gateway] MQTT exception during subscribeWithProps: " << e.what());
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(logger, "[thing_gateway] Exception during subscribeWithProps: " << e.what());
    }
}

void ProductMqttTopicRouter::deliver(mqtt::const_message_ptr mqtt_msg, const rclcpp::Logger &logger,
                                           const std::atomic<bool> &is_shutting_down) const {
    if (!mqtt_msg) {
        return;
    }
    if (is_shutting_down.load(std::memory_order_acquire)) {
        return;
    }

    const std::string topic = mqtt_msg->get_topic();
    const std::string message = mqtt_msg->to_string();
    const mqtt::properties props = mqtt_msg->get_properties();

    bool has_props_handler = false;
    MessageHandlerWithProps fn_props;
    MessageHandler fn;

    {
        std::lock_guard<std::mutex> lk(mu_);

        if (!props.empty()) {
            const auto handler_it = with_props_.find(topic);
            if (handler_it != with_props_.end()) {
                fn_props = handler_it->second;
                has_props_handler = true;
            }
        }

        if (!has_props_handler) {
            const auto handler_it = plain_.find(topic);
            if (handler_it == plain_.end()) {
                RCLCPP_WARN_STREAM(logger, "No handler registered for topic: " << topic);
                return;
            }
            fn = handler_it->second;
        }
    }

    if (is_shutting_down.load(std::memory_order_acquire)) {
        return;
    }

    try {
        if (has_props_handler) {
            fn_props(topic, message, props);
        } else {
            fn(topic, message);
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(logger, "Exception in message handler for topic " << topic << ": " << e.what());
    }
    catch (...) {
        RCLCPP_ERROR_STREAM(logger, "Unknown exception in message handler for topic " << topic);
    }
}

} // namespace thing_gateway
