#include "thing_gateway/product_mqtt_topics.hpp"

// thing/product/{sn}/… 完整 topic 字符串
namespace thing_gateway {

std::string ProductMqttTopics::events() const { return "thing/product/" + sn + "/events"; }

std::string ProductMqttTopics::services() const { return "thing/product/" + sn + "/services"; }

std::string ProductMqttTopics::services_reply() const { return "thing/product/" + sn + "/services_reply"; }

std::string ProductMqttTopics::osd() const { return "thing/product/" + sn + "/osd"; }

} // namespace thing_gateway
