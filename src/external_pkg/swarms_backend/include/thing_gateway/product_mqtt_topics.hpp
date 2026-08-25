// thing/product/{sn}/ + events | services | services_reply | osd（sn 由调用方写入 mqtt_topics_.sn）
// 云端 services 各 method → ROS 话题、osd/events QoS 等：见 docs/thing_gateway_业务与数据流.md
//

#ifndef THING_GATEWAY_PRODUCT_MQTT_TOPICS_HPP
#define THING_GATEWAY_PRODUCT_MQTT_TOPICS_HPP

#include <string>

namespace thing_gateway {

struct ProductMqttTopics {
    std::string sn;

    std::string events() const;
    std::string services() const;
    std::string services_reply() const;
    std::string osd() const;
};

} // namespace thing_gateway

#endif // THING_GATEWAY_PRODUCT_MQTT_TOPICS_HPP
