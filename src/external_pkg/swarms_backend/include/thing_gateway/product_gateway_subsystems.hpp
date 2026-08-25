// ProductGatewaySubsystems：网关节点外的可复用子系统。
// 包含 services 分派表 + MQTT 入站 topic 表。
// 与 ProductGatewayNode 组合使用时须先构造本对象并保证其生命周期覆盖节点。

#ifndef THING_GATEWAY_PRODUCT_GATEWAY_SUBSYSTEMS_HPP
#define THING_GATEWAY_PRODUCT_GATEWAY_SUBSYSTEMS_HPP

#include "thing_gateway/product_thing_services_dispatcher.hpp"
#include "thing_gateway/product_mqtt_topic_router.hpp"

namespace thing_gateway {

/// 网关节点外的可复用子系统：services 分派表 + MQTT 入站 topic 表。
/// 与 ProductGatewayNode 组合使用时须先构造本对象并保证其生命周期覆盖节点。
class ProductGatewaySubsystems {
public:
    void install_builtin_handlers() { register_builtin_product_handlers(thing_services_); }

    ProductThingServicesDispatcher &thing_services() { return thing_services_; }
    ProductMqttTopicRouter &mqtt_topic_router() { return mqtt_topic_router_; }

private:
    ProductThingServicesDispatcher thing_services_;
    ProductMqttTopicRouter mqtt_topic_router_;
};

} // namespace thing_gateway

#endif // THING_GATEWAY_PRODUCT_GATEWAY_SUBSYSTEMS_HPP
