#include <rclcpp/rclcpp.hpp>
#include "mavlink_ros/mavlink_ros_interface.h"
#include "mqttnode/mqttnode.hpp"

#include "backend_ros/backend_node.hpp"
#include "thing_gateway/product_gateway_node.hpp"
#include "thing_gateway/product_gateway_subsystems.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    boost::asio::io_service io_service;

    auto backend_node = std::make_shared<BackendNode>("backend_node",true);
    auto mqtt_node = std::make_shared<MqttNode>("mqtt_node",true);

    thing_gateway::ProductGatewaySubsystems thing_gw;
    thing_gw.install_builtin_handlers();
    auto thing_gateway_node = std::make_shared<thing_gateway::ProductGatewayNode>(
            true, thing_gw.thing_services(), thing_gw.mqtt_topic_router());
    auto mavlink_node = std::make_shared<MavlinkRosInterface>(io_service,"mavlink_node",true);

    mqtt_node->connect();
    thing_gateway_node->connect();

    mavlink_node->start();   // 启动读写线程
    std::thread io_thread([&io_service]() {
        io_service.run();  // 启动 Boost.Asio 的事件循环
    });

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(backend_node);
    executor.add_node(mqtt_node);
    executor.add_node(thing_gateway_node);
    executor.add_node(mavlink_node);

    executor.spin();

    mavlink_node->stop();
    io_service.stop(); // 停止 io_service 事件循环
    if (io_thread.joinable()) {
        io_thread.join(); // 等待 io_thread 完成
    }

    rclcpp::shutdown();
    return 0;
}
