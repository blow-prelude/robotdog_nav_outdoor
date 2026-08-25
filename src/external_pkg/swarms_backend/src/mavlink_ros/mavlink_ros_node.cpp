#include <iostream>
#include <stdio.h>
#include <time.h>
#include "mavlink_ros/mavlink_ros_interface.h"


using std::string;
using namespace std;


//int main(int argc, char **argv) {
//
//    rclcpp::init(argc, argv);
//    auto time = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
//    time_t timestamp = time.time_since_epoch().count();
//    boost::asio::io_service io_service;
//
//#ifdef NDEBUG
//    std::cout << "release --------------------------------------- " << std::endl;
//    // mqttnode yaml
//    std::string config_file_path = "/etc/mavlink_ros.yaml";
//    YAML::Node config;
//    try {
//        config = YAML::LoadFile(config_file_path);
//    } catch (const YAML::BadFile &e) {
//        std::cerr << "Failed to load config file: " << config_file_path << std::endl;
//        return -1;
//    }
//
//    std::string udp_remote_url = config["udp_remote_url"].as<std::string>("10.88.105.142:13550");
//    int udp_local_port = config["udp_local_port"].as<int>(14890);
//    std::string serial_url = config["serial_url"].as<std::string>("/dev/ttyUSB0");
//    int serial_baudrate = config["serial_baudrate"].as<int>(115200);
//    bool udp_use = config["udp_use"].as<bool>(true);
//    bool debug = config["debug"].as<bool>(false); // 是否显示消息原始数据
//    std::string prefix = config["prefix"].as<std::string>("drone_");
//    int vehicle_id = config["vehicle_id"].as<int>(0);
//
//    std::string curr_drone_id = "/" + prefix + std::to_string(vehicle_id);
//    std::string node_name = "mavlink_foxy_node_" + std::to_string(vehicle_id);
//    std::string node_namespace = prefix + std::to_string(vehicle_id);
//
//    auto api = std::make_shared<MavlinkRosInterface>(io_service,
//        curr_drone_id, prefix, vehicle_id, udp_remote_url, udp_local_port,
//        serial_url, serial_baudrate, udp_use, debug, node_name, node_namespace
//    );
//    api->start();   // 启动读写线程
//
//    rclcpp::executors::MultiThreadedExecutor executor;
//    executor.add_node(api);
//    std::thread io_thread([&io_service]() {
//        io_service.run();  // 启动 Boost.Asio 的事件循环
//    });
//    executor.spin();
//    api->stop();
//    io_service.stop(); // 停止 io_service 事件循环
//
//    if (io_thread.joinable()) {
//        io_thread.join(); // 等待 io_thread 完成
//    }
//    rclcpp::shutdown();
//
//#else
//    std::cout << "debug --------------------------------------- " << std::endl;
//
//    auto api = std::make_shared<MavlinkRosInterface>(io_service);
//    api->start();   // 启动读写线程
//
//
//    rclcpp::executors::MultiThreadedExecutor executor;
//    executor.add_node(api);
//    std::thread io_thread([&io_service]() {
//        io_service.run();  // 启动 Boost.Asio 的事件循环
//    });
//    executor.spin();
//    api->stop();
//    io_service.stop(); // 停止 io_service 事件循环
//
//    if (io_thread.joinable()) {
//        io_thread.join(); // 等待 io_thread 完成
//    }
//    rclcpp::shutdown();
//#endif
//    return 0;
//}