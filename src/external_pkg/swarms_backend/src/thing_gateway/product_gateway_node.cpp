#include "thing_gateway/product_gateway_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <utility>
#include <vector>

#include <mqtt/create_options.h>

#include "thing_gateway/product_thing_services_dispatcher.hpp"
#include "thing_gateway/product_mqtt_ros_bridge.hpp"

// thing_gateway 主节点：参数、MQTT v5 客户端、services 订阅与分派、ProductMqttRosBridge 生命周期
namespace thing_gateway {
namespace {

std::string trim(const std::string &str) {
    size_t first = str.find_first_not_of(" \t\n\r");
    if (std::string::npos == first) {
        return str;
    }
    size_t last = str.find_last_not_of(" \t\n\r");
    return str.substr(first, (last - first + 1));
}

int clamp_mqtt_qos(int q) {
    if (q < 0) {
        return 0;
    }
    if (q > 2) {
        return 2;
    }
    return q;
}

int clamp_mqtt_max_inflight(int v) {
    constexpr int k_min = 1;
    constexpr int k_max = 65535;
    if (v < k_min) {
        return k_min;
    }
    if (v > k_max) {
        return k_max;
    }
    return v;
}

} // namespace

ProductGatewayNode::ProductGatewayNode(bool intra_process_comms, ProductThingServicesDispatcher &thing_services,
                                       ProductMqttTopicRouter &mqtt_topic_router)
    : rclcpp::Node("thing_gateway_node", "",
                   rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
      qos_default_(intra_process_comms
                           ? rclcpp::QoS(rclcpp::KeepLast(10))
                                     .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                     .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                           : rclcpp::SystemDefaultsQoS()),
      thing_services_(thing_services),
      mqtt_topic_router_(mqtt_topic_router) {

    // SN：参数 usv_sn →（若空）文件 usv_sn_config_file →（若仍空）环境变量 USV_SN
    std::string sn_param = this->declare_parameter("usv_sn", std::string("dev"));
    const std::string sn_config_file = this->declare_parameter("usv_sn_config_file", std::string(""));

    // ========== 提前声明 prefix 和 vehicle_id（后续 get_parameter 需要） ==========
    const std::string prefix_param = this->declare_parameter("prefix", std::string("usv"));
    const int vehicle_id_param = this->declare_parameter("vehicle_id", 0);

    // ========== 处理 usv_sn 默认值 ==========
    // 注意：文件和环境变量读取代码暂时保留但不会执行（命令行参数优先）
    // 如果需要启用文件/环境变量，请删除下面的 if 判断
    if (sn_param.empty() || sn_param == "dev") {
        sn_param = prefix_param + std::to_string(vehicle_id_param);
        RCLCPP_INFO_STREAM(get_logger(),
            "[thing_gateway] No usv_sn provided, using vehicle-based identification: " << sn_param);
    } else {
        RCLCPP_INFO_STREAM(get_logger(),
            "[thing_gateway] Using usv_sn: " << sn_param);
    }

    if (sn_param.empty() && !sn_config_file.empty()) {
        sn_param = read_sn_from_file(sn_config_file);
        if (!sn_param.empty()) {
            RCLCPP_INFO_STREAM(get_logger(), "[thing_gateway] Read device SN from config file: " << sn_config_file);
        }
    }

    if (sn_param.empty()) {
        const char *env_sn = std::getenv("USV_SN");
        if (env_sn != nullptr && std::string(env_sn).length() > 0) {
            sn_param = std::string(env_sn);
            RCLCPP_INFO_STREAM(get_logger(), "[thing_gateway] Read device SN from environment variable USV_SN");
        }
    }

    device_.sn = sn_param;
    device_.name = this->declare_parameter("device_name", std::string(""));
    device_.vendor = this->declare_parameter("device_vendor", std::string(""));
    device_.type = static_cast<thing_json::DeviceType>(this->declare_parameter("device_type", 3));
    device_.sub_type = this->declare_parameter("device_sub_type", 0);

    this->declare_parameter("curr_drone_id", std::string("")); // launch 给其它节点用，thing 不拿来拼话题

    mqtt_address_ = this->declare_parameter("address", std::string("tcp://127.0.0.1:1883"));
    mqtt_username_ = this->declare_parameter("mqtt_username", std::string("admin"));
    mqtt_passwd_ = this->declare_parameter("mqtt_passwd", std::string("nanhu@2020"));
    mqtt_timeout_ = this->declare_parameter("mqtt_timeout", 60);
    mqtt_keep_alive_interval_ = this->declare_parameter("mqtt_keep_alive_interval", 100);
    mqtt_max_inflight_ = clamp_mqtt_max_inflight(this->declare_parameter("mqtt_max_inflight", 100));
    // 与 launch 共用参数名；本节点未使用，仅声明以免参数表缺项。
    this->declare_parameter("mqtt_buffer_size", 62914560);
    vehicle_id_ = this->get_parameter("vehicle_id").as_int();

    const auto time_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
    const uint64_t ts = time_ns.time_since_epoch().count();
    const uint32_t rand_part = static_cast<uint32_t>(ts ^ (ts >> 32));
    // client_id 需唯一：纳秒时间戳 + vehicle_id + 随机段，避免多进程同时连同一 broker 冲突
    client_id_ = std::to_string(ts) + "thinggw" + std::to_string(vehicle_id_) + "_" + std::to_string(rand_part);

    mqtt_topics_.sn = device_.sn;
    relax_device_sn_ = this->declare_parameter("thing_gateway.relax_device_sn", false);

    if (device_.sn.empty()) {
        RCLCPP_ERROR_STREAM(get_logger(),
            "[thing_gateway] DISABLED - Device SN is required!"
            "\nThing Product API requires a valid device SN to work."
            "\nPlease configure SN via one of these methods:"
            "\n  1. Set parameter: usv_sn:=YOUR_DEVICE_SN"
            "\n  2. Set parameter: usv_sn_config_file:=/path/to/sn.txt"
            "\n  3. Set environment variable: export USV_SN=YOUR_DEVICE_SN");
        return;
    }

    RCLCPP_INFO_STREAM(get_logger(), "[thing_gateway] initialized, services=" << mqtt_topics_.services());

    services_subscribe_qos_ =
            clamp_mqtt_qos(this->declare_parameter("thing_gateway.services_subscribe_qos", 1));

    // 使用 curr_drone_id 参数构建 ROS 话题前缀（已由 launch 文件计算好并保证合法）
    const std::string curr_drone_id = this->get_parameter("curr_drone_id").as_string();
    const std::string root = curr_drone_id;

    ProductMqttRosTopics ros_topics;
    ros_topics.rtsp_flag =
            this->declare_parameter("thing_gateway.ros_topic_rtsp_flag", root + "/perception/input/rtsp_flag");
    ros_topics.ptz_ctrl =
            this->declare_parameter("thing_gateway.ros_topic_ptz_ctrl", root + "/perception/input/ptz_ctrl");
    ros_topics.lamp_ctrl =
            this->declare_parameter("thing_gateway.ros_topic_lamp_ctrl", root + "/perception/input/lamp_ctrl");
    ros_topics.lamp_status =
            this->declare_parameter("thing_gateway.ros_topic_lamp_status", root + "/perception/output/lamp_status");
    ros_topics.megaphone_ctrl =
            this->declare_parameter("thing_gateway.ros_topic_megaphone_ctrl", root + "/usv_manager/output/megaphone_ctrl");
    ros_topics.megaphone_status = this->declare_parameter("thing_gateway.ros_topic_megaphone_status",
                                                         root + "/usv_manager/output/megaphone_status");
    ros_topics.video_record_ctrl = this->declare_parameter(
            "thing_gateway.ros_topic_video_record_ctrl", root + "/perception/input/video_record/ctrl_flag");
    ros_topics.from_mqtt_alarm_set = this->declare_parameter("thing_gateway.ros_topic_from_mqtt_alarm_set",
                                                             root + "/usv_manager/input/alarm_set");
    ros_topics.label_filter =
            this->declare_parameter("thing_gateway.ros_topic_label_filter", root + "/perception/input/label_filter");
    ros_topics.ptz_status = this->declare_parameter("thing_gateway.ros_topic_ptz_status",
                                                    root + "/perception/output/ptz_status");
    ros_topics.video_record_state =
            this->declare_parameter("thing_gateway.ros_topic_video_record_state",
                                    root + "/perception/output/video_record/state");
    ros_topics.events_bridge = this->declare_parameter("thing_gateway.ros_topic_events_bridge",
                                                       root + "/usv_manager/output/alarm_event");

    ProductMqttRosBridgeOptions io_opts;
    const int osd_gimbal_sec = this->declare_parameter("thing_gateway.osd_throttle_gimbal_sec", 2);
    const int osd_camera_sec = this->declare_parameter("thing_gateway.osd_throttle_camera_sec", 5);
    const int osd_lamp_sec = this->declare_parameter("thing_gateway.osd_throttle_lamp_sec", 5);
    const int osd_megaphone_sec = this->declare_parameter("thing_gateway.osd_throttle_megaphone_sec", 2);
    io_opts.gimbal_osd_min_interval = std::chrono::seconds(std::max(0, osd_gimbal_sec));
    io_opts.camera_osd_min_interval = std::chrono::seconds(std::max(0, osd_camera_sec));
    io_opts.lamp_osd_min_interval = std::chrono::seconds(std::max(0, osd_lamp_sec));
    io_opts.megaphone_osd_min_interval = std::chrono::seconds(std::max(0, osd_megaphone_sec));

    mqtt_bridge_ = std::make_unique<ProductMqttRosBridge>(
            *this, device_, ros_topics, qos_default_, mqtt_topics_.events(), mqtt_topics_.osd(),
            [this](const std::string &topic, const std::string &payload, int qos) {
                return publish_mqtt(topic, payload, qos);
            },
            [this]() { return get_active_task_bid(); }, io_opts);

    product_mqtt_init();
}

void ProductGatewayNode::product_mqtt_init() {
    // MQTT 5 异步客户端；断连时不堆积无限发送、缓冲满时丢最旧（见各 builder 选项）
    const auto opts = mqtt::create_options_builder()
                              .send_while_disconnected(false, false)
                              .max_buffered_messages(100)
                              .delete_oldest_messages()
                              .mqtt_version(MQTTVERSION_5)
                              .finalize();

    client_ = std::make_shared<mqtt::async_client>(mqtt_address_, client_id_, opts);
    client_->set_callback(*this);
}

std::string ProductGatewayNode::get_active_task_bid() const {
    std::lock_guard<std::mutex> lk(task_mu_);
    return active_task_bid_;
}

void ProductGatewayNode::apply_task_session(int task_action, const std::string &bid) {
    std::lock_guard<std::mutex> lk(task_mu_);
    if (task_action == 1) {
        active_task_bid_ = bid;
    } else if (task_action == 0) {
        active_task_bid_.clear();
    }
}

void ProductGatewayNode::connect() {
    if (!client_) {
        return;
    }
    // max_inflight：未获 broker 确认的 QoS1/2 在途消息上限；clean_session(false)：不按「每次全新会话」丢弃 broker 侧状态
    auto connOpts = mqtt::connect_options_builder()
                            .mqtt_version(MQTTVERSION_5)
                            .properties({{mqtt::property::SESSION_EXPIRY_INTERVAL, 604800}})
                            .automatic_reconnect(true)
                            .max_inflight(mqtt_max_inflight_)
                            .clean_session(false)
                            .user_name(mqtt_username_)
                            .password(mqtt_passwd_)
                            .keep_alive_interval(std::chrono::seconds(mqtt_keep_alive_interval_))
                            .connect_timeout(std::chrono::seconds(mqtt_timeout_))
                            .finalize();
    try {
        client_->connect(connOpts, nullptr, *this);
        RCLCPP_INFO_STREAM(get_logger(), "[thing_gateway] Initiating connection to broker: " << mqtt_address_);
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "[thing_gateway] Connection to broker failed: " << e.what());
    }
}

bool ProductGatewayNode::isConnected() { return is_connected_.load(std::memory_order_acquire); }

void ProductGatewayNode::connected(const std::string & /*cause*/) {
    is_connected_.store(true, std::memory_order_release);
    RCLCPP_INFO_STREAM(get_logger(), "Connected to broker successful");
    register_product_service_handler_topic();
}

void ProductGatewayNode::connection_lost(const std::string & /*cause*/) {
    if (is_shutting_down_.load(std::memory_order_acquire)) {
        return;
    }
    RCLCPP_WARN_STREAM(get_logger(), "Connection to broker lost, will try to reconnect...");
    is_connected_.store(false, std::memory_order_release);
    try {
        connect();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Exception in connection_lost during reconnect: " << e.what());
    }
    catch (...) {
        RCLCPP_ERROR_STREAM(get_logger(), "Unknown exception in connection_lost during reconnect");
    }
}

void ProductGatewayNode::fill_message_dispatch_context(ProductMessageDispatchContext &ctx) {
    ctx.logger = get_logger();
    ctx.device = device_;
    ctx.topics = mqtt_topics_;
    ctx.relax_device_sn = relax_device_sn_;
    ctx.thing_services = &thing_services_;

    // ctx.mqtt_publish → Broker（不经 bridge）。ctx.ros.* → ProductMqttRosBridge：经 bridge_mu_ 与析构 reset 互斥。
    ctx.mqtt_publish = [this](const std::string &topic, const std::string &payload, int qos) {
        if (is_shutting_down_.load(std::memory_order_acquire)) {
            return false;
        }
        return publish_mqtt(topic, payload, qos);
    };

    ctx.ros.publish_rtsp_flag = [this](float v) {
        std::lock_guard<std::mutex> lk(bridge_mu_);
        if (is_shutting_down_.load(std::memory_order_acquire)) {
            return;
        }
        if (mqtt_bridge_) {
            mqtt_bridge_->publish_rtsp_flag(v);
        }
    };

    ctx.ros.publish_alert_to_ros = [this](bool alert_on) {
        std::lock_guard<std::mutex> lk(bridge_mu_);
        if (is_shutting_down_.load(std::memory_order_acquire)) {
            return;
        }
        if (mqtt_bridge_) {
            mqtt_bridge_->publish_alert_to_ros(alert_on);
        }
    };

    ctx.ros.get_active_task_bid = [this]() -> std::string {
        if (is_shutting_down_.load(std::memory_order_acquire)) {
            return std::string();
        }
        return get_active_task_bid();
    };

    ctx.ros.publish_label_filter = [this](const std::vector<int8_t> &types) {
        std::lock_guard<std::mutex> lk(bridge_mu_);
        if (is_shutting_down_.load(std::memory_order_acquire)) {
            return;
        }
        if (mqtt_bridge_) {
            mqtt_bridge_->publish_label_filter(types);
        }
    };

    ctx.ros.publish_ptz_ctrl = [this](const std::vector<float> &data) {
        std::lock_guard<std::mutex> lk(bridge_mu_);
        if (is_shutting_down_.load(std::memory_order_acquire)) {
            return;
        }
        if (mqtt_bridge_) {
            mqtt_bridge_->publish_ptz_ctrl(data);
        }
    };

    ctx.ros.publish_lamp_ctrl = [this](int8_t v) {
        std::lock_guard<std::mutex> lk(bridge_mu_);
        if (is_shutting_down_.load(std::memory_order_acquire)) {
            return;
        }
        if (mqtt_bridge_) {
            mqtt_bridge_->publish_lamp_ctrl(v);
        }
    };

    ctx.ros.publish_megaphone_ctrl = [this](int8_t v) {
        std::lock_guard<std::mutex> lk(bridge_mu_);
        if (is_shutting_down_.load(std::memory_order_acquire)) {
            return;
        }
        if (mqtt_bridge_) {
            mqtt_bridge_->publish_megaphone_ctrl(v);
        }
    };

    ctx.ros.publish_video_record_ctrl = [this](int8_t v) {
        std::lock_guard<std::mutex> lk(bridge_mu_);
        if (is_shutting_down_.load(std::memory_order_acquire)) {
            return;
        }
        if (mqtt_bridge_) {
            mqtt_bridge_->publish_video_record_ctrl(v);
        }
    };

    ctx.apply_task_session = [this](int action, const std::string &bid) {
        if (is_shutting_down_.load(std::memory_order_acquire)) {
            return;
        }
        this->apply_task_session(action, bid);
    };
}

void ProductGatewayNode::register_product_service_handler_topic() {
    const std::string t = mqtt_topics_.services();
    if (!services_handler_ready_) {
        // 缓存 lambda，避免 connected() 重复订阅时替换不同 std::function 实例
        cached_services_handler_ = [this](const std::string & /*topic*/, const std::string &message) {
            ProductMessageDispatchContext ctx;
            fill_message_dispatch_context(ctx);
            thing_services_.dispatch(ctx, message);
        };
        services_handler_ready_ = true;
    }
    subscribe(t, cached_services_handler_, services_subscribe_qos_);
}

void ProductGatewayNode::subscribe(const std::string &topic, MessageHandler handler, int qos) {
    if (!client_) {
        return;
    }
    mqtt_topic_router_.subscribePlain(*client_, get_logger(), topic, std::move(handler), qos, is_connected_);
}

void ProductGatewayNode::message_arrived(mqtt::const_message_ptr mqtt_msg) {
    mqtt_topic_router_.deliver(mqtt_msg, get_logger(), is_shutting_down_);
}

void ProductGatewayNode::delivery_complete(mqtt::delivery_token_ptr /*token*/) {
    // 可选：QoS>0 时在此确认「已到 broker」；当前未实现业务逻辑
}

void ProductGatewayNode::on_success(const mqtt::token & /*tok*/) {
    // iaction_listener：connect 等异步操作成功
    is_connected_.store(true, std::memory_order_release);
}

void ProductGatewayNode::on_failure(const mqtt::token &token) {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Connection to broker failed (return code " << std::to_string(token.get_return_code())
                                                                    << "), will automatically retry...");
    is_connected_.store(false, std::memory_order_release);
}

bool ProductGatewayNode::publish_mqtt(const std::string &topic, const std::string &payload, int qos) {
    if (is_shutting_down_.load(std::memory_order_acquire)) {
        return false;
    }
    if (!is_connected_.load(std::memory_order_acquire)) {
        RCLCPP_ERROR_STREAM(get_logger(), "[thing_gateway] MQTT client is not connected. Cannot publish to topic: " << topic);
        return false;
    }
    try {
        auto msg = mqtt::make_message(topic, payload);
        msg->set_qos(qos);
        auto token = client_->publish(msg);
        // 不 wait token，避免在 Paho 回调线程上阻塞；失败由库内重试/日志侧观察
        (void)token;
        return true;
    }
    catch (const mqtt::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "[thing_gateway] MQTT publish exception for topic " << topic << ": " << e.what());
        return false;
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(get_logger(), "[thing_gateway] Exception during MQTT publish to topic " << topic << ": " << e.what());
        return false;
    }
}

void ProductGatewayNode::subscribeWithProps(const std::string &topic, MessageHandlerWithProps handler) {
    if (!client_) {
        return;
    }
    mqtt_topic_router_.subscribeWithProps(*client_, get_logger(), topic, std::move(handler), is_connected_);
}

bool ProductGatewayNode::publish_mqtt_with_props(const std::string &topic, const std::string &payload, int qos, const mqtt::properties& props) {
    if (is_shutting_down_.load(std::memory_order_acquire)) {
        return false;
    }
    if (!is_connected_.load(std::memory_order_acquire)) {
        RCLCPP_ERROR_STREAM(get_logger(), "[thing_gateway] MQTT client is not connected. Cannot publish to topic: " << topic);
        return false;
    }
    try {
        auto msg = mqtt::make_message(topic, payload);
        msg->set_qos(qos);
        msg->set_properties(props);
        auto token = client_->publish(msg);
        (void)token;
        return true;
    }
    catch (const mqtt::exception& e) {
        RCLCPP_ERROR_STREAM(get_logger(), "[thing_gateway] MQTT publish exception for topic " << topic << ": " << e.what());
        return false;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(get_logger(), "[thing_gateway] Exception during MQTT publish to topic " << topic << ": " << e.what());
        return false;
    }
}

ProductGatewayNode::~ProductGatewayNode() {
    is_shutting_down_.store(true, std::memory_order_release);

    // 1) 禁止再进 message_arrived（须在动 bridge 之前，避免回调里用已销毁的 bridge）
    if (client_) {
        try {
            client_->disable_callbacks();
        }
        catch (const mqtt::exception &) {
        }
    }

    // 2) bridge：与 ctx.ros.* 闭包同锁，避免与 MQTT 回调路径上的 publish 交叉 reset
    {
        std::lock_guard<std::mutex> lk(bridge_mu_);
        if (mqtt_bridge_) {
            mqtt_bridge_->shutdown();
        }
        mqtt_bridge_.reset();
    }

    // 3) 短等待 disconnect，再释放 client
    if (client_) {
        try {
            auto dtok = client_->disconnect();
            dtok->wait_for(std::chrono::milliseconds(100));
        }
        catch (const mqtt::exception &) {
        }
    }

    client_.reset();
}

std::string ProductGatewayNode::read_sn_from_file(const std::string &file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        return "";
    }

    std::string sn;
    std::getline(file, sn);
    file.close();

    sn = trim(sn);

    // UTF-8 BOM：仅当完整 EF BB BF 时去掉 3 字节，避免首字节偶然为 0xEF 时误删。
    if (sn.size() >= 3 && static_cast<unsigned char>(sn[0]) == 0xEF && static_cast<unsigned char>(sn[1]) == 0xBB &&
        static_cast<unsigned char>(sn[2]) == 0xBF) {
        sn.erase(0, 3);
    }

    return sn;
}

} // namespace thing_gateway
