#include "thing_gateway/thing_json/serialize.hpp"

#include <chrono>
#include <random>
#include <string>

// MQTT 出站：services_reply、payload_status（osd）正文
namespace thing_gateway::thing_json {

namespace {

int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

/// 信封 `tid`：文档要求 UUID v4
std::string make_uuid_v4() {
    static thread_local std::mt19937 gen{std::random_device{}()};
    std::uniform_int_distribution<int> dis(0, 15);
    std::uniform_int_distribution<int> dis2(8, 11);
    static const char* hex = "0123456789abcdef";
    std::string u;
    u.reserve(36);
    for (int i = 0; i < 36; ++i) {
        if (i == 8 || i == 13 || i == 18 || i == 23) {
            u += '-';
        } else if (i == 14) {
            u += '4';
        } else if (i == 19) {
            u += hex[dis2(gen)];
        } else {
            u += hex[dis(gen)];
        }
    }
    return u;
}

} // namespace

std::string build_services_reply(
    const DeviceIdentity& dev,
    const std::string& tid,
    const std::string& bid,
    const std::string& method,
    int32_t result,
    const json& extra_common) {
    json root;
    root["tid"] = tid;
    if (!bid.empty()) {
        root["bid"] = bid;
    }
    root["ts"] = now_ms();
    root["method"] = method;
    root["device"] = device_to_json(dev);

    json common;
    // 先合并扩展字段，最后写 result，避免 extra_common 中的 "result" 覆盖真实结果码
    if (!extra_common.empty() && extra_common.is_object()) {
        for (auto& [key, value] : extra_common.items()) {
            common[key] = value;
        }
    }
    common["result"] = result;

    json data;
    data["common"] = common;
    attach_data_ext(data, dev.type);
    root["data"] = data;

    return root.dump();
}

std::string build_payload_status(
    const DeviceIdentity& dev,
    const std::string& bid,
    int payload_type,
    const json& attributes) {
    const int64_t ts_ms = now_ms();
    json root;
    root["tid"] = make_uuid_v4();
    if (!bid.empty()) {
        root["bid"] = bid;
    }
    root["ts"] = ts_ms;
    root["method"] = "payload_status";
    root["device"] = device_to_json(dev);

    json common;
    common["payload_type"] = payload_type;
    common["attributes"] = attributes.is_object() ? attributes : json::object();

    json data;
    data["common"] = common;
    attach_data_ext(data, dev.type);
    root["data"] = data;

    return root.dump();
}

std::string build_product_event(const DeviceIdentity& dev, const std::string& bid, const json& common) {
    const int64_t ts_ms = now_ms();
    json root;
    root["tid"] = make_uuid_v4();
    if (!bid.empty()) {
        root["bid"] = bid;
    }
    root["ts"] = ts_ms;
    root["method"] = "alarm_state";
    root["device"] = device_to_json(dev);

    json data;
    data["common"] = common.is_object() ? common : json::object();
    attach_data_ext(data, dev.type);
    root["data"] = data;

    return root.dump();
}

} // namespace thing_gateway::thing_json
