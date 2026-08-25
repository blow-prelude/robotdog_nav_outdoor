#pragma once

#include <string>
#include <nlohmann/json.hpp>

namespace thing_gateway::thing_json {

using json = nlohmann::json;

enum class DeviceType : int {
    kUnknown = 0,
    kUAV = 1,
    kUSV = 3,
    kUGV = 4,
};

namespace result_code {
constexpr int32_t kSuccess = 0;
constexpr int32_t kInvalidParam = 1001;
constexpr int32_t kInvalidUrl = 1002;
constexpr int32_t kConnectFailed = 1003;
constexpr int32_t kUnknown = 1999;
}

struct DeviceIdentity {
    std::string sn;
    std::string name;
    std::string vendor;
    DeviceType type{DeviceType::kUSV};
    int sub_type{0};
};

/// 一次 services 消息的解析结果
struct ParsedService {
    json root;              // 完整JSON（nlohmann::json 是值类型）
    std::string tid;        // 事务ID
    std::string bid;        // 业务ID（可选）
    std::string method;     // 方法名
    const json* common;     // 指向 root["data"]["common"]（非拥有，const）
};

/// 辅助函数：DeviceIdentity → json
json device_to_json(const DeviceIdentity& dev);

/// 辅助函数：附加 data.ext 字段（根据设备类型）
void attach_data_ext(json& data, DeviceType device_type);

} // namespace thing_gateway::thing_json
