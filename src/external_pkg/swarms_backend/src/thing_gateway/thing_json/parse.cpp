#include "thing_gateway/thing_json/parse.hpp"

#include <cstdint>

// services 信封字段、device.sn 校验、data.common 指针（供 dispatch 与 handlers 共用）
namespace thing_gateway::thing_json {

json device_to_json(const DeviceIdentity& dev) {
    json j;
    j["sn"] = dev.sn;
    j["name"] = dev.name;
    j["type"] = static_cast<int>(dev.type);
    j["sub_type"] = dev.sub_type;
    j["vendor"] = dev.vendor;
    return j;
}

void attach_data_ext(json& data, DeviceType device_type) {
    json ext;
    const char* key = nullptr;
    switch (device_type) {
        case DeviceType::kUAV: key = "uav_ext"; break;
        case DeviceType::kUSV: key = "usv_ext"; break;
        case DeviceType::kUGV: key = "ugv_ext"; break;
        default: key = "usv_ext"; break;
    }
    ext[key] = json::object();
    data["ext"] = ext;
}

bool parse_root(const std::string& payload, json& root, std::string& err) {
    try {
        root = json::parse(payload);
        err.clear();
        return true;
    } catch (const json::parse_error& e) {
        err = std::string("JSON parse error: ") + e.what();
        return false;
    } catch (const json::exception& e) {
        err = std::string("JSON exception: ") + e.what();
        return false;
    }
}

bool parse_service_envelope(const json& root, ParsedService& out, std::string& err) {
    out.root = root;
    out.common = nullptr;

    // 提取 tid
    if (!root.contains("tid") || !root["tid"].is_string()) {
        err = "missing tid";
        return false;
    }
    out.tid = root["tid"].get<std::string>();

    // 提取 bid（可选）
    out.bid.clear();
    if (root.contains("bid") && root["bid"].is_string()) {
        out.bid = root["bid"].get<std::string>();
    }

    // 提取 method
    if (!root.contains("method") || !root["method"].is_string()) {
        err = "missing method";
        return false;
    }
    out.method = root["method"].get<std::string>();

    // 提取 data.common 指针
    out.common = get_data_common(root);

    err.clear();
    return true;
}

bool parse_service_message(const std::string& payload, ParsedService& out, std::string& err) {
    json root;
    if (!parse_root(payload, root, err)) {
        return false;
    }
    return parse_service_envelope(root, out, err);
}

bool extract_partial_ids(const json& root, std::string& tid, std::string& bid, std::string& method) {
    bid.clear();

    if (!root.contains("tid") || !root["tid"].is_string()) {
        return false;
    }
    tid = root["tid"].get<std::string>();
    if (tid.empty()) {
        return false;
    }

    if (root.contains("bid") && root["bid"].is_string()) {
        bid = root["bid"].get<std::string>();
    }

    if (root.contains("method") && root["method"].is_string() && !root["method"].empty()) {
        method = root["method"].get<std::string>();
    } else {
        method = "invalid_envelope";
    }

    return true;
}

bool device_sn_matches(const json& root, const std::string& expect_sn, bool relax) {
    if (expect_sn.empty()) {
        return true;
    }

    if (relax) {
        // 宽松模式：无 device 或无 sn 字段时放过
        if (!root.contains("device") || !root["device"].is_object()) {
            return true;
        }
        const auto& dev = root["device"];
        if (!dev.contains("sn") || !dev["sn"].is_string()) {
            return true;
        }
        const std::string req = dev["sn"].get<std::string>();
        if (req.empty()) {
            return true;
        }
        return req == expect_sn;
    }

    // 严格模式：必须匹配
    if (!root.contains("device") || !root["device"].is_object()) {
        return false;
    }
    const auto& dev = root["device"];
    if (!dev.contains("sn") || !dev["sn"].is_string()) {
        return false;
    }
    const std::string req = dev["sn"].get<std::string>();
    if (req.empty()) {
        return false;
    }
    return req == expect_sn;
}

bool json_to_int8(const json& j, int8_t& out) {
    if (!j.is_number_integer()) {
        return false;
    }
    int64_t val = j.get<int64_t>();
    if (val < 0 || val > 127) {
        return false;
    }
    out = static_cast<int8_t>(val);
    return true;
}

bool has_data_common(const json& root) {
    return get_data_common(root) != nullptr;
}

const json* get_data_common(const json& root) {
    if (!root.contains("data") || !root["data"].is_object()) {
        return nullptr;
    }
    const auto& data = root["data"];
    if (!data.contains("common") || !data["common"].is_object()) {
        return nullptr;
    }
    return &data["common"];
}

} // namespace thing_gateway::thing_json
