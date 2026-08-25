#pragma once

#include "thing_gateway/thing_json/types.hpp"

#include <string>

namespace thing_gateway::thing_json {

/// 构建 services_reply 响应
/// extra_common: 可选扩展字段（object），先合并进 common，再以参数 result 覆盖写入（结果码以此为准）
[[nodiscard]] std::string build_services_reply(
    const DeviceIdentity& dev,
    const std::string& tid,
    const std::string& bid,
    const std::string& method,
    int32_t result,
    const json& extra_common = json::object());

/// 构建 payload_status OSD上报消息
/// attributes: 载荷属性，须为 JSON object；否则按空 object 上报
[[nodiscard]] std::string build_payload_status(
    const DeviceIdentity& dev,
    const std::string& bid,
    int payload_type,
    const json& attributes);

/// 构建 thing/product/{sn}/events 上报（与 payload_status 同信封：tid/bid/ts/method/device/data）
/// `method` 固定为 `alarm_state`；common 须含 `enabled` 等，写入 data.common
[[nodiscard]] std::string build_product_event(
    const DeviceIdentity& dev,
    const std::string& bid,
    const json& common);

} // namespace thing_gateway::thing_json
