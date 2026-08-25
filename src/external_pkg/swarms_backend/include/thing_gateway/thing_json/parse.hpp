#pragma once

#include "thing_gateway/thing_json/types.hpp"

#include <optional>
#include <string>

namespace thing_gateway::thing_json {

/// 解析JSON字符串为 nlohmann::json
/// 若解析失败，返回 false 并在 err 中填充错误信息
[[nodiscard]] bool parse_root(
    const std::string& payload,
    json& root,
    std::string& err);

/// 从已解析的 root 中提取 tid/bid/method/common
/// common 指针指向 root["data"]["common"]，生命周期绑定到 root
[[nodiscard]] bool parse_service_envelope(
    const json& root,
    ParsedService& out,
    std::string& err);

/// 一站式解析：parse_root + parse_service_envelope
[[nodiscard]] bool parse_service_message(
    const std::string& payload,
    ParsedService& out,
    std::string& err);

/// 当信封不完整时，尽量提取 tid/bid/method（用于错误回复）
/// 若无有效 tid 则返回 false
[[nodiscard]] bool extract_partial_ids(
    const json& root,
    std::string& tid,
    std::string& bid,
    std::string& method);

/// 校验 device.sn 是否匹配
/// expect_sn 为空时返回 true；relax 为 true 时宽松校验（兼容旧云端）
[[nodiscard]] bool device_sn_matches(
    const json& root,
    const std::string& expect_sn,
    bool relax = false);

/// 辅助：从 json 值安全提取 int8（如 label_filter enabled_types，范围 0..127）
[[nodiscard]] bool json_to_int8(const json& j, int8_t& out);

/// 辅助：检查是否有 data.common 字段
[[nodiscard]] bool has_data_common(const json& root);

/// 辅助：获取 data.common 指针
[[nodiscard]] const json* get_data_common(const json& root);

/// 辅助：require data.common（简化 handler 中的重复代码）；无则 `nullopt`。
[[nodiscard]] inline std::optional<const json*> require_data_common(const json& root) {
    const json* common = get_data_common(root);
    if (!common) {
        return std::nullopt;
    }
    return common;
}

} // namespace thing_gateway::thing_json
