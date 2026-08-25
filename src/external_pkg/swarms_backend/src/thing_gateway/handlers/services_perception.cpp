#include "thing_gateway/product_thing_services_dispatcher.hpp"
#include "thing_gateway/thing_json/parse.hpp"
#include "thing_gateway/thing_json/serialize.hpp"

#include <vector>

#include <rclcpp/rclcpp.hpp>

// stream_start/stop、target_types_config（云台回中见 payload_control + control_params.reset）
namespace thing_gateway {
namespace {

using namespace thing_json;

void handle_target_types_config(
    const ProductMessageDispatchContext& ctx,
    const json& root,
    const std::string& /*tid*/,
    const std::string& /*bid*/,
    const std::string& /*method*/,
    const std::string& /*raw_payload*/,
    const ReplyErr& reply_err,
    const ReplyOk& reply_ok) {

    const json* common = get_data_common(root);
    if (!common) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    if (!common->contains("enabled_types") || !(*common)["enabled_types"].is_array()) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    const auto& enabled_types = (*common)["enabled_types"];
    std::vector<int8_t> types;

    // MQTT 与《无人船》文档一致：仅 `[]` 表示全检。ROS `label_filter`（Int8MultiArray）仍用 -1 表示全检。
    if (enabled_types.empty()) {
        types.push_back(-1);
    } else {
        types.reserve(enabled_types.size());
        for (const auto& v : enabled_types) {
            int8_t one = 0;
            if (!json_to_int8(v, one)) {
                reply_err(result_code::kInvalidParam);
                return;
            }
            types.push_back(one);
        }
    }

    if (ctx.ros.publish_label_filter) {
        ctx.ros.publish_label_filter(types);
    }

    reply_ok();
}

void handle_stream_start(
    const ProductMessageDispatchContext& ctx,
    const json& /*root*/,
    const std::string& /*tid*/,
    const std::string& /*bid*/,
    const std::string& /*method*/,
    const std::string& /*raw_payload*/,
    const ReplyErr& /*reply_err*/,
    const ReplyOk& reply_ok) {

    if (ctx.ros.publish_rtsp_flag) {
        ctx.ros.publish_rtsp_flag(1.0f);
    }

    reply_ok();
}

void handle_stream_stop(
    const ProductMessageDispatchContext& ctx,
    const json& /*root*/,
    const std::string& /*tid*/,
    const std::string& /*bid*/,
    const std::string& /*method*/,
    const std::string& /*raw_payload*/,
    const ReplyErr& /*reply_err*/,
    const ReplyOk& reply_ok) {

    if (ctx.ros.publish_rtsp_flag) {
        ctx.ros.publish_rtsp_flag(-1.0f);
    }

    reply_ok();
}

} // namespace

void register_perception_handlers(ProductThingServicesDispatcher &d) {
    d.registerServiceHandler("target_types_config", handle_target_types_config);
    d.registerServiceHandler("stream_start", handle_stream_start);
    d.registerServiceHandler("stream_stop", handle_stream_stop);
}

} // namespace thing_gateway
