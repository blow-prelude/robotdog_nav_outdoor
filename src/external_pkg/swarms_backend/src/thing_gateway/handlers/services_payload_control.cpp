#include "thing_gateway/product_thing_services_dispatcher.hpp"
#include "thing_gateway/thing_json/parse.hpp"
#include "thing_gateway/thing_json/serialize.hpp"

#include <rclcpp/rclcpp.hpp>

// MQTT method payload_control：按 data.common.payload_type 分发给下方子 handler
namespace thing_gateway {
namespace {

using namespace thing_json;

// 与 perception 约定：ROS 8 元 [home, pos_flag, roll, pitch, yaw, zoom_flag, zoom, wiper]
// MQTT `payload_control` 云台（非 reset）：pos_flag 固定为 kPtzRosVelocityModeFlag（-2），三槽为 roll_rate/pitch_rate/yaw_rate；
// ROS 首槽 home 固定 **-1**（不回正）；MQTT **不得**再带 `home` 键。
// 回中：`reset:true` 且 **control_params 仅含键 reset** → ROS home=1、pos_flag=-2（与角速度帧一致）；与其它键混用 → 1001。
constexpr float kPtzRosVelocityModeFlag = -2.0f;

/// zoom 缺省或 JSON 无字段：不变倍，zoom_value=0。zoom==0：不变倍，zoom_value=0。zoom 非 0：变倍并带倍数。非数字：失败。
bool resolve_ptz_zoom(const json &control_params, float &zoom_flag, float &zoom_value, const ReplyErr &reply_err) {
    if (!control_params.contains("zoom") || control_params["zoom"].is_null()) {
        zoom_flag = -1.0f;
        zoom_value = 0.0f;
        return true;
    }
    const auto &zj = control_params["zoom"];
    if (!zj.is_number()) {
        reply_err(result_code::kInvalidParam);
        return false;
    }
    const double z = zj.get<double>();
    if (z == 0.0) {
        zoom_flag = -1.0f;
        zoom_value = 0.0f;
        return true;
    }
    zoom_flag = 1.0f;
    zoom_value = static_cast<float>(z);
    return true;
}

void handle_ptz_control(
    const ProductMessageDispatchContext& ctx,
    const json& control_params,
    const ReplyErr& reply_err,
    const ReplyOk& reply_ok) {

    if (control_params.empty()) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    if (!ctx.ros.publish_ptz_ctrl) {
        RCLCPP_WARN_STREAM(ctx.logger,
                           "[thing_gateway] payload_control type=1: no ptz_ctrl handler available, cannot process");
        reply_err(result_code::kInvalidParam);
        return;
    }

    if (control_params.contains("reset")) {
        const auto &rj = control_params["reset"];
        if (!rj.is_boolean()) {
            reply_err(result_code::kInvalidParam);
            return;
        }
        if (rj.get<bool>()) {
            if (control_params.size() != 1u) {
                reply_err(result_code::kInvalidParam);
                return;
            }
            constexpr float kHomeGo = 1.0f;
            const std::vector<float> ptz = {
                    kHomeGo, kPtzRosVelocityModeFlag, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, -1.0f};
            ctx.ros.publish_ptz_ctrl(ptz);
            reply_ok();
            return;
        }
        // reset:false → 按普通云台帧处理（须另有 roll_rate/pitch_rate/yaw_rate 等）
    }

    if (control_params.contains("home")) {
        RCLCPP_WARN_STREAM(ctx.logger,
                           "[thing_gateway] payload_control type=1: MQTT key `home` is not used; ROS home slot is "
                           "fixed to -1 for non-reset frames");
        reply_err(result_code::kInvalidParam);
        return;
    }

    if (control_params.contains("roll") || control_params.contains("pitch") || control_params.contains("yaw") ||
        control_params.contains("pos_flag")) {
        RCLCPP_WARN_STREAM(ctx.logger,
                           "[thing_gateway] payload_control type=1: angle keys roll/pitch/yaw or pos_flag are not "
                           "accepted; use roll_rate/pitch_rate/yaw_rate");
        reply_err(result_code::kInvalidParam);
        return;
    }

    const bool has_rr = control_params.contains("roll_rate");
    const bool has_pr = control_params.contains("pitch_rate");
    const bool has_yr = control_params.contains("yaw_rate");
    // if (!has_rr && !has_pr && !has_yr) {
    //     RCLCPP_WARN_STREAM(ctx.logger,
    //                        "[thing_gateway] payload_control type=1: need at least one of roll_rate, pitch_rate, "
    //                        "yaw_rate (or use reset:true for home)");
    //     reply_err(result_code::kInvalidParam);
    //     return;
    // }

    float zoom_flag = -1.0f;
    float zoom_value = 1.0f;
    if (!resolve_ptz_zoom(control_params, zoom_flag, zoom_value, reply_err)) {
        return;
    }

    auto opt_float = [&control_params](const char *key, float fallback) -> float {
        if (!control_params.contains(key)) {
            return fallback;
        }
        const auto &v = control_params[key];
        if (v.is_number()) {
            return static_cast<float>(v.get<double>());
        }
        return fallback;
    };

    constexpr float kHomeNoGo = -1.0f;
    const float wiper = opt_float("wiper", -1.0f);
    const float r = opt_float("roll_rate", 0.0f);
    const float p = opt_float("pitch_rate", 0.0f);
    const float y = opt_float("yaw_rate", 0.0f);

    const std::vector<float> ptz = {kHomeNoGo, kPtzRosVelocityModeFlag, r, p, y, zoom_flag, zoom_value, wiper};

    ctx.ros.publish_ptz_ctrl(ptz);
    reply_ok();
}

void handle_video_record_control(
    const ProductMessageDispatchContext& ctx,
    const json& control_params,
    const ReplyErr& reply_err,
    const ReplyOk& reply_ok) {

    if (!ctx.ros.publish_video_record_ctrl) {
        RCLCPP_WARN_STREAM(ctx.logger,
                           "[thing_gateway] payload_control type=2: no video_record handler available, cannot process");
        reply_err(result_code::kInvalidParam);
        return;
    }

    if (!control_params.contains("action") || !control_params["action"].is_number_integer()) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    const int a = control_params["action"].get<int>();
    if (a != 0 && a != 1) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    const int8_t v = (a == 1) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
    ctx.ros.publish_video_record_ctrl(v);
    reply_ok();
}

void handle_lamp_control(
    const ProductMessageDispatchContext& ctx,
    const json& control_params,
    const ReplyErr& reply_err,
    const ReplyOk& reply_ok) {

    if (!ctx.ros.publish_lamp_ctrl) {
        RCLCPP_WARN_STREAM(ctx.logger,
                           "[thing_gateway] payload_control type=3: no lamp_ctrl handler available, cannot process");
        reply_err(result_code::kInvalidParam);
        return;
    }

    if (!control_params.contains("action") || !control_params["action"].is_number_integer()) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    const int a = control_params["action"].get<int>();
    if (a != 0 && a != 1) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    const int8_t v = (a == 1) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
    ctx.ros.publish_lamp_ctrl(v);
    reply_ok();
}

void handle_megaphone_control(
    const ProductMessageDispatchContext& ctx,
    const json& control_params,
    const ReplyErr& reply_err,
    const ReplyOk& reply_ok) {

    if (!ctx.ros.publish_megaphone_ctrl) {
        RCLCPP_WARN_STREAM(ctx.logger,
                           "[thing_gateway] payload_control type=4: no megaphone_ctrl handler available, cannot process");
        reply_err(result_code::kInvalidParam);
        return;
    }

    if (!control_params.contains("action") || !control_params["action"].is_number_integer()) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    const int a = control_params["action"].get<int>();
    if (a != 0 && a != 1) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    const int8_t v = (a == 1) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
    ctx.ros.publish_megaphone_ctrl(v);
    reply_ok();
}

void handle_payload_control(
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

    if (!common->contains("payload_type")) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    const auto& pt_json = (*common)["payload_type"];
    if (!pt_json.is_number_integer()) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    const int pt = pt_json.get<int>();
    if (pt < 1) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    if (!common->contains("control_params") || !(*common)["control_params"].is_object()) {
        reply_err(result_code::kInvalidParam);
        return;
    }
    const auto& cp = (*common)["control_params"];

    if (!ctx.thing_services) {
        reply_err(result_code::kInvalidParam);
        return;
    }
    PayloadControlHandler handler = ctx.thing_services->findPayloadControlHandler(pt);
    if (!handler) {
        RCLCPP_WARN_STREAM(ctx.logger, "[thing_gateway] payload_control: unsupported payload_type " << pt);
        reply_err(result_code::kInvalidParam);
        return;
    }

    handler(ctx, cp, reply_err, reply_ok);
}

} // namespace

void register_payload_control_handlers(ProductThingServicesDispatcher &d) {
    d.registerServiceHandler("payload_control", handle_payload_control);
    d.registerPayloadControlHandler(1, handle_ptz_control);
    d.registerPayloadControlHandler(2, handle_video_record_control);
    d.registerPayloadControlHandler(3, handle_lamp_control);
    d.registerPayloadControlHandler(4, handle_megaphone_control);
}

} // namespace thing_gateway
