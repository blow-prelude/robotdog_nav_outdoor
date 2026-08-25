#include "thing_gateway/product_thing_services_dispatcher.hpp"
#include "thing_gateway/thing_json/parse.hpp"

#include <rclcpp/rclcpp.hpp>

// alarm_set、task_control
namespace thing_gateway {
namespace {

using namespace thing_json;

void handle_alarm_set(
    const ProductMessageDispatchContext& ctx,
    const json& root,
    const std::string& /*tid*/,
    const std::string& bid,
    const std::string& /*method*/,
    const std::string& /*raw_payload*/,
    const ReplyErr& reply_err,
    const ReplyOk& reply_ok) {

    (void)bid;

    // 协议：`data.common.enabled` 布尔；true=开启告警，false=关闭 → ROS std_msgs/Bool
    const json* common = get_data_common(root);
    if (!common) {
        RCLCPP_WARN_STREAM(ctx.logger, "[thing_gateway] alarm_set: missing data.common");
        reply_err(result_code::kInvalidParam);
        return;
    }

    if (!common->contains("enabled") || !(*common)["enabled"].is_boolean()) {
        RCLCPP_WARN_STREAM(ctx.logger, "[thing_gateway] alarm_set: missing or invalid enabled field");
        reply_err(result_code::kInvalidParam);
        return;
    }

    const bool alert_on = (*common)["enabled"].get<bool>();

    if (ctx.ros.publish_alert_to_ros) {
        ctx.ros.publish_alert_to_ros(alert_on);
    }

    reply_ok();
}

void handle_task_control(
    const ProductMessageDispatchContext& ctx,
    const json& root,
    const std::string& /*tid*/,
    const std::string& bid,
    const std::string& /*method*/,
    const std::string& /*raw_payload*/,
    const ReplyErr& reply_err,
    const ReplyOk& reply_ok) {

    // 根级 bid 可选：无任务周期 ID 时允许省略或空串；OSD/events 仍可不带来源 bid

    const json* common = get_data_common(root);
    if (!common) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    if (!common->contains("task_action")) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    int action = -1;
    const auto& ta = (*common)["task_action"];
    if (ta.is_number_integer()) {
        action = ta.get<int>();
    } else {
        reply_err(result_code::kInvalidParam);
        return;
    }

    if (action != 0 && action != 1) {
        reply_err(result_code::kInvalidParam);
        return;
    }

    if (ctx.apply_task_session) {
        ctx.apply_task_session(action, bid);
    }

    reply_ok();
}

} // namespace

void register_alert_task_handlers(ProductThingServicesDispatcher &d) {
    d.registerServiceHandler("alarm_set", handle_alarm_set);
    d.registerServiceHandler("task_control", handle_task_control);
}

} // namespace thing_gateway
