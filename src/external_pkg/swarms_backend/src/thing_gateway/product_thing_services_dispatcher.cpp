#include "thing_gateway/product_thing_services_dispatcher.hpp"

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include "thing_gateway/thing_json/parse.hpp"
#include "thing_gateway/thing_json/serialize.hpp"

namespace thing_gateway {

void ProductThingServicesDispatcher::registerServiceHandler(const std::string &method, ProductServiceHandler handler) {
    std::lock_guard<std::mutex> lk(mu_);
    by_method_[method] = std::move(handler);
}

void ProductThingServicesDispatcher::registerPayloadControlHandler(int payload_type, PayloadControlHandler handler) {
    std::lock_guard<std::mutex> lk(mu_);
    by_payload_type_[payload_type] = std::move(handler);
}

PayloadControlHandler ProductThingServicesDispatcher::findPayloadControlHandler(int payload_type) const {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = by_payload_type_.find(payload_type);
    if (it != by_payload_type_.end()) {
        return it->second;
    }
    return nullptr;
}

void ProductThingServicesDispatcher::dispatch(const ProductMessageDispatchContext &ctx, const std::string &payload) const {
    using namespace thing_json;

    json root;
    std::string err;
    if (!parse_root(payload, root, err)) {
        RCLCPP_WARN_STREAM(ctx.logger, "[thing_gateway] services JSON parse: " << err << " (cannot reply without tid)");
        return;
    }

    ParsedService parsed;
    if (!parse_service_envelope(root, parsed, err)) {
        std::string tid, bid, method;
        if (extract_partial_ids(root, tid, bid, method)) {
            const std::string r = build_services_reply(
                    ctx.device, tid, bid, method, result_code::kInvalidParam);
            if (!ctx.mqtt_publish(ctx.topics.services_reply(), r, 1)) {
                RCLCPP_WARN_STREAM(ctx.logger,
                                   "[thing_gateway] services_reply publish failed (invalid envelope)");
            }
        } else {
            RCLCPP_WARN_STREAM(ctx.logger, "[thing_gateway] services envelope: " << err << " (no tid, cannot reply)");
        }
        return;
    }

    if (!device_sn_matches(root, ctx.device.sn, ctx.relax_device_sn)) {
        const std::string r = build_services_reply(
                ctx.device, parsed.tid, parsed.bid, parsed.method, result_code::kInvalidParam);
        if (!ctx.mqtt_publish(ctx.topics.services_reply(), r, 1)) {
            RCLCPP_WARN_STREAM(ctx.logger, "[thing_gateway] services_reply publish failed (device sn mismatch)");
        }
        return;
    }

    const ReplyErr reply_err = [&](int32_t code) {
        const std::string r = build_services_reply(
                ctx.device, parsed.tid, parsed.bid, parsed.method, code);
        if (!ctx.mqtt_publish(ctx.topics.services_reply(), r, 1)) {
            RCLCPP_WARN_STREAM(ctx.logger, "[thing_gateway] services_reply publish failed, result=" << code);
        }
    };

    const ReplyOk reply_ok = [&]() {
        const std::string r = build_services_reply(
                ctx.device, parsed.tid, parsed.bid, parsed.method, result_code::kSuccess);
        if (!ctx.mqtt_publish(ctx.topics.services_reply(), r, 1)) {
            RCLCPP_WARN_STREAM(ctx.logger, "[thing_gateway] services_reply publish failed (success ack)");
        }
    };

    ProductServiceHandler fn;
    {
        std::lock_guard<std::mutex> lk(mu_);
        auto it = by_method_.find(parsed.method);
        if (it == by_method_.end()) {
            RCLCPP_WARN_STREAM(ctx.logger, "[thing_gateway] unknown method: " << parsed.method);
            reply_err(result_code::kUnknown);
            return;
        }
        fn = it->second;
    }

    try {
        fn(ctx, parsed.root, parsed.tid, parsed.bid, parsed.method, payload, reply_err, reply_ok);
    } catch (const json::exception &e) {
        RCLCPP_ERROR_STREAM(ctx.logger, "[thing_gateway] JSON exception in ProductThingServicesDispatcher::dispatch: " << e.what());
        reply_err(result_code::kInvalidParam);
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(ctx.logger, "[thing_gateway] Exception in ProductThingServicesDispatcher::dispatch: " << e.what());
        reply_err(result_code::kInvalidParam);
    } catch (...) {
        RCLCPP_ERROR_STREAM(ctx.logger, "[thing_gateway] Unknown exception in ProductThingServicesDispatcher::dispatch");
        reply_err(result_code::kInvalidParam);
    }
}

void register_builtin_product_handlers(ProductThingServicesDispatcher &d) {
    register_alert_task_handlers(d);
    register_perception_handlers(d);
    register_payload_control_handlers(d);
}

} // namespace thing_gateway
