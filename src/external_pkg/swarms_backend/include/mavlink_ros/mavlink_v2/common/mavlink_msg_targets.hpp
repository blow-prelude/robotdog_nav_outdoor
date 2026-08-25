// MESSAGE TARGETS support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief TARGETS message
 *
 * 包含多个目标的目标信息消息
 */
struct TARGETS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13031;
    static constexpr size_t LENGTH = 236;
    static constexpr size_t MIN_LENGTH = 236;
    static constexpr uint8_t CRC_EXTRA = 215;
    static constexpr auto NAME = "TARGETS";


    std::array<int32_t, 55> target; /*<  每批最大数量目标信息 */
    int32_t batch_target_count; /*<  每批实际目标数量 */
    int32_t total_target_count; /*<  目标总数量 */
    int32_t total_batches; /*<  总批次数量 */
    int32_t current_batch; /*<  当前批次索引 */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  target: [" << to_string(target) << "]" << std::endl;
        ss << "  batch_target_count: " << batch_target_count << std::endl;
        ss << "  total_target_count: " << total_target_count << std::endl;
        ss << "  total_batches: " << total_batches << std::endl;
        ss << "  current_batch: " << current_batch << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << target;                        // offset: 0
        map << batch_target_count;            // offset: 220
        map << total_target_count;            // offset: 224
        map << total_batches;                 // offset: 228
        map << current_batch;                 // offset: 232
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> target;                        // offset: 0
        map >> batch_target_count;            // offset: 220
        map >> total_target_count;            // offset: 224
        map >> total_batches;                 // offset: 228
        map >> current_batch;                 // offset: 232
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
