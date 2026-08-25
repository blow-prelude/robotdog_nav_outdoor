// MESSAGE CAN_CONFIRMED_FLAG support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief CAN_CONFIRMED_FLAG message
 *
 * 用于标识确认状态的消息
 */
struct CAN_CONFIRMED_FLAG : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13030;
    static constexpr size_t LENGTH = 1;
    static constexpr size_t MIN_LENGTH = 1;
    static constexpr uint8_t CRC_EXTRA = 53;
    static constexpr auto NAME = "CAN_CONFIRMED_FLAG";


    uint8_t data; /*<  确认标志数据 */


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
        ss << "  data: " << +data << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << data;                          // offset: 0
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> data;                          // offset: 0
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
