// MESSAGE PTZ_STATE support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief PTZ_STATE message
 *
 * 云台状态
 */
struct PTZ_STATE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13035;
    static constexpr size_t LENGTH = 4;
    static constexpr size_t MIN_LENGTH = 4;
    static constexpr uint8_t CRC_EXTRA = 166;
    static constexpr auto NAME = "PTZ_STATE";


    int32_t data; /*<  确认标志数据 */


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
        ss << "  data: " << data << std::endl;

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
