// MESSAGE FOLLOW_COVER_PASS_WAYPOINT support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief FOLLOW_COVER_PASS_WAYPOINT message
 *
 * 跟随覆盖任务的航点通过消息
 */
struct FOLLOW_COVER_PASS_WAYPOINT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13029;
    static constexpr size_t LENGTH = 3;
    static constexpr size_t MIN_LENGTH = 3;
    static constexpr uint8_t CRC_EXTRA = 185;
    static constexpr auto NAME = "FOLLOW_COVER_PASS_WAYPOINT";


    uint8_t groupId; /*<  任务组 ID */
    uint16_t currentPointId; /*<  当前航点 ID */


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
        ss << "  groupId: " << +groupId << std::endl;
        ss << "  currentPointId: " << currentPointId << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << currentPointId;                // offset: 0
        map << groupId;                       // offset: 2
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> currentPointId;                // offset: 0
        map >> groupId;                       // offset: 2
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
