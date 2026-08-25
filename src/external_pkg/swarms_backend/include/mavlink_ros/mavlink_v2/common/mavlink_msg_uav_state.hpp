// MESSAGE UAV_STATE support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief UAV_STATE message
 *
 * 机器人状态消息，包含主要状态和位置信息
 */
struct UAV_STATE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13027;
    static constexpr size_t LENGTH = 85;
    static constexpr size_t MIN_LENGTH = 85;
    static constexpr uint8_t CRC_EXTRA = 219;
    static constexpr auto NAME = "UAV_STATE";


    int32_t robotMajorState; /*<  机器人主要状态 */
    uint8_t connected; /*<  连接状态（0：未连接，1：已连接） */
    uint8_t armed; /*<  解锁状态（0：未解锁，1：已解锁） */
    uint8_t guided; /*<  引导模式（0：手动，1：引导） */
    uint8_t manual_input; /*<  手动输入状态（0：未启用，1：启用） */
    std::array<char, 30> mode; /*<  飞行模式名称 */
    uint16_t mode_len; /*<  飞行模式名称长度 */
    int8_t status; /*<  系统状态（-128 至 127） */
    uint16_t service; /*<  服务标识符 */
    int32_t latitude; /*< [degE7] 纬度，单位为度*1E7 */
    int32_t longitude; /*< [degE7] 经度，单位为度*1E7 */
    int32_t altitude; /*< [degE5] 海拔高度，单位为毫米 */
    uint32_t satellite_count; /*<  卫星数量 */
    float voltage; /*<  电压 */
    float yaw; /*<  yaw角 */
    float dis; /*<  水平距离 */
    float dis_vel; /*<  水平速度 */
    float high; /*<  垂直高度 */
    float high_vel; /*<  垂直速度 */
    int16_t field_flags; /*<  字段有效标志位 */


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
        ss << "  robotMajorState: " << robotMajorState << std::endl;
        ss << "  connected: " << +connected << std::endl;
        ss << "  armed: " << +armed << std::endl;
        ss << "  guided: " << +guided << std::endl;
        ss << "  manual_input: " << +manual_input << std::endl;
        ss << "  mode: \"" << to_string(mode) << "\"" << std::endl;
        ss << "  mode_len: " << mode_len << std::endl;
        ss << "  status: " << +status << std::endl;
        ss << "  service: " << service << std::endl;
        ss << "  latitude: " << latitude << std::endl;
        ss << "  longitude: " << longitude << std::endl;
        ss << "  altitude: " << altitude << std::endl;
        ss << "  satellite_count: " << satellite_count << std::endl;
        ss << "  voltage: " << voltage << std::endl;
        ss << "  yaw: " << yaw << std::endl;
        ss << "  dis: " << dis << std::endl;
        ss << "  dis_vel: " << dis_vel << std::endl;
        ss << "  high: " << high << std::endl;
        ss << "  high_vel: " << high_vel << std::endl;
        ss << "  field_flags: " << field_flags << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << robotMajorState;               // offset: 0
        map << latitude;                      // offset: 4
        map << longitude;                     // offset: 8
        map << altitude;                      // offset: 12
        map << satellite_count;               // offset: 16
        map << voltage;                       // offset: 20
        map << yaw;                           // offset: 24
        map << dis;                           // offset: 28
        map << dis_vel;                       // offset: 32
        map << high;                          // offset: 36
        map << high_vel;                      // offset: 40
        map << mode_len;                      // offset: 44
        map << service;                       // offset: 46
        map << field_flags;                   // offset: 48
        map << connected;                     // offset: 50
        map << armed;                         // offset: 51
        map << guided;                        // offset: 52
        map << manual_input;                  // offset: 53
        map << mode;                          // offset: 54
        map << status;                        // offset: 84
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> robotMajorState;               // offset: 0
        map >> latitude;                      // offset: 4
        map >> longitude;                     // offset: 8
        map >> altitude;                      // offset: 12
        map >> satellite_count;               // offset: 16
        map >> voltage;                       // offset: 20
        map >> yaw;                           // offset: 24
        map >> dis;                           // offset: 28
        map >> dis_vel;                       // offset: 32
        map >> high;                          // offset: 36
        map >> high_vel;                      // offset: 40
        map >> mode_len;                      // offset: 44
        map >> service;                       // offset: 46
        map >> field_flags;                   // offset: 48
        map >> connected;                     // offset: 50
        map >> armed;                         // offset: 51
        map >> guided;                        // offset: 52
        map >> manual_input;                  // offset: 53
        map >> mode;                          // offset: 54
        map >> status;                        // offset: 84
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
