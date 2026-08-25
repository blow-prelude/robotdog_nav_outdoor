#pragma once
// MESSAGE FOLLOW_COVER_PASS_WAYPOINT PACKING

#define MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT 13029


typedef struct __mavlink_follow_cover_pass_waypoint_t {
 uint16_t currentPointId; /*<  当前航点 ID*/
 uint8_t groupId; /*<  任务组 ID*/
} mavlink_follow_cover_pass_waypoint_t;

#define MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN 3
#define MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_MIN_LEN 3
#define MAVLINK_MSG_ID_13029_LEN 3
#define MAVLINK_MSG_ID_13029_MIN_LEN 3

#define MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_CRC 185
#define MAVLINK_MSG_ID_13029_CRC 185



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FOLLOW_COVER_PASS_WAYPOINT { \
    13029, \
    "FOLLOW_COVER_PASS_WAYPOINT", \
    2, \
    {  { "groupId", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_follow_cover_pass_waypoint_t, groupId) }, \
         { "currentPointId", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_follow_cover_pass_waypoint_t, currentPointId) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FOLLOW_COVER_PASS_WAYPOINT { \
    "FOLLOW_COVER_PASS_WAYPOINT", \
    2, \
    {  { "groupId", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_follow_cover_pass_waypoint_t, groupId) }, \
         { "currentPointId", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_follow_cover_pass_waypoint_t, currentPointId) }, \
         } \
}
#endif

/**
 * @brief Pack a follow_cover_pass_waypoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param groupId  任务组 ID
 * @param currentPointId  当前航点 ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_follow_cover_pass_waypoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t groupId, uint16_t currentPointId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN];
    _mav_put_uint16_t(buf, 0, currentPointId);
    _mav_put_uint8_t(buf, 2, groupId);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN);
#else
    mavlink_follow_cover_pass_waypoint_t packet;
    packet.currentPointId = currentPointId;
    packet.groupId = groupId;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_CRC);
}

/**
 * @brief Pack a follow_cover_pass_waypoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param groupId  任务组 ID
 * @param currentPointId  当前航点 ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_follow_cover_pass_waypoint_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t groupId, uint16_t currentPointId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN];
    _mav_put_uint16_t(buf, 0, currentPointId);
    _mav_put_uint8_t(buf, 2, groupId);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN);
#else
    mavlink_follow_cover_pass_waypoint_t packet;
    packet.currentPointId = currentPointId;
    packet.groupId = groupId;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN);
#endif
}

/**
 * @brief Pack a follow_cover_pass_waypoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param groupId  任务组 ID
 * @param currentPointId  当前航点 ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_follow_cover_pass_waypoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t groupId,uint16_t currentPointId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN];
    _mav_put_uint16_t(buf, 0, currentPointId);
    _mav_put_uint8_t(buf, 2, groupId);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN);
#else
    mavlink_follow_cover_pass_waypoint_t packet;
    packet.currentPointId = currentPointId;
    packet.groupId = groupId;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_CRC);
}

/**
 * @brief Encode a follow_cover_pass_waypoint struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param follow_cover_pass_waypoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_follow_cover_pass_waypoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_follow_cover_pass_waypoint_t* follow_cover_pass_waypoint)
{
    return mavlink_msg_follow_cover_pass_waypoint_pack(system_id, component_id, msg, follow_cover_pass_waypoint->groupId, follow_cover_pass_waypoint->currentPointId);
}

/**
 * @brief Encode a follow_cover_pass_waypoint struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param follow_cover_pass_waypoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_follow_cover_pass_waypoint_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_follow_cover_pass_waypoint_t* follow_cover_pass_waypoint)
{
    return mavlink_msg_follow_cover_pass_waypoint_pack_chan(system_id, component_id, chan, msg, follow_cover_pass_waypoint->groupId, follow_cover_pass_waypoint->currentPointId);
}

/**
 * @brief Encode a follow_cover_pass_waypoint struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param follow_cover_pass_waypoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_follow_cover_pass_waypoint_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_follow_cover_pass_waypoint_t* follow_cover_pass_waypoint)
{
    return mavlink_msg_follow_cover_pass_waypoint_pack_status(system_id, component_id, _status, msg,  follow_cover_pass_waypoint->groupId, follow_cover_pass_waypoint->currentPointId);
}

/**
 * @brief Send a follow_cover_pass_waypoint message
 * @param chan MAVLink channel to send the message
 *
 * @param groupId  任务组 ID
 * @param currentPointId  当前航点 ID
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_follow_cover_pass_waypoint_send(mavlink_channel_t chan, uint8_t groupId, uint16_t currentPointId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN];
    _mav_put_uint16_t(buf, 0, currentPointId);
    _mav_put_uint8_t(buf, 2, groupId);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT, buf, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_CRC);
#else
    mavlink_follow_cover_pass_waypoint_t packet;
    packet.currentPointId = currentPointId;
    packet.groupId = groupId;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT, (const char *)&packet, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_CRC);
#endif
}

/**
 * @brief Send a follow_cover_pass_waypoint message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_follow_cover_pass_waypoint_send_struct(mavlink_channel_t chan, const mavlink_follow_cover_pass_waypoint_t* follow_cover_pass_waypoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_follow_cover_pass_waypoint_send(chan, follow_cover_pass_waypoint->groupId, follow_cover_pass_waypoint->currentPointId);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT, (const char *)follow_cover_pass_waypoint, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_follow_cover_pass_waypoint_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t groupId, uint16_t currentPointId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, currentPointId);
    _mav_put_uint8_t(buf, 2, groupId);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT, buf, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_CRC);
#else
    mavlink_follow_cover_pass_waypoint_t *packet = (mavlink_follow_cover_pass_waypoint_t *)msgbuf;
    packet->currentPointId = currentPointId;
    packet->groupId = groupId;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT, (const char *)packet, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_MIN_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_CRC);
#endif
}
#endif

#endif

// MESSAGE FOLLOW_COVER_PASS_WAYPOINT UNPACKING


/**
 * @brief Get field groupId from follow_cover_pass_waypoint message
 *
 * @return  任务组 ID
 */
static inline uint8_t mavlink_msg_follow_cover_pass_waypoint_get_groupId(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field currentPointId from follow_cover_pass_waypoint message
 *
 * @return  当前航点 ID
 */
static inline uint16_t mavlink_msg_follow_cover_pass_waypoint_get_currentPointId(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a follow_cover_pass_waypoint message into a struct
 *
 * @param msg The message to decode
 * @param follow_cover_pass_waypoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_follow_cover_pass_waypoint_decode(const mavlink_message_t* msg, mavlink_follow_cover_pass_waypoint_t* follow_cover_pass_waypoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    follow_cover_pass_waypoint->currentPointId = mavlink_msg_follow_cover_pass_waypoint_get_currentPointId(msg);
    follow_cover_pass_waypoint->groupId = mavlink_msg_follow_cover_pass_waypoint_get_groupId(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN? msg->len : MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN;
        memset(follow_cover_pass_waypoint, 0, MAVLINK_MSG_ID_FOLLOW_COVER_PASS_WAYPOINT_LEN);
    memcpy(follow_cover_pass_waypoint, _MAV_PAYLOAD(msg), len);
#endif
}
