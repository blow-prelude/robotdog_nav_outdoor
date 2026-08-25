#pragma once
// MESSAGE CAN_CONFIRMED_FLAG PACKING

#define MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG 13030


typedef struct __mavlink_can_confirmed_flag_t {
 uint8_t data; /*<  确认标志数据*/
} mavlink_can_confirmed_flag_t;

#define MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN 1
#define MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_MIN_LEN 1
#define MAVLINK_MSG_ID_13030_LEN 1
#define MAVLINK_MSG_ID_13030_MIN_LEN 1

#define MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_CRC 53
#define MAVLINK_MSG_ID_13030_CRC 53



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAN_CONFIRMED_FLAG { \
    13030, \
    "CAN_CONFIRMED_FLAG", \
    1, \
    {  { "data", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_can_confirmed_flag_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAN_CONFIRMED_FLAG { \
    "CAN_CONFIRMED_FLAG", \
    1, \
    {  { "data", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_can_confirmed_flag_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a can_confirmed_flag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param data  确认标志数据
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_confirmed_flag_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN];
    _mav_put_uint8_t(buf, 0, data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN);
#else
    mavlink_can_confirmed_flag_t packet;
    packet.data = data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_MIN_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_CRC);
}

/**
 * @brief Pack a can_confirmed_flag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param data  确认标志数据
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_confirmed_flag_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN];
    _mav_put_uint8_t(buf, 0, data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN);
#else
    mavlink_can_confirmed_flag_t packet;
    packet.data = data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_MIN_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_MIN_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN);
#endif
}

/**
 * @brief Pack a can_confirmed_flag message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param data  确认标志数据
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_confirmed_flag_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN];
    _mav_put_uint8_t(buf, 0, data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN);
#else
    mavlink_can_confirmed_flag_t packet;
    packet.data = data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_MIN_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_CRC);
}

/**
 * @brief Encode a can_confirmed_flag struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param can_confirmed_flag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_confirmed_flag_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_can_confirmed_flag_t* can_confirmed_flag)
{
    return mavlink_msg_can_confirmed_flag_pack(system_id, component_id, msg, can_confirmed_flag->data);
}

/**
 * @brief Encode a can_confirmed_flag struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param can_confirmed_flag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_confirmed_flag_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_can_confirmed_flag_t* can_confirmed_flag)
{
    return mavlink_msg_can_confirmed_flag_pack_chan(system_id, component_id, chan, msg, can_confirmed_flag->data);
}

/**
 * @brief Encode a can_confirmed_flag struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param can_confirmed_flag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_confirmed_flag_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_can_confirmed_flag_t* can_confirmed_flag)
{
    return mavlink_msg_can_confirmed_flag_pack_status(system_id, component_id, _status, msg,  can_confirmed_flag->data);
}

/**
 * @brief Send a can_confirmed_flag message
 * @param chan MAVLink channel to send the message
 *
 * @param data  确认标志数据
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_can_confirmed_flag_send(mavlink_channel_t chan, uint8_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN];
    _mav_put_uint8_t(buf, 0, data);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG, buf, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_MIN_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_CRC);
#else
    mavlink_can_confirmed_flag_t packet;
    packet.data = data;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG, (const char *)&packet, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_MIN_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_CRC);
#endif
}

/**
 * @brief Send a can_confirmed_flag message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_can_confirmed_flag_send_struct(mavlink_channel_t chan, const mavlink_can_confirmed_flag_t* can_confirmed_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_can_confirmed_flag_send(chan, can_confirmed_flag->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG, (const char *)can_confirmed_flag, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_MIN_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_can_confirmed_flag_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, data);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG, buf, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_MIN_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_CRC);
#else
    mavlink_can_confirmed_flag_t *packet = (mavlink_can_confirmed_flag_t *)msgbuf;
    packet->data = data;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG, (const char *)packet, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_MIN_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_CRC);
#endif
}
#endif

#endif

// MESSAGE CAN_CONFIRMED_FLAG UNPACKING


/**
 * @brief Get field data from can_confirmed_flag message
 *
 * @return  确认标志数据
 */
static inline uint8_t mavlink_msg_can_confirmed_flag_get_data(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a can_confirmed_flag message into a struct
 *
 * @param msg The message to decode
 * @param can_confirmed_flag C-struct to decode the message contents into
 */
static inline void mavlink_msg_can_confirmed_flag_decode(const mavlink_message_t* msg, mavlink_can_confirmed_flag_t* can_confirmed_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    can_confirmed_flag->data = mavlink_msg_can_confirmed_flag_get_data(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN? msg->len : MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN;
        memset(can_confirmed_flag, 0, MAVLINK_MSG_ID_CAN_CONFIRMED_FLAG_LEN);
    memcpy(can_confirmed_flag, _MAV_PAYLOAD(msg), len);
#endif
}
