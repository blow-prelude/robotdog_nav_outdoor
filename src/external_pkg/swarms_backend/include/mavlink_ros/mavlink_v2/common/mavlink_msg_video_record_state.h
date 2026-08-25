#pragma once
// MESSAGE VIDEO_RECORD_STATE PACKING

#define MAVLINK_MSG_ID_VIDEO_RECORD_STATE 13033


typedef struct __mavlink_video_record_state_t {
 int8_t data; /*<  确认标志数据*/
} mavlink_video_record_state_t;

#define MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN 1
#define MAVLINK_MSG_ID_VIDEO_RECORD_STATE_MIN_LEN 1
#define MAVLINK_MSG_ID_13033_LEN 1
#define MAVLINK_MSG_ID_13033_MIN_LEN 1

#define MAVLINK_MSG_ID_VIDEO_RECORD_STATE_CRC 151
#define MAVLINK_MSG_ID_13033_CRC 151



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VIDEO_RECORD_STATE { \
    13033, \
    "VIDEO_RECORD_STATE", \
    1, \
    {  { "data", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_video_record_state_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VIDEO_RECORD_STATE { \
    "VIDEO_RECORD_STATE", \
    1, \
    {  { "data", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_video_record_state_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a video_record_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param data  确认标志数据
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_video_record_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int8_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN];
    _mav_put_int8_t(buf, 0, data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN);
#else
    mavlink_video_record_state_t packet;
    packet.data = data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIDEO_RECORD_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_CRC);
}

/**
 * @brief Pack a video_record_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param data  确认标志数据
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_video_record_state_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int8_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN];
    _mav_put_int8_t(buf, 0, data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN);
#else
    mavlink_video_record_state_t packet;
    packet.data = data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIDEO_RECORD_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN);
#endif
}

/**
 * @brief Pack a video_record_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param data  确认标志数据
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_video_record_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int8_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN];
    _mav_put_int8_t(buf, 0, data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN);
#else
    mavlink_video_record_state_t packet;
    packet.data = data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIDEO_RECORD_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_CRC);
}

/**
 * @brief Encode a video_record_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param video_record_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_video_record_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_video_record_state_t* video_record_state)
{
    return mavlink_msg_video_record_state_pack(system_id, component_id, msg, video_record_state->data);
}

/**
 * @brief Encode a video_record_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param video_record_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_video_record_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_video_record_state_t* video_record_state)
{
    return mavlink_msg_video_record_state_pack_chan(system_id, component_id, chan, msg, video_record_state->data);
}

/**
 * @brief Encode a video_record_state struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param video_record_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_video_record_state_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_video_record_state_t* video_record_state)
{
    return mavlink_msg_video_record_state_pack_status(system_id, component_id, _status, msg,  video_record_state->data);
}

/**
 * @brief Send a video_record_state message
 * @param chan MAVLink channel to send the message
 *
 * @param data  确认标志数据
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_video_record_state_send(mavlink_channel_t chan, int8_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN];
    _mav_put_int8_t(buf, 0, data);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIDEO_RECORD_STATE, buf, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_CRC);
#else
    mavlink_video_record_state_t packet;
    packet.data = data;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIDEO_RECORD_STATE, (const char *)&packet, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_CRC);
#endif
}

/**
 * @brief Send a video_record_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_video_record_state_send_struct(mavlink_channel_t chan, const mavlink_video_record_state_t* video_record_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_video_record_state_send(chan, video_record_state->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIDEO_RECORD_STATE, (const char *)video_record_state, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_video_record_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int8_t data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int8_t(buf, 0, data);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIDEO_RECORD_STATE, buf, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_CRC);
#else
    mavlink_video_record_state_t *packet = (mavlink_video_record_state_t *)msgbuf;
    packet->data = data;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIDEO_RECORD_STATE, (const char *)packet, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE VIDEO_RECORD_STATE UNPACKING


/**
 * @brief Get field data from video_record_state message
 *
 * @return  确认标志数据
 */
static inline int8_t mavlink_msg_video_record_state_get_data(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  0);
}

/**
 * @brief Decode a video_record_state message into a struct
 *
 * @param msg The message to decode
 * @param video_record_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_video_record_state_decode(const mavlink_message_t* msg, mavlink_video_record_state_t* video_record_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    video_record_state->data = mavlink_msg_video_record_state_get_data(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN? msg->len : MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN;
        memset(video_record_state, 0, MAVLINK_MSG_ID_VIDEO_RECORD_STATE_LEN);
    memcpy(video_record_state, _MAV_PAYLOAD(msg), len);
#endif
}
