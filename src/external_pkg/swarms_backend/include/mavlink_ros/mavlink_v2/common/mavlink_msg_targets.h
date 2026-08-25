#pragma once
// MESSAGE TARGETS PACKING

#define MAVLINK_MSG_ID_TARGETS 13031


typedef struct __mavlink_targets_t {
 int32_t target[55]; /*<  每批最大数量目标信息*/
 int32_t batch_target_count; /*<  每批实际目标数量*/
 int32_t total_target_count; /*<  目标总数量*/
 int32_t total_batches; /*<  总批次数量*/
 int32_t current_batch; /*<  当前批次索引*/
} mavlink_targets_t;

#define MAVLINK_MSG_ID_TARGETS_LEN 236
#define MAVLINK_MSG_ID_TARGETS_MIN_LEN 236
#define MAVLINK_MSG_ID_13031_LEN 236
#define MAVLINK_MSG_ID_13031_MIN_LEN 236

#define MAVLINK_MSG_ID_TARGETS_CRC 215
#define MAVLINK_MSG_ID_13031_CRC 215

#define MAVLINK_MSG_TARGETS_FIELD_TARGET_LEN 55

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TARGETS { \
    13031, \
    "TARGETS", \
    5, \
    {  { "target", NULL, MAVLINK_TYPE_INT32_T, 55, 0, offsetof(mavlink_targets_t, target) }, \
         { "batch_target_count", NULL, MAVLINK_TYPE_INT32_T, 0, 220, offsetof(mavlink_targets_t, batch_target_count) }, \
         { "total_target_count", NULL, MAVLINK_TYPE_INT32_T, 0, 224, offsetof(mavlink_targets_t, total_target_count) }, \
         { "total_batches", NULL, MAVLINK_TYPE_INT32_T, 0, 228, offsetof(mavlink_targets_t, total_batches) }, \
         { "current_batch", NULL, MAVLINK_TYPE_INT32_T, 0, 232, offsetof(mavlink_targets_t, current_batch) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TARGETS { \
    "TARGETS", \
    5, \
    {  { "target", NULL, MAVLINK_TYPE_INT32_T, 55, 0, offsetof(mavlink_targets_t, target) }, \
         { "batch_target_count", NULL, MAVLINK_TYPE_INT32_T, 0, 220, offsetof(mavlink_targets_t, batch_target_count) }, \
         { "total_target_count", NULL, MAVLINK_TYPE_INT32_T, 0, 224, offsetof(mavlink_targets_t, total_target_count) }, \
         { "total_batches", NULL, MAVLINK_TYPE_INT32_T, 0, 228, offsetof(mavlink_targets_t, total_batches) }, \
         { "current_batch", NULL, MAVLINK_TYPE_INT32_T, 0, 232, offsetof(mavlink_targets_t, current_batch) }, \
         } \
}
#endif

/**
 * @brief Pack a targets message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target  每批最大数量目标信息
 * @param batch_target_count  每批实际目标数量
 * @param total_target_count  目标总数量
 * @param total_batches  总批次数量
 * @param current_batch  当前批次索引
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_targets_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const int32_t *target, int32_t batch_target_count, int32_t total_target_count, int32_t total_batches, int32_t current_batch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGETS_LEN];
    _mav_put_int32_t(buf, 220, batch_target_count);
    _mav_put_int32_t(buf, 224, total_target_count);
    _mav_put_int32_t(buf, 228, total_batches);
    _mav_put_int32_t(buf, 232, current_batch);
    _mav_put_int32_t_array(buf, 0, target, 55);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGETS_LEN);
#else
    mavlink_targets_t packet;
    packet.batch_target_count = batch_target_count;
    packet.total_target_count = total_target_count;
    packet.total_batches = total_batches;
    packet.current_batch = current_batch;
    mav_array_memcpy(packet.target, target, sizeof(int32_t)*55);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGETS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGETS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TARGETS_MIN_LEN, MAVLINK_MSG_ID_TARGETS_LEN, MAVLINK_MSG_ID_TARGETS_CRC);
}

/**
 * @brief Pack a targets message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target  每批最大数量目标信息
 * @param batch_target_count  每批实际目标数量
 * @param total_target_count  目标总数量
 * @param total_batches  总批次数量
 * @param current_batch  当前批次索引
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_targets_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const int32_t *target, int32_t batch_target_count, int32_t total_target_count, int32_t total_batches, int32_t current_batch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGETS_LEN];
    _mav_put_int32_t(buf, 220, batch_target_count);
    _mav_put_int32_t(buf, 224, total_target_count);
    _mav_put_int32_t(buf, 228, total_batches);
    _mav_put_int32_t(buf, 232, current_batch);
    _mav_put_int32_t_array(buf, 0, target, 55);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGETS_LEN);
#else
    mavlink_targets_t packet;
    packet.batch_target_count = batch_target_count;
    packet.total_target_count = total_target_count;
    packet.total_batches = total_batches;
    packet.current_batch = current_batch;
    mav_array_memcpy(packet.target, target, sizeof(int32_t)*55);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGETS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGETS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TARGETS_MIN_LEN, MAVLINK_MSG_ID_TARGETS_LEN, MAVLINK_MSG_ID_TARGETS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TARGETS_MIN_LEN, MAVLINK_MSG_ID_TARGETS_LEN);
#endif
}

/**
 * @brief Pack a targets message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target  每批最大数量目标信息
 * @param batch_target_count  每批实际目标数量
 * @param total_target_count  目标总数量
 * @param total_batches  总批次数量
 * @param current_batch  当前批次索引
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_targets_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const int32_t *target,int32_t batch_target_count,int32_t total_target_count,int32_t total_batches,int32_t current_batch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGETS_LEN];
    _mav_put_int32_t(buf, 220, batch_target_count);
    _mav_put_int32_t(buf, 224, total_target_count);
    _mav_put_int32_t(buf, 228, total_batches);
    _mav_put_int32_t(buf, 232, current_batch);
    _mav_put_int32_t_array(buf, 0, target, 55);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGETS_LEN);
#else
    mavlink_targets_t packet;
    packet.batch_target_count = batch_target_count;
    packet.total_target_count = total_target_count;
    packet.total_batches = total_batches;
    packet.current_batch = current_batch;
    mav_array_memcpy(packet.target, target, sizeof(int32_t)*55);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGETS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGETS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TARGETS_MIN_LEN, MAVLINK_MSG_ID_TARGETS_LEN, MAVLINK_MSG_ID_TARGETS_CRC);
}

/**
 * @brief Encode a targets struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param targets C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_targets_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_targets_t* targets)
{
    return mavlink_msg_targets_pack(system_id, component_id, msg, targets->target, targets->batch_target_count, targets->total_target_count, targets->total_batches, targets->current_batch);
}

/**
 * @brief Encode a targets struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param targets C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_targets_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_targets_t* targets)
{
    return mavlink_msg_targets_pack_chan(system_id, component_id, chan, msg, targets->target, targets->batch_target_count, targets->total_target_count, targets->total_batches, targets->current_batch);
}

/**
 * @brief Encode a targets struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param targets C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_targets_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_targets_t* targets)
{
    return mavlink_msg_targets_pack_status(system_id, component_id, _status, msg,  targets->target, targets->batch_target_count, targets->total_target_count, targets->total_batches, targets->current_batch);
}

/**
 * @brief Send a targets message
 * @param chan MAVLink channel to send the message
 *
 * @param target  每批最大数量目标信息
 * @param batch_target_count  每批实际目标数量
 * @param total_target_count  目标总数量
 * @param total_batches  总批次数量
 * @param current_batch  当前批次索引
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_targets_send(mavlink_channel_t chan, const int32_t *target, int32_t batch_target_count, int32_t total_target_count, int32_t total_batches, int32_t current_batch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGETS_LEN];
    _mav_put_int32_t(buf, 220, batch_target_count);
    _mav_put_int32_t(buf, 224, total_target_count);
    _mav_put_int32_t(buf, 228, total_batches);
    _mav_put_int32_t(buf, 232, current_batch);
    _mav_put_int32_t_array(buf, 0, target, 55);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGETS, buf, MAVLINK_MSG_ID_TARGETS_MIN_LEN, MAVLINK_MSG_ID_TARGETS_LEN, MAVLINK_MSG_ID_TARGETS_CRC);
#else
    mavlink_targets_t packet;
    packet.batch_target_count = batch_target_count;
    packet.total_target_count = total_target_count;
    packet.total_batches = total_batches;
    packet.current_batch = current_batch;
    mav_array_memcpy(packet.target, target, sizeof(int32_t)*55);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGETS, (const char *)&packet, MAVLINK_MSG_ID_TARGETS_MIN_LEN, MAVLINK_MSG_ID_TARGETS_LEN, MAVLINK_MSG_ID_TARGETS_CRC);
#endif
}

/**
 * @brief Send a targets message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_targets_send_struct(mavlink_channel_t chan, const mavlink_targets_t* targets)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_targets_send(chan, targets->target, targets->batch_target_count, targets->total_target_count, targets->total_batches, targets->current_batch);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGETS, (const char *)targets, MAVLINK_MSG_ID_TARGETS_MIN_LEN, MAVLINK_MSG_ID_TARGETS_LEN, MAVLINK_MSG_ID_TARGETS_CRC);
#endif
}

#if MAVLINK_MSG_ID_TARGETS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_targets_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const int32_t *target, int32_t batch_target_count, int32_t total_target_count, int32_t total_batches, int32_t current_batch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 220, batch_target_count);
    _mav_put_int32_t(buf, 224, total_target_count);
    _mav_put_int32_t(buf, 228, total_batches);
    _mav_put_int32_t(buf, 232, current_batch);
    _mav_put_int32_t_array(buf, 0, target, 55);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGETS, buf, MAVLINK_MSG_ID_TARGETS_MIN_LEN, MAVLINK_MSG_ID_TARGETS_LEN, MAVLINK_MSG_ID_TARGETS_CRC);
#else
    mavlink_targets_t *packet = (mavlink_targets_t *)msgbuf;
    packet->batch_target_count = batch_target_count;
    packet->total_target_count = total_target_count;
    packet->total_batches = total_batches;
    packet->current_batch = current_batch;
    mav_array_memcpy(packet->target, target, sizeof(int32_t)*55);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGETS, (const char *)packet, MAVLINK_MSG_ID_TARGETS_MIN_LEN, MAVLINK_MSG_ID_TARGETS_LEN, MAVLINK_MSG_ID_TARGETS_CRC);
#endif
}
#endif

#endif

// MESSAGE TARGETS UNPACKING


/**
 * @brief Get field target from targets message
 *
 * @return  每批最大数量目标信息
 */
static inline uint16_t mavlink_msg_targets_get_target(const mavlink_message_t* msg, int32_t *target)
{
    return _MAV_RETURN_int32_t_array(msg, target, 55,  0);
}

/**
 * @brief Get field batch_target_count from targets message
 *
 * @return  每批实际目标数量
 */
static inline int32_t mavlink_msg_targets_get_batch_target_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  220);
}

/**
 * @brief Get field total_target_count from targets message
 *
 * @return  目标总数量
 */
static inline int32_t mavlink_msg_targets_get_total_target_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  224);
}

/**
 * @brief Get field total_batches from targets message
 *
 * @return  总批次数量
 */
static inline int32_t mavlink_msg_targets_get_total_batches(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  228);
}

/**
 * @brief Get field current_batch from targets message
 *
 * @return  当前批次索引
 */
static inline int32_t mavlink_msg_targets_get_current_batch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  232);
}

/**
 * @brief Decode a targets message into a struct
 *
 * @param msg The message to decode
 * @param targets C-struct to decode the message contents into
 */
static inline void mavlink_msg_targets_decode(const mavlink_message_t* msg, mavlink_targets_t* targets)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_targets_get_target(msg, targets->target);
    targets->batch_target_count = mavlink_msg_targets_get_batch_target_count(msg);
    targets->total_target_count = mavlink_msg_targets_get_total_target_count(msg);
    targets->total_batches = mavlink_msg_targets_get_total_batches(msg);
    targets->current_batch = mavlink_msg_targets_get_current_batch(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TARGETS_LEN? msg->len : MAVLINK_MSG_ID_TARGETS_LEN;
        memset(targets, 0, MAVLINK_MSG_ID_TARGETS_LEN);
    memcpy(targets, _MAV_PAYLOAD(msg), len);
#endif
}
