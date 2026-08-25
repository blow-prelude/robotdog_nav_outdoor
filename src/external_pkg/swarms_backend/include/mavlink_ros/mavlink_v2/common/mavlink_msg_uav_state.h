#pragma once
// MESSAGE UAV_STATE PACKING

#define MAVLINK_MSG_ID_UAV_STATE 13027


typedef struct __mavlink_uav_state_t {
 int32_t robotMajorState; /*<  机器人主要状态*/
 int32_t latitude; /*< [degE7] 纬度，单位为度*1E7*/
 int32_t longitude; /*< [degE7] 经度，单位为度*1E7*/
 int32_t altitude; /*< [degE5] 海拔高度，单位为毫米*/
 uint32_t satellite_count; /*<  卫星数量*/
 float voltage; /*<  电压*/
 float yaw; /*<  yaw角*/
 float dis; /*<  水平距离*/
 float dis_vel; /*<  水平速度*/
 float high; /*<  垂直高度*/
 float high_vel; /*<  垂直速度*/
 uint16_t mode_len; /*<  飞行模式名称长度*/
 uint16_t service; /*<  服务标识符*/
 int16_t field_flags; /*<  字段有效标志位*/
 uint8_t connected; /*<  连接状态（0：未连接，1：已连接）*/
 uint8_t armed; /*<  解锁状态（0：未解锁，1：已解锁）*/
 uint8_t guided; /*<  引导模式（0：手动，1：引导）*/
 uint8_t manual_input; /*<  手动输入状态（0：未启用，1：启用）*/
 char mode[30]; /*<  飞行模式名称*/
 int8_t status; /*<  系统状态（-128 至 127）*/
} mavlink_uav_state_t;

#define MAVLINK_MSG_ID_UAV_STATE_LEN 85
#define MAVLINK_MSG_ID_UAV_STATE_MIN_LEN 85
#define MAVLINK_MSG_ID_13027_LEN 85
#define MAVLINK_MSG_ID_13027_MIN_LEN 85

#define MAVLINK_MSG_ID_UAV_STATE_CRC 219
#define MAVLINK_MSG_ID_13027_CRC 219

#define MAVLINK_MSG_UAV_STATE_FIELD_MODE_LEN 30

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAV_STATE { \
    13027, \
    "UAV_STATE", \
    20, \
    {  { "robotMajorState", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_uav_state_t, robotMajorState) }, \
         { "connected", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_uav_state_t, connected) }, \
         { "armed", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_uav_state_t, armed) }, \
         { "guided", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_uav_state_t, guided) }, \
         { "manual_input", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_uav_state_t, manual_input) }, \
         { "mode", NULL, MAVLINK_TYPE_CHAR, 30, 54, offsetof(mavlink_uav_state_t, mode) }, \
         { "mode_len", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_uav_state_t, mode_len) }, \
         { "status", NULL, MAVLINK_TYPE_INT8_T, 0, 84, offsetof(mavlink_uav_state_t, status) }, \
         { "service", NULL, MAVLINK_TYPE_UINT16_T, 0, 46, offsetof(mavlink_uav_state_t, service) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_uav_state_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_uav_state_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_uav_state_t, altitude) }, \
         { "satellite_count", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_uav_state_t, satellite_count) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_uav_state_t, voltage) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_uav_state_t, yaw) }, \
         { "dis", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_uav_state_t, dis) }, \
         { "dis_vel", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_uav_state_t, dis_vel) }, \
         { "high", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_uav_state_t, high) }, \
         { "high_vel", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_uav_state_t, high_vel) }, \
         { "field_flags", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_uav_state_t, field_flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAV_STATE { \
    "UAV_STATE", \
    20, \
    {  { "robotMajorState", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_uav_state_t, robotMajorState) }, \
         { "connected", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_uav_state_t, connected) }, \
         { "armed", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_uav_state_t, armed) }, \
         { "guided", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_uav_state_t, guided) }, \
         { "manual_input", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_uav_state_t, manual_input) }, \
         { "mode", NULL, MAVLINK_TYPE_CHAR, 30, 54, offsetof(mavlink_uav_state_t, mode) }, \
         { "mode_len", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_uav_state_t, mode_len) }, \
         { "status", NULL, MAVLINK_TYPE_INT8_T, 0, 84, offsetof(mavlink_uav_state_t, status) }, \
         { "service", NULL, MAVLINK_TYPE_UINT16_T, 0, 46, offsetof(mavlink_uav_state_t, service) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_uav_state_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_uav_state_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_uav_state_t, altitude) }, \
         { "satellite_count", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_uav_state_t, satellite_count) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_uav_state_t, voltage) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_uav_state_t, yaw) }, \
         { "dis", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_uav_state_t, dis) }, \
         { "dis_vel", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_uav_state_t, dis_vel) }, \
         { "high", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_uav_state_t, high) }, \
         { "high_vel", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_uav_state_t, high_vel) }, \
         { "field_flags", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_uav_state_t, field_flags) }, \
         } \
}
#endif

/**
 * @brief Pack a uav_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param robotMajorState  机器人主要状态
 * @param connected  连接状态（0：未连接，1：已连接）
 * @param armed  解锁状态（0：未解锁，1：已解锁）
 * @param guided  引导模式（0：手动，1：引导）
 * @param manual_input  手动输入状态（0：未启用，1：启用）
 * @param mode  飞行模式名称
 * @param mode_len  飞行模式名称长度
 * @param status  系统状态（-128 至 127）
 * @param service  服务标识符
 * @param latitude [degE7] 纬度，单位为度*1E7
 * @param longitude [degE7] 经度，单位为度*1E7
 * @param altitude [degE5] 海拔高度，单位为毫米
 * @param satellite_count  卫星数量
 * @param voltage  电压
 * @param yaw  yaw角
 * @param dis  水平距离
 * @param dis_vel  水平速度
 * @param high  垂直高度
 * @param high_vel  垂直速度
 * @param field_flags  字段有效标志位
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t robotMajorState, uint8_t connected, uint8_t armed, uint8_t guided, uint8_t manual_input, const char *mode, uint16_t mode_len, int8_t status, uint16_t service, int32_t latitude, int32_t longitude, int32_t altitude, uint32_t satellite_count, float voltage, float yaw, float dis, float dis_vel, float high, float high_vel, int16_t field_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_STATE_LEN];
    _mav_put_int32_t(buf, 0, robotMajorState);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int32_t(buf, 12, altitude);
    _mav_put_uint32_t(buf, 16, satellite_count);
    _mav_put_float(buf, 20, voltage);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, dis);
    _mav_put_float(buf, 32, dis_vel);
    _mav_put_float(buf, 36, high);
    _mav_put_float(buf, 40, high_vel);
    _mav_put_uint16_t(buf, 44, mode_len);
    _mav_put_uint16_t(buf, 46, service);
    _mav_put_int16_t(buf, 48, field_flags);
    _mav_put_uint8_t(buf, 50, connected);
    _mav_put_uint8_t(buf, 51, armed);
    _mav_put_uint8_t(buf, 52, guided);
    _mav_put_uint8_t(buf, 53, manual_input);
    _mav_put_int8_t(buf, 84, status);
    _mav_put_char_array(buf, 54, mode, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_STATE_LEN);
#else
    mavlink_uav_state_t packet;
    packet.robotMajorState = robotMajorState;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.satellite_count = satellite_count;
    packet.voltage = voltage;
    packet.yaw = yaw;
    packet.dis = dis;
    packet.dis_vel = dis_vel;
    packet.high = high;
    packet.high_vel = high_vel;
    packet.mode_len = mode_len;
    packet.service = service;
    packet.field_flags = field_flags;
    packet.connected = connected;
    packet.armed = armed;
    packet.guided = guided;
    packet.manual_input = manual_input;
    packet.status = status;
    mav_array_memcpy(packet.mode, mode, sizeof(char)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_UAV_STATE_LEN, MAVLINK_MSG_ID_UAV_STATE_CRC);
}

/**
 * @brief Pack a uav_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param robotMajorState  机器人主要状态
 * @param connected  连接状态（0：未连接，1：已连接）
 * @param armed  解锁状态（0：未解锁，1：已解锁）
 * @param guided  引导模式（0：手动，1：引导）
 * @param manual_input  手动输入状态（0：未启用，1：启用）
 * @param mode  飞行模式名称
 * @param mode_len  飞行模式名称长度
 * @param status  系统状态（-128 至 127）
 * @param service  服务标识符
 * @param latitude [degE7] 纬度，单位为度*1E7
 * @param longitude [degE7] 经度，单位为度*1E7
 * @param altitude [degE5] 海拔高度，单位为毫米
 * @param satellite_count  卫星数量
 * @param voltage  电压
 * @param yaw  yaw角
 * @param dis  水平距离
 * @param dis_vel  水平速度
 * @param high  垂直高度
 * @param high_vel  垂直速度
 * @param field_flags  字段有效标志位
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_state_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int32_t robotMajorState, uint8_t connected, uint8_t armed, uint8_t guided, uint8_t manual_input, const char *mode, uint16_t mode_len, int8_t status, uint16_t service, int32_t latitude, int32_t longitude, int32_t altitude, uint32_t satellite_count, float voltage, float yaw, float dis, float dis_vel, float high, float high_vel, int16_t field_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_STATE_LEN];
    _mav_put_int32_t(buf, 0, robotMajorState);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int32_t(buf, 12, altitude);
    _mav_put_uint32_t(buf, 16, satellite_count);
    _mav_put_float(buf, 20, voltage);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, dis);
    _mav_put_float(buf, 32, dis_vel);
    _mav_put_float(buf, 36, high);
    _mav_put_float(buf, 40, high_vel);
    _mav_put_uint16_t(buf, 44, mode_len);
    _mav_put_uint16_t(buf, 46, service);
    _mav_put_int16_t(buf, 48, field_flags);
    _mav_put_uint8_t(buf, 50, connected);
    _mav_put_uint8_t(buf, 51, armed);
    _mav_put_uint8_t(buf, 52, guided);
    _mav_put_uint8_t(buf, 53, manual_input);
    _mav_put_int8_t(buf, 84, status);
    _mav_put_char_array(buf, 54, mode, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_STATE_LEN);
#else
    mavlink_uav_state_t packet;
    packet.robotMajorState = robotMajorState;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.satellite_count = satellite_count;
    packet.voltage = voltage;
    packet.yaw = yaw;
    packet.dis = dis;
    packet.dis_vel = dis_vel;
    packet.high = high;
    packet.high_vel = high_vel;
    packet.mode_len = mode_len;
    packet.service = service;
    packet.field_flags = field_flags;
    packet.connected = connected;
    packet.armed = armed;
    packet.guided = guided;
    packet.manual_input = manual_input;
    packet.status = status;
    mav_array_memcpy(packet.mode, mode, sizeof(char)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_UAV_STATE_LEN, MAVLINK_MSG_ID_UAV_STATE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_UAV_STATE_LEN);
#endif
}

/**
 * @brief Pack a uav_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param robotMajorState  机器人主要状态
 * @param connected  连接状态（0：未连接，1：已连接）
 * @param armed  解锁状态（0：未解锁，1：已解锁）
 * @param guided  引导模式（0：手动，1：引导）
 * @param manual_input  手动输入状态（0：未启用，1：启用）
 * @param mode  飞行模式名称
 * @param mode_len  飞行模式名称长度
 * @param status  系统状态（-128 至 127）
 * @param service  服务标识符
 * @param latitude [degE7] 纬度，单位为度*1E7
 * @param longitude [degE7] 经度，单位为度*1E7
 * @param altitude [degE5] 海拔高度，单位为毫米
 * @param satellite_count  卫星数量
 * @param voltage  电压
 * @param yaw  yaw角
 * @param dis  水平距离
 * @param dis_vel  水平速度
 * @param high  垂直高度
 * @param high_vel  垂直速度
 * @param field_flags  字段有效标志位
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t robotMajorState,uint8_t connected,uint8_t armed,uint8_t guided,uint8_t manual_input,const char *mode,uint16_t mode_len,int8_t status,uint16_t service,int32_t latitude,int32_t longitude,int32_t altitude,uint32_t satellite_count,float voltage,float yaw,float dis,float dis_vel,float high,float high_vel,int16_t field_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_STATE_LEN];
    _mav_put_int32_t(buf, 0, robotMajorState);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int32_t(buf, 12, altitude);
    _mav_put_uint32_t(buf, 16, satellite_count);
    _mav_put_float(buf, 20, voltage);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, dis);
    _mav_put_float(buf, 32, dis_vel);
    _mav_put_float(buf, 36, high);
    _mav_put_float(buf, 40, high_vel);
    _mav_put_uint16_t(buf, 44, mode_len);
    _mav_put_uint16_t(buf, 46, service);
    _mav_put_int16_t(buf, 48, field_flags);
    _mav_put_uint8_t(buf, 50, connected);
    _mav_put_uint8_t(buf, 51, armed);
    _mav_put_uint8_t(buf, 52, guided);
    _mav_put_uint8_t(buf, 53, manual_input);
    _mav_put_int8_t(buf, 84, status);
    _mav_put_char_array(buf, 54, mode, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_STATE_LEN);
#else
    mavlink_uav_state_t packet;
    packet.robotMajorState = robotMajorState;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.satellite_count = satellite_count;
    packet.voltage = voltage;
    packet.yaw = yaw;
    packet.dis = dis;
    packet.dis_vel = dis_vel;
    packet.high = high;
    packet.high_vel = high_vel;
    packet.mode_len = mode_len;
    packet.service = service;
    packet.field_flags = field_flags;
    packet.connected = connected;
    packet.armed = armed;
    packet.guided = guided;
    packet.manual_input = manual_input;
    packet.status = status;
    mav_array_memcpy(packet.mode, mode, sizeof(char)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_UAV_STATE_LEN, MAVLINK_MSG_ID_UAV_STATE_CRC);
}

/**
 * @brief Encode a uav_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uav_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uav_state_t* uav_state)
{
    return mavlink_msg_uav_state_pack(system_id, component_id, msg, uav_state->robotMajorState, uav_state->connected, uav_state->armed, uav_state->guided, uav_state->manual_input, uav_state->mode, uav_state->mode_len, uav_state->status, uav_state->service, uav_state->latitude, uav_state->longitude, uav_state->altitude, uav_state->satellite_count, uav_state->voltage, uav_state->yaw, uav_state->dis, uav_state->dis_vel, uav_state->high, uav_state->high_vel, uav_state->field_flags);
}

/**
 * @brief Encode a uav_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uav_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uav_state_t* uav_state)
{
    return mavlink_msg_uav_state_pack_chan(system_id, component_id, chan, msg, uav_state->robotMajorState, uav_state->connected, uav_state->armed, uav_state->guided, uav_state->manual_input, uav_state->mode, uav_state->mode_len, uav_state->status, uav_state->service, uav_state->latitude, uav_state->longitude, uav_state->altitude, uav_state->satellite_count, uav_state->voltage, uav_state->yaw, uav_state->dis, uav_state->dis_vel, uav_state->high, uav_state->high_vel, uav_state->field_flags);
}

/**
 * @brief Encode a uav_state struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uav_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_state_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uav_state_t* uav_state)
{
    return mavlink_msg_uav_state_pack_status(system_id, component_id, _status, msg,  uav_state->robotMajorState, uav_state->connected, uav_state->armed, uav_state->guided, uav_state->manual_input, uav_state->mode, uav_state->mode_len, uav_state->status, uav_state->service, uav_state->latitude, uav_state->longitude, uav_state->altitude, uav_state->satellite_count, uav_state->voltage, uav_state->yaw, uav_state->dis, uav_state->dis_vel, uav_state->high, uav_state->high_vel, uav_state->field_flags);
}

/**
 * @brief Send a uav_state message
 * @param chan MAVLink channel to send the message
 *
 * @param robotMajorState  机器人主要状态
 * @param connected  连接状态（0：未连接，1：已连接）
 * @param armed  解锁状态（0：未解锁，1：已解锁）
 * @param guided  引导模式（0：手动，1：引导）
 * @param manual_input  手动输入状态（0：未启用，1：启用）
 * @param mode  飞行模式名称
 * @param mode_len  飞行模式名称长度
 * @param status  系统状态（-128 至 127）
 * @param service  服务标识符
 * @param latitude [degE7] 纬度，单位为度*1E7
 * @param longitude [degE7] 经度，单位为度*1E7
 * @param altitude [degE5] 海拔高度，单位为毫米
 * @param satellite_count  卫星数量
 * @param voltage  电压
 * @param yaw  yaw角
 * @param dis  水平距离
 * @param dis_vel  水平速度
 * @param high  垂直高度
 * @param high_vel  垂直速度
 * @param field_flags  字段有效标志位
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uav_state_send(mavlink_channel_t chan, int32_t robotMajorState, uint8_t connected, uint8_t armed, uint8_t guided, uint8_t manual_input, const char *mode, uint16_t mode_len, int8_t status, uint16_t service, int32_t latitude, int32_t longitude, int32_t altitude, uint32_t satellite_count, float voltage, float yaw, float dis, float dis_vel, float high, float high_vel, int16_t field_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_STATE_LEN];
    _mav_put_int32_t(buf, 0, robotMajorState);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int32_t(buf, 12, altitude);
    _mav_put_uint32_t(buf, 16, satellite_count);
    _mav_put_float(buf, 20, voltage);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, dis);
    _mav_put_float(buf, 32, dis_vel);
    _mav_put_float(buf, 36, high);
    _mav_put_float(buf, 40, high_vel);
    _mav_put_uint16_t(buf, 44, mode_len);
    _mav_put_uint16_t(buf, 46, service);
    _mav_put_int16_t(buf, 48, field_flags);
    _mav_put_uint8_t(buf, 50, connected);
    _mav_put_uint8_t(buf, 51, armed);
    _mav_put_uint8_t(buf, 52, guided);
    _mav_put_uint8_t(buf, 53, manual_input);
    _mav_put_int8_t(buf, 84, status);
    _mav_put_char_array(buf, 54, mode, 30);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_STATE, buf, MAVLINK_MSG_ID_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_UAV_STATE_LEN, MAVLINK_MSG_ID_UAV_STATE_CRC);
#else
    mavlink_uav_state_t packet;
    packet.robotMajorState = robotMajorState;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.satellite_count = satellite_count;
    packet.voltage = voltage;
    packet.yaw = yaw;
    packet.dis = dis;
    packet.dis_vel = dis_vel;
    packet.high = high;
    packet.high_vel = high_vel;
    packet.mode_len = mode_len;
    packet.service = service;
    packet.field_flags = field_flags;
    packet.connected = connected;
    packet.armed = armed;
    packet.guided = guided;
    packet.manual_input = manual_input;
    packet.status = status;
    mav_array_memcpy(packet.mode, mode, sizeof(char)*30);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_STATE, (const char *)&packet, MAVLINK_MSG_ID_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_UAV_STATE_LEN, MAVLINK_MSG_ID_UAV_STATE_CRC);
#endif
}

/**
 * @brief Send a uav_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uav_state_send_struct(mavlink_channel_t chan, const mavlink_uav_state_t* uav_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uav_state_send(chan, uav_state->robotMajorState, uav_state->connected, uav_state->armed, uav_state->guided, uav_state->manual_input, uav_state->mode, uav_state->mode_len, uav_state->status, uav_state->service, uav_state->latitude, uav_state->longitude, uav_state->altitude, uav_state->satellite_count, uav_state->voltage, uav_state->yaw, uav_state->dis, uav_state->dis_vel, uav_state->high, uav_state->high_vel, uav_state->field_flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_STATE, (const char *)uav_state, MAVLINK_MSG_ID_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_UAV_STATE_LEN, MAVLINK_MSG_ID_UAV_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAV_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uav_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t robotMajorState, uint8_t connected, uint8_t armed, uint8_t guided, uint8_t manual_input, const char *mode, uint16_t mode_len, int8_t status, uint16_t service, int32_t latitude, int32_t longitude, int32_t altitude, uint32_t satellite_count, float voltage, float yaw, float dis, float dis_vel, float high, float high_vel, int16_t field_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, robotMajorState);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int32_t(buf, 12, altitude);
    _mav_put_uint32_t(buf, 16, satellite_count);
    _mav_put_float(buf, 20, voltage);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, dis);
    _mav_put_float(buf, 32, dis_vel);
    _mav_put_float(buf, 36, high);
    _mav_put_float(buf, 40, high_vel);
    _mav_put_uint16_t(buf, 44, mode_len);
    _mav_put_uint16_t(buf, 46, service);
    _mav_put_int16_t(buf, 48, field_flags);
    _mav_put_uint8_t(buf, 50, connected);
    _mav_put_uint8_t(buf, 51, armed);
    _mav_put_uint8_t(buf, 52, guided);
    _mav_put_uint8_t(buf, 53, manual_input);
    _mav_put_int8_t(buf, 84, status);
    _mav_put_char_array(buf, 54, mode, 30);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_STATE, buf, MAVLINK_MSG_ID_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_UAV_STATE_LEN, MAVLINK_MSG_ID_UAV_STATE_CRC);
#else
    mavlink_uav_state_t *packet = (mavlink_uav_state_t *)msgbuf;
    packet->robotMajorState = robotMajorState;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;
    packet->satellite_count = satellite_count;
    packet->voltage = voltage;
    packet->yaw = yaw;
    packet->dis = dis;
    packet->dis_vel = dis_vel;
    packet->high = high;
    packet->high_vel = high_vel;
    packet->mode_len = mode_len;
    packet->service = service;
    packet->field_flags = field_flags;
    packet->connected = connected;
    packet->armed = armed;
    packet->guided = guided;
    packet->manual_input = manual_input;
    packet->status = status;
    mav_array_memcpy(packet->mode, mode, sizeof(char)*30);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_STATE, (const char *)packet, MAVLINK_MSG_ID_UAV_STATE_MIN_LEN, MAVLINK_MSG_ID_UAV_STATE_LEN, MAVLINK_MSG_ID_UAV_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE UAV_STATE UNPACKING


/**
 * @brief Get field robotMajorState from uav_state message
 *
 * @return  机器人主要状态
 */
static inline int32_t mavlink_msg_uav_state_get_robotMajorState(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field connected from uav_state message
 *
 * @return  连接状态（0：未连接，1：已连接）
 */
static inline uint8_t mavlink_msg_uav_state_get_connected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field armed from uav_state message
 *
 * @return  解锁状态（0：未解锁，1：已解锁）
 */
static inline uint8_t mavlink_msg_uav_state_get_armed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Get field guided from uav_state message
 *
 * @return  引导模式（0：手动，1：引导）
 */
static inline uint8_t mavlink_msg_uav_state_get_guided(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field manual_input from uav_state message
 *
 * @return  手动输入状态（0：未启用，1：启用）
 */
static inline uint8_t mavlink_msg_uav_state_get_manual_input(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Get field mode from uav_state message
 *
 * @return  飞行模式名称
 */
static inline uint16_t mavlink_msg_uav_state_get_mode(const mavlink_message_t* msg, char *mode)
{
    return _MAV_RETURN_char_array(msg, mode, 30,  54);
}

/**
 * @brief Get field mode_len from uav_state message
 *
 * @return  飞行模式名称长度
 */
static inline uint16_t mavlink_msg_uav_state_get_mode_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  44);
}

/**
 * @brief Get field status from uav_state message
 *
 * @return  系统状态（-128 至 127）
 */
static inline int8_t mavlink_msg_uav_state_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  84);
}

/**
 * @brief Get field service from uav_state message
 *
 * @return  服务标识符
 */
static inline uint16_t mavlink_msg_uav_state_get_service(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  46);
}

/**
 * @brief Get field latitude from uav_state message
 *
 * @return [degE7] 纬度，单位为度*1E7
 */
static inline int32_t mavlink_msg_uav_state_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field longitude from uav_state message
 *
 * @return [degE7] 经度，单位为度*1E7
 */
static inline int32_t mavlink_msg_uav_state_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field altitude from uav_state message
 *
 * @return [degE5] 海拔高度，单位为毫米
 */
static inline int32_t mavlink_msg_uav_state_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field satellite_count from uav_state message
 *
 * @return  卫星数量
 */
static inline uint32_t mavlink_msg_uav_state_get_satellite_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field voltage from uav_state message
 *
 * @return  电压
 */
static inline float mavlink_msg_uav_state_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yaw from uav_state message
 *
 * @return  yaw角
 */
static inline float mavlink_msg_uav_state_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field dis from uav_state message
 *
 * @return  水平距离
 */
static inline float mavlink_msg_uav_state_get_dis(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field dis_vel from uav_state message
 *
 * @return  水平速度
 */
static inline float mavlink_msg_uav_state_get_dis_vel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field high from uav_state message
 *
 * @return  垂直高度
 */
static inline float mavlink_msg_uav_state_get_high(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field high_vel from uav_state message
 *
 * @return  垂直速度
 */
static inline float mavlink_msg_uav_state_get_high_vel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field field_flags from uav_state message
 *
 * @return  字段有效标志位
 */
static inline int16_t mavlink_msg_uav_state_get_field_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  48);
}

/**
 * @brief Decode a uav_state message into a struct
 *
 * @param msg The message to decode
 * @param uav_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_uav_state_decode(const mavlink_message_t* msg, mavlink_uav_state_t* uav_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uav_state->robotMajorState = mavlink_msg_uav_state_get_robotMajorState(msg);
    uav_state->latitude = mavlink_msg_uav_state_get_latitude(msg);
    uav_state->longitude = mavlink_msg_uav_state_get_longitude(msg);
    uav_state->altitude = mavlink_msg_uav_state_get_altitude(msg);
    uav_state->satellite_count = mavlink_msg_uav_state_get_satellite_count(msg);
    uav_state->voltage = mavlink_msg_uav_state_get_voltage(msg);
    uav_state->yaw = mavlink_msg_uav_state_get_yaw(msg);
    uav_state->dis = mavlink_msg_uav_state_get_dis(msg);
    uav_state->dis_vel = mavlink_msg_uav_state_get_dis_vel(msg);
    uav_state->high = mavlink_msg_uav_state_get_high(msg);
    uav_state->high_vel = mavlink_msg_uav_state_get_high_vel(msg);
    uav_state->mode_len = mavlink_msg_uav_state_get_mode_len(msg);
    uav_state->service = mavlink_msg_uav_state_get_service(msg);
    uav_state->field_flags = mavlink_msg_uav_state_get_field_flags(msg);
    uav_state->connected = mavlink_msg_uav_state_get_connected(msg);
    uav_state->armed = mavlink_msg_uav_state_get_armed(msg);
    uav_state->guided = mavlink_msg_uav_state_get_guided(msg);
    uav_state->manual_input = mavlink_msg_uav_state_get_manual_input(msg);
    mavlink_msg_uav_state_get_mode(msg, uav_state->mode);
    uav_state->status = mavlink_msg_uav_state_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAV_STATE_LEN? msg->len : MAVLINK_MSG_ID_UAV_STATE_LEN;
        memset(uav_state, 0, MAVLINK_MSG_ID_UAV_STATE_LEN);
    memcpy(uav_state, _MAV_PAYLOAD(msg), len);
#endif
}
