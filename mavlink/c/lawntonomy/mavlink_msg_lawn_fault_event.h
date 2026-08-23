#pragma once
// MESSAGE LAWN_FAULT_EVENT PACKING

#define MAVLINK_MSG_ID_LAWN_FAULT_EVENT 42013


typedef struct __mavlink_lawn_fault_event_t {
 uint64_t t_meas_us; /*< [us] */
 uint16_t code; /*<  */
 uint8_t nav_state; /*<  */
 uint8_t latched; /*<  1 = requires explicit clear.*/
} mavlink_lawn_fault_event_t;

#define MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN 12
#define MAVLINK_MSG_ID_LAWN_FAULT_EVENT_MIN_LEN 12
#define MAVLINK_MSG_ID_42013_LEN 12
#define MAVLINK_MSG_ID_42013_MIN_LEN 12

#define MAVLINK_MSG_ID_LAWN_FAULT_EVENT_CRC 223
#define MAVLINK_MSG_ID_42013_CRC 223



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LAWN_FAULT_EVENT { \
    42013, \
    "LAWN_FAULT_EVENT", \
    4, \
    {  { "t_meas_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lawn_fault_event_t, t_meas_us) }, \
         { "code", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_lawn_fault_event_t, code) }, \
         { "nav_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_lawn_fault_event_t, nav_state) }, \
         { "latched", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_lawn_fault_event_t, latched) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LAWN_FAULT_EVENT { \
    "LAWN_FAULT_EVENT", \
    4, \
    {  { "t_meas_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lawn_fault_event_t, t_meas_us) }, \
         { "code", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_lawn_fault_event_t, code) }, \
         { "nav_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_lawn_fault_event_t, nav_state) }, \
         { "latched", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_lawn_fault_event_t, latched) }, \
         } \
}
#endif

/**
 * @brief Pack a lawn_fault_event message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param t_meas_us [us] 
 * @param code  
 * @param nav_state  
 * @param latched  1 = requires explicit clear.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_fault_event_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t t_meas_us, uint16_t code, uint8_t nav_state, uint8_t latched)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint16_t(buf, 8, code);
    _mav_put_uint8_t(buf, 10, nav_state);
    _mav_put_uint8_t(buf, 11, latched);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN);
#else
    mavlink_lawn_fault_event_t packet;
    packet.t_meas_us = t_meas_us;
    packet.code = code;
    packet.nav_state = nav_state;
    packet.latched = latched;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_FAULT_EVENT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_MIN_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_CRC);
}

/**
 * @brief Pack a lawn_fault_event message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param t_meas_us [us] 
 * @param code  
 * @param nav_state  
 * @param latched  1 = requires explicit clear.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_fault_event_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t t_meas_us, uint16_t code, uint8_t nav_state, uint8_t latched)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint16_t(buf, 8, code);
    _mav_put_uint8_t(buf, 10, nav_state);
    _mav_put_uint8_t(buf, 11, latched);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN);
#else
    mavlink_lawn_fault_event_t packet;
    packet.t_meas_us = t_meas_us;
    packet.code = code;
    packet.nav_state = nav_state;
    packet.latched = latched;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_FAULT_EVENT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_MIN_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_MIN_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN);
#endif
}

/**
 * @brief Pack a lawn_fault_event message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param t_meas_us [us] 
 * @param code  
 * @param nav_state  
 * @param latched  1 = requires explicit clear.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_fault_event_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t t_meas_us,uint16_t code,uint8_t nav_state,uint8_t latched)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint16_t(buf, 8, code);
    _mav_put_uint8_t(buf, 10, nav_state);
    _mav_put_uint8_t(buf, 11, latched);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN);
#else
    mavlink_lawn_fault_event_t packet;
    packet.t_meas_us = t_meas_us;
    packet.code = code;
    packet.nav_state = nav_state;
    packet.latched = latched;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_FAULT_EVENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_MIN_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_CRC);
}

/**
 * @brief Encode a lawn_fault_event struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lawn_fault_event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_fault_event_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lawn_fault_event_t* lawn_fault_event)
{
    return mavlink_msg_lawn_fault_event_pack(system_id, component_id, msg, lawn_fault_event->t_meas_us, lawn_fault_event->code, lawn_fault_event->nav_state, lawn_fault_event->latched);
}

/**
 * @brief Encode a lawn_fault_event struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lawn_fault_event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_fault_event_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lawn_fault_event_t* lawn_fault_event)
{
    return mavlink_msg_lawn_fault_event_pack_chan(system_id, component_id, chan, msg, lawn_fault_event->t_meas_us, lawn_fault_event->code, lawn_fault_event->nav_state, lawn_fault_event->latched);
}

/**
 * @brief Encode a lawn_fault_event struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param lawn_fault_event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_fault_event_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_lawn_fault_event_t* lawn_fault_event)
{
    return mavlink_msg_lawn_fault_event_pack_status(system_id, component_id, _status, msg,  lawn_fault_event->t_meas_us, lawn_fault_event->code, lawn_fault_event->nav_state, lawn_fault_event->latched);
}

/**
 * @brief Send a lawn_fault_event message
 * @param chan MAVLink channel to send the message
 *
 * @param t_meas_us [us] 
 * @param code  
 * @param nav_state  
 * @param latched  1 = requires explicit clear.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lawn_fault_event_send(mavlink_channel_t chan, uint64_t t_meas_us, uint16_t code, uint8_t nav_state, uint8_t latched)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint16_t(buf, 8, code);
    _mav_put_uint8_t(buf, 10, nav_state);
    _mav_put_uint8_t(buf, 11, latched);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_FAULT_EVENT, buf, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_MIN_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_CRC);
#else
    mavlink_lawn_fault_event_t packet;
    packet.t_meas_us = t_meas_us;
    packet.code = code;
    packet.nav_state = nav_state;
    packet.latched = latched;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_FAULT_EVENT, (const char *)&packet, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_MIN_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_CRC);
#endif
}

/**
 * @brief Send a lawn_fault_event message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lawn_fault_event_send_struct(mavlink_channel_t chan, const mavlink_lawn_fault_event_t* lawn_fault_event)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lawn_fault_event_send(chan, lawn_fault_event->t_meas_us, lawn_fault_event->code, lawn_fault_event->nav_state, lawn_fault_event->latched);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_FAULT_EVENT, (const char *)lawn_fault_event, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_MIN_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_CRC);
#endif
}

#if MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lawn_fault_event_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t t_meas_us, uint16_t code, uint8_t nav_state, uint8_t latched)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint16_t(buf, 8, code);
    _mav_put_uint8_t(buf, 10, nav_state);
    _mav_put_uint8_t(buf, 11, latched);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_FAULT_EVENT, buf, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_MIN_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_CRC);
#else
    mavlink_lawn_fault_event_t *packet = (mavlink_lawn_fault_event_t *)msgbuf;
    packet->t_meas_us = t_meas_us;
    packet->code = code;
    packet->nav_state = nav_state;
    packet->latched = latched;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_FAULT_EVENT, (const char *)packet, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_MIN_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_CRC);
#endif
}
#endif

#endif

// MESSAGE LAWN_FAULT_EVENT UNPACKING


/**
 * @brief Get field t_meas_us from lawn_fault_event message
 *
 * @return [us] 
 */
static inline uint64_t mavlink_msg_lawn_fault_event_get_t_meas_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field code from lawn_fault_event message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_lawn_fault_event_get_code(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field nav_state from lawn_fault_event message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_lawn_fault_event_get_nav_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field latched from lawn_fault_event message
 *
 * @return  1 = requires explicit clear.
 */
static inline uint8_t mavlink_msg_lawn_fault_event_get_latched(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Decode a lawn_fault_event message into a struct
 *
 * @param msg The message to decode
 * @param lawn_fault_event C-struct to decode the message contents into
 */
static inline void mavlink_msg_lawn_fault_event_decode(const mavlink_message_t* msg, mavlink_lawn_fault_event_t* lawn_fault_event)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lawn_fault_event->t_meas_us = mavlink_msg_lawn_fault_event_get_t_meas_us(msg);
    lawn_fault_event->code = mavlink_msg_lawn_fault_event_get_code(msg);
    lawn_fault_event->nav_state = mavlink_msg_lawn_fault_event_get_nav_state(msg);
    lawn_fault_event->latched = mavlink_msg_lawn_fault_event_get_latched(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN? msg->len : MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN;
        memset(lawn_fault_event, 0, MAVLINK_MSG_ID_LAWN_FAULT_EVENT_LEN);
    memcpy(lawn_fault_event, _MAV_PAYLOAD(msg), len);
#endif
}
