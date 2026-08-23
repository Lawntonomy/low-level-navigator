#pragma once
// MESSAGE LAWN_NAV_STATUS PACKING

#define MAVLINK_MSG_ID_LAWN_NAV_STATUS 42010


typedef struct __mavlink_lawn_nav_status_t {
 uint64_t t_meas_us; /*< [us] Low-level monotonic clock. ADR-0007.*/
 uint16_t cmd_age_ms; /*< [ms] Age of the last accepted drive
      request, on the low-level clock. 65535 = none ever accepted.*/
 uint8_t nav_state; /*<  */
 uint8_t armed; /*<  1 = drive enable asserted.*/
 uint8_t fault; /*<  Currently latched fault.*/
} mavlink_lawn_nav_status_t;

#define MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN 13
#define MAVLINK_MSG_ID_LAWN_NAV_STATUS_MIN_LEN 13
#define MAVLINK_MSG_ID_42010_LEN 13
#define MAVLINK_MSG_ID_42010_MIN_LEN 13

#define MAVLINK_MSG_ID_LAWN_NAV_STATUS_CRC 131
#define MAVLINK_MSG_ID_42010_CRC 131



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LAWN_NAV_STATUS { \
    42010, \
    "LAWN_NAV_STATUS", \
    5, \
    {  { "t_meas_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lawn_nav_status_t, t_meas_us) }, \
         { "cmd_age_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_lawn_nav_status_t, cmd_age_ms) }, \
         { "nav_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_lawn_nav_status_t, nav_state) }, \
         { "armed", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_lawn_nav_status_t, armed) }, \
         { "fault", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_lawn_nav_status_t, fault) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LAWN_NAV_STATUS { \
    "LAWN_NAV_STATUS", \
    5, \
    {  { "t_meas_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lawn_nav_status_t, t_meas_us) }, \
         { "cmd_age_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_lawn_nav_status_t, cmd_age_ms) }, \
         { "nav_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_lawn_nav_status_t, nav_state) }, \
         { "armed", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_lawn_nav_status_t, armed) }, \
         { "fault", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_lawn_nav_status_t, fault) }, \
         } \
}
#endif

/**
 * @brief Pack a lawn_nav_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param t_meas_us [us] Low-level monotonic clock. ADR-0007.
 * @param cmd_age_ms [ms] Age of the last accepted drive
      request, on the low-level clock. 65535 = none ever accepted.
 * @param nav_state  
 * @param armed  1 = drive enable asserted.
 * @param fault  Currently latched fault.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_nav_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t t_meas_us, uint16_t cmd_age_ms, uint8_t nav_state, uint8_t armed, uint8_t fault)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint16_t(buf, 8, cmd_age_ms);
    _mav_put_uint8_t(buf, 10, nav_state);
    _mav_put_uint8_t(buf, 11, armed);
    _mav_put_uint8_t(buf, 12, fault);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN);
#else
    mavlink_lawn_nav_status_t packet;
    packet.t_meas_us = t_meas_us;
    packet.cmd_age_ms = cmd_age_ms;
    packet.nav_state = nav_state;
    packet.armed = armed;
    packet.fault = fault;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_NAV_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LAWN_NAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_CRC);
}

/**
 * @brief Pack a lawn_nav_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param t_meas_us [us] Low-level monotonic clock. ADR-0007.
 * @param cmd_age_ms [ms] Age of the last accepted drive
      request, on the low-level clock. 65535 = none ever accepted.
 * @param nav_state  
 * @param armed  1 = drive enable asserted.
 * @param fault  Currently latched fault.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_nav_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t t_meas_us, uint16_t cmd_age_ms, uint8_t nav_state, uint8_t armed, uint8_t fault)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint16_t(buf, 8, cmd_age_ms);
    _mav_put_uint8_t(buf, 10, nav_state);
    _mav_put_uint8_t(buf, 11, armed);
    _mav_put_uint8_t(buf, 12, fault);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN);
#else
    mavlink_lawn_nav_status_t packet;
    packet.t_meas_us = t_meas_us;
    packet.cmd_age_ms = cmd_age_ms;
    packet.nav_state = nav_state;
    packet.armed = armed;
    packet.fault = fault;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_NAV_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_NAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_NAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN);
#endif
}

/**
 * @brief Pack a lawn_nav_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param t_meas_us [us] Low-level monotonic clock. ADR-0007.
 * @param cmd_age_ms [ms] Age of the last accepted drive
      request, on the low-level clock. 65535 = none ever accepted.
 * @param nav_state  
 * @param armed  1 = drive enable asserted.
 * @param fault  Currently latched fault.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_nav_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t t_meas_us,uint16_t cmd_age_ms,uint8_t nav_state,uint8_t armed,uint8_t fault)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint16_t(buf, 8, cmd_age_ms);
    _mav_put_uint8_t(buf, 10, nav_state);
    _mav_put_uint8_t(buf, 11, armed);
    _mav_put_uint8_t(buf, 12, fault);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN);
#else
    mavlink_lawn_nav_status_t packet;
    packet.t_meas_us = t_meas_us;
    packet.cmd_age_ms = cmd_age_ms;
    packet.nav_state = nav_state;
    packet.armed = armed;
    packet.fault = fault;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_NAV_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LAWN_NAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_CRC);
}

/**
 * @brief Encode a lawn_nav_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lawn_nav_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_nav_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lawn_nav_status_t* lawn_nav_status)
{
    return mavlink_msg_lawn_nav_status_pack(system_id, component_id, msg, lawn_nav_status->t_meas_us, lawn_nav_status->cmd_age_ms, lawn_nav_status->nav_state, lawn_nav_status->armed, lawn_nav_status->fault);
}

/**
 * @brief Encode a lawn_nav_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lawn_nav_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_nav_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lawn_nav_status_t* lawn_nav_status)
{
    return mavlink_msg_lawn_nav_status_pack_chan(system_id, component_id, chan, msg, lawn_nav_status->t_meas_us, lawn_nav_status->cmd_age_ms, lawn_nav_status->nav_state, lawn_nav_status->armed, lawn_nav_status->fault);
}

/**
 * @brief Encode a lawn_nav_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param lawn_nav_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_nav_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_lawn_nav_status_t* lawn_nav_status)
{
    return mavlink_msg_lawn_nav_status_pack_status(system_id, component_id, _status, msg,  lawn_nav_status->t_meas_us, lawn_nav_status->cmd_age_ms, lawn_nav_status->nav_state, lawn_nav_status->armed, lawn_nav_status->fault);
}

/**
 * @brief Send a lawn_nav_status message
 * @param chan MAVLink channel to send the message
 *
 * @param t_meas_us [us] Low-level monotonic clock. ADR-0007.
 * @param cmd_age_ms [ms] Age of the last accepted drive
      request, on the low-level clock. 65535 = none ever accepted.
 * @param nav_state  
 * @param armed  1 = drive enable asserted.
 * @param fault  Currently latched fault.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lawn_nav_status_send(mavlink_channel_t chan, uint64_t t_meas_us, uint16_t cmd_age_ms, uint8_t nav_state, uint8_t armed, uint8_t fault)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint16_t(buf, 8, cmd_age_ms);
    _mav_put_uint8_t(buf, 10, nav_state);
    _mav_put_uint8_t(buf, 11, armed);
    _mav_put_uint8_t(buf, 12, fault);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_NAV_STATUS, buf, MAVLINK_MSG_ID_LAWN_NAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_CRC);
#else
    mavlink_lawn_nav_status_t packet;
    packet.t_meas_us = t_meas_us;
    packet.cmd_age_ms = cmd_age_ms;
    packet.nav_state = nav_state;
    packet.armed = armed;
    packet.fault = fault;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_NAV_STATUS, (const char *)&packet, MAVLINK_MSG_ID_LAWN_NAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_CRC);
#endif
}

/**
 * @brief Send a lawn_nav_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lawn_nav_status_send_struct(mavlink_channel_t chan, const mavlink_lawn_nav_status_t* lawn_nav_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lawn_nav_status_send(chan, lawn_nav_status->t_meas_us, lawn_nav_status->cmd_age_ms, lawn_nav_status->nav_state, lawn_nav_status->armed, lawn_nav_status->fault);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_NAV_STATUS, (const char *)lawn_nav_status, MAVLINK_MSG_ID_LAWN_NAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lawn_nav_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t t_meas_us, uint16_t cmd_age_ms, uint8_t nav_state, uint8_t armed, uint8_t fault)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint16_t(buf, 8, cmd_age_ms);
    _mav_put_uint8_t(buf, 10, nav_state);
    _mav_put_uint8_t(buf, 11, armed);
    _mav_put_uint8_t(buf, 12, fault);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_NAV_STATUS, buf, MAVLINK_MSG_ID_LAWN_NAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_CRC);
#else
    mavlink_lawn_nav_status_t *packet = (mavlink_lawn_nav_status_t *)msgbuf;
    packet->t_meas_us = t_meas_us;
    packet->cmd_age_ms = cmd_age_ms;
    packet->nav_state = nav_state;
    packet->armed = armed;
    packet->fault = fault;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_NAV_STATUS, (const char *)packet, MAVLINK_MSG_ID_LAWN_NAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN, MAVLINK_MSG_ID_LAWN_NAV_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE LAWN_NAV_STATUS UNPACKING


/**
 * @brief Get field t_meas_us from lawn_nav_status message
 *
 * @return [us] Low-level monotonic clock. ADR-0007.
 */
static inline uint64_t mavlink_msg_lawn_nav_status_get_t_meas_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field cmd_age_ms from lawn_nav_status message
 *
 * @return [ms] Age of the last accepted drive
      request, on the low-level clock. 65535 = none ever accepted.
 */
static inline uint16_t mavlink_msg_lawn_nav_status_get_cmd_age_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field nav_state from lawn_nav_status message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_lawn_nav_status_get_nav_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field armed from lawn_nav_status message
 *
 * @return  1 = drive enable asserted.
 */
static inline uint8_t mavlink_msg_lawn_nav_status_get_armed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field fault from lawn_nav_status message
 *
 * @return  Currently latched fault.
 */
static inline uint8_t mavlink_msg_lawn_nav_status_get_fault(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Decode a lawn_nav_status message into a struct
 *
 * @param msg The message to decode
 * @param lawn_nav_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_lawn_nav_status_decode(const mavlink_message_t* msg, mavlink_lawn_nav_status_t* lawn_nav_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lawn_nav_status->t_meas_us = mavlink_msg_lawn_nav_status_get_t_meas_us(msg);
    lawn_nav_status->cmd_age_ms = mavlink_msg_lawn_nav_status_get_cmd_age_ms(msg);
    lawn_nav_status->nav_state = mavlink_msg_lawn_nav_status_get_nav_state(msg);
    lawn_nav_status->armed = mavlink_msg_lawn_nav_status_get_armed(msg);
    lawn_nav_status->fault = mavlink_msg_lawn_nav_status_get_fault(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN? msg->len : MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN;
        memset(lawn_nav_status, 0, MAVLINK_MSG_ID_LAWN_NAV_STATUS_LEN);
    memcpy(lawn_nav_status, _MAV_PAYLOAD(msg), len);
#endif
}
