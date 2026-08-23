#pragma once
// MESSAGE LAWN_STOP_REQ PACKING

#define MAVLINK_MSG_ID_LAWN_STOP_REQ 42003


typedef struct __mavlink_lawn_stop_req_t {
 uint8_t reason; /*<  Diagnostic only.*/
} mavlink_lawn_stop_req_t;

#define MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN 1
#define MAVLINK_MSG_ID_LAWN_STOP_REQ_MIN_LEN 1
#define MAVLINK_MSG_ID_42003_LEN 1
#define MAVLINK_MSG_ID_42003_MIN_LEN 1

#define MAVLINK_MSG_ID_LAWN_STOP_REQ_CRC 119
#define MAVLINK_MSG_ID_42003_CRC 119



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LAWN_STOP_REQ { \
    42003, \
    "LAWN_STOP_REQ", \
    1, \
    {  { "reason", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_lawn_stop_req_t, reason) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LAWN_STOP_REQ { \
    "LAWN_STOP_REQ", \
    1, \
    {  { "reason", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_lawn_stop_req_t, reason) }, \
         } \
}
#endif

/**
 * @brief Pack a lawn_stop_req message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param reason  Diagnostic only.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_stop_req_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN];
    _mav_put_uint8_t(buf, 0, reason);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN);
#else
    mavlink_lawn_stop_req_t packet;
    packet.reason = reason;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_STOP_REQ;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LAWN_STOP_REQ_MIN_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_CRC);
}

/**
 * @brief Pack a lawn_stop_req message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param reason  Diagnostic only.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_stop_req_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN];
    _mav_put_uint8_t(buf, 0, reason);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN);
#else
    mavlink_lawn_stop_req_t packet;
    packet.reason = reason;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_STOP_REQ;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_STOP_REQ_MIN_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_STOP_REQ_MIN_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN);
#endif
}

/**
 * @brief Pack a lawn_stop_req message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param reason  Diagnostic only.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_stop_req_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN];
    _mav_put_uint8_t(buf, 0, reason);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN);
#else
    mavlink_lawn_stop_req_t packet;
    packet.reason = reason;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_STOP_REQ;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LAWN_STOP_REQ_MIN_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_CRC);
}

/**
 * @brief Encode a lawn_stop_req struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lawn_stop_req C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_stop_req_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lawn_stop_req_t* lawn_stop_req)
{
    return mavlink_msg_lawn_stop_req_pack(system_id, component_id, msg, lawn_stop_req->reason);
}

/**
 * @brief Encode a lawn_stop_req struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lawn_stop_req C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_stop_req_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lawn_stop_req_t* lawn_stop_req)
{
    return mavlink_msg_lawn_stop_req_pack_chan(system_id, component_id, chan, msg, lawn_stop_req->reason);
}

/**
 * @brief Encode a lawn_stop_req struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param lawn_stop_req C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_stop_req_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_lawn_stop_req_t* lawn_stop_req)
{
    return mavlink_msg_lawn_stop_req_pack_status(system_id, component_id, _status, msg,  lawn_stop_req->reason);
}

/**
 * @brief Send a lawn_stop_req message
 * @param chan MAVLink channel to send the message
 *
 * @param reason  Diagnostic only.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lawn_stop_req_send(mavlink_channel_t chan, uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN];
    _mav_put_uint8_t(buf, 0, reason);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_STOP_REQ, buf, MAVLINK_MSG_ID_LAWN_STOP_REQ_MIN_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_CRC);
#else
    mavlink_lawn_stop_req_t packet;
    packet.reason = reason;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_STOP_REQ, (const char *)&packet, MAVLINK_MSG_ID_LAWN_STOP_REQ_MIN_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_CRC);
#endif
}

/**
 * @brief Send a lawn_stop_req message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lawn_stop_req_send_struct(mavlink_channel_t chan, const mavlink_lawn_stop_req_t* lawn_stop_req)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lawn_stop_req_send(chan, lawn_stop_req->reason);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_STOP_REQ, (const char *)lawn_stop_req, MAVLINK_MSG_ID_LAWN_STOP_REQ_MIN_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_CRC);
#endif
}

#if MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lawn_stop_req_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, reason);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_STOP_REQ, buf, MAVLINK_MSG_ID_LAWN_STOP_REQ_MIN_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_CRC);
#else
    mavlink_lawn_stop_req_t *packet = (mavlink_lawn_stop_req_t *)msgbuf;
    packet->reason = reason;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_STOP_REQ, (const char *)packet, MAVLINK_MSG_ID_LAWN_STOP_REQ_MIN_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN, MAVLINK_MSG_ID_LAWN_STOP_REQ_CRC);
#endif
}
#endif

#endif

// MESSAGE LAWN_STOP_REQ UNPACKING


/**
 * @brief Get field reason from lawn_stop_req message
 *
 * @return  Diagnostic only.
 */
static inline uint8_t mavlink_msg_lawn_stop_req_get_reason(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a lawn_stop_req message into a struct
 *
 * @param msg The message to decode
 * @param lawn_stop_req C-struct to decode the message contents into
 */
static inline void mavlink_msg_lawn_stop_req_decode(const mavlink_message_t* msg, mavlink_lawn_stop_req_t* lawn_stop_req)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lawn_stop_req->reason = mavlink_msg_lawn_stop_req_get_reason(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN? msg->len : MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN;
        memset(lawn_stop_req, 0, MAVLINK_MSG_ID_LAWN_STOP_REQ_LEN);
    memcpy(lawn_stop_req, _MAV_PAYLOAD(msg), len);
#endif
}
