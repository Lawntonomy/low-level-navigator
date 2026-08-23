#pragma once
// MESSAGE LAWN_WHEEL_STATE PACKING

#define MAVLINK_MSG_ID_LAWN_WHEEL_STATE 42011


typedef struct __mavlink_lawn_wheel_state_t {
 uint64_t t_meas_us; /*< [us] Time the speed was measured, not sent.*/
 int16_t left_drpm; /*< [drpm] Measured left, deci-rpm.*/
 int16_t right_drpm; /*< [drpm] Measured right, deci-rpm.*/
 int16_t left_cmd_drpm; /*< [drpm] Applied left target after limiting.*/
 int16_t right_cmd_drpm; /*< [drpm] Applied right target after limiting.*/
 uint8_t flags; /*<  */
} mavlink_lawn_wheel_state_t;

#define MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN 17
#define MAVLINK_MSG_ID_LAWN_WHEEL_STATE_MIN_LEN 17
#define MAVLINK_MSG_ID_42011_LEN 17
#define MAVLINK_MSG_ID_42011_MIN_LEN 17

#define MAVLINK_MSG_ID_LAWN_WHEEL_STATE_CRC 90
#define MAVLINK_MSG_ID_42011_CRC 90



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LAWN_WHEEL_STATE { \
    42011, \
    "LAWN_WHEEL_STATE", \
    6, \
    {  { "t_meas_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lawn_wheel_state_t, t_meas_us) }, \
         { "left_drpm", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_lawn_wheel_state_t, left_drpm) }, \
         { "right_drpm", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_lawn_wheel_state_t, right_drpm) }, \
         { "left_cmd_drpm", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_lawn_wheel_state_t, left_cmd_drpm) }, \
         { "right_cmd_drpm", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_lawn_wheel_state_t, right_cmd_drpm) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_lawn_wheel_state_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LAWN_WHEEL_STATE { \
    "LAWN_WHEEL_STATE", \
    6, \
    {  { "t_meas_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lawn_wheel_state_t, t_meas_us) }, \
         { "left_drpm", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_lawn_wheel_state_t, left_drpm) }, \
         { "right_drpm", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_lawn_wheel_state_t, right_drpm) }, \
         { "left_cmd_drpm", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_lawn_wheel_state_t, left_cmd_drpm) }, \
         { "right_cmd_drpm", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_lawn_wheel_state_t, right_cmd_drpm) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_lawn_wheel_state_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a lawn_wheel_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param t_meas_us [us] Time the speed was measured, not sent.
 * @param left_drpm [drpm] Measured left, deci-rpm.
 * @param right_drpm [drpm] Measured right, deci-rpm.
 * @param left_cmd_drpm [drpm] Applied left target after limiting.
 * @param right_cmd_drpm [drpm] Applied right target after limiting.
 * @param flags  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_wheel_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t t_meas_us, int16_t left_drpm, int16_t right_drpm, int16_t left_cmd_drpm, int16_t right_cmd_drpm, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_int16_t(buf, 8, left_drpm);
    _mav_put_int16_t(buf, 10, right_drpm);
    _mav_put_int16_t(buf, 12, left_cmd_drpm);
    _mav_put_int16_t(buf, 14, right_cmd_drpm);
    _mav_put_uint8_t(buf, 16, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN);
#else
    mavlink_lawn_wheel_state_t packet;
    packet.t_meas_us = t_meas_us;
    packet.left_drpm = left_drpm;
    packet.right_drpm = right_drpm;
    packet.left_cmd_drpm = left_cmd_drpm;
    packet.right_cmd_drpm = right_cmd_drpm;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_WHEEL_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_CRC);
}

/**
 * @brief Pack a lawn_wheel_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param t_meas_us [us] Time the speed was measured, not sent.
 * @param left_drpm [drpm] Measured left, deci-rpm.
 * @param right_drpm [drpm] Measured right, deci-rpm.
 * @param left_cmd_drpm [drpm] Applied left target after limiting.
 * @param right_cmd_drpm [drpm] Applied right target after limiting.
 * @param flags  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_wheel_state_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t t_meas_us, int16_t left_drpm, int16_t right_drpm, int16_t left_cmd_drpm, int16_t right_cmd_drpm, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_int16_t(buf, 8, left_drpm);
    _mav_put_int16_t(buf, 10, right_drpm);
    _mav_put_int16_t(buf, 12, left_cmd_drpm);
    _mav_put_int16_t(buf, 14, right_cmd_drpm);
    _mav_put_uint8_t(buf, 16, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN);
#else
    mavlink_lawn_wheel_state_t packet;
    packet.t_meas_us = t_meas_us;
    packet.left_drpm = left_drpm;
    packet.right_drpm = right_drpm;
    packet.left_cmd_drpm = left_cmd_drpm;
    packet.right_cmd_drpm = right_cmd_drpm;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_WHEEL_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN);
#endif
}

/**
 * @brief Pack a lawn_wheel_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param t_meas_us [us] Time the speed was measured, not sent.
 * @param left_drpm [drpm] Measured left, deci-rpm.
 * @param right_drpm [drpm] Measured right, deci-rpm.
 * @param left_cmd_drpm [drpm] Applied left target after limiting.
 * @param right_cmd_drpm [drpm] Applied right target after limiting.
 * @param flags  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_wheel_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t t_meas_us,int16_t left_drpm,int16_t right_drpm,int16_t left_cmd_drpm,int16_t right_cmd_drpm,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_int16_t(buf, 8, left_drpm);
    _mav_put_int16_t(buf, 10, right_drpm);
    _mav_put_int16_t(buf, 12, left_cmd_drpm);
    _mav_put_int16_t(buf, 14, right_cmd_drpm);
    _mav_put_uint8_t(buf, 16, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN);
#else
    mavlink_lawn_wheel_state_t packet;
    packet.t_meas_us = t_meas_us;
    packet.left_drpm = left_drpm;
    packet.right_drpm = right_drpm;
    packet.left_cmd_drpm = left_cmd_drpm;
    packet.right_cmd_drpm = right_cmd_drpm;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_WHEEL_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_CRC);
}

/**
 * @brief Encode a lawn_wheel_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lawn_wheel_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_wheel_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lawn_wheel_state_t* lawn_wheel_state)
{
    return mavlink_msg_lawn_wheel_state_pack(system_id, component_id, msg, lawn_wheel_state->t_meas_us, lawn_wheel_state->left_drpm, lawn_wheel_state->right_drpm, lawn_wheel_state->left_cmd_drpm, lawn_wheel_state->right_cmd_drpm, lawn_wheel_state->flags);
}

/**
 * @brief Encode a lawn_wheel_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lawn_wheel_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_wheel_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lawn_wheel_state_t* lawn_wheel_state)
{
    return mavlink_msg_lawn_wheel_state_pack_chan(system_id, component_id, chan, msg, lawn_wheel_state->t_meas_us, lawn_wheel_state->left_drpm, lawn_wheel_state->right_drpm, lawn_wheel_state->left_cmd_drpm, lawn_wheel_state->right_cmd_drpm, lawn_wheel_state->flags);
}

/**
 * @brief Encode a lawn_wheel_state struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param lawn_wheel_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_wheel_state_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_lawn_wheel_state_t* lawn_wheel_state)
{
    return mavlink_msg_lawn_wheel_state_pack_status(system_id, component_id, _status, msg,  lawn_wheel_state->t_meas_us, lawn_wheel_state->left_drpm, lawn_wheel_state->right_drpm, lawn_wheel_state->left_cmd_drpm, lawn_wheel_state->right_cmd_drpm, lawn_wheel_state->flags);
}

/**
 * @brief Send a lawn_wheel_state message
 * @param chan MAVLink channel to send the message
 *
 * @param t_meas_us [us] Time the speed was measured, not sent.
 * @param left_drpm [drpm] Measured left, deci-rpm.
 * @param right_drpm [drpm] Measured right, deci-rpm.
 * @param left_cmd_drpm [drpm] Applied left target after limiting.
 * @param right_cmd_drpm [drpm] Applied right target after limiting.
 * @param flags  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lawn_wheel_state_send(mavlink_channel_t chan, uint64_t t_meas_us, int16_t left_drpm, int16_t right_drpm, int16_t left_cmd_drpm, int16_t right_cmd_drpm, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_int16_t(buf, 8, left_drpm);
    _mav_put_int16_t(buf, 10, right_drpm);
    _mav_put_int16_t(buf, 12, left_cmd_drpm);
    _mav_put_int16_t(buf, 14, right_cmd_drpm);
    _mav_put_uint8_t(buf, 16, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_WHEEL_STATE, buf, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_CRC);
#else
    mavlink_lawn_wheel_state_t packet;
    packet.t_meas_us = t_meas_us;
    packet.left_drpm = left_drpm;
    packet.right_drpm = right_drpm;
    packet.left_cmd_drpm = left_cmd_drpm;
    packet.right_cmd_drpm = right_cmd_drpm;
    packet.flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_WHEEL_STATE, (const char *)&packet, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_CRC);
#endif
}

/**
 * @brief Send a lawn_wheel_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lawn_wheel_state_send_struct(mavlink_channel_t chan, const mavlink_lawn_wheel_state_t* lawn_wheel_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lawn_wheel_state_send(chan, lawn_wheel_state->t_meas_us, lawn_wheel_state->left_drpm, lawn_wheel_state->right_drpm, lawn_wheel_state->left_cmd_drpm, lawn_wheel_state->right_cmd_drpm, lawn_wheel_state->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_WHEEL_STATE, (const char *)lawn_wheel_state, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lawn_wheel_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t t_meas_us, int16_t left_drpm, int16_t right_drpm, int16_t left_cmd_drpm, int16_t right_cmd_drpm, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_int16_t(buf, 8, left_drpm);
    _mav_put_int16_t(buf, 10, right_drpm);
    _mav_put_int16_t(buf, 12, left_cmd_drpm);
    _mav_put_int16_t(buf, 14, right_cmd_drpm);
    _mav_put_uint8_t(buf, 16, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_WHEEL_STATE, buf, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_CRC);
#else
    mavlink_lawn_wheel_state_t *packet = (mavlink_lawn_wheel_state_t *)msgbuf;
    packet->t_meas_us = t_meas_us;
    packet->left_drpm = left_drpm;
    packet->right_drpm = right_drpm;
    packet->left_cmd_drpm = left_cmd_drpm;
    packet->right_cmd_drpm = right_cmd_drpm;
    packet->flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_WHEEL_STATE, (const char *)packet, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_MIN_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE LAWN_WHEEL_STATE UNPACKING


/**
 * @brief Get field t_meas_us from lawn_wheel_state message
 *
 * @return [us] Time the speed was measured, not sent.
 */
static inline uint64_t mavlink_msg_lawn_wheel_state_get_t_meas_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field left_drpm from lawn_wheel_state message
 *
 * @return [drpm] Measured left, deci-rpm.
 */
static inline int16_t mavlink_msg_lawn_wheel_state_get_left_drpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field right_drpm from lawn_wheel_state message
 *
 * @return [drpm] Measured right, deci-rpm.
 */
static inline int16_t mavlink_msg_lawn_wheel_state_get_right_drpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field left_cmd_drpm from lawn_wheel_state message
 *
 * @return [drpm] Applied left target after limiting.
 */
static inline int16_t mavlink_msg_lawn_wheel_state_get_left_cmd_drpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field right_cmd_drpm from lawn_wheel_state message
 *
 * @return [drpm] Applied right target after limiting.
 */
static inline int16_t mavlink_msg_lawn_wheel_state_get_right_cmd_drpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field flags from lawn_wheel_state message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_lawn_wheel_state_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Decode a lawn_wheel_state message into a struct
 *
 * @param msg The message to decode
 * @param lawn_wheel_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_lawn_wheel_state_decode(const mavlink_message_t* msg, mavlink_lawn_wheel_state_t* lawn_wheel_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lawn_wheel_state->t_meas_us = mavlink_msg_lawn_wheel_state_get_t_meas_us(msg);
    lawn_wheel_state->left_drpm = mavlink_msg_lawn_wheel_state_get_left_drpm(msg);
    lawn_wheel_state->right_drpm = mavlink_msg_lawn_wheel_state_get_right_drpm(msg);
    lawn_wheel_state->left_cmd_drpm = mavlink_msg_lawn_wheel_state_get_left_cmd_drpm(msg);
    lawn_wheel_state->right_cmd_drpm = mavlink_msg_lawn_wheel_state_get_right_cmd_drpm(msg);
    lawn_wheel_state->flags = mavlink_msg_lawn_wheel_state_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN? msg->len : MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN;
        memset(lawn_wheel_state, 0, MAVLINK_MSG_ID_LAWN_WHEEL_STATE_LEN);
    memcpy(lawn_wheel_state, _MAV_PAYLOAD(msg), len);
#endif
}
