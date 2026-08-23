#pragma once
// MESSAGE LAWN_DRIVE_CMD PACKING

#define MAVLINK_MSG_ID_LAWN_DRIVE_CMD 42001


typedef struct __mavlink_lawn_drive_cmd_t {
 int16_t left_drpm; /*< [drpm] Left wheel target, deci-rpm. Signed.*/
 int16_t right_drpm; /*< [drpm] Right wheel target, deci-rpm. Signed.*/
} mavlink_lawn_drive_cmd_t;

#define MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN 4
#define MAVLINK_MSG_ID_LAWN_DRIVE_CMD_MIN_LEN 4
#define MAVLINK_MSG_ID_42001_LEN 4
#define MAVLINK_MSG_ID_42001_MIN_LEN 4

#define MAVLINK_MSG_ID_LAWN_DRIVE_CMD_CRC 158
#define MAVLINK_MSG_ID_42001_CRC 158



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LAWN_DRIVE_CMD { \
    42001, \
    "LAWN_DRIVE_CMD", \
    2, \
    {  { "left_drpm", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_lawn_drive_cmd_t, left_drpm) }, \
         { "right_drpm", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_lawn_drive_cmd_t, right_drpm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LAWN_DRIVE_CMD { \
    "LAWN_DRIVE_CMD", \
    2, \
    {  { "left_drpm", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_lawn_drive_cmd_t, left_drpm) }, \
         { "right_drpm", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_lawn_drive_cmd_t, right_drpm) }, \
         } \
}
#endif

/**
 * @brief Pack a lawn_drive_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param left_drpm [drpm] Left wheel target, deci-rpm. Signed.
 * @param right_drpm [drpm] Right wheel target, deci-rpm. Signed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_drive_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t left_drpm, int16_t right_drpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN];
    _mav_put_int16_t(buf, 0, left_drpm);
    _mav_put_int16_t(buf, 2, right_drpm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN);
#else
    mavlink_lawn_drive_cmd_t packet;
    packet.left_drpm = left_drpm;
    packet.right_drpm = right_drpm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_DRIVE_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_CRC);
}

/**
 * @brief Pack a lawn_drive_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param left_drpm [drpm] Left wheel target, deci-rpm. Signed.
 * @param right_drpm [drpm] Right wheel target, deci-rpm. Signed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_drive_cmd_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int16_t left_drpm, int16_t right_drpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN];
    _mav_put_int16_t(buf, 0, left_drpm);
    _mav_put_int16_t(buf, 2, right_drpm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN);
#else
    mavlink_lawn_drive_cmd_t packet;
    packet.left_drpm = left_drpm;
    packet.right_drpm = right_drpm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_DRIVE_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN);
#endif
}

/**
 * @brief Pack a lawn_drive_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param left_drpm [drpm] Left wheel target, deci-rpm. Signed.
 * @param right_drpm [drpm] Right wheel target, deci-rpm. Signed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_drive_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t left_drpm,int16_t right_drpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN];
    _mav_put_int16_t(buf, 0, left_drpm);
    _mav_put_int16_t(buf, 2, right_drpm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN);
#else
    mavlink_lawn_drive_cmd_t packet;
    packet.left_drpm = left_drpm;
    packet.right_drpm = right_drpm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_DRIVE_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_CRC);
}

/**
 * @brief Encode a lawn_drive_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lawn_drive_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_drive_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lawn_drive_cmd_t* lawn_drive_cmd)
{
    return mavlink_msg_lawn_drive_cmd_pack(system_id, component_id, msg, lawn_drive_cmd->left_drpm, lawn_drive_cmd->right_drpm);
}

/**
 * @brief Encode a lawn_drive_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lawn_drive_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_drive_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lawn_drive_cmd_t* lawn_drive_cmd)
{
    return mavlink_msg_lawn_drive_cmd_pack_chan(system_id, component_id, chan, msg, lawn_drive_cmd->left_drpm, lawn_drive_cmd->right_drpm);
}

/**
 * @brief Encode a lawn_drive_cmd struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param lawn_drive_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_drive_cmd_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_lawn_drive_cmd_t* lawn_drive_cmd)
{
    return mavlink_msg_lawn_drive_cmd_pack_status(system_id, component_id, _status, msg,  lawn_drive_cmd->left_drpm, lawn_drive_cmd->right_drpm);
}

/**
 * @brief Send a lawn_drive_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param left_drpm [drpm] Left wheel target, deci-rpm. Signed.
 * @param right_drpm [drpm] Right wheel target, deci-rpm. Signed.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lawn_drive_cmd_send(mavlink_channel_t chan, int16_t left_drpm, int16_t right_drpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN];
    _mav_put_int16_t(buf, 0, left_drpm);
    _mav_put_int16_t(buf, 2, right_drpm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_DRIVE_CMD, buf, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_CRC);
#else
    mavlink_lawn_drive_cmd_t packet;
    packet.left_drpm = left_drpm;
    packet.right_drpm = right_drpm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_DRIVE_CMD, (const char *)&packet, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_CRC);
#endif
}

/**
 * @brief Send a lawn_drive_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lawn_drive_cmd_send_struct(mavlink_channel_t chan, const mavlink_lawn_drive_cmd_t* lawn_drive_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lawn_drive_cmd_send(chan, lawn_drive_cmd->left_drpm, lawn_drive_cmd->right_drpm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_DRIVE_CMD, (const char *)lawn_drive_cmd, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lawn_drive_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t left_drpm, int16_t right_drpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, left_drpm);
    _mav_put_int16_t(buf, 2, right_drpm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_DRIVE_CMD, buf, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_CRC);
#else
    mavlink_lawn_drive_cmd_t *packet = (mavlink_lawn_drive_cmd_t *)msgbuf;
    packet->left_drpm = left_drpm;
    packet->right_drpm = right_drpm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_DRIVE_CMD, (const char *)packet, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE LAWN_DRIVE_CMD UNPACKING


/**
 * @brief Get field left_drpm from lawn_drive_cmd message
 *
 * @return [drpm] Left wheel target, deci-rpm. Signed.
 */
static inline int16_t mavlink_msg_lawn_drive_cmd_get_left_drpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field right_drpm from lawn_drive_cmd message
 *
 * @return [drpm] Right wheel target, deci-rpm. Signed.
 */
static inline int16_t mavlink_msg_lawn_drive_cmd_get_right_drpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Decode a lawn_drive_cmd message into a struct
 *
 * @param msg The message to decode
 * @param lawn_drive_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_lawn_drive_cmd_decode(const mavlink_message_t* msg, mavlink_lawn_drive_cmd_t* lawn_drive_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lawn_drive_cmd->left_drpm = mavlink_msg_lawn_drive_cmd_get_left_drpm(msg);
    lawn_drive_cmd->right_drpm = mavlink_msg_lawn_drive_cmd_get_right_drpm(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN? msg->len : MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN;
        memset(lawn_drive_cmd, 0, MAVLINK_MSG_ID_LAWN_DRIVE_CMD_LEN);
    memcpy(lawn_drive_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
