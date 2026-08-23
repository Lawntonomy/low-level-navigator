#pragma once
// MESSAGE LAWN_ARM_CMD PACKING

#define MAVLINK_MSG_ID_LAWN_ARM_CMD 42002


typedef struct __mavlink_lawn_arm_cmd_t {
 uint16_t magic; /*<  Must be 0xA57E to arm. Any other value disarms.
      Guards against a corrupted or accidental frame enabling drive.*/
 uint8_t arm; /*<  1 = arm, 0 = disarm.*/
} mavlink_lawn_arm_cmd_t;

#define MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN 3
#define MAVLINK_MSG_ID_LAWN_ARM_CMD_MIN_LEN 3
#define MAVLINK_MSG_ID_42002_LEN 3
#define MAVLINK_MSG_ID_42002_MIN_LEN 3

#define MAVLINK_MSG_ID_LAWN_ARM_CMD_CRC 10
#define MAVLINK_MSG_ID_42002_CRC 10



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LAWN_ARM_CMD { \
    42002, \
    "LAWN_ARM_CMD", \
    2, \
    {  { "magic", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_lawn_arm_cmd_t, magic) }, \
         { "arm", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_lawn_arm_cmd_t, arm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LAWN_ARM_CMD { \
    "LAWN_ARM_CMD", \
    2, \
    {  { "magic", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_lawn_arm_cmd_t, magic) }, \
         { "arm", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_lawn_arm_cmd_t, arm) }, \
         } \
}
#endif

/**
 * @brief Pack a lawn_arm_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param magic  Must be 0xA57E to arm. Any other value disarms.
      Guards against a corrupted or accidental frame enabling drive.
 * @param arm  1 = arm, 0 = disarm.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_arm_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t magic, uint8_t arm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN];
    _mav_put_uint16_t(buf, 0, magic);
    _mav_put_uint8_t(buf, 2, arm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN);
#else
    mavlink_lawn_arm_cmd_t packet;
    packet.magic = magic;
    packet.arm = arm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_ARM_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LAWN_ARM_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_CRC);
}

/**
 * @brief Pack a lawn_arm_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param magic  Must be 0xA57E to arm. Any other value disarms.
      Guards against a corrupted or accidental frame enabling drive.
 * @param arm  1 = arm, 0 = disarm.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_arm_cmd_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t magic, uint8_t arm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN];
    _mav_put_uint16_t(buf, 0, magic);
    _mav_put_uint8_t(buf, 2, arm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN);
#else
    mavlink_lawn_arm_cmd_t packet;
    packet.magic = magic;
    packet.arm = arm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_ARM_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_ARM_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_ARM_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN);
#endif
}

/**
 * @brief Pack a lawn_arm_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magic  Must be 0xA57E to arm. Any other value disarms.
      Guards against a corrupted or accidental frame enabling drive.
 * @param arm  1 = arm, 0 = disarm.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_arm_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t magic,uint8_t arm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN];
    _mav_put_uint16_t(buf, 0, magic);
    _mav_put_uint8_t(buf, 2, arm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN);
#else
    mavlink_lawn_arm_cmd_t packet;
    packet.magic = magic;
    packet.arm = arm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_ARM_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LAWN_ARM_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_CRC);
}

/**
 * @brief Encode a lawn_arm_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lawn_arm_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_arm_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lawn_arm_cmd_t* lawn_arm_cmd)
{
    return mavlink_msg_lawn_arm_cmd_pack(system_id, component_id, msg, lawn_arm_cmd->magic, lawn_arm_cmd->arm);
}

/**
 * @brief Encode a lawn_arm_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lawn_arm_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_arm_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lawn_arm_cmd_t* lawn_arm_cmd)
{
    return mavlink_msg_lawn_arm_cmd_pack_chan(system_id, component_id, chan, msg, lawn_arm_cmd->magic, lawn_arm_cmd->arm);
}

/**
 * @brief Encode a lawn_arm_cmd struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param lawn_arm_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_arm_cmd_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_lawn_arm_cmd_t* lawn_arm_cmd)
{
    return mavlink_msg_lawn_arm_cmd_pack_status(system_id, component_id, _status, msg,  lawn_arm_cmd->magic, lawn_arm_cmd->arm);
}

/**
 * @brief Send a lawn_arm_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param magic  Must be 0xA57E to arm. Any other value disarms.
      Guards against a corrupted or accidental frame enabling drive.
 * @param arm  1 = arm, 0 = disarm.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lawn_arm_cmd_send(mavlink_channel_t chan, uint16_t magic, uint8_t arm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN];
    _mav_put_uint16_t(buf, 0, magic);
    _mav_put_uint8_t(buf, 2, arm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_ARM_CMD, buf, MAVLINK_MSG_ID_LAWN_ARM_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_CRC);
#else
    mavlink_lawn_arm_cmd_t packet;
    packet.magic = magic;
    packet.arm = arm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_ARM_CMD, (const char *)&packet, MAVLINK_MSG_ID_LAWN_ARM_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_CRC);
#endif
}

/**
 * @brief Send a lawn_arm_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lawn_arm_cmd_send_struct(mavlink_channel_t chan, const mavlink_lawn_arm_cmd_t* lawn_arm_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lawn_arm_cmd_send(chan, lawn_arm_cmd->magic, lawn_arm_cmd->arm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_ARM_CMD, (const char *)lawn_arm_cmd, MAVLINK_MSG_ID_LAWN_ARM_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lawn_arm_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t magic, uint8_t arm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, magic);
    _mav_put_uint8_t(buf, 2, arm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_ARM_CMD, buf, MAVLINK_MSG_ID_LAWN_ARM_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_CRC);
#else
    mavlink_lawn_arm_cmd_t *packet = (mavlink_lawn_arm_cmd_t *)msgbuf;
    packet->magic = magic;
    packet->arm = arm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_ARM_CMD, (const char *)packet, MAVLINK_MSG_ID_LAWN_ARM_CMD_MIN_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN, MAVLINK_MSG_ID_LAWN_ARM_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE LAWN_ARM_CMD UNPACKING


/**
 * @brief Get field magic from lawn_arm_cmd message
 *
 * @return  Must be 0xA57E to arm. Any other value disarms.
      Guards against a corrupted or accidental frame enabling drive.
 */
static inline uint16_t mavlink_msg_lawn_arm_cmd_get_magic(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field arm from lawn_arm_cmd message
 *
 * @return  1 = arm, 0 = disarm.
 */
static inline uint8_t mavlink_msg_lawn_arm_cmd_get_arm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a lawn_arm_cmd message into a struct
 *
 * @param msg The message to decode
 * @param lawn_arm_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_lawn_arm_cmd_decode(const mavlink_message_t* msg, mavlink_lawn_arm_cmd_t* lawn_arm_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lawn_arm_cmd->magic = mavlink_msg_lawn_arm_cmd_get_magic(msg);
    lawn_arm_cmd->arm = mavlink_msg_lawn_arm_cmd_get_arm(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN? msg->len : MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN;
        memset(lawn_arm_cmd, 0, MAVLINK_MSG_ID_LAWN_ARM_CMD_LEN);
    memcpy(lawn_arm_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
