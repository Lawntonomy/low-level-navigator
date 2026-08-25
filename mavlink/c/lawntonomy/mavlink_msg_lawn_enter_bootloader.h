#pragma once
// MESSAGE LAWN_ENTER_BOOTLOADER PACKING

#define MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER 42004


typedef struct __mavlink_lawn_enter_bootloader_t {
 uint32_t magic; /*<  Must be 0xB00710AD; wider than and unrelated to LAWN_ARM_CMD.magic so neither can be mistaken for the other.*/
} mavlink_lawn_enter_bootloader_t;

#define MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN 4
#define MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_MIN_LEN 4
#define MAVLINK_MSG_ID_42004_LEN 4
#define MAVLINK_MSG_ID_42004_MIN_LEN 4

#define MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_CRC 68
#define MAVLINK_MSG_ID_42004_CRC 68



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LAWN_ENTER_BOOTLOADER { \
    42004, \
    "LAWN_ENTER_BOOTLOADER", \
    1, \
    {  { "magic", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_lawn_enter_bootloader_t, magic) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LAWN_ENTER_BOOTLOADER { \
    "LAWN_ENTER_BOOTLOADER", \
    1, \
    {  { "magic", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_lawn_enter_bootloader_t, magic) }, \
         } \
}
#endif

/**
 * @brief Pack a lawn_enter_bootloader message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param magic  Must be 0xB00710AD; wider than and unrelated to LAWN_ARM_CMD.magic so neither can be mistaken for the other.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_enter_bootloader_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t magic)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN];
    _mav_put_uint32_t(buf, 0, magic);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN);
#else
    mavlink_lawn_enter_bootloader_t packet;
    packet.magic = magic;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_MIN_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_CRC);
}

/**
 * @brief Pack a lawn_enter_bootloader message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param magic  Must be 0xB00710AD; wider than and unrelated to LAWN_ARM_CMD.magic so neither can be mistaken for the other.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_enter_bootloader_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t magic)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN];
    _mav_put_uint32_t(buf, 0, magic);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN);
#else
    mavlink_lawn_enter_bootloader_t packet;
    packet.magic = magic;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_MIN_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_MIN_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN);
#endif
}

/**
 * @brief Pack a lawn_enter_bootloader message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magic  Must be 0xB00710AD; wider than and unrelated to LAWN_ARM_CMD.magic so neither can be mistaken for the other.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_enter_bootloader_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t magic)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN];
    _mav_put_uint32_t(buf, 0, magic);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN);
#else
    mavlink_lawn_enter_bootloader_t packet;
    packet.magic = magic;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_MIN_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_CRC);
}

/**
 * @brief Encode a lawn_enter_bootloader struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lawn_enter_bootloader C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_enter_bootloader_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lawn_enter_bootloader_t* lawn_enter_bootloader)
{
    return mavlink_msg_lawn_enter_bootloader_pack(system_id, component_id, msg, lawn_enter_bootloader->magic);
}

/**
 * @brief Encode a lawn_enter_bootloader struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lawn_enter_bootloader C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_enter_bootloader_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lawn_enter_bootloader_t* lawn_enter_bootloader)
{
    return mavlink_msg_lawn_enter_bootloader_pack_chan(system_id, component_id, chan, msg, lawn_enter_bootloader->magic);
}

/**
 * @brief Encode a lawn_enter_bootloader struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param lawn_enter_bootloader C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_enter_bootloader_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_lawn_enter_bootloader_t* lawn_enter_bootloader)
{
    return mavlink_msg_lawn_enter_bootloader_pack_status(system_id, component_id, _status, msg,  lawn_enter_bootloader->magic);
}

/**
 * @brief Send a lawn_enter_bootloader message
 * @param chan MAVLink channel to send the message
 *
 * @param magic  Must be 0xB00710AD; wider than and unrelated to LAWN_ARM_CMD.magic so neither can be mistaken for the other.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lawn_enter_bootloader_send(mavlink_channel_t chan, uint32_t magic)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN];
    _mav_put_uint32_t(buf, 0, magic);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER, buf, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_MIN_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_CRC);
#else
    mavlink_lawn_enter_bootloader_t packet;
    packet.magic = magic;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER, (const char *)&packet, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_MIN_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_CRC);
#endif
}

/**
 * @brief Send a lawn_enter_bootloader message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lawn_enter_bootloader_send_struct(mavlink_channel_t chan, const mavlink_lawn_enter_bootloader_t* lawn_enter_bootloader)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lawn_enter_bootloader_send(chan, lawn_enter_bootloader->magic);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER, (const char *)lawn_enter_bootloader, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_MIN_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_CRC);
#endif
}

#if MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lawn_enter_bootloader_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t magic)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, magic);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER, buf, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_MIN_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_CRC);
#else
    mavlink_lawn_enter_bootloader_t *packet = (mavlink_lawn_enter_bootloader_t *)msgbuf;
    packet->magic = magic;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER, (const char *)packet, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_MIN_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_CRC);
#endif
}
#endif

#endif

// MESSAGE LAWN_ENTER_BOOTLOADER UNPACKING


/**
 * @brief Get field magic from lawn_enter_bootloader message
 *
 * @return  Must be 0xB00710AD; wider than and unrelated to LAWN_ARM_CMD.magic so neither can be mistaken for the other.
 */
static inline uint32_t mavlink_msg_lawn_enter_bootloader_get_magic(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a lawn_enter_bootloader message into a struct
 *
 * @param msg The message to decode
 * @param lawn_enter_bootloader C-struct to decode the message contents into
 */
static inline void mavlink_msg_lawn_enter_bootloader_decode(const mavlink_message_t* msg, mavlink_lawn_enter_bootloader_t* lawn_enter_bootloader)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lawn_enter_bootloader->magic = mavlink_msg_lawn_enter_bootloader_get_magic(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN? msg->len : MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN;
        memset(lawn_enter_bootloader, 0, MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_LEN);
    memcpy(lawn_enter_bootloader, _MAV_PAYLOAD(msg), len);
#endif
}
