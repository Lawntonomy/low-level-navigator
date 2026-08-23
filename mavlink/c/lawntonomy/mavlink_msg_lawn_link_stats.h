#pragma once
// MESSAGE LAWN_LINK_STATS PACKING

#define MAVLINK_MSG_ID_LAWN_LINK_STATS 42012


typedef struct __mavlink_lawn_link_stats_t {
 uint64_t t_meas_us; /*< [us] */
 uint32_t frames_accepted; /*<  Accepted this window.*/
 uint32_t frames_rejected; /*<  CRC, framing, or identity rejects this window.*/
 uint32_t dropped_tx; /*<  Frames the low-level tier discarded because a
      transmit buffer was full. SAF-50 drop-on-full.*/
 uint16_t window_ms; /*< [ms] Width of the measurement window.*/
 uint16_t heartbeats_missed; /*<  Missed this window.*/
 uint8_t quality; /*<  0-100. The value the degrade decision uses.*/
} mavlink_lawn_link_stats_t;

#define MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN 25
#define MAVLINK_MSG_ID_LAWN_LINK_STATS_MIN_LEN 25
#define MAVLINK_MSG_ID_42012_LEN 25
#define MAVLINK_MSG_ID_42012_MIN_LEN 25

#define MAVLINK_MSG_ID_LAWN_LINK_STATS_CRC 214
#define MAVLINK_MSG_ID_42012_CRC 214



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LAWN_LINK_STATS { \
    42012, \
    "LAWN_LINK_STATS", \
    7, \
    {  { "t_meas_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lawn_link_stats_t, t_meas_us) }, \
         { "frames_accepted", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_lawn_link_stats_t, frames_accepted) }, \
         { "frames_rejected", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_lawn_link_stats_t, frames_rejected) }, \
         { "dropped_tx", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_lawn_link_stats_t, dropped_tx) }, \
         { "window_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_lawn_link_stats_t, window_ms) }, \
         { "heartbeats_missed", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_lawn_link_stats_t, heartbeats_missed) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_lawn_link_stats_t, quality) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LAWN_LINK_STATS { \
    "LAWN_LINK_STATS", \
    7, \
    {  { "t_meas_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lawn_link_stats_t, t_meas_us) }, \
         { "frames_accepted", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_lawn_link_stats_t, frames_accepted) }, \
         { "frames_rejected", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_lawn_link_stats_t, frames_rejected) }, \
         { "dropped_tx", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_lawn_link_stats_t, dropped_tx) }, \
         { "window_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_lawn_link_stats_t, window_ms) }, \
         { "heartbeats_missed", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_lawn_link_stats_t, heartbeats_missed) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_lawn_link_stats_t, quality) }, \
         } \
}
#endif

/**
 * @brief Pack a lawn_link_stats message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param t_meas_us [us] 
 * @param frames_accepted  Accepted this window.
 * @param frames_rejected  CRC, framing, or identity rejects this window.
 * @param dropped_tx  Frames the low-level tier discarded because a
      transmit buffer was full. SAF-50 drop-on-full.
 * @param window_ms [ms] Width of the measurement window.
 * @param heartbeats_missed  Missed this window.
 * @param quality  0-100. The value the degrade decision uses.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_link_stats_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t t_meas_us, uint32_t frames_accepted, uint32_t frames_rejected, uint32_t dropped_tx, uint16_t window_ms, uint16_t heartbeats_missed, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint32_t(buf, 8, frames_accepted);
    _mav_put_uint32_t(buf, 12, frames_rejected);
    _mav_put_uint32_t(buf, 16, dropped_tx);
    _mav_put_uint16_t(buf, 20, window_ms);
    _mav_put_uint16_t(buf, 22, heartbeats_missed);
    _mav_put_uint8_t(buf, 24, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN);
#else
    mavlink_lawn_link_stats_t packet;
    packet.t_meas_us = t_meas_us;
    packet.frames_accepted = frames_accepted;
    packet.frames_rejected = frames_rejected;
    packet.dropped_tx = dropped_tx;
    packet.window_ms = window_ms;
    packet.heartbeats_missed = heartbeats_missed;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_LINK_STATS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LAWN_LINK_STATS_MIN_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_CRC);
}

/**
 * @brief Pack a lawn_link_stats message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param t_meas_us [us] 
 * @param frames_accepted  Accepted this window.
 * @param frames_rejected  CRC, framing, or identity rejects this window.
 * @param dropped_tx  Frames the low-level tier discarded because a
      transmit buffer was full. SAF-50 drop-on-full.
 * @param window_ms [ms] Width of the measurement window.
 * @param heartbeats_missed  Missed this window.
 * @param quality  0-100. The value the degrade decision uses.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_link_stats_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t t_meas_us, uint32_t frames_accepted, uint32_t frames_rejected, uint32_t dropped_tx, uint16_t window_ms, uint16_t heartbeats_missed, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint32_t(buf, 8, frames_accepted);
    _mav_put_uint32_t(buf, 12, frames_rejected);
    _mav_put_uint32_t(buf, 16, dropped_tx);
    _mav_put_uint16_t(buf, 20, window_ms);
    _mav_put_uint16_t(buf, 22, heartbeats_missed);
    _mav_put_uint8_t(buf, 24, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN);
#else
    mavlink_lawn_link_stats_t packet;
    packet.t_meas_us = t_meas_us;
    packet.frames_accepted = frames_accepted;
    packet.frames_rejected = frames_rejected;
    packet.dropped_tx = dropped_tx;
    packet.window_ms = window_ms;
    packet.heartbeats_missed = heartbeats_missed;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_LINK_STATS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_LINK_STATS_MIN_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_LINK_STATS_MIN_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN);
#endif
}

/**
 * @brief Pack a lawn_link_stats message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param t_meas_us [us] 
 * @param frames_accepted  Accepted this window.
 * @param frames_rejected  CRC, framing, or identity rejects this window.
 * @param dropped_tx  Frames the low-level tier discarded because a
      transmit buffer was full. SAF-50 drop-on-full.
 * @param window_ms [ms] Width of the measurement window.
 * @param heartbeats_missed  Missed this window.
 * @param quality  0-100. The value the degrade decision uses.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_link_stats_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t t_meas_us,uint32_t frames_accepted,uint32_t frames_rejected,uint32_t dropped_tx,uint16_t window_ms,uint16_t heartbeats_missed,uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint32_t(buf, 8, frames_accepted);
    _mav_put_uint32_t(buf, 12, frames_rejected);
    _mav_put_uint32_t(buf, 16, dropped_tx);
    _mav_put_uint16_t(buf, 20, window_ms);
    _mav_put_uint16_t(buf, 22, heartbeats_missed);
    _mav_put_uint8_t(buf, 24, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN);
#else
    mavlink_lawn_link_stats_t packet;
    packet.t_meas_us = t_meas_us;
    packet.frames_accepted = frames_accepted;
    packet.frames_rejected = frames_rejected;
    packet.dropped_tx = dropped_tx;
    packet.window_ms = window_ms;
    packet.heartbeats_missed = heartbeats_missed;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_LINK_STATS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LAWN_LINK_STATS_MIN_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_CRC);
}

/**
 * @brief Encode a lawn_link_stats struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lawn_link_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_link_stats_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lawn_link_stats_t* lawn_link_stats)
{
    return mavlink_msg_lawn_link_stats_pack(system_id, component_id, msg, lawn_link_stats->t_meas_us, lawn_link_stats->frames_accepted, lawn_link_stats->frames_rejected, lawn_link_stats->dropped_tx, lawn_link_stats->window_ms, lawn_link_stats->heartbeats_missed, lawn_link_stats->quality);
}

/**
 * @brief Encode a lawn_link_stats struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lawn_link_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_link_stats_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lawn_link_stats_t* lawn_link_stats)
{
    return mavlink_msg_lawn_link_stats_pack_chan(system_id, component_id, chan, msg, lawn_link_stats->t_meas_us, lawn_link_stats->frames_accepted, lawn_link_stats->frames_rejected, lawn_link_stats->dropped_tx, lawn_link_stats->window_ms, lawn_link_stats->heartbeats_missed, lawn_link_stats->quality);
}

/**
 * @brief Encode a lawn_link_stats struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param lawn_link_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_link_stats_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_lawn_link_stats_t* lawn_link_stats)
{
    return mavlink_msg_lawn_link_stats_pack_status(system_id, component_id, _status, msg,  lawn_link_stats->t_meas_us, lawn_link_stats->frames_accepted, lawn_link_stats->frames_rejected, lawn_link_stats->dropped_tx, lawn_link_stats->window_ms, lawn_link_stats->heartbeats_missed, lawn_link_stats->quality);
}

/**
 * @brief Send a lawn_link_stats message
 * @param chan MAVLink channel to send the message
 *
 * @param t_meas_us [us] 
 * @param frames_accepted  Accepted this window.
 * @param frames_rejected  CRC, framing, or identity rejects this window.
 * @param dropped_tx  Frames the low-level tier discarded because a
      transmit buffer was full. SAF-50 drop-on-full.
 * @param window_ms [ms] Width of the measurement window.
 * @param heartbeats_missed  Missed this window.
 * @param quality  0-100. The value the degrade decision uses.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lawn_link_stats_send(mavlink_channel_t chan, uint64_t t_meas_us, uint32_t frames_accepted, uint32_t frames_rejected, uint32_t dropped_tx, uint16_t window_ms, uint16_t heartbeats_missed, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint32_t(buf, 8, frames_accepted);
    _mav_put_uint32_t(buf, 12, frames_rejected);
    _mav_put_uint32_t(buf, 16, dropped_tx);
    _mav_put_uint16_t(buf, 20, window_ms);
    _mav_put_uint16_t(buf, 22, heartbeats_missed);
    _mav_put_uint8_t(buf, 24, quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_LINK_STATS, buf, MAVLINK_MSG_ID_LAWN_LINK_STATS_MIN_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_CRC);
#else
    mavlink_lawn_link_stats_t packet;
    packet.t_meas_us = t_meas_us;
    packet.frames_accepted = frames_accepted;
    packet.frames_rejected = frames_rejected;
    packet.dropped_tx = dropped_tx;
    packet.window_ms = window_ms;
    packet.heartbeats_missed = heartbeats_missed;
    packet.quality = quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_LINK_STATS, (const char *)&packet, MAVLINK_MSG_ID_LAWN_LINK_STATS_MIN_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_CRC);
#endif
}

/**
 * @brief Send a lawn_link_stats message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lawn_link_stats_send_struct(mavlink_channel_t chan, const mavlink_lawn_link_stats_t* lawn_link_stats)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lawn_link_stats_send(chan, lawn_link_stats->t_meas_us, lawn_link_stats->frames_accepted, lawn_link_stats->frames_rejected, lawn_link_stats->dropped_tx, lawn_link_stats->window_ms, lawn_link_stats->heartbeats_missed, lawn_link_stats->quality);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_LINK_STATS, (const char *)lawn_link_stats, MAVLINK_MSG_ID_LAWN_LINK_STATS_MIN_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_CRC);
#endif
}

#if MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lawn_link_stats_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t t_meas_us, uint32_t frames_accepted, uint32_t frames_rejected, uint32_t dropped_tx, uint16_t window_ms, uint16_t heartbeats_missed, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_uint32_t(buf, 8, frames_accepted);
    _mav_put_uint32_t(buf, 12, frames_rejected);
    _mav_put_uint32_t(buf, 16, dropped_tx);
    _mav_put_uint16_t(buf, 20, window_ms);
    _mav_put_uint16_t(buf, 22, heartbeats_missed);
    _mav_put_uint8_t(buf, 24, quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_LINK_STATS, buf, MAVLINK_MSG_ID_LAWN_LINK_STATS_MIN_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_CRC);
#else
    mavlink_lawn_link_stats_t *packet = (mavlink_lawn_link_stats_t *)msgbuf;
    packet->t_meas_us = t_meas_us;
    packet->frames_accepted = frames_accepted;
    packet->frames_rejected = frames_rejected;
    packet->dropped_tx = dropped_tx;
    packet->window_ms = window_ms;
    packet->heartbeats_missed = heartbeats_missed;
    packet->quality = quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_LINK_STATS, (const char *)packet, MAVLINK_MSG_ID_LAWN_LINK_STATS_MIN_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN, MAVLINK_MSG_ID_LAWN_LINK_STATS_CRC);
#endif
}
#endif

#endif

// MESSAGE LAWN_LINK_STATS UNPACKING


/**
 * @brief Get field t_meas_us from lawn_link_stats message
 *
 * @return [us] 
 */
static inline uint64_t mavlink_msg_lawn_link_stats_get_t_meas_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field frames_accepted from lawn_link_stats message
 *
 * @return  Accepted this window.
 */
static inline uint32_t mavlink_msg_lawn_link_stats_get_frames_accepted(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field frames_rejected from lawn_link_stats message
 *
 * @return  CRC, framing, or identity rejects this window.
 */
static inline uint32_t mavlink_msg_lawn_link_stats_get_frames_rejected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field dropped_tx from lawn_link_stats message
 *
 * @return  Frames the low-level tier discarded because a
      transmit buffer was full. SAF-50 drop-on-full.
 */
static inline uint32_t mavlink_msg_lawn_link_stats_get_dropped_tx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field window_ms from lawn_link_stats message
 *
 * @return [ms] Width of the measurement window.
 */
static inline uint16_t mavlink_msg_lawn_link_stats_get_window_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field heartbeats_missed from lawn_link_stats message
 *
 * @return  Missed this window.
 */
static inline uint16_t mavlink_msg_lawn_link_stats_get_heartbeats_missed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field quality from lawn_link_stats message
 *
 * @return  0-100. The value the degrade decision uses.
 */
static inline uint8_t mavlink_msg_lawn_link_stats_get_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Decode a lawn_link_stats message into a struct
 *
 * @param msg The message to decode
 * @param lawn_link_stats C-struct to decode the message contents into
 */
static inline void mavlink_msg_lawn_link_stats_decode(const mavlink_message_t* msg, mavlink_lawn_link_stats_t* lawn_link_stats)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lawn_link_stats->t_meas_us = mavlink_msg_lawn_link_stats_get_t_meas_us(msg);
    lawn_link_stats->frames_accepted = mavlink_msg_lawn_link_stats_get_frames_accepted(msg);
    lawn_link_stats->frames_rejected = mavlink_msg_lawn_link_stats_get_frames_rejected(msg);
    lawn_link_stats->dropped_tx = mavlink_msg_lawn_link_stats_get_dropped_tx(msg);
    lawn_link_stats->window_ms = mavlink_msg_lawn_link_stats_get_window_ms(msg);
    lawn_link_stats->heartbeats_missed = mavlink_msg_lawn_link_stats_get_heartbeats_missed(msg);
    lawn_link_stats->quality = mavlink_msg_lawn_link_stats_get_quality(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN? msg->len : MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN;
        memset(lawn_link_stats, 0, MAVLINK_MSG_ID_LAWN_LINK_STATS_LEN);
    memcpy(lawn_link_stats, _MAV_PAYLOAD(msg), len);
#endif
}
