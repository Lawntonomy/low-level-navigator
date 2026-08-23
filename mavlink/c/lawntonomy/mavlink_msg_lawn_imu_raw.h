#pragma once
// MESSAGE LAWN_IMU_RAW PACKING

#define MAVLINK_MSG_ID_LAWN_IMU_RAW 42030


typedef struct __mavlink_lawn_imu_raw_t {
 uint64_t t_meas_us; /*< [us] Data-ready edge, hardware captured.*/
 int16_t ax; /*<  Accelerometer X, raw counts.*/
 int16_t ay; /*<  Accelerometer Y, raw counts.*/
 int16_t az; /*<  Accelerometer Z, raw counts.*/
 int16_t gx; /*<  Gyroscope X, raw counts.*/
 int16_t gy; /*<  Gyroscope Y, raw counts.*/
 int16_t gz; /*<  Gyroscope Z, raw counts.*/
 uint8_t gap; /*<  Samples lost immediately before this one, 255 = saturated.
      ADR-0007: gaps are reported, never silently interpolated.*/
} mavlink_lawn_imu_raw_t;

#define MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN 21
#define MAVLINK_MSG_ID_LAWN_IMU_RAW_MIN_LEN 21
#define MAVLINK_MSG_ID_42030_LEN 21
#define MAVLINK_MSG_ID_42030_MIN_LEN 21

#define MAVLINK_MSG_ID_LAWN_IMU_RAW_CRC 63
#define MAVLINK_MSG_ID_42030_CRC 63



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LAWN_IMU_RAW { \
    42030, \
    "LAWN_IMU_RAW", \
    8, \
    {  { "t_meas_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lawn_imu_raw_t, t_meas_us) }, \
         { "ax", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_lawn_imu_raw_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_lawn_imu_raw_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_lawn_imu_raw_t, az) }, \
         { "gx", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_lawn_imu_raw_t, gx) }, \
         { "gy", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_lawn_imu_raw_t, gy) }, \
         { "gz", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_lawn_imu_raw_t, gz) }, \
         { "gap", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_lawn_imu_raw_t, gap) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LAWN_IMU_RAW { \
    "LAWN_IMU_RAW", \
    8, \
    {  { "t_meas_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lawn_imu_raw_t, t_meas_us) }, \
         { "ax", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_lawn_imu_raw_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_lawn_imu_raw_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_lawn_imu_raw_t, az) }, \
         { "gx", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_lawn_imu_raw_t, gx) }, \
         { "gy", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_lawn_imu_raw_t, gy) }, \
         { "gz", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_lawn_imu_raw_t, gz) }, \
         { "gap", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_lawn_imu_raw_t, gap) }, \
         } \
}
#endif

/**
 * @brief Pack a lawn_imu_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param t_meas_us [us] Data-ready edge, hardware captured.
 * @param ax  Accelerometer X, raw counts.
 * @param ay  Accelerometer Y, raw counts.
 * @param az  Accelerometer Z, raw counts.
 * @param gx  Gyroscope X, raw counts.
 * @param gy  Gyroscope Y, raw counts.
 * @param gz  Gyroscope Z, raw counts.
 * @param gap  Samples lost immediately before this one, 255 = saturated.
      ADR-0007: gaps are reported, never silently interpolated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_imu_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t t_meas_us, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, uint8_t gap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_int16_t(buf, 8, ax);
    _mav_put_int16_t(buf, 10, ay);
    _mav_put_int16_t(buf, 12, az);
    _mav_put_int16_t(buf, 14, gx);
    _mav_put_int16_t(buf, 16, gy);
    _mav_put_int16_t(buf, 18, gz);
    _mav_put_uint8_t(buf, 20, gap);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN);
#else
    mavlink_lawn_imu_raw_t packet;
    packet.t_meas_us = t_meas_us;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.gx = gx;
    packet.gy = gy;
    packet.gz = gz;
    packet.gap = gap;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_IMU_RAW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LAWN_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_CRC);
}

/**
 * @brief Pack a lawn_imu_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param t_meas_us [us] Data-ready edge, hardware captured.
 * @param ax  Accelerometer X, raw counts.
 * @param ay  Accelerometer Y, raw counts.
 * @param az  Accelerometer Z, raw counts.
 * @param gx  Gyroscope X, raw counts.
 * @param gy  Gyroscope Y, raw counts.
 * @param gz  Gyroscope Z, raw counts.
 * @param gap  Samples lost immediately before this one, 255 = saturated.
      ADR-0007: gaps are reported, never silently interpolated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_imu_raw_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t t_meas_us, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, uint8_t gap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_int16_t(buf, 8, ax);
    _mav_put_int16_t(buf, 10, ay);
    _mav_put_int16_t(buf, 12, az);
    _mav_put_int16_t(buf, 14, gx);
    _mav_put_int16_t(buf, 16, gy);
    _mav_put_int16_t(buf, 18, gz);
    _mav_put_uint8_t(buf, 20, gap);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN);
#else
    mavlink_lawn_imu_raw_t packet;
    packet.t_meas_us = t_meas_us;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.gx = gx;
    packet.gy = gy;
    packet.gz = gz;
    packet.gap = gap;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_IMU_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LAWN_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN);
#endif
}

/**
 * @brief Pack a lawn_imu_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param t_meas_us [us] Data-ready edge, hardware captured.
 * @param ax  Accelerometer X, raw counts.
 * @param ay  Accelerometer Y, raw counts.
 * @param az  Accelerometer Z, raw counts.
 * @param gx  Gyroscope X, raw counts.
 * @param gy  Gyroscope Y, raw counts.
 * @param gz  Gyroscope Z, raw counts.
 * @param gap  Samples lost immediately before this one, 255 = saturated.
      ADR-0007: gaps are reported, never silently interpolated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lawn_imu_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t t_meas_us,int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,uint8_t gap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_int16_t(buf, 8, ax);
    _mav_put_int16_t(buf, 10, ay);
    _mav_put_int16_t(buf, 12, az);
    _mav_put_int16_t(buf, 14, gx);
    _mav_put_int16_t(buf, 16, gy);
    _mav_put_int16_t(buf, 18, gz);
    _mav_put_uint8_t(buf, 20, gap);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN);
#else
    mavlink_lawn_imu_raw_t packet;
    packet.t_meas_us = t_meas_us;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.gx = gx;
    packet.gy = gy;
    packet.gz = gz;
    packet.gap = gap;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LAWN_IMU_RAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LAWN_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_CRC);
}

/**
 * @brief Encode a lawn_imu_raw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lawn_imu_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_imu_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lawn_imu_raw_t* lawn_imu_raw)
{
    return mavlink_msg_lawn_imu_raw_pack(system_id, component_id, msg, lawn_imu_raw->t_meas_us, lawn_imu_raw->ax, lawn_imu_raw->ay, lawn_imu_raw->az, lawn_imu_raw->gx, lawn_imu_raw->gy, lawn_imu_raw->gz, lawn_imu_raw->gap);
}

/**
 * @brief Encode a lawn_imu_raw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lawn_imu_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_imu_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lawn_imu_raw_t* lawn_imu_raw)
{
    return mavlink_msg_lawn_imu_raw_pack_chan(system_id, component_id, chan, msg, lawn_imu_raw->t_meas_us, lawn_imu_raw->ax, lawn_imu_raw->ay, lawn_imu_raw->az, lawn_imu_raw->gx, lawn_imu_raw->gy, lawn_imu_raw->gz, lawn_imu_raw->gap);
}

/**
 * @brief Encode a lawn_imu_raw struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param lawn_imu_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lawn_imu_raw_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_lawn_imu_raw_t* lawn_imu_raw)
{
    return mavlink_msg_lawn_imu_raw_pack_status(system_id, component_id, _status, msg,  lawn_imu_raw->t_meas_us, lawn_imu_raw->ax, lawn_imu_raw->ay, lawn_imu_raw->az, lawn_imu_raw->gx, lawn_imu_raw->gy, lawn_imu_raw->gz, lawn_imu_raw->gap);
}

/**
 * @brief Send a lawn_imu_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param t_meas_us [us] Data-ready edge, hardware captured.
 * @param ax  Accelerometer X, raw counts.
 * @param ay  Accelerometer Y, raw counts.
 * @param az  Accelerometer Z, raw counts.
 * @param gx  Gyroscope X, raw counts.
 * @param gy  Gyroscope Y, raw counts.
 * @param gz  Gyroscope Z, raw counts.
 * @param gap  Samples lost immediately before this one, 255 = saturated.
      ADR-0007: gaps are reported, never silently interpolated.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lawn_imu_raw_send(mavlink_channel_t chan, uint64_t t_meas_us, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, uint8_t gap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN];
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_int16_t(buf, 8, ax);
    _mav_put_int16_t(buf, 10, ay);
    _mav_put_int16_t(buf, 12, az);
    _mav_put_int16_t(buf, 14, gx);
    _mav_put_int16_t(buf, 16, gy);
    _mav_put_int16_t(buf, 18, gz);
    _mav_put_uint8_t(buf, 20, gap);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_IMU_RAW, buf, MAVLINK_MSG_ID_LAWN_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_CRC);
#else
    mavlink_lawn_imu_raw_t packet;
    packet.t_meas_us = t_meas_us;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.gx = gx;
    packet.gy = gy;
    packet.gz = gz;
    packet.gap = gap;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_IMU_RAW, (const char *)&packet, MAVLINK_MSG_ID_LAWN_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_CRC);
#endif
}

/**
 * @brief Send a lawn_imu_raw message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lawn_imu_raw_send_struct(mavlink_channel_t chan, const mavlink_lawn_imu_raw_t* lawn_imu_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lawn_imu_raw_send(chan, lawn_imu_raw->t_meas_us, lawn_imu_raw->ax, lawn_imu_raw->ay, lawn_imu_raw->az, lawn_imu_raw->gx, lawn_imu_raw->gy, lawn_imu_raw->gz, lawn_imu_raw->gap);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_IMU_RAW, (const char *)lawn_imu_raw, MAVLINK_MSG_ID_LAWN_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_CRC);
#endif
}

#if MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lawn_imu_raw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t t_meas_us, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, uint8_t gap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, t_meas_us);
    _mav_put_int16_t(buf, 8, ax);
    _mav_put_int16_t(buf, 10, ay);
    _mav_put_int16_t(buf, 12, az);
    _mav_put_int16_t(buf, 14, gx);
    _mav_put_int16_t(buf, 16, gy);
    _mav_put_int16_t(buf, 18, gz);
    _mav_put_uint8_t(buf, 20, gap);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_IMU_RAW, buf, MAVLINK_MSG_ID_LAWN_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_CRC);
#else
    mavlink_lawn_imu_raw_t *packet = (mavlink_lawn_imu_raw_t *)msgbuf;
    packet->t_meas_us = t_meas_us;
    packet->ax = ax;
    packet->ay = ay;
    packet->az = az;
    packet->gx = gx;
    packet->gy = gy;
    packet->gz = gz;
    packet->gap = gap;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LAWN_IMU_RAW, (const char *)packet, MAVLINK_MSG_ID_LAWN_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN, MAVLINK_MSG_ID_LAWN_IMU_RAW_CRC);
#endif
}
#endif

#endif

// MESSAGE LAWN_IMU_RAW UNPACKING


/**
 * @brief Get field t_meas_us from lawn_imu_raw message
 *
 * @return [us] Data-ready edge, hardware captured.
 */
static inline uint64_t mavlink_msg_lawn_imu_raw_get_t_meas_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field ax from lawn_imu_raw message
 *
 * @return  Accelerometer X, raw counts.
 */
static inline int16_t mavlink_msg_lawn_imu_raw_get_ax(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field ay from lawn_imu_raw message
 *
 * @return  Accelerometer Y, raw counts.
 */
static inline int16_t mavlink_msg_lawn_imu_raw_get_ay(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field az from lawn_imu_raw message
 *
 * @return  Accelerometer Z, raw counts.
 */
static inline int16_t mavlink_msg_lawn_imu_raw_get_az(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field gx from lawn_imu_raw message
 *
 * @return  Gyroscope X, raw counts.
 */
static inline int16_t mavlink_msg_lawn_imu_raw_get_gx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field gy from lawn_imu_raw message
 *
 * @return  Gyroscope Y, raw counts.
 */
static inline int16_t mavlink_msg_lawn_imu_raw_get_gy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field gz from lawn_imu_raw message
 *
 * @return  Gyroscope Z, raw counts.
 */
static inline int16_t mavlink_msg_lawn_imu_raw_get_gz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field gap from lawn_imu_raw message
 *
 * @return  Samples lost immediately before this one, 255 = saturated.
      ADR-0007: gaps are reported, never silently interpolated.
 */
static inline uint8_t mavlink_msg_lawn_imu_raw_get_gap(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a lawn_imu_raw message into a struct
 *
 * @param msg The message to decode
 * @param lawn_imu_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_lawn_imu_raw_decode(const mavlink_message_t* msg, mavlink_lawn_imu_raw_t* lawn_imu_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lawn_imu_raw->t_meas_us = mavlink_msg_lawn_imu_raw_get_t_meas_us(msg);
    lawn_imu_raw->ax = mavlink_msg_lawn_imu_raw_get_ax(msg);
    lawn_imu_raw->ay = mavlink_msg_lawn_imu_raw_get_ay(msg);
    lawn_imu_raw->az = mavlink_msg_lawn_imu_raw_get_az(msg);
    lawn_imu_raw->gx = mavlink_msg_lawn_imu_raw_get_gx(msg);
    lawn_imu_raw->gy = mavlink_msg_lawn_imu_raw_get_gy(msg);
    lawn_imu_raw->gz = mavlink_msg_lawn_imu_raw_get_gz(msg);
    lawn_imu_raw->gap = mavlink_msg_lawn_imu_raw_get_gap(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN? msg->len : MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN;
        memset(lawn_imu_raw, 0, MAVLINK_MSG_ID_LAWN_IMU_RAW_LEN);
    memcpy(lawn_imu_raw, _MAV_PAYLOAD(msg), len);
#endif
}
