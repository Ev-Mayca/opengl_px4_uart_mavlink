#pragma once
// MESSAGE CA_TRAJECTORY PACKING

#define MAVLINK_MSG_ID_CA_TRAJECTORY 166

MAVPACKED(
typedef struct __mavlink_ca_trajectory_t {
 uint64_t timestamp; /*< Timestamp in milliseconds since system boot*/
 uint64_t time_start_usec; /*< start time, unit usec.*/
 uint64_t time_stop_usec; /*< stop time, unit usec.*/
 uint32_t coefficients; /*< as it says.*/
 uint16_t seq_id; /*< can not cheat any more.*/
}) mavlink_ca_trajectory_t;

#define MAVLINK_MSG_ID_CA_TRAJECTORY_LEN 30
#define MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN 30
#define MAVLINK_MSG_ID_166_LEN 30
#define MAVLINK_MSG_ID_166_MIN_LEN 30

#define MAVLINK_MSG_ID_CA_TRAJECTORY_CRC 136
#define MAVLINK_MSG_ID_166_CRC 136



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CA_TRAJECTORY { \
    166, \
    "CA_TRAJECTORY", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ca_trajectory_t, timestamp) }, \
         { "time_start_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_ca_trajectory_t, time_start_usec) }, \
         { "time_stop_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_ca_trajectory_t, time_stop_usec) }, \
         { "coefficients", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_ca_trajectory_t, coefficients) }, \
         { "seq_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_ca_trajectory_t, seq_id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CA_TRAJECTORY { \
    "CA_TRAJECTORY", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ca_trajectory_t, timestamp) }, \
         { "time_start_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_ca_trajectory_t, time_start_usec) }, \
         { "time_stop_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_ca_trajectory_t, time_stop_usec) }, \
         { "coefficients", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_ca_trajectory_t, coefficients) }, \
         { "seq_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_ca_trajectory_t, seq_id) }, \
         } \
}
#endif

/**
 * @brief Pack a ca_trajectory message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp in milliseconds since system boot
 * @param time_start_usec start time, unit usec.
 * @param time_stop_usec stop time, unit usec.
 * @param coefficients as it says.
 * @param seq_id can not cheat any more.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ca_trajectory_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint64_t time_start_usec, uint64_t time_stop_usec, uint32_t coefficients, uint16_t seq_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CA_TRAJECTORY_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, time_start_usec);
    _mav_put_uint64_t(buf, 16, time_stop_usec);
    _mav_put_uint32_t(buf, 24, coefficients);
    _mav_put_uint16_t(buf, 28, seq_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN);
#else
    mavlink_ca_trajectory_t packet;
    packet.timestamp = timestamp;
    packet.time_start_usec = time_start_usec;
    packet.time_stop_usec = time_stop_usec;
    packet.coefficients = coefficients;
    packet.seq_id = seq_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CA_TRAJECTORY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
}

/**
 * @brief Pack a ca_trajectory message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp in milliseconds since system boot
 * @param time_start_usec start time, unit usec.
 * @param time_stop_usec stop time, unit usec.
 * @param coefficients as it says.
 * @param seq_id can not cheat any more.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ca_trajectory_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint64_t time_start_usec,uint64_t time_stop_usec,uint32_t coefficients,uint16_t seq_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CA_TRAJECTORY_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, time_start_usec);
    _mav_put_uint64_t(buf, 16, time_stop_usec);
    _mav_put_uint32_t(buf, 24, coefficients);
    _mav_put_uint16_t(buf, 28, seq_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN);
#else
    mavlink_ca_trajectory_t packet;
    packet.timestamp = timestamp;
    packet.time_start_usec = time_start_usec;
    packet.time_stop_usec = time_stop_usec;
    packet.coefficients = coefficients;
    packet.seq_id = seq_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CA_TRAJECTORY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
}

/**
 * @brief Encode a ca_trajectory struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ca_trajectory C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ca_trajectory_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ca_trajectory_t* ca_trajectory)
{
    return mavlink_msg_ca_trajectory_pack(system_id, component_id, msg, ca_trajectory->timestamp, ca_trajectory->time_start_usec, ca_trajectory->time_stop_usec, ca_trajectory->coefficients, ca_trajectory->seq_id);
}

/**
 * @brief Encode a ca_trajectory struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ca_trajectory C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ca_trajectory_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ca_trajectory_t* ca_trajectory)
{
    return mavlink_msg_ca_trajectory_pack_chan(system_id, component_id, chan, msg, ca_trajectory->timestamp, ca_trajectory->time_start_usec, ca_trajectory->time_stop_usec, ca_trajectory->coefficients, ca_trajectory->seq_id);
}

/**
 * @brief Send a ca_trajectory message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp in milliseconds since system boot
 * @param time_start_usec start time, unit usec.
 * @param time_stop_usec stop time, unit usec.
 * @param coefficients as it says.
 * @param seq_id can not cheat any more.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ca_trajectory_send(mavlink_channel_t chan, uint64_t timestamp, uint64_t time_start_usec, uint64_t time_stop_usec, uint32_t coefficients, uint16_t seq_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CA_TRAJECTORY_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, time_start_usec);
    _mav_put_uint64_t(buf, 16, time_stop_usec);
    _mav_put_uint32_t(buf, 24, coefficients);
    _mav_put_uint16_t(buf, 28, seq_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECTORY, buf, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
#else
    mavlink_ca_trajectory_t packet;
    packet.timestamp = timestamp;
    packet.time_start_usec = time_start_usec;
    packet.time_stop_usec = time_stop_usec;
    packet.coefficients = coefficients;
    packet.seq_id = seq_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECTORY, (const char *)&packet, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
#endif
}

/**
 * @brief Send a ca_trajectory message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ca_trajectory_send_struct(mavlink_channel_t chan, const mavlink_ca_trajectory_t* ca_trajectory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ca_trajectory_send(chan, ca_trajectory->timestamp, ca_trajectory->time_start_usec, ca_trajectory->time_stop_usec, ca_trajectory->coefficients, ca_trajectory->seq_id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECTORY, (const char *)ca_trajectory, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
#endif
}

#if MAVLINK_MSG_ID_CA_TRAJECTORY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ca_trajectory_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint64_t time_start_usec, uint64_t time_stop_usec, uint32_t coefficients, uint16_t seq_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, time_start_usec);
    _mav_put_uint64_t(buf, 16, time_stop_usec);
    _mav_put_uint32_t(buf, 24, coefficients);
    _mav_put_uint16_t(buf, 28, seq_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECTORY, buf, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
#else
    mavlink_ca_trajectory_t *packet = (mavlink_ca_trajectory_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->time_start_usec = time_start_usec;
    packet->time_stop_usec = time_stop_usec;
    packet->coefficients = coefficients;
    packet->seq_id = seq_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECTORY, (const char *)packet, MAVLINK_MSG_ID_CA_TRAJECTORY_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN, MAVLINK_MSG_ID_CA_TRAJECTORY_CRC);
#endif
}
#endif

#endif

// MESSAGE CA_TRAJECTORY UNPACKING


/**
 * @brief Get field timestamp from ca_trajectory message
 *
 * @return Timestamp in milliseconds since system boot
 */
static inline uint64_t mavlink_msg_ca_trajectory_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_start_usec from ca_trajectory message
 *
 * @return start time, unit usec.
 */
static inline uint64_t mavlink_msg_ca_trajectory_get_time_start_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field time_stop_usec from ca_trajectory message
 *
 * @return stop time, unit usec.
 */
static inline uint64_t mavlink_msg_ca_trajectory_get_time_stop_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Get field coefficients from ca_trajectory message
 *
 * @return as it says.
 */
static inline uint32_t mavlink_msg_ca_trajectory_get_coefficients(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field seq_id from ca_trajectory message
 *
 * @return can not cheat any more.
 */
static inline uint16_t mavlink_msg_ca_trajectory_get_seq_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Decode a ca_trajectory message into a struct
 *
 * @param msg The message to decode
 * @param ca_trajectory C-struct to decode the message contents into
 */
static inline void mavlink_msg_ca_trajectory_decode(const mavlink_message_t* msg, mavlink_ca_trajectory_t* ca_trajectory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ca_trajectory->timestamp = mavlink_msg_ca_trajectory_get_timestamp(msg);
    ca_trajectory->time_start_usec = mavlink_msg_ca_trajectory_get_time_start_usec(msg);
    ca_trajectory->time_stop_usec = mavlink_msg_ca_trajectory_get_time_stop_usec(msg);
    ca_trajectory->coefficients = mavlink_msg_ca_trajectory_get_coefficients(msg);
    ca_trajectory->seq_id = mavlink_msg_ca_trajectory_get_seq_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CA_TRAJECTORY_LEN? msg->len : MAVLINK_MSG_ID_CA_TRAJECTORY_LEN;
        memset(ca_trajectory, 0, MAVLINK_MSG_ID_CA_TRAJECTORY_LEN);
    memcpy(ca_trajectory, _MAV_PAYLOAD(msg), len);
#endif
}
