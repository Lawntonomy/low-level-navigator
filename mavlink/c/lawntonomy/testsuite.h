/** @file
 *    @brief MAVLink comm protocol testsuite generated from lawntonomy.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef LAWNTONOMY_TESTSUITE_H
#define LAWNTONOMY_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_minimal(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_lawntonomy(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_minimal(system_id, component_id, last_msg);
    mavlink_test_lawntonomy(system_id, component_id, last_msg);
}
#endif

#include "../minimal/testsuite.h"


static void mavlink_test_lawn_drive_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LAWN_DRIVE_CMD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lawn_drive_cmd_t packet_in = {
        17235,17339
    };
    mavlink_lawn_drive_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.left_drpm = packet_in.left_drpm;
        packet1.right_drpm = packet_in.right_drpm;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LAWN_DRIVE_CMD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LAWN_DRIVE_CMD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_drive_cmd_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lawn_drive_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_drive_cmd_pack(system_id, component_id, &msg , packet1.left_drpm , packet1.right_drpm );
    mavlink_msg_lawn_drive_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_drive_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.left_drpm , packet1.right_drpm );
    mavlink_msg_lawn_drive_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lawn_drive_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_drive_cmd_send(MAVLINK_COMM_1 , packet1.left_drpm , packet1.right_drpm );
    mavlink_msg_lawn_drive_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LAWN_DRIVE_CMD") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LAWN_DRIVE_CMD) != NULL);
#endif
}

static void mavlink_test_lawn_arm_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LAWN_ARM_CMD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lawn_arm_cmd_t packet_in = {
        17235,139
    };
    mavlink_lawn_arm_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.magic = packet_in.magic;
        packet1.arm = packet_in.arm;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LAWN_ARM_CMD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LAWN_ARM_CMD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_arm_cmd_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lawn_arm_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_arm_cmd_pack(system_id, component_id, &msg , packet1.magic , packet1.arm );
    mavlink_msg_lawn_arm_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_arm_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.magic , packet1.arm );
    mavlink_msg_lawn_arm_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lawn_arm_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_arm_cmd_send(MAVLINK_COMM_1 , packet1.magic , packet1.arm );
    mavlink_msg_lawn_arm_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LAWN_ARM_CMD") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LAWN_ARM_CMD) != NULL);
#endif
}

static void mavlink_test_lawn_stop_req(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LAWN_STOP_REQ >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lawn_stop_req_t packet_in = {
        5
    };
    mavlink_lawn_stop_req_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.reason = packet_in.reason;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LAWN_STOP_REQ_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LAWN_STOP_REQ_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_stop_req_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lawn_stop_req_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_stop_req_pack(system_id, component_id, &msg , packet1.reason );
    mavlink_msg_lawn_stop_req_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_stop_req_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.reason );
    mavlink_msg_lawn_stop_req_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lawn_stop_req_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_stop_req_send(MAVLINK_COMM_1 , packet1.reason );
    mavlink_msg_lawn_stop_req_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LAWN_STOP_REQ") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LAWN_STOP_REQ) != NULL);
#endif
}

static void mavlink_test_lawn_enter_bootloader(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lawn_enter_bootloader_t packet_in = {
        963497464
    };
    mavlink_lawn_enter_bootloader_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.magic = packet_in.magic;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_enter_bootloader_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lawn_enter_bootloader_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_enter_bootloader_pack(system_id, component_id, &msg , packet1.magic );
    mavlink_msg_lawn_enter_bootloader_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_enter_bootloader_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.magic );
    mavlink_msg_lawn_enter_bootloader_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lawn_enter_bootloader_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_enter_bootloader_send(MAVLINK_COMM_1 , packet1.magic );
    mavlink_msg_lawn_enter_bootloader_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LAWN_ENTER_BOOTLOADER") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER) != NULL);
#endif
}

static void mavlink_test_lawn_nav_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LAWN_NAV_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lawn_nav_status_t packet_in = {
        93372036854775807ULL,17651,163,230,41
    };
    mavlink_lawn_nav_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.t_meas_us = packet_in.t_meas_us;
        packet1.cmd_age_ms = packet_in.cmd_age_ms;
        packet1.nav_state = packet_in.nav_state;
        packet1.armed = packet_in.armed;
        packet1.fault = packet_in.fault;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LAWN_NAV_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LAWN_NAV_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_nav_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lawn_nav_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_nav_status_pack(system_id, component_id, &msg , packet1.t_meas_us , packet1.cmd_age_ms , packet1.nav_state , packet1.armed , packet1.fault );
    mavlink_msg_lawn_nav_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_nav_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.t_meas_us , packet1.cmd_age_ms , packet1.nav_state , packet1.armed , packet1.fault );
    mavlink_msg_lawn_nav_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lawn_nav_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_nav_status_send(MAVLINK_COMM_1 , packet1.t_meas_us , packet1.cmd_age_ms , packet1.nav_state , packet1.armed , packet1.fault );
    mavlink_msg_lawn_nav_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LAWN_NAV_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LAWN_NAV_STATUS) != NULL);
#endif
}

static void mavlink_test_lawn_wheel_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LAWN_WHEEL_STATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lawn_wheel_state_t packet_in = {
        93372036854775807ULL,17651,17755,17859,17963,53
    };
    mavlink_lawn_wheel_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.t_meas_us = packet_in.t_meas_us;
        packet1.left_drpm = packet_in.left_drpm;
        packet1.right_drpm = packet_in.right_drpm;
        packet1.left_cmd_drpm = packet_in.left_cmd_drpm;
        packet1.right_cmd_drpm = packet_in.right_cmd_drpm;
        packet1.flags = packet_in.flags;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LAWN_WHEEL_STATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LAWN_WHEEL_STATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_wheel_state_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lawn_wheel_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_wheel_state_pack(system_id, component_id, &msg , packet1.t_meas_us , packet1.left_drpm , packet1.right_drpm , packet1.left_cmd_drpm , packet1.right_cmd_drpm , packet1.flags );
    mavlink_msg_lawn_wheel_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_wheel_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.t_meas_us , packet1.left_drpm , packet1.right_drpm , packet1.left_cmd_drpm , packet1.right_cmd_drpm , packet1.flags );
    mavlink_msg_lawn_wheel_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lawn_wheel_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_wheel_state_send(MAVLINK_COMM_1 , packet1.t_meas_us , packet1.left_drpm , packet1.right_drpm , packet1.left_cmd_drpm , packet1.right_cmd_drpm , packet1.flags );
    mavlink_msg_lawn_wheel_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LAWN_WHEEL_STATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LAWN_WHEEL_STATE) != NULL);
#endif
}

static void mavlink_test_lawn_link_stats(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LAWN_LINK_STATS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lawn_link_stats_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,18275,18379,77
    };
    mavlink_lawn_link_stats_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.t_meas_us = packet_in.t_meas_us;
        packet1.frames_accepted = packet_in.frames_accepted;
        packet1.frames_rejected = packet_in.frames_rejected;
        packet1.dropped_tx = packet_in.dropped_tx;
        packet1.window_ms = packet_in.window_ms;
        packet1.heartbeats_missed = packet_in.heartbeats_missed;
        packet1.quality = packet_in.quality;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LAWN_LINK_STATS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LAWN_LINK_STATS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_link_stats_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lawn_link_stats_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_link_stats_pack(system_id, component_id, &msg , packet1.t_meas_us , packet1.frames_accepted , packet1.frames_rejected , packet1.dropped_tx , packet1.window_ms , packet1.heartbeats_missed , packet1.quality );
    mavlink_msg_lawn_link_stats_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_link_stats_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.t_meas_us , packet1.frames_accepted , packet1.frames_rejected , packet1.dropped_tx , packet1.window_ms , packet1.heartbeats_missed , packet1.quality );
    mavlink_msg_lawn_link_stats_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lawn_link_stats_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_link_stats_send(MAVLINK_COMM_1 , packet1.t_meas_us , packet1.frames_accepted , packet1.frames_rejected , packet1.dropped_tx , packet1.window_ms , packet1.heartbeats_missed , packet1.quality );
    mavlink_msg_lawn_link_stats_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LAWN_LINK_STATS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LAWN_LINK_STATS) != NULL);
#endif
}

static void mavlink_test_lawn_fault_event(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LAWN_FAULT_EVENT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lawn_fault_event_t packet_in = {
        93372036854775807ULL,17651,163,230
    };
    mavlink_lawn_fault_event_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.t_meas_us = packet_in.t_meas_us;
        packet1.code = packet_in.code;
        packet1.nav_state = packet_in.nav_state;
        packet1.latched = packet_in.latched;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LAWN_FAULT_EVENT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LAWN_FAULT_EVENT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_fault_event_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lawn_fault_event_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_fault_event_pack(system_id, component_id, &msg , packet1.t_meas_us , packet1.code , packet1.nav_state , packet1.latched );
    mavlink_msg_lawn_fault_event_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_fault_event_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.t_meas_us , packet1.code , packet1.nav_state , packet1.latched );
    mavlink_msg_lawn_fault_event_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lawn_fault_event_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_fault_event_send(MAVLINK_COMM_1 , packet1.t_meas_us , packet1.code , packet1.nav_state , packet1.latched );
    mavlink_msg_lawn_fault_event_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LAWN_FAULT_EVENT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LAWN_FAULT_EVENT) != NULL);
#endif
}

static void mavlink_test_lawn_timesync(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LAWN_TIMESYNC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lawn_timesync_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,77
    };
    mavlink_lawn_timesync_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.t1_us = packet_in.t1_us;
        packet1.t2_us = packet_in.t2_us;
        packet1.t3_us = packet_in.t3_us;
        packet1.exchange_seq = packet_in.exchange_seq;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LAWN_TIMESYNC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LAWN_TIMESYNC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_timesync_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lawn_timesync_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_timesync_pack(system_id, component_id, &msg , packet1.t1_us , packet1.t2_us , packet1.t3_us , packet1.exchange_seq );
    mavlink_msg_lawn_timesync_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_timesync_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.t1_us , packet1.t2_us , packet1.t3_us , packet1.exchange_seq );
    mavlink_msg_lawn_timesync_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lawn_timesync_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_timesync_send(MAVLINK_COMM_1 , packet1.t1_us , packet1.t2_us , packet1.t3_us , packet1.exchange_seq );
    mavlink_msg_lawn_timesync_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LAWN_TIMESYNC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LAWN_TIMESYNC) != NULL);
#endif
}

static void mavlink_test_lawn_imu_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LAWN_IMU_RAW >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lawn_imu_raw_t packet_in = {
        93372036854775807ULL,17651,17755,17859,17963,18067,18171,65
    };
    mavlink_lawn_imu_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.t_meas_us = packet_in.t_meas_us;
        packet1.ax = packet_in.ax;
        packet1.ay = packet_in.ay;
        packet1.az = packet_in.az;
        packet1.gx = packet_in.gx;
        packet1.gy = packet_in.gy;
        packet1.gz = packet_in.gz;
        packet1.gap = packet_in.gap;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LAWN_IMU_RAW_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LAWN_IMU_RAW_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_imu_raw_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lawn_imu_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_imu_raw_pack(system_id, component_id, &msg , packet1.t_meas_us , packet1.ax , packet1.ay , packet1.az , packet1.gx , packet1.gy , packet1.gz , packet1.gap );
    mavlink_msg_lawn_imu_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_imu_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.t_meas_us , packet1.ax , packet1.ay , packet1.az , packet1.gx , packet1.gy , packet1.gz , packet1.gap );
    mavlink_msg_lawn_imu_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lawn_imu_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lawn_imu_raw_send(MAVLINK_COMM_1 , packet1.t_meas_us , packet1.ax , packet1.ay , packet1.az , packet1.gx , packet1.gy , packet1.gz , packet1.gap );
    mavlink_msg_lawn_imu_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LAWN_IMU_RAW") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LAWN_IMU_RAW) != NULL);
#endif
}

static void mavlink_test_lawntonomy(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_lawn_drive_cmd(system_id, component_id, last_msg);
    mavlink_test_lawn_arm_cmd(system_id, component_id, last_msg);
    mavlink_test_lawn_stop_req(system_id, component_id, last_msg);
    mavlink_test_lawn_enter_bootloader(system_id, component_id, last_msg);
    mavlink_test_lawn_nav_status(system_id, component_id, last_msg);
    mavlink_test_lawn_wheel_state(system_id, component_id, last_msg);
    mavlink_test_lawn_link_stats(system_id, component_id, last_msg);
    mavlink_test_lawn_fault_event(system_id, component_id, last_msg);
    mavlink_test_lawn_timesync(system_id, component_id, last_msg);
    mavlink_test_lawn_imu_raw(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // LAWNTONOMY_TESTSUITE_H
