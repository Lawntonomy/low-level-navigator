/** @file
 *  @brief MAVLink comm protocol generated from lawntonomy.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_LAWNTONOMY_H
#define MAVLINK_LAWNTONOMY_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_LAWNTONOMY.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_LAWNTONOMY_XML_HASH 9065425222778940082

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 50, 9, 9, 0, 0, 0}, {42001, 158, 4, 4, 0, 0, 0}, {42002, 10, 3, 3, 0, 0, 0}, {42003, 119, 1, 1, 0, 0, 0}, {42010, 131, 13, 13, 0, 0, 0}, {42011, 90, 17, 17, 0, 0, 0}, {42012, 214, 25, 25, 0, 0, 0}, {42013, 223, 12, 12, 0, 0, 0}, {42020, 72, 25, 25, 0, 0, 0}, {42030, 63, 21, 21, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_LAWNTONOMY

// ENUM DEFINITIONS


/** @brief Navigator state machine. Mirrors low-level-navigator/README.md. */
#ifndef HAVE_ENUM_LAWN_NAV_STATE
#define HAVE_ENUM_LAWN_NAV_STATE
typedef enum LAWN_NAV_STATE
{
   LAWN_NAV_PRECAL_IDLE=0, /* No valid calibration. Zero speed. Drive not enabled. | */
   LAWN_NAV_CALIBRATING=1, /* Calibration in progress. Motion may be commanded internally. | */
   LAWN_NAV_IDLE=2, /* Valid calibration, armed or not, no guidance commands being executed. | */
   LAWN_NAV_ACTIVE=3, /* Executing guidance commands. | */
   LAWN_NAV_EXITING=4, /* No fresh commands; ramping both wheels to zero. SAF-1. | */
   LAWN_NAV_FAULT=5, /* Latched fault. Drive deasserted. Requires explicit clear. | */
   LAWN_NAV_STATE_ENUM_END=6, /*  | */
} LAWN_NAV_STATE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_LAWN_WHEEL_FLAGS
#define HAVE_ENUM_LAWN_WHEEL_FLAGS
typedef enum LAWN_WHEEL_FLAGS
{
   LAWN_WHEEL_LEFT_VALID=1, /* Left wheel speed is fresh. SAF-20. | */
   LAWN_WHEEL_RIGHT_VALID=2, /* Right wheel speed is fresh. SAF-20. | */
   LAWN_WHEEL_LEFT_STALL=4, /* Left wheel stall detected. SAF-21. | */
   LAWN_WHEEL_RIGHT_STALL=8, /* Right wheel stall detected. SAF-21. | */
   LAWN_WHEEL_DIR_UNMEASURED=16, /* Direction is inferred from command, not measured. Always set until
        direction-observable encoders exist. See ADR-0002. | */
   LAWN_WHEEL_FLAGS_ENUM_END=17, /*  | */
} LAWN_WHEEL_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_LAWN_FAULT_CODE
#define HAVE_ENUM_LAWN_FAULT_CODE
typedef enum LAWN_FAULT_CODE
{
   LAWN_FAULT_NONE=0, /*  | */
   LAWN_FAULT_CMD_TIMEOUT=1, /* No valid command within the timeout. SAF-1. | */
   LAWN_FAULT_LINK_DEGRADED=2, /* Link quality below threshold while still nominally up. SAF-54. | */
   LAWN_FAULT_WHEEL_STALL=3, /*  | */
   LAWN_FAULT_WHEEL_INVALID=4, /* Wheel speed stale or implausible. SAF-20. | */
   LAWN_FAULT_DIR_MISMATCH=5, /* Commanded direction disagrees with chassis motion. SAF-23. | */
   LAWN_FAULT_TILT=6, /*  | */
   LAWN_FAULT_INIT_FAILED=7, /*  | */
   LAWN_FAULT_OVERTEMP=8, /*  | */
   LAWN_FAULT_CODE_ENUM_END=9, /*  | */
} LAWN_FAULT_CODE;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_lawn_drive_cmd.h"
#include "./mavlink_msg_lawn_arm_cmd.h"
#include "./mavlink_msg_lawn_stop_req.h"
#include "./mavlink_msg_lawn_nav_status.h"
#include "./mavlink_msg_lawn_wheel_state.h"
#include "./mavlink_msg_lawn_link_stats.h"
#include "./mavlink_msg_lawn_fault_event.h"
#include "./mavlink_msg_lawn_timesync.h"
#include "./mavlink_msg_lawn_imu_raw.h"

// base include
#include "../minimal/minimal.h"


#if MAVLINK_LAWNTONOMY_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_LAWN_DRIVE_CMD, MAVLINK_MESSAGE_INFO_LAWN_ARM_CMD, MAVLINK_MESSAGE_INFO_LAWN_STOP_REQ, MAVLINK_MESSAGE_INFO_LAWN_NAV_STATUS, MAVLINK_MESSAGE_INFO_LAWN_WHEEL_STATE, MAVLINK_MESSAGE_INFO_LAWN_LINK_STATS, MAVLINK_MESSAGE_INFO_LAWN_FAULT_EVENT, MAVLINK_MESSAGE_INFO_LAWN_TIMESYNC, MAVLINK_MESSAGE_INFO_LAWN_IMU_RAW}
# define MAVLINK_MESSAGE_NAMES {{ "HEARTBEAT", 0 }, { "LAWN_ARM_CMD", 42002 }, { "LAWN_DRIVE_CMD", 42001 }, { "LAWN_FAULT_EVENT", 42013 }, { "LAWN_IMU_RAW", 42030 }, { "LAWN_LINK_STATS", 42012 }, { "LAWN_NAV_STATUS", 42010 }, { "LAWN_STOP_REQ", 42003 }, { "LAWN_TIMESYNC", 42020 }, { "LAWN_WHEEL_STATE", 42011 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_LAWNTONOMY_H
