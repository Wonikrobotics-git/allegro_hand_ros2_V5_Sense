/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Wonik Robotics.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Wonik Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  @file tcpdef.h
 *  @brief UDP-only protocol: single datagram channel (port 7000), version 0x01.
 *
 *  Created on:         March 23, 2026
 *  Author:             Wonik Robotics
 */

#ifndef __ALLEGROHAND_TCPDEF_H__
#define __ALLEGROHAND_TCPDEF_H__

#include <stdint.h>

/*=====================================================*/
/*       Transport                                     */
/*=====================================================*/

#define PROTO_MAGIC              0x32423556  /* Frame sync / magic word (little-endian) */
#define PROTO_VERSION            0x01

/** Default / recommended UDP port (PC bind + FW peer); configurable. */
#define DEFAULT_UDP_PORT         7000
#define UDP_LOCAL_PORT           DEFAULT_UDP_PORT
#define UDP_PEER_PORT            DEFAULT_UDP_PORT

/*=====================================================*/
/*       Packet Types (UDP-only spec)                   */
/*=====================================================*/

/* PC -> FW commands */
#define TYPE_HELLO_REQ           0x10
#define TYPE_JAC_REQ             0x20
#define TYPE_NET_CONFIG_REQ      0x30
#define TYPE_NET_CONFIG_WRITE    0x31
#define TYPE_GFF_RPY_REQ         0x61
#define TYPE_SN_READ_REQ         0x34
#define TYPE_TORQUE_CMD          0x03
#define TYPE_HINF_MODE           0x60
#define TYPE_CLI_CMD_REQ         0x40

/* FW -> PC stream / ACK */
#define TYPE_TELEMETRY_RAW       0x02
#define TYPE_HELLO_RESP          0x11
#define TYPE_CAL_DONE            0x21
#define TYPE_NET_CONFIG_RESP     0x33
#define TYPE_TTL_RESP            0x37
#define TYPE_MOTOR_ERROR         0x50
#define TYPE_CLI_CMD_RESP        0x41

/*
 * SN_READ_REQ -> SN_RESP: exact code not listed in the UDP-only summary table;
 * 0x35 reserved between SN_READ_REQ (0x34) and TTL_RESP (0x37).
 */
#define TYPE_SN_RESP             0x35

/*=====================================================*/
/*       Payload Sizes                                 */
/*=====================================================*/

#define HEADER_SIZE            16
#define TELEMETRY_PAYLOAD      164
#define TORQUE_CMD_PAYLOAD     32
#define HELLO_RESP_PAYLOAD     7
#define TTL_RESP_PAYLOAD       8
#define SN_RESP_PAYLOAD        8
#define MOTOR_ERROR_PAYLOAD    2
#define NET_CONFIG_PAYLOAD     14
#define GFF_RPY_PAYLOAD        6
#define HINF_MODE_PAYLOAD      1

#define TELEMETRY_PACKET_SIZE  (HEADER_SIZE + TELEMETRY_PAYLOAD)
#define TORQUE_CMD_PACKET_SIZE (HEADER_SIZE + TORQUE_CMD_PAYLOAD)

/*=====================================================*/
/*       Timing (spec recommendations)                  */
/*=====================================================*/

#define TELEMETRY_PERIOD_MS      3
#define CMD_ACK_TIMEOUT_MS       1000
#define CMD_ACK_MAX_RETRIES      3

/*=====================================================*/
/*       Packed Structs                                */
/*=====================================================*/

#pragma pack(push, 1)

typedef struct {
    uint32_t magic;
    uint8_t  version;
    uint8_t  type;
    uint16_t length;
    uint32_t seq;
    uint32_t timestamp_ms;
} PacketHeader;

typedef struct {
    uint32_t sample_seq;
    int16_t  position[16];
    int16_t  current[16];
    uint16_t sensor[16];
    int32_t  velocity[16];
} TelemetryPacket;

typedef struct {
    int16_t  torque[16];
} TorqueCmd;

typedef struct {
    int16_t  roll_cdeg;
    int16_t  pitch_cdeg;
    int16_t  yaw_cdeg;
} GffRpyReq;

/** Same 7-byte layout as CAN hand-info (not serial). R/L comes via SN_RESP. */
typedef struct {
    uint8_t  data[7];
} HelloResp;

typedef struct {
    uint32_t motor_fail_mask;
    uint32_t sensor_fail_mask;
} TtlResp;

typedef struct {
    uint8_t  serial[8];
} SerialResp;

typedef struct {
    uint8_t  joint_index;
    uint8_t  error_status;
} MotorError;

#pragma pack(pop)

#endif /* __ALLEGROHAND_TCPDEF_H__ */
