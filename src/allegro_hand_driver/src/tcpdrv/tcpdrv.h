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
 *  @file tcpdrv.h
 *  @brief UDP-only datagram API (legacy file name: historically TCP+UDP).
 *
 *  Created on:         March 23, 2026
 *  Author:             Wonik Robotics
 */

#ifndef __ALLEGROHAND_TCPDRV_H__
#define __ALLEGROHAND_TCPDRV_H__

#include "tcpdef.h"

namespace TCPAPI {

int udp_open(const char* fw_ip, int peer_port, int local_bind_port);

int udp_close();

bool udp_is_open();

/**
 * Block until the UDP socket is readable or timeout.
 * @return 1 readable, 0 timeout, -1 error
 */
int udp_wait_readable(int timeout_ms);

/**
 * Send one protocol datagram (header + optional payload).
 * @param out_seq if non-null, receives the seq field sent in the header (for ACK matching).
 */
int udp_send_packet(uint8_t type, const void* payload, uint16_t length,
                    uint32_t* out_seq = nullptr);

int udp_send_torque_cmd(const TorqueCmd* cmd);

int udp_send_hello_req(uint32_t* out_seq = nullptr);

int udp_send_header_only(uint8_t type, uint32_t* out_seq = nullptr);

int udp_send_net_config_req(uint32_t* out_seq = nullptr);
int udp_send_net_config_write(const NetConfigPayload* cfg, uint32_t* out_seq = nullptr);

/**
 * Non-blocking receive: one datagram if available.
 * @return 0 complete packet, -1 no packet / short / bad magic, -2 fatal
 */
int udp_recv_packet(PacketHeader* header, uint8_t* payload, int max_payload);

} /* namespace TCPAPI */

#endif /* __ALLEGROHAND_TCPDRV_H__ */
