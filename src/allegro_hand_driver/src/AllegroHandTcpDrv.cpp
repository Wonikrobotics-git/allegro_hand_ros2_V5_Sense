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
 *  @file AllegroHandTcpDrv.cpp
 *  @brief Allegro Hand Ethernet — UDP-only (commands + telemetry + torque on port 7000)
 *
 *  Created on:         March 23, 2026
 *  Author:             Wonik Robotics
 */

#include <chrono>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

#include "tcpdrv/tcpdrv.h"
#include "allegro_hand_driver/AllegroHandTcpDrv.h"

using namespace std;

#define MAX_DOF 16
#define LOG_TAG "allegro_hand_eth_drv"

#define TORQUE_TO_MA    (1.43 * 1000.0)
#define TORQUE_LIMIT_MA 240.0
#define ENCODER_TO_RAD  (M_PI / 180.0 * 0.088)
#define RAD_TO_ENCODER  (180.0 / (M_PI * 0.088))


namespace allegro
{

static int16_t degreesToCentiDegrees(double degrees)
{
    double value = degrees * 100.0;
    if (value > 32767.0) value = 32767.0;
    if (value < -32768.0) value = -32768.0;
    return static_cast<int16_t>(lround(value));
}

static int16_t radiansToEncoderCount(double radians)
{
    double value = radians * RAD_TO_ENCODER;
    if (value > 32767.0) value = 32767.0;
    if (value < -32768.0) value = -32768.0;
    return static_cast<int16_t>(lround(value));
}

const int AllegroHandTcpDrv::joint_remapping[DOF_JOINTS] = {
     4,  5,  6,  7,
     8,  9, 10, 11,
    12, 13, 14, 15,
     0,  1,  2,  3
};

const int AllegroHandTcpDrv::FingertipSensorMap[4]  = { 3, 7, 11, 15 };
const int AllegroHandTcpDrv::MadiSensorMap[11]       = { 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14 };
const int AllegroHandTcpDrv::CurrentMap[16]          = { 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3 };

AllegroHandTcpDrv::AllegroHandTcpDrv()
    : _hand_version(0.0)
    , _tau_cov_const(1.0)
    , _input_voltage(0.0)
    , _curr_position_get(0)
    , _emergency_stop(false)
{
    memset(_curr_position,  0, sizeof(_curr_position));
    memset(_curr_torque,    0, sizeof(_curr_torque));
    memset(_desired_torque, 0, sizeof(_desired_torque));
    memset(_curr_current,   0, sizeof(_curr_current));
    memset(_curr_velocity,  0, sizeof(_curr_velocity));
    memset(_curr_sensor,    0, sizeof(_curr_sensor));
    memset(_sensor_buf,     0, sizeof(_sensor_buf));

    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
        "AllegroHandTcpDrv (UDP Ethernet) constructed.");
}

AllegroHandTcpDrv::~AllegroHandTcpDrv()
{
    TCPAPI::udp_close();
}

bool AllegroHandTcpDrv::_parseAddress(const std::string& addr,
                                       std::string& ip, int& port)
{
    port = DEFAULT_UDP_PORT;

    size_t colon_pos = addr.rfind(':');
    if (colon_pos == std::string::npos) {
        ip = addr;
    } else {
        ip = addr.substr(0, colon_pos);
        std::string port_str = addr.substr(colon_pos + 1);
        try {
            port = std::stoi(port_str);
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
                "Invalid port in address: %s", addr.c_str());
            return false;
        }
    }

    if (ip.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
            "Empty IP address");
        return false;
    }

    return true;
}

bool AllegroHandTcpDrv::init(std::string addr, int mode)
{
    (void)mode;

    while (!addr.empty() && isspace(static_cast<unsigned char>(addr.back())))
        addr.pop_back();

    if (addr.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
            "Invalid (empty) address. Use IP or IP:PORT (UDP, default %d)",
            DEFAULT_UDP_PORT);
        return false;
    }

    std::string ip;
    int peer_port = DEFAULT_UDP_PORT;
    if (!_parseAddress(addr, ip, peer_port))
        return false;

    if (TCPAPI::udp_open(ip.c_str(), peer_port, UDP_LOCAL_PORT) != 0)
        return false;

    bool hello_ok = false;

    for (int attempt = 0; attempt < CMD_ACK_MAX_RETRIES && !hello_ok; attempt++) {
        uint32_t hello_seq = 0;
        RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
            "UDP: HELLO_REQ attempt %d/%d", attempt + 1, CMD_ACK_MAX_RETRIES);

        if (TCPAPI::udp_send_packet(TYPE_HELLO_REQ, nullptr, 0, &hello_seq) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
                "UDP: Failed to send HELLO_REQ");
            TCPAPI::udp_close();
            return false;
        }

        const auto t0 = std::chrono::steady_clock::now();

        while (true) {
            const auto elapsed = std::chrono::steady_clock::now() - t0;
            if (elapsed >= std::chrono::milliseconds(CMD_ACK_TIMEOUT_MS)) {
                break;
            }
            int ms_left = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::milliseconds(CMD_ACK_TIMEOUT_MS) - elapsed).count());
            if (ms_left < 1) ms_left = 1;

            int wr = TCPAPI::udp_wait_readable(ms_left);
            if (wr < 0) {
                RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG), "UDP: poll error");
                TCPAPI::udp_close();
                return false;
            }
            if (wr == 0) break;

            PacketHeader header;
            uint8_t payload[256];
            if (TCPAPI::udp_recv_packet(&header, payload, sizeof(payload)) != 0)
                continue;
            if (header.type != TYPE_HELLO_RESP || header.length < HELLO_RESP_PAYLOAD)
                continue;
            if (header.seq != hello_seq) {
                RCLCPP_WARN(rclcpp::get_logger(LOG_TAG),
                    "UDP: HELLO_RESP seq %u != req %u; accepting anyway",
                    static_cast<unsigned>(header.seq),
                    static_cast<unsigned>(hello_seq));
            }
            _parseHelloResp(reinterpret_cast<const HelloResp*>(payload));
            hello_ok = true;
            RCLCPP_INFO(rclcpp::get_logger(LOG_TAG), "UDP: HELLO acknowledged");
            break;
        }
    }

    if (!hello_ok) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
            "UDP: HELLO timeout (%d ms, %d attempts). "
            "Check IP/port, firewall, and firmware UDP HELLO support.",
            CMD_ACK_TIMEOUT_MS, CMD_ACK_MAX_RETRIES);
        TCPAPI::udp_close();
        return false;
    }

    usleep(3000);

    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG), "UDP: Session ready");

    _readSerialNumberBlocking();

    return true;
}

int AllegroHandTcpDrv::readFrames()
{
    if (_emergency_stop)
        return -1;

    _readDevices();

    return 0;
}

int AllegroHandTcpDrv::writeJointTorque()
{
    _writeDevices();

    if (_emergency_stop) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
            "Emergency stop in writeJointTorque()");
        return -1;
    }

    return 0;
}

bool AllegroHandTcpDrv::isJointInfoReady()
{
    return (_curr_position_get == (0x01 | 0x02 | 0x04 | 0x08));
}

void AllegroHandTcpDrv::resetJointInfoReady()
{
    _curr_position_get = 0;
}

void AllegroHandTcpDrv::setTorque(double *torque)
{
    for (int findex = 0; findex < 4; findex++) {
        _desired_torque[4 * findex + 0] = torque[4 * findex + 0] * TORQUE_TO_MA;
        _desired_torque[4 * findex + 1] = torque[4 * findex + 1] * TORQUE_TO_MA;
        _desired_torque[4 * findex + 2] = torque[4 * findex + 2] * TORQUE_TO_MA;
        _desired_torque[4 * findex + 3] = torque[4 * findex + 3] * TORQUE_TO_MA;
    }

    _desired_torque[1] = 0.5 * _desired_torque[1];
    _desired_torque[5] = 0.5 * _desired_torque[5];
    _desired_torque[9] = 0.5 * _desired_torque[9];
}

void AllegroHandTcpDrv::getJointInfo(double *position)
{
    for (int i = 0; i < DOF_JOINTS; i++) {
        position[i] = _curr_position[joint_remapping[i]];
    }
}

void AllegroHandTcpDrv::getJointCurrent(double *current)
{
    for (int i = 0; i < DOF_JOINTS; i++) {
        current[i] = _curr_current[joint_remapping[i]];
    }
}

void AllegroHandTcpDrv::getJointVelocity(int32_t *velocity)
{
    for (int i = 0; i < DOF_JOINTS; i++) {
        velocity[i] = _curr_velocity[joint_remapping[i]];
    }
}

void AllegroHandTcpDrv::getSensorData(uint16_t *sensor)
{
    for (int i = 0; i < DOF_JOINTS; i++) {
        sensor[i] = _curr_sensor[i];
    }
}

void AllegroHandTcpDrv::calibrate()
{
    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
        "UDP: Sending joint angle calibration (JAC_REQ)");
    if (TCPAPI::udp_send_header_only(TYPE_JAC_REQ) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
            "UDP: Failed to send JAC_REQ");
    }
}

void AllegroHandTcpDrv::sendPositionDirect(const double* q_rad)
{
    TorqueCmd cmd;
    memset(&cmd, 0, sizeof(cmd));

    for (int i = 0; i < DOF_JOINTS; i++) {
        cmd.torque[joint_remapping[i]] = radiansToEncoderCount(q_rad[i]);
    }

    if (TCPAPI::udp_send_packet(TYPE_TORQUE_CMD, &cmd, TORQUE_CMD_PAYLOAD,
            nullptr) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
            "UDP: Failed to send H-inf position target packet");
        _emergency_stop = true;
    }
}

void AllegroHandTcpDrv::setHinfMode(bool enable)
{
    uint8_t payload = enable ? 1 : 0;

    for (int i = 0; i < 3; i++) {
        if (TCPAPI::udp_send_packet(TYPE_HINF_MODE, &payload, HINF_MODE_PAYLOAD,
                nullptr) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
                "UDP: Failed to send HINF_MODE");
            return;
        }
        usleep(2000);
    }

    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
        "UDP: H-inf mode %s (type 0x%02X)", enable ? "ON" : "OFF",
        TYPE_HINF_MODE);
}

void AllegroHandTcpDrv::sendGravityFeedForwardRPY(double roll_deg,
                                                  double pitch_deg,
                                                  double yaw_deg)
{
    GffRpyReq req;
    req.roll_cdeg = degreesToCentiDegrees(roll_deg);
    req.pitch_cdeg = degreesToCentiDegrees(pitch_deg);
    req.yaw_cdeg = degreesToCentiDegrees(yaw_deg);

    if (TCPAPI::udp_send_packet(TYPE_GFF_RPY_REQ, &req, GFF_RPY_PAYLOAD,
            nullptr) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
            "UDP: Failed to send GFF_RPY_REQ");
        _emergency_stop = true;
    }
}

void AllegroHandTcpDrv::_readDevices()
{
    PacketHeader header;
    uint8_t payload[256];
    int ret;

    while (TCPAPI::udp_is_open()) {
        ret = TCPAPI::udp_recv_packet(&header, payload, sizeof(payload));
        if (ret == 0) {
            if (header.type == TYPE_TELEMETRY_RAW &&
                header.length >= TELEMETRY_PAYLOAD) {
                _parseTelemetry(reinterpret_cast<const TelemetryPacket*>(payload));
            } else if (header.type == TYPE_SN_RESP &&
                       header.length >= SN_RESP_PAYLOAD) {
                _applySerialPayload(reinterpret_cast<const SerialResp*>(payload));
            } else if (header.type == TYPE_MOTOR_ERROR &&
                       header.length >= MOTOR_ERROR_PAYLOAD) {
                _handleMotorError(reinterpret_cast<const MotorError*>(payload));
            } else {
                RCLCPP_DEBUG(rclcpp::get_logger(LOG_TAG),
                    "UDP: packet type 0x%02X len %u", header.type,
                    static_cast<unsigned>(header.length));
            }
        } else if (ret == -2) {
            _emergency_stop = true;
            break;
        } else {
            break;
        }
    }
}

void AllegroHandTcpDrv::_writeDevices()
{
    if (!isJointInfoReady())
        return;

    TorqueCmd cmd;

    for (int i = 0; i < DOF_JOINTS; i++) {
        double val = _desired_torque[i];

        if (val > TORQUE_LIMIT_MA)       val = TORQUE_LIMIT_MA;
        else if (val < -TORQUE_LIMIT_MA) val = -TORQUE_LIMIT_MA;

        cmd.torque[joint_remapping[i]] = static_cast<int16_t>(val);
    }

    if (TCPAPI::udp_send_packet(TYPE_TORQUE_CMD, &cmd, TORQUE_CMD_PAYLOAD,
            nullptr) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
            "UDP: Failed to send TORQUE_CMD");
        _emergency_stop = true;
    }
}

void AllegroHandTcpDrv::_parseTelemetry(const TelemetryPacket* telemetry)
{
    for (int i = 0; i < DOF_JOINTS; i++) {
        _curr_position[i] = static_cast<double>(telemetry->position[i]) * ENCODER_TO_RAD;
    }

    for (int i = 0; i < DOF_JOINTS; i++) {
        _curr_current[i] = static_cast<double>(telemetry->current[i]);
    }
    for (int i = 0; i < DOF_JOINTS; i++) {
        motor_current[CurrentMap[i]] =
            static_cast<int>(_curr_current[joint_remapping[i]]);
    }

    for (int i = 0; i < DOF_JOINTS; i++) {
        _curr_sensor[i] = telemetry->sensor[i];
    }

    for (int i = 0; i < DOF_JOINTS; i++) {
        _curr_velocity[i] = telemetry->velocity[i];
    }

    float alpha = 0.25f;

    for (int i = 0; i < DOF_JOINTS; i++) {
        float raw = static_cast<float>(telemetry->sensor[i]) / 10.0f;
        if (raw < 0 || raw > 1000) raw = 0;
        _sensor_buf[i] = alpha * raw + (1.0f - alpha) * _sensor_buf[i];
    }

    for (int i = 0; i < 4; i++)
        fingertip_sensor[i] = _sensor_buf[FingertipSensorMap[i]];
    for (int i = 0; i < 11; i++)
        madi_sensor[i] = _sensor_buf[MadiSensorMap[i]];
    palm_sensor = _sensor_buf[0];

    _curr_position_get = 0x01 | 0x02 | 0x04 | 0x08;
}

void AllegroHandTcpDrv::_applySerialPayload(const SerialResp* sr)
{
    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
        "AllegroHand serial number: %c%c%c%c%c%c%c%c",
        sr->serial[0], sr->serial[1], sr->serial[2], sr->serial[3],
        sr->serial[4], sr->serial[5], sr->serial[6], sr->serial[7]);
    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
        "                      hardware type: %s",
        (sr->serial[3] == 'R') ? "right" : "left");

    if (sr->serial[3] == 'R') {
        RIGHT_HAND = true;
    } else {
        RIGHT_HAND = false;
    }
}

void AllegroHandTcpDrv::_readSerialNumberBlocking()
{
    const uint8_t req = TYPE_SN_READ_REQ;
    const uint8_t resp_type = TYPE_SN_RESP;

    for (int attempt = 0; attempt < CMD_ACK_MAX_RETRIES; attempt++) {
        uint32_t sn_seq = 0;
        RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
            "UDP: SN_READ_REQ 0x%02X (%d/%d)",
            req, attempt + 1, CMD_ACK_MAX_RETRIES);

        if (TCPAPI::udp_send_packet(req, nullptr, 0, &sn_seq) != 0) {
            RCLCPP_WARN(rclcpp::get_logger(LOG_TAG),
                "UDP: failed to send SN_READ_REQ; keeping default hand side");
            return;
        }

        const auto t0 = std::chrono::steady_clock::now();

        while (true) {
            const auto elapsed = std::chrono::steady_clock::now() - t0;
            if (elapsed >= std::chrono::milliseconds(CMD_ACK_TIMEOUT_MS)) {
                break;
            }
            int ms_left = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::milliseconds(CMD_ACK_TIMEOUT_MS) - elapsed).count());
            if (ms_left < 1) {
                ms_left = 1;
            }

            int wr = TCPAPI::udp_wait_readable(ms_left);
            if (wr < 0) {
                RCLCPP_WARN(rclcpp::get_logger(LOG_TAG),
                    "UDP: poll error during SN read");
                return;
            }
            if (wr == 0) {
                break;
            }

            PacketHeader header;
            uint8_t payload[256];
            int ret = TCPAPI::udp_recv_packet(&header, payload, sizeof(payload));
            if (ret != 0) {
                continue;
            }
            if (header.type == resp_type && header.length >= SN_RESP_PAYLOAD) {
                if (header.seq != sn_seq) {
                    RCLCPP_WARN(rclcpp::get_logger(LOG_TAG),
                        "UDP: SN_RESP seq %u != req %u; still applying",
                        static_cast<unsigned>(header.seq),
                        static_cast<unsigned>(sn_seq));
                }
                _applySerialPayload(reinterpret_cast<const SerialResp*>(payload));
                return;
            }
            /* Drain other traffic (telemetry, etc.) */
        }
    }

    RCLCPP_WARN(rclcpp::get_logger(LOG_TAG),
        "UDP: serial number timeout; RIGHT_HAND left at default — "
        "may still arrive later over SN_RESP");
}

void AllegroHandTcpDrv::_parseHelloResp(const HelloResp* resp)
{
    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
        "Hand hardware version: 0x%02x%02x",
        resp->data[1], resp->data[0]);
    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
        "Firmware version: 0x%02x%02x",
        resp->data[3], resp->data[2]);
    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG),
        "Servo status: %s",
        (resp->data[6] & 0x01) ? "ON" : "OFF");
}

void AllegroHandTcpDrv::_handleMotorError(const MotorError* err)
{
    RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG),
        "Motor error: joint %d (1-based), status 0x%02X",
        err->joint_index, err->error_status);

    _emergency_stop = true;
}

} /* namespace allegro */
