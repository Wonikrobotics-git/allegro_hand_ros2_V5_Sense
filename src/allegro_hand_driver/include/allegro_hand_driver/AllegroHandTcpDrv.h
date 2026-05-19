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
 *  @file AllegroHandTcpDrv.h
 *  @brief Allegro Hand Ethernet driver — UDP-only (port 7000): HELLO/commands,
 *         TELEMETRY_RAW, TORQUE_CMD, MOTOR_ERROR on one datagram socket.
 *
 *  Drop-in replacement for AllegroHandDrv using Ethernet instead of CAN bus.
 *  Maintains the same public interface for minimal controller code changes.
 *
 *  Controller migration:
 *    1. #include "allegro_hand_driver/AllegroHandTcpDrv.h"
 *    2. allegro::AllegroHandTcpDrv* canDevice = new allegro::AllegroHandTcpDrv();
 *    3. canDevice->init("192.168.1.100");       // optional :PORT (UDP peer, default 7000)
 *    4. canDevice->readCANFrames();              // same API (alias for readFrames)
 *
 *  Created on:         March 23, 2026
 *  Author:             Wonik Robotics
 */

#ifndef __ALLEGROHAND_TCP_DRV_H__
#define __ALLEGROHAND_TCP_DRV_H__

#include <string>
#include "AllegroHandDef.h"
#include "AllegroHandDrvBase.h"
#include "tcpdrv/tcpdef.h"

namespace allegro
{

class AllegroHandTcpDrv : public AllegroHandDrvBase
{
public:
    AllegroHandTcpDrv();
    ~AllegroHandTcpDrv() override;

    bool init(std::string peer_addr, int mode = 0) override;

    void setTorque(double *torque) override;
    void getJointInfo(double *position) override;

    bool emergencyStop() override { return _emergency_stop; }
    double torqueConversion() override { return _tau_cov_const; }
    double inputVoltage() override { return _input_voltage; }
    void calibrate() override;

    int readFrames() override;               ///< drain UDP telemetry / events
    int writeJointTorque() override;
    bool isJointInfoReady() override;
    void resetJointInfoReady() override;

    /// H-inf position targets are sent on the torque packet type in H-inf mode.
    void sendPositionDirect(const double* q_rad) override;
    void setHinfMode(bool enable) override;
    void sendGravityFeedForwardRPY(double roll_deg,
                                   double pitch_deg,
                                   double yaw_deg) override;

    /*=== Ethernet (UDP) extras ===*/

    void getJointCurrent(double *current);   ///< get measured motor current (mA)
    void getJointVelocity(int32_t *velocity); ///< get raw motor velocity
    void getSensorData(uint16_t *sensor);    ///< get raw 16-channel sensor values

private:
    double _curr_position[DOF_JOINTS];       ///< current joint position (radian)
    double _curr_torque[DOF_JOINTS];         ///< current joint torque (Nm)
    double _desired_torque[DOF_JOINTS];      ///< desired joint torque (internal unit, mA)

    double _curr_current[DOF_JOINTS];        ///< measured motor current from telemetry (mA)
    int32_t _curr_velocity[DOF_JOINTS];      ///< measured velocity from telemetry
    uint16_t _curr_sensor[DOF_JOINTS];       ///< raw sensor values from telemetry

    double _hand_version;                    ///< hand hardware version
    double _tau_cov_const;                   ///< torque conversion constant
    double _input_voltage;                   ///< input voltage

    int _curr_position_get;                  ///< bit flag for position update (0x0F = all ready)

    volatile bool _emergency_stop;           ///< emergency stop flag

    /*
    * Remap firmware wire order -> CAN / URDF joint order
    *   Wire: thumb(0-3), index(4-7), middle(8-11), ring(12-15)
    *   CAN:  index(0-3), middle(4-7), ring(8-11),  thumb(12-15)
    */
    static const int joint_remapping[DOF_JOINTS];

    /* Sensor/current mapping — identical rules to AllegroHandDrv (CAN) */
    static const int FingertipSensorMap[4];   ///< sensor_buf indices for fingertip sensors
    static const int MadiSensorMap[11];       ///< sensor_buf indices for MADI sensors
    static const int CurrentMap[16];          ///< maps internal current order → motor_current[]

    float _sensor_buf[DOF_JOINTS] = {0};      ///< EMA-filtered sensor buffer in CAN joint order

private:
    void _readDevices();
    void _writeDevices();
    void _parseTelemetry(const TelemetryPacket* telemetry);
    void _parseHelloResp(const HelloResp* resp);
    void _handleMotorError(const MotorError* err);
    bool _parseAddress(const std::string& addr, std::string& ip, int& port);

    void _applySerialPayload(const SerialResp* sr);
    /** After HELLO: request serial (same role as CAN ID_RTR_SERIAL). */
    void _readSerialNumberBlocking();
};

} /* namespace allegro */

#endif /* __ALLEGROHAND_TCP_DRV_H__ */
