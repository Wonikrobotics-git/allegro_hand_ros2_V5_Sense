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
 *  @file AllegroHandDrvBase.h
 *  @brief Abstract interface for Allegro Hand communication drivers.
 *
 *  Both AllegroHandDrv (CAN) and AllegroHandTcpDrv (UDP over Ethernet)
 *  implement this interface so that controller code can work with
 *  either transport without modification.
 *
 *  Created on:         March 23, 2026
 *  Author:             Wonik Robotics
 */

#ifndef __ALLEGROHAND_DRV_BASE_H__
#define __ALLEGROHAND_DRV_BASE_H__

#include <string>
#include "AllegroHandDef.h"

// Shared global variables used by both CAN and TCP drivers and controllers.
inline int OperatingMode = 0;
inline double f[4];
inline double x[4];
inline double y[4];
inline double z[4];
inline float fingertip_sensor[4];
inline float madi_sensor[11];
inline float palm_sensor;

inline int motor_current[16];

namespace allegro
{

class AllegroHandDrvBase
{
public:
    virtual ~AllegroHandDrvBase() {}

    virtual bool init(std::string channel, int mode = 0) = 0;

    virtual void setTorque(double *torque) = 0;
    virtual void getJointInfo(double *position) = 0;

    virtual int readFrames() = 0;
    virtual int writeJointTorque() = 0;
    virtual bool isJointInfoReady() = 0;
    virtual void resetJointInfoReady() = 0;

    virtual bool emergencyStop() = 0;
    virtual double torqueConversion() = 0;
    virtual double inputVoltage() = 0;

    /// Send motor calibration command (resets all encoder values to 0).
    virtual void calibrate() {}

    /// Send joint position targets directly to the hand (H-inf onboard control mode).
    /// q_rad[16]: desired joint angles in radians.
    /// Converts rad -> encoder counts and sends via command_set_pose x4 (one per finger).
    /// No-op in the base class (only implemented for CAN transport).
    virtual void sendPositionDirect(const double* /*q_rad*/) {}

    /// Enable/Disable H-inf onboard control mode on the firmware.
    /// enable=true: firmware interprets IDs 0x180..0x18C as position targets.
    /// enable=false: firmware interprets IDs 0x180..0x18C as torques (default).
    virtual void setHinfMode(bool /*enable*/) {}

    /// Send roll/pitch/yaw for firmware gravity feed-forward.
    /// Values are degrees and are converted by the transport to centi-degrees.
    virtual void sendGravityFeedForwardRPY(double /*roll_deg*/,
                                           double /*pitch_deg*/,
                                           double /*yaw_deg*/) {}

    bool RIGHT_HAND = true;
};

} // namespace allegro

#endif // __ALLEGROHAND_DRV_BASE_H__
