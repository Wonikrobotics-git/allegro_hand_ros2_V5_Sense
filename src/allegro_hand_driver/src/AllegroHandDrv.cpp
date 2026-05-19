/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Wonik Robotics.
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
 *  @file AllegroHandDrv.cpp
 *  @brief Allegro Hand Driver
 *
 *  Created on:         Nov 15, 2012
 *  Added to Project:   Jan 17, 2013
 *  Author:             Sean Yi, K.C.Chang, Seungsu Kim, & Alex Alspach
 *  Maintained by:      Sean Yi(seanyi@wonikrobotics.com)
 */

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "candrv/candrv.h"
#include "allegro_hand_driver/AllegroHandDrv.h"
#include "unistd.h"

using namespace std;

#define MAX_DOF 16

#define PWM_LIMIT_ROLL 250.0*1.5
#define PWM_LIMIT_NEAR 450.0*1.5
#define PWM_LIMIT_MIDDLE 300.0*1.5
#define PWM_LIMIT_FAR 190.0*1.5

#define PWM_LIMIT_THUMB_ROLL 350.0*1.5
#define PWM_LIMIT_THUMB_NEAR 270.0*1.5
#define PWM_LIMIT_THUMB_MIDDLE 180.0*1.5
#define PWM_LIMIT_THUMB_FAR 180.0*1.5

#define PWM_LIMIT_GLOBAL_8V 800.0 // maximum: 1200
#define PWM_LIMIT_GLOBAL_24V 500.0
#define PWM_LIMIT_GLOBAL_12V 1200.0


namespace allegro
{

static int16_t degreesToCentiDegrees(double degrees)
{
    double value = degrees * 100.0;
    if (value > 32767.0) value = 32767.0;
    if (value < -32768.0) value = -32768.0;
    return static_cast<int16_t>(lround(value));
}

    const int AllegroHandDrv::FingertipSensorMap[4] = { 3, 7, 11, 15 };
    const int AllegroHandDrv::MadiSensorMap[11] = { 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};
    const int AllegroHandDrv::CurrentMap[16] = { 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3};


AllegroHandDrv::AllegroHandDrv()
    : _can_handle(0)
    , _curr_position_get(0)
    , _emergency_stop(false)
{
    RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"), "AllegroHandDrv instance is constructed.");
}

AllegroHandDrv::~AllegroHandDrv()
{
    if (_can_handle != 0) {
        RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"), "CAN: System Off");
        CANAPI::command_set_period(_can_handle, 0);
        usleep(10000);
        RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"), "CAN: Close CAN channel");
        CANAPI::command_can_close(_can_handle);
    }
}

// trim from end. see http://stackoverflow.com/a/217605/256798
static inline std::string &rtrim(std::string &s)
{
    s.erase(std::find_if(
        s.rbegin(), s.rend(),
        std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

bool AllegroHandDrv::init(std::string CAN_CH, int mode)
{
    // ros::param::get("~comm/CAN_CH", CAN_CH);
    rtrim(CAN_CH);  // Ensure the ROS parameter has no trailing whitespace.

    if (CAN_CH.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("allegro_hand_drv"), "Invalid (empty) CAN channel, cannot proceed. Check PCAN comms.");
        return false;
    }

    if (CANAPI::command_can_open_with_name(_can_handle, CAN_CH.c_str())) {
        _can_handle = 0;
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"), "CAN: Flush CAN receive buffer");
    CANAPI::command_can_flush(_can_handle);
    usleep(100);

    RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"), "CAN: System Off");
    CANAPI::command_servo_off(_can_handle);
    usleep(100);

    RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"), "CAN: Request Hand Information");
    CANAPI::request_hand_information(_can_handle);
    usleep(100);

    RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"), "CAN: Request Hand Serial");
    CANAPI::request_hand_serial(_can_handle);
    usleep(100);

    RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"), "CAN: Setting loop period(:= 3ms) and initialize system");
    short comm_period[3] = {3, 0, 0}; // millisecond {position, imu, temperature}
    CANAPI::command_set_period(_can_handle, comm_period);

    RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"), "CAN: System ON");
    CANAPI::command_servo_on(_can_handle);
    usleep(100);

    RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"), "CAN: Communicating");

    return true;
}

void AllegroHandDrv::calibrate()
{
    RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"),
        "CAN: Sending position calibration command");
    CANAPI::command_calibration(_can_handle);
}

int AllegroHandDrv::readFrames()
{
    if (_emergency_stop)
        return -1;

    _readDevices();

    return 0;
}

int AllegroHandDrv::writeJointTorque()
{
    _writeDevices();

    if (_emergency_stop) {
        RCLCPP_ERROR(rclcpp::get_logger("allegro_hand_drv"), "Emergency stop in writeJointTorque()");
        return -1;
    }

    return 0;
}

bool AllegroHandDrv::isJointInfoReady()
{
    return (_curr_position_get == (0x01 | 0x02 | 0x04 | 0x08));// | 0x10 | 0x20));
}

void AllegroHandDrv::resetJointInfoReady()
{
    _curr_position_get = 0;
}

void AllegroHandDrv::setTorque(double *torque)
{
        for (int findex = 0; findex < 4; findex++) {
            _desired_torque[4 * findex + 0] = torque[4 * findex + 0] * 1.43 * 1000;
            _desired_torque[4 * findex + 1] = torque[4 * findex + 1] * 1.43 * 1000;
            _desired_torque[4 * findex + 2] = torque[4 * findex + 2] * 1.43 * 1000;
            _desired_torque[4 * findex + 3] = torque[4 * findex + 3] * 1.43 * 1000;
        }

        _desired_torque[1] = 0.5 * _desired_torque[1];
        _desired_torque[5] = 0.5 * _desired_torque[5];
        _desired_torque[9] = 0.5 * _desired_torque[9];
        
}

void AllegroHandDrv::getJointInfo(double *position)
{
    for (int i = 0; i < DOF_JOINTS; i++) {
        position[i] = _curr_position[i];
    }
}

// Send desired joint angles directly to the hand firmware (H-inf onboard control mode).
// q_rad[16]: desired angles in radians.
// Converts to encoder counts using the same scale the firmware uses:
//   count = round( q_rad / (pi/180 * 0.088) )  == round( q_deg / 0.088 )
// Packs 4 joints per finger and sends via command_set_pose_direct
// (raw CAN wire IDs 0x180/0x184/0x188/0x18C — index/middle/little/thumb).
void AllegroHandDrv::sendPositionDirect(const double* q_rad)
{

    static const double RAD_TO_COUNT = 180.0 / (M_PI * 0.088); // rad -> encoder count
    for (int findex = 0; findex < 4; findex++) {
        short jpos[4];
        for (int j = 0; j < 4; j++) {
            double cnt = q_rad[findex * 4 + j] * RAD_TO_COUNT;
            if (cnt >  32767.0) cnt =  32767.0;
            if (cnt < -32768.0) cnt = -32768.0;
            jpos[j] = static_cast<short>(cnt);
        }
        CANAPI::command_set_pose_direct(_can_handle, findex, jpos);
    }
}

void AllegroHandDrv::setHinfMode(bool enable)
{
    unsigned char data[8] = {0};
    data[0] = enable ? 1 : 0;
    
    // Send 3 times with a short delay to ensure robust reception during startup.
    for (int i = 0; i < 3; i++) {
        int ret = CANAPI::can_write_message_raw(_can_handle, ID_CMD_HINF_MODE, 8, data, TRUE, 0);
        if (ret != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("allegro_hand_drv"),
                         "CAN: Failed to send H-inf mode command! (ret=%d, id=0x%03X)", ret, ID_CMD_HINF_MODE);
        }
        usleep(2000); 
    }

    RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"),
                "CAN: H-inf mode %s (sent 3x to 0x%03X)", enable ? "ON" : "OFF", ID_CMD_HINF_MODE);
}

void AllegroHandDrv::sendGravityFeedForwardRPY(double roll_deg,
                                               double pitch_deg,
                                               double yaw_deg)
{
    const int16_t rpy_cdeg[3] = {
        degreesToCentiDegrees(roll_deg),
        degreesToCentiDegrees(pitch_deg),
        degreesToCentiDegrees(yaw_deg)
    };
    unsigned char data[8] = {0};

    for (int i = 0; i < 3; i++) {
        data[2 * i] = static_cast<unsigned char>(rpy_cdeg[i] & 0x00ff);
        data[2 * i + 1] = static_cast<unsigned char>((rpy_cdeg[i] >> 8) & 0x00ff);
    }

    int ret = CANAPI::can_write_message_raw(_can_handle, ID_CMD_GFF_RPY, 8, data, TRUE, 0);
    if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("allegro_hand_drv"),
                     "CAN: Failed to send GFF RPY command! (ret=%d, id=0x%03X)",
                     ret, ID_CMD_GFF_RPY);
    }
}

void AllegroHandDrv::_readDevices()
{
    int err;
    int id;    
    int len;
    unsigned char data[8];

    err = CANAPI::can_read_message(_can_handle, &id, &len, data, FALSE, 0);
    while (!err) {
        _parseMessage(id, len, data);
        err = CANAPI::can_read_message(_can_handle, &id, &len, data, FALSE, 0);
    }
}

void AllegroHandDrv::_writeDevices()
{
    double pwmDouble[DOF_JOINTS];
    short pwm[DOF_JOINTS];

    if (!isJointInfoReady())
        return;

    // convert to torque to pwm
    for (int i = 0; i < DOF_JOINTS; i++) {
    if (_desired_torque[i] > 240) _desired_torque[i] = 240;
	else if (_desired_torque[i] < -240) _desired_torque[i] = -240;
   
        pwmDouble[i] = _desired_torque[i];

        pwm[i] = (short) pwmDouble[i];
    }

    for (int findex = 0; findex < 4; findex++) {
        CANAPI::command_set_torque(_can_handle, findex, &pwm[findex*4]);
       
    }
}

void AllegroHandDrv::_parseMessage(int id, int len, unsigned char* data)
{
    int tmppos[4];
    int lIndexBase;
 
    switch (id) 
    {
        case ID_RTR_HAND_INFO:
        {
            RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"),">CAN(%x): AllegroHand hardware version: 0x%02x%02x\n", _can_handle, data[1], data[0]);
            RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"),"                      firmware version: 0x%02x%02x\n", data[3], data[2]);
            RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"),"                      servo status: %s\n", (data[6] & 0x01 ? "ON" : "OFF"));

        }
            break;
        case ID_RTR_SERIAL:
        {
            RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"),">CAN(%d): AllegroHand serial number: %c%c%c%c%c%c%c%c\n", _can_handle,
                     data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
            RCLCPP_INFO(rclcpp::get_logger("allegro_hand_drv"),"                      hardware type: %s\n", (data[3] == 'R' ? "right" : "left"));

        if(data[3] == 'R') RIGHT_HAND = true;
        else RIGHT_HAND  = false;
        }
        break;

        case ID_RTR_FINGER_CURRENT_1:
        case ID_RTR_FINGER_CURRENT_2:
        case ID_RTR_FINGER_CURRENT_3:
        case ID_RTR_FINGER_CURRENT_4:
        {
            int findex = (id & 0x00000007);

            // Device sends 2 compressed frames covering all 16 joints:
            //   findex=0 (id=0x00) → fingerA=0, fingerB=1 → joints  0-7
            //   findex=3 (id=0x03) → fingerA=2, fingerB=3 → joints 8-15
            // Map findex to the actual finger pair index (not findex*2)
            static const int kCurrentFingerPairMap[4] = {0, -1, -1, 2};
            int fingerPair = kCurrentFingerPairMap[findex];
            if (fingerPair < 0) break;  // unexpected findex, skip

            int fingerA = fingerPair;
            int fingerB = fingerPair + 1;

            // Compressed mode: 8x int8, two fingers per frame
            // scale decode: mA ~= round(i8 * 240 / 127)
            for (int i = 0; i < 4; i++) {
                int8_t q = (int8_t)data[i];
                _curr_motor_current[fingerA * 4 + i] = (short)((q * 240 + (q >= 0 ? 63 : -63)) / 127);
            }
            for (int i = 0; i < 4; i++) {
                int8_t q = (int8_t)data[4 + i];
                _curr_motor_current[fingerB * 4 + i] = (short)((q * 240 + (q >= 0 ? 63 : -63)) / 127);
            }

            _curr_current_get |= (0x01 << fingerA);
            _curr_current_get |= (0x01 << fingerB);

            if (_curr_current_get == (0x01 | 0x02 | 0x04 | 0x08))
            {
                // Apply sensor mapping rules
                for (int i = 0; i < 16; i++)
                    motor_current[CurrentMap[i]] = _curr_motor_current[i];

                _curr_current_get = 0;
            }
        }
        break;
        case ID_RTR_FINGER_POSE_1:
        case ID_RTR_FINGER_POSE_2:
        case ID_RTR_FINGER_POSE_3:
        case ID_RTR_FINGER_POSE_4:
        {
            int findex = (id & 0x00000007);

            tmppos[0] = (short) (data[0] | (data[1] << 8));
            tmppos[1] = (short) (data[2] | (data[3] << 8));
            tmppos[2] = (short) (data[4] | (data[5] << 8));
            tmppos[3] = (short) (data[6] | (data[7] << 8));

            lIndexBase = findex * 4;

            _curr_position[lIndexBase+0] = (double)(tmppos[0]) * (M_PI / 180.0) * 0.088;
            _curr_position[lIndexBase+1] = (double)(tmppos[1]) * (M_PI / 180.0) * 0.088;
            _curr_position[lIndexBase+2] = (double)(tmppos[2]) * (M_PI / 180.0) * 0.088;
            _curr_position[lIndexBase+3] = (double)(tmppos[3]) * (M_PI / 180.0) * 0.088;

            _curr_position_get |= (0x01 << (findex));
        }
        break;

        case ID_CMD_FINGERTIP_1:
        case ID_CMD_FINGERTIP_2:
        case ID_CMD_FINGERTIP_3:
        case ID_CMD_FINGERTIP_4:
        {
            int findex = (id & 0x00000007) - 4;
            float alpha = 0.25f;

            if (findex <= 1)
            {
                // Compressed mode: 8x uint8, two fingers per frame
                int fingerA = findex * 2;
                int fingerB = fingerA + 1;

                for (int i = 0; i < 4; i++) {
                    int idx = fingerA * 4 + i;
                    int raw_val = ((int)data[i]) << 2;
                    if (data[i] == 0xFF || raw_val < 0) raw_val = 0;
                    _sensor_buf[idx] = alpha * raw_val + (1 - alpha) * _sensor_buf[idx];
                }
                for (int i = 0; i < 4; i++) {
                    int idx = fingerB * 4 + i;
                    int raw_val = ((int)data[4 + i]) << 2;
                    if (data[4 + i] == 0xFF || raw_val < 0) raw_val = 0;
                    _sensor_buf[idx] = alpha * raw_val + (1 - alpha) * _sensor_buf[idx];
                }
            }
            else
            {
                // Legacy mode: 4x int16 big-endian per frame
                for (int i = 0; i < 4; i++) {
                    int idx = findex * 4 + i;
                    int raw_val = (data[2 * i] << 8) | data[2 * i + 1];
                    if (raw_val < 0) raw_val = 0;
                    _sensor_buf[idx] = alpha * raw_val + (1 - alpha) * _sensor_buf[idx];
                }
            }

            // Map to global sensor variables
            for (int i = 0; i < 4; i++)
                fingertip_sensor[i] = _sensor_buf[FingertipSensorMap[i]] / 10.0f;
            for (int i = 0; i < 11; i++)
                madi_sensor[i] = _sensor_buf[MadiSensorMap[i]] / 10.0f;
            palm_sensor = _sensor_buf[0];

        }
        break;
        default:
            RCLCPP_WARN(rclcpp::get_logger("allegro_hand_drv"), "unknown command %d, len %d", id, len);
            return;
    }
}

} // namespace allegro
