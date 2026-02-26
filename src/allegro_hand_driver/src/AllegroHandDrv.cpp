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

int AllegroHandDrv::readCANFrames()
{
    if (_emergency_stop)
        return -1;

    _readDevices();
    //usleep(10);

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
    return (_curr_position_get == (0x01 | 0x02 | 0x04 | 0x08));
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
    //ROS_ERROR("can_read_message returns %d.", err); // PCAN_ERROR_QRCVEMPTY(32) from Peak CAN means "Receive queue is empty". It is not an error.
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
    int i;
    int tmpcur[16];
 
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

            tmpcur[findex * 4 + 0] = (short) (data[0] | (data[1] << 8));
            tmpcur[findex * 4 + 1] = (short) (data[2] | (data[3] << 8));
            tmpcur[findex * 4 + 2] = (short) (data[4] | (data[5] << 8));
            tmpcur[findex * 4 + 3] = (short) (data[6] | (data[7] << 8));

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
            int16_t fingertip1 = (int16_t)(data[6] << 8 | data[7]);

            int16_t sensor1 = (int16_t)(data[0] << 8 | data[1]);
            int16_t sensor2 = (int16_t)(data[2] << 8 | data[3]);
            int16_t sensor3 = (int16_t)(data[4] << 8 | data[5]);

            // Raw CAN value: last digit = 0.1 place, rest = integer; store as value/10.0
            fingertip_sensor[findex] = fingertip1 / 10.0f;

            float alpha = 0.25;

            if(findex == 0)
            {
                palm_sensor = sensor1 / 10.0f;
                madi_sensor[0] = sensor2 / 10.0f;
                madi_sensor[1] = sensor3 / 10.0f;

                if(palm_sensor < 0)
                    palm_sensor = 0;
                palm_sensor = alpha * palm_sensor + (1 - alpha) * palm_sensor_pre;
                palm_sensor_pre = palm_sensor;
                for(int j=0; j<2; j++)
                 {
                    if(madi_sensor[j] < 0)
                        madi_sensor[j] = 0;
                    madi_sensor[j] = alpha * madi_sensor[j] + (1 - alpha) * madi_sensor_pre[j];
                    madi_sensor_pre[j] = madi_sensor[j];
                 }
            }
            else
            {
                madi_sensor[(findex-1)*3 + 2] = sensor1 / 10.0f;
                madi_sensor[(findex-1)*3 + 3] = sensor2 / 10.0f;
                madi_sensor[(findex-1)*3 + 4] = sensor3 / 10.0f;
                for(int j=0; j<3; j++)
                 {
                    int idx = (findex-1)*3 + 2 + j;
                    if(madi_sensor[idx] < 0)
                        madi_sensor[idx] = 0;
                    madi_sensor[idx] = alpha * madi_sensor[idx] + (1 - alpha) * madi_sensor_pre[idx];
                    madi_sensor_pre[idx] = madi_sensor[idx];
                 }
            }

            if (fingertip_sensor[findex] < 0)
                fingertip_sensor[findex] = 0;

            fingertip_sensor[findex] = alpha * fingertip_sensor[findex] + (1 - alpha) * fingertip_sensor_pre[findex];
            fingertip_sensor_pre[findex] = fingertip_sensor[findex];
        }
        break;
        default:
            RCLCPP_WARN(rclcpp::get_logger("allegro_hand_drv"), "unknown command %d, len %d", id, len);
            for(int nd=0; nd<len; nd++)
                printf("%d \n ", data[nd]);
            return;
    }
}

void AllegroHandDrv::sendPickCommand()
{
    command_pick(_can_handle);
}

void AllegroHandDrv::sendPlaceCommand()
{
    command_place(_can_handle);
}

} // namespace allegro
