// Copyright 2022 Leo Drive Teknoloji A.Ş.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2022 Leo Drive Teknoloji A.Ş.
/// \file
/// \brief This file defines the vehicle_interface class.
#ifndef BUILD_VEHICLE_INTERFACE_H
#define BUILD_VEHICLE_INTERFACE_H

#include "checksum.h"

#include <cstdint>

using namespace std;

const uint8_t comp_to_llc_msg_frame_id{16U};
const uint8_t comp_to_llc_msg_eof_id{17U};

const uint8_t llc_to_comp_msg_frame_id{99U};
const uint8_t llc_to_comp_msg_eof_id{100U};

// added for universe

struct DoorStatus_
{
  uint8_t front_door : 4;
  uint8_t rear_door : 4;
};
struct vehicle_current_state_
{
  autoware_auto_vehicle_msgs::msg::VelocityReport twist;
  tier4_vehicle_msgs::msg::SteeringWheelStatusStamped steering_wheel_status_msg;
  autoware_auto_vehicle_msgs::msg::SteeringReport steering_tire_status_msg;
  autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_report;
  autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
  autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_msg;
  tier4_api_msgs::msg::DoorStatus door_status_msg;
  char * debug_str_last{};
};
struct CompToLlcData_
{
  uint32_t counter_ = {0};
  float set_long_accel_mps2_{};
  float set_limit_velocity_mps_{};
  float set_front_wheel_angle_rad_{};
  DoorStatus_ door_;
  uint8_t blinker_{};
  uint8_t headlight_{1};
  uint8_t wiper_{1};
  uint8_t gear_{};
  uint8_t mode_{};
  uint8_t hand_brake{0};
  uint8_t takeover_request{0};  // no takeover for 0, takeover for 1
  // Not used data: wiper, headlight, handbrake
  // Default values wiper = 1 (Off), headlight = 1 (Off), handbrake = 0 (not used in LLC)
};

struct StateReport_
{
  uint8_t fuel;
  uint8_t blinker;
  uint8_t headlight;
  uint8_t wiper;
  uint8_t gear;
  uint8_t mode;
  uint8_t hand_brake;
  uint8_t takeover_request;
  uint8_t intervention;
  uint8_t ready;
  uint8_t motion_allow;
  uint8_t throttle;  // %
  uint8_t brake;     // %
  DoorStatus_ door_status;
  uint8_t rsv;
  char debugstr[24];
};

struct VehicleOdometry_
{
  float velocity_mps;
  float front_wheel_angle_rad;
};

struct LlcToCompData
{
  uint8_t frame_id1;
  uint8_t frame_id2;
  uint16_t errors;
  uint32_t counter;
  VehicleOdometry_ vehicle_odometry;
  StateReport_ state_report;
  uint16_t crc;
  uint8_t eof_id1;
  uint8_t eof_id2;
};

struct VehicleControlCommand_
{
  float set_long_accel_mps2;
  float set_limit_velocity_mps;
  float set_front_wheel_angle_rad;
};

struct VehicleStateCommand_
{
  uint8_t blinker;
  uint8_t headlight;
  uint8_t wiper;
  uint8_t gear;
  uint8_t mode;
  uint8_t hand_brake;        // bool
  uint8_t takeover_request;  // bool
  DoorStatus_ door;
};

struct CompToLlcData
{
  explicit CompToLlcData(
    uint32_t counter_, float set_long_accel_mps2_, float set_limit_velocity_mps_,
    float set_front_wheel_angle_rad_, DoorStatus_ door_, uint8_t blinker_, uint8_t headlight_,
    uint8_t wiper_, uint8_t gear_, uint8_t mode_, uint8_t hand_brake, uint8_t takeover_request)
  :

    frame_id1{comp_to_llc_msg_frame_id},
    frame_id2{comp_to_llc_msg_frame_id},
    reserved{0},
    counter{counter_},
    vehicle_control_cmd{set_long_accel_mps2_, set_limit_velocity_mps_, set_front_wheel_angle_rad_},
    vehicle_state_cmd{blinker_, headlight_, wiper_,           gear_,
                      mode_,    hand_brake, takeover_request, door_},
    crc{0},
    eof_id1{comp_to_llc_msg_eof_id},
    eof_id2{comp_to_llc_msg_eof_id}
  {
    crc = crc_16(reinterpret_cast<const uint8_t *>(this), sizeof(CompToLlcData) - 4);
  }

  uint8_t frame_id1;
  uint8_t frame_id2;
  uint16_t reserved;
  uint32_t counter;
  VehicleControlCommand_ vehicle_control_cmd;
  VehicleStateCommand_ vehicle_state_cmd;
  uint16_t crc;
  uint8_t eof_id1;
  uint8_t eof_id2;
};

#endif  // BUILD_VEHICLE_INTERFACE_H
