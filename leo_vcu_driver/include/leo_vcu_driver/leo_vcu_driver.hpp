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
//
// Co-developed by Tier IV, Inc. Apex.AI, Inc. and Leo Drive Teknoloji A.Ş.

/// \copyright Copyright 2022 Leo Drive Teknoloji A.Ş.
/// \file
/// \brief This file defines the leo_vcu_driver class.

#ifndef LEO_VCU_DRIVER__LEO_VCU_DRIVER_HPP_
#define LEO_VCU_DRIVER__LEO_VCU_DRIVER_HPP_

#include <leo_vcu_driver/visibility_control.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>
#include <vector>
#include <string>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>


#include <std_msgs/msg/string.hpp>

#include <leo_vcu_driver/AsyncSerial.h>
#include <leo_vcu_driver/checksum.h>
#include <leo_vcu_driver/vehicle_interface.h>
#include <experimental/optional>
#include <rclcpp/rclcpp.hpp>


class LeoVcuDriver : public rclcpp::Node
{
public:
  LeoVcuDriver();
  ~LeoVcuDriver() override
  {
    serial.close();
  }

/**
 * @brief It is callback function which takes data from "/control/command/control_cmd" topic in Autoware Universe.
 */
  void ctrl_cmd_callback(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
/**
 * @brief It is callback function which takes data from "/control/command/turn_indicators_cmd" topic in Autoware Universe.
 */
  void turn_indicators_cmd_callback(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
/**
 * @brief It is callback function which takes data from "/control/command/hazard_lights_cmd" topic from Autoware Universe.
 */
  void hazard_lights_cmd_callback(
    const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
/**
 * @brief It is callback function which takes data from "/control/command/gear_cmd" topic from Autoware Universe.
 */
  void gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
/**
 * @brief It is callback function which takes data from "/vehicle/engage" topic from Autoware Universe.
 */
  void engage_cmd_callback(const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg);
/**
 * @brief It is callback function which takes data from "/control/command/emergency_cmd" topic from Autoware Universe.
 */
  void emergency_cmd_callback(
    const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);
/**
 * @brief It is callback function which takes data from "/control/current_gate_mode" topic from Autoware Universe.
 */
  void gate_mode_cmd_callback(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg);
/**
 * @brief It sends data from interface to low level controller.
 */
  void llc_publisher();
/**
 * @brief It takes data from interface to low level controller.
 */
  void serial_receive_callback(const char * data, unsigned int len);
/**
 * @brief It converts the steering angle to steering wheel angle.
 * Steering angle means "Teker açısı" and which is radian.
 * Steering wheel angle means "Direksiyon açısı" and which is degree.
 */
  float steering_tire_to_steering_wheel_angle(float input);
/**
 * @brief It converts the steering wheel angle to steering angle.
 * Steering angle means "Teker açısı" and which is radian.
 * Steering wheel angle means "Direksiyon açısı" and which is degree.
 */
  float steering_wheel_to_steering_tire_angle(float & input);
/**
 * @brief It converts the gear data which is taken from LLC wrt Autoware Universe messages.
 */
  uint8_t gear_adapter_to_autoware(uint8_t & input);
/**
 * @brief It converts the gear data which is taken from autoware universe wrt LLC messages.
 */
  void gear_adapter_to_llc(const uint8_t & input);
/**
 * @brief It converts the control mode data which is taken from LLC wrt Autoware Universe messages.
 */
  uint8_t control_mode_adapter_to_autoware(uint8_t & input);
/**
 * @brief It converts the control mode data which is taken from autoware universe wrt LLC messages.
 */
  void control_mode_adapter_to_llc();
/**
 * @brief It converts the indicator data which is taken from LLC wrt Autoware Universe messages.
 */
  void indicator_adapter_to_autoware(uint8_t & input);
/**
 * @brief It converts the indicator data which is taken from autoware universe wrt LLC messages.
 */
  void indicator_adapter_to_llc();
/**
 * @brief It is the meta converter function that takes all data from LLC and convert them to Autoware messages
 * which are defined as global variable.
 */
  void llc_to_autoware_msg_adapter(std::experimental::optional<LlcToCompData> & received_data);
/**
 * @brief It is the meta converter function that takes all data from Autoware and convert them to LLC Data Structure
 * which are defined as global variable.
 */
  void autoware_to_llc_msg_adapter();

private:
  std::experimental::optional<LlcToCompData> find_llc_to_comp_msg(
    const char * data,
    unsigned int len);
  static std::vector<char> pack_serial_data(const CompToLlcData_ & data);

  std::vector<uint8_t> receive_buffer_;


  /* input values */

  // From Autoware

  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr turn_indicators_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr hazard_lights_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;
  tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr emergency_cmd_ptr;
  tier4_control_msgs::msg::GateMode::ConstSharedPtr gate_mode_cmd_ptr;
  bool engage_cmd_{0};

  /* Variables */
  rclcpp::Time control_command_received_time_;

  // Current state of vehicle (Got from LLC)

  vehicle_current_state_ current_state;

  // To LLC

  CompToLlcData_ send_data;
  // const std::string serial_name_; // I think it is unnecessary
  CallbackAsyncSerial serial;

  /* subscribers */

  // From Autoware
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    turn_indicators_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    hazard_lights_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engage_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_sub_;
  rclcpp::Subscription<tier4_control_msgs::msg::GateMode>::ConstSharedPtr gate_mode_sub_;

  /* publishers */

  // To Autoware
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_indicators_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
    hazard_lights_status_pub_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>::SharedPtr
    steering_wheel_status_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr tim_data_sender_;

  /* ros params */
  vehicle_info_util::VehicleInfo vehicle_info_;

  std::string base_frame_id_;
  double loop_rate_{};                   // [Hz]
  float wheel_base_{};                  // [m]
  double command_timeout_ms_{};    // vehicle_cmd timeout [ms]
  bool reverse_gear_enabled_{false};   // reverse gear enabled or not
  float emergency_stop_acceleration{};   // [m/s^2]
  float gear_shift_velocity_threshold{};   // [m/s]
  float max_steering_wheel_angle{}; // [degree]
  float min_steering_wheel_angle{}; // [degree]
  float max_steering_wheel_angle_rate{}; // [degree/sec]
  bool check_steering_angle_rate{};
};
#endif  // LEO_VCU_DRIVER__LEO_VCU_DRIVER_HPP_
