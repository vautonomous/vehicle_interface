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


#include <leo_vcu_driver/leo_vcu_driver.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

LeoVcuDriver::LeoVcuDriver()
: Node("leo_vcu_driver"),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  /// Get Params Here!

  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000.0);
  loop_rate_ = declare_parameter("loop_rate", 100.0);

  /* parameters for vehicle specifications */
  wheel_base_ = static_cast<float>(vehicle_info_.wheel_base_m);
  reverse_gear_enabled_ = declare_parameter("reverse_gear_enabled", false);
  emergency_stop_acceleration =
    static_cast<float>(declare_parameter("emergency_stop_acceleration", -5.0));
  gear_shift_velocity_threshold =
    static_cast<float>(declare_parameter("gear_shift_velocity_threshold", 0.1));
  max_steering_wheel_angle =
    static_cast<float>(declare_parameter("max_steering_wheel_angle", 750.0));
  min_steering_wheel_angle = static_cast<float>(declare_parameter(
      "min_steering_wheel_angle",
      -750.0));
  max_steering_wheel_angle_rate =
    static_cast<float>(declare_parameter("max_steering_wheel_angle_rate", 300.0));
  check_steering_angle_rate = declare_parameter("check_steering_angle_rate", true);

  /* Subscribers */

  using namespace std::placeholders;

  // From Autoware
  control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1, std::bind(&LeoVcuDriver::ctrl_cmd_callback, this, _1));
  gear_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&LeoVcuDriver::gear_cmd_callback, this, _1));
  turn_indicators_cmd_sub_ =
    create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
    "/control/command/turn_indicators_cmd", rclcpp::QoS{1},
    std::bind(&LeoVcuDriver::turn_indicators_cmd_callback, this, _1));
  hazard_lights_cmd_sub_ =
    create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
    "/control/command/hazard_lights_cmd", rclcpp::QoS{1},
    std::bind(&LeoVcuDriver::hazard_lights_cmd_callback, this, _1));
  engage_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::Engage>(
    "/vehicle/engage", rclcpp::QoS{1}, std::bind(&LeoVcuDriver::engage_cmd_callback, this, _1));
  gate_mode_sub_ = create_subscription<tier4_control_msgs::msg::GateMode>(
    "/control/current_gate_mode", 1,
    std::bind(&LeoVcuDriver::gate_mode_cmd_callback, this, _1));
  emergency_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1,
    std::bind(&LeoVcuDriver::emergency_cmd_callback, this, _1));

  /* publisher */

  // To Autoware
  control_mode_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  vehicle_twist_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
  steering_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});
  gear_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});
  turn_indicators_status_pub_ =
    create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
    "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});
  hazard_lights_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
  steering_wheel_status_pub_ =
    create_publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>(
    "/vehicle/status/steering_wheel_status", 1);

  // Timer
  const auto period_ns = rclcpp::Rate(loop_rate_).period();
  tim_data_sender_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&LeoVcuDriver::llc_publisher, this));

  // From LLC
  serial.setCallback(bind(&LeoVcuDriver::serial_receive_callback, this, _1, _2));
}

void LeoVcuDriver::ctrl_cmd_callback(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  control_command_received_time_ = this->now();
  control_cmd_ptr_ = msg;
}

void LeoVcuDriver::emergency_cmd_callback(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  emergency_cmd_ptr = msg;
}

void LeoVcuDriver::gear_cmd_callback(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  gear_cmd_ptr_ = msg;
}

void LeoVcuDriver::turn_indicators_cmd_callback(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  turn_indicators_cmd_ptr_ = msg;
}

void LeoVcuDriver::hazard_lights_cmd_callback(
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_lights_cmd_ptr_ = msg;
}

void LeoVcuDriver::engage_cmd_callback(
  const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
{
  engage_cmd_ = msg->engage;
}

void LeoVcuDriver::gate_mode_cmd_callback(
  const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  gate_mode_cmd_ptr = msg;   // AUTO = 0, EXTERNAL = 1
}

void LeoVcuDriver::serial_receive_callback(const char * data, unsigned int len)
{
  auto received_data{find_llc_to_comp_msg(data, len)};
  if (!received_data) {
    RCLCPP_INFO(this->get_logger(), "Received data is not viable!\n");
    return;
  }
  if (received_data->state_report.debugstr != current_state.debug_str_last) {
    RCLCPP_INFO(this->get_logger(), "%s", current_state.debug_str_last);
  }

  // Message adapter
  llc_to_autoware_msg_adapter(received_data);

  std_msgs::msg::Header header;
  header.frame_id = this->base_frame_id_;
  header.stamp = get_clock()->now();

  /* publish current steering tire status */
  {
    current_state.steering_tire_status_msg.stamp = header.stamp;
    steering_status_pub_->publish(current_state.steering_tire_status_msg);
  }

  /* publish steering wheel status */
  {
    current_state.steering_wheel_status_msg.stamp = header.stamp;
    steering_wheel_status_pub_->publish(current_state.steering_wheel_status_msg);
  }

  /* publish vehicle status control_mode */
  {
    current_state.control_mode_report.stamp = header.stamp;
    control_mode_pub_->publish(current_state.control_mode_report);
  }

  /* publish vehicle status twist */
  {
    current_state.twist.header = header;
    vehicle_twist_pub_->publish(current_state.twist);
  }

  /* publish current shift */
  {
    current_state.gear_report_msg.stamp = header.stamp;
    gear_status_pub_->publish(current_state.gear_report_msg);
  }
  /* publish current hazard and turn signal */
  {
    current_state.turn_msg.stamp = header.stamp;
    turn_indicators_status_pub_->publish(current_state.turn_msg);

    current_state.hazard_msg.stamp = header.stamp;
    hazard_lights_status_pub_->publish(current_state.hazard_msg);
  }
}

void LeoVcuDriver::llc_to_autoware_msg_adapter(
  std::experimental::optional<LlcToCompData> & received_data)
{
  current_state.steering_wheel_status_msg.data =
    received_data->vehicle_odometry.front_wheel_angle_rad;  // Steering wheel angle fb from LLC
  current_state.twist.longitudinal_velocity = received_data->vehicle_odometry.velocity_mps;
  current_state.steering_tire_status_msg.steering_tire_angle =
    steering_wheel_to_steering_tire_angle(
    current_state.steering_wheel_status_msg.data);    // Calculates current_steering_tire_angle
  current_state.control_mode_report.mode = control_mode_adapter_to_autoware(
    received_data->state_report.mode);
  current_state.twist.heading_rate = current_state.twist.longitudinal_velocity * std::tan(
    current_state.steering_tire_status_msg.steering_tire_angle) / wheel_base_;  // [rad/s]
  current_state.gear_report_msg.report = gear_adapter_to_autoware(received_data->state_report.gear);
  indicator_adapter_to_autoware(received_data->state_report.blinker);
  current_state.debug_str_last = received_data->state_report.debugstr;
}

void LeoVcuDriver::autoware_to_llc_msg_adapter()
{
  if (current_state.gear_report_msg.report != gear_cmd_ptr_->command) {
    // velocity is low -> the shift can be changed
    if (std::fabs(current_state.twist.longitudinal_velocity) < gear_shift_velocity_threshold) {
      // TODO(berkay): check here again!
      gear_adapter_to_llc(gear_cmd_ptr_->command);
    } else {
      RCLCPP_WARN(
        get_logger(), "Gear change is not allowed, current_velocity = %f",
        static_cast<double>(current_state.twist.longitudinal_velocity));
    }
  }
  indicator_adapter_to_llc();
  send_data.set_long_accel_mps2_ = control_cmd_ptr_->longitudinal.acceleration;
  send_data.set_front_wheel_angle_rad_ =
    steering_tire_to_steering_wheel_angle(control_cmd_ptr_->lateral.steering_tire_angle);
  send_data.set_front_wheel_angle_rate_ = steering_tire_to_steering_wheel_angle(
    control_cmd_ptr_->lateral.steering_tire_angle +
    control_cmd_ptr_->lateral.steering_tire_rotation_rate) -
    send_data.set_front_wheel_angle_rad_;
  control_mode_adapter_to_llc();
}

void LeoVcuDriver::control_mode_adapter_to_llc()
{
/* send mode */

  if (!engage_cmd_) {
    send_data.mode_ = 3;     // DISENGAGED. IT IS PRIOR
  } else if (gate_mode_cmd_ptr->data == tier4_control_msgs::msg::GateMode::AUTO) {
    send_data.mode_ = 1;
  } else if (gate_mode_cmd_ptr->data == tier4_control_msgs::msg::GateMode::EXTERNAL) {
    send_data.mode_ = 2;     // MANUAL
  } else {
    send_data.mode_ = 4;     // NOT READY
  }
}


uint8_t LeoVcuDriver::control_mode_adapter_to_autoware(uint8_t & input)
{
  if (input == 3) {
    return autoware_auto_vehicle_msgs::msg::ControlModeReport::DISENGAGED;
  } else if (input == 1) {
    return autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  } else if (input == 2) {
    return autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
  } else if (input == 4) {
    return autoware_auto_vehicle_msgs::msg::ControlModeReport::NOT_READY;
  } else {
    return autoware_auto_vehicle_msgs::msg::ControlModeReport::NO_COMMAND;
  }
}

void LeoVcuDriver::indicator_adapter_to_autoware(uint8_t & input)
{
  if (input == 4) {
    current_state.hazard_msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE;
    current_state.turn_msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
  } else {
    current_state.hazard_msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::DISABLE;
    current_state.turn_msg.report = input;
  }
}

std::experimental::optional<LlcToCompData> LeoVcuDriver::find_llc_to_comp_msg(
  const char * data,
  unsigned int len)
{
  receive_buffer_.insert(receive_buffer_.end(), data, data + len);

  // Look data size for checking there is enough data to contain a data structure
  if (receive_buffer_.size() < sizeof(LlcToCompData)) {
    return std::experimental::nullopt;
  }

  size_t search_range = receive_buffer_.size() - sizeof(LlcToCompData) + 1;
  for (size_t i = 0; i < search_range; i++) {
    // Look for frame ids and end-of-frames
    const auto llc_data = reinterpret_cast<LlcToCompData *>(receive_buffer_.data() + i);
    if (llc_data->frame_id1 == llc_to_comp_msg_frame_id &&
      llc_data->frame_id2 == llc_to_comp_msg_frame_id &&
      llc_data->eof_id1 == llc_to_comp_msg_eof_id &&
      llc_data->eof_id2 == llc_to_comp_msg_eof_id)
    {
      const uint16_t crc = crc_16(receive_buffer_.data() + i, sizeof(LlcToCompData) - 4);
      if (crc == llc_data->crc) {
        LlcToCompData valid_data{*llc_data};
        receive_buffer_.erase(
          receive_buffer_.begin(),
          receive_buffer_.begin() +
          static_cast<long>(i) +
          sizeof(LlcToCompData));
        return std::experimental::make_optional(valid_data);
      }
    }
  }
  return std::experimental::nullopt;
}

std::vector<char> LeoVcuDriver::pack_serial_data(const CompToLlcData_ & data)
{
  const auto ptr{reinterpret_cast<const uint8_t *>(&data)};
  std::vector<char> dataVec(ptr, ptr + sizeof data);
  return dataVec;
}

float LeoVcuDriver::steering_tire_to_steering_wheel_angle(float input)   // rad input degree output, maybe constants needs re-calculation
{   // TODO: If input or output is out of boundry, what we will do?
  const long double a0 = 31.199461665454701533044340519L;
  const long double a1 = 188.36170978503590077938134L;
  const long double a2 = 366.1782284667889888443542437L;
  const long double a3 = -33.453398194869886816L;
  const long double a4 = 832.269397717359360226L;
  const long double a5 = 0.371560022468346L;
//    const long double low_input_boundary = -0.7494655984494486291955L;
//    const long double high_input_boundary = 0.69883410378631689642371L;
//    const double low_output_boundary = -750;
//    const double high_output_boundary = 750;


  auto output = static_cast<float>(a0 * (pow(static_cast<long double>(input), 5)) +
    a1 * (pow(static_cast<long double>(input), 4)) +
    a2 * (pow(static_cast<long double>(input), 3)) +
    a3 * (pow(static_cast<long double>(input), 2)) +
    a4 * static_cast<long double>(input) + a5);
  return output;
}

float LeoVcuDriver::steering_wheel_to_steering_tire_angle(float & input)   // degree input rad output, maybe constants needs re-calculation
{   // TODO: If input or output is out of boundry, what we will do?
  const long double a0 = 0.0000000000000003633366321943L;
  const long double a1 = -0.000000000000085484279566027149L;
  const long double a2 = -0.000000000591615714741844L;
  const long double a3 = -0.000000001019639103513L;
  const long double a4 = 0.00119229890869585713718L;
  const long double a5 = 0.0007330044078284527L;
//    const long double low_output_boundary = -0.7494655984494486291955L;
//    const long double high_output_boundary = 0.69883410378631689642371L;
//    const double low_input_boundary = -750;
//    const double high_input_boundary = 750;

  auto output = static_cast<float>(a0 * (pow(static_cast<long double>(input), 5)) +
    a1 * (pow(static_cast<long double>(input), 4)) +
    a2 * (pow(static_cast<long double>(input), 3)) +
    a3 * (pow(static_cast<long double>(input), 2)) +
    a4 * static_cast<long double>(input) + a5);
  return output;
}

void LeoVcuDriver::llc_publisher()
{
  // TODO(brkay54): Check the jerk data is enabled?
  const rclcpp::Time current_time = get_clock()->now();
  autoware_to_llc_msg_adapter();
  /* check emergency and timeout */

  const double control_cmd_delta_time_ms =
    (current_time - control_command_received_time_).seconds() * 1000.0;
  bool timeouted = false;
  const double t_out = command_timeout_ms_;
  if (t_out >= 0 && control_cmd_delta_time_ms > t_out) {
    timeouted = true;
  }
  if (emergency_cmd_ptr->emergency || timeouted) {
    RCLCPP_ERROR(
      get_logger(), "Emergency Stopping, emergency = %d, timeouted = %d", emergency_cmd_ptr->emergency,
      timeouted);
    send_data.set_long_accel_mps2_ = emergency_stop_acceleration;
  }

  /* check the steering wheel angle and steering wheel angle rate limits */

  if (send_data.set_front_wheel_angle_rad_ < min_steering_wheel_angle ||
    send_data.set_front_wheel_angle_rad_ > max_steering_wheel_angle)
  {
    send_data.set_front_wheel_angle_rad_ = std::min(
      max_steering_wheel_angle,
      std::max(
        send_data.set_front_wheel_angle_rad_,
        min_steering_wheel_angle));
  }

  if ((fabsf(send_data.set_front_wheel_angle_rate_) > max_steering_wheel_angle_rate) &&
    check_steering_angle_rate)
  {
    send_data.set_front_wheel_angle_rate_ = std::min(
      max_steering_wheel_angle_rate,
      std::max(
        send_data.set_front_wheel_angle_rate_,
        -max_steering_wheel_angle_rate));
  }

  const auto serialData = pack_serial_data(send_data);
  serial.write(serialData);
  send_data.counter_++;
}

void LeoVcuDriver::indicator_adapter_to_llc()
{
  /* send turn and hazard commad */

  if (hazard_lights_cmd_ptr_->command ==
    autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ENABLE)   // It is prior!
  {
    send_data.blinker_ = 4;
  } else if (turn_indicators_cmd_ptr_->command ==
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT)
  {
    send_data.blinker_ = 2;
  } else if (turn_indicators_cmd_ptr_->command ==
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT)
  {
    send_data.blinker_ = 3;
  } else {
    send_data.blinker_ = 1;
  }
}


uint8_t LeoVcuDriver::gear_adapter_to_autoware(uint8_t & input)  // TODO(berkay): Check here! Maybe we can make it faster!
{
  switch (input) {
    case 1:
      return 2;
    case 2:
      return 0;
    case 3:
      return 22;
    case 4:
      return 23;
    default:
      return 0;
  }
}

void LeoVcuDriver::gear_adapter_to_llc(const uint8_t & input)
{
  if (input <= 19 && input >= 2) {
    send_data.gear_ = 1;
  } else if (input == 20) {
    if (reverse_gear_enabled_) {
      send_data.gear_ = 2;
    } else {
      send_data.gear_ = 0;
    }
  } else if (input == 22) {
    send_data.gear_ = 3;
  } else if (input == 23) {
    send_data.gear_ = 4;
  } else if (input == 1) {
    send_data.gear_ = 5;
  } else {
    send_data.gear_ = 0;
  }
}
