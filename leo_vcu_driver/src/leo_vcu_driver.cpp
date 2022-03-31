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
            create_publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>("/vehicle/status/steering_wheel_status", 1);

    // Timer
    const auto period_ns = rclcpp::Rate(loop_rate_).period();
    tim_data_sender_ = rclcpp::create_timer(
            this, get_clock(), period_ns, std::bind(&LeoVcuDriver::llc_publisher, this));

    // From LLC
    serial.setCallback(bind(&LeoVcuDriver::serial_receive_callback, this, _1, _2));
}

void LeoVcuDriver::ctrl_cmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
    control_command_received_time_ = this->now();
    control_cmd_ptr_ = msg;
}

void LeoVcuDriver::emergency_cmd_callback(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
    emergency_cmd_ptr = msg;
}

void LeoVcuDriver::gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
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

void LeoVcuDriver::engage_cmd_callback(const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
{
    engage_cmd_ = msg->engage;
}

void LeoVcuDriver::gate_mode_cmd_callback(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
    gate_mode_cmd_ptr = msg; // AUTO = 0, EXTERNAL = 1
}

void LeoVcuDriver::serial_receive_callback(const char *data, unsigned int len)
{
    auto received_data{find_llc_to_comp_msg(data, len)};
    if (!received_data) {
        RCLCPP_INFO(this->get_logger(), "Received data is not viable!\n");
        return;
    }
    if(received_data->state_report.debugstr != debug_str_last)
    {
        debug_str_last = received_data->state_report.debugstr;
        RCLCPP_INFO(this->get_logger(), "%s", debug_str_last);
    }
    this->current_velocity = received_data->vehicle_odometry.velocity_mps;
    this->current_steering_wheel_angle = received_data->vehicle_odometry.front_wheel_angle_rad; // Steering wheel angle feedback from LLC
    current_steering_tire_angle = swa_to_sa(this->current_steering_wheel_angle); // Calculates current_steering_tire_angle
    std_msgs::msg::Header header;
    header.frame_id = this->base_frame_id_;
    header.stamp = get_clock()->now();

    /* publish current steering tire status */
    {
        autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg;
        steer_msg.stamp = header.stamp;
        steer_msg.steering_tire_angle = current_steering_tire_angle;
        steering_status_pub_->publish(steer_msg);
    }

    /* publish steering wheel status */
    {
        tier4_vehicle_msgs::msg::SteeringWheelStatusStamped steering_wheel_status_msg;
        steering_wheel_status_msg.stamp = header.stamp;
        steering_wheel_status_msg.data = current_steering_wheel_angle;
        steering_wheel_status_pub_->publish(steering_wheel_status_msg);
    }

    /* publish vehicle status control_mode */
    {
        autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
        control_mode_msg.stamp = header.stamp;

        if(received_data->state_report.mode == 3) // Handbrake is boolean check when it is true
        {
            control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::DISENGAGED;
        }
        // TODO(berkay): Check again here! Maybe we need to change structure!
        else if (/*gate_mode_cmd_ptr->data == tier4_control_msgs::msg::GateMode::AUTO &&*/ received_data->state_report.mode == 1)
        {
            control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
        }
        else if (/*gate_mode_cmd_ptr->data == tier4_control_msgs::msg::GateMode::EXTERNAL && */ received_data->state_report.mode == 2)
        {
            control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
        }
        else if (received_data->state_report.mode == 4) {
            control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::NOT_READY;
        }
        else{
            control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::NO_COMMAND;
        }
        control_mode_pub_->publish(control_mode_msg);
    }

    /* publish vehicle status twist */
    {
        autoware_auto_vehicle_msgs::msg::VelocityReport twist;
        twist.header = header;
        twist.longitudinal_velocity = current_velocity;                                 // [m/s]
        twist.heading_rate = current_velocity * std::tan(current_steering_tire_angle) / wheel_base_;  // [rad/s]
        vehicle_twist_pub_->publish(twist);
    }

    /* publish current shift */
    {
        autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg;
        current_gear = gear_adapter_to_autoware(received_data->state_report.gear); // Dont use numbers!
        gear_report_msg.stamp = header.stamp;
        gear_report_msg.report = current_gear;
        gear_status_pub_->publish(gear_report_msg);
    }
    /* publish current turn signal */
    {
        autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
        autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_msg;

        if(received_data->state_report.blinker == 4){
            hazard_msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE;
            turn_msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
        }
        else {
            hazard_msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::DISABLE;
            turn_msg.report = received_data->state_report.blinker;
        }
        turn_msg.stamp = header.stamp;
        turn_indicators_status_pub_->publish(turn_msg);

        hazard_msg.stamp = header.stamp;
        hazard_lights_status_pub_->publish(hazard_msg);
    }
}

std::experimental::optional<LlcToCompData> LeoVcuDriver::find_llc_to_comp_msg(const char *data, unsigned int len)
{
    receive_buffer_.insert(receive_buffer_.end(), data, data + len);

    // Look data size for checking there is enough data to contain a data structure
    if (receive_buffer_.size() < sizeof(LlcToCompData))
    {
        return std::experimental::nullopt;
    }

    size_t search_range = receive_buffer_.size() - sizeof(LlcToCompData) + 1;
    for (size_t i = 0; i < search_range; i++) {
        // Look for frame ids and end-of-frames
        const auto llc_data = reinterpret_cast<LlcToCompData *>(receive_buffer_.data() + i);
        if (llc_data->frame_id1 == llc_to_comp_msg_frame_id &&
            llc_data->frame_id2 == llc_to_comp_msg_frame_id &&
            llc_data->eof_id1 == llc_to_comp_msg_eof_id &&
            llc_data->eof_id2 == llc_to_comp_msg_eof_id) {
            const uint16_t crc = crc_16(receive_buffer_.data() + i, sizeof(LlcToCompData) - 4);
            if (crc == llc_data->crc) {
                LlcToCompData valid_data{*llc_data};
                receive_buffer_.erase(receive_buffer_.begin(),
                                      receive_buffer_.begin() +
                                      static_cast<long>(i) +
                                      sizeof(LlcToCompData));
                return std::experimental::make_optional(valid_data);
            }
        }
    }
    return std::experimental::nullopt;
}

std::vector<char> LeoVcuDriver::pack_serial_data(const CompToLlcData &data) {
    const auto ptr{reinterpret_cast<const uint8_t *>(&data)};
    std::vector<char> dataVec(ptr, ptr + sizeof data);
    return dataVec;
}

float LeoVcuDriver::sa_to_swa(float input) { // rad input degree output, maybe constants needs re-calculation
    // TODO: If input or output is out of boundry, what we will do?
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


    auto output =static_cast<float>(a0 * (pow(static_cast<long double>(input), 5)) +
                        a1 * (pow(static_cast<long double>(input), 4)) +
                        a2 * (pow(static_cast<long double>(input), 3)) +
                        a3 * (pow(static_cast<long double>(input), 2)) +
                        a4 * static_cast<long double>(input) + a5);
    return output;
}

float LeoVcuDriver::swa_to_sa(float input) { // degree input rad output, maybe constants needs re-calculation
    // TODO: If input or output is out of boundry, what we will do?
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
    const rclcpp::Time current_time = get_clock()->now();
    desired_state.gear = gear_adapter_to_llc(current_gear);
    desired_state.blinker = 1; // default is off
    float desired_acceleration = control_cmd_ptr_->longitudinal.acceleration;
    /* check emergency and timeout */

    const double control_cmd_delta_time_ms =
            (current_time - control_command_received_time_).seconds() * 1000.0;
    bool timeouted = false;
    const int t_out = command_timeout_ms_;
    if (t_out >= 0 && control_cmd_delta_time_ms > t_out) {
        timeouted = true;
    }
    if (emergency_cmd_ptr->emergency || timeouted) {
        RCLCPP_ERROR(
                get_logger(), "Emergency Stopping, emergency = %d, timeouted = %d", emergency_cmd_ptr->emergency, timeouted);
        desired_acceleration = emergency_stop_acceleration; // PARAMETER!!
    }
    // Calculate the desired wheel angle and desired wheel angle rate
    steering_wheel_angle_cmd = sa_to_swa(control_cmd_ptr_->lateral.steering_tire_angle);
    steering_wheel_angle_rate_cmd = sa_to_swa(control_cmd_ptr_->lateral.steering_tire_angle
            + control_cmd_ptr_->lateral.steering_tire_rotation_rate) - steering_wheel_angle_cmd; // calculate the rate include rate of ctrl output and publisher rate

    /* check shift change */

    if (current_gear != gear_cmd_ptr_->command) {  // velocity is low -> the shift can be changed
        if (std::fabs(current_velocity) < 0.1) {  // need shift
        // TODO(berkay): check here again!
        desired_state.gear = gear_adapter_to_llc(gear_cmd_ptr_->command);
        } else {
            RCLCPP_WARN(get_logger(), "Gear change is not allowed, current_velocity = %f", current_velocity);
        }
    }

    /* send mode */

    if(!engage_cmd_){
        desired_state.mode = 3; // DISENGAGED. IT IS PRIOR
    }
    else if(gate_mode_cmd_ptr->data == tier4_control_msgs::msg::GateMode::AUTO){
        desired_state.mode = 1;
    }
    else if(gate_mode_cmd_ptr->data == tier4_control_msgs::msg::GateMode::EXTERNAL){
        desired_state.mode = 2; // MANUAL
    }
    else {
        desired_state.mode =4; // NOT READY
    }

    // TODO(berkay): Check the handbrake again!

    /* send turn and hazard commad */

    if(hazard_lights_cmd_ptr_->command == autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ENABLE)
    { // It is prior!
        desired_state.blinker = 4;
    }
    else if(turn_indicators_cmd_ptr_->command == autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT)
    {
        desired_state.blinker = 2;
    }
    else if(turn_indicators_cmd_ptr_->command == autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT)
    {
        desired_state.blinker = 3;
    }
    else
    {
        desired_state.blinker = 1;
    }

    //
    CompToLlcData send_data(counter_,
                            desired_acceleration,
                            control_cmd_ptr_->longitudinal.speed,
                            steering_wheel_angle_cmd,
                            steering_wheel_angle_rate_cmd,
                            desired_state.blinker,
                            desired_state.headlight,
                            desired_state.wiper,
                            desired_state.gear,
                            desired_state.mode,
                            desired_state.hand_brake,
                            desired_state.horn, 0
    );
    const auto serialData = pack_serial_data(send_data);
    serial.write(serialData);
    counter_++;
}


uint8_t LeoVcuDriver::gear_adapter_to_autoware(uint8_t input){
    switch (input) {
        case 1:
            return 2;
        case 2:
            if(reverse_gear_enabled_) {
                return 20;
            } else {
                return 0;
            }
        case 3:
            return 22;
        case 4:
            return 23;
        default:
            return 0;
    }
}

uint8_t LeoVcuDriver::gear_adapter_to_llc(uint8_t input){

    if(input <= 19 && input >= 2) {
        return 1;
    }
    else if (input == 20){
        if(reverse_gear_enabled_) {
            return 2;
        } else {
            return 0;
        }
    }
    else if (input == 22){
        return 3;
    }
    else if (input == 23){
        return 4;
    }
    else if (input == 1){
        return 5;
    }
    else{
        return 0;
    }
}