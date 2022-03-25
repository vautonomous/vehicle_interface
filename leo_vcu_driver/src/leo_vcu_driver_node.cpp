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

#include "leo_vcu_driver/leo_vcu_driver_node.hpp"

using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;

namespace autoware {
    namespace leo_vcu_driver {

        LeoVcuDriverNode::LeoVcuDriverNode(const rclcpp::NodeOptions &options)
                : Node("leo_vcu_driver", options),

                  odom_topic_{"/vehicle/odometry"},

                  state_report_topic_{"/vehicle/state_report"},

                  control_command_topic_{"/vehicle/ackermann_vehicle_command"},

                  state_command_topic_{"/vehicle/state_command"},

//                  debug_str_topic_{"/vehicle/debug_str"},

                  gear_report_topic_{"/vehicle/gear_report"},

                  isuzu_odom_topic_{"/isuzu/odom"},

                  vehicle_kinematic_state_topic_{"/vehicle/vehicle_kinematic_state"},

                  last_control_cmd_{},

                  last_state_cmd_{},

                  serial_name_{"/dev/ttyLLC"},

                  serial(serial_name_, 115200),

                  pub_veh_odom_{
                          create_publisher<VehicleOdometry>(odom_topic_, rclcpp::QoS{10}, PubAllocT{})},

                  pub_veh_state_report_{
                          create_publisher<VehicleStateReport>(state_report_topic_, rclcpp::QoS{10}, PubAllocT{})},

                  gear_report_pub{
                          create_publisher<GearReport>(gear_report_topic_, rclcpp::QoS{10}, PubAllocT{})},

                  kinematic_state_pub_{
                          create_publisher<VehicleKinematicState>(vehicle_kinematic_state_topic_, rclcpp::QoS{10},
                                                                  PubAllocT{})},

//                  pub_debug_str_{
//                          create_publisher<std_msgs::msg::String>(debug_str_topic_, rclcpp::QoS{10}, PubAllocT{})},

                  sub_veh_cmd_{create_subscription<AckermannControlCommand>(
                          control_command_topic_, rclcpp::QoS{10},
                          [this](const AckermannControlCommand::SharedPtr msg) { ctrl_cmd_callback(msg); },
                          SubAllocT{})},

                  sub_veh_state_cmd_{create_subscription<VehicleStateCommand>(
                          state_command_topic_, rclcpp::QoS{10},
                          [this](const VehicleStateCommand::SharedPtr msg) { state_cmd_callback(msg); },
                          SubAllocT{})},

                  sub_isuzu_odom_{create_subscription<nav_msgs::msg::Odometry>(
                          isuzu_odom_topic_, rclcpp::QoS{10},
                          [this](const nav_msgs::msg::Odometry::SharedPtr msg) { isuzu_odom_callback(msg); },
                          SubAllocT{})},

                  tim_data_sender_{this->create_wall_timer(10ms, [this] { data_send_callback(); })} {


            using namespace std::placeholders;
            serial.setCallback(bind(&LeoVcuDriverNode::serial_receive_callback, this, _1, _2));

            // Initialize the delta values of kinematic state
            vehicle_kinematic_state_.delta.translation.set__x(0);
            vehicle_kinematic_state_.delta.translation.set__y(0);
            vehicle_kinematic_state_.delta.translation.set__z(0);
            vehicle_kinematic_state_.delta.rotation.set__x(0);
            vehicle_kinematic_state_.delta.rotation.set__y(0);
            vehicle_kinematic_state_.delta.rotation.set__z(0);
            vehicle_kinematic_state_.delta.rotation.set__w(1);
            RCLCPP_INFO_ONCE(this->get_logger(), "Node created successfully!\n");
        }

        void LeoVcuDriverNode::isuzu_odom_callback(const nav_msgs::msg::Odometry::SharedPtr &msg) {
            vehicle_kinematic_state_.set__header(msg->header);
            vehicle_kinematic_state_.state.set__pose(msg->pose.pose);
            vehicle_kinematic_state_.state.set__heading_rate_rps(
                    static_cast<decltype(vehicle_kinematic_state_.state.heading_rate_rps)>(msg->twist.twist.angular.z));
            kinematic_state_pub_->publish(vehicle_kinematic_state_);
        }

        void LeoVcuDriverNode::ctrl_cmd_callback(const AckermannControlCommand::SharedPtr &msg) {
            last_control_cmd_ = *msg;
        }

        void LeoVcuDriverNode::state_cmd_callback(const VehicleStateCommand::SharedPtr &msg) {
            last_state_cmd_ = *msg;
        }

//        void LeoVcuDriverNode::publish_vehicle_odom(const VehicleOdometry &state) const {
//            pub_veh_odom_->publish(state);
//        }
//
//        void LeoVcuDriverNode::publish_state_report(const VehicleStateReport &state_report) const {
//            pub_veh_state_report_->publish(state_report);
//        }

        void LeoVcuDriverNode::data_send_callback() {
            static uint32_t counter = 0;
            sa_to_swa(last_control_cmd_.lateral.steering_tire_angle); // steering angle to steering wheel angle

            CompToLlcData send_data(counter,
                                    last_control_cmd_.longitudinal.acceleration,
                                    last_control_cmd_.longitudinal.speed,
                                    steering_wheel_angle_converted,
                                    last_state_cmd_.blinker,
                                    last_state_cmd_.headlight,
                                    last_state_cmd_.wiper,
                                    last_state_cmd_.gear,
                                    last_state_cmd_.mode,
                                    last_state_cmd_.hand_brake,
                                    last_state_cmd_.horn, 0
            );

            const auto serialData = pack_serial_data(send_data);

//            std::cout << "[ ";
//            for (unsigned i = 0; i < sizeof(CompToLlcData) ; i++) {
//                std::cout << static_cast<int>(serialData.at(i)) << ", ";
//            }
//            std::cout << " ]\n";


            serial.write(serialData);
            counter++;
        }

        void LeoVcuDriverNode::serial_receive_callback(const char *data, unsigned int len) {

//            std::cout << "[ ";
//            for (unsigned i = 0; i < len; i++) {
//                std::cout << static_cast<int>(data[i]) << ", ";
//            }
//            std::cout << " ]\n";

            auto received_data{find_llc_to_comp_msg(data, len)};
            if (!received_data) {
                RCLCPP_INFO(this->get_logger(), "Received data is not viable!\n");

                return;
            }
// TODO(berkay): vehicle/odom is not used by autoware, maybe it should be removed.

//            VehicleOdometry odom{};
            VehicleStateReport state_report{};
            std_msgs::msg::String debug_message{};

            debug_message.data.assign(received_data->state_report.debugstr);
            static std::string debug_str_last{};
            std::string debug_str{received_data->state_report.debugstr};
            if (debug_str != debug_str_last) {
                RCLCPP_INFO(this->get_logger(), debug_str);
            }
            debug_str_last.assign(debug_str);

            swa_to_sa(received_data->vehicle_odometry.front_wheel_angle_rad); // steering wheel angle to steering angle
//            odom.stamp = this->now();
//            odom.front_wheel_angle_rad = steering_angle_converted; // This is steering wheel angle
//            odom.velocity_mps = received_data->vehicle_odometry.velocity_mps;

            vehicle_kinematic_state_.state.set__front_wheel_angle_rad(steering_angle_converted);
            vehicle_kinematic_state_.state.set__longitudinal_velocity_mps(received_data->vehicle_odometry.velocity_mps);
            vehicle_kinematic_state_.header.set__stamp(this->now());
            kinematic_state_pub_->publish(vehicle_kinematic_state_);

            state_report.stamp = this->now();
            state_report.blinker = received_data->state_report.blinker;
            state_report.fuel = received_data->state_report.fuel;
            state_report.gear = received_data->state_report.gear;
            state_report.hand_brake = received_data->state_report.hand_brake;
            state_report.headlight = received_data->state_report.headlight;
            state_report.horn = received_data->state_report.horn;
            state_report.mode = received_data->state_report.mode;

            gear_report.set__report(
                    received_data->state_report.gear); // control this, in lgsvl reverse = 20, drive = 2;
            gear_report_pub->publish(gear_report);
//            pub_veh_odom_->publish(odom);
            pub_veh_state_report_->publish(state_report);
//            pub_debug_str_->publish(debug_message);

        }

        std::experimental::optional<LlcToCompData>
        LeoVcuDriverNode::find_llc_to_comp_msg(const char *data, unsigned int len) {

            receive_buffer_.insert(receive_buffer_.end(), data, data + len);

            // Look data size for checking there is enough data to contain a data structure
            if (receive_buffer_.size() < sizeof(LlcToCompData))
                return std::experimental::nullopt;

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

        std::vector<char> LeoVcuDriverNode::pack_serial_data(const CompToLlcData &data) {
            const auto ptr{reinterpret_cast<const uint8_t *>(&data)};
            std::vector<char> dataVec(ptr, ptr + sizeof data);
            return dataVec;
        }

        void
        LeoVcuDriverNode::sa_to_swa(float &input) { // rad input degree output, maybe constants needs re-calculation
            // TODO: If input or output is out of boundry, what we will do?
            const long double a0 = 31.199461665454701533044340519L;
            const long double a1 = 188.36170978503590077938134L;
            const long double a2 = 366.1782284667889888443542437L;
            const long double a3 = -33.453398194869886816L;
            const long double a4 = 832.269397717359360226L;
            const long double a5 = 0.371560022468346L;
            const long double low_input_boundary = -0.7494655984494486291955L;
            const long double high_input_boundary = 0.69883410378631689642371L;
            const double low_output_boundary = -750;
            const double high_output_boundary = 750;

            if (static_cast<long double>(input) > high_input_boundary) {
                input = static_cast<float>(high_input_boundary);
                //RCLCPP_INFO(this->get_logger(), "Input: Steering Angle is out of boundaries");
            } else if (static_cast<long double>(input) < low_input_boundary) {
                input = static_cast<float>(low_input_boundary);
                //RCLCPP_INFO(this->get_logger(), "Input: Steering Angle is out of boundaries");
            }

            long double temp = (a0 * (pow(static_cast<long double>(input), 5)) +
                                a1 * (pow(static_cast<long double>(input), 4)) +
                                a2 * (pow(static_cast<long double>(input), 3)) +
                                a3 * (pow(static_cast<long double>(input), 2)) +
                                a4 * static_cast<long double>(input) + a5);

            if (temp <= static_cast<long double>(high_output_boundary) &&
                temp >= static_cast<long double>(low_output_boundary)) {
                steering_wheel_angle_converted = static_cast<float>(temp);
            } else {
                RCLCPP_INFO(this->get_logger(), "Output: Steering Wheel Angle is out of boundaries");
            }
        }

        void
        LeoVcuDriverNode::swa_to_sa(float &input) { // degree input rad output, maybe constants needs re-calculation
            // TODO: If input or output is out of boundry, what we will do?
            const long double a0 = 0.0000000000000003633366321943L;
            const long double a1 = -0.000000000000085484279566027149L;
            const long double a2 = -0.000000000591615714741844L;
            const long double a3 = -0.000000001019639103513L;
            const long double a4 = 0.00119229890869585713718L;
            const long double a5 = 0.0007330044078284527L;
            const long double low_output_boundary = -0.7494655984494486291955L;
            const long double high_output_boundary = 0.69883410378631689642371L;
            const double low_input_boundary = -750;
            const double high_input_boundary = 750;

            if (static_cast<double>(input) > high_input_boundary) {
                input = static_cast<float>(high_input_boundary);
                RCLCPP_INFO(this->get_logger(), "Input: Steering Wheel Angle is out of boundaries");
            } else if (static_cast<double>(input) < low_input_boundary) {
                input = static_cast<float>(low_input_boundary);
                RCLCPP_INFO(this->get_logger(), "Input: Steering Wheel Angle is out of boundaries");
            }

            long double temp = (a0 * (pow(static_cast<long double>(input), 5)) +
                                a1 * (pow(static_cast<long double>(input), 4)) +
                                a2 * (pow(static_cast<long double>(input), 3)) +
                                a3 * (pow(static_cast<long double>(input), 2)) +
                                a4 * static_cast<long double>(input) + a5);

            if (temp <= high_output_boundary && temp >= low_output_boundary) {
                steering_angle_converted = static_cast<float>(temp);
            } else {
                RCLCPP_INFO(this->get_logger(), "Output: Steering Angle is out of boundaries");
            }
        }
    } // namespace leo_vcu_driver
} // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::leo_vcu_driver::LeoVcuDriverNode)
