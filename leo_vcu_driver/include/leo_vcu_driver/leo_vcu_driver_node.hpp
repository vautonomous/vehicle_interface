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
/// \brief This file defines the leo_vcu_driver_node class.

#ifndef LEO_VCU_DRIVER__LEO_VCU_DRIVER_NODE_HPP_
#define LEO_VCU_DRIVER__LEO_VCU_DRIVER_NODE_HPP_

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

#include <leo_vcu_driver/leo_vcu_driver.hpp>
#include <leo_vcu_driver/AsyncSerial.h>
#include <leo_vcu_driver/checksum.h>
#include <leo_vcu_driver/vehicle_interface.h>

#include <rclcpp/rclcpp.hpp>

namespace autoware {
    namespace leo_vcu_driver {


/// \class LeoVcuDriverNode
/**
 * @brief This is vehicle interface node. The purpose of this node is that provide communication between
 * low level controller and Autoware.Auto.
 * You can see the input output diagram in:
 * https://onedrive.live.com/redir?resid=15A5A24ED106BA89!740&authkey=!AAn3rnbvRcmLhao&ithint=file%2cxlsx&e=lW0yJg
 */
        class LEO_VCU_DRIVER_PUBLIC LeoVcuDriverNode : public rclcpp::Node {
        public:
            /// \brief default constructor, starts driver
            /// \throw runtime error if failed to start threads or configure driver
            explicit LeoVcuDriverNode(const rclcpp::NodeOptions &options);

            ~LeoVcuDriverNode() override {
                serial.close();
            }
/**
 * @brief It is callback function which takes data from "/vehicle/vehicle_command" topic in Autoware.Auto.
 */
            void ctrl_cmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr& msg);
/**
 * @brief It is callback function which takes data from "/vehicle/state_command" topic in Autoware.Auto.
 */
            void turn_indicators_cmd_callback(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr& msg);
/**
 * @brief It is callback function which takes data from "/isuzu/odom" topic from ISUZU.
 */
            void hazard_lights_cmd_callback(const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::SharedPtr& msg);
            void gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr& msg);
            void engage_cmd_callback(const autoware_auto_vehicle_msgs::msg::Engage::SharedPtr& msg);
            void emergency_cmd_callback(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::SharedPtr& msg);
            void gate_mode_cmd_callback(const tier4_control_msgs::msg::GateMode::SharedPtr& msg);

/**
 * @brief It sends data from interface to low level controller.
 */
            void data_send_callback();
/**
 * @brief It takes data from interface to low level controller.
 */
            void serial_receive_callback(const char *data, unsigned int len);
/**
 * @brief It converts the steering angle to steering wheel angle.
 * Steering angle means "Teker açısı" and which is radian.
 * Steering wheel angle means "Direksiyon açısı" and which is degree.
 */
            void sa_to_swa(float & input);
/**
 * @brief It converts the steering wheel angle to steering angle.
 * Steering angle means "Teker açısı" and which is radian.
 * Steering wheel angle means "Direksiyon açısı" and which is degree.
 */
            void swa_to_sa(float & input);


        private:

            std::experimental::optional<LlcToCompData>
            find_llc_to_comp_msg(const char *data, unsigned int len);

            static std::vector<char>
            pack_serial_data(const CompToLlcData &data);

            std::vector<uint8_t> receive_buffer_;

            // Topics from autoware
            const std::string control_cmd_topic_;
            const std::string gear_cmd_topic_;
            const std::string emergency_cmd_topic_;
            const std::string engage_cmd_topic_;
            const std::string hazard_lights_cmd_topic_;
            const std::string turn_indicators_cmd_topic_;
            const std::string current_gate_topic_;

            /* input values */
            autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
            autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr turn_indicators_cmd_ptr_;
            autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr hazard_lights_cmd_ptr_;
            autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;
            autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr engage_cmd_ptr;
            tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr emergency_cmd_ptr;
            tier4_control_msgs::msg::GateMode::ConstSharedPtr gate_mode_cmd_ptr;


            // Topics to autoware
            const std::string control_mode_topic;
            const std::string velocity_status_topic;
            const std::string hazard_lights_status_topic;
            const std::string steering_status_topic;
            const std::string gear_status_topic;
            const std::string turn_indicators_status_topic;


            const std::string debug_str_topic_;

            float steering_angle_converted = 0;
            float steering_wheel_angle_converted = 0;


            const std::string serial_name_;
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
            rclcpp::Publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>::SharedPtr steering_wheel_status_pub_;


            rclcpp::TimerBase::SharedPtr tim_data_sender_;

        };
    }  // namespace leo_vcu_driver
}  // namespace autoware

#endif  // LEO_VCU_DRIVER__LEO_VCU_DRIVER_NODE_HPP_
