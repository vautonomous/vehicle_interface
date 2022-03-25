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

#include <autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/string.hpp>

#include <leo_vcu_driver/leo_vcu_driver.hpp>
#include <leo_vcu_driver/AsyncSerial.h>
#include <leo_vcu_driver/checksum.h>
#include <leo_vcu_driver/vehicle_interface.h>

#include <rclcpp/rclcpp.hpp>

using autoware_auto_vehicle_msgs::msg::VehicleOdometry;
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::VehicleStateReport;
using autoware_auto_vehicle_msgs::msg::VehicleStateCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::VehicleKinematicState;



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
            void ctrl_cmd_callback(const AckermannControlCommand::SharedPtr& msg);
/**
 * @brief It is callback function which takes data from "/vehicle/state_command" topic in Autoware.Auto.
 */
            void state_cmd_callback(const VehicleStateCommand::SharedPtr& msg);
/**
 * @brief It is callback function which takes data from "/isuzu/odom" topic from ISUZU.
 */
            void isuzu_odom_callback(const nav_msgs::msg::Odometry ::SharedPtr& msg);

//            void publish_vehicle_odom(const VehicleOdometry &state) const;
//
//            void publish_state_report(const VehicleStateReport &state_report) const;
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

            const std::string odom_topic_;
            const std::string state_report_topic_;
            const std::string debug_str_topic_;
            const std::string control_command_topic_;
            const std::string state_command_topic_;
            const std::string gear_report_topic_;
            const std::string isuzu_odom_topic_;
            const std::string vehicle_kinematic_state_topic_;


            float steering_angle_converted = 0;
            float steering_wheel_angle_converted = 0;

            nav_msgs::msg::Odometry isuzu_odom;
            GearReport gear_report;
            AckermannControlCommand last_control_cmd_;
            VehicleStateCommand last_state_cmd_;
            VehicleKinematicState vehicle_kinematic_state_;

            const std::string serial_name_;
            CallbackAsyncSerial serial;

            rclcpp::Publisher<VehicleOdometry>::SharedPtr pub_veh_odom_;
            rclcpp::Publisher<VehicleStateReport>::SharedPtr pub_veh_state_report_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_str_;
            rclcpp::Publisher<GearReport>::SharedPtr gear_report_pub;
            rclcpp::Publisher<VehicleKinematicState>::SharedPtr kinematic_state_pub_;
            rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_veh_cmd_;
            rclcpp::Subscription<VehicleStateCommand>::SharedPtr sub_veh_state_cmd_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_isuzu_odom_;
            rclcpp::TimerBase::SharedPtr tim_data_sender_;

        };
    }  // namespace leo_vcu_driver
}  // namespace autoware

#endif  // LEO_VCU_DRIVER__LEO_VCU_DRIVER_NODE_HPP_
