// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// This file was last modified by Michael Artuerhof
// (michael.artuerhof@gmail.com), on 2024-03-07.

#ifndef ARTEBOT_CONTROL__ARTEBOT_SYSTEM_HPP_
#define ARTEBOT_CONTROL__ARTEBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "artebot_control/serial_motor.hpp"
#include "artebot_control/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace artebot_control
{
    class ArteBotSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        ArteBotSystemHardware();
        RCLCPP_SHARED_PTR_DEFINITIONS(ArteBotSystemHardware)

        ARTEBOT_CONTROL_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        ARTEBOT_CONTROL_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces()
            override;

        ARTEBOT_CONTROL_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces()
            override;

        ARTEBOT_CONTROL_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        ARTEBOT_CONTROL_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        ARTEBOT_CONTROL_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override;

        ARTEBOT_CONTROL_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        rclcpp::Logger logger_;
        std::shared_ptr<SerialMotor> motors_;
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
    };

} // namespace artebot_control

#endif  // ARTEBOT_CONTROL__ARTEBOT_SYSTEM_HPP_
