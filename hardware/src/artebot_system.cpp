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
// (michael.artuerhof@gmail.com), on 2024-05-26.

#include "artebot_control/artebot_system.hpp"
#include "artebot_control/serial_motor.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

using namespace artebot_control;

// #define DEBUG_TELE 1
// #define DEBUG_SP 1

/// @brief radian position transform constant
static const double kInRad = 2.0f * std::numbers::pi / 65535.0f;
/// @brief speed transform constant from/to rad/sec for the STM32 convertion
/// it calculates it every 100 times over 1ms interval (100 * 10^(-3)s = 10^(-1) s)
/// div makes it a multification
static const double kSpeedTransform = kInRad * 100.0f;

ArteBotSystemHardware::ArteBotSystemHardware() : 
  logger_(rclcpp::get_logger("ArteBotSystemHardware")) {}

hardware_interface::CallbackReturn ArteBotSystemHardware::on_init(
    const hardware_interface::HardwareInfo &info)
{
  RCLCPP_INFO(logger_, ">> At the begging of the system loading!");
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(),
                       std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo &joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(logger_, "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
          logger_, "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(logger_, "Joint '%s' has %zu state interfaces. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("DiffBotSystemHardware"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
          logger_, "Joint '%s' have '%s' as only state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  std::string serial_port =
      info_.hardware_parameters["serial_port"];
  int baud_rate =
      std::stoi(info_.hardware_parameters["baud_rate"]);
  try
  {
    motors_ = std::make_shared<SerialMotor>(serial_port.c_str(), baud_rate);
  }
  catch (std::exception &ex)
  {
    RCLCPP_FATAL(logger_, "Error while initializing motors: '%s'", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, ">> At the end of the system initialization, port: %s!", serial_port.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArteBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArteBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ArteBotSystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  // set 0s as default values
  for (auto i = 0u; i < hw_velocities_.size(); i++)
  {
    if (std::isnan(hw_velocities_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  try
  {
    motors_->connect();
  }
  catch (std::exception &ex)
  {
    RCLCPP_FATAL(logger_, "Error while trying to connect to motors: '%s'", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  motors_->setMotorsSpeed(0, 0);

  RCLCPP_INFO(logger_, "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArteBotSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    motors_->disconnect();
  }
  catch (std::exception &ex)
  {
    RCLCPP_FATAL(logger_, "Error while trying to discconnect from motors: '%s'", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArteBotSystemHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
#ifdef DEBUG_TELE
  double pos1 = hw_positions_[0];
  double pos2 = hw_positions_[1];
#endif
  try
  {
    struct MotorsOdometry odo = motors_->readOdometry();
    double vel1 = odo.vel1 * kSpeedTransform;
    double vel2 = odo.vel2 * kSpeedTransform;
    hw_velocities_[0] = vel1;
    hw_positions_[0] += vel1 * dt;
    hw_velocities_[1] = vel2;
    hw_positions_[1] += vel2 * dt;
#ifdef DEBUG_TELE
  double v1 = (hw_positions_[0] - pos1) / dt;
  double v2 = (hw_positions_[1] - pos2) / dt;

  RCLCPP_INFO(logger_, ">> v1: %f %f %d; v2: %f %f %d",
              hw_velocities_[0], v1, odo.pos1,
              hw_velocities_[1], v2, odo.pos2);
#endif
}
  catch (std::exception &ex)
  {
    RCLCPP_FATAL(logger_, "Error while trying to read odometry from motors: '%s'", ex.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArteBotSystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
#ifdef DEBUG_SP
  RCLCPP_INFO(logger_, ">> sv1: %f; sv2: %f", hw_commands_[0], hw_commands_[1]);
#endif
  motors_->setMotorsSpeed(hw_commands_[0] / kSpeedTransform, hw_commands_[1] / kSpeedTransform);
  // motors_->setMotorsSpeed(hw_commands_[0] / kInRad / 100, hw_commands_[1] / kInRad / 100);
  return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(artebot_control::ArteBotSystemHardware,
                       hardware_interface::SystemInterface)