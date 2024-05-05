/*
Copyright (c) 2024 Kazuya Oguma. All rights reserved.
Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef UNITREE_HARDWARE__UNITREE_HARDWARE_HPP_
#define UNITREE_HARDWARE__UNITREE_HARDWARE_HPP_

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "visibility_control.h"
#include "unitree_actuator.hpp"

#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

#include <rclcpp/macros.hpp>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace unitree_hardware {
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue joint_command{};
  JointValue ator_command{};
  JointValue prev_command{};
};

enum class ControlMode
{
  First,
  Position,
  Velocity,
  Effort
};

class UnitreeHardware : public hardware_interface::SystemInterface 
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeHardware)

  UNITREE_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  UNITREE_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  UNITREE_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  UNITREE_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  UNITREE_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  return_type set_motor_mode(const MotorMode & mode);

  return_type reset_command();

  return_type set_joint_positions();
  return_type set_joint_velocities();
  return_type set_joint_efforts();

  std::vector<Joint> joints_;
  std::vector<UnitreeActuator> ators_;
  std::vector<transmission_interface::TransmissionSharedPtr> transmissions_;
  bool use_dummy_{false};
  ControlMode last_control_mode_{ControlMode::First};
};

}  // namespace unitree_hardware

#endif  // UNITREE_HARDWARE__UNITREE_HARDWARE_HPP_