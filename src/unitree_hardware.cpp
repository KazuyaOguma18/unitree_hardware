/*
Copyright (c) 2024 Kazuya Oguma. All rights reserved.
Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include "unitree_hardware/unitree_hardware.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <transmission_interface/simple_transmission.hpp>

namespace unitree_hardware {

static const std::string HW_NAME = "UnitreeHardware";

CallbackReturn UnitreeHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "on_init");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  ators_.resize(info_.joints.size());
  transmissions_.resize(info_.joints.size());
  joint_limits_.resize(info_.joints.size());

  for (uint i = 0; i < info_.joints.size(); i++) {
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].ator_command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].ator_command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].ator_command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].ator_command.position;
    joints_[i].prev_command.velocity = joints_[i].ator_command.velocity;
    joints_[i].prev_command.effort = joints_[i].ator_command.effort;
  }

  auto transmission_loader = transmission_interface::SimpleTransmissionLoader();

  // Initialize Transmission & JointLimits
  for (size_t i = 0; i < info_.joints.size(); i++) {
    // Initialize JointLimits
    {
      joint_limits_[i] = std::make_shared<joint_limits_interface::JointLimits>();
      if (!joint_limits_[i]->load(info_.original_xml)) {
        return CallbackReturn::ERROR;
      }
    
      joint_limits_interface::JointHandle joint_handle_position(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].joint_command.position);
      joint_limits_interface::JointHandle joint_handle_velocity(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].joint_command.velocity);
      joint_limits_interface::JointHandle joint_handle_effort(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].joint_command.effort);
      if (!joint_limits_[i]->configure({joint_handle_position, joint_handle_velocity, joint_handle_effort})) {
        return CallbackReturn::ERROR;
      }
    }

    // Initialize Transmission
    for (auto transmission_info : info_.transmissions) {
      if (transmission_info.joints.size() != 1) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Ignoring transmission '" << transmission_info.name << "' because it has more than one joint!");
        continue;
      }
      if (transmission_info.type != "transmission_interface/SimpleTransmission") {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Transmission '" << transmission_info.name << "' of type '" << transmission_info.type << "' is not supported!");
        return CallbackReturn::ERROR;        
      }

      transmission_interface::TransmissionSharedPtr transmission;
      try {
        transmission = transmission_loader.load(transmission_info);
      }
      catch (const transmission_interface::TransmissionInterfaceException &exc) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger(HW_NAME), "Error while loading '" << transmission_info.name << "': " << exc.what());
        return hardware_interface::CallbackReturn::ERROR;        
      }

      std::vector<transmission_interface::JointHandle> joint_handles;
      for (const auto &joint_info : transmission_info.joints)
      {
        transmission_interface::JointHandle joint_handle_position(joint_info.name, hardware_interface::HW_IF_POSITION, &joints_[i].joint_command.position);
        transmission_interface::JointHandle joint_handle_velocity(joint_info.name, hardware_interface::HW_IF_VELOCITY, &joints_[i].joint_command.velocity);
        transmission_interface::JointHandle joint_handle_effort(joint_info.name, hardware_interface::HW_IF_EFFORT, &joints_[i].joint_command.effort);

        joint_handles.push_back(joint_handle_position);
        joint_handles.push_back(joint_handle_velocity);
        joint_handles.push_back(joint_handle_effort);
      }

      std::vector<transmission_interface::ActuatorHandle> actuator_handles;
      for (const auto &actuator_info : transmission_info.actuators) {
        transmission_interface::ActuatorHandle actuator_handle_position(actuator_info.name, hardware_interface::HW_IF_POSITION, &joints_[i].ator_command.position);
        transmission_interface::ActuatorHandle actuator_handle_velocity(actuator_info.name, hardware_interface::HW_IF_VELOCITY, &joints_[i].ator_command.velocity);
        transmission_interface::ActuatorHandle actuator_handle_effort(actuator_info.name, hardware_interface::HW_IF_EFFORT, &joints_[i].ator_command.effort);

        actuator_handles.push_back(actuator_handle_position);
        actuator_handles.push_back(actuator_handle_velocity);
        actuator_handles.push_back(actuator_handle_effort);
      }

      try {
        transmission->configure(joint_handles, actuator_handles);
      }
      catch (const transmission_interface::TransmissionInterfaceException &exc) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger(HW_NAME), "Error while configuring '" << transmission_info.name << "': " << exc.what());
        return hardware_interface::CallbackReturn::ERROR;        
      }

      transmissions_[i] = transmission;   
      RCLCPP_INFO_STREAM(rclcpp::get_logger(HW_NAME), "Loaded transmission '" << transmission_info.name << "' for joint '" << info_.joints[i].name << "'");
    }

    if (!transmissions_[i])
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Could find transmission for joint '" << info_.joints[i].name << "'");
      return CallbackReturn::ERROR;
    }
  }

  // Use dummy ?
  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    info_.hardware_parameters.at("use_dummy") == "true")
  {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "dummy mode");
    return CallbackReturn::SUCCESS;
  }  

  if (info_.hardware_parameters.find("serial_interface") == info_.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to find parameter 'serial_interface'");
    return CallbackReturn::ERROR;
  }
  auto serial_port = std::make_shared<SerialPort>(info_.hardware_parameters.at("serial_interface"));

  for (size_t i = 0; i < info_.joints.size(); i++) {
    // Initialize Actuator
    auto ator = UnitreeActuator();
    if (!ator.init(serial_port, info_.joints[i]) || !ator.set_torque_limit(joint_limits_[i]->get_max_effort())) {
      RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to initialize actuator");
      return CallbackReturn::ERROR;
    }
    ators_[i] = ator;
    RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "joint_id %ld: %d", i, ators_[i].get_id());
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> UnitreeHardware::export_state_interfaces() 
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "export_state_interface");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;  
}

std::vector<hardware_interface::CommandInterface> UnitreeHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "export_command_interface");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].joint_command.position));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].joint_command.velocity));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].joint_command.effort));
  }

  return command_interfaces;  
}

CallbackReturn UnitreeHardware::on_activate(const rclcpp_lifecycle::State & previous_state [[maybe_unused]])
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "on_activate");
  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }

  if (!use_dummy_) {
    set_motor_mode(MotorMode::FOC);
  }

  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

CallbackReturn UnitreeHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state [[maybe_unused]])
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "on_deactivate");
  return CallbackReturn::SUCCESS;
}

return_type UnitreeHardware::read(const rclcpp::Time & time [[maybe_unused]], const rclcpp::Duration & period [[maybe_unused]]) 
{
  if (use_dummy_) {
    return return_type::OK;
  }

  for (size_t i=0; i < joints_.size(); i++) {
    double pos, vel, eff;
    if (!ators_[i].read(pos, vel, eff)) {
      RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to read status: %s", ators_[i].get_log().c_str());
      return return_type::ERROR;
    }
    const double reduction = std::static_pointer_cast<transmission_interface::SimpleTransmission>(transmissions_[i])->get_actuator_reduction();
    joints_[i].state.position = pos / reduction;
    joints_[i].state.velocity = vel / reduction;
    joints_[i].state.effort = eff * reduction;
  }
  return return_type::OK;
}

return_type UnitreeHardware::write(const rclcpp::Time & time [[maybe_unused]], const rclcpp::Duration & period [[maybe_unused]])
{
  // JointLimits
  for (auto joint_limit : joint_limits_) {
    joint_limit->update();
  }

  // Transmission
  for (auto transmission : transmissions_) {
    transmission->joint_to_actuator();
  }

  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.prev_command.position = joint.ator_command.position;
      joint.state.position = joint.joint_command.position;
    }
    return return_type::OK;
  }

  // Actuator
  // Position control
  if (std::any_of(joints_.cbegin(), joints_.cend(), [](auto j) {
    return j.ator_command.position != j.prev_command.position;}))
  {
    last_control_mode_ = ControlMode::Position;
    return set_joint_positions(); 
  }

  // Velocity control
  if (std::any_of(joints_.cbegin(), joints_.cend(), [](auto j) {
    return j.ator_command.velocity != j.prev_command.velocity;}))
  {
    last_control_mode_ = ControlMode::Velocity;
    return set_joint_velocities(); 
  }  

  // Effort control
  if (std::any_of(joints_.cbegin(), joints_.cend(), [](auto j) {
    return j.ator_command.effort != j.prev_command.effort;}))
  {
    last_control_mode_ = ControlMode::Effort;
    return set_joint_efforts(); 
  }  

  // If all command values are unchanged, then remain in existing control mode and set
  // corresponding command values
  switch (last_control_mode_) {
    case ControlMode::Position:
      return set_joint_positions();
      break;
    case ControlMode::Velocity:
      return set_joint_velocities();
      break;
    case ControlMode::Effort:
      return set_joint_efforts();
      break;
    case ControlMode::First:
      return return_type::OK;
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Invalid control method!");
      return return_type::ERROR;
      break;
  }
}

return_type UnitreeHardware::set_motor_mode(const MotorMode & mode)
{
  for (size_t i=0; i<joints_.size(); i++) {
    if(!ators_[i].set_motor_mode(mode)) {
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

return_type UnitreeHardware::reset_command()
{
  for (size_t i = 0; i < joints_.size(); i++) {
    joints_[i].joint_command.position = joints_[i].state.position;
    joints_[i].joint_command.velocity = 0.0;
    joints_[i].joint_command.effort = 0.0;
    transmissions_[i]->joint_to_actuator();
    joints_[i].prev_command.position = joints_[i].ator_command.position;
    joints_[i].prev_command.velocity = joints_[i].ator_command.velocity;
    joints_[i].prev_command.effort = joints_[i].ator_command.effort;
  }

  return return_type::OK;
}

return_type UnitreeHardware::set_joint_positions()
{
  for (size_t i=0; i<joints_.size(); i++) {
    joints_[i].prev_command.position = joints_[i].ator_command.position;
    if (!ators_[i].set_position(joints_[i].ator_command.position)) {
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

return_type UnitreeHardware::set_joint_velocities()
{
  for (size_t i=0; i<joints_.size(); i++) {
    joints_[i].prev_command.velocity = joints_[i].ator_command.velocity;
    if (!ators_[i].set_velocity(joints_[i].ator_command.velocity)) {
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

return_type UnitreeHardware::set_joint_efforts()
{
  for (size_t i=0; i<joints_.size(); i++) {
    joints_[i].prev_command.effort = joints_[i].ator_command.effort;
    if (!ators_[i].set_effort(joints_[i].ator_command.effort)) {
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}
}  // namespace unitree_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unitree_hardware::UnitreeHardware, hardware_interface::SystemInterface)