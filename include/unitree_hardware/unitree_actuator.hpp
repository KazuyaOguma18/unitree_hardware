/*
Copyright (c) 2024 Kazuya Oguma. All rights reserved.
Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef UNITREE_HARDWARE__UNITREE_ACTUATOR_HPP_
#define UNITREE_HARDWARE__UNITREE_ACTUATOR_HPP_

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <hardware_interface/hardware_info.hpp>
#include <memory>

namespace unitree_hardware {
class UnitreeActuator {
public:
  UnitreeActuator();
  bool init(const std::shared_ptr<SerialPort> & serial_port, const hardware_interface::ComponentInfo & info);
  int get_id() const;

  bool set_motor_mode(const MotorMode & mode);

  bool set_position(const double& pos);
  bool set_velocity(const double& vel);
  bool set_effort(const double& eff);
  bool read(double& pos, double& vel, double& eff);
  std::string get_log();

  bool set_torque_limit(const double & torque_limit);

private:
  void clear_cmd();
  double compute_approximately_input_torque(const double& pos_cmd, const double& vel_cmd, const double& eff_cmd) const;
  double compute_approximately_input_torque(const double& pos_cmd, const double& vel_cmd, const double& eff_cmd, const double &pos_gain, const double &vel_gain) const;

  std::shared_ptr<SerialPort> serial_port_;
  MotorCmd motor_cmd_;
  MotorData motor_data_;

  // parameters
  MotorType motor_type_;
  int id_;
  double pos_gain_{0.};
  double vel_gain_{0.};
  double torque_limit_{0.};
  double temperature_limit_{90.};
};
}  // namespace unitree_hardware
#endif  // UNITREE_HARDWARE__UNITREE_ACTUATOR_HPP_