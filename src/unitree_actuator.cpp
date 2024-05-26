#include "unitree_hardware/unitree_actuator.hpp"
#include <rclcpp/rclcpp.hpp>

namespace unitree_hardware {
UnitreeActuator::UnitreeActuator(){}

bool UnitreeActuator::init(const std::shared_ptr<SerialPort> & serial_port, const hardware_interface::ComponentInfo & info)
{
  serial_port_ = serial_port;

  // Actuator Parameter
  // TODO : use default value & avoid segmentation fault
  id_ = std::stoi(info.parameters.at("id"));
  pos_gain_ = std::stod(info.parameters.at("pos_gain"));
  vel_gain_ = std::stod(info.parameters.at("vel_gain"));
  temperature_limit_ = std::stod(info.parameters.at("temperature_limit"));
  std::string motor_type_str = info.parameters.at("motor_type");

  if (!(motor_type_str == "GO-M8010-6" || motor_type_str == "A1" || motor_type_str == "B1")) {
    RCLCPP_ERROR(rclcpp::get_logger("UnitreeActuator"), "Unabled motor type: usage(GO-M8010-6, A1, A1)");
    return false;
  }

  if (motor_type_str == "A1")              { motor_type_ = MotorType::A1;         }
  else if (motor_type_str == "B1")         { motor_type_ = MotorType::B1;         }
  else if (motor_type_str == "GO-M8010-6") { motor_type_ = MotorType::GO_M8010_6; }

  // Motor Init & Check motor
  motor_data_.motorType = motor_type_;
  motor_cmd_.motorType = motor_type_;
  motor_cmd_.mode = queryMotorMode(motor_type_, MotorMode::BRAKE);
  motor_cmd_.id = id_;
  if (!serial_port_->sendRecv(&motor_cmd_, &motor_data_)) {
    return false;
  }
  return motor_data_.merror == 0;
}

int UnitreeActuator::get_id() const
{
  return id_;
}

bool UnitreeActuator::set_motor_mode(const MotorMode & mode)
{
  motor_cmd_.mode = queryMotorMode(motor_type_, mode);
  return true;
}

bool UnitreeActuator::set_position(const double & pos_cmd)
{
  // Update latest state
  if (!serial_port_->sendRecv(&motor_cmd_, &motor_data_)) {
    return false;
  }
  clear_cmd();

  double pos_gain = pos_gain_;
  // Torque Limit
  if (torque_limit_ != 0.) {
    const double approx_input_torque = abs(compute_approximately_input_torque(pos_cmd, 0., 0., pos_gain, 0.));
    if (approx_input_torque > torque_limit_) {
      pos_gain = torque_limit_ / approx_input_torque * pos_gain;
    }
  }

  // Temperature Limit
  if (motor_data_.temp > temperature_limit_) {
    pos_gain = 0.;
  } 

  motor_cmd_.q = pos_cmd * queryGearRatio(motor_type_);
  motor_cmd_.kp = pos_gain;
  return serial_port_->sendRecv(&motor_cmd_, &motor_data_);
}

bool UnitreeActuator::set_velocity(const double & vel_cmd)
{
  // Update latest state
  if (!serial_port_->sendRecv(&motor_cmd_, &motor_data_)) {
    return false;
  }
  clear_cmd();

  // Stopping
  if (vel_cmd == 0.) {
    motor_cmd_.q = motor_data_.q;
    motor_cmd_.kp = pos_gain_;
    return serial_port_->sendRecv(&motor_cmd_, &motor_data_);
  }

  double vel_gain = vel_gain_;
  // Torque Limit
  if (torque_limit_ != 0.) {
    const double approx_input_torque = abs(compute_approximately_input_torque(0., vel_cmd, 0., 0., vel_gain));
    if (approx_input_torque > torque_limit_) {
      vel_gain = torque_limit_ / approx_input_torque * vel_gain;
    }
  }

  // Temperature Limit
  if (motor_data_.temp > temperature_limit_) {
    vel_gain = 0.;
  }
  
  motor_cmd_.dq = vel_cmd * queryGearRatio(motor_type_);
  motor_cmd_.kd = vel_gain;
  return serial_port_->sendRecv(&motor_cmd_, &motor_data_);
}

bool UnitreeActuator::set_effort(const double & eff_cmd)
{
  double out_eff_cmd = eff_cmd;
  clear_cmd();

  // Torque Limit
  if (torque_limit_ != 0.) {
    const double approx_input_torque = abs(compute_approximately_input_torque(eff_cmd, 0., 0.));

    if (approx_input_torque > torque_limit_) {
      out_eff_cmd = torque_limit_ / approx_input_torque * eff_cmd;
    }
  }


  // Temperature Limit
  if (motor_data_.temp > temperature_limit_) {
    out_eff_cmd = 0.;
  }

  motor_cmd_.tau = out_eff_cmd;
  return serial_port_->sendRecv(&motor_cmd_, &motor_data_);
}


bool UnitreeActuator::read(double & pos, double & vel, double & eff)
{
  if (!serial_port_->sendRecv(&motor_cmd_, &motor_data_)) {
    return false;
  }
  pos = motor_data_.q / queryGearRatio(motor_type_);
  vel = motor_data_.dq / queryGearRatio(motor_type_);
  eff = motor_data_.tau;
  return true;
}

std::string UnitreeActuator::get_log()
{
  switch (motor_data_.merror)
  {
  case 0:
    return "Normal";
    break;
  case 1:
    return "Overheating";
    break;
  case 2:
    return "Overcurrent";
    break;
  case 3:
    return "Overvoltage";
    break;
  case 4:
    return "Encoder malfunction";
    break;
  default:
    break;
  }
  return "";
}

bool UnitreeActuator::set_torque_limit(const double & torque_limit) {
  if (torque_limit <= 0.) {
    RCLCPP_ERROR(rclcpp::get_logger("UnitreeActuator"), "Invalid torque limit (0) detected! please set torque limit greater than 0");
    return false;
  } 
  torque_limit_ = torque_limit;
  return true;
}

void UnitreeActuator::clear_cmd()
{
  motor_cmd_.q = 0.;
  motor_cmd_.kp = 0.;
  motor_cmd_.dq = 0.;
  motor_cmd_.kd = 0.;
  motor_cmd_.tau = 0.;
}

double UnitreeActuator::compute_approximately_input_torque(const double& pos_cmd, const double& vel_cmd, const double& eff_cmd) const
{
  return compute_approximately_input_torque(pos_cmd, vel_cmd, eff_cmd, motor_cmd_.kp, motor_cmd_.kd);
}

double UnitreeActuator::compute_approximately_input_torque(const double& pos_cmd, const double& vel_cmd, const double& eff_cmd, const double &pos_gain, const double &vel_gain) const
{
  const double input_pos = pos_gain * (pos_cmd * queryGearRatio(motor_type_) - motor_data_.q);
  const double input_vel = vel_gain * (vel_cmd * queryGearRatio(motor_type_) - motor_data_.dq);
  const double input_eff = eff_cmd;
  return input_pos + input_vel + input_eff;
}

}  // namespace unitree_hardware