#include "unitree_hardware/joint_limits.hpp"
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace joint_limits_interface {
bool JointLimits::load(const std::string & urdf_xml)
{
  model_ = std::shared_ptr<urdf::Model>();
  if (!model_->initString(urdf_xml)) {
    RCLCPP_ERROR(rclcpp::get_logger("JointLimits"), "Failed to parse URDF");
    return false;
  }
  return true;
}

bool JointLimits::configure(const std::vector<JointHandle> & joint_handles)
{
  if (joint_handles.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("JointLimits"), "No joint handles were passed in");
    return false;
  }
  if (!are_names_identical(joint_handles))
  {
    RCLCPP_ERROR(rclcpp::get_logger("JointLimits"), "Joint names should be identical");
    return false;
  }

  joint_position_ = get_by_interface(joint_handles, hardware_interface::HW_IF_POSITION);
  joint_velocity_ = get_by_interface(joint_handles, hardware_interface::HW_IF_VELOCITY);
  joint_effort_   = get_by_interface(joint_handles, hardware_interface::HW_IF_EFFORT);

  urdf::JointConstSharedPtr joint = model_->getJoint(joint_handles[0].get_prefix_name());
  if (!joint) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("JointLimits"), "Failed to find joint frame : '" << joint_handles[0].get_prefix_name() << "'");
    return false;
  }
  if (!joint->limits) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("JointLimits"), "Failed to find limits for joint frame : '" << joint_handles[0].get_prefix_name() << "'");
    return false;    
  }
  joint_limits_.min_position = joint->limits->lower;
  joint_limits_.max_position = joint->limits->upper;
  joint_limits_.max_velocity = joint->limits->velocity;
  joint_limits_.max_effort   = joint->limits->effort;

  joint_limits_.has_position_limits = true;
  joint_limits_.has_velocity_limits = true;
  joint_limits_.has_effort_limits = true;
  return true;
}

void JointLimits::update()
{
  if (joint_position_ && joint_limits_.has_position_limits) {
    if (joint_limits_.min_position > joint_position_.get_value()) {
      joint_position_.set_value(joint_limits_.min_position);
    }
    if (joint_position_.get_value() > joint_limits_.max_position) {
      joint_position_.set_value(joint_limits_.max_position);
    }
  }

  if (joint_velocity_ && joint_limits_.has_velocity_limits) {
    if (abs(joint_velocity_.get_value()) > joint_limits_.max_velocity) {
      joint_velocity_.set_value(joint_limits_.max_velocity * joint_velocity_.get_value() / abs(joint_velocity_.get_value()));
    }
  }

  if (joint_effort_ && joint_limits_.has_effort_limits) {
    if (abs(joint_effort_.get_value()) > joint_limits_.max_effort) {
      joint_effort_.set_value(joint_limits_.max_effort * joint_effort_.get_value() / abs(joint_effort_.get_value()));
    }    
  }
}

JointHandle JointLimits::get_by_interface(const std::vector<JointHandle> & handles, const std::string & interface_name)
{
  const auto result = std::find_if(
    handles.cbegin(), handles.cend(),
    [&interface_name](const auto handle) { return handle.get_interface_name() == interface_name; });
  if (result == handles.cend())
  {
    return JointHandle(handles.cbegin()->get_prefix_name(), interface_name, nullptr);
  }
  return *result;  
}

bool are_names_identical(const std::vector<JointHandle> & handles)
{
  std::vector<std::string> names;
  std::transform(
    handles.cbegin(), handles.cend(), std::back_inserter(names),
    [](const auto & handle) { return handle.get_prefix_name(); });
  return std::equal(names.cbegin() + 1, names.cend(), names.cbegin());
}
}