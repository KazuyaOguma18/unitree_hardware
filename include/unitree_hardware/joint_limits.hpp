/*
Copyright (c) 2024 Kazuya Oguma. All rights reserved.
Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <urdf/model.h>
#include <hardware_interface/handle.hpp>
#include <joint_limits/joint_limits.hpp>
#include <rclcpp/macros.hpp>
namespace joint_limits_interface {

class JointHandle : public hardware_interface::ReadWriteHandle
{
public:
  using hardware_interface::ReadWriteHandle::ReadWriteHandle;
};

class JointLimits {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JointLimits)

  bool load(const std::string & urdf_xml);
  bool configure(const std::vector<JointHandle> & joint_handles);
  void update();
  JointHandle get_by_interface(
    const std::vector<JointHandle> & handles, const std::string & interface_name);
  bool are_names_identical(const std::vector<JointHandle> & handles);
private:
  urdf::ModelSharedPtr model_;
  joint_limits::JointLimits joint_limits_;
  JointHandle joint_position_ = {"", "", nullptr};
  JointHandle joint_velocity_ = {"", "", nullptr};
  JointHandle joint_effort_ = {"", "", nullptr};
};


}  // namespace joint_limits_interface