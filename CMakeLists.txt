cmake_minimum_required(VERSION 3.8)
project(unitree_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(unitree_actuator_sdk)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(joint_limits REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/joint_limits.cpp
  src/unitree_actuator.cpp
  src/unitree_hardware.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  unitree_actuator_sdk/include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  hardware_interface
  transmission_interface
  joint_limits
  urdf
  pluginlib
)
set(UNITREE_ACTUATOR_SDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/unitree_actuator_sdk/lib)
link_directories(${UNITREE_ACTUATOR_SDK_LIB})
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64") 
    set(EXTRA_LIBS libUnitreeMotorSDK_Arm64.so)
else()
    set(EXTRA_LIBS libUnitreeMotorSDK_Linux64.so)
endif()

target_link_libraries(
  ${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/unitree_actuator_sdk/lib/${EXTRA_LIBS}
)

pluginlib_export_plugin_description_file(hardware_interface unitree_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  urdf
  controllers
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

ament_package()
