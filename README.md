# UnitreeHardware

The [`ros2_control`](https://github.com/ros-controls/ros2_control) implementation for any kind of Unitree motors.

The `dynamixel_hardware` package is the [`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp) implementation for the multiple ROBOTIS Dynamixel servos.

`It was only tested "GO-M8010-6". "A1" and "B1" were not tested.`

# Set up
First [install ROS 2 Humble on Ubuntu 22.04](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Then follow the instruction below.

```shell
$ source /opt/ros/humble/setup.bash
$ mkdir -p ~/ros/${YourWorkSpace} && cd ~/ros/${YourWorkSpace}/src
$ git clone https://github.com/KazuyaOguma18/unitree_hardware.git
$ cd -
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install
$ . install/setup.bash
```

# Usage
## Start controllers
### Dummy
```shell
$ ros2 launch unitree_hardware dummy.launch.py
```

### SingleMotor
```shell
$ ros2 launch unitree_hardware single_motor.launch.py
```

## Send commands
### Position
```shell
$ ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [2.0]"
```

### Velocity
```shell
$ ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [2.0]"
```

# Parameters
These parameters are described in urdf/*.ros2_control.urdf.xacro
```
<ros2_control name="hoge" type="system">
  <hardware>
    <plugin>unitree_hardware/UnitreeHardware</plugin>
    <param name="serial_interface">/dev/ttyUSB0</param>  # USB port name 
    <param name="use_dummy">false</param>                # If true, it is in dummy mode and motors are not controlled
  </hardware>
  <joint name="pan_joint">
    <param name="id">0</param>
    <param name="motor_type">GO-M8010-6</param>          # GO-M8010-6, A1, B1
    <param name="pos_gain">0.1</param>                   # position gain, Make it too big and it becomes unstable.
    <param name="vel_gain">0.01</param>                  # velocity gain, Make it too big and it becomes unstable.
    <param name="temperature_limit">80.</param>          # temperature limit, when the motor' temperature is exceeded it, the motor stops.
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <transmission name="pan_joint_transmission">
    <plugin>transmission_interface/SimpleTransmission</plugin>
    <actuator name="pan_motor" role="pan_motor"/>
    <joint name="pan_joint" role="pan_joint">
        <mechanical_reduction>1</mechanical_reduction>
    </joint>
  </transmission>
</ros2_control>
```
