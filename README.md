# My Bot - Differential Drive Robot Platform

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![License](https://img.shields.io/badge/License-Apache2.0-green)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2022.04-orange)

<p align="center">
  <img src="https://raw.githubusercontent.com/ros-controls/ros2_control/master/doc/images/ros2_control_architecture.png" alt="ROS2 Control Architecture" width="800"/>
</p>

## 🤖 Overview

**My Bot** is a fully-featured differential drive robot platform built with ROS2 Humble. It features a complete ROS2 Control hardware interface, URDF model, simulation support, and various teleop options for precise control in both real-world and simulated environments.

### Features

- **Complete Hardware Abstraction Layer** - Seamless integration between software and hardware
- **Differential Drive Control** - Precise velocity-based control of differential drive robot
- **Flexible Robot Control** - Support for both stamped and non-stamped velocity commands
- **Multiple Teleop Options** - Control via keyboard, joystick, or programmatically
- **Simulation Support** - Works with Gazebo and mock hardware plugins
- **Sensor Integration** - Ready for LIDAR, camera, and other sensors

## 📋 Table of Contents

- [Quick Start](#-quick-start)
- [Installation](#-installation)
- [Hardware Setup](#-hardware-setup)
- [Usage](#-usage)
- [Teleoperation](#-teleoperation)
- [Simulation](#-simulation)
- [Configuration](#-configuration)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [License](#-license)

## 🚀 Quick Start

```bash
# 1. Create and setup workspace
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src

# 2. Clone the repository
git clone https://github.com/yourusername/my_bot.git

# 3. Install dependencies
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
colcon build --symlink-install

# 5. Source the workspace
source install/setup.bash

# 6. Launch the robot
ros2 launch my_bot launch_robot.launch.py
```

## 🔧 Installation

### Prerequisites

1. **Ubuntu 22.04**
   ```bash
   # Check Ubuntu version
   lsb_release -a
   ```

2. **ROS2 Humble**
   ```bash
   # Install ROS2 Humble if not already installed
   sudo apt update && sudo apt install ros-humble-desktop
   ```

3. **Colcon build tools**
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

4. **Additional dependencies**
   ```bash
   sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
   sudo apt install ros-humble-gazebo-ros2-control
   ```

### Build from Source

```bash
# Create a workspace
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src

# Clone this repository
git clone https://github.com/yourusername/my_bot.git

# Install dependencies
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## 🔌 Hardware Setup

### Supported Hardware

- Any Arduino-compatible microcontroller
- Differential drive motors with encoders
- Serial communication (USB, UART)
- Supported sensors: LIDAR, cameras, IMUs

### Hardware Interface

The robot uses a custom hardware interface that implements the ROS2 Control `SystemInterface`. Key features include:

- Serial communication with Arduino via `/dev/ttyUSB0` at 57600 baud
- Velocity control with limits: ±5 rad/s per wheel
- Encoder resolution: 1440 counts per revolution
- Loop rate: 30 Hz
- Timeout: 1000 ms

## 🎮 Usage

### Launch the Robot

```bash
# Launch the real robot
ros2 launch my_bot launch_robot.launch.py

# Launch with visualization
ros2 launch my_bot rsp.launch.py
```

### Check Status

```bash
# Check if hardware interface is loaded
ros2 control list_hardware_components

# Check if controllers are running
ros2 control list_controllers
```

## 🎮 Teleoperation

### Keyboard Control

Control the robot using your keyboard:

```bash
ros2 launch my_bot teleop/teleop_keyboard.launch.py
```

### Joystick Control

Control the robot using a joystick/gamepad:

```bash
ros2 launch my_bot teleop/teleop_joy.launch.py
```

### Using Stamped Velocity Commands

For controllers that require timestamped commands:

```bash
ros2 launch my_bot teleop/teleop_joy.launch.py use_stamped:=true
```

## 🎯 Simulation

### Gazebo Simulation

```bash
ros2 launch my_bot launch_sim.launch.py
```

### Mock Hardware Testing

For testing without physical hardware:

```bash
# The robot will use simulated hardware components
ros2 launch my_bot launch_robot.launch.py use_mock_hardware:=true
```

## ⚙️ Configuration

### Robot Parameters

The robot parameters are defined in:
- `config/my_controllers.yaml` - Controller configuration
- `description/ros2_control.xacro` - Hardware interface parameters
- `config/joystick.yaml` - Joystick configuration

### Customize Controller Settings

Edit `config/my_controllers.yaml` to modify:

- PID gains
- Velocity limits
- Acceleration limits
- Joint names

## 🔍 Troubleshooting

### Common Issues and Solutions

1. **Hardware Not Found**
   ```bash
   # Check if device exists
   ls -l /dev/ttyUSB0
   
   # Fix permissions
   sudo chmod 666 /dev/ttyUSB0
   
   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   ```

2. **Joystick Not Working**
   ```bash
   # Check joystick detection
   ls -l /dev/input/js*
   
   # Fix permissions
   sudo chmod a+rw /dev/input/js*
   
   # Test joystick
   jstest /dev/input/js0
   ```

3. **Controller Errors**
   ```bash
   # Check controller status
   ros2 control list_controllers
   
   # Verify topic publication
   ros2 topic echo /joint_states
   ```

4. **Build Errors**
   ```bash
   # Clean build
   rm -rf build/ install/ log/
   
   # Rebuild
   colcon build --symlink-install
   ```

### Debugging Commands

```bash
# Check hardware status
ros2 control list_hardware_interfaces

# Verify topic publication
ros2 topic echo /joint_states

# Monitor controller status
ros2 control list_controllers
```

### Serial Communication Issues
1. Check device permissions:
   ```bash
   ls -l /dev/ttyUSB0
   sudo chmod 666 /dev/ttyUSB0
   ```
2. Test direct Arduino communication:
   ```bash
   python3 -m serial.tools.miniterm /dev/ttyUSB0 57600 --eol CR
   ```
3. Verify Arduino is responding to commands:
   - Send `e\r` to read encoders
   - Send `o 100 100\r` to test motors

#### Controller Issues
- Verify velocity limits in `ros2_control.xacro`
- Check controller status:
  ```bash
  ros2 control list_controllers
  ros2 control list_hardware_interfaces
  ```

#### Motor Control Issues
- Test motor response using direct commands:
  ```bash
  ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```
- Verify motor connections and power supply

## 🤝 Contributing

Contributions are welcome! Here's how to get started:

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE.md) file for details.

---

> **Note:** This is a GitHub template. You can make your own copy by clicking the green "Use this template" button. If you change the repo/package name, ensure you do a "Find all" using your IDE and rename all instances of `my_bot` to your project's name.