#!/bin/bash

# Setup ROS 2 network configuration for cross-machine communication

# Set ROS_DOMAIN_ID (must be same on both machines)
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc

# Make sure ROS_LOCALHOST_ONLY is not set or set to 0
sed -i '/export ROS_LOCALHOST_ONLY=1/d' ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc

# Configure Fast DDS to use our XML config
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds.xml' >> ~/.bashrc

# Apply changes
source ~/.bashrc

echo "ROS 2 network configuration has been set up."
echo "Please run this script on both machines (development and Raspberry Pi)."
echo "After running on both machines, test communication with:"
echo "  On machine 1: ros2 topic pub /test std_msgs/msg/String \"data: hello\""
echo "  On machine 2: ros2 topic echo /test" 