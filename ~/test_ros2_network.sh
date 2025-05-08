#!/bin/bash

echo "ROS 2 Network Test Utility"
echo "=========================="
echo "This script helps validate ROS 2 network communication between machines."
echo ""

# Check network configuration
echo "Current ROS 2 network configuration:"
echo "ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY = $ROS_LOCALHOST_ONLY"
echo "FASTRTPS_DEFAULT_PROFILES_FILE = $FASTRTPS_DEFAULT_PROFILES_FILE"
echo ""

# Check if Fast DDS XML file exists
if [ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
  echo "Fast DDS config file found at: $FASTRTPS_DEFAULT_PROFILES_FILE"
else
  echo "WARNING: Fast DDS config file not found at: $FASTRTPS_DEFAULT_PROFILES_FILE"
fi
echo ""

# Option selection
echo "Choose an option:"
echo "1. Run as publisher (run this on one machine)"
echo "2. Run as subscriber (run this on the other machine)"
read -p "Option (1 or 2): " option

if [ "$option" = "1" ]; then
  echo "Running as publisher. Press Ctrl+C to stop."
  ros2 topic pub /network_test std_msgs/msg/String "data: 'Hello from $(hostname)'" --rate 1
elif [ "$option" = "2" ]; then
  echo "Running as subscriber. Press Ctrl+C to stop."
  echo "Waiting for messages from publisher..."
  ros2 topic echo /network_test
else
  echo "Invalid option. Please choose 1 or 2."
fi 