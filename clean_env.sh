#!/bin/bash

echo "Creating a clean ROS environment..."

# Save original ROS setup
echo "# Original ROS setup" > ~/clean_ros_setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/clean_ros_setup.bash
echo "source ~/dev_ws/install/setup.bash" >> ~/clean_ros_setup.bash

# Make the file executable
chmod +x ~/clean_ros_setup.bash

echo "====================================================="
echo "CLEAN ENVIRONMENT SCRIPT COMPLETED"
echo "====================================================="
echo ""
echo "To use a clean environment without turtlebot3 references:"
echo ""
echo "1. Close your current terminal"
echo "2. Open a new terminal"
echo "3. Run: source ~/clean_ros_setup.bash"
echo "4. Then: cd ~/dev_ws/src/my_bot"
echo ""
echo "This will give you a clean environment with only your my_bot package."
echo "=====================================================" 