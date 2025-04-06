#!/bin/bash

# Make backup of workspace setup files
echo "Creating backups of original setup files..."
cp ~/dev_ws/install/setup.bash ~/dev_ws/install/setup.bash.backup
cp ~/dev_ws/install/setup.sh ~/dev_ws/install/setup.sh.backup
cp ~/dev_ws/install/setup.zsh ~/dev_ws/install/setup.zsh.backup
cp ~/dev_ws/install/setup.ps1 ~/dev_ws/install/setup.ps1.backup

# Remove turtlebot3 references from setup files
echo "Removing turtlebot3 references from workspace setup files..."
sed -i '/turtlebot3_ws/d' ~/dev_ws/install/setup.bash
sed -i '/turtlebot3_ws/d' ~/dev_ws/install/setup.sh
sed -i '/turtlebot3_ws/d' ~/dev_ws/install/setup.zsh
sed -i '/turtlebot3_ws/d' ~/dev_ws/install/setup.ps1

echo "Rebuilding the workspace from scratch..."
cd ~/dev_ws
rm -rf build install log

echo "Rebuilding workspace with colcon..."
cd ~/dev_ws
colcon build --packages-select my_bot

echo "Cleanup complete. Please run:"
echo "source ~/dev_ws/install/setup.bash"
echo "to apply changes and test your configuration." 