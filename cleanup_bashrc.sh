#!/bin/bash

# Make a backup of the original bashrc
cp ~/.bashrc ~/.bashrc.backup_$(date +%Y%m%d_%H%M%S)

# Remove turtlebot3 related lines from bashrc
sed -i '/TURTLEBOT3_MODEL/d' ~/.bashrc
sed -i '/turtlebot3_ws/d' ~/.bashrc

echo "Removed turtlebot3 references from ~/.bashrc"
echo "A backup was created at ~/.bashrc.backup_$(date +%Y%m%d_%H%M%S)"
echo "Please run 'source ~/.bashrc' to apply changes" 