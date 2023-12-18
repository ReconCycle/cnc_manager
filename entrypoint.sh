#!/bin/bash

# Set GPIO permissions
#sudo usermod -aG gpio ${USER}

# Source ROS setup
source /opt/ros/noetic/setup.bash

# Start your ROS nodes or launch files
# Example: roslaunch my_package my_launch_file.launch

# Keep the container running
exec "$@"

