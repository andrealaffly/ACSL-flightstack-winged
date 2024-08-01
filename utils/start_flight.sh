#!/bin/bash

# Before running this program, make it executable with the command:
# chmod +x ~/run_flightstack.sh 

# Change to the ROS2 root directory
cd ../

# Source the ROS2 and workspace setup scripts and run the flightstack as root
# Uncomment for ROS2 galactic 
echo "odroid" | sudo -S bash -c "source /opt/ros/galactic/setup.bash && source install/local_setup.bash && ros2 run acsl_flight start"

# Uncomment for ROS2 humble
# echo "odroid" | sudo -S bash -c "source /opt/ros/humble/setup.bash && source install/local_setup.bash && ros2 run acsl_flight start"

# Uncomment for ROS2 humble compiled from source as in given in the instructions
# echo "odroid" | sudo -S bash -c "source ~/ros2_humble/install/local_setup.bash && source install/local_setup.bash && ros2 run acsl_flight start"
