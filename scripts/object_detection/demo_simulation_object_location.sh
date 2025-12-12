#!/bin/bash
echo "Starting semantic map sequence..."
echo "Updating permissions..."
chmod +x ./launch_turtlebot_simulation.sh ./launch_robosync_node.sh ./launch_detection_node.sh ./launch_location_node.sh ./launch_map_node.sh ./launch_map_viewer.sh
sleep 3
echo "Launching simulation separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_turtlebot_simulation.sh; exec bash"
echo "Waiting for simulation to start..."
sleep 15
echo "Launching Robosync node in separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_robosync_node.sh; exec bash"
sleep 2
echo "Launching Detection Node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_detection_node.sh; exec bash"
sleep 2
echo "Launching Location node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_location_node.sh; exec bash"
sleep 2
echo "Launching Mapping node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_map_node.sh; exec bash"
sleep 2
echo "Launching viewer..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_map_viewer.sh; exec bash"
echo "Launching temp viewers..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_temp_viewer.sh; exec bash"
echo "Startup complete."
# echo "Sourcing ROS2 files..."
# source /opt/ros/jazzy/setup.bash
# source ../install/setup.bash
# echo "Running python script..."
# ros2 run pathfind predefined