#!/bin/bash
echo "Starting sequence..."
echo "Updating permissions..."
chmod +x ./launch_robosync_node.sh ./launch_executor_node.sh
chmod +x ./launch_detection_node.sh ./launch_location_node.sh ./launch_map_node.sh 
chmod +x ./launch_map_viewer.sh ./launch_temp_viewer.sh ./launch_navigation_server.sh
sleep 3

sleep 3
echo "Launching simulation separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_turtlebot_simulation.sh; exec bash"
echo "Waiting for simulation to start..."
sleep 15

echo "Launching Executor node in separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_executor_node.sh; exec bash"
echo "Launching Robosync node in separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_robosync_node.sh; exec bash"
echo "Launching Detection Node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_detection_node.sh; exec bash"
echo "Launching Location node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_location_node.sh; exec bash"
echo "Launching Mapping node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_map_node.sh; exec bash"
echo "Launching viewer..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_map_viewer.sh; exec bash"
echo "Launching temp viewer..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_temp_viewer.sh; exec bash"
echo "Launching navigation node..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_navigation_server.sh; exec bash"
echo "Startup complete."
